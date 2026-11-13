/* SPDX-License-Identifier: Apache-2.0 */
/* The NINA socket table and the firmware's BSD-style socket commands
 * (0x70..0x7f): the Zephyr build's drivers/nina/nina_socket.c without the
 * Zephyr socket layer (file descriptors, errno, the waiting loops' sleeps).
 * Each command is a single lwIP socket call made inside the module's SPI
 * command handler, which serves Wi-Fi management and every socket.  A call
 * that blocks there stalls all of them, so every socket is made
 * non-blocking as soon as it exists and waiting is done on this side by
 * polling: in nina_poller.c, which turns the polls into usrsock events.
 *
 * The functions keep the Zephyr build's names, branches, order, constants
 * and errno names (nina_sock.h gives the line of each).  The differences:
 *
 * - fail(err) (errno = err, return -1) becomes "return -err".
 * - socket_lock is the caller's lock (nina_sock.h), so open_socket() no
 *   longer takes it and the k_mutex calls are gone.
 * - The file descriptor is NuttX's (usrsock gives it); the number of the
 *   table entry is the usrsock id.
 * - connect() waits in nina_poller.c, not in the caller (see
 *   nina_sock_connect()).
 * - Only what reaches the driver on NuttX is here: no "nonblocking" and
 *   no SO_RCVTIMEO (NuttX's socket layer keeps both).  Whether a call
 *   waits arrives as MSG_DONTWAIT in its flags.
 * - The wait loops of send and receive make one pass per call here.  The
 *   caller (nina_usrsock.c) does the waiting through usrsock: a receive
 *   that finds nothing waits for the socket's RECVFROM_AVAIL event, which
 *   nina_poller.c raises from its polls every NINA_POLL_INTERVAL_MS; a send
 *   that finds no room waits NINA_SEND_RETRY_MS for SENDTO_READY from
 *   nina_poller.c, and usrsock then repeats the request: the next pass,
 *   again in the calling thread.  The send keeps its deadline and its
 *   next state check in struct nina_sock between passes.
 * - A poll that found data is remembered (read_hint) and stands for the
 *   first poll of the next receive.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>

#include "nina_os.h"
#include "nina_proto.h"
#include "nina_sock.h"

#ifndef MIN
#  define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef CLAMP
#  define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#endif

struct nina_sock g_nina_sockets[NINA_SOCK_MAX];
#define sockets g_nina_sockets

/* sendmsg_lock and udp_lock of the Zephyr build are not needed: NuttX's
 * usrsock handles one request at a time, under the caller's lock.
 */
static uint8_t sendmsg_buffer[NINA_TCP_MAX_IO];
/* Address, port and data of one received datagram. */
static uint8_t udp_buffer[6 + NINA_MAX_IO];

struct nina_sock *nina_sock_get(int usockid)
{
	if (usockid < 0 || usockid >= NINA_SOCK_MAX || !sockets[usockid].used) {
		return NULL;
	}
	return &sockets[usockid];
}

static int cmd1(uint8_t cmd, uint8_t id, uint8_t *data, size_t cap, size_t *n)
{
	struct nina_param p = {&id, 1};

	return nina_request(cmd, &p, 1, false, false, data, cap, n);
}

static int poll_state(struct nina_sock *s, uint8_t *state)
{
	size_t n;
	int ret = cmd1(NINA_CMD_POLL, s->id, state, 1, &n);

	if (ret) {
		return ret;
	}
	ret = n == 1 && !(*state & NINA_POLL_FAIL) ? 0 : -EIO;
	/* NuttX: what the poll says about data stands for the next receive's
	 * first poll (read_hint). */
	s->read_hint = !ret && (*state & NINA_POLL_READ) && !s->listening;
	return ret;
}

int nina_sock_poll_state(struct nina_sock *s, uint8_t *state)
{
	return poll_state(s, state);
}

static int set_nonblocking(uint8_t id)
{
	uint8_t cmd[4] = {0x7e, 0x66, 0x04, 0x80}; /* lwIP FIONBIO */
	uint8_t value[4] = {1, 0, 0, 0}, reply[4];
	struct nina_param p[] = {{&id, 1}, {cmd, sizeof(cmd)}, {value, sizeof(value)}};
	size_t n;
	int ret = nina_request(NINA_CMD_IOCTL, p, 3, false, false, reply, sizeof(reply), &n);

	return ret ? ret : (n == sizeof(reply) ? 0 : -EIO);
}

static void close_id(struct nina_sock *s)
{
	uint8_t result;
	size_t n;

	if (s->id != NINA_NO_SOCKET) {
		(void)cmd1(NINA_CMD_CLOSE, s->id, &result, 1, &n);
		s->id = NINA_NO_SOCKET;
	}
	s->bound = false;
	s->connected = false;
	s->listening = false;
	/* NuttX: no connection is left in progress.  "connect_xid" stays for
	 * the answer to the connect request (nina_usrsock.c), "reported" too: it
	 * is what usrsock holds, which closing the firmware socket does not
	 * change. */
	s->connecting = false;
	s->send_waiting = false;
	s->read_hint = false;
}

/* Creates the firmware socket on first use. Caller holds socket_lock. */
static int open_locked(struct nina_sock *s)
{
	uint8_t type = s->udp ? 2 : 1, proto = s->udp ? 17 : 6, result;
	struct nina_param args[2] = {{&type, 1}, {&proto, 1}};
	size_t n;
	int ret;

	if (s->id != NINA_NO_SOCKET) {
		return 0;
	}
	ret = nina_request(NINA_CMD_SOCKET, args, 2, false, false, &result, 1, &n);
	if (ret || n != 1 || result == NINA_NO_SOCKET) {
		return ret ? ret : -ENFILE;
	}
	s->id = result;
	ret = set_nonblocking(s->id);
	if (ret) {
		close_id(s);
	}
	return ret;
}

/* The Zephyr build takes socket_lock here; the caller holds it already. */
static int open_socket(struct nina_sock *s)
{
	return open_locked(s);
}

/* SO_ERROR of the firmware socket as reported by its lwIP (ESP-IDF errno
 * numbers), or a negative errno if it could not be read. */
static int socket_error(struct nina_sock *s)
{
	uint8_t name[4] = {0x07, 0x10, 0, 0}, length = 4, value[4];
	struct nina_param p[] = {{&s->id, 1}, {name, sizeof(name)}, {&length, 1}};
	size_t n;
	int ret = nina_request(NINA_CMD_GETSOCKOPT, p, 3, false, false, value, sizeof(value), &n);

	if (ret || n != sizeof(value)) {
		return ret ? ret : -EIO;
	}
	return value[0] | (value[1] << 8) | (value[2] << 16) | ((int)value[3] << 24);
}

static int connect_errno(int lwip_error)
{
	switch (lwip_error) {
	case 104: return ECONNRESET;
	case 113: return ECONNABORTED;
	case 116: return ETIMEDOUT;
	case 118: return EHOSTUNREACH;
	default: return ECONNREFUSED;
	}
}

int nina_sock_bind(int usockid, const struct sockaddr *addr, size_t addrlen)
{
	struct nina_sock *s = nina_sock_get(usockid);
	const struct sockaddr_in *in = (const struct sockaddr_in *)addr;
	struct nina_param args[2];
	uint8_t result;
	size_t n;
	int ret;

	if (!s) {
		return -EBADF;
	}
	if (!addr || addrlen < sizeof(*in) || addr->sa_family != AF_INET) {
		return -EINVAL;
	}
	if (in->sin_addr.s_addr != 0) {
		return -EADDRNOTAVAIL;
	}
	if (s->bound || s->connected) {
		return -EINVAL;
	}
	ret = open_locked(s);
	if (!ret) {
		args[0] = (struct nina_param){&s->id, 1};
		args[1] = (struct nina_param){(const uint8_t *)&in->sin_port, 2};
		ret = nina_request(NINA_CMD_BIND, args, 2, false, false, &result, 1, &n);
		if (!ret && (n != 1 || result != 1)) {
			ret = -EADDRINUSE;
		}
	}
	if (!ret) {
		s->bound = true;
	}
	return ret;
}

int nina_sock_listen(int usockid, int backlog)
{
	struct nina_sock *s = nina_sock_get(usockid);
	uint8_t value, reply;
	struct nina_param p[2];
	size_t n;
	int ret;

	if (!s) {
		return -EBADF;
	}
	value = CLAMP(backlog, 1, 255);
	p[0] = (struct nina_param){&s->id, 1};
	p[1] = (struct nina_param){&value, 1};
	if (s->udp || !s->bound || s->listening) {
		return -EINVAL;
	}
	ret = nina_request(NINA_CMD_LISTEN, p, 2, false, false, &reply, 1, &n);
	if (!ret && (n != 1 || reply != 1)) {
		ret = -EIO;
	}
	if (ret) {
		return ret;
	}
	s->listening = true;
	return 0;
}

int nina_sock_accept(int usockid, struct sockaddr_in *peer)
{
	struct nina_sock *listener = nina_sock_get(usockid), *client = NULL;
	uint8_t reply[7];
	size_t n;
	int i = 0, ret;

	if (!listener) {
		return -EBADF;
	}
	/* The Zephyr build also checks that addr and addrlen are both given
	 * or both not; NuttX's accept() checks them before usrsock. */
	if (!listener->listening) {
		return -EINVAL;
	}
	for (i = 0; i < NINA_SOCK_MAX; i++) {
		if (!sockets[i].used) {
			client = &sockets[i];
			break;
		}
	}
	if (!client) {
		return -EMFILE;
	}
	ret = cmd1(NINA_CMD_ACCEPT, listener->id, reply, sizeof(reply), &n);
	if (ret || n != sizeof(reply) || reply[0] == NINA_NO_SOCKET) {
		return ret ? ret : -EAGAIN;
	}
	memset(client, 0, sizeof(*client));
	client->id = reply[0];
	client->used = true;
	client->connected = true;
	ret = set_nonblocking(client->id);
	if (ret) {
		close_id(client);
		client->used = false;
		return ret;
	}
	memset(peer, 0, sizeof(*peer));
	peer->sin_family = AF_INET;
	memcpy(&peer->sin_addr, reply + 1, 4);
	/* The firmware returns the two bytes of its network-order integer in
	 * big endian; convert that integer back to its native representation. */
	((uint8_t *)&peer->sin_port)[0] = reply[6];
	((uint8_t *)&peer->sin_port)[1] = reply[5];
	return i;
}

/* One pass of do_connect()'s wait loop (nina_socket.c:343-365): 0 when
 * connected, 1 to try again after NINA_POLL_INTERVAL_MS, or a negative
 * errno, the firmware socket then closed ("failed:", :366-371).
 */
static int connect_pass(struct nina_sock *s, int64_t now)
{
	uint8_t state;
	int ret;

	ret = poll_state(s, &state);
	if (ret) {
		goto failed;
	}
	if (state & (NINA_POLL_WRITE | NINA_POLL_ERROR)) {
		ret = socket_error(s);
		if (ret < 0) {
			goto failed;
		}
		if (ret > 0 || (state & NINA_POLL_ERROR)) {
			ret = -connect_errno(ret);
			goto failed;
		}
		s->connecting = false;
		s->connect_xid = 0;
		s->connected = true;
		return 0;
	}
	if (now >= s->connect_deadline) {
		ret = -ETIMEDOUT;
		goto failed;
	}
	return 1;
failed:
	/* A failed lwIP socket cannot be connected again. */
	close_id(s);
	return ret;
}

int nina_sock_connect(int usockid, const struct sockaddr *addr, size_t addrlen)
{
	struct nina_sock *s = nina_sock_get(usockid);
	const struct sockaddr_in *in = (const struct sockaddr_in *)addr;
	uint8_t result;
	size_t n;
	struct nina_param p[3];
	int ret;

	if (!s) {
		return -EBADF;
	}
	if (!addr || addrlen < sizeof(*in) || addr->sa_family != AF_INET) {
		return -EINVAL;
	}
	if (s->udp) {
		/* Kept here: datagrams from other senders are filtered on receive. */
		ret = open_socket(s);
		if (ret) {
			return ret;
		}
		s->peer = *in;
		s->connected = true;
		return 0;
	}
	if (s->connected) {
		return -EISCONN;
	}
	if (s->listening) {
		return -EINVAL;
	}
	ret = open_locked(s);
	if (!ret) {
		p[0] = (struct nina_param){&s->id, 1};
		p[1] = (struct nina_param){(const uint8_t *)&in->sin_addr, 4};
		p[2] = (struct nina_param){(const uint8_t *)&in->sin_port, 2};
		ret = nina_request(NINA_CMD_CONNECT, p, 3, false, false, &result, 1, &n);
		if (!ret && n != 1) {
			ret = -EPROTO;
		}
	}
	if (ret) {
		close_id(s);
		return ret;
	}
	/* The socket is non-blocking, so "not connected yet" is the normal
	 * answer. It becomes writable when the handshake completes and reports
	 * an error when it fails. */
	s->connect_deadline = nina_os_uptime_ms() + NINA_CONNECT_WAIT_MS;
	s->connecting = true;
	return connect_pass(s, nina_os_uptime_ms());
}

int nina_sock_connect_check(int usockid, int64_t now)
{
	struct nina_sock *s = nina_sock_get(usockid);

	if (!s || !s->connecting) {
		return -EINVAL;
	}
	return connect_pass(s, now);
}

static ssize_t udp_send(struct nina_sock *s, const void *buf, size_t len,
			int flags, const struct sockaddr *to,
			size_t tolen)
{
	const struct sockaddr_in *peer = (const struct sockaddr_in *)to;
	struct nina_param p[4];
	uint8_t answer[2];
	size_t n;
	int ret;

	if ((flags & ~MSG_DONTWAIT) || len > NINA_MAX_IO) {
		return len > NINA_MAX_IO ? -EMSGSIZE : -EOPNOTSUPP;
	}
	if (len && !buf) {
		return -EINVAL;
	}
	if (to && (tolen < sizeof(*peer) || to->sa_family != AF_INET)) {
		return -EINVAL;
	}
	if (!to) {
		if (!s->connected) {
			return -EDESTADDRREQ;
		}
		peer = &s->peer;
	}
	ret = open_socket(s);
	if (ret) {
		return ret;
	}
	p[0] = (struct nina_param){&s->id, 1};
	p[1] = (struct nina_param){(const uint8_t *)&peer->sin_addr, 4};
	p[2] = (struct nina_param){(const uint8_t *)&peer->sin_port, 2};
	p[3] = (struct nina_param){buf, len};
	ret = nina_request(NINA_CMD_SENDTO, p, 4, true, false, answer, sizeof(answer), &n);
	if (ret) {
		return ret;
	}
	if (n != 2) {
		return -EPROTO;
	}
	/* The firmware reports every failure as zero bytes sent. */
	if ((((size_t)answer[0] << 8) | answer[1]) != len) {
		return -EAGAIN;
	}
	return len;
}

/* do_send(), one pass of its loop (see nina_sock_send()).  KEY and KEY_LEN
 * tell the passes of one call from a new call: the caller's data and its
 * length. */
static ssize_t do_send(struct nina_sock *s, const void *buf, size_t len, int flags,
		       const struct sockaddr *to, size_t tolen,
		       const void *key, size_t key_len, bool *retry)
{
	uint8_t answer[2];
	size_t n;
	int ret;
	struct nina_param p[2];
	unsigned int written;

	*retry = false;
	if (s->udp) {
		return udp_send(s, buf, len, flags, to, tolen);
	}
	if (to || (flags & ~MSG_DONTWAIT)) {
		return -EOPNOTSUPP;
	}
	if (!s->connected) {
		return -ENOTCONN;
	}
	if (!len) {
		return 0;
	}
	if (!buf) {
		return -EINVAL;
	}

	len = MIN(len, NINA_TCP_MAX_IO);
	p[0] = (struct nina_param){&s->id, 1};
	p[1] = (struct nina_param){buf, len};
	/* The loop's "deadline" and "next_state_check", set when a call
	 * starts, kept by the passes that continue it. */
	if (!s->send_waiting || s->send_key != key || s->send_key_len != key_len) {
		s->send_deadline = nina_os_uptime_ms() + NINA_SEND_WAIT_MS;
		s->send_next_state_check = 0;
	}
	s->send_waiting = false;

	ret = nina_request(NINA_CMD_SEND, p, 2, true, false, answer,
			   sizeof(answer), &n);
	if (ret) {
		return ret;
	}
	if (n != 2) {
		return -EPROTO;
	}
	written = ((unsigned int)answer[0] << 8) | answer[1];
	if (written > len) {
		return -EPROTO;
	}
	if (written) {
		return written;
	}
	if (flags & MSG_DONTWAIT) {
		return -EAGAIN;
	}
	/* Zero means a full send buffer or a failed connection; only the
	 * socket state tells them apart. */
	if (nina_os_uptime_ms() >= s->send_next_state_check)
	{
		uint8_t state;

		ret = poll_state(s, &state);
		if (ret || (state & NINA_POLL_ERROR)) {
			return ret ? ret : -EPIPE;
		}
		s->send_next_state_check = nina_os_uptime_ms() + NINA_SEND_STATE_INTERVAL_MS;
	}
	if (nina_os_uptime_ms() >= s->send_deadline) {
		return -EAGAIN;
	}
	/* k_sleep(K_MSEC(CONFIG_NINA_SEND_RETRY_MS)), then the next pass:
	 * the caller's. */
	s->send_waiting = true;
	s->send_key = key;
	s->send_key_len = key_len;
	*retry = true;
	return -EAGAIN;
}

ssize_t nina_sock_send(int usockid, const void *buf, size_t len, int flags,
		       const struct sockaddr *to, size_t tolen, bool *retry)
{
	struct nina_sock *s = nina_sock_get(usockid);

	*retry = false;
	if (!s) {
		return -EBADF;
	}
	return do_send(s, buf, len, flags, to, tolen, buf, len, retry);
}

ssize_t nina_sock_sendmsg(int usockid, const struct iovec *iov, size_t iovcnt,
			  bool has_name, int flags, bool *retry)
{
	struct nina_sock *s = nina_sock_get(usockid);
	const void *key = NULL;
	size_t total = 0, key_len = 0;

	*retry = false;
	if (!s) {
		return -EBADF;
	}
	if (iovcnt && !iov) {
		return -EINVAL;
	}
	/* NuttX: no control data reaches the driver. */
	if (has_name) {
		return -EOPNOTSUPP;
	}
	if (s->udp) {
		return -EOPNOTSUPP;
	}
	/* Coalesce vectors so MQTT's header and payload share an SPI request.
	 * Return the actual partial write; the caller advances its iovecs.
	 */
	for (size_t i = 0; i < iovcnt; i++) {
		key_len += iov[i].iov_len;
	}
	for (size_t i = 0; i < iovcnt; i++) {
		size_t n = MIN(iov[i].iov_len, sizeof(sendmsg_buffer) - total);

		if (n && !iov[i].iov_base) {
			return -EFAULT;
		}
		if (n) {
			memcpy(sendmsg_buffer + total, iov[i].iov_base, n);
			total += n;
			if (!key) {
				key = iov[i].iov_base;
			}
		}
		if (total == sizeof(sendmsg_buffer)) {
			break;
		}
	}
	return total ? do_send(s, sendmsg_buffer, total, flags, NULL, 0, key, key_len,
			       retry) : 0;
}

/* The first poll of a receive pass: the last poll's finding if it saw
 * data, else a new poll. */
static int receive_state(struct nina_sock *s, uint8_t *state)
{
	if (s->read_hint) {
		s->read_hint = false;
		*state = NINA_POLL_READ;
		return 0;
	}
	return poll_state(s, state);
}

static ssize_t udp_recv(struct nina_sock *s, void *buf, size_t len, int flags,
			struct sockaddr_in *from)
{
	struct sockaddr_in sender;
	uint8_t state, size[2];
	struct nina_param p[2];
	size_t n, copied;
	int ret;

	memset(&sender, 0, sizeof(sender));
	sender.sin_family = AF_INET;
	/* The Zephyr build also refuses a sender address without its length
	 * or the other way round; NuttX's recvfrom() checks that. */
	if (flags & ~MSG_DONTWAIT) {
		return -EOPNOTSUPP;
	}
	if (len && !buf) {
		return -EINVAL;
	}
	ret = open_socket(s);
	if (ret) {
		return ret;
	}
	for (;;) {
		/* Never ask for a datagram that is not there: the reply cannot
		 * tell "nothing received" from an empty datagram. */
		ret = receive_state(s, &state);
		if (ret) {
			return ret;
		}
		s->read_hint = false;
		if (!(state & NINA_POLL_READ)) {
			/* The caller waits (or not) for RECVFROM_AVAIL. */
			return -EAGAIN;
		}
		/* lwIP drops what does not fit, which keeps datagram boundaries. */
		copied = CLAMP(len, 1, NINA_MAX_IO);
		size[0] = copied & 0xff;
		size[1] = copied >> 8;
		p[0] = (struct nina_param){&s->id, 1};
		p[1] = (struct nina_param){size, 2};
		ret = nina_request(NINA_CMD_RECVFROM, p, 2, false, true, udp_buffer,
				   6 + copied, &n);
		if (ret || n < 6) {
			return ret ? ret : -EPROTO;
		}
		memcpy(&sender.sin_addr, udp_buffer, 4);
		/* Same byte order as the port in the accept reply. */
		((uint8_t *)&sender.sin_port)[0] = udp_buffer[5];
		((uint8_t *)&sender.sin_port)[1] = udp_buffer[4];
		copied = MIN(n - 6, len);
		if (copied) {
			memcpy(buf, udp_buffer + 6, copied);
		}
		if (s->connected && (sender.sin_addr.s_addr != s->peer.sin_addr.s_addr ||
				     sender.sin_port != s->peer.sin_port)) {
			continue;
		}
		if (from) {
			*from = sender;
		}
		return copied;
	}
}

ssize_t nina_sock_recv(int usockid, void *buf, size_t len, int flags,
		       struct sockaddr_in *from)
{
	struct nina_sock *s = nina_sock_get(usockid);
	uint8_t state, bytes[2];
	size_t n;
	int ret;
	struct nina_param p[2];

	if (!s) {
		return -EBADF;
	}
	if (s->udp) {
		return udp_recv(s, buf, len, flags, from);
	}
	if (from || (flags & ~MSG_DONTWAIT)) {
		return -EOPNOTSUPP;
	}
	if (!s->connected) {
		return -ENOTCONN;
	}
	if (!len) {
		return 0;
	}
	if (!buf) {
		return -EINVAL;
	}
	ret = receive_state(s, &state);
	if (ret) {
		return ret;
	}
	s->read_hint = false;
	if (!(state & NINA_POLL_READ)) {
		/* The caller waits (or not) for RECVFROM_AVAIL. */
		return -EAGAIN;
	}
	len = MIN(len, NINA_MAX_IO);
	bytes[0] = len & 0xff;
	bytes[1] = len >> 8;
	p[0] = (struct nina_param){&s->id, 1};
	p[1] = (struct nina_param){bytes, 2};
	ret = nina_request(NINA_CMD_RECV, p, 2, false, true, buf, len, &n);
	if (ret) {
		return ret;
	}
	/* Readable with nothing to read: the peer closed or the connection
	 * failed. The firmware reports both as zero bytes. */
	return (ssize_t)n;
}

int nina_sock_fionread(int usockid)
{
	struct nina_sock *s = nina_sock_get(usockid);
	uint8_t state;
	int ret;

	if (!s) {
		return -EBADF;
	}
	if (!s->udp && !s->connected) {
		return -ENOTCONN;
	}
	/* Nothing can arrive on a socket the firmware does not have yet. */
	if (s->id == NINA_NO_SOCKET) {
		return 0;
	}
	ret = poll_state(s, &state);
	if (ret) {
		return ret;
	}
	return (state & NINA_POLL_READ) ? 1 : 0;
}

int nina_sock_close(int usockid)
{
	struct nina_sock *s = nina_sock_get(usockid);
	uint8_t result;
	size_t n;
	int ret = 0;

	if (!s) {
		return -EBADF;
	}
	if (s->id != NINA_NO_SOCKET) {
		ret = cmd1(NINA_CMD_CLOSE, s->id, &result, 1, &n);
		if (!ret && (n != 1 || result != 1)) {
			ret = -EIO;
		}
	}
	s->used = false;
	s->connected = false;
	s->bound = false;
	s->listening = false;
	s->id = NINA_NO_SOCKET;
	s->connecting = false;
	s->connect_xid = 0;
	s->reported = 0;
	s->send_waiting = false;
	s->read_hint = false;
	return ret;
}

/* Only TCP_NODELAY reaches here on NuttX besides the options that are
 * refused (SO_RCVTIMEO is NuttX's own). Options are not passed on to the
 * module: its setsockopt command reaches SOL_SOCKET options only, and the
 * sockets there are always non-blocking. */
int nina_sock_setsockopt(int usockid, int level, int option, const void *value,
			 size_t valuelen)
{
	struct nina_sock *s = nina_sock_get(usockid);

	if (!s) {
		return -EBADF;
	}
	/* Accepted without effect: the module's setsockopt command cannot reach
	 * TCP-level options, so Nagle's algorithm stays on. Callers such as the
	 * iperf3 client set this on a control connection and give up if they
	 * cannot. */
	if (level == IPPROTO_TCP && option == TCP_NODELAY && !s->udp) {
		return value && valuelen == sizeof(int) ? 0 : -EINVAL;
	}
	return -ENOPROTOOPT;
}

bool nina_sock_supported(int family, int type, int proto)
{
	return family == AF_INET &&
	       ((type == SOCK_STREAM &&
		 (proto == 0 || proto == IPPROTO_TCP)) ||
		(type == SOCK_DGRAM &&
		 (proto == 0 || proto == IPPROTO_UDP)));
}

int nina_sock_create(int family, int type, int proto)
{
	struct nina_sock *s = NULL;
	int i;

	if (!nina_sock_supported(family, type, proto)) {
		return -EAFNOSUPPORT;
	}
	for (i = 0; i < NINA_SOCK_MAX; i++) {
		if (!sockets[i].used) {
			s = &sockets[i];
			break;
		}
	}
	if (!s) {
		return -EMFILE;
	}
	memset(s, 0, sizeof(*s));
	s->id = NINA_NO_SOCKET;
	s->used = true;
	s->connected = false;
	s->udp = type == SOCK_DGRAM;
	s->bound = false;
	s->listening = false;
	return i;
}

uint8_t nina_sock_watch(const struct nina_sock *s)
{
	uint8_t want;

	if (!s->used || s->id == NINA_NO_SOCKET) {
		return 0;
	}
	if (s->connecting) {
		return NINA_SOCK_WATCH_CONNECT;
	}
	if (s->listening) {
		want = NINA_SOCK_EV_READ;
	} else if (s->udp) {
		want = NINA_SOCK_EV_READ;
	} else if (s->connected) {
		want = NINA_SOCK_EV_READ | NINA_SOCK_EV_WRITE;
		if (s->send_waiting) {
			want = NINA_SOCK_EV_READ;
		}
	} else {
		return 0;
	}
	if (s->reported & NINA_SOCK_EV_CLOSED) {
		return 0;
	}
	return want & (uint8_t)~s->reported;
}

uint8_t nina_sock_events(const struct nina_sock *s, int poll_result, uint8_t state)
{
	uint8_t want = nina_sock_watch(s), ev = 0;

	if (want == 0 || want == NINA_SOCK_WATCH_CONNECT) {
		return 0;
	}
	if (poll_result) {
		return !s->udp && !s->listening && s->connected ? NINA_SOCK_EV_CLOSED : 0;
	}
	if ((state & NINA_POLL_READ) && (want & NINA_SOCK_EV_READ)) {
		ev |= NINA_SOCK_EV_READ;
	}
	if ((state & NINA_POLL_WRITE) && (want & NINA_SOCK_EV_WRITE)) {
		ev |= NINA_SOCK_EV_WRITE;
	}
	if ((state & NINA_POLL_ERROR) && !s->udp && !s->listening && s->connected) {
		ev |= NINA_SOCK_EV_CLOSED;
	}
	return ev;
}
