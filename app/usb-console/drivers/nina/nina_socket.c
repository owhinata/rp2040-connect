/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr socket offload on the NINA firmware's BSD-style socket commands
 * (0x70..0x7f). Each command is a single lwIP socket call made inside the
 * module's SPI command handler, which serves Wi-Fi management and every
 * socket. A call that blocks there stalls all of them, so every socket is
 * made non-blocking as soon as it exists and waiting is done here by polling.
 * The older per-type client commands (0x28..0x46) are not used.
 */
#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/fdtable.h>
#include "nina_proto.h"

/* The firmware's lwIP is built with CONFIG_LWIP_MAX_SOCKETS=10. */
#define MAX_NINA_SOCKETS 10
#define NINA_MAX_IO      512
#define NINA_NO_SOCKET   UINT8_MAX
#define NINA_SEND_WAIT_MS 10000
#define NINA_CONNECT_WAIT_MS 10000
/* While a send finds no buffer space, how often the socket is checked for an
 * error. Every check is another command to the module. */
#define NINA_SEND_STATE_INTERVAL_MS 100
#define NINA_POLL_INTERVAL_MS 20

#define CMD_SOCKET     0x70
#define CMD_CLOSE      0x71
#define CMD_BIND       0x73
#define CMD_LISTEN     0x74
#define CMD_ACCEPT     0x75
#define CMD_CONNECT    0x76
#define CMD_SEND       0x77
#define CMD_RECV       0x78
#define CMD_SENDTO     0x79
#define CMD_RECVFROM   0x7a
#define CMD_IOCTL      0x7b
#define CMD_POLL       0x7c
#define CMD_GETSOCKOPT 0x7e

#define POLL_READ  0x01
#define POLL_WRITE 0x02
#define POLL_ERROR 0x04
#define POLL_FAIL  0x80

struct nina_sock {
	bool used;
	bool connected;
	bool udp;
	bool bound;
	bool listening;
	bool nonblocking;
	uint8_t id;
	int32_t receive_timeout_ms; /* SO_RCVTIMEO; 0 waits without limit */
	struct net_sockaddr_in peer;
};
static struct nina_sock sockets[MAX_NINA_SOCKETS];
K_MUTEX_DEFINE(socket_lock);
K_MUTEX_DEFINE(udp_lock);
K_MUTEX_DEFINE(sendmsg_lock);
static uint8_t sendmsg_buffer[CONFIG_NINA_TCP_MAX_IO];
/* Address, port and data of one received datagram. Protected by udp_lock. */
static uint8_t udp_buffer[6 + NINA_MAX_IO];
static const struct socket_op_vtable socket_ops;

static int cmd1(uint8_t cmd, uint8_t id, uint8_t *data, size_t cap, size_t *n)
{
	struct nina_param p = {&id, 1};

	return nina_request(cmd, &p, 1, false, false, data, cap, n);
}

static int fail(int err)
{
	errno = err > 0 ? err : -err;
	return -1;
}

static int poll_state(struct nina_sock *s, uint8_t *state)
{
	size_t n;
	int ret = cmd1(CMD_POLL, s->id, state, 1, &n);

	if (ret) {
		return ret;
	}
	return n == 1 && !(*state & POLL_FAIL) ? 0 : -EIO;
}

static int set_nonblocking(uint8_t id)
{
	uint8_t cmd[4] = {0x7e, 0x66, 0x04, 0x80}; /* lwIP FIONBIO */
	uint8_t value[4] = {1, 0, 0, 0}, reply[4];
	struct nina_param p[] = {{&id, 1}, {cmd, sizeof(cmd)}, {value, sizeof(value)}};
	size_t n;
	int ret = nina_request(CMD_IOCTL, p, 3, false, false, reply, sizeof(reply), &n);

	return ret ? ret : (n == sizeof(reply) ? 0 : -EIO);
}

static void close_id(struct nina_sock *s)
{
	uint8_t result;
	size_t n;

	if (s->id != NINA_NO_SOCKET) {
		(void)cmd1(CMD_CLOSE, s->id, &result, 1, &n);
		s->id = NINA_NO_SOCKET;
	}
	s->bound = false;
	s->connected = false;
	s->listening = false;
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
	ret = nina_request(CMD_SOCKET, args, 2, false, false, &result, 1, &n);
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

static int open_socket(struct nina_sock *s)
{
	int ret;

	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = open_locked(s);
	k_mutex_unlock(&socket_lock);
	return ret;
}

/* SO_ERROR of the firmware socket as reported by its lwIP (ESP-IDF errno
 * numbers), or a negative errno if it could not be read. */
static int socket_error(struct nina_sock *s)
{
	uint8_t name[4] = {0x07, 0x10, 0, 0}, length = 4, value[4];
	struct nina_param p[] = {{&s->id, 1}, {name, sizeof(name)}, {&length, 1}};
	size_t n;
	int ret = nina_request(CMD_GETSOCKOPT, p, 3, false, false, value, sizeof(value), &n);

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

static int do_bind(void *obj, const struct net_sockaddr *addr,
		   net_socklen_t addrlen)
{
	struct nina_sock *s = obj;
	const struct net_sockaddr_in *in = (const struct net_sockaddr_in *)addr;
	struct nina_param args[2];
	uint8_t result;
	size_t n;
	int ret;

	if (!addr || addrlen < sizeof(*in) || addr->sa_family != NET_AF_INET) {
		return fail(EINVAL);
	}
	if (in->sin_addr.s_addr != 0) {
		return fail(EADDRNOTAVAIL);
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	if (s->bound || s->connected) {
		k_mutex_unlock(&socket_lock);
		return fail(EINVAL);
	}
	ret = open_locked(s);
	if (!ret) {
		args[0] = (struct nina_param){&s->id, 1};
		args[1] = (struct nina_param){(const uint8_t *)&in->sin_port, 2};
		ret = nina_request(CMD_BIND, args, 2, false, false, &result, 1, &n);
		if (!ret && (n != 1 || result != 1)) {
			ret = -EADDRINUSE;
		}
	}
	if (!ret) {
		s->bound = true;
	}
	k_mutex_unlock(&socket_lock);
	return ret ? fail(ret) : 0;
}

static int do_listen(void *obj, int backlog)
{
	struct nina_sock *s = obj;
	uint8_t value = CLAMP(backlog, 1, 255), reply;
	struct nina_param p[] = {{&s->id, 1}, {&value, 1}};
	size_t n;
	int ret;

	if (s->udp || !s->bound || s->listening) {
		return fail(EINVAL);
	}
	ret = nina_request(CMD_LISTEN, p, 2, false, false, &reply, 1, &n);
	if (!ret && (n != 1 || reply != 1)) {
		ret = -EIO;
	}
	if (ret) {
		return fail(ret);
	}
	s->listening = true;
	return 0;
}

static int do_accept(void *obj, struct net_sockaddr *addr, net_socklen_t *addrlen)
{
	struct nina_sock *listener = obj, *client = NULL;
	uint8_t reply[7];
	struct net_sockaddr_in peer = {.sin_family = NET_AF_INET};
	size_t n;
	int fd, ret;

	if (!listener->listening || (!!addr != !!addrlen)) {
		return fail(EINVAL);
	}
	fd = zvfs_reserve_fd();
	if (fd < 0) {
		return -1;
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	for (int i = 0; i < MAX_NINA_SOCKETS; i++) {
		if (!sockets[i].used) {
			client = &sockets[i];
			break;
		}
	}
	if (!client) {
		ret = -EMFILE;
		goto error;
	}
	ret = cmd1(CMD_ACCEPT, listener->id, reply, sizeof(reply), &n);
	if (ret || n != sizeof(reply) || reply[0] == NINA_NO_SOCKET) {
		ret = ret ? ret : -EAGAIN;
		goto error;
	}
	memset(client, 0, sizeof(*client));
	client->id = reply[0];
	client->used = true;
	client->connected = true;
	ret = set_nonblocking(client->id);
	if (ret) {
		close_id(client);
		client->used = false;
		goto error;
	}
	memcpy(&peer.sin_addr, reply + 1, 4);
	/* The firmware returns the two bytes of its network-order integer in
	 * big endian; convert that integer back to its native representation. */
	((uint8_t *)&peer.sin_port)[0] = reply[6];
	((uint8_t *)&peer.sin_port)[1] = reply[5];
	if (addr) {
		memcpy(addr, &peer, MIN(*addrlen, sizeof(peer)));
		*addrlen = sizeof(peer);
	}
	k_mutex_unlock(&socket_lock);
	zvfs_finalize_typed_fd(fd, client, (const struct fd_op_vtable *)&socket_ops,
			       ZVFS_MODE_IFSOCK);
	return fd;
error:
	k_mutex_unlock(&socket_lock);
	zvfs_free_fd(fd);
	return fail(ret);
}

static int do_connect(void *obj, const struct net_sockaddr *addr,
		      net_socklen_t addrlen)
{
	struct nina_sock *s = obj;
	const struct net_sockaddr_in *in = (const struct net_sockaddr_in *)addr;
	uint8_t result, state;
	size_t n;
	struct nina_param p[3];
	int64_t deadline;
	int ret;

	if (!addr || addrlen < sizeof(*in) || addr->sa_family != NET_AF_INET) {
		return fail(EINVAL);
	}
	if (s->udp) {
		/* Kept here: datagrams from other senders are filtered on receive. */
		ret = open_socket(s);
		if (ret) {
			return fail(ret);
		}
		s->peer = *in;
		s->connected = true;
		return 0;
	}
	if (s->connected) {
		return fail(EISCONN);
	}
	if (s->listening) {
		return fail(EINVAL);
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = open_locked(s);
	if (!ret) {
		p[0] = (struct nina_param){&s->id, 1};
		p[1] = (struct nina_param){(const uint8_t *)&in->sin_addr, 4};
		p[2] = (struct nina_param){(const uint8_t *)&in->sin_port, 2};
		ret = nina_request(CMD_CONNECT, p, 3, false, false, &result, 1, &n);
		if (!ret && n != 1) {
			ret = -EPROTO;
		}
	}
	k_mutex_unlock(&socket_lock);
	if (ret) {
		goto failed;
	}
	/* The socket is non-blocking, so "not connected yet" is the normal
	 * answer. It becomes writable when the handshake completes and reports
	 * an error when it fails. */
	deadline = k_uptime_get() + NINA_CONNECT_WAIT_MS;
	for (;;) {
		ret = poll_state(s, &state);
		if (ret) {
			goto failed;
		}
		if (state & (POLL_WRITE | POLL_ERROR)) {
			ret = socket_error(s);
			if (ret < 0) {
				goto failed;
			}
			if (ret > 0 || (state & POLL_ERROR)) {
				ret = -connect_errno(ret);
				goto failed;
			}
			s->connected = true;
			return 0;
		}
		if (k_uptime_get() >= deadline) {
			ret = -ETIMEDOUT;
			goto failed;
		}
		k_sleep(K_MSEC(NINA_POLL_INTERVAL_MS));
	}
failed:
	/* A failed lwIP socket cannot be connected again. */
	k_mutex_lock(&socket_lock, K_FOREVER);
	close_id(s);
	k_mutex_unlock(&socket_lock);
	return fail(ret);
}

static ssize_t udp_send(struct nina_sock *s, const void *buf, size_t len,
			int flags, const struct net_sockaddr *to,
			net_socklen_t tolen)
{
	const struct net_sockaddr_in *peer = (const struct net_sockaddr_in *)to;
	struct nina_param p[4];
	uint8_t answer[2];
	size_t n;
	int ret;

	if ((flags & ~ZSOCK_MSG_DONTWAIT) || len > NINA_MAX_IO) {
		return fail(len > NINA_MAX_IO ? EMSGSIZE : EOPNOTSUPP);
	}
	if (len && !buf) {
		return fail(EINVAL);
	}
	if (to && (tolen < sizeof(*peer) || to->sa_family != NET_AF_INET)) {
		return fail(EINVAL);
	}
	if (!to) {
		if (!s->connected) {
			return fail(EDESTADDRREQ);
		}
		peer = &s->peer;
	}
	ret = open_socket(s);
	if (ret) {
		return fail(ret);
	}
	p[0] = (struct nina_param){&s->id, 1};
	p[1] = (struct nina_param){(const uint8_t *)&peer->sin_addr, 4};
	p[2] = (struct nina_param){(const uint8_t *)&peer->sin_port, 2};
	p[3] = (struct nina_param){buf, len};
	ret = nina_request(CMD_SENDTO, p, 4, true, false, answer, sizeof(answer), &n);
	if (ret) {
		return fail(ret);
	}
	if (n != 2) {
		return fail(EPROTO);
	}
	/* The firmware reports every failure as zero bytes sent. */
	if ((((size_t)answer[0] << 8) | answer[1]) != len) {
		return fail(EAGAIN);
	}
	return len;
}

static ssize_t do_send(void *obj, const void *buf, size_t len, int flags,
		       const struct net_sockaddr *to, net_socklen_t tolen)
{
	struct nina_sock *s = obj;
	uint8_t answer[2];
	size_t n;
	int ret;
	struct nina_param p[2];
	int64_t deadline;
	int64_t next_state_check = 0;

	if (s->udp) {
		return udp_send(s, buf, len, flags, to, tolen);
	}
	if (to || (flags & ~ZSOCK_MSG_DONTWAIT)) {
		return fail(EOPNOTSUPP);
	}
	if (!s->connected) {
		return fail(ENOTCONN);
	}
	if (!len) {
		return 0;
	}
	if (!buf) {
		return fail(EINVAL);
	}

	len = MIN(len, CONFIG_NINA_TCP_MAX_IO);
	p[0] = (struct nina_param){&s->id, 1};
	p[1] = (struct nina_param){buf, len};
	deadline = k_uptime_get() + NINA_SEND_WAIT_MS;
	for (;;) {
		unsigned int written;

		ret = nina_request(CMD_SEND, p, 2, true, false, answer,
				   sizeof(answer), &n);
		if (ret) {
			return fail(ret);
		}
		if (n != 2) {
			return fail(EPROTO);
		}
		written = ((unsigned int)answer[0] << 8) | answer[1];
		if (written > len) {
			return fail(EPROTO);
		}
		if (written) {
			return written;
		}
		if ((flags & ZSOCK_MSG_DONTWAIT) || s->nonblocking) {
			return fail(EAGAIN);
		}
		/* Zero means a full send buffer or a failed connection; only the
		 * socket state tells them apart. */
		if (k_uptime_get() >= next_state_check)
		{
			uint8_t state;

			ret = poll_state(s, &state);
			if (ret || (state & POLL_ERROR)) {
				return fail(ret ? ret : EPIPE);
			}
			next_state_check = k_uptime_get() + NINA_SEND_STATE_INTERVAL_MS;
		}
		if (k_uptime_get() >= deadline) {
			return fail(EAGAIN);
		}
		k_sleep(K_MSEC(CONFIG_NINA_SEND_RETRY_MS));
	}
}

static ssize_t do_sendmsg(void *obj, const struct net_msghdr *msg, int flags)
{
	struct nina_sock *s = obj;
	size_t total = 0;
	ssize_t ret;

	if (!msg || (msg->msg_iovlen && !msg->msg_iov)) {
		return fail(EINVAL);
	}
	if (msg->msg_name || msg->msg_controllen) {
		return fail(EOPNOTSUPP);
	}
	if (s->udp) {
		return fail(EOPNOTSUPP);
	}
	/* Coalesce vectors so MQTT's header and payload share an SPI request.
	 * Return the actual partial write; the caller advances its iovecs.
	 */
	k_mutex_lock(&sendmsg_lock, K_FOREVER);
	for (size_t i = 0; i < msg->msg_iovlen; i++) {
		size_t n = MIN(msg->msg_iov[i].iov_len, sizeof(sendmsg_buffer) - total);

		if (n && !msg->msg_iov[i].iov_base) {
			k_mutex_unlock(&sendmsg_lock);
			return fail(EFAULT);
		}
		if (n) {
			memcpy(sendmsg_buffer + total, msg->msg_iov[i].iov_base, n);
			total += n;
		}
		if (total == sizeof(sendmsg_buffer)) {
			break;
		}
	}
	ret = total ? do_send(obj, sendmsg_buffer, total, flags, NULL, 0) : 0;
	k_mutex_unlock(&sendmsg_lock);
	return ret;
}

static int64_t receive_deadline(const struct nina_sock *s)
{
	return s->receive_timeout_ms > 0 ? k_uptime_get() + s->receive_timeout_ms : INT64_MAX;
}

static ssize_t udp_recv(struct nina_sock *s, void *buf, size_t len, int flags,
			struct net_sockaddr *from, net_socklen_t *fromlen)
{
	struct net_sockaddr_in sender = {.sin_family = NET_AF_INET};
	uint8_t state, size[2];
	struct nina_param p[2];
	size_t n, copied;
	int64_t deadline;
	int ret;

	if ((flags & ~ZSOCK_MSG_DONTWAIT) || (!!from != !!fromlen)) {
		return fail(EOPNOTSUPP);
	}
	if (len && !buf) {
		return fail(EINVAL);
	}
	ret = open_socket(s);
	if (ret) {
		return fail(ret);
	}
	deadline = receive_deadline(s);
	for (;;) {
		/* Never ask for a datagram that is not there: the reply cannot
		 * tell "nothing received" from an empty datagram. */
		ret = poll_state(s, &state);
		if (ret) {
			return fail(ret);
		}
		if (!(state & POLL_READ)) {
			if ((flags & ZSOCK_MSG_DONTWAIT) || s->nonblocking ||
			    k_uptime_get() >= deadline) {
				return fail(EAGAIN);
			}
			k_sleep(K_MSEC(NINA_POLL_INTERVAL_MS));
			continue;
		}
		/* lwIP drops what does not fit, which keeps datagram boundaries. */
		copied = CLAMP(len, 1, NINA_MAX_IO);
		size[0] = copied & 0xff;
		size[1] = copied >> 8;
		p[0] = (struct nina_param){&s->id, 1};
		p[1] = (struct nina_param){size, 2};
		k_mutex_lock(&udp_lock, K_FOREVER);
		ret = nina_request(CMD_RECVFROM, p, 2, false, true, udp_buffer,
				   6 + copied, &n);
		if (ret || n < 6) {
			k_mutex_unlock(&udp_lock);
			return fail(ret ? ret : EPROTO);
		}
		memcpy(&sender.sin_addr, udp_buffer, 4);
		/* Same byte order as the port in the accept reply. */
		((uint8_t *)&sender.sin_port)[0] = udp_buffer[5];
		((uint8_t *)&sender.sin_port)[1] = udp_buffer[4];
		copied = MIN(n - 6, len);
		if (copied) {
			memcpy(buf, udp_buffer + 6, copied);
		}
		k_mutex_unlock(&udp_lock);
		if (s->connected && (sender.sin_addr.s_addr != s->peer.sin_addr.s_addr ||
				     sender.sin_port != s->peer.sin_port)) {
			continue;
		}
		if (from) {
			net_socklen_t capacity = *fromlen;

			memcpy(from, &sender, MIN(capacity, sizeof(sender)));
			*fromlen = sizeof(sender);
		}
		return copied;
	}
}

static ssize_t do_recv(void *obj, void *buf, size_t len, int flags,
		       struct net_sockaddr *from, net_socklen_t *fromlen)
{
	struct nina_sock *s = obj;
	uint8_t state, bytes[2];
	size_t n;
	int64_t deadline;
	int ret;
	struct nina_param p[2];

	if (s->udp) {
		return udp_recv(s, buf, len, flags, from, fromlen);
	}
	if (from || fromlen || (flags & ~ZSOCK_MSG_DONTWAIT)) {
		return fail(EOPNOTSUPP);
	}
	if (!s->connected) {
		return fail(ENOTCONN);
	}
	if (!len) {
		return 0;
	}
	if (!buf) {
		return fail(EINVAL);
	}
	deadline = receive_deadline(s);
	for (;;) {
		ret = poll_state(s, &state);
		if (ret) {
			return fail(ret);
		}
		if (state & POLL_READ) {
			break;
		}
		if ((flags & ZSOCK_MSG_DONTWAIT) || s->nonblocking || k_uptime_get() >= deadline) {
			return fail(EAGAIN);
		}
		k_sleep(K_MSEC(NINA_POLL_INTERVAL_MS));
	}
	len = MIN(len, NINA_MAX_IO);
	bytes[0] = len & 0xff;
	bytes[1] = len >> 8;
	p[0] = (struct nina_param){&s->id, 1};
	p[1] = (struct nina_param){bytes, 2};
	ret = nina_request(CMD_RECV, p, 2, false, true, buf, len, &n);
	if (ret) {
		return fail(ret);
	}
	/* Readable with nothing to read: the peer closed or the connection
	 * failed. The firmware reports both as zero bytes. */
	return (ssize_t)n;
}

static int do_close(void *obj)
{
	struct nina_sock *s = obj;
	uint8_t result;
	size_t n;
	int ret = 0;

	k_mutex_lock(&socket_lock, K_FOREVER);
	if (s->id != NINA_NO_SOCKET) {
		ret = cmd1(CMD_CLOSE, s->id, &result, 1, &n);
		if (!ret && (n != 1 || result != 1)) {
			ret = -EIO;
		}
	}
	s->used = false;
	s->connected = false;
	s->bound = false;
	s->listening = false;
	s->id = NINA_NO_SOCKET;
	k_mutex_unlock(&socket_lock);
	return ret ? fail(ret) : 0;
}

static ssize_t do_read(void *obj, void *buf, size_t count)
{
	return do_recv(obj, buf, count, 0, NULL, NULL);
}

static ssize_t do_write(void *obj, const void *buf, size_t count)
{
	return do_send(obj, buf, count, 0, NULL, 0);
}

static int do_poll(struct zsock_pollfd *fds, int nfds, int timeout)
{
	int64_t deadline = timeout < 0 ? INT64_MAX : k_uptime_get() + timeout;

	for (;;) {
		int ready = 0;

		for (int i = 0; i < nfds; i++) {
			struct nina_sock *s;
			const struct fd_op_vtable *vtable;
			uint8_t state;
			int ret;

			fds[i].revents = 0;
			if (fds[i].fd < 0) {
				continue;
			}
			s = zvfs_get_fd_obj_and_vtable(fds[i].fd, &vtable,
						       NULL);
			if (!s) {
				fds[i].revents = ZSOCK_POLLNVAL;
			} else if (vtable !=
				   (const struct fd_op_vtable *)&socket_ops) {
				struct zsock_pollfd native = fds[i];

				ret = zsock_poll(&native, 1, 0);
				if (ret < 0) {
					return -1;
				}
				fds[i].revents = native.revents;
			} else if (s->id == NINA_NO_SOCKET) {
				/* Nothing can arrive on a socket the firmware does
				 * not have yet; an unconnected TCP socket is hung up. */
				if (!s->udp) {
					fds[i].revents = ZSOCK_POLLHUP;
				} else if (fds[i].events & ZSOCK_POLLOUT) {
					fds[i].revents = ZSOCK_POLLOUT;
				}
			} else {
				ret = poll_state(s, &state);
				if (ret) {
					return fail(ret);
				}
				if ((state & POLL_READ) && (fds[i].events & ZSOCK_POLLIN)) {
					fds[i].revents |= ZSOCK_POLLIN;
				}
				if ((state & POLL_WRITE) && (fds[i].events & ZSOCK_POLLOUT)) {
					fds[i].revents |= ZSOCK_POLLOUT;
				}
				if (state & POLL_ERROR) {
					fds[i].revents |= ZSOCK_POLLERR;
				}
			}
			if (fds[i].revents) {
				ready++;
			}
		}
		if (ready) {
			return ready;
		}
		if (k_uptime_get() >= deadline) {
			return 0;
		}
		k_sleep(K_MSEC(NINA_POLL_INTERVAL_MS));
	}
}

/* Only the receive timeout takes effect, and it is kept here. Options are
 * not passed on to the module: its setsockopt command reaches SOL_SOCKET
 * options only, and the sockets there are always non-blocking. */
static int do_setsockopt(void *obj, int level, int optname, const void *optval,
			 net_socklen_t optlen)
{
	struct nina_sock *s = obj;
	const struct zsock_timeval *tv = optval;
	int64_t ms;

	/* Accepted without effect: the module's setsockopt command cannot reach
	 * TCP-level options, so Nagle's algorithm stays on. Callers such as the
	 * iperf3 client set this on a control connection and give up if they
	 * cannot. */
	if (level == NET_IPPROTO_TCP && optname == ZSOCK_TCP_NODELAY && !s->udp) {
		return optval && optlen == sizeof(int) ? 0 : fail(EINVAL);
	}
	if (level != ZSOCK_SOL_SOCKET || optname != ZSOCK_SO_RCVTIMEO) {
		return fail(ENOPROTOOPT);
	}
	if (!optval || optlen != sizeof(*tv) || tv->tv_sec < 0 || tv->tv_usec < 0 ||
	    tv->tv_usec >= USEC_PER_SEC) {
		return fail(EINVAL);
	}
	ms = (int64_t)tv->tv_sec * MSEC_PER_SEC + (tv->tv_usec + USEC_PER_MSEC - 1) / USEC_PER_MSEC;
	if (ms > INT32_MAX) {
		return fail(EDOM);
	}
	s->receive_timeout_ms = (int32_t)ms;
	return 0;
}

static int do_ioctl(void *obj, unsigned int request, va_list args)
{
	struct nina_sock *s = obj;

	switch (request) {
	case ZVFS_F_GETFL:
		return s->nonblocking ? ZVFS_O_NONBLOCK : 0;
	case ZVFS_F_SETFL: {
		int flags = va_arg(args, int);

		if (flags & ~ZVFS_O_NONBLOCK) {
			return fail(EOPNOTSUPP);
		}
		s->nonblocking = (flags & ZVFS_O_NONBLOCK) != 0;
		return 0;
	}
	case ZFD_IOCTL_POLL_PREPARE:
		return -EXDEV;
	case ZFD_IOCTL_POLL_OFFLOAD: {
		struct zsock_pollfd *fds = va_arg(args, struct zsock_pollfd *);
		int nfds = va_arg(args, int);
		int timeout = va_arg(args, int);
		return do_poll(fds, nfds, timeout);
	}
	default:
		return fail(ENOTSUP);
	}
}
static const struct socket_op_vtable socket_ops = {
	.fd_vtable = {.read = do_read,
		      .write = do_write,
		      .close = do_close,
		      .ioctl = do_ioctl},
	.bind = do_bind,
	.connect = do_connect,
	.setsockopt = do_setsockopt,
	.listen = do_listen,
	.accept = do_accept,
	.sendto = do_send,
	.sendmsg = do_sendmsg,
	.recvfrom = do_recv,
};
static bool supported(int family, int type, int proto)
{
	return family == NET_AF_INET &&
	       ((type == NET_SOCK_STREAM &&
		 (proto == 0 || proto == NET_IPPROTO_TCP)) ||
		(type == NET_SOCK_DGRAM &&
		 (proto == 0 || proto == NET_IPPROTO_UDP)));
}

int nina_socket_create(int family, int type, int proto)
{
	int fd;
	struct nina_sock *s = NULL;

	if (!supported(family, type, proto)) {
		return fail(EAFNOSUPPORT);
	}
	fd = zvfs_reserve_fd();
	if (fd < 0) {
		return -1;
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	for (int i = 0; i < MAX_NINA_SOCKETS; i++) {
		if (!sockets[i].used) {
			s = &sockets[i];
			break;
		}
	}
	if (!s) {
		k_mutex_unlock(&socket_lock);
		zvfs_free_fd(fd);
		return fail(EMFILE);
	}
	s->id = NINA_NO_SOCKET;
	s->used = true;
	s->connected = false;
	s->udp = type == NET_SOCK_DGRAM;
	s->bound = false;
	s->listening = false;
	s->nonblocking = false;
	s->receive_timeout_ms = 0;
	k_mutex_unlock(&socket_lock);
	zvfs_finalize_typed_fd(fd, s, (const struct fd_op_vtable *)&socket_ops,
			       ZVFS_MODE_IFSOCK);
	return fd;
}
NET_SOCKET_OFFLOAD_REGISTER(nina, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY,
			    NET_AF_INET, supported, nina_socket_create);
