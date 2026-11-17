/* SPDX-License-Identifier: MIT */
/* The sockets of the NINA-W102 on FreeRTOS: the zsock_*() calls of the
 * Zephyr compatibility layer (<zephyr/net/socket.h>) on the shared NINA
 * core (common/drivers/nina/nina_sock.c, nina_dns.c).  This is the Zephyr
 * build's socket offload (zephyr/app/usb-console/drivers/nina/
 * nina_socket.c, "nina_socket.c" below) with the parts of Zephyr's socket
 * layer it relies on (subsys/net/lib/sockets/sockets.c, lib/os/zvfs), in
 * the form of the offload: every call runs in the caller's thread and
 * waits there, polling the module, as there.  There is no event thread
 * (the NuttX build's usrsock daemon and nina_poller.c): the core's
 * functions make one pass of each of the Zephyr build's wait loops and this
 * file makes the loops, with the Zephyr build's sleeps.
 *
 * The descriptors.  A descriptor is the number of its NINA socket in the
 * core's table (0..NINA_SOCK_MAX-1, the firmware's ten lwIP sockets), and
 * fds[] holds what Zephyr's descriptor table and the offload's struct
 * nina_sock hold besides the core's state: the descriptor's lock (Zephyr's
 * per-descriptor mutex, which its socket layer holds around each call),
 * O_NONBLOCK and SO_RCVTIMEO.  Zephyr numbers its descriptors and the
 * offload its sockets independently, both from the lowest free entry; with
 * only sockets in the table (CONFIG_ZVFS_OPEN_MAX 16 is more than ten, so
 * the eleventh socket gets EMFILE from the offload, not ENFILE from the
 * table) the two always agree, so one number serves.  The numbers are not
 * newlib's descriptors (0, 1 and 2 are the console's): only the zsock_*()
 * calls take them, as in the Zephyr build (no POSIX names).
 *
 * The locks.  socket_lock is the table lock of the core (nina_sock.h:
 * every core call holds it), in place of the offload's socket_lock,
 * udp_lock and sendmsg_lock: it is held for one pass of a loop (a few
 * NINA commands at most), never across a sleep.  Each NINA command also
 * takes the NINA lock inside nina_request(): the order is always the
 * descriptor's lock, socket_lock, the NINA lock.  The descriptor's lock is
 * held around each call as Zephyr's socket layer holds it (VTABLE_CALL(),
 * zvfs_close(), the fcntl), across its waits: the calls on one descriptor
 * run one at a time, and a close waits for a call in progress.  zsock_poll()
 * takes no descriptor lock (Zephyr's offloaded poll takes none).  All are
 * the layer's k_mutex: FreeRTOS's priority inheritance, one level.
 *
 * Each call against nina_socket.c ("as there" means the same branches,
 * errno and sleeps):
 *
 *   zsock_socket()       supported(): AF_INET with SOCK_STREAM (0 or TCP) or
 *                        SOCK_DGRAM (0 or UDP), else EAFNOSUPPORT
 *                        (z_impl_zsock_socket()); nina_socket_create(): a
 *                        free entry or EMFILE, blocking, no receive timeout.
 *                        The firmware socket comes with the first use.
 *   zsock_close()        do_close() under the descriptor's lock
 *                        (zvfs_close()): CMD_CLOSE when the firmware socket
 *                        exists; the entry is freed even when it fails.
 *   zsock_shutdown()     ENOTSUP (the offload has no shutdown).
 *   zsock_bind(), _listen()  do_bind(), do_listen().
 *   zsock_accept()       do_accept(): never waits (EAGAIN when no client is
 *                        there, blocking or not); addr and addrlen both or
 *                        neither (EINVAL); the new socket is blocking.
 *   zsock_connect()      do_connect(): a TCP connection is waited for in
 *                        the call, also on a non-blocking socket (as
 *                        there): a check at once, then every
 *                        NINA_POLL_INTERVAL_MS (a sleep) up to
 *                        NINA_CONNECT_WAIT_MS (ETIMEDOUT); the lwIP error
 *                        becomes ECONNRESET, ECONNABORTED, ETIMEDOUT,
 *                        EHOSTUNREACH or ECONNREFUSED; a failed socket is
 *                        closed in the firmware.  UDP: the peer is kept.
 *                        The deadline is compared with the time taken
 *                        before each check (nina_socket.c: after it).
 *   zsock_sendto()       do_send()/udp_send(): a TCP send of up to
 *                        NINA_TCP_MAX_IO (3584) bytes in one command; no
 *                        room: EAGAIN at once with MSG_DONTWAIT or
 *                        O_NONBLOCK, else tried again after
 *                        NINA_SEND_RETRY_MS (1 ms, a sleep) with a check
 *                        of the socket every NINA_SEND_STATE_INTERVAL_MS
 *                        (EPIPE on its error) until NINA_SEND_WAIT_MS
 *                        (EAGAIN).  A datagram of up to NINA_MAX_IO (512)
 *                        bytes, EAGAIN if the module did not take it.
 *   zsock_sendmsg()      do_sendmsg(): the vectors joined (up to
 *                        NINA_TCP_MAX_IO bytes) and sent as above; TCP
 *                        only, no address, no control data.  The offload
 *                        holds sendmsg_lock across the whole call; here the
 *                        vectors are copied again at each pass, under
 *                        socket_lock.
 *   zsock_recvfrom()     do_recv()/udp_recv(): CMD_POLL, then the read when
 *                        the module has data (TCP: up to NINA_MAX_IO bytes;
 *                        0 is the end of the stream or a failed
 *                        connection); nothing: EAGAIN at once with
 *                        MSG_DONTWAIT or O_NONBLOCK, else again every
 *                        NINA_POLL_INTERVAL_MS until SO_RCVTIMEO has gone
 *                        by (EAGAIN; 0 waits without limit).  A connected
 *                        UDP socket drops datagrams from other senders.
 *                        Every pass polls the module, as there (the core's
 *                        read_hint, which stands for the poll on NuttX, is
 *                        cleared first).
 *   zsock_poll()         do_poll(): every descriptor at once (POLLIN and
 *                        POLLOUT as asked, POLLERR on the module's error,
 *                        POLLHUP alone for a TCP socket the firmware does
 *                        not have yet, POLLOUT if asked for such a UDP
 *                        socket, POLLNVAL for a descriptor that is not
 *                        open; a negative one is skipped), then again every
 *                        NINA_POLL_INTERVAL_MS until one is ready or the
 *                        timeout (ms, negative: none) has gone by.  A
 *                        failed poll of the module ends the call (-1, its
 *                        errno).
 *   zsock_setsockopt()   do_setsockopt(): TCP_NODELAY on a TCP socket is
 *                        accepted without effect (an int); SO_RCVTIMEO (a
 *                        struct timeval: rounded up to ms, EINVAL for a bad
 *                        one, EDOM above INT32_MAX ms); anything else
 *                        ENOPROTOOPT (the core's own NINA_SOL options of the
 *                        NuttX build included).
 *   zsock_getsockopt(), zsock_getsockname()
 *                        EOPNOTSUPP (the offload has neither: VTABLE_CALL()).
 *   zsock_fcntl()        do_ioctl(): F_GETFL, F_SETFL with O_NONBLOCK only
 *                        (else EOPNOTSUPP); other commands ENOTSUP.
 *   zsock_getaddrinfo(), zsock_freeaddrinfo()
 *                        the module's resolver (nina_getaddrinfo(), the
 *                        Zephyr build's nina_dns.c registered as the
 *                        offload's resolver), on the heap rather than a
 *                        2048-byte k_heap.
 *
 * The waits are the core's boundary (nina_os.h, freertos/drivers/nina/
 * nina_os.c): nina_os_sleep_ms() is k_sleep(K_MSEC()), the time rounded up
 * to 100 us ticks plus one tick (20 ms: 201 ticks, 1 ms: 11 ticks, as in
 * the Zephyr build with its 10 kHz tick), and nina_os_uptime_ms() is
 * k_uptime_get(), the clock of the core's deadlines; the host test
 * (freertos/tests/socket) replaces both.
 *
 * A descriptor that is not open (or out of range) gives EBADF, as Zephyr's
 * socket layer does, before anything else.  The errors of the module's
 * link (nina_request(): ETIMEDOUT, EPROTO, EMSGSIZE, ENODEV, ...) pass
 * through as errno, as there.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>

#include "nina_dns.h"
#include "nina_os.h"
#include "nina_sock.h"

/* <zephyr/sys/time_units.h>'s (the layer's <zephyr/sys/util.h> has the
 * last). */
#ifndef MSEC_PER_SEC
#define MSEC_PER_SEC 1000
#endif
#ifndef USEC_PER_MSEC
#define USEC_PER_MSEC 1000
#endif
#ifndef USEC_PER_SEC
#define USEC_PER_SEC 1000000
#endif

struct zsock_fd {
	struct k_mutex lock;
	bool nonblocking;
	int32_t receive_timeout_ms; /* SO_RCVTIMEO; 0 waits without limit */
};

static struct zsock_fd fds[NINA_SOCK_MAX];
static K_MUTEX_DEFINE(socket_lock);

static int fail(int err)
{
	errno = err > 0 ? err : -err;
	return -1;
}

static bool is_open(int sock)
{
	bool open;

	k_mutex_lock(&socket_lock, K_FOREVER);
	open = nina_sock_get(sock) != NULL;
	k_mutex_unlock(&socket_lock);
	return open;
}

/* get_sock_vtable() and the lock of VTABLE_CALL(): the descriptor's entry,
 * locked, or NULL with errno EBADF.  Checked again under the lock: a close
 * that came first has freed it.
 */
static struct zsock_fd *fd_lock(int sock)
{
	struct zsock_fd *fd;

	if (sock < 0 || sock >= NINA_SOCK_MAX) {
		errno = EBADF;
		return NULL;
	}
	fd = &fds[sock];
	k_mutex_lock(&fd->lock, K_FOREVER);
	if (!is_open(sock)) {
		k_mutex_unlock(&fd->lock);
		errno = EBADF;
		return NULL;
	}
	return fd;
}

static void fd_unlock(struct zsock_fd *fd)
{
	k_mutex_unlock(&fd->lock);
}

/* A new descriptor's state (nina_socket_create(), do_accept()). Caller
 * holds socket_lock. */
static void fd_reset(int sock)
{
	fds[sock].nonblocking = false;
	fds[sock].receive_timeout_ms = 0;
}

int zsock_socket(int family, int type, int proto)
{
	int sock;

	if (!nina_sock_supported(family, type, proto)) {
		return fail(EAFNOSUPPORT);
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	sock = nina_sock_create(family, type, proto);
	if (sock >= 0) {
		fd_reset(sock);
	}
	k_mutex_unlock(&socket_lock);
	return sock < 0 ? fail(sock) : sock;
}

int zsock_close(int sock)
{
	struct zsock_fd *fd = fd_lock(sock);
	int ret;

	if (!fd) {
		return -1;
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = nina_sock_close(sock);
	fd_reset(sock);
	k_mutex_unlock(&socket_lock);
	fd_unlock(fd);
	return ret ? fail(ret) : 0;
}

int zsock_shutdown(int sock, int how)
{
	(void)how;
	return fail(is_open(sock) ? ENOTSUP : EBADF);
}

int zsock_bind(int sock, const struct sockaddr *addr, socklen_t addrlen)
{
	struct zsock_fd *fd = fd_lock(sock);
	int ret;

	if (!fd) {
		return -1;
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = nina_sock_bind(sock, addr, addrlen);
	k_mutex_unlock(&socket_lock);
	fd_unlock(fd);
	return ret ? fail(ret) : 0;
}

int zsock_listen(int sock, int backlog)
{
	struct zsock_fd *fd = fd_lock(sock);
	int ret;

	if (!fd) {
		return -1;
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = nina_sock_listen(sock, backlog);
	k_mutex_unlock(&socket_lock);
	fd_unlock(fd);
	return ret ? fail(ret) : 0;
}

int zsock_accept(int sock, struct sockaddr *addr, socklen_t *addrlen)
{
	struct zsock_fd *fd = fd_lock(sock);
	struct sockaddr_in peer;
	int ret;

	if (!fd) {
		return -1;
	}
	/* do_accept() checks this with "listening" (the core's): EINVAL. */
	if (!!addr != !!addrlen) {
		fd_unlock(fd);
		return fail(EINVAL);
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = nina_sock_accept(sock, &peer);
	if (ret >= 0) {
		fd_reset(ret);
	}
	k_mutex_unlock(&socket_lock);
	fd_unlock(fd);
	if (ret < 0) {
		return fail(ret);
	}
	if (addr) {
		memcpy(addr, &peer, MIN(*addrlen, sizeof(peer)));
		*addrlen = sizeof(peer);
	}
	return ret;
}

int zsock_connect(int sock, const struct sockaddr *addr, socklen_t addrlen)
{
	struct zsock_fd *fd = fd_lock(sock);
	int ret;

	if (!fd) {
		return -1;
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = nina_sock_connect(sock, addr, addrlen);
	k_mutex_unlock(&socket_lock);
	/* do_connect()'s wait loop: 1 while the handshake is in progress. */
	while (ret == 1) {
		nina_os_sleep_ms(NINA_POLL_INTERVAL_MS);
		k_mutex_lock(&socket_lock, K_FOREVER);
		ret = nina_sock_connect_check(sock, nina_os_uptime_ms());
		k_mutex_unlock(&socket_lock);
	}
	fd_unlock(fd);
	return ret ? fail(ret) : 0;
}

ssize_t zsock_sendto(int sock, const void *buf, size_t len, int flags,
		     const struct sockaddr *dest_addr, socklen_t addrlen)
{
	struct zsock_fd *fd = fd_lock(sock);
	ssize_t ret;
	bool retry;

	if (!fd) {
		return -1;
	}
	/* do_send(): "(flags & ZSOCK_MSG_DONTWAIT) || s->nonblocking". */
	if (fd->nonblocking) {
		flags |= MSG_DONTWAIT;
	}
	for (;;) {
		k_mutex_lock(&socket_lock, K_FOREVER);
		ret = nina_sock_send(sock, buf, len, flags, dest_addr, addrlen, &retry);
		k_mutex_unlock(&socket_lock);
		if (ret != -EAGAIN || !retry) {
			break;
		}
		nina_os_sleep_ms(NINA_SEND_RETRY_MS);
	}
	fd_unlock(fd);
	return ret < 0 ? fail((int)ret) : ret;
}

ssize_t zsock_sendmsg(int sock, const struct msghdr *msg, int flags)
{
	struct zsock_fd *fd = fd_lock(sock);
	ssize_t ret;
	bool retry;

	if (!fd) {
		return -1;
	}
	if (!msg) {
		fd_unlock(fd);
		return fail(EINVAL);
	}
	if (fd->nonblocking) {
		flags |= MSG_DONTWAIT;
	}
	for (;;) {
		k_mutex_lock(&socket_lock, K_FOREVER);
		ret = nina_sock_sendmsg(sock, msg->msg_iov, msg->msg_iovlen,
					msg->msg_name || msg->msg_controllen, flags, &retry);
		k_mutex_unlock(&socket_lock);
		if (ret != -EAGAIN || !retry) {
			break;
		}
		nina_os_sleep_ms(NINA_SEND_RETRY_MS);
	}
	fd_unlock(fd);
	return ret < 0 ? fail((int)ret) : ret;
}

ssize_t zsock_recvfrom(int sock, void *buf, size_t max_len, int flags, struct sockaddr *src_addr,
		       socklen_t *addrlen)
{
	struct zsock_fd *fd = fd_lock(sock);
	struct sockaddr_in from;
	struct nina_sock *s;
	int64_t deadline;
	bool nonblocking;
	ssize_t ret;

	if (!fd) {
		return -1;
	}
	/* do_recv() refuses an address for TCP, udp_recv() an address without
	 * its length or the other way round: EOPNOTSUPP.  The core sees the
	 * rest (an address for TCP, the flags). */
	if (!!src_addr != !!addrlen) {
		fd_unlock(fd);
		return fail(EOPNOTSUPP);
	}
	nonblocking = (flags & MSG_DONTWAIT) || fd->nonblocking;
	/* receive_deadline() */
	deadline = fd->receive_timeout_ms > 0 ? nina_os_uptime_ms() + fd->receive_timeout_ms
					      : INT64_MAX;
	for (;;) {
		k_mutex_lock(&socket_lock, K_FOREVER);
		s = nina_sock_get(sock);
		if (s) {
			s->read_hint = false;
		}
		ret = nina_sock_recv(sock, buf, max_len, flags, src_addr ? &from : NULL);
		k_mutex_unlock(&socket_lock);
		if (ret != -EAGAIN || nonblocking || nina_os_uptime_ms() >= deadline) {
			break;
		}
		nina_os_sleep_ms(NINA_POLL_INTERVAL_MS);
	}
	fd_unlock(fd);
	if (ret < 0) {
		return fail((int)ret);
	}
	if (src_addr) {
		memcpy(src_addr, &from, MIN(*addrlen, sizeof(from)));
		*addrlen = sizeof(from);
	}
	return ret;
}

int zsock_poll(struct zsock_pollfd *pfds, int nfds, int timeout)
{
	int64_t deadline = timeout < 0 ? INT64_MAX : nina_os_uptime_ms() + timeout;

	for (;;) {
		int ready = 0;

		for (int i = 0; i < nfds; i++) {
			int revents;

			pfds[i].revents = 0;
			if (pfds[i].fd < 0) {
				continue;
			}
			k_mutex_lock(&socket_lock, K_FOREVER);
			if (!nina_sock_get(pfds[i].fd)) {
				revents = ZSOCK_POLLNVAL;
			} else {
				revents = nina_sock_poll_revents(pfds[i].fd);
			}
			k_mutex_unlock(&socket_lock);
			if (revents < 0) {
				return fail(revents);
			}
			if (revents != ZSOCK_POLLNVAL) {
				revents &= pfds[i].events | ZSOCK_POLLERR | ZSOCK_POLLHUP;
			}
			pfds[i].revents = (short)revents;
			if (revents) {
				ready++;
			}
		}
		if (ready) {
			return ready;
		}
		if (nina_os_uptime_ms() >= deadline) {
			return 0;
		}
		nina_os_sleep_ms(NINA_POLL_INTERVAL_MS);
	}
}

int zsock_setsockopt(int sock, int level, int optname, const void *optval, socklen_t optlen)
{
	struct zsock_fd *fd = fd_lock(sock);
	const struct zsock_timeval *tv = optval;
	int64_t ms;
	int ret;

	if (!fd) {
		return -1;
	}
	if (level == SOL_SOCKET && optname == SO_RCVTIMEO) {
		if (!optval || optlen != sizeof(*tv) || tv->tv_sec < 0 || tv->tv_usec < 0 ||
		    tv->tv_usec >= USEC_PER_SEC) {
			ret = -EINVAL;
		} else {
			ms = (int64_t)tv->tv_sec * MSEC_PER_SEC +
			     (tv->tv_usec + USEC_PER_MSEC - 1) / USEC_PER_MSEC;
			if (ms > INT32_MAX) {
				ret = -EDOM;
			} else {
				fd->receive_timeout_ms = (int32_t)ms;
				ret = 0;
			}
		}
	} else if (level == NINA_SOL) {
		/* The NuttX build's own options (nina_sockopt.h): not Zephyr's. */
		ret = -ENOPROTOOPT;
	} else {
		k_mutex_lock(&socket_lock, K_FOREVER);
		ret = nina_sock_setsockopt(sock, level, optname, optval, optlen);
		k_mutex_unlock(&socket_lock);
	}
	fd_unlock(fd);
	return ret ? fail(ret) : 0;
}

int zsock_getsockopt(int sock, int level, int optname, void *optval, socklen_t *optlen)
{
	(void)level;
	(void)optname;
	(void)optval;
	(void)optlen;
	return fail(is_open(sock) ? EOPNOTSUPP : EBADF);
}

int zsock_getsockname(int sock, struct sockaddr *addr, socklen_t *addrlen)
{
	(void)addr;
	(void)addrlen;
	return fail(is_open(sock) ? EOPNOTSUPP : EBADF);
}

int zsock_fcntl_impl(int sock, int cmd, int flags)
{
	struct zsock_fd *fd = fd_lock(sock);
	int ret;

	if (!fd) {
		return -1;
	}
	switch (cmd) {
	case F_GETFL:
		ret = fd->nonblocking ? O_NONBLOCK : 0;
		break;
	case F_SETFL:
		if (flags & ~O_NONBLOCK) {
			ret = -EOPNOTSUPP;
		} else {
			fd->nonblocking = (flags & O_NONBLOCK) != 0;
			ret = 0;
		}
		break;
	default:
		ret = -ENOTSUP;
		break;
	}
	fd_unlock(fd);
	return ret < 0 ? fail(ret) : ret;
}

int zsock_getaddrinfo(const char *host, const char *service, const struct zsock_addrinfo *hints,
		      struct zsock_addrinfo **res)
{
	return nina_getaddrinfo(host, service, hints, res);
}

void zsock_freeaddrinfo(struct zsock_addrinfo *ai)
{
	nina_freeaddrinfo(ai);
}
