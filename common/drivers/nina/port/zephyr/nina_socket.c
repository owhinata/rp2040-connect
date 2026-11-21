/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr socket offload on the NINA firmware's BSD-style socket commands
 * (0x70..0x7f). Each command is a single lwIP socket call made inside the
 * module's SPI command handler, which serves Wi-Fi management and every
 * socket. A call that blocks there stalls all of them, so every socket is
 * made non-blocking as soon as it exists and waiting is done here by polling.
 * The older per-type client commands (0x28..0x46) are not used.
 *
 * The socket table and the commands are the shared core's
 * (common/drivers/nina/nina_sock.c, nina_sock.h; also the NuttX and FreeRTOS
 * builds'). The core makes one pass of each wait loop and returns a
 * negative errno; this file is the offload around it: the file descriptors
 * (zvfs), errno, O_NONBLOCK and SO_RCVTIMEO (kept here, as the core leaves
 * them to the OS), and the loops with their sleeps:
 *
 * - connect() of a TCP socket: a check at once, then every
 *   NINA_POLL_INTERVAL_MS until NINA_CONNECT_WAIT_MS (the core compares the
 *   deadline with the time taken before each check).
 * - send(): tried again every CONFIG_NINA_SEND_RETRY_MS while the module
 *   has no room (the core keeps the deadline and the socket checks between
 *   the passes).
 * - recv(): polled every NINA_POLL_INTERVAL_MS until data, O_NONBLOCK or
 *   MSG_DONTWAIT, or SO_RCVTIMEO. Every pass polls the module: the core's
 *   read_hint (a poll that found data standing for the next receive's
 *   first poll, which the NuttX build uses) is cleared first.
 * - poll(): every descriptor at once, then every NINA_POLL_INTERVAL_MS. A
 *   socket closed meanwhile is ZSOCK_POLLNVAL, a failed poll of the module
 *   ZSOCK_POLLERR on that socket; the call fails only when a descriptor of
 *   another kind does (see poll_failed_revents()).
 *
 * The sleeps between the passes are nina_wait() (common/drivers/nina/
 * nina_wait.h): the intervals above, or, when the module reports socket
 * events on its IRQ line, until the line's next edge (at most
 * NINA_EVENT_TIMEOUT_MS, not past the call's deadline).  Each loop notes
 * the edge count (nina_wait_mark()) before its pass's commands.  A send
 * waits for the edge only after a pass that polled without sending (the
 * core's send_polled); after a pass that sent, CONFIG_NINA_SEND_RETRY_MS
 * as before.  A poll() with a descriptor of another kind waits at most
 * NINA_POLL_INTERVAL_MS, as before, for that descriptor.
 *
 * The locks. socket_lock is the core's table lock (nina_sock.h): every core
 * call holds it, for one pass of a loop (a few NINA commands at most), never
 * across a sleep. It also protects the core's datagram buffer (udp_lock
 * before the core). sendmsg_lock is held across a whole sendmsg(), sleeps
 * included, as before: the core copies the vectors into its one buffer at
 * every pass. The order is: the descriptor's lock (Zephyr's socket layer
 * holds it around each call), sendmsg_lock, socket_lock, then the NINA lock
 * inside nina_request().
 *
 * The resolver (common/drivers/nina/nina_dns.c, compiled with
 * NINA_OS_DNS_OFFLOAD) is registered here as the offload's getaddrinfo(),
 * its results on a 2048-byte k_heap.
 *
 * The core's POSIX names (struct sockaddr_in, AF_INET, MSG_DONTWAIT,
 * POLLIN, ...) are Zephyr's through the headers of posix/ here.
 */
#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/socket_offload.h>
#include <zephyr/sys/fdtable.h>
#include "nina_dns.h"
#include "nina_os.h"
#include "nina_proto.h"
#include "nina_sock.h"
#include "nina_wait.h"

#ifndef NINA_OS_DNS_OFFLOAD
#error "nina_dns.c is compiled with NINA_OS_DNS_OFFLOAD (CMakeLists.txt)"
#endif

/* What the offload keeps for each socket of the core's table (the same
 * index) besides the core's state. */
struct nina_fd {
	bool nonblocking;
	int32_t receive_timeout_ms; /* SO_RCVTIMEO; 0 waits without limit */
};
static struct nina_fd fds[NINA_SOCK_MAX];
K_MUTEX_DEFINE(socket_lock);
K_MUTEX_DEFINE(sendmsg_lock);
static const struct socket_op_vtable socket_ops;

/* The core's number of a descriptor's object. */
static int id_of(const void *obj)
{
	return (const struct nina_fd *)obj - fds;
}

/* A new socket's state (nina_socket_create(), do_accept()). */
static void fd_reset(int id)
{
	fds[id].nonblocking = false;
	fds[id].receive_timeout_ms = 0;
}

static int fail(int err)
{
	errno = err > 0 ? err : -err;
	return -1;
}

static int do_bind(void *obj, const struct net_sockaddr *addr, net_socklen_t addrlen)
{
	int ret;

	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = nina_sock_bind(id_of(obj), addr, addrlen);
	k_mutex_unlock(&socket_lock);
	return ret ? fail(ret) : 0;
}

static int do_listen(void *obj, int backlog)
{
	int ret;

	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = nina_sock_listen(id_of(obj), backlog);
	k_mutex_unlock(&socket_lock);
	return ret ? fail(ret) : 0;
}

static int do_accept(void *obj, struct net_sockaddr *addr, net_socklen_t *addrlen)
{
	struct nina_sock *listener;
	struct net_sockaddr_in peer;
	bool listening;
	int fd, ret;

	k_mutex_lock(&socket_lock, K_FOREVER);
	listener = nina_sock_get(id_of(obj));
	listening = listener && listener->listening;
	k_mutex_unlock(&socket_lock);
	/* Before the descriptor is reserved, as before the core. */
	if (!listening || (!!addr != !!addrlen)) {
		return fail(EINVAL);
	}
	fd = zvfs_reserve_fd();
	if (fd < 0) {
		return -1;
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = nina_sock_accept(id_of(obj), &peer);
	if (ret >= 0) {
		fd_reset(ret);
		if (addr) {
			memcpy(addr, &peer, MIN(*addrlen, sizeof(peer)));
			*addrlen = sizeof(peer);
		}
	}
	k_mutex_unlock(&socket_lock);
	if (ret < 0) {
		zvfs_free_fd(fd);
		return fail(ret);
	}
	zvfs_finalize_typed_fd(fd, &fds[ret], (const struct fd_op_vtable *)&socket_ops,
			       ZVFS_MODE_IFSOCK);
	return fd;
}

/* The deadline of a pass that is to be continued (connect or send), read
 * under socket_lock right after it. */
static int64_t pass_deadline(int id, bool connect)
{
	struct nina_sock *s = nina_sock_get(id);

	if (!s) {
		return 0;
	}
	return connect ? s->connect_deadline : s->send_deadline;
}

static int do_connect(void *obj, const struct net_sockaddr *addr, net_socklen_t addrlen)
{
	struct nina_wait wait = NINA_WAIT_INIT;
	int id = id_of(obj), ret;
	int64_t deadline;

	nina_wait_mark(&wait);
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = nina_sock_connect(id, addr, addrlen);
	deadline = pass_deadline(id, true);
	k_mutex_unlock(&socket_lock);
	/* 1 while the handshake is in progress. */
	while (ret == 1) {
		nina_wait(&wait, NINA_POLL_INTERVAL_MS, deadline);
		nina_wait_mark(&wait);
		k_mutex_lock(&socket_lock, K_FOREVER);
		ret = nina_sock_connect_check(id, k_uptime_get());
		k_mutex_unlock(&socket_lock);
	}
	return ret ? fail(ret) : 0;
}

/* After a send pass that sent nothing and waits: the edge after a pass
 * that only polled, else CONFIG_NINA_SEND_RETRY_MS. */
static void send_wait(struct nina_wait *wait, bool polled, int64_t deadline)
{
	if (polled) {
		nina_wait(wait, CONFIG_NINA_SEND_RETRY_MS, deadline);
	} else {
		k_sleep(K_MSEC(CONFIG_NINA_SEND_RETRY_MS));
	}
}

static ssize_t do_send(void *obj, const void *buf, size_t len, int flags,
		       const struct net_sockaddr *to, net_socklen_t tolen)
{
	struct nina_wait wait = NINA_WAIT_INIT;
	int id = id_of(obj);
	int64_t deadline;
	ssize_t ret;
	bool retry, polled;

	/* The core waits unless MSG_DONTWAIT; UDP never waits. */
	if (fds[id].nonblocking) {
		flags |= ZSOCK_MSG_DONTWAIT;
	}
	for (;;) {
		nina_wait_mark(&wait);
		k_mutex_lock(&socket_lock, K_FOREVER);
		ret = nina_sock_send(id, buf, len, flags, to, tolen, &retry);
		polled = retry && nina_sock_get(id)->send_polled;
		deadline = pass_deadline(id, false);
		k_mutex_unlock(&socket_lock);
		if (ret != -EAGAIN || !retry) {
			break;
		}
		send_wait(&wait, polled, deadline);
	}
	return ret < 0 ? fail((int)ret) : ret;
}

static ssize_t do_sendmsg(void *obj, const struct net_msghdr *msg, int flags)
{
	struct nina_wait wait = NINA_WAIT_INIT;
	int id = id_of(obj);
	int64_t deadline;
	ssize_t ret;
	bool retry, polled;

	if (!msg) {
		return fail(EINVAL);
	}
	if (fds[id].nonblocking) {
		flags |= ZSOCK_MSG_DONTWAIT;
	}
	/* Coalesce vectors so MQTT's header and payload share an SPI request
	 * (the core joins them). Return the actual partial write; the caller
	 * advances its iovecs.
	 */
	k_mutex_lock(&sendmsg_lock, K_FOREVER);
	for (;;) {
		nina_wait_mark(&wait);
		k_mutex_lock(&socket_lock, K_FOREVER);
		ret = nina_sock_sendmsg(id, msg->msg_iov, msg->msg_iovlen,
					msg->msg_name || msg->msg_controllen, flags, &retry);
		polled = retry && nina_sock_get(id)->send_polled;
		deadline = pass_deadline(id, false);
		k_mutex_unlock(&socket_lock);
		if (ret != -EAGAIN || !retry) {
			break;
		}
		send_wait(&wait, polled, deadline);
	}
	k_mutex_unlock(&sendmsg_lock);
	return ret < 0 ? fail((int)ret) : ret;
}

static ssize_t do_recv(void *obj, void *buf, size_t len, int flags, struct net_sockaddr *from,
		       net_socklen_t *fromlen)
{
	struct nina_wait wait = NINA_WAIT_INIT;
	int id = id_of(obj);
	struct net_sockaddr_in sender;
	struct nina_sock *s;
	int64_t deadline;
	bool nonblocking;
	ssize_t ret;

	/* TCP takes no address, UDP an address with its length or neither:
	 * EOPNOTSUPP. The core sees the rest (an address for TCP, the
	 * flags). */
	if (!!from != !!fromlen) {
		return fail(EOPNOTSUPP);
	}
	nonblocking = (flags & ZSOCK_MSG_DONTWAIT) || fds[id].nonblocking;
	deadline = fds[id].receive_timeout_ms > 0 ? k_uptime_get() + fds[id].receive_timeout_ms
						  : INT64_MAX;
	for (;;) {
		nina_wait_mark(&wait);
		k_mutex_lock(&socket_lock, K_FOREVER);
		s = nina_sock_get(id);
		if (s) {
			s->read_hint = false;
		}
		ret = nina_sock_recv(id, buf, len, flags, from ? &sender : NULL);
		k_mutex_unlock(&socket_lock);
		if (ret != -EAGAIN || nonblocking || k_uptime_get() >= deadline) {
			break;
		}
		nina_wait(&wait, NINA_POLL_INTERVAL_MS, deadline);
	}
	if (ret < 0) {
		return fail((int)ret);
	}
	if (from) {
		net_socklen_t capacity = *fromlen;

		memcpy(from, &sender, MIN(capacity, sizeof(sender)));
		*fromlen = sizeof(sender);
	}
	return ret;
}

static int do_close(void *obj)
{
	int ret;

	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = nina_sock_close(id_of(obj));
	fd_reset(id_of(obj));
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

/* What a NINA socket's poll reports when the core could not ask the module
 * (RET < 0). The call itself does not fail: a caller such as Zephyr's
 * socket service thread stops for good when poll() returns -1, and each
 * descriptor's answer should be its own, as in Zephyr's native poll().
 * - -EBADF: the socket was closed while the poll ran (another thread's
 *   close() between the descriptor's lookup and the core call; the
 *   descriptor is freed right after): ZSOCK_POLLNVAL, as for a closed
 *   descriptor.
 * - Anything else (the module's poll failed, or the SPI link: -EIO,
 *   -ETIMEDOUT, -EPROTO, ...): ZSOCK_POLLERR on that socket only. */
static short poll_failed_revents(int ret)
{
	return ret == -EBADF ? ZSOCK_POLLNVAL : ZSOCK_POLLERR;
}

static int do_poll(struct zsock_pollfd *fds_in, int nfds, int timeout)
{
	struct nina_wait wait = NINA_WAIT_INIT;
	int64_t deadline = timeout < 0 ? INT64_MAX : k_uptime_get() + timeout;

	for (;;) {
		bool other = false;
		int ready = 0;

		nina_wait_mark(&wait);

		for (int i = 0; i < nfds; i++) {
			void *obj;
			const struct fd_op_vtable *vtable;
			int ret;

			fds_in[i].revents = 0;
			if (fds_in[i].fd < 0) {
				continue;
			}
			obj = zvfs_get_fd_obj_and_vtable(fds_in[i].fd, &vtable, NULL);
			if (!obj) {
				fds_in[i].revents = ZSOCK_POLLNVAL;
			} else if (vtable != (const struct fd_op_vtable *)&socket_ops) {
				struct zsock_pollfd native = fds_in[i];

				other = true;
				ret = zsock_poll(&native, 1, 0);
				if (ret < 0) {
					return -1;
				}
				fds_in[i].revents = native.revents;
			} else {
				/* The core answers for both ZSOCK_POLLIN and
				 * ZSOCK_POLLOUT: keep what was asked for. A TCP
				 * socket the firmware does not have yet is hung up
				 * (ZSOCK_POLLHUP alone), such a UDP socket only
				 * writable. */
				k_mutex_lock(&socket_lock, K_FOREVER);
				ret = nina_sock_poll_revents(id_of(obj));
				k_mutex_unlock(&socket_lock);
				if (ret < 0) {
					fds_in[i].revents = poll_failed_revents(ret);
				} else {
					fds_in[i].revents = ret & (fds_in[i].events |
								   ZSOCK_POLLERR | ZSOCK_POLLHUP);
				}
			}
			if (fds_in[i].revents) {
				ready++;
			}
		}
		if (ready) {
			return ready;
		}
		if (k_uptime_get() >= deadline) {
			return 0;
		}
		nina_wait(&wait, NINA_POLL_INTERVAL_MS,
			  other ? MIN(deadline, k_uptime_get() + NINA_POLL_INTERVAL_MS) : deadline);
	}
}

/* Only the receive timeout takes effect, and it is kept here. Options are
 * not passed on to the module: its setsockopt command reaches SOL_SOCKET
 * options only, and the sockets there are always non-blocking. TCP_NODELAY
 * on a TCP socket is accepted without effect (the core). */
static int do_setsockopt(void *obj, int level, int optname, const void *optval,
			 net_socklen_t optlen)
{
	const struct zsock_timeval *tv = optval;
	int64_t ms;
	int ret;

	if (level == ZSOCK_SOL_SOCKET && optname == ZSOCK_SO_RCVTIMEO) {
		if (!optval || optlen != sizeof(*tv) || tv->tv_sec < 0 || tv->tv_usec < 0 ||
		    tv->tv_usec >= USEC_PER_SEC) {
			return fail(EINVAL);
		}
		ms = (int64_t)tv->tv_sec * MSEC_PER_SEC +
		     (tv->tv_usec + USEC_PER_MSEC - 1) / USEC_PER_MSEC;
		if (ms > INT32_MAX) {
			return fail(EDOM);
		}
		fds[id_of(obj)].receive_timeout_ms = (int32_t)ms;
		return 0;
	}
	/* The NuttX build's own options (nina_sockopt.h) are not Zephyr's. */
	if (level == NINA_SOL) {
		return fail(ENOPROTOOPT);
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	ret = nina_sock_setsockopt(id_of(obj), level, optname, optval, optlen);
	k_mutex_unlock(&socket_lock);
	return ret ? fail(ret) : 0;
}

static int do_ioctl(void *obj, unsigned int request, va_list args)
{
	struct nina_fd *s = obj;

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
		struct zsock_pollfd *fds_in = va_arg(args, struct zsock_pollfd *);
		int nfds = va_arg(args, int);
		int timeout = va_arg(args, int);
		return do_poll(fds_in, nfds, timeout);
	}
	default:
		return fail(ENOTSUP);
	}
}
static const struct socket_op_vtable socket_ops = {
	.fd_vtable = {.read = do_read, .write = do_write, .close = do_close, .ioctl = do_ioctl},
	.bind = do_bind,
	.connect = do_connect,
	.setsockopt = do_setsockopt,
	.listen = do_listen,
	.accept = do_accept,
	.sendto = do_send,
	.sendmsg = do_sendmsg,
	.recvfrom = do_recv,
};

int nina_socket_create(int family, int type, int proto)
{
	int fd, id;

	if (!nina_sock_supported(family, type, proto)) {
		return fail(EAFNOSUPPORT);
	}
	fd = zvfs_reserve_fd();
	if (fd < 0) {
		return -1;
	}
	k_mutex_lock(&socket_lock, K_FOREVER);
	id = nina_sock_create(family, type, proto);
	if (id >= 0) {
		fd_reset(id);
	}
	k_mutex_unlock(&socket_lock);
	if (id < 0) {
		zvfs_free_fd(fd);
		return fail(id);
	}
	zvfs_finalize_typed_fd(fd, &fds[id], (const struct fd_op_vtable *)&socket_ops,
			       ZVFS_MODE_IFSOCK);
	return fd;
}
NET_SOCKET_OFFLOAD_REGISTER(nina, CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY, NET_AF_INET,
			    nina_sock_supported, nina_socket_create);

/* The resolver's side of the core on Zephyr (nina_os.h, NINA_OS_DNS_OFFLOAD):
 * its results on their own heap, and its registration as the offload's
 * resolver (nina_dns_init(), from iface_init() of nina_net.c). */
K_HEAP_DEFINE(nina_dns_heap, 2048);

void *nina_os_dns_alloc(size_t size)
{
	return k_heap_alloc(&nina_dns_heap, size, K_NO_WAIT);
}

void nina_os_dns_free(void *ptr)
{
	k_heap_free(&nina_dns_heap, ptr);
}

static const struct socket_dns_offload dns_ops = {
	.getaddrinfo = nina_dns_getaddrinfo,
	.freeaddrinfo = nina_dns_freeaddrinfo,
};

void nina_os_dns_register(void)
{
	socket_offload_dns_register(&dns_ops);
}
