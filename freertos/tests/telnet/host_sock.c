/* SPDX-License-Identifier: MIT */
/* The zsock_*() calls of the layer's <zephyr/net/socket.h> on the host's
 * sockets, for the host tests of the Telnet shell and the iperf3 of "zperf"
 * (freertos/tests/telnet) and of the MQTT transport against the host's
 * receiver (freertos/tests/mqtt/live.c), with what the NINA's sockets do
 * (freertos/drivers/nina/nina_socket.c over common/drivers/nina/nina_sock.c)
 * where the code under test depends on it:
 *
 * - ten descriptors, numbered 0..9 from the lowest free one (EMFILE for an
 *   eleventh): a closed descriptor is soon another socket's, as on the
 *   board;
 * - accept never waits (EAGAIN); bind to an address other than any is
 *   EADDRNOTAVAIL; SO_REUSEADDR is ENOPROTOOPT, TCP_NODELAY is taken,
 *   SO_RCVTIMEO is kept; getsockopt and getsockname are EOPNOTSUPP;
 * - a TCP send takes at most 3584 bytes, a read gives at most 512; sendmsg
 *   joins the vectors (up to 3584 bytes) into one send (no address, no
 *   control data);
 * - the waits poll every 20 ms with the kernel's sleep (a blocking send
 *   every 1 ms up to 10 s, then EAGAIN; connect up to 10 s); poll reports
 *   POLLERR for a connection that failed (fake_sock_fail_connections()),
 *   on which a read gives 0 and a send EPIPE.
 *
 * The host calls themselves never block: the tasks of the FreeRTOS POSIX
 * port are threads of which one runs at a time.  They are made with the
 * scheduler suspended (no switch in the middle) and tried again on EINTR
 * (the port's tick is a signal).  The host's sockets are non-blocking,
 * SO_REUSEADDR for the listening ones (so that the test can run again at
 * once), and the accepted ones have a small send buffer, so that a client
 * that does not read fills it soon; so do the connected ones when the test
 * asks for it (host_sock_connect_sndbuf, host_sock.h).
 */

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/net/socket.h>

#include "host_sock.h"

#define SOCK_MAX         10
#define MAX_IO           512
#define TCP_MAX_IO       3584
#define POLL_INTERVAL_MS 20
#define SEND_RETRY_MS    1
#define SEND_WAIT_MS     10000
#define CONNECT_WAIT_MS  10000
#define ACCEPTED_SNDBUF  4096

struct hsock {
	bool used;
	bool listening;
	bool failed;
	bool nonblocking;
	int fd;
	int32_t rcvtimeo_ms;
};

static struct hsock socks[SOCK_MAX];

int host_sock_connect_sndbuf;

static int fail(int err)
{
	errno = err;
	return -1;
}

static int64_t now_ms(void)
{
	return (int64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void sleep_ms(uint32_t ms)
{
	vTaskDelay(pdMS_TO_TICKS(ms) + 1);
}

/* The host's errno of a call made with the scheduler suspended. */
#define HOST_CALL(expr)                                                                            \
	({                                                                                         \
		long _r;                                                                           \
		int _e;                                                                            \
		do {                                                                               \
			vTaskSuspendAll();                                                         \
			_r = (long)(expr);                                                         \
			_e = errno;                                                                \
			(void)xTaskResumeAll();                                                    \
		} while (_r < 0 && _e == EINTR);                                                   \
		errno = _e;                                                                        \
		_r;                                                                                \
	})

static struct hsock *get(int sock)
{
	if (sock < 0 || sock >= SOCK_MAX || !socks[sock].used) {
		return NULL;
	}
	return &socks[sock];
}

static int alloc(int fd)
{
	for (int i = 0; i < SOCK_MAX; i++) {
		if (!socks[i].used) {
			socks[i] = (struct hsock){.used = true, .fd = fd};
			return i;
		}
	}
	return -1;
}

int zsock_socket(int family, int type, int proto)
{
	int fd;
	int sock;

	(void)proto;
	if (family != AF_INET || (type != SOCK_STREAM && type != SOCK_DGRAM)) {
		return fail(EAFNOSUPPORT);
	}
	fd = (int)HOST_CALL(socket(AF_INET, type | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
	if (fd < 0) {
		return -1;
	}
	sock = alloc(fd);
	if (sock < 0) {
		close(fd);
		return fail(EMFILE);
	}
	return sock;
}

int zsock_close(int sock)
{
	struct hsock *s = get(sock);

	if (s == NULL) {
		return fail(EBADF);
	}
	(void)HOST_CALL(close(s->fd));
	s->used = false;
	return 0;
}

int zsock_shutdown(int sock, int how)
{
	(void)how;
	return fail(get(sock) ? ENOTSUP : EBADF);
}

int zsock_bind(int sock, const struct sockaddr *addr, socklen_t addrlen)
{
	struct hsock *s = get(sock);
	const struct sockaddr_in *in = (const struct sockaddr_in *)addr;
	int on = 1;

	if (s == NULL) {
		return fail(EBADF);
	}
	if (addrlen < sizeof(*in) || in->sin_family != AF_INET) {
		return fail(EINVAL);
	}
	if (in->sin_addr.s_addr != htonl(INADDR_ANY)) {
		return fail(EADDRNOTAVAIL);
	}
	(void)setsockopt(s->fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
	return HOST_CALL(bind(s->fd, addr, addrlen)) < 0 ? -1 : 0;
}

int zsock_listen(int sock, int backlog)
{
	struct hsock *s = get(sock);

	if (s == NULL) {
		return fail(EBADF);
	}
	if (HOST_CALL(listen(s->fd, backlog)) < 0) {
		return -1;
	}
	s->listening = true;
	return 0;
}

int zsock_accept(int sock, struct sockaddr *addr, socklen_t *addrlen)
{
	struct hsock *s = get(sock);
	int sndbuf = ACCEPTED_SNDBUF;
	int fd;
	int n;

	if (s == NULL) {
		return fail(EBADF);
	}
	fd = (int)HOST_CALL(accept4(s->fd, addr, addrlen, SOCK_NONBLOCK | SOCK_CLOEXEC));
	if (fd < 0) {
		return fail(errno == EWOULDBLOCK ? EAGAIN : errno);
	}
	(void)setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
	n = alloc(fd);
	if (n < 0) {
		close(fd);
		return fail(EMFILE);
	}
	return n;
}

int zsock_connect(int sock, const struct sockaddr *addr, socklen_t addrlen)
{
	struct hsock *s = get(sock);
	int64_t deadline = now_ms() + CONNECT_WAIT_MS;

	if (s == NULL) {
		return fail(EBADF);
	}
	if (host_sock_connect_sndbuf > 0) {
		(void)setsockopt(s->fd, SOL_SOCKET, SO_SNDBUF, &host_sock_connect_sndbuf,
				 sizeof(host_sock_connect_sndbuf));
	}
	if (HOST_CALL(connect(s->fd, addr, addrlen)) == 0) {
		return 0;
	}
	if (errno != EINPROGRESS) {
		return -1;
	}
	for (;;) {
		struct pollfd p = {.fd = s->fd, .events = POLLOUT};

		if (HOST_CALL(poll(&p, 1, 0)) > 0) {
			int err = 0;
			socklen_t len = sizeof(err);

			getsockopt(s->fd, SOL_SOCKET, SO_ERROR, &err, &len);
			return err ? fail(err == ECONNREFUSED ? ECONNREFUSED : err) : 0;
		}
		if (now_ms() >= deadline) {
			return fail(ETIMEDOUT);
		}
		sleep_ms(POLL_INTERVAL_MS);
	}
}

ssize_t zsock_sendto(int sock, const void *buf, size_t len, int flags,
		     const struct sockaddr *dest_addr, socklen_t addrlen)
{
	struct hsock *s = get(sock);
	int64_t deadline = now_ms() + SEND_WAIT_MS;
	bool nonblocking;

	(void)dest_addr;
	(void)addrlen;
	if (s == NULL) {
		return fail(EBADF);
	}
	nonblocking = (flags & MSG_DONTWAIT) || s->nonblocking;
	len = len > TCP_MAX_IO ? TCP_MAX_IO : len;
	for (;;) {
		ssize_t ret;

		if (s->failed) {
			return fail(EPIPE);
		}
		ret = HOST_CALL(send(s->fd, buf, len, MSG_DONTWAIT | MSG_NOSIGNAL));
		if (ret >= 0) {
			return ret;
		}
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			return -1;
		}
		if (nonblocking || now_ms() >= deadline) {
			return fail(EAGAIN);
		}
		sleep_ms(SEND_RETRY_MS);
	}
}

ssize_t zsock_sendmsg(int sock, const struct msghdr *msg, int flags)
{
	static uint8_t joined[TCP_MAX_IO];
	size_t len = 0;

	if (get(sock) == NULL) {
		return fail(EBADF);
	}
	if (msg == NULL) {
		return fail(EINVAL);
	}
	if (msg->msg_name != NULL || msg->msg_controllen) {
		return fail(ENOTSUP);
	}
	for (size_t i = 0; i < (size_t)msg->msg_iovlen && len < sizeof(joined); i++) {
		size_t n = msg->msg_iov[i].iov_len;

		n = n > sizeof(joined) - len ? sizeof(joined) - len : n;
		memcpy(joined + len, msg->msg_iov[i].iov_base, n);
		len += n;
	}
	return zsock_sendto(sock, joined, len, flags, NULL, 0);
}

ssize_t zsock_recvfrom(int sock, void *buf, size_t max_len, int flags, struct sockaddr *src_addr,
		       socklen_t *addrlen)
{
	struct hsock *s = get(sock);
	int64_t deadline;
	bool nonblocking;

	(void)src_addr;
	(void)addrlen;
	if (s == NULL) {
		return fail(EBADF);
	}
	if (flags & ~MSG_DONTWAIT) {
		return fail(EOPNOTSUPP);
	}
	nonblocking = (flags & MSG_DONTWAIT) || s->nonblocking;
	deadline = s->rcvtimeo_ms > 0 ? now_ms() + s->rcvtimeo_ms : INT64_MAX;
	max_len = max_len > MAX_IO ? MAX_IO : max_len;
	for (;;) {
		ssize_t ret;

		if (s->failed) {
			return 0;
		}
		ret = HOST_CALL(recv(s->fd, buf, max_len, MSG_DONTWAIT));
		if (ret >= 0) {
			return ret;
		}
		if (errno == ECONNRESET) {
			return 0; /* the NINA reports a failed connection as the end */
		}
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			return -1;
		}
		if (nonblocking || now_ms() >= deadline) {
			return fail(EAGAIN);
		}
		sleep_ms(POLL_INTERVAL_MS);
	}
}

int zsock_poll(struct zsock_pollfd *fds, int nfds, int timeout)
{
	int64_t deadline = timeout < 0 ? INT64_MAX : now_ms() + timeout;

	for (;;) {
		int ready = 0;

		for (int i = 0; i < nfds; i++) {
			struct hsock *s;
			struct pollfd p;

			fds[i].revents = 0;
			if (fds[i].fd < 0) {
				continue;
			}
			s = get(fds[i].fd);
			if (s == NULL) {
				fds[i].revents = POLLNVAL;
			} else if (s->failed) {
				fds[i].revents = POLLERR;
			} else {
				p = (struct pollfd){.fd = s->fd, .events = POLLIN | POLLOUT};
				if (HOST_CALL(poll(&p, 1, 0)) > 0) {
					short ev = 0;

					/* The NINA: readable also at the end of the
					 * stream; no POLLHUP for a connected socket.
					 */
					if (p.revents & (POLLIN | POLLHUP | POLLERR)) {
						ev |= POLLIN;
					}
					if ((p.revents & POLLOUT) && !s->listening) {
						ev |= POLLOUT;
					}
					fds[i].revents = ev & fds[i].events;
				}
			}
			if (fds[i].revents) {
				ready++;
			}
		}
		if (ready) {
			return ready;
		}
		if (now_ms() >= deadline) {
			return 0;
		}
		sleep_ms(POLL_INTERVAL_MS);
	}
}

int zsock_getsockopt(int sock, int level, int optname, void *optval, socklen_t *optlen)
{
	(void)level;
	(void)optname;
	(void)optval;
	(void)optlen;
	return fail(get(sock) ? EOPNOTSUPP : EBADF);
}

int zsock_setsockopt(int sock, int level, int optname, const void *optval, socklen_t optlen)
{
	struct hsock *s = get(sock);

	if (s == NULL) {
		return fail(EBADF);
	}
	if (level == SOL_SOCKET && optname == SO_RCVTIMEO) {
		const struct timeval *tv = optval;

		if (optval == NULL || optlen != sizeof(*tv)) {
			return fail(EINVAL);
		}
		s->rcvtimeo_ms = (int32_t)(tv->tv_sec * 1000 + (tv->tv_usec + 999) / 1000);
		return 0;
	}
	if (level == IPPROTO_TCP && optname == TCP_NODELAY) {
		return 0;
	}
	return fail(ENOPROTOOPT);
}

int zsock_getsockname(int sock, struct sockaddr *addr, socklen_t *addrlen)
{
	(void)addr;
	(void)addrlen;
	return fail(get(sock) ? EOPNOTSUPP : EBADF);
}

int zsock_fcntl_impl(int sock, int cmd, int flags)
{
	struct hsock *s = get(sock);

	if (s == NULL) {
		return fail(EBADF);
	}
	if (cmd == F_GETFL) {
		return s->nonblocking ? O_NONBLOCK : 0;
	}
	if (cmd == F_SETFL) {
		if (flags & ~O_NONBLOCK) {
			return fail(EOPNOTSUPP);
		}
		s->nonblocking = (flags & O_NONBLOCK) != 0;
		return 0;
	}
	return fail(ENOTSUP);
}

int zsock_getaddrinfo(const char *host, const char *service, const struct zsock_addrinfo *hints,
		      struct zsock_addrinfo **res)
{
	(void)host;
	(void)service;
	(void)hints;
	(void)res;
	return EAI_FAIL;
}

void zsock_freeaddrinfo(struct zsock_addrinfo *ai)
{
	(void)ai;
}

int fake_sock_open_count(void)
{
	int n = 0;

	for (int i = 0; i < SOCK_MAX; i++) {
		n += socks[i].used ? 1 : 0;
	}
	return n;
}

void fake_sock_fail_connections(void)
{
	for (int i = 0; i < SOCK_MAX; i++) {
		if (socks[i].used && !socks[i].listening && !socks[i].failed) {
			socks[i].failed = true;
			/* The peer sees the connection go, as after a Wi-Fi drop
			 * (the NINA closes it).
			 */
			(void)HOST_CALL(shutdown(socks[i].fd, SHUT_RDWR));
		}
	}
}
