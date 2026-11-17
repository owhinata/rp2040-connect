/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/net/socket.h> (Zephyr v4.5.0) that the
 * FreeRTOS build uses: the sockets of the NINA-W102 under the zsock_*()
 * names, their poll, the module's name resolution, and the address
 * conversions.  The calls are implemented on the shared NINA core by
 * freertos/drivers/nina/nina_socket.c (the Zephyr build's socket offload,
 * drivers/nina/nina_socket.c, with the Zephyr socket layer's parts it
 * relies on: the descriptor table, its per-descriptor lock, the dispatch).
 * Their results are Zephyr's: 0 or a count on success, -1 with errno on
 * failure (newlib's errno values, which are Zephyr's), except
 * zsock_getaddrinfo(), which returns 0 or an EAI_* (<netdb.h>, Zephyr's
 * DNS_EAI_* values).
 *
 * The types and constants are POSIX's of freertos/drivers/libc/include
 * (newlib has none of them) under Zephyr's ZSOCK_ names.  zsock_inet_pton()
 * and zsock_inet_ntop() are the C library's inet_pton() and inet_ntop()
 * (freertos/drivers/libc/inet.c, AF_INET only), as in Zephyr.  Zephyr 4.5's
 * net_* names (struct net_sockaddr, NET_AF_INET, net_htons(), ...) are not
 * here: no shared source uses them.
 *
 * Only the zsock_*() names: there is no socket(), poll(), close() or
 * fcntl() on these descriptors, which are not newlib's (its 0, 1 and 2 are
 * the console's, freertos/drivers/shell); see nina_socket.c for the table.
 *
 * Part of the FreeRTOS implementation of the small Zephyr compatibility
 * layer of the port (freertos/drivers/compat).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H

#include <stdarg.h>
#include <stddef.h>
#include <sys/time.h>
#include <sys/types.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/uio.h>

#define ZSOCK_POLLIN   POLLIN
#define ZSOCK_POLLPRI  POLLPRI
#define ZSOCK_POLLOUT  POLLOUT
#define ZSOCK_POLLERR  POLLERR
#define ZSOCK_POLLHUP  POLLHUP
#define ZSOCK_POLLNVAL POLLNVAL

#define ZSOCK_MSG_PEEK     MSG_PEEK
#define ZSOCK_MSG_DONTWAIT MSG_DONTWAIT
#define ZSOCK_MSG_WAITALL  MSG_WAITALL

#define ZSOCK_SHUT_RD   SHUT_RD
#define ZSOCK_SHUT_WR   SHUT_WR
#define ZSOCK_SHUT_RDWR SHUT_RDWR

#define ZSOCK_SOL_SOCKET      SOL_SOCKET
#define ZSOCK_SO_REUSEADDR    SO_REUSEADDR
#define ZSOCK_SO_ERROR        SO_ERROR
#define ZSOCK_SO_RCVTIMEO     SO_RCVTIMEO
#define ZSOCK_SO_SNDTIMEO     SO_SNDTIMEO
#define ZSOCK_SO_BINDTODEVICE SO_BINDTODEVICE
#define ZSOCK_TCP_NODELAY     TCP_NODELAY

#define ZSOCK_AI_PASSIVE     AI_PASSIVE
#define ZSOCK_AI_CANONNAME   AI_CANONNAME
#define ZSOCK_AI_NUMERICHOST AI_NUMERICHOST
#define ZSOCK_AI_ADDRCONFIG  AI_ADDRCONFIG
#define ZSOCK_AI_NUMERICSERV AI_NUMERICSERV

#define zsock_pollfd   pollfd
#define zsock_addrinfo addrinfo
#define zsock_timeval  timeval

int zsock_socket(int family, int type, int proto);
int zsock_close(int sock);
int zsock_shutdown(int sock, int how);
int zsock_bind(int sock, const struct sockaddr *addr, socklen_t addrlen);
int zsock_connect(int sock, const struct sockaddr *addr, socklen_t addrlen);
int zsock_listen(int sock, int backlog);
int zsock_accept(int sock, struct sockaddr *addr, socklen_t *addrlen);
ssize_t zsock_sendto(int sock, const void *buf, size_t len, int flags,
		     const struct sockaddr *dest_addr, socklen_t addrlen);
ssize_t zsock_sendmsg(int sock, const struct msghdr *msg, int flags);
ssize_t zsock_recvfrom(int sock, void *buf, size_t max_len, int flags, struct sockaddr *src_addr,
		       socklen_t *addrlen);
int zsock_poll(struct zsock_pollfd *fds, int nfds, int timeout);
int zsock_getsockopt(int sock, int level, int optname, void *optval, socklen_t *optlen);
int zsock_setsockopt(int sock, int level, int optname, const void *optval, socklen_t optlen);
int zsock_getsockname(int sock, struct sockaddr *addr, socklen_t *addrlen);
int zsock_fcntl_impl(int sock, int cmd, int flags);
int zsock_getaddrinfo(const char *host, const char *service, const struct zsock_addrinfo *hints,
		      struct zsock_addrinfo **res);
void zsock_freeaddrinfo(struct zsock_addrinfo *ai);

static inline ssize_t zsock_send(int sock, const void *buf, size_t len, int flags)
{
	return zsock_sendto(sock, buf, len, flags, NULL, 0);
}

static inline ssize_t zsock_recv(int sock, void *buf, size_t max_len, int flags)
{
	return zsock_recvfrom(sock, buf, max_len, flags, NULL, NULL);
}

/* Zephyr's zsock_fcntl(): F_GETFL, F_SETFL with O_NONBLOCK (newlib's
 * <fcntl.h>: 3, 4 and 0x4000, Zephyr's ZVFS_* values).
 */
static inline int zsock_fcntl_wrapper(int sock, int cmd, ...)
{
	va_list args;
	int flags;

	va_start(args, cmd);
	flags = va_arg(args, int);
	va_end(args);
	return zsock_fcntl_impl(sock, cmd, flags);
}

#define zsock_fcntl zsock_fcntl_wrapper

#define zsock_inet_pton(family, src, dst)       inet_pton(family, src, dst)
#define zsock_inet_ntop(family, src, dst, size) inet_ntop(family, src, dst, size)

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H */
