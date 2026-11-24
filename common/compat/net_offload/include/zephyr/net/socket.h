/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/net/socket.h> (Zephyr v4.5.0) that the
 * FreeRTOS build uses: Zephyr's socket layer down to its list of socket
 * implementations, without a network stack of its own.
 *
 * - The zsock_*() calls, Zephyr's (subsys/net/lib/sockets/sockets.c here,
 *   its z_impl_zsock_*() functions under the zsock_*() names: there is no
 *   user mode, whose syscall wrapper is the only difference): zsock_socket()
 *   takes the first registered implementation, in priority order, whose
 *   family matches and whose is_supported() takes the socket, and the
 *   implementation's handler makes the descriptor; every other call looks
 *   the descriptor up in the descriptor table (<zephyr/sys/fdtable.h>,
 *   lib/os/zvfs here) and calls the implementation's struct
 *   socket_op_vtable under the descriptor's mutex (EBADF for a descriptor
 *   that is not open, EOPNOTSUPP for an operation the implementation does
 *   not have, ENOTSUP for shutdown(); fcntl() goes to its ioctl method);
 *   zsock_close() is zvfs_close(), zsock_poll() zvfs_poll() (an offloaded
 *   socket takes the whole call: ZFD_IOCTL_POLL_PREPARE answers -EXDEV,
 *   then ZFD_IOCTL_POLL_OFFLOAD).  The results are Zephyr's: 0 or a count,
 *   or -1 with errno (newlib's errno values, which are Zephyr's).
 * - The registration: NET_SOCKET_REGISTER() and NET_SOCKET_OFFLOAD_REGISTER()
 *   put a struct net_socket_register into the linker section
 *   ._net_socket_register.static.__net_socket_register_<prio>_<name>, as
 *   Zephyr's STRUCT_SECTION_ITERABLE() does; the linker script collects them
 *   sorted by name, that is by priority (the same number of digits), between
 *   _net_socket_register_list_start and _net_socket_register_list_end
 *   (common/compat/port/pico_sdk/ld/section_extra_post_text.incl on the board,
 *   freertos/tests/sockets/net_socket_register.ld in the host tests).
 * - Name resolution: zsock_getaddrinfo() and zsock_freeaddrinfo() go to the
 *   registered resolver (<zephyr/net/socket_offload.h>,
 *   socket_offload_dns_register()); without one the result is EAI_FAIL
 *   (no resolver of the layer's own).
 *
 * The NINA-W102's sockets register themselves in the Zephyr binding of the
 * shared driver, common/drivers/nina/port/zephyr/nina_socket.c, which the
 * FreeRTOS build compiles as the Zephyr build does.
 *
 * The types and constants are POSIX's of common/compat/libc_net/include
 * (newlib has none of them) under Zephyr's ZSOCK_ names, and the net_* names
 * of <zephyr/net/net_ip.h> here.  zsock_inet_pton() and zsock_inet_ntop()
 * are the C library's inet_pton() and inet_ntop()
 * (common/compat/libc_net/inet.c, AF_INET only), as in Zephyr.
 *
 * Not here: the dispatcher of Zephyr's offloaded interfaces
 * (SO_BINDTODEVICE; net_if_socket_offload_set() does nothing), Zephyr's
 * native IP stack and its sockets, TLS, socketpair(), zsock_ioctl(), the
 * object core statistics, the offloaded port tracking.  Only the zsock_*()
 * names: there is no socket(), poll(), close() or fcntl() on these
 * descriptors, which are not newlib's (its 0, 1 and 2 are the console's,
 * common/compat/include/shell_console.h).
 *
 * Part of the network part of the Zephyr compatibility layer for an OS
 * without a network stack of its own (common/compat/net_offload), which the
 * FreeRTOS build uses.
 */

#ifndef COMMON_COMPAT_NET_OFFLOAD_INCLUDE_ZEPHYR_NET_SOCKET_H
#define COMMON_COMPAT_NET_OFFLOAD_INCLUDE_ZEPHYR_NET_SOCKET_H

#include <stdarg.h>
#include <stdbool.h>
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

#include <zephyr/net/net_ip.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/sys/util.h>

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

/* <zephyr/net/socket_poll.h>, <zephyr/net/socket_types.h> */
#define zsock_pollfd   zvfs_pollfd
#define zsock_addrinfo addrinfo
#define zsock_timeval  timeval

/* The Zephyr build's CONFIG_NET_SOCKETS_PRIORITY_DEFAULT (50), the priority
 * of NET_SOCKET_DEFAULT_PRIO.  CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY (40), with
 * which an offloaded implementation registers, is in the prelude of the
 * shared sources (common/compat/port/generic/common_prelude.h), as Zephyr's
 * Kconfig values are in every source.
 */
#ifndef CONFIG_NET_SOCKETS_PRIORITY_DEFAULT
#define CONFIG_NET_SOCKETS_PRIORITY_DEFAULT 50
#endif

int zsock_socket(int family, int type, int proto);
int zsock_close(int sock);
int zsock_shutdown(int sock, int how);
int zsock_bind(int sock, const struct net_sockaddr *addr, net_socklen_t addrlen);
int zsock_connect(int sock, const struct net_sockaddr *addr, net_socklen_t addrlen);
int zsock_listen(int sock, int backlog);
int zsock_accept(int sock, struct net_sockaddr *addr, net_socklen_t *addrlen);
ssize_t zsock_sendto(int sock, const void *buf, size_t len, int flags,
		     const struct net_sockaddr *dest_addr, net_socklen_t addrlen);
ssize_t zsock_sendmsg(int sock, const struct net_msghdr *msg, int flags);
ssize_t zsock_recvfrom(int sock, void *buf, size_t max_len, int flags,
		       struct net_sockaddr *src_addr, net_socklen_t *addrlen);
ssize_t zsock_recvmsg(int sock, struct net_msghdr *msg, int flags);
int zsock_poll(struct zsock_pollfd *fds, int nfds, int timeout);
int zsock_getsockopt(int sock, int level, int optname, void *optval, net_socklen_t *optlen);
int zsock_setsockopt(int sock, int level, int optname, const void *optval, net_socklen_t optlen);
int zsock_getpeername(int sock, struct net_sockaddr *addr, net_socklen_t *addrlen);
int zsock_getsockname(int sock, struct net_sockaddr *addr, net_socklen_t *addrlen);
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

/* Zephyr's zsock_fcntl(): the command and one int, to the implementation's
 * ioctl method (zsock_fcntl_impl()).
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

/* The callers pass the C library's F_GETFL, F_SETFL and O_NONBLOCK, the
 * implementations see Zephyr's ZVFS_* values: the same numbers with newlib
 * (the board).  A host test passes the ZVFS_* values (the host's O_NONBLOCK
 * is another number).
 */
#ifdef __NEWLIB__
BUILD_ASSERT(F_GETFL == ZVFS_F_GETFL && F_SETFL == ZVFS_F_SETFL && O_NONBLOCK == ZVFS_O_NONBLOCK,
	     "newlib's fcntl values are Zephyr's");
#endif

#define zsock_inet_pton(family, src, dst)       inet_pton(family, src, dst)
#define zsock_inet_ntop(family, src, dst, size) inet_ntop(family, src, dst, size)

/** @cond INTERNAL_HIDDEN */
/**
 * @brief Registration information for a given BSD socket family.
 */
struct net_socket_register {
	int family;
	bool is_offloaded;
	bool (*is_supported)(int family, int type, int proto);
	int (*handler)(int family, int type, int proto);
};

#define NET_SOCKET_DEFAULT_PRIO CONFIG_NET_SOCKETS_PRIORITY_DEFAULT

#define NET_SOCKET_GET_NAME(socket_name, prio) __net_socket_register_##prio##_##socket_name

/* STRUCT_SECTION_ITERABLE(net_socket_register, name) of Zephyr's
 * <zephyr/sys/iterable_sections.h>: the entry in its own input section, kept
 * and sorted by the linker script (the layer's <zephyr/sys/iterable_sections.h>
 * has no sections of its own).
 */
#define Z_NET_SOCKET_REGISTER_ITERABLE(name)                                                       \
	struct net_socket_register name __aligned(__alignof(struct net_socket_register))           \
	__attribute__((section("._net_socket_register.static." STRINGIFY(name)), used))

#define _NET_SOCKET_REGISTER(socket_name, prio, _family, _is_supported, _handler, _is_offloaded)   \
	static const Z_NET_SOCKET_REGISTER_ITERABLE(NET_SOCKET_GET_NAME(socket_name, prio)) = {    \
		.family = _family,                                                                 \
		.is_offloaded = _is_offloaded,                                                     \
		.is_supported = _is_supported,                                                     \
		.handler = _handler,                                                               \
	}

#define NET_SOCKET_REGISTER(socket_name, prio, _family, _is_supported, _handler)                   \
	_NET_SOCKET_REGISTER(socket_name, prio, _family, _is_supported, _handler, false)

#define NET_SOCKET_OFFLOAD_REGISTER(socket_name, prio, _family, _is_supported, _handler)           \
	_NET_SOCKET_REGISTER(socket_name, prio, _family, _is_supported, _handler, true)

struct socket_op_vtable {
	struct fd_op_vtable fd_vtable;
	int (*shutdown)(void *obj, int how);
	int (*bind)(void *obj, const struct net_sockaddr *addr, net_socklen_t addrlen);
	int (*connect)(void *obj, const struct net_sockaddr *addr, net_socklen_t addrlen);
	int (*listen)(void *obj, int backlog);
	int (*accept)(void *obj, struct net_sockaddr *addr, net_socklen_t *addrlen);
	ssize_t (*sendto)(void *obj, const void *buf, size_t len, int flags,
			  const struct net_sockaddr *dest_addr, net_socklen_t addrlen);
	ssize_t (*recvfrom)(void *obj, void *buf, size_t max_len, int flags,
			    struct net_sockaddr *src_addr, net_socklen_t *addrlen);
	int (*getsockopt)(void *obj, int level, int optname, void *optval, net_socklen_t *optlen);
	int (*setsockopt)(void *obj, int level, int optname, const void *optval,
			  net_socklen_t optlen);
	ssize_t (*sendmsg)(void *obj, const struct net_msghdr *msg, int flags);
	ssize_t (*recvmsg)(void *obj, struct net_msghdr *msg, int flags);
	int (*getpeername)(void *obj, struct net_sockaddr *addr, net_socklen_t *addrlen);
	int (*getsockname)(void *obj, struct net_sockaddr *addr, net_socklen_t *addrlen);
};
/** @endcond */

#endif /* COMMON_COMPAT_NET_OFFLOAD_INCLUDE_ZEPHYR_NET_SOCKET_H */
