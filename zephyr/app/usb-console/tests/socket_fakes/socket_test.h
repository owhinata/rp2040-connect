/* SPDX-License-Identifier: MIT */
/* Fakes of the Zephyr calls of drivers/nina/nina_socket.c for the host test
 * tests/nina_socket.c, which defines the functions: the kernel's mutexes,
 * sleep, clock and heap, the descriptor table (zvfs) and the socket
 * offload's registration.  Zephyr's socket names map to the host's (the
 * shared core is built on the host's <sys/socket.h>, as the common tests
 * do); Zephyr's own values do not matter here.
 */
#ifndef SOCKET_TEST_H_
#define SOCKET_TEST_H_

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>

/* <zephyr/kernel.h> */
struct k_mutex {
	int held;
};
struct k_heap {
	int unused;
};
typedef struct {
	int64_t ms;
} k_timeout_t;
#define K_MUTEX_DEFINE(name)                struct k_mutex name
#define K_HEAP_DEFINE(name, size)           struct k_heap name
#define K_FOREVER                           ((k_timeout_t){-1})
#define K_NO_WAIT                           ((k_timeout_t){0})
#define K_MSEC(n)                           ((k_timeout_t){(n)})
#define MIN(a, b)                           ((a) < (b) ? (a) : (b))
#define USEC_PER_SEC                        1000000
#define MSEC_PER_SEC                        1000
#define USEC_PER_MSEC                       1000
#define CONFIG_NINA_SEND_RETRY_MS           1
#define CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY 40
int k_mutex_lock(struct k_mutex *lock, k_timeout_t timeout);
int k_mutex_unlock(struct k_mutex *lock);
int32_t k_sleep(k_timeout_t timeout);
int64_t k_uptime_get(void);
void *k_heap_alloc(struct k_heap *heap, size_t bytes, k_timeout_t timeout);
void k_heap_free(struct k_heap *heap, void *mem);

/* <zephyr/net/socket.h> */
#define net_sockaddr       sockaddr
#define net_sockaddr_in    sockaddr_in
#define net_socklen_t      socklen_t
#define net_msghdr         msghdr
#define zsock_pollfd       pollfd
#define zsock_timeval      timeval
#define zsock_addrinfo     addrinfo
#define NET_AF_INET        AF_INET
#define ZSOCK_MSG_DONTWAIT MSG_DONTWAIT
#define ZSOCK_POLLIN       POLLIN
#define ZSOCK_POLLOUT      POLLOUT
#define ZSOCK_POLLERR      POLLERR
#define ZSOCK_POLLHUP      POLLHUP
#define ZSOCK_POLLNVAL     POLLNVAL
#define ZSOCK_SOL_SOCKET   SOL_SOCKET
#define ZSOCK_SO_RCVTIMEO  SO_RCVTIMEO
int zsock_poll(struct zsock_pollfd *fds, int nfds, int timeout);

/* <zephyr/sys/fdtable.h> */
struct fd_op_vtable {
	ssize_t (*read)(void *obj, void *buf, size_t sz);
	ssize_t (*write)(void *obj, const void *buf, size_t sz);
	int (*close)(void *obj);
	int (*ioctl)(void *obj, unsigned int request, va_list args);
};
struct socket_op_vtable {
	struct fd_op_vtable fd_vtable;
	int (*bind)(void *obj, const struct net_sockaddr *addr, net_socklen_t addrlen);
	int (*connect)(void *obj, const struct net_sockaddr *addr, net_socklen_t addrlen);
	int (*setsockopt)(void *obj, int level, int optname, const void *optval,
			  net_socklen_t optlen);
	int (*listen)(void *obj, int backlog);
	int (*accept)(void *obj, struct net_sockaddr *addr, net_socklen_t *addrlen);
	ssize_t (*sendto)(void *obj, const void *buf, size_t len, int flags,
			  const struct net_sockaddr *dest_addr, net_socklen_t addrlen);
	ssize_t (*sendmsg)(void *obj, const struct net_msghdr *msg, int flags);
	ssize_t (*recvfrom)(void *obj, void *buf, size_t max_len, int flags,
			    struct net_sockaddr *src_addr, net_socklen_t *addrlen);
};
#define ZVFS_MODE_IFSOCK       0x1000
#define ZVFS_F_GETFL           3
#define ZVFS_F_SETFL           4
#define ZVFS_O_NONBLOCK        0x4000
#define ZFD_IOCTL_POLL_PREPARE 0x101
#define ZFD_IOCTL_POLL_OFFLOAD 0x103
int zvfs_reserve_fd(void);
void zvfs_free_fd(int fd);
void zvfs_finalize_typed_fd(int fd, void *obj, const struct fd_op_vtable *vtable, uint32_t mode);
void *zvfs_get_fd_obj_and_vtable(int fd, const struct fd_op_vtable **vtable, struct k_mutex **lock);

/* <zephyr/net/socket_offload.h> */
struct socket_dns_offload {
	int (*getaddrinfo)(const char *node, const char *service,
			   const struct zsock_addrinfo *hints, struct zsock_addrinfo **res);
	void (*freeaddrinfo)(struct zsock_addrinfo *res);
};
void socket_offload_dns_register(const struct socket_dns_offload *ops);
struct fake_offload {
	bool (*supported)(int family, int type, int proto);
	int (*create)(int family, int type, int proto);
};
#define NET_SOCKET_OFFLOAD_REGISTER(name, prio, fam, supp, create)                                 \
	const struct fake_offload fake_offload_##name = {(supp), (create)}

#endif
