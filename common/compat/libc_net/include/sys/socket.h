/* SPDX-License-Identifier: MIT */
/* The types and constants of POSIX's <sys/socket.h> that the FreeRTOS build
 * needs, which newlib does not have.  There are no calls here: the build's
 * sockets are the NINA-W102's, reached through the zsock_*() calls of the
 * Zephyr compatibility layer (<zephyr/net/socket.h>,
 * common/drivers/nina/port/zephyr/nina_socket.c), as in the Zephyr build.
 *
 * The values are Linux's (glibc's), which are Zephyr's (ZSOCK_*, NET_*)
 * except for the address family: Zephyr's NET_AF_INET is 1, Linux's,
 * NuttX's and the BSDs' AF_INET is 2.  No value reaches the module or the
 * network (the NINA core compares them and sends its own codes,
 * common/drivers/nina/nina_sock.c), nor the key-value store, so only their
 * consistency within the build matters.  The NuttX build's values differ
 * for SO_RCVTIMEO (10), SO_BINDTODEVICE (17), SHUT_* (1..3); the shared
 * sources use the names only.
 * Part of the POSIX network headers for a C library that has none
 * (common/compat/libc_net), which the FreeRTOS build uses with newlib.
 */

#ifndef COMMON_COMPAT_LIBC_NET_INCLUDE_SYS_SOCKET_H
#define COMMON_COMPAT_LIBC_NET_INCLUDE_SYS_SOCKET_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/uio.h>

typedef uint16_t sa_family_t;
typedef uint32_t socklen_t;

#define AF_UNSPEC 0
#define AF_INET   2
#define AF_INET6  10
#define PF_UNSPEC AF_UNSPEC
#define PF_INET   AF_INET
#define PF_INET6  AF_INET6

#define SOCK_STREAM 1
#define SOCK_DGRAM  2

/* Socket options at level SOL_SOCKET.  Of these, the NINA's sockets take
 * SO_RCVTIMEO only (setsockopt(); getsockopt() is not supported, as in the
 * Zephyr build).
 */
#define SOL_SOCKET      1
#define SO_REUSEADDR    2
#define SO_ERROR        4
#define SO_RCVTIMEO     20
#define SO_SNDTIMEO     21
#define SO_BINDTODEVICE 25

/* The flags of the send and receive calls.  The NINA's sockets take
 * MSG_DONTWAIT only.
 */
#define MSG_PEEK     0x02
#define MSG_DONTWAIT 0x40
#define MSG_WAITALL  0x100

#define SHUT_RD   0
#define SHUT_WR   1
#define SHUT_RDWR 2

struct sockaddr {
	sa_family_t sa_family;
	char sa_data[14];
};

/* Big enough for every address of the build (struct sockaddr_in6). */
struct sockaddr_storage {
	sa_family_t ss_family;
	uint8_t ss_data[26];
} __attribute__((aligned(4)));

struct msghdr {
	void *msg_name;
	socklen_t msg_namelen;
	struct iovec *msg_iov;
	size_t msg_iovlen;
	void *msg_control;
	size_t msg_controllen;
	int msg_flags;
};

#endif /* COMMON_COMPAT_LIBC_NET_INCLUDE_SYS_SOCKET_H */
