/* SPDX-License-Identifier: MIT */
/* The types and constants of POSIX's <netdb.h> for getaddrinfo(), which
 * newlib does not have.  There is no getaddrinfo() here: names are resolved
 * by the NINA-W102 (common/drivers/nina/nina_dns.c, nina_dns_getaddrinfo()),
 * which zsock_getaddrinfo() of the Zephyr compatibility layer calls
 * (<zephyr/net/socket.h>) as the offloaded resolver that the NINA's binding
 * registers (common/drivers/nina/port/zephyr/nina_socket.c), as in the Zephyr build.
 *
 * The values are glibc's, which are Zephyr's: the AI_* flags of
 * ZSOCK_AI_*, and the errors of DNS_EAI_* (<zephyr/net/dns_resolve.h>),
 * negative, so that zsock_getaddrinfo() returns the Zephyr build's numbers.
 * NuttX's EAI_* are positive and its AI_NUMERICSERV and AI_ADDRCONFIG
 * differ; the shared sources use the names only (the time synchronization
 * looks at whether the result is 0, common/imu/time_sync.c).
 * Part of the POSIX network headers for a C library that has none
 * (common/compat/libc_net), which the FreeRTOS build uses with newlib.
 */

#ifndef COMMON_COMPAT_LIBC_NET_INCLUDE_NETDB_H
#define COMMON_COMPAT_LIBC_NET_INCLUDE_NETDB_H

#include <sys/socket.h>

struct addrinfo {
	int ai_flags;
	int ai_family;
	int ai_socktype;
	int ai_protocol;
	socklen_t ai_addrlen;
	struct sockaddr *ai_addr;
	char *ai_canonname;
	struct addrinfo *ai_next;
};

#define AI_PASSIVE     0x1
#define AI_CANONNAME   0x2
#define AI_NUMERICHOST 0x4
#define AI_ADDRCONFIG  0x20
#define AI_NUMERICSERV 0x400

#define EAI_BADFLAGS -1
#define EAI_NONAME   -2
#define EAI_AGAIN    -3
#define EAI_FAIL     -4
#define EAI_FAMILY   -6
#define EAI_SOCKTYPE -7
#define EAI_SERVICE  -8
#define EAI_MEMORY   -10
#define EAI_SYSTEM   -11
#define EAI_OVERFLOW -12

#endif /* COMMON_COMPAT_LIBC_NET_INCLUDE_NETDB_H */
