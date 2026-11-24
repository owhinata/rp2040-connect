/* SPDX-License-Identifier: MIT */
/* The part of POSIX's <netinet/in.h> that the FreeRTOS build needs, which
 * newlib does not have: the IPv4 address and socket address types, the
 * protocol numbers, and the IPv6 socket address, whose size the MQTT client
 * takes for an address that is not IPv4 (common/imu/mqtt_min.c; the build
 * has no IPv6).  Addresses and ports are in network byte order, as in POSIX
 * and Zephyr.  The byte order functions are <arpa/inet.h>'s.  The values
 * are Linux's (and Zephyr's and NuttX's): IPPROTO_TCP 6, IPPROTO_UDP 17.
 * Part of the POSIX network headers for a C library that has none
 * (common/compat/libc_net), which the FreeRTOS build uses with newlib.
 */

#ifndef COMMON_COMPAT_LIBC_NET_INCLUDE_NETINET_IN_H
#define COMMON_COMPAT_LIBC_NET_INCLUDE_NETINET_IN_H

#include <stdint.h>
#include <sys/socket.h>

typedef uint16_t in_port_t;
typedef uint32_t in_addr_t;

#define IPPROTO_IP  0
#define IPPROTO_TCP 6
#define IPPROTO_UDP 17

struct in_addr {
	in_addr_t s_addr;
};

struct sockaddr_in {
	sa_family_t sin_family;
	in_port_t sin_port;
	struct in_addr sin_addr;
	uint8_t sin_zero[8];
};

struct in6_addr {
	uint8_t s6_addr[16];
};

struct sockaddr_in6 {
	sa_family_t sin6_family;
	in_port_t sin6_port;
	uint32_t sin6_flowinfo;
	struct in6_addr sin6_addr;
	uint32_t sin6_scope_id;
};

#define INADDR_ANY       ((in_addr_t)0x00000000)
#define INET_ADDRSTRLEN  16
#define INET6_ADDRSTRLEN 46

#endif /* COMMON_COMPAT_LIBC_NET_INCLUDE_NETINET_IN_H */
