/* SPDX-License-Identifier: MIT */
/* The part of POSIX's <arpa/inet.h> that the FreeRTOS build needs so far,
 * which newlib does not have: the byte order conversions (the RP2040 is
 * little-endian) and the text conversions of IPv4 addresses, inet_pton()
 * and inet_ntop() (common/compat/libc_net/inet.c; Zephyr's zsock_inet_pton()
 * and zsock_inet_ntop() of <zephyr/net/socket.h> are these).  Only AF_INET:
 * the port has no IPv6.
 * Part of the POSIX network headers for a C library that has none
 * (common/compat/libc_net), which the FreeRTOS build uses with newlib.
 */

#ifndef COMMON_COMPAT_LIBC_NET_INCLUDE_ARPA_INET_H
#define COMMON_COMPAT_LIBC_NET_INCLUDE_ARPA_INET_H

#include <stdint.h>
#include <netinet/in.h>
#include <sys/socket.h>

static inline uint32_t htonl(uint32_t v)
{
	return __builtin_bswap32(v);
}

static inline uint16_t htons(uint16_t v)
{
	return __builtin_bswap16(v);
}

static inline uint32_t ntohl(uint32_t v)
{
	return __builtin_bswap32(v);
}

static inline uint16_t ntohs(uint16_t v)
{
	return __builtin_bswap16(v);
}

/* POSIX's: AF_INET and a dotted decimal address of four numbers 0..255
 * (no leading '+', '-' or space, at most three digits each, nothing after
 * the fourth) into *dst (struct in_addr, network order): 1; text that is
 * not such an address: 0, *dst unchanged; another family: -1 with errno
 * EAFNOSUPPORT.
 */
int inet_pton(int af, const char *src, void *dst);

/* POSIX's: AF_INET, *src (struct in_addr) as "a.b.c.d" into dst: dst; a
 * buffer shorter than the text and its '\0': NULL with errno ENOSPC;
 * another family: NULL with errno EAFNOSUPPORT.
 */
const char *inet_ntop(int af, const void *src, char *dst, socklen_t size);

#endif /* COMMON_COMPAT_LIBC_NET_INCLUDE_ARPA_INET_H */
