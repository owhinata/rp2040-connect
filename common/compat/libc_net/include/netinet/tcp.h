/* SPDX-License-Identifier: MIT */
/* TCP_NODELAY of POSIX's <netinet/tcp.h>, which newlib does not have: the
 * option at level IPPROTO_TCP that the NINA's sockets accept without effect
 * (common/drivers/nina/nina_sock.c, nina_sock_setsockopt()).  The value is
 * Linux's and Zephyr's (ZSOCK_TCP_NODELAY 1; NuttX's is 16).
 * Part of the POSIX network headers for a C library that has none
 * (common/compat/libc_net), which the FreeRTOS build uses with newlib.
 */

#ifndef FREERTOS_DRIVERS_LIBC_NETINET_TCP_H
#define FREERTOS_DRIVERS_LIBC_NETINET_TCP_H

#define TCP_NODELAY 1

#endif /* FREERTOS_DRIVERS_LIBC_NETINET_TCP_H */
