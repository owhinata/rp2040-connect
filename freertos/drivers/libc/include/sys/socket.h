/* SPDX-License-Identifier: MIT */
/* The part of POSIX's <sys/socket.h> that the FreeRTOS build needs so far,
 * which newlib does not have: the address family of IPv4 and the generic
 * address types.  The values are NuttX's (and BSD's): the shared NINA core
 * (common/drivers/nina) was written and checked against them on NuttX.
 * Zephyr's own values differ (NET_AF_INET is 1), which no shared source
 * depends on: the families are only compared, never sent to the module.
 *
 * Only what the IMU service (common/imu/imu_stream.c, through
 * <zephyr/net/socket.h> and <netinet/in.h>) uses.  The socket layer of the
 * build (the zsock_*() calls on the NINA's sockets) adds the rest: the
 * socket types, options and calls.  Part of newlib's glue of the FreeRTOS
 * build (freertos/drivers/libc).
 */

#ifndef FREERTOS_DRIVERS_LIBC_SYS_SOCKET_H
#define FREERTOS_DRIVERS_LIBC_SYS_SOCKET_H

#include <stdint.h>

typedef uint16_t sa_family_t;
typedef uint32_t socklen_t;

#define AF_UNSPEC 0
#define AF_INET   2
#define PF_UNSPEC AF_UNSPEC
#define PF_INET   AF_INET

struct sockaddr {
	sa_family_t sa_family;
	char sa_data[14];
};

#endif /* FREERTOS_DRIVERS_LIBC_SYS_SOCKET_H */
