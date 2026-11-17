/* SPDX-License-Identifier: MIT */
/* The part of Zephyr's <zephyr/net/socket.h> (Zephyr v4.5.0) that the IMU
 * service (common/imu/imu_stream.c) uses: the address conversions
 * zsock_inet_pton() and zsock_inet_ntop(), which in Zephyr are the C
 * library's inet_pton() and inet_ntop() under its own names (1 / the buffer
 * on success), here those of freertos/drivers/libc (AF_INET only), as on
 * NuttX (nuttx/drivers/compat/zephyr/net/socket.h).
 *
 * Not here yet: the sockets (zsock_socket() and the other calls on the
 * NINA's sockets, the poll and name resolution), which the build's socket
 * layer adds.  Part of the FreeRTOS implementation of the small Zephyr
 * compatibility layer of the port (freertos/drivers/compat).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#define zsock_inet_pton(family, src, dst)       inet_pton(family, src, dst)
#define zsock_inet_ntop(family, src, dst, size) inet_ntop(family, src, dst, size)

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_SOCKET_H */
