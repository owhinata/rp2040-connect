/* SPDX-License-Identifier: MIT */
/* Zephyr's <zephyr/net/net_if.h>, empty for now: the IMU service
 * (common/imu/imu_stream.c) includes it but uses none of it, as the NuttX
 * build's was until it had its network device.  The network interface of the
 * NINA (struct net_if and its functions, nuttx/drivers/compat/zephyr/net/
 * net_if.h on NuttX) comes with the build's Wi-Fi.  Part of the FreeRTOS
 * implementation of the small Zephyr compatibility layer of the port
 * (freertos/drivers/compat).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_NET_IF_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_NET_IF_H

#include <netinet/in.h>

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_NET_NET_IF_H */
