/* SPDX-License-Identifier: MIT */
/* Zephyr's <zephyr/net/net_ip.h>, for the IPv4 address type struct in_addr
 * (imu_raw_mqtt.h): NuttX's <netinet/in.h> has it with the same member
 * (s_addr, network byte order).  Part of the small Zephyr compatibility
 * layer of the port (common/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_IP_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_IP_H

#include <netinet/in.h>

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_NET_NET_IP_H */
