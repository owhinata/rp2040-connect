/* SPDX-License-Identifier: Apache-2.0 */
/* The applications' Zephyr Wi-Fi management on NuttX (nina_wifi_mgmt.c),
 * for wlan0's binding (nina_netdev.c).
 */

#ifndef __COMMON_DRIVERS_NINA_NUTTX_NINA_WIFI_MGMT_H
#define __COMMON_DRIVERS_NINA_NUTTX_NINA_WIFI_MGMT_H

#include <stdbool.h>

struct net_driver_s;

/* Starts net_mgmt's event thread and makes DEV the compatibility layer's
 * interface (net_if_get_default(), net_if_get_wifi_sta()), whose
 * NET_REQUEST_WIFI_CONNECT reaches nina_wifi_connect().  0 or a negative
 * errno.
 */

int nina_wifi_mgmt_initialize(struct net_driver_s *dev);

/* NET_EVENT_WIFI_CONNECT_RESULT with STATUS, and
 * NET_EVENT_WIFI_DISCONNECT_RESULT with 0, on that interface.
 */

void nina_wifi_mgmt_connect_result(int status);
void nina_wifi_mgmt_disconnect_result(void);

#endif /* __COMMON_DRIVERS_NINA_NUTTX_NINA_WIFI_MGMT_H */
