/* SPDX-License-Identifier: Apache-2.0 */
/* The NINA-W102 network device wlan0 (nuttx/drivers/nina/nina_netdev.c and
 * nina_net.c, the Zephyr build's drivers/nina/nina_net.c).  The Wi-Fi
 * operations themselves are reached as in the Zephyr build, through the
 * compatibility layer's net_mgmt() requests and events
 * (<zephyr/net/wifi_mgmt.h>) and the interface functions
 * (<zephyr/net/net_if.h>).
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_NINA_NET_H
#define __NUTTX_DRIVERS_INCLUDE_NINA_NET_H

/* Starts the network event thread ("net_mgmt"), registers wlan0 and
 * initializes the interface as Zephyr does at boot: dormant, and its MAC
 * address read from the module (the module's first command, so this
 * waits for its reset: 10 ms + 750 ms in nina_spi.c).  The board calls
 * it in its bring-up; the init task then starts the threads nina_scan_tid,
 * nina_connect_tid and nina_monitor_tid.  0 or a negative errno.
 */

int nina_net_initialize(void);

#endif /* __NUTTX_DRIVERS_INCLUDE_NINA_NET_H */
