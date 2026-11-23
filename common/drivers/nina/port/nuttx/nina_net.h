/* SPDX-License-Identifier: Apache-2.0 */
/* The NINA-W102 network device wlan0 (nina_netdev.c): the NuttX binding of
 * the core's Wi-Fi operations (common/drivers/nina/include/nina_wifi.h).  wapi's
 * wireless ioctls reach the core directly; the applications' Zephyr Wi-Fi
 * management (net_mgmt() requests and events of the compatibility layer)
 * is served by nina_wifi_mgmt.c.
 */

#ifndef __COMMON_DRIVERS_NINA_NUTTX_NINA_NET_H
#define __COMMON_DRIVERS_NINA_NUTTX_NINA_NET_H

/* Starts the network event thread ("net_mgmt"), registers wlan0 and
 * initializes it as Zephyr does at boot: no carrier, and its MAC address
 * read from the module (the module's first command, so this waits for its
 * reset: 10 ms + 750 ms in nina_proto.c).  The board calls it in its
 * bring-up.  0 or a negative errno.
 */

int nina_net_initialize(void);

/* Starts the core's thread nina_wifi_tid (a kernel thread on a static
 * stack, at the priority and with the stack of priorities.h) and returns
 * once it runs, where Zephyr starts its static threads: the init task
 * calls it.  0 or the negative errno of the thread's creation.
 */

int nina_net_start(void);

#endif /* __COMMON_DRIVERS_NINA_NUTTX_NINA_NET_H */
