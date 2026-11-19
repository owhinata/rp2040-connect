/* SPDX-License-Identifier: MIT */
/* The NINA-W102 as the network interface of the FreeRTOS build
 * (nina_netif.c, with the core's Zephyr binding
 * common/drivers/nina/zephyr/nina_net.c, shared with the Zephyr build).  The
 * Wi-Fi operations themselves (common/drivers/nina/nina_wifi.c) are reached
 * as in the Zephyr build, through the compatibility layer's net_mgmt()
 * requests and events (<zephyr/net/wifi_mgmt.h>) and the interface
 * functions (<zephyr/net/net_if.h>).  The NuttX build's is
 * common/drivers/nina/nuttx/nina_net.h.
 */

#ifndef FREERTOS_DRIVERS_NINA_NINA_NET_H
#define FREERTOS_DRIVERS_NINA_NINA_NET_H

/* Starts the network event thread ("net_mgmt"), registers the interface
 * and initializes it as Zephyr's net_init() does at boot: driver_init(),
 * then the interface's init (dormant, and its MAC address read from the
 * module: the module's first command, so this waits for its reset, 10 ms +
 * 750 ms in nina_spi.c, or for its failure when it does not answer).  From
 * a task, once, before the threads nina_scan_tid, nina_connect_tid and
 * nina_monitor_tid start.  0 or a negative errno (the event thread could
 * not start); a module that does not answer is not an error here (the
 * interface has no MAC address, and its requests fail later).
 */
int nina_net_initialize(void);

#endif /* FREERTOS_DRIVERS_NINA_NINA_NET_H */
