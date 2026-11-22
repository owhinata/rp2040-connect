/* SPDX-License-Identifier: MIT */
/* The start of the layer's network part for an OS without a network stack of
 * its own (common/compat/net_offload, zephyr_net_init.c), for the start-up
 * code: like <zephyr_compat.h> it includes none of the layer's Zephyr
 * headers.  The FreeRTOS build's main() task calls it.
 */

#ifndef COMMON_COMPAT_NET_OFFLOAD_ZEPHYR_NET_INIT_H
#define COMMON_COMPAT_NET_OFFLOAD_ZEPHYR_NET_INIT_H

/* Starts the layer's network part as Zephyr's net_init() does at boot
 * (zephyr_net_init.c): the network event thread ("net_mgmt"), then the
 * one interface of the driver's NET_DEVICE_DT_OFFLOAD_DEFINE() (the NINA's,
 * common/drivers/nina/port/zephyr/nina_net.c): registered with the device's
 * MTU, the device's init and the interface's init (the NINA's: dormant, its
 * MAC address read from the module, which is the module's first command and
 * waits for its reset, 10 ms + 750 ms in nina_proto.c, or for its failure
 * when it does not answer; then its resolver is registered).  From a task,
 * once, before the driver's threads start.  0 or a negative errno (the event
 * thread could not start, or the device's init failed); a module that does
 * not answer is not an error here (the interface has no MAC address, and its
 * requests fail later).
 */
int zephyr_net_initialize(void);

#endif /* COMMON_COMPAT_NET_OFFLOAD_ZEPHYR_NET_INIT_H */
