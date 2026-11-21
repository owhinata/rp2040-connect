/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the start of the Zephyr compatibility layer
 * (common/compat/zephyr_compat.h, which has zephyr_compat_initialize() and
 * K_THREAD_DEFINE_START()): what the application's kernel hooks call, and
 * the start of the network part.  Like zephyr_compat.h it includes none of
 * the layer's Zephyr headers.  Part of
 * the FreeRTOS implementation of the layer (freertos/drivers/compat).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_COMPAT_OS_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_COMPAT_OS_H

/* Runs the timeouts of the layer (the timers of delayable work items,
 * z_add_timeout()) that are due: one call per kernel tick, from the tick
 * interrupt (FreeRTOS's vApplicationTickHook(), configUSE_TICK_HOOK), as
 * Zephyr runs its timeouts in the system clock interrupt.  The functions run
 * there, in the interrupt; the layer counts the calls as its ticks.
 */
void zephyr_compat_tick(void);

/* Starts the layer's network part as Zephyr's net_init() does at boot
 * (zephyr_net_init.c): the network event thread ("net_mgmt"), then the
 * one interface of the driver's NET_DEVICE_DT_OFFLOAD_DEFINE() (the NINA's,
 * common/drivers/nina/port/zephyr/nina_net.c): registered with the device's MTU,
 * the device's init and the interface's init (the NINA's: dormant, its MAC
 * address read from the module, which is the module's first command and
 * waits for its reset, 10 ms + 750 ms in nina_proto.c, or for its failure when
 * it does not answer; then its resolver is registered).  From a task, once,
 * before the driver's threads start.  0 or a negative errno (the event
 * thread could not start, or the device's init failed); a module that does
 * not answer is not an error here (the interface has no MAC address, and its
 * requests fail later).
 */
int zephyr_net_initialize(void);

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_COMPAT_OS_H */
