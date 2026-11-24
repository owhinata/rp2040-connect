/* SPDX-License-Identifier: MIT */
/* The part of the start of the Zephyr compatibility layer that the OS's
 * kernel hooks call, for an OS on the generic kernel part (this directory;
 * common/compat/include/zephyr_compat.h has zephyr_compat_initialize() and
 * K_THREAD_DEFINE_START(); the start of the network part is
 * common/compat/net_offload/include/zephyr_net_init.h).  Like
 * zephyr_compat.h it includes none of the layer's Zephyr headers.
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_COMPAT_OS_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_COMPAT_OS_H

/* Runs the timeouts of the layer (the timers of delayable work items,
 * z_add_timeout()) that are due: one call per kernel tick, from the tick
 * interrupt (FreeRTOS: vApplicationTickHook(), ThreadX: isr_systick, in
 * their builds' app/usb-console/src/tick.c), as
 * Zephyr runs its timeouts in the system clock interrupt.  The functions run
 * there, in the interrupt; the layer counts the calls as its ticks.
 */
void zephyr_compat_tick(void);

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_COMPAT_OS_H */
