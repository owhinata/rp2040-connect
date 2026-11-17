/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the start of the Zephyr compatibility layer
 * (common/compat/zephyr_compat.h, which has zephyr_compat_initialize() and
 * K_THREAD_DEFINE_START()): what the application's kernel hooks call.  Like
 * zephyr_compat.h it includes none of the layer's Zephyr headers.  Part of
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

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_COMPAT_OS_H */
