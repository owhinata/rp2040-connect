/* SPDX-License-Identifier: MIT */
/* ThreadX configuration of the host test of the Zephyr compatibility layer
 * (threadx/tests/compat, ThreadX's Linux port), read by tx_port.h
 * (TX_INCLUDE_USER_DEFINE_FILE): what the layer's ThreadX part
 * (common/compat/port/threadx/zephyr_kernel_os_impl.h) needs, as the board will
 * have it.
 *
 * - The tick: 1 ms (Z_OS_TICK_HZ; ThreadX's default is 10 ms).
 * - Timers, thread sleeps and timeouts expire in the tick interrupt
 *   (TX_TIMER_PROCESS_IN_ISR), as the layer's own timeouts do
 *   (zephyr_compat_tick()): no system timer thread, so no thread and stack
 *   above every other one that each wake-up of a sleep would pass through.
 * - TX_THREAD_USER_EXTENSION: the thread's pointer of its own (Zephyr's
 *   custom data, z_os_thread_local_get()), cleared by tx_thread_create().
 * - TX_DISABLE_NOTIFY_CALLBACKS: the layer registers no notify callback.
 *
 * The API's error checking stays on (the layer's calls get ThreadX's
 * checks of their arguments and callers), and so do the preemption
 * thresholds, the interruptible service paths and the stack filling.
 */

#ifndef THREADX_TESTS_COMPAT_TX_USER_H
#define THREADX_TESTS_COMPAT_TX_USER_H

#define TX_TIMER_TICKS_PER_SECOND 1000
#define TX_TIMER_PROCESS_IN_ISR
#define TX_DISABLE_NOTIFY_CALLBACKS

#define TX_THREAD_USER_EXTENSION VOID *tx_thread_zephyr_local;

#endif /* THREADX_TESTS_COMPAT_TX_USER_H */
