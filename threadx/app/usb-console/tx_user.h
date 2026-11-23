/* SPDX-License-Identifier: MIT */
/* ThreadX kernel configuration of the board (RP2040, one core; ThreadX's
 * Cortex-M0 port, ports/cortex_m0/gnu), read by tx_port.h
 * (TX_INCLUDE_USER_DEFINE_FILE).  What the ThreadX part of the Zephyr
 * compatibility layer (threadx/drivers/compat/zephyr_kernel_os_impl.h)
 * needs, as the host test has it (threadx/tests/compat/tx_user.h):
 *
 * - The tick: 1 ms (Z_OS_TICK_HZ; ThreadX's default is 10 ms).  SysTick
 *   makes it, set up by the board's _tx_initialize_low_level()
 *   (src/tx_initialize_low_level.c), handled by src/tick.c.
 * - Timers, thread sleeps and timeouts expire in the tick interrupt
 *   (TX_TIMER_PROCESS_IN_ISR), as the layer's own timeouts do
 *   (zephyr_compat_tick()): no system timer thread.
 * - TX_THREAD_USER_EXTENSION: the thread's pointer of its own (Zephyr's
 *   custom data, z_os_thread_local_get()), cleared by tx_thread_create().
 * - TX_DISABLE_NOTIFY_CALLBACKS: the layer registers no notify callback.
 *
 * The kernel allocates nothing: every object's memory is the caller's, and
 * no byte or block pool is created (tx_application_define() gets no free
 * memory, _tx_initialize_unused_memory is TX_NULL).  The API's error
 * checking stays on (ThreadX checks the arguments and the callers of the
 * layer's calls), and so do the preemption thresholds, the interruptible
 * service paths and the stack filling.  The scheduler waits for a ready
 * thread in a loop inside PendSV, without WFI (TX_ENABLE_WFI) and without
 * the low-power hooks (TX_LOW_POWER).
 */

#ifndef THREADX_APP_USB_CONSOLE_TX_USER_H
#define THREADX_APP_USB_CONSOLE_TX_USER_H

#define TX_TIMER_TICKS_PER_SECOND 1000
#define TX_TIMER_PROCESS_IN_ISR
#define TX_DISABLE_NOTIFY_CALLBACKS

#define TX_THREAD_USER_EXTENSION VOID *tx_thread_zephyr_local;

#endif /* THREADX_APP_USB_CONSOLE_TX_USER_H */
