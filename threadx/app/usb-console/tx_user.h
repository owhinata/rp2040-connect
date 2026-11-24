/* SPDX-License-Identifier: MIT */
/* ThreadX kernel configuration of the board (RP2040, one core; ThreadX's
 * Cortex-M0 port, ports/cortex_m0/gnu), read by tx_port.h and by the port's
 * assembly sources (TX_INCLUDE_USER_DEFINE_FILE, which threadx_kernel.cmake
 * passes to both; so everything here outside the C part at the end is
 * plain #define).  What the ThreadX part of the Zephyr compatibility layer
 * (threadx/drivers/compat/zephyr_kernel_os_impl.h) needs, as the host test
 * has it (threadx/tests/compat/tx_user.h):
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
 * The board's own:
 *
 * - The idle wait (src/idle.c): with no thread ready, the scheduler waits
 *   inside PendSV, interrupts masked, calling tx_low_power_enter(), then
 *   WFI, then tx_low_power_exit(), then unmasking them, in a loop
 *   (TX_LOW_POWER, TX_ENABLE_WFI; tx_thread_schedule.S).  The two hooks
 *   open and close the CPU load's window around the WFI.
 * - The run time of each thread for "ps" (src/run_time.h): ThreadX's
 *   execution profile kit (utility/execution_profile_kit,
 *   TX_EXECUTION_PROFILE_ENABLE), which the scheduler calls at every switch
 *   out of a thread and into one.  Its clock is the low word of the RP2040
 *   TIMER, 1 us, read without side effects (TIMERAWL, rp2040_timer_us.h);
 *   the kit takes a 32-bit source that wraps at TX_EXECUTION_MAX_TIME_SOURCE
 *   (its default, 0xFFFFFFFF, every 71.6 minutes: a run of one thread may
 *   be up to that long) and keeps the totals in 64 bits (EXECUTION_TIME).
 *
 * The kernel allocates nothing: every object's memory is the caller's, and
 * no byte or block pool is created (tx_application_define() gets no free
 * memory, _tx_initialize_unused_memory is TX_NULL).  The API's error
 * checking stays on (ThreadX checks the arguments and the callers of the
 * layer's calls), and so do the preemption thresholds, the interruptible
 * service paths and the stack filling.
 */

#ifndef THREADX_APP_USB_CONSOLE_TX_USER_H
#define THREADX_APP_USB_CONSOLE_TX_USER_H

#define TX_TIMER_TICKS_PER_SECOND 1000
#define TX_TIMER_PROCESS_IN_ISR
#define TX_DISABLE_NOTIFY_CALLBACKS

#define TX_THREAD_USER_EXTENSION VOID *tx_thread_zephyr_local;

#define TX_LOW_POWER
#define TX_ENABLE_WFI

#define TX_EXECUTION_PROFILE_ENABLE

#ifndef __ASSEMBLER__
#include "rp2040_timer_us.h"

/* Read by the kit's C code only (tx_execution_profile.c), after its
 * typedef of EXECUTION_TIME_SOURCE_TYPE.
 */
#define TX_EXECUTION_TIME_SOURCE                                                                   \
	((EXECUTION_TIME_SOURCE_TYPE)RP2040_TIMER_READ(RP2040_TIMER_TIMERAWL_OFFSET))
#endif

#endif /* THREADX_APP_USB_CONSOLE_TX_USER_H */
