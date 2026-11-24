/* SPDX-License-Identifier: MIT */
/* The run time of the threads, for the CPU column of "ps"
 * (common/shell/ps_interval.h): counters that only grow, in microseconds
 * of the RP2040 TIMER, as the FreeRTOS build's run time counters are.
 *
 * ThreadX's execution profile kit (tx_user.h: TX_EXECUTION_PROFILE_ENABLE)
 * adds a thread's time at each switch out of it: from the switch in
 * (PendSV) to the switch out (the next PendSV), so the interrupt handlers
 * that interrupt a thread count in it (the Pico SDK's handlers do not call
 * the kit's _tx_execution_isr_enter()/_exit(), which only
 * _tx_thread_context_save()/_restore() would), as in FreeRTOS's run time
 * stats.  The time the scheduler waits with no thread ready, handlers in
 * it included, is the kit's idle time.  "cpu_active" is not this idle
 * time but the idle windows of common/pico_sdk/app/usb-console/src/idle.c
 * (idle_os.h), where handlers count as load, as in the other builds.
 *
 * The kit's clock is the TIMER's low word: a run of one thread longer than
 * 71.6 minutes without a switch would lose 2^32 us, and a run that starts
 * when the low word is exactly 0 is not counted (the kit takes a start of
 * 0 for "not running").  The totals are 64-bit.
 */

#ifndef THREADX_APP_USB_CONSOLE_SRC_RUN_TIME_H
#define THREADX_APP_USB_CONSOLE_SRC_RUN_TIME_H

#include <stdint.h>

#include "tx_api.h"

#include "rp2040_timer_us.h"

/* All the time since boot, the counter the threads' times are a share of
 * (FreeRTOS: app_run_time_us()).
 */
static inline uint64_t run_time_total_us(void)
{
	return rp2040_timer_us();
}

/* The time thread has run since it was created, its current run included
 * when it is the caller.  thread is a created thread (ended ones too, until
 * deleted).  From a thread.
 */
uint64_t run_time_thread_us(TX_THREAD *thread);

/* The time the scheduler has waited with no thread ready since it started.
 * From a thread.
 */
uint64_t run_time_idle_us(void);

#endif /* THREADX_APP_USB_CONSOLE_SRC_RUN_TIME_H */
