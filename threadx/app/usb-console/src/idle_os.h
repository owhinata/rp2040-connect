/* SPDX-License-Identifier: MIT */
/* The ThreadX part of the idle wait (common/imu/port/pico_sdk/
 * idle.c, which describes it): ThreadX has no idle thread.  With no thread
 * ready, its Cortex-M0 scheduler waits inside PendSV (tx_thread_schedule.S,
 * __tx_ts_wait), in a loop of
 *
 *   CPSID i; tx_low_power_enter(); DSB; WFI; ISB; tx_low_power_exit(); CPSIE i
 *
 * (tx_user.h: TX_LOW_POWER, TX_ENABLE_WFI), so the two halves of idle.c are
 * those hooks.  They run in handler mode (PendSV, the lowest priority) on
 * the main stack with interrupts masked.  When a handler makes a thread
 * ready, the loop leaves the wait and switches to it; the next wait opens a
 * new window.
 */

#ifndef IDLE_OS_H
#define IDLE_OS_H

#include "tx_api.h"

#if !defined(TX_LOW_POWER) || !defined(TX_ENABLE_WFI)
#error "the idle wait needs TX_LOW_POWER and TX_ENABLE_WFI (tx_user.h)"
#endif

#define IDLE_OS_ENTER tx_low_power_enter
#define IDLE_OS_EXIT  tx_low_power_exit

/* Called by the scheduler's wait, declared in no header of ThreadX. */
void tx_low_power_enter(void);
void tx_low_power_exit(void);

#endif /* IDLE_OS_H */
