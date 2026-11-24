/* SPDX-License-Identifier: MIT */
/* The run time of the threads from ThreadX's execution profile kit
 * (run_time.h).  The kit changes a thread's total and its start only in
 * PendSV, the idle total also in the scheduler's wait: read with
 * interrupts masked, so that no switch comes between the two words of a
 * total.
 */

#include <stdint.h>

#include "tx_api.h"
#include "tx_thread.h"

#include "run_time.h"

#ifndef TX_EXECUTION_PROFILE_ENABLE
#error "run_time.c needs TX_EXECUTION_PROFILE_ENABLE (tx_user.h)"
#endif

/* The kit's (tx_execution_profile.c), declared in no header. */
extern EXECUTION_TIME _tx_execution_idle_time_total;

uint64_t run_time_thread_us(TX_THREAD *thread)
{
	TX_INTERRUPT_SAVE_AREA
	uint64_t total;
	EXECUTION_TIME_SOURCE_TYPE start;

	TX_DISABLE
	total = thread->tx_thread_execution_time_total;
	start = thread->tx_thread_execution_time_last_start;
	/* Non-zero only while it runs: the caller's run since its switch in. */
	if (start != 0u) {
		total += (EXECUTION_TIME_SOURCE_TYPE)(TX_EXECUTION_TIME_SOURCE - start);
	}
	TX_RESTORE
	return total;
}

uint64_t run_time_idle_us(void)
{
	TX_INTERRUPT_SAVE_AREA
	uint64_t total;

	TX_DISABLE
	total = _tx_execution_idle_time_total;
	TX_RESTORE
	return total;
}
