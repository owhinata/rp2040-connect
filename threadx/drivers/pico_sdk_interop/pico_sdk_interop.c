/* SPDX-License-Identifier: MIT */
/* The Pico SDK's lock hooks on ThreadX (pico_sdk_interop.h).  The logic is
 * that of FreeRTOS's RP2040 port (FreeRTOS-Kernel portable/ThirdParty/GCC/
 * RP2040/port.c, configSUPPORT_PICO_SYNC_INTEROP: MIT, Copyright (C) 2020
 * Amazon.com, Inc. or its affiliates, Copyright (c) 2021 Raspberry Pi
 * (Trading) Ltd.) for one core, on ThreadX's calls:
 *
 * - One event flags group; the bit of a lock is the number of its spin
 *   lock (the RP2040 has 32, ThreadX's flags are 32 bits, so no two spin
 *   locks share a bit; lock_cores that share a striped spin lock share it).
 * - A wait unlocks the spin lock, then gets the bit with TX_OR_CLEAR.  A
 *   notify that comes between the two is kept in the bit, so the get
 *   returns at once.  The SDK calls the waits in a loop that checks the
 *   state again, so a stale bit costs one pass.
 * - A notify sets the bit (tx_event_flags_set(), from a thread or an
 *   interrupt handler; ThreadX switches to a woken thread at once, or at
 *   the end of the handler).  It resumes every thread waiting for the bit
 *   and clears it, as FreeRTOS's event group does (xClearOnExit): so a
 *   thread that has unlocked the spin lock but not yet suspended when
 *   another waiter takes the bit misses that notify, the case the SDK's
 *   LOCK_INTERNAL_SPIN_UNLOCK_WITH_NOTIFY_WAKES_ALL 0 (its default with
 *   these hooks) is about.
 * - The group is created by the first wait that blocks, before it unlocks
 *   the spin lock (interrupts masked), so no notify can come between; until
 *   then no thread waits, and a notify only does what the SDK's does.
 *
 * A thread blocks only where ThreadX can suspend it and the caller expects
 * no more than the SDK does: in thread mode with the scheduler running
 * (ThreadX's system state 0: not before tx_kernel_enter()'s scheduler, not
 * in a handler), outside ThreadX's preemption lock (_tx_thread_preempt_disable,
 * which the layer's z_os_task_lock() also takes; a get would return at once
 * there), and with interrupts enabled before the spin lock was taken (save's
 * PRIMASK; ThreadX's switch lets interrupts in for a moment).  Elsewhere
 * the hooks are the SDK's own: WFE, or best_effort_wfe_or_timeout(), and
 * SEV at every notify for such a waiter.
 */

#include <stdbool.h>
#include <stdint.h>

#include "pico.h"
#include "hardware/sync.h"
#include "pico/lock_core.h"
#include "pico/time.h"

#include "tx_api.h"
#include "tx_event_flags.h"
#include "tx_thread.h"

#include "pico_sdk_interop.h"

#define TICK_US (1000000u / TX_TIMER_TICKS_PER_SECOND)

static TX_EVENT_FLAGS_GROUP events;

static inline ULONG event_bit(struct lock_core *lock)
{
	return (ULONG)1u << spin_lock_get_num(lock->spin_lock);
}

static inline bool can_block(uint32_t save)
{
	return (save & 1u) == 0u && TX_THREAD_GET_SYSTEM_STATE() == 0u &&
	       _tx_thread_current_ptr != TX_NULL && _tx_thread_preempt_disable == 0u;
}

/* With the spin lock held (interrupts masked), in a thread. */
static inline void events_create(void)
{
	if (events.tx_event_flags_group_id != TX_EVENT_FLAGS_ID) {
		(void)tx_event_flags_create(&events, "pico_sdk");
	}
}

/* The ticks to block that surely end before t, a tick ahead of it (a
 * ThreadX timeout of n ticks ends n - 1 to n ticks later); 0 when that is
 * less than a tick.  FreeRTOS's prvGetTicksToWaitBefore().
 */
static ULONG ticks_before(absolute_time_t t)
{
	int64_t us = absolute_time_diff_us(get_absolute_time(), t) - TICK_US;

	if (us < (int64_t)TICK_US) {
		return 0;
	}
	if (us > INT32_MAX) {
		us = INT32_MAX;
	}
	return (ULONG)((uint32_t)us / TICK_US);
}

uint32_t pico_sdk_interop_owner_id(void)
{
	uint32_t exception = __get_current_exception();

	/* Handlers: small numbers, below any TX_THREAD in RAM. */
	if (exception != 0u) {
		return exception * 2u;
	}
	return (uint32_t)(uintptr_t)_tx_thread_current_ptr;
}

void pico_sdk_interop_wait(struct lock_core *lock, uint32_t save)
{
	ULONG bit = event_bit(lock);
	ULONG actual;

	if (!can_block(save)) {
		spin_unlock(lock->spin_lock, save);
		__wfe();
		return;
	}
	events_create();
	spin_unlock(lock->spin_lock, save);
	(void)tx_event_flags_get(&events, bit, TX_OR_CLEAR, &actual, TX_WAIT_FOREVER);
}

void pico_sdk_interop_notify(struct lock_core *lock, uint32_t save)
{
	ULONG bit = event_bit(lock);

	spin_unlock(lock->spin_lock, save);
	__sev();
	if (events.tx_event_flags_group_id == TX_EVENT_FLAGS_ID) {
		(void)tx_event_flags_set(&events, bit, TX_OR);
	}
}

bool pico_sdk_interop_wait_until(struct lock_core *lock, uint32_t save, uint64_t until_us)
{
	absolute_time_t until = from_us_since_boot(until_us);
	ULONG bit = event_bit(lock);
	ULONG actual;
	ULONG ticks;

	if (!can_block(save)) {
		spin_unlock(lock->spin_lock, save);
		return best_effort_wfe_or_timeout(until);
	}
	ticks = ticks_before(until);
	if (ticks != 0u) {
		events_create();
		spin_unlock(lock->spin_lock, save);
		(void)tx_event_flags_get(&events, bit, TX_OR_CLEAR, &actual, ticks);
	} else {
		spin_unlock(lock->spin_lock, save);
	}
	if (time_reached(until)) {
		return true;
	}
	/* Less than two ticks left, or woken: the SDK checks again; let the
	 * threads of the same priority run meanwhile.
	 */
	tx_thread_relinquish();
	return false;
}
