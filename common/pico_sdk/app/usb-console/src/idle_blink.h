/* SPDX-License-Identifier: MIT */
/* The LED's blink from the idle pass (idle.c): the LED changes every
 * IDLE_BLINK_PERIOD_US while the idle pass runs, so it blinks as long as the
 * lowest priority still gets the processor.  The pass checks every time,
 * with the time it reads for the CPU load's window; the tick (1 ms) wakes
 * it at least that often.  No catch-up: when the idle pass did not run for
 * longer than a period, the next change is a period after this one.  Kept
 * apart for the host test (common/tests/idle_blink.c).
 */

#ifndef IDLE_BLINK_H
#define IDLE_BLINK_H

#include <stdbool.h>
#include <stdint.h>

#define IDLE_BLINK_PERIOD_US 500000u

/* True when the LED is to change at now_us (microseconds); *next_us is the
 * time of the next change, 0 at the start (the first pass changes it).
 */
static inline bool idle_blink_due(uint64_t *next_us, uint64_t now_us)
{
	if (now_us < *next_us) {
		return false;
	}
	*next_us += IDLE_BLINK_PERIOD_US;
	if (*next_us <= now_us) {
		*next_us = now_us + IDLE_BLINK_PERIOD_US;
	}
	return true;
}

#endif /* IDLE_BLINK_H */
