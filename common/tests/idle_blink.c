/* SPDX-License-Identifier: MIT */
/* The host test of the LED's blink from the idle pass
 * (common/pico_sdk/app/usb-console/src/idle_blink.h): the first pass, the
 * period, passes in between, and a late pass.  No kernel; run by
 * common/tests/run.sh.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "idle_blink.h"

int main(void)
{
	uint64_t next = 0;
	uint64_t now;
	int changes = 0;

	/* The first pass changes the LED, the next change is a period later. */
	assert(idle_blink_due(&next, 1234567));
	assert(next == 1234567 + IDLE_BLINK_PERIOD_US);

	/* Passes every 1 ms (the tick) for 10 s: one change every period, on
	 * the first pass at or after it.
	 */
	for (now = 1235567; now < 1234567 + 10 * 1000000ull; now += 1000) {
		if (idle_blink_due(&next, now)) {
			assert(now % IDLE_BLINK_PERIOD_US == 1234567 % IDLE_BLINK_PERIOD_US);
			changes++;
		}
	}
	assert(changes == 19);

	/* Passes closer together change nothing in between. */
	assert(!idle_blink_due(&next, next - 1));
	now = next;
	assert(idle_blink_due(&next, now));
	assert(next == now + IDLE_BLINK_PERIOD_US);

	/* Late by more than a period (the idle task did not run): one change,
	 * the next a period after it, without catching up.
	 */
	now = next + 3 * IDLE_BLINK_PERIOD_US + 7;
	assert(idle_blink_due(&next, now));
	assert(next == now + IDLE_BLINK_PERIOD_US);
	assert(!idle_blink_due(&next, now + 1));

	/* Late by less than a period: the schedule is kept. */
	now = next + 100;
	assert(idle_blink_due(&next, now));
	assert(next == now - 100 + IDLE_BLINK_PERIOD_US);

	printf("idle blink: ok\n");
	return 0;
}
