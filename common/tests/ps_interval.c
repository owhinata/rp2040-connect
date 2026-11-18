/* SPDX-License-Identifier: MIT */
/* common/shell/ps_interval.h: the CPU column of "ps" (FreeRTOS build) and
 * "kernel ps" (Zephyr build), each task's share since the previous list.
 */
#include <assert.h>
#include <stdint.h>
#include "ps_interval.h"

static struct ps_interval p;

int main(void)
{
	uint64_t elapsed;

	/* The shares. */
	assert(ps_interval_permille(0, 0) == 0);
	assert(ps_interval_permille(5, 0) == 0);
	assert(ps_interval_permille(0, 1000) == 0);
	assert(ps_interval_permille(1, 3) == 333);
	assert(ps_interval_permille(2, 3) == 667);  /* rounded to the nearest */
	assert(ps_interval_permille(1, 2000) == 1); /* 0.05% -> 0.1% */
	assert(ps_interval_permille(1, 2001) == 0);
	assert(ps_interval_permille(999999, 1000000) == 1000);
	assert(ps_interval_permille(1000, 1000) == 1000);
	assert(ps_interval_permille(1500, 1000) == 1000); /* read later than the total */
	/* Large counters (cycles at 125 MHz for years): no overflow. */
	assert(ps_interval_permille(UINT64_MAX / 4, UINT64_MAX / 2) == 500);
	assert(ps_interval_permille(UINT64_MAX / 10, UINT64_MAX) == 100);

	/* First list: since the start. */
	elapsed = ps_interval_begin(&p, 1000);
	assert(elapsed == 1000);
	assert(ps_interval_task(&p, 0xa0, 1, 900) == 900); /* idle */
	assert(ps_interval_task(&p, 0xb0, 2, 60) == 60);
	assert(ps_interval_task(&p, 0xc0, 3, 40) == 40);
	ps_interval_end(&p);

	/* Second list: the differences. */
	elapsed = ps_interval_begin(&p, 3000);
	assert(elapsed == 2000);
	assert(ps_interval_task(&p, 0xa0, 1, 1900) == 1000);
	assert(ps_interval_permille(1000, elapsed) == 500);
	assert(ps_interval_task(&p, 0xb0, 2, 860) == 800);
	assert(ps_interval_task(&p, 0xc0, 3, 240) == 200);
	ps_interval_end(&p);

	/* A task gone (0xc0), a new one (0xd0, counted from 0), two with the
	 * same name (names are not used) and the order changed.
	 */
	elapsed = ps_interval_begin(&p, 4000);
	assert(elapsed == 1000);
	assert(ps_interval_task(&p, 0xd0, 4, 300) == 300);
	assert(ps_interval_task(&p, 0xb0, 2, 960) == 100);
	assert(ps_interval_task(&p, 0xa0, 1, 2500) == 600);
	ps_interval_end(&p);
	assert(p.prev_count == 3);

	/* The handle of the gone task used again by a new one: another serial
	 * (FreeRTOS's task number) counts from 0; without a serial (Zephyr),
	 * a counter lower than the previous reading does.
	 */
	elapsed = ps_interval_begin(&p, 5000);
	assert(ps_interval_task(&p, 0xd0, 5, 50) == 50);
	assert(ps_interval_task(&p, 0xb0, 2, 1000) == 40);
	ps_interval_end(&p);
	elapsed = ps_interval_begin(&p, 6000);
	assert(ps_interval_task(&p, 0xd0, 5, 20) == 20); /* lower: a new task */
	ps_interval_end(&p);
	/* 0xb0 was not in the last list: from 0 again. */
	elapsed = ps_interval_begin(&p, 7000);
	assert(ps_interval_task(&p, 0xb0, 2, 1100) == 1100);
	assert(ps_interval_task(&p, 0xd0, 5, 120) == 100);
	ps_interval_end(&p);

	/* No time since the previous list. */
	elapsed = ps_interval_begin(&p, 7000);
	assert(elapsed == 0);
	assert(ps_interval_permille(ps_interval_task(&p, 0xb0, 2, 1100), elapsed) == 0);
	ps_interval_end(&p);

	/* More tasks than the table holds: those beyond count from 0 each
	 * time; the others keep their readings.
	 */
	elapsed = ps_interval_begin(&p, 8000);
	for (unsigned int i = 0; i < PS_INTERVAL_TASKS_MAX + 2; i++) {
		(void)ps_interval_task(&p, 0x1000 + i, 0, 10 * i);
	}
	ps_interval_end(&p);
	assert(p.prev_count == PS_INTERVAL_TASKS_MAX);
	elapsed = ps_interval_begin(&p, 9000);
	assert(elapsed == 1000);
	assert(ps_interval_task(&p, 0x1000 + 3, 0, 35) == 5);
	assert(ps_interval_task(&p, 0x1000 + PS_INTERVAL_TASKS_MAX, 0, 300) == 300);
	ps_interval_end(&p);
	return 0;
}
