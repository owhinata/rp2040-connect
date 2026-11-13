/* SPDX-License-Identifier: Apache-2.0 */
/* boards/arduino-nano-rp2040-connect/src/nano_cpuload.h against the steps of
 * Zephyr's cpu_load_get_cpu() (lib/os/cpu_load/cpu_load_backend_idle.c).
 */
#include <assert.h>
#include <stdio.h>
#include "nano_cpuload.h"

static int get(struct nano_cpuload_s *s, uint64_t now, int reset)
{
	uint64_t total, idle;

	nano_cpuload_sample(s, now, now, reset, &total, &idle);
	return nano_cpuload_ratio(total, idle);
}

int main(void)
{
	struct nano_cpuload_s s = {0};

	/* No time has passed. */
	assert(get(&s, 0, 0) == 0);

	/* Since boot: 250 us idle of 1000. */
	nano_cpuload_enter(&s, 100);
	nano_cpuload_exit(&s, 350);
	nano_cpuload_exit(&s, 900); /* idempotent */
	assert(get(&s, 1000, 0) == 750);
	/* Without reset the window keeps growing from boot. */
	assert(get(&s, 2000, 0) == 875);
	/* Reset: this reading still covers boot..2000, the next starts at 2000. */
	assert(get(&s, 2000, 1) == 875);
	assert(get(&s, 2000, 0) == 0);
	/* Fully idle, rounded down: 1 us active of 3 -> 333. */
	nano_cpuload_enter(&s, 2001);
	nano_cpuload_exit(&s, 2003);
	assert(get(&s, 2003, 0) == 333);
	/* All busy. */
	assert(get(&s, 2003, 1) == 333);
	assert(get(&s, 5003, 1) == 1000);

	/* A window open while reading (another CPU in Zephyr): counted up to
	 * idle_now; with reset it is reopened there, not counted twice.
	 */
	nano_cpuload_enter(&s, 5503);
	{
		uint64_t total, idle;

		nano_cpuload_sample(&s, 6003, 6003, 1, &total, &idle);
		assert(total == 1000 && idle == 500);
		assert(nano_cpuload_ratio(total, idle) == 500);
		assert(s.enter == 6003 && s.idle == 0 && s.start == 6003);
	}
	nano_cpuload_exit(&s, 6503);
	assert(get(&s, 7003, 0) == 500);

	/* Idle can not exceed total. */
	assert(nano_cpuload_ratio(10, 20) == 0);
	puts("nano_cpuload ok");
	return 0;
}
