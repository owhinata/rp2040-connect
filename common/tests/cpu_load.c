/* SPDX-License-Identifier: Apache-2.0 */
/* The arithmetic of common/compat/cpu_load.h against the steps of
 * Zephyr's cpu_load_get_cpu() (lib/os/cpu_load/cpu_load_backend_idle.c).
 */
#include <assert.h>
#include <stdio.h>
#include "cpu_load.h"

static int get(struct cpu_load_window *s, uint64_t now, int reset)
{
	uint64_t total, idle;

	cpu_load_sample(s, now, now, reset, &total, &idle);
	return cpu_load_ratio(total, idle);
}

int main(void)
{
	struct cpu_load_window s = {0};

	/* No time has passed. */
	assert(get(&s, 0, 0) == 0);

	/* Since boot: 250 us idle of 1000. */
	cpu_load_enter(&s, 100);
	cpu_load_exit(&s, 350);
	cpu_load_exit(&s, 900); /* idempotent */
	assert(get(&s, 1000, 0) == 750);
	/* Without reset the window keeps growing from boot. */
	assert(get(&s, 2000, 0) == 875);
	/* Reset: this reading still covers boot..2000, the next starts at 2000. */
	assert(get(&s, 2000, 1) == 875);
	assert(get(&s, 2000, 0) == 0);
	/* Fully idle, rounded down: 1 us active of 3 -> 333. */
	cpu_load_enter(&s, 2001);
	cpu_load_exit(&s, 2003);
	assert(get(&s, 2003, 0) == 333);
	/* All busy. */
	assert(get(&s, 2003, 1) == 333);
	assert(get(&s, 5003, 1) == 1000);

	/* A window open while reading (another CPU in Zephyr): counted up to
	 * idle_now; with reset it is reopened there, not counted twice.
	 */
	cpu_load_enter(&s, 5503);
	{
		uint64_t total, idle;

		cpu_load_sample(&s, 6003, 6003, 1, &total, &idle);
		assert(total == 1000 && idle == 500);
		assert(cpu_load_ratio(total, idle) == 500);
		assert(s.enter == 6003 && s.idle == 0 && s.start == 6003);
	}
	cpu_load_exit(&s, 6503);
	assert(get(&s, 7003, 0) == 500);

	/* Idle can not exceed total. */
	assert(cpu_load_ratio(10, 20) == 0);
	puts("cpu_load ok");
	return 0;
}
