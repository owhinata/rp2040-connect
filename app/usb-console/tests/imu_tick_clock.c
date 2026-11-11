/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <stdlib.h>
#include "imu_tick_clock.h"

int main(void)
{
	struct imu_tick_clock c;
	int64_t mono, previous;

	imu_tick_clock_init(&c);
	assert(!imu_tick_clock_convert(&c, 100, &mono));

	/* One reading: nominal 25 us per tick, in both directions. */
	imu_tick_clock_add(&c, 1000000, 5000000);
	assert(imu_tick_clock_convert(&c, 1000000, &mono) && mono == 5000000);
	assert(imu_tick_clock_convert(&c, 1000400, &mono) && mono == 5010000);
	assert(!c.rate_measured);

	/* With the trim value 4 a tick is 1 / 40240 s. */
	imu_tick_clock_set_trim(&c, 4);
	assert(imu_tick_clock_convert(&c, 1000000 + 40240, &mono) && mono == 6000000);
	imu_tick_clock_set_trim(&c, -8);
	assert(imu_tick_clock_convert(&c, 1000000 + 2 * 39520, &mono) && mono == 7000000);

	/* Real sensor: 40219.2 ticks per second. After a second the measured
	 * rate replaces the nominal one. */
	imu_tick_clock_init(&c);
	for (int i = 0; i <= 20; i++) {
		imu_tick_clock_add(&c, 1000000 + (uint64_t)i * 40219 + i / 5,
				   5000000 + (int64_t)i * 1000000);
	}
	assert(c.rate_measured && c.count == IMU_TICK_CLOCK_POINTS);
	imu_tick_clock_set_trim(&c, 100); /* no effect once measured */
	/* 0.5 s after the newest reading: 20109.6 ticks. */
	assert(imu_tick_clock_convert(&c, 1000000 + 20 * 40219 + 4 + 20110, &mono));
	assert(llabs(mono - 25500000) < 20);

	/* Readings with +-150 us jitter: converted times of successive batch
	 * starts (1156 ticks apart) always increase. */
	imu_tick_clock_init(&c);
	srand(3);
	previous = 0;
	for (int i = 0; i < 60; i++) {
		uint64_t tick = 2000000 + (uint64_t)i * 40219;

		imu_tick_clock_add(&c, tick, 9000000 + (int64_t)i * 1000000 + rand() % 300 - 150);
		for (int b = 0; b < 34; b++) {
			assert(imu_tick_clock_convert(&c, tick + (uint64_t)b * 1156, &mono));
			assert(mono > previous);
			previous = mono;
		}
	}
	/* The same tick gives the same time; an earlier tick is answered but
	 * does not move the guard. */
	assert(imu_tick_clock_convert(&c, c.last_tick, &mono) && mono == previous);
	assert(imu_tick_clock_convert(&c, c.last_tick - 1156, &mono) && mono < previous);
	assert(imu_tick_clock_convert(&c, c.last_tick, &mono) && mono == previous);

	/* A conversion that would go back is held just above the last one. */
	imu_tick_clock_init(&c);
	imu_tick_clock_add(&c, 1000, 1000000);
	assert(imu_tick_clock_convert(&c, 1400, &mono) && mono == 1010000);
	imu_tick_clock_add(&c, 1401, 1005000); /* contradicts the previous reading */
	assert(imu_tick_clock_convert(&c, 1402, &mono) && mono == 1010001 && c.held == 1);

	/* Restart keeps the measured rate and the guard, drops the readings. */
	imu_tick_clock_init(&c);
	imu_tick_clock_add(&c, 0, 0);
	imu_tick_clock_add(&c, 80000, 1989000);
	assert(c.rate_measured && imu_tick_clock_convert(&c, 80000, &mono) && mono == 1989000);
	imu_tick_clock_restart(&c);
	assert(!imu_tick_clock_convert(&c, 90000, &mono));
	imu_tick_clock_add(&c, 200000, 5000000);
	assert(imu_tick_clock_convert(&c, 240000, &mono) && mono == 5994500);
	/* Out-of-order readings are ignored. */
	imu_tick_clock_add(&c, 200000, 6000000);
	imu_tick_clock_add(&c, 300000, 4000000);
	assert(c.count == 1);
	return 0;
}
