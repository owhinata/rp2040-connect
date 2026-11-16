/* SPDX-License-Identifier: Apache-2.0 */
#include "imu_tick_clock.h"

#include <string.h>

void imu_tick_clock_init(struct imu_tick_clock *c)
{
	memset(c, 0, sizeof(*c));
	c->rate_us = 1000000;
	c->rate_ticks = IMU_TICK_CLOCK_NOMINAL_TICKS_PER_SECOND;
}

void imu_tick_clock_set_trim(struct imu_tick_clock *c, int8_t freq_fine)
{
	if (c->rate_measured) {
		return;
	}
	/* 10000 s take 40000 * (10000 + 15 * trim) ticks. */
	c->rate_us = 10000LL * 1000000LL;
	c->rate_ticks = (uint64_t)IMU_TICK_CLOCK_NOMINAL_TICKS_PER_SECOND *
			(uint64_t)(10000 + 15 * freq_fine);
}

void imu_tick_clock_restart(struct imu_tick_clock *c)
{
	c->count = 0;
}

void imu_tick_clock_add(struct imu_tick_clock *c, uint64_t tick, int64_t mono_us)
{
	const struct imu_tick_point *first;

	if (c->count) {
		const struct imu_tick_point *last = &c->points[c->count - 1];
		int64_t ticks = (int64_t)(tick - last->tick), ns = (mono_us - last->mono_us) * 1000;

		if (ticks <= 0 || ns <= 0 || ns < ticks * IMU_TICK_CLOCK_MIN_NS_PER_TICK ||
		    ns > ticks * IMU_TICK_CLOCK_MAX_NS_PER_TICK) {
			/* Start over from this reading. A measured rate stays in
			 * use until the new readings give another. */
			c->count = 0;
			c->restarts++;
		}
	}
	if (c->count == IMU_TICK_CLOCK_POINTS) {
		memmove(&c->points[0], &c->points[1],
			(IMU_TICK_CLOCK_POINTS - 1) * sizeof(c->points[0]));
		c->count--;
	}
	c->points[c->count++] = (struct imu_tick_point){tick, mono_us};
	first = &c->points[0];
	if (tick - first->tick >= IMU_TICK_CLOCK_MIN_SPAN_TICKS) {
		c->rate_us = mono_us - first->mono_us;
		c->rate_ticks = tick - first->tick;
		c->rate_measured = true;
	}
}

bool imu_tick_clock_convert(struct imu_tick_clock *c, uint64_t tick, int64_t *mono_us)
{
	const struct imu_tick_point *last;
	int64_t delta, result;

	if (!c->count) {
		return false;
	}
	if (c->emitted && tick == c->last_tick) {
		*mono_us = c->last_mono_us;
		return true;
	}
	/* Anchored at the newest reading: samples are at most about a second
	 * away from it, so an error in the rate has little leverage. */
	last = &c->points[c->count - 1];
	delta = (int64_t)(tick - last->tick);
	if (delta > (int64_t)IMU_TICK_CLOCK_MAX_AHEAD_TICKS ||
	    delta < -(int64_t)IMU_TICK_CLOCK_MAX_BEHIND_TICKS) {
		c->refused++;
		return false;
	}
	result = last->mono_us + delta * c->rate_us / (int64_t)c->rate_ticks;
	if (c->emitted && tick > c->last_tick && result <= c->last_mono_us) {
		result = c->last_mono_us + 1;
		c->held++;
	}
	if (!c->emitted || tick > c->last_tick ||
	    tick + IMU_TICK_CLOCK_MAX_BEHIND_TICKS < c->last_tick) {
		/* The last case: the counter restarted far below where it was. */
		c->emitted = true;
		c->last_tick = tick;
		c->last_mono_us = result;
	}
	*mono_us = result;
	return true;
}
