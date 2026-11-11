/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <stdint.h>
#include "imu_raw_time.h"

static void stamp(struct imu_raw_time *t, uint32_t raw, uint8_t counter)
{
	const uint8_t payload[6] = {(uint8_t)raw, (uint8_t)(raw >> 8), (uint8_t)(raw >> 16),
				    (uint8_t)(raw >> 24), 0, 0xaa};

	imu_raw_time_timestamp(t, payload, counter);
}

int main(void)
{
	struct imu_raw_time t;
	uint64_t tick = 99;

	/* Layout seen on hardware: timestamp, then gyro and accel per slot. */
	imu_raw_time_init(&t);
	assert(!imu_raw_time_sample(&t, 3, &tick) && tick == 99); /* before any timestamp */
	stamp(&t, 1000, 0);
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 1000); /* gyro */
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 1000); /* accel, same slot */
	/* The period is unknown until a second timestamp arrives. */
	assert(!imu_raw_time_sample(&t, 1, &tick));
	for (unsigned int slot = 2; slot < 8; slot++) {
		assert(!imu_raw_time_sample(&t, slot & 3, &tick));
	}
	stamp(&t, 1048, 0);
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 1048 && t.slot_ticks == 6);
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 1048);
	assert(imu_raw_time_sample(&t, 1, &tick) && tick == 1054);
	assert(imu_raw_time_sample(&t, 2, &tick) && tick == 1060);
	assert(t.anchors == 2 && t.irregular_periods == 0 && t.min_anchor_ticks == 48 &&
	       t.max_anchor_ticks == 48 && t.min_anchor_slots == 8 && t.max_anchor_slots == 8);

	/* A changed but whole period is adopted and counted. */
	for (unsigned int slot = 3; slot < 8; slot++) {
		assert(imu_raw_time_sample(&t, slot & 3, &tick));
	}
	assert(tick == 1048 + 7 * 6);
	stamp(&t, 1048 + 56, 0);
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 1104 && t.slot_ticks == 7 &&
	       t.irregular_periods == 1);
	/* A fractional period keeps the timestamped slot but not later slots. */
	for (unsigned int slot = 1; slot < 8; slot++) {
		assert(imu_raw_time_sample(&t, slot & 3, &tick));
	}
	stamp(&t, 1104 + 50, 0);
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 1154 && t.irregular_periods == 2);
	assert(!imu_raw_time_sample(&t, 1, &tick));

	/* A missing slot invalidates the position until the next timestamp. */
	imu_raw_time_init(&t);
	stamp(&t, 10, 0);
	assert(imu_raw_time_sample(&t, 0, &tick));
	for (unsigned int slot = 1; slot < 8; slot++) {
		(void)imu_raw_time_sample(&t, slot & 3, &tick);
	}
	stamp(&t, 58, 0);
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 58);
	assert(imu_raw_time_sample(&t, 1, &tick) && tick == 64);
	assert(!imu_raw_time_sample(&t, 3, &tick) && t.counter_skips == 1);
	assert(!imu_raw_time_sample(&t, 0, &tick));
	stamp(&t, 200, 1);
	assert(imu_raw_time_sample(&t, 1, &tick) && tick == 200);
	assert(!imu_raw_time_sample(&t, 2, &tick)); /* period must be measured again */

	/* A timestamp whose counter does not match the next slot is not used. */
	imu_raw_time_init(&t);
	stamp(&t, 10, 2);
	assert(!imu_raw_time_sample(&t, 0, &tick) && t.counter_mismatches == 1 && t.anchors == 0);
	stamp(&t, 20, 0);
	assert(!imu_raw_time_sample(&t, 0, &tick) && t.counter_mismatches == 2);

	/* The 32-bit counter is extended across a wrap; an epoch change forgets
	 * the slot position but not the extension. */
	imu_raw_time_init(&t);
	stamp(&t, 0xfffffff0U, 0);
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 0xfffffff0U);
	for (unsigned int slot = 1; slot < 8; slot++) {
		(void)imu_raw_time_sample(&t, slot & 3, &tick);
	}
	stamp(&t, 0x20, 0);
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 0x100000020ULL && t.slot_ticks == 6);
	imu_raw_time_new_epoch(&t);
	assert(!imu_raw_time_sample(&t, 1, &tick));
	stamp(&t, 0x80, 2);
	assert(imu_raw_time_sample(&t, 2, &tick) && tick == 0x100000080ULL);
	assert(!imu_raw_time_sample(&t, 3, &tick) && t.anchors == 3);

	/* Confirmation: one timestamp is provisional, the second confirms it. */
	imu_raw_time_init(&t);
	uint32_t generation = t.generation;

	stamp(&t, 1000, 0);
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 1000 && !imu_raw_time_confirmed(&t));
	for (unsigned int slot = 1; slot < 8; slot++) {
		(void)imu_raw_time_sample(&t, slot & 3, &tick);
	}
	stamp(&t, 1048, 0);
	assert(imu_raw_time_sample(&t, 0, &tick) && imu_raw_time_confirmed(&t) &&
	       t.generation == generation);
	/* A timestamp that goes back without being a wrap is a restarted
	 * counter: earlier ticks are void, and the new one needs confirming. */
	for (unsigned int slot = 1; slot < 8; slot++) {
		(void)imu_raw_time_sample(&t, slot & 3, &tick);
	}
	stamp(&t, 12, 0);
	assert(t.restarts == 1 && t.wraps == 0 && t.generation == generation + 1 &&
	       !imu_raw_time_confirmed(&t));
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 12 && !imu_raw_time_confirmed(&t));
	for (unsigned int slot = 1; slot < 8; slot++) {
		assert(!imu_raw_time_sample(&t, slot & 3, &tick));
	}
	stamp(&t, 60, 0);
	assert(imu_raw_time_sample(&t, 0, &tick) && tick == 60 && imu_raw_time_confirmed(&t));
	/* A new epoch and a skipped slot also change the generation. */
	imu_raw_time_new_epoch(&t);
	assert(t.generation == generation + 2);
	(void)imu_raw_time_sample(&t, 1, &tick);
	(void)imu_raw_time_sample(&t, 3, &tick);
	assert(t.generation == generation + 3 && t.counter_skips == 1);
	return 0;
}
