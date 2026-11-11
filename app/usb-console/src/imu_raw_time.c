/* SPDX-License-Identifier: Apache-2.0 */
#include "imu_raw_time.h"

#include <string.h>

void imu_raw_time_init(struct imu_raw_time *time)
{
	memset(time, 0, sizeof(*time));
	time->min_anchor_ticks = UINT32_MAX;
	time->min_anchor_slots = UINT32_MAX;
}

void imu_raw_time_new_epoch(struct imu_raw_time *time)
{
	time->anchored = false;
	time->period_valid = false;
	time->slot_open = false;
	time->anchor_pending = false;
	time->generation++;
}

void imu_raw_time_timestamp(struct imu_raw_time *time, const uint8_t payload[6], uint8_t counter)
{
	uint32_t raw = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
		       ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);

	if (time->raw_seen && raw < time->last_raw) {
		if (time->last_raw >= 0xC0000000U && raw < 0x40000000U) {
			/* The 32-bit counter wraps after about 30 hours. */
			time->wraps++;
		} else {
			/* Not a wrap: the counter restarted, or one of the two
			 * values is bad. Nothing before this point can be related
			 * to what follows. */
			time->restarts++;
			time->anchored = false;
			time->period_valid = false;
			time->generation++;
		}
	}
	time->raw_seen = true;
	time->last_raw = raw;
	time->pending_tick = (time->wraps << 32) | raw;
	time->pending_counter = counter & 3U;
	time->anchor_pending = true;
}

static void accept_anchor(struct imu_raw_time *time)
{
	if (time->anchored) {
		uint32_t slots = time->slots_since_anchor;
		uint64_t ticks = time->pending_tick - time->anchor_tick;
		/* 32-bit arithmetic: this runs for every timestamp word. */
		bool whole = time->pending_tick > time->anchor_tick && ticks <= UINT32_MAX &&
			     (uint32_t)ticks % slots == 0;

		if (ticks <= UINT32_MAX) {
			if ((uint32_t)ticks < time->min_anchor_ticks) {
				time->min_anchor_ticks = (uint32_t)ticks;
			}
			if ((uint32_t)ticks > time->max_anchor_ticks) {
				time->max_anchor_ticks = (uint32_t)ticks;
			}
		}
		if (slots < time->min_anchor_slots) {
			time->min_anchor_slots = slots;
		}
		if (slots > time->max_anchor_slots) {
			time->max_anchor_slots = slots;
		}
		uint32_t slot_ticks = whole ? (uint32_t)ticks / slots : 0;

		if (!whole || (time->period_valid && time->slot_ticks != slot_ticks)) {
			time->irregular_periods++;
		}
		time->period_valid = whole;
		time->slot_ticks = slot_ticks;
	}
	time->anchor_tick = time->pending_tick;
	time->slots_since_anchor = 0;
	time->anchored = true;
	time->anchors++;
}

void imu_raw_time_new_slot(struct imu_raw_time *time, uint8_t counter)
{
	counter &= 3U;
	if (!time->slot_open || counter != time->slot_counter) {
		/* First word of a new slot. */
		bool consecutive = !time->slot_open ||
				   counter == ((time->slot_counter + 1U) & 3U);

		if (!consecutive) {
			/* Slots are missing, so the distance to the last timestamp
			 * is no longer known. */
			time->counter_skips++;
			time->anchored = false;
			time->period_valid = false;
			time->generation++;
		}
		time->slots_since_anchor++;
		if (time->anchor_pending) {
			if (counter == time->pending_counter) {
				accept_anchor(time);
			} else {
				time->counter_mismatches++;
			}
			time->anchor_pending = false;
		}
		time->slot_counter = counter;
		time->slot_open = true;
	} else if (time->anchor_pending) {
		/* A timestamp inside a slot contradicts the assumed layout. */
		time->counter_mismatches++;
		time->anchor_pending = false;
	}
}

bool imu_raw_time_get(const struct imu_raw_time *time, uint64_t *tick)
{
	if (!time->anchored || (time->slots_since_anchor && !time->period_valid)) {
		return false;
	}
	*tick = time->anchor_tick + (uint64_t)time->slots_since_anchor * time->slot_ticks;
	return true;
}

bool imu_raw_time_sample(struct imu_raw_time *time, uint8_t counter, uint64_t *tick)
{
	imu_raw_time_advance(time, counter);
	return imu_raw_time_get(time, tick);
}
