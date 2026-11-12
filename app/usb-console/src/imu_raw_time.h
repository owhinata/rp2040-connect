/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_RAW_TIME_H_
#define IMU_RAW_TIME_H_

#include <stdbool.h>
#include <stdint.h>

/* Assigns LSM6DSO timestamp ticks to FIFO samples (experimental).
 *
 * The sensor writes the words of one sampling instant ("slot") together and
 * gives them the same 2-bit tag counter, which advances by one per slot. A
 * timestamp word precedes the samples of the slot it describes and carries
 * that slot's counter. This was observed on hardware with timestamp
 * decimation 8; it is checked at run time and counted when it does not hold.
 *
 * A sample in the timestamped slot gets that tick. Later slots are
 * extrapolated with the tick distance per slot measured between the two most
 * recent timestamps, and only if that distance is a whole number of ticks.
 *
 * A tick is provisional until the timestamp it rests on has been followed by
 * another one at a consistent distance (imu_raw_time_confirmed()). A single
 * timestamp cannot be told from a stale or corrupt one; the counter has been
 * seen to restart shortly after a FIFO start.
 * Ticks are in the sensor's own clock (nominally 25 us) extended to 64 bits.
 * Nothing here relates them to the host clock or to the physical sampling
 * instant.
 */
struct imu_raw_time {
	uint64_t anchor_tick, pending_tick, wraps;
	uint32_t last_raw, slots_since_anchor, slot_ticks;
	uint8_t slot_counter, pending_counter;
	bool raw_seen, anchored, period_valid, slot_open, anchor_pending;
	/* Changes whenever ticks handed out earlier stop being comparable with
	 * later ones: new epoch, missing slots, restarted counter. */
	uint32_t generation;
	/* Diagnostics for the whole session. */
	uint64_t anchors, irregular_periods, counter_mismatches, counter_skips, restarts;
	uint32_t min_anchor_ticks, max_anchor_ticks, min_anchor_slots, max_anchor_slots;
};

void imu_raw_time_init(struct imu_raw_time *time);
/* After a FIFO restart or lost words: forget the slot position. The 64-bit
 * extension and the diagnostics continue. */
void imu_raw_time_new_epoch(struct imu_raw_time *time);
/* raw is the sensor counter carried by a timestamp word, counter its tag counter. */
void imu_raw_time_timestamp(struct imu_raw_time *time, uint32_t raw, uint8_t counter);
/* Call for every accelerometer or gyroscope word in FIFO order. This runs
 * for each sample at the full data rate, so the common case (another word of
 * the current slot) stays inline. */
void imu_raw_time_new_slot(struct imu_raw_time *time, uint8_t counter);
static inline void imu_raw_time_advance(struct imu_raw_time *time, uint8_t counter)
{
	if (!time->slot_open || time->anchor_pending || (counter & 3U) != time->slot_counter) {
		imu_raw_time_new_slot(time, counter);
	}
}
/* Returns true and sets *tick when the time of the current slot is known,
 * provisionally. */
bool imu_raw_time_get(const struct imu_raw_time *time, uint64_t *tick);
/* True once the current anchor has been confirmed by a second timestamp. A
 * provisional tick may be used if this holds later and `generation` has not
 * changed since the tick was obtained. */
static inline bool imu_raw_time_confirmed(const struct imu_raw_time *time)
{
	return time->anchored && time->period_valid;
}
/* imu_raw_time_advance() followed by imu_raw_time_get(). */
bool imu_raw_time_sample(struct imu_raw_time *time, uint8_t counter, uint64_t *tick);
#endif
