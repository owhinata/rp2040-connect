/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_TICK_CLOCK_H_
#define IMU_TICK_CLOCK_H_

#include <stdbool.h>
#include <stdint.h>

/* Converts sensor ticks to the board clock from readings of both taken
 * together (about once per second).
 *
 * The tick length is measured, not assumed: the sensor oscillator was found
 * 0.55% off its nominal 25 us and 0.05% off the value its trim register
 * implies. The conversion is the line through the oldest and newest reading
 * kept, so a single reading's jitter (a few hundred microseconds) is spread
 * over the kept span. Until two readings are far enough apart, the rate of
 * the previous measurement is used, or the nominal one.
 *
 * Results never decrease from one call to the next as long as the ticks do
 * not; a result that would go back is held just above the previous one.
 */
#define IMU_TICK_CLOCK_POINTS 16
#define IMU_TICK_CLOCK_MIN_SPAN_TICKS 40000U /* about one second */
#define IMU_TICK_CLOCK_NOMINAL_TICKS_PER_SECOND 40000U

struct imu_tick_point {
	uint64_t tick;
	int64_t mono_us;
};

struct imu_tick_clock {
	struct imu_tick_point points[IMU_TICK_CLOCK_POINTS];
	unsigned int count;
	/* Rate as microseconds per `rate_ticks` ticks. */
	int64_t rate_us;
	uint64_t rate_ticks;
	bool rate_measured;
	bool emitted;
	uint64_t last_tick;
	int64_t last_mono_us;
	uint32_t held; /* results held to stay monotonic */
};

void imu_tick_clock_init(struct imu_tick_clock *c);
/* New measurement or FIFO restart: forget the readings, keep the rate and
 * the monotonic guard. */
void imu_tick_clock_restart(struct imu_tick_clock *c);
/* Rate to use until one has been measured, from the sensor's
 * INTERNAL_FREQ_FINE trim: one tick is 1 / (40000 * (1 + 0.0015 * trim)) s.
 * On the board tested this was 0.05% off the measured rate, against 0.55%
 * for the plain nominal value. Ignored once a rate has been measured. */
void imu_tick_clock_set_trim(struct imu_tick_clock *c, int8_t freq_fine);
/* Ignores a reading that is not later than the previous one. */
void imu_tick_clock_add(struct imu_tick_clock *c, uint64_t tick, int64_t mono_us);
/* False until a reading exists. */
bool imu_tick_clock_convert(struct imu_tick_clock *c, uint64_t tick, int64_t *mono_us);
#endif
