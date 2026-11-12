/* SPDX-License-Identifier: Apache-2.0 */

#include <stdlib.h>
#include <string.h>
#include "lsm6dso_zero.h"

#define ONE (1 << LSM6DSO_ZERO_BIAS_FRAC)

/* Segment means may differ by 0.3 dps and 2 mg. At 6667 Hz the noise of a
 * segment mean is about 0.06 dps and 0.2 mg, so resting data passes with a
 * wide margin; a tilt of 0.1 degrees within a window does not.
 */
#define GYRO_SPREAD_UDPS  300000
#define ACCEL_SPREAD_UG   2000
/* How far the bias may be from the reference: a stored calibration is trusted
 * to within 1 dps; without one, the zero-rate level may be a few dps.
 */
#define GYRO_RANGE_CALIBRATED_UDPS 1000000
#define GYRO_RANGE_NOMINAL_UDPS    5000000

void lsm6dso_zero_limits(struct lsm6dso_zero_limits *limits, uint32_t gyro_gain,
			 uint32_t accel_gain, bool reference_is_calibrated)
{
	uint32_t range = reference_is_calibrated ? GYRO_RANGE_CALIBRATED_UDPS :
						    GYRO_RANGE_NOMINAL_UDPS;

	if (gyro_gain == 0 || accel_gain == 0) {
		memset(limits, 0, sizeof(*limits));
		return;
	}
	limits->gyro_spread = (int32_t)((int64_t)GYRO_SPREAD_UDPS * ONE / gyro_gain);
	limits->accel_spread = (int32_t)((int64_t)ACCEL_SPREAD_UG * ONE / accel_gain);
	limits->gyro_range = (int32_t)((int64_t)range * ONE / gyro_gain);
}

static void restart_segment(struct lsm6dso_zero *zero)
{
	zero->gyro_count = 0;
	zero->accel_count = 0;
	memset(zero->gyro_sum, 0, sizeof(zero->gyro_sum));
	memset(zero->accel_sum, 0, sizeof(zero->accel_sum));
}

static void restart_window(struct lsm6dso_zero *zero)
{
	zero->segments = 0;
	memset(zero->window_gyro, 0, sizeof(zero->window_gyro));
}

void lsm6dso_zero_reset(struct lsm6dso_zero *zero, const struct lsm6dso_zero_limits *limits,
			const int32_t reference[3])
{
	uint32_t segment_samples = zero->segment_samples;

	memset(zero, 0, sizeof(*zero));
	zero->limits = *limits;
	zero->segment_samples = segment_samples;
	if (reference != NULL) {
		memcpy(zero->reference, reference, sizeof(zero->reference));
	}
}

void lsm6dso_zero_set_rate(struct lsm6dso_zero *zero, uint32_t gyro_hz)
{
	uint32_t samples = gyro_hz / LSM6DSO_ZERO_SEGMENTS;

	/* Below about 26 Hz a segment is too short to average anything. */
	if (samples < 3) {
		samples = 0;
	}
	if (samples != zero->segment_samples) {
		zero->segment_samples = samples;
		restart_segment(zero);
		restart_window(zero);
	}
}

void lsm6dso_zero_accel(struct lsm6dso_zero *zero, const int16_t xyz[3])
{
	zero->accel_sum[0] += xyz[0];
	zero->accel_sum[1] += xyz[1];
	zero->accel_sum[2] += xyz[2];
	zero->accel_count++;
}

static bool within(const int32_t a[3], const int32_t b[3], int32_t limit)
{
	return abs(a[0] - b[0]) <= limit && abs(a[1] - b[1]) <= limit && abs(a[2] - b[2]) <= limit;
}

/* Blends the mean of a resting window into the estimate. */
static bool accept(struct lsm6dso_zero *zero)
{
	int32_t mean[3];
	uint32_t weight;

	for (int axis = 0; axis < 3; axis++) {
		mean[axis] = (int32_t)(zero->window_gyro[axis] / LSM6DSO_ZERO_SEGMENTS);
	}
	if (!within(mean, zero->reference, zero->limits.gyro_range)) {
		zero->rejected++;
		return false;
	}
	weight = zero->updates < LSM6DSO_ZERO_WARMUP ? zero->updates + 1 : LSM6DSO_ZERO_WARMUP;
	for (int axis = 0; axis < 3; axis++) {
		zero->bias[axis] += (mean[axis] - zero->bias[axis]) / (int32_t)weight;
	}
	zero->updates++;
	return true;
}

bool lsm6dso_zero_gyro(struct lsm6dso_zero *zero, const int16_t xyz[3])
{
	int32_t gyro[3], accel[3];
	bool changed = false;

	if (zero->segment_samples == 0) {
		return false;
	}
	zero->gyro_sum[0] += xyz[0];
	zero->gyro_sum[1] += xyz[1];
	zero->gyro_sum[2] += xyz[2];
	if (++zero->gyro_count < zero->segment_samples) {
		return false;
	}
	/* Without acceleration there is nothing to tell a tilt from a bias. */
	if (zero->accel_count == 0) {
		restart_segment(zero);
		restart_window(zero);
		return false;
	}
	for (int axis = 0; axis < 3; axis++) {
		gyro[axis] = (int32_t)((int64_t)zero->gyro_sum[axis] * ONE / (int32_t)zero->gyro_count);
		accel[axis] = (int32_t)((int64_t)zero->accel_sum[axis] * ONE /
					(int32_t)zero->accel_count);
	}
	restart_segment(zero);
	if (zero->segments > 0 && (!within(gyro, zero->first_gyro, zero->limits.gyro_spread) ||
				   !within(accel, zero->first_accel, zero->limits.accel_spread))) {
		/* Moved: this segment starts a new window. */
		restart_window(zero);
	}
	if (zero->segments == 0) {
		memcpy(zero->first_gyro, gyro, sizeof(gyro));
		memcpy(zero->first_accel, accel, sizeof(accel));
	}
	for (int axis = 0; axis < 3; axis++) {
		zero->window_gyro[axis] += gyro[axis];
	}
	if (++zero->segments == LSM6DSO_ZERO_SEGMENTS) {
		changed = accept(zero);
		restart_window(zero);
	}
	return changed;
}
