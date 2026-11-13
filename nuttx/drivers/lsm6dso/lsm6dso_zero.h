/* SPDX-License-Identifier: Apache-2.0 */
#ifndef LSM6DSO_ZERO_H_
#define LSM6DSO_ZERO_H_

#include <stdbool.h>
#include <stdint.h>

/* Tracks the gyro bias from the samples that pass while the sensor is at rest.
 *
 * Samples are grouped into segments of an eighth of a second. The sensor
 * counts as resting once LSM6DSO_ZERO_SEGMENTS consecutive segments have
 * the same mean angular rate and the same mean acceleration, within the
 * limits below. The mean rate of that window is then taken as the bias
 * seen now and blended into the estimate: a plain average over the first
 * windows, then an exponential one.
 *
 * Comparing segment means, not single samples, is deliberate: at 6667 Hz one
 * gyro sample has more noise than any useful limit. What biases the result
 * is net rotation during the window, and that is what the means show.
 *
 * Rotation about the gravity axis does not change the acceleration, so a slow
 * steady turn about it looks like a bias. The caller bounds the damage by
 * rejecting a window whose mean is far from a reference bias.
 */
#define LSM6DSO_ZERO_SEGMENTS      8
#define LSM6DSO_ZERO_BIAS_FRAC     8  /* fraction bits of the bias, as in lsm6dso_cal */
#define LSM6DSO_ZERO_WARMUP        20 /* windows averaged plainly before smoothing */

struct lsm6dso_zero_limits {
	int32_t gyro_spread;  /* segment means may differ by this, LSB << BIAS_FRAC */
	int32_t accel_spread; /* same for the acceleration */
	int32_t gyro_range;   /* window mean may be this far from the reference */
};

struct lsm6dso_zero {
	struct lsm6dso_zero_limits limits;
	int32_t reference[3];      /* LSB << BIAS_FRAC */
	uint32_t segment_samples;  /* gyro samples per segment; 0 disables tracking */
	/* Current segment. */
	uint32_t gyro_count, accel_count;
	int32_t gyro_sum[3], accel_sum[3];
	/* Current window. */
	uint8_t segments;
	int32_t first_gyro[3], first_accel[3]; /* means of its first segment */
	int64_t window_gyro[3];                /* sum of its segment means */
	/* Estimate. */
	int32_t bias[3];           /* LSB << BIAS_FRAC; valid when updates > 0 */
	uint32_t updates;
	uint32_t rejected;         /* resting windows outside gyro_range */
};

/* Limits for a sensitivity in udps/LSB (gyro) and ug/LSB (accel). */
void lsm6dso_zero_limits(struct lsm6dso_zero_limits *limits, uint32_t gyro_gain,
			 uint32_t accel_gain, bool reference_is_calibrated);
/* Forgets the estimate. reference is the bias to stay close to, or NULL for zero. */
void lsm6dso_zero_reset(struct lsm6dso_zero *zero, const struct lsm6dso_zero_limits *limits,
			const int32_t reference[3]);
/* Sets the gyro output data rate in Hz. Keeps the estimate, restarts the window. */
void lsm6dso_zero_set_rate(struct lsm6dso_zero *zero, uint32_t gyro_hz);
void lsm6dso_zero_accel(struct lsm6dso_zero *zero, const int16_t xyz[3]);
/* Returns true when this sample completed a resting window and changed the bias. */
bool lsm6dso_zero_gyro(struct lsm6dso_zero *zero, const int16_t xyz[3]);

#endif
