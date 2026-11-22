/* SPDX-License-Identifier: Apache-2.0 */
/* The LSM6DSO's conversion to physical units, its stored calibration, and
 * the tracking of the gyro bias at rest that feeds its bias (lsm6dso_cal.c):
 * the arithmetic of the driver's single reads and of its FIFO decoder, with
 * no OS.
 */
#ifndef LSM6DSO_CAL_H_
#define LSM6DSO_CAL_H_

#include <stdbool.h>
#include <stdint.h>

/* Conversion of raw accelerometer or gyro readings to physical units:
 *
 *   out = M * (raw - bias)
 *
 * M combines scale and axis misalignment.  Everything is integer arithmetic.
 * The result is Q31 with shift 9, that is, in units of 2^-22 m/s^2 or rad/s.
 * The same function serves calibrated and uncalibrated readings: without a
 * calibration, M is the nominal sensitivity on the diagonal and bias is zero.
 */
#define LSM6DSO_CAL_Q31_SHIFT  9
#define LSM6DSO_CAL_COEFF_FRAC 14 /* fraction bits of M, on top of the output unit */
#define LSM6DSO_CAL_BIAS_FRAC  8  /* fraction bits of bias, in LSB */

enum lsm6dso_cal_kind {
	LSM6DSO_CAL_ACCEL,
	LSM6DSO_CAL_GYRO,
};

struct lsm6dso_cal {
	int32_t m[3][3]; /* output units per LSB, times 2^LSM6DSO_CAL_COEFF_FRAC */
	int32_t bias[3]; /* LSB, times 2^LSM6DSO_CAL_BIAS_FRAC */
};

/* Supplies the calibration stored for a sensitivity.  Returns false when there
 * is none, in which case the nominal conversion is used.  Called in thread
 * context, at registration, on reload, and whenever a range is set.
 */
typedef bool (*lsm6dso_cal_lookup_t)(enum lsm6dso_cal_kind kind, uint32_t gain,
				     struct lsm6dso_cal *cal);

struct lsm6dso_cal_status {
	uint32_t accel_gain;
	uint32_t gyro_gain;
	bool accel_calibrated;
	bool gyro_calibrated;
	/* Gyro bias in use, in LSB times 2^LSM6DSO_CAL_BIAS_FRAC. It follows the
	 * readings taken at rest once gyro_bias_updates is not zero; until then
	 * it is the stored calibration's, or zero.
	 */
	int32_t gyro_bias[3];
	uint32_t gyro_bias_updates;
	uint32_t gyro_bias_rejected;
};

/* gain is the sensitivity in ug/LSB (accel) or udps/LSB (gyro). */
void lsm6dso_cal_nominal(struct lsm6dso_cal *cal, enum lsm6dso_cal_kind kind, uint32_t gain);
void lsm6dso_cal_apply(const struct lsm6dso_cal *cal, const int16_t raw[3], int32_t out[3]);
/* Converts one output value to millionths of the unit (um/s^2 or urad/s). */
int64_t lsm6dso_cal_to_micro(int32_t value);

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
#define LSM6DSO_ZERO_BIAS_FRAC     8  /* fraction bits of the bias, as in struct lsm6dso_cal */
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

#endif /* LSM6DSO_CAL_H_ */
