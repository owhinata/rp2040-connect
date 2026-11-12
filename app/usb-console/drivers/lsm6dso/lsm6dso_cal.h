/* SPDX-License-Identifier: Apache-2.0 */
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

/* gain is the sensitivity in ug/LSB (accel) or udps/LSB (gyro). */
void lsm6dso_cal_nominal(struct lsm6dso_cal *cal, enum lsm6dso_cal_kind kind, uint32_t gain);
void lsm6dso_cal_apply(const struct lsm6dso_cal *cal, const int16_t raw[3], int32_t out[3]);
/* Converts one output value to millionths of the unit (um/s^2 or urad/s). */
int64_t lsm6dso_cal_to_micro(int32_t value);

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
};

struct device;
void lsm6dso_cal_set_lookup(const struct device *dev, lsm6dso_cal_lookup_t lookup);
void lsm6dso_cal_reload(const struct device *dev);
void lsm6dso_cal_status(const struct device *dev, struct lsm6dso_cal_status *status);

#endif
