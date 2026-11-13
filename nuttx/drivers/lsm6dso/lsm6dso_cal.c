/* SPDX-License-Identifier: Apache-2.0 */

#include <string.h>
#include "lsm6dso_cal.h"

#define OUT_FRAC (31 - LSM6DSO_CAL_Q31_SHIFT)

/* Output units per ug (standard gravity) and per udps, times
 * 2^(COEFF_FRAC + 20).  The extra 20 bits keep the constant exact enough
 * that a full-scale reading is off by less than one output unit.
 */
#define ACCEL_PER_UG  706643604572ULL /* 9.80665e-6 * 2^56 */
#define GYRO_PER_UDPS 1257642267ULL   /* pi / 180e6 * 2^56 */

_Static_assert(OUT_FRAC + LSM6DSO_CAL_COEFF_FRAC == 36, "constants assume 2^36 per unit");

void lsm6dso_cal_nominal(struct lsm6dso_cal *cal, enum lsm6dso_cal_kind kind, uint32_t gain)
{
	uint64_t per_gain = kind == LSM6DSO_CAL_ACCEL ? ACCEL_PER_UG : GYRO_PER_UDPS;
	int32_t m = (int32_t)((gain * per_gain + (1U << 19)) >> 20);

	memset(cal, 0, sizeof(*cal));
	cal->m[0][0] = m;
	cal->m[1][1] = m;
	cal->m[2][2] = m;
}

void lsm6dso_cal_apply(const struct lsm6dso_cal *cal, const int16_t raw[3], int32_t out[3])
{
	const int shift = LSM6DSO_CAL_COEFF_FRAC + LSM6DSO_CAL_BIAS_FRAC;
	int64_t in[3];

	for (int axis = 0; axis < 3; axis++) {
		in[axis] = (int64_t)raw[axis] * (1 << LSM6DSO_CAL_BIAS_FRAC) - cal->bias[axis];
	}
	for (int axis = 0; axis < 3; axis++) {
		int64_t acc = cal->m[axis][0] * in[0] + cal->m[axis][1] * in[1] +
			      cal->m[axis][2] * in[2];

		/* Round to nearest; the shift of a negative value rounds down. */
		acc = (acc + (1LL << (shift - 1))) >> shift;
		out[axis] = acc > INT32_MAX ? INT32_MAX : acc < INT32_MIN ? INT32_MIN : (int32_t)acc;
	}
}

int64_t lsm6dso_cal_to_micro(int32_t value)
{
	int64_t scaled = (int64_t)value * 1000000;
	int64_t half = 1LL << (OUT_FRAC - 1);

	/* Round to nearest, symmetrically around zero. */
	return scaled >= 0 ? (scaled + half) >> OUT_FRAC : -((-scaled + half) >> OUT_FRAC);
}
