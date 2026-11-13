/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "imu_cal.h"
#include "lsm6dso_cal.h"

#define UNIT 4194304.0 /* 2^22 output units per m/s^2 or rad/s */
#define PI   3.14159265358979323846

static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}

/* The conversion the decoder used before this module existed. */
static int32_t legacy(int16_t raw, uint32_t gain, int accel)
{
	int64_t micro = raw;

	micro = accel ? micro * gain * 9806650LL / 1000000LL :
			micro * gain * 3141592LL / 180000000LL;
	return (int32_t)(micro * (1LL << 22) / 1000000LL);
}

static void test_nominal(void)
{
	static const uint32_t accel_gains[] = { 61, 122, 244, 488 };
	static const uint32_t gyro_gains[] = { 4375, 8750, 17500, 35000, 70000 };
	struct lsm6dso_cal cal;
	int32_t out[3];

	for (size_t g = 0; g < 4; g++) {
		lsm6dso_cal_nominal(&cal, LSM6DSO_CAL_ACCEL, accel_gains[g]);
		assert(cal.m[0][1] == 0 && cal.m[2][0] == 0 && cal.bias[1] == 0);
		for (int raw = -32768; raw <= 32767; raw += 257) {
			int16_t in[3] = { raw, -raw - 1, raw / 2 };
			double want = raw * (accel_gains[g] * 1e-6 * 9.80665) * UNIT;

			lsm6dso_cal_apply(&cal, in, out);
			/* Rounding (0.5) plus the coefficient step at full scale (1.0). */
			assert(fabs(out[0] - want) <= 1.5);
			/* Stays within the rounding of the previous conversion. */
			assert(labs((long)out[0] - legacy(in[0], accel_gains[g], 1)) <= 8);
			assert(labs((long)out[1] - legacy(in[1], accel_gains[g], 1)) <= 8);
		}
	}
	for (size_t g = 0; g < 5; g++) {
		lsm6dso_cal_nominal(&cal, LSM6DSO_CAL_GYRO, gyro_gains[g]);
		for (int raw = -32768; raw <= 32767; raw += 257) {
			int16_t in[3] = { 0, 0, raw };
			double want = raw * (gyro_gains[g] * 1e-6 * PI / 180.0) * UNIT;

			lsm6dso_cal_apply(&cal, in, out);
			assert(out[0] == 0 && fabs(out[2] - want) <= 1.5);
			/* The previous conversion used pi truncated to 3.141592
			 * (2e-7 low) and truncating divisions.
			 */
			assert(fabs((double)out[2] - legacy(in[2], gyro_gains[g], 0)) <=
			       8 + fabs(want) * 4e-7);
		}
	}
	/* The value the FIFO decoder test has always expected. */
	lsm6dso_cal_nominal(&cal, LSM6DSO_CAL_ACCEL, 1000);
	lsm6dso_cal_apply(&cal, (int16_t[]){ 1000, -1000, 0 }, out);
	assert(out[0] == 41132071 && out[1] == -41132071 && out[2] == 0);
}

static void test_calibrated(void)
{
	/* A full matrix and fractional biases, as a calibration produces. */
	static const double m[3][3] = {
		{ 2.38944e-3, -3.13e-6, -9.61e-6 },
		{ 1.2e-7, 2.40864e-3, 5.46e-6 },
		{ -2.3e-7, 4.1e-7, 2.39538e-3 },
	};
	static const double bias[3] = { -3.34, -48.07, 50.96 };
	struct lsm6dso_cal cal;
	int32_t out[3];

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			cal.m[i][j] = (int32_t)lround(m[i][j] * UNIT * (1 << LSM6DSO_CAL_COEFF_FRAC));
		}
		cal.bias[i] = (int32_t)lround(bias[i] * (1 << LSM6DSO_CAL_BIAS_FRAC));
	}
	for (int raw = -32768; raw <= 32767; raw += 331) {
		int16_t in[3] = { raw, 32767 - (raw + 32768), raw / 3 - 7 };

		lsm6dso_cal_apply(&cal, in, out);
		for (int i = 0; i < 3; i++) {
			double want = 0;

			for (int j = 0; j < 3; j++) {
				want += m[i][j] * (in[j] - bias[j]);
			}
			/* Half a bias step (1/512 LSB) of about 10000 units, plus rounding. */
			assert(fabs(out[i] - want * UNIT) <= 25.0);
		}
	}
}

static void test_limits(void)
{
	struct lsm6dso_cal cal = { .m = { { INT32_MAX, INT32_MAX, INT32_MAX } } };
	int32_t out[3];

	lsm6dso_cal_apply(&cal, (int16_t[]){ 32767, 32767, 32767 }, out);
	assert(out[0] == INT32_MAX && out[1] == 0);
	lsm6dso_cal_apply(&cal, (int16_t[]){ -32768, -32768, -32768 }, out);
	assert(out[0] == INT32_MIN);

	assert(lsm6dso_cal_to_micro(1 << 22) == 1000000);
	assert(lsm6dso_cal_to_micro(-(1 << 22)) == -1000000);
	assert(lsm6dso_cal_to_micro(0) == 0);
	assert(lsm6dso_cal_to_micro(2) == 0 && lsm6dso_cal_to_micro(-2) == 0);
	assert(lsm6dso_cal_to_micro(3) == 1 && lsm6dso_cal_to_micro(-3) == -1);
	assert(lsm6dso_cal_to_micro(41132071) == 9806650);
}

static void test_stored(void)
{
	uint8_t blob[IMU_CAL_BLOB_SIZE] = { IMU_CAL_VERSION, 1 };
	struct lsm6dso_cal cal;
	char key[IMU_CAL_KEY_MAX];

	assert(imu_cal_key(LSM6DSO_CAL_ACCEL, 244, key, sizeof(key)) == 17);
	assert(strcmp(key, "imu.cal.accel.244") == 0);
	assert(imu_cal_key(LSM6DSO_CAL_GYRO, 70000, key, sizeof(key)) > 0);
	assert(strcmp(key, "imu.cal.gyro.70000") == 0);
	assert(imu_cal_key(LSM6DSO_CAL_GYRO, UINT32_MAX, key, sizeof(key)) > 0);
	assert(imu_cal_key(LSM6DSO_CAL_GYRO, 70000, key, 8) == -ENAMETOOLONG);

	put_le32(blob + 4, 70000);
	for (int i = 0; i < 12; i++) {
		put_le32(blob + 8 + 4 * i, (uint32_t)(i % 2 ? -(1000 + i) : 1000 + i));
	}
	assert(imu_cal_decode(blob, sizeof(blob), LSM6DSO_CAL_GYRO, 70000, &cal) == 0);
	assert(cal.m[0][0] == 1000 && cal.m[0][1] == -1001 && cal.m[1][0] == -1003);
	assert(cal.m[2][2] == 1008 && cal.bias[0] == -1009 && cal.bias[2] == -1011);
	/* Not for this sensor or sensitivity. */
	assert(imu_cal_decode(blob, sizeof(blob), LSM6DSO_CAL_ACCEL, 70000, &cal) == -ENOENT);
	assert(imu_cal_decode(blob, sizeof(blob), LSM6DSO_CAL_GYRO, 35000, &cal) == -ENOENT);
	/* Malformed. */
	assert(imu_cal_decode(blob, sizeof(blob) - 1, LSM6DSO_CAL_GYRO, 70000, &cal) == -EINVAL);
	blob[0] = IMU_CAL_VERSION + 1;
	assert(imu_cal_decode(blob, sizeof(blob), LSM6DSO_CAL_GYRO, 70000, &cal) == -EINVAL);
	blob[0] = IMU_CAL_VERSION;
	blob[3] = 1;
	assert(imu_cal_decode(blob, sizeof(blob), LSM6DSO_CAL_GYRO, 70000, &cal) == -EINVAL);
}

int main(void)
{
	test_nominal();
	test_calibrated();
	test_limits();
	test_stored();
	return 0;
}
