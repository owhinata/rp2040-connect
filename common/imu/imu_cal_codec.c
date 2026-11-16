/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <stdio.h>
#include "imu_cal.h"

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

int imu_cal_key(enum lsm6dso_cal_kind kind, uint32_t gain, char *key, size_t capacity)
{
	int len = snprintf(key, capacity, "imu.cal.%s.%u",
			   kind == LSM6DSO_CAL_ACCEL ? "accel" : "gyro", (unsigned int)gain);

	return len < 0 || (size_t)len >= capacity ? -ENAMETOOLONG : len;
}

int imu_cal_decode(const uint8_t *blob, size_t len, enum lsm6dso_cal_kind kind, uint32_t gain,
		   struct lsm6dso_cal *cal)
{
	if (len != IMU_CAL_BLOB_SIZE || blob[0] != IMU_CAL_VERSION || blob[2] != 0 || blob[3] != 0) {
		return -EINVAL;
	}
	if (blob[1] != (kind == LSM6DSO_CAL_ACCEL ? 0 : 1) || get_le32(blob + 4) != gain) {
		return -ENOENT;
	}
	for (int i = 0; i < 9; i++) {
		cal->m[i / 3][i % 3] = (int32_t)get_le32(blob + 8 + 4 * i);
	}
	for (int i = 0; i < 3; i++) {
		cal->bias[i] = (int32_t)get_le32(blob + 44 + 4 * i);
	}
	return 0;
}
