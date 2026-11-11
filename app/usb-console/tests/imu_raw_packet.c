/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include "imu_raw_packet.h"

int main(void)
{
	struct imu_raw_metadata meta = {.batch_seq = 0x1234, .accel_first_index = 4,
		.gyro_first_index = 9, .epoch_id = 0x12345678};
	struct lsm6dso_fifo_sample samples[IMU_RAW_MAX_WORDS] = {
		{LSM6DSO_FIFO_GYRO, {-32768, 32767, -1}},
		{LSM6DSO_FIFO_ACCEL, {1, -2, 0x1234}},
	};
	uint8_t out[IMU_RAW_MAX_SIZE];
	size_t written;
	const uint8_t expected[] = {0x00, 0x80, 0xff, 0x7f, 0xff, 0xff,
				    0x01, 0x00, 0xfe, 0xff, 0x34, 0x12};

	memset(out, 0xa5, sizeof(out));
	assert(imu_raw_encode(out, 92, &written, &meta, samples, 2) == -ENOSPC);
	assert(written == 0 && out[0] == 0xa5);
	assert(imu_raw_encode(out, sizeof(out), &written, &meta, samples, 2) == 0);
	assert(written == 93 && memcmp(out, "IMU1\1\0\120\0", 8) == 0);
	assert(out[24] == 0x34 && out[25] == 0x12 && out[32] == 4 && out[40] == 9);
	assert(out[64] == 2 && out[66] == 1 && out[68] == 0x78 && out[71] == 0x12);
	assert(out[80] == 2 && !memcmp(out + 81, expected, sizeof(expected)));
	meta.flags = IMU_RAW_ACCEL_TIME_VALID;
	meta.accel_first_tick = 7;
	assert(imu_raw_encode(out, sizeof(out), &written, &meta, samples, 2) == 0);
	assert(out[5] == 1 && out[48] == 7);
	meta.flags = 0;
	assert(imu_raw_encode(out, sizeof(out), &written, &meta, samples, 2) == -EINVAL);
	meta.accel_first_tick = 0;
	meta.gyro_first_index = UINT64_MAX;
	assert(imu_raw_encode(out, sizeof(out), &written, &meta, samples + 1, 1) == 0);
	assert(written == 87 && out[80] == 1 && out[40] == 0xff);
	meta.accel_first_index = UINT64_MAX - 1;
	assert(imu_raw_encode(out, sizeof(out), &written, &meta, samples + 1, 1) == 0);
	meta.accel_first_index = UINT64_MAX;
	assert(imu_raw_encode(out, sizeof(out), &written, &meta, samples + 1, 1) == -EINVAL);
	meta.accel_first_index = 0;
	for (size_t i = 0; i < IMU_RAW_MAX_WORDS; i++) {
		samples[i].kind = LSM6DSO_FIFO_ACCEL;
	}
	assert(imu_raw_encode(out, sizeof(out), &written, &meta, samples, IMU_RAW_MAX_WORDS) == 0);
	assert(written == IMU_RAW_MAX_SIZE && out[64] == 0x80 && out[65] == 1 && out[66] == 48);
	assert(imu_raw_encode(out, sizeof(out), &written, &meta, samples, 0) == -EINVAL);
	assert(imu_raw_encode(out, sizeof(out), &written, &meta, samples, IMU_RAW_MAX_WORDS + 1) == -EINVAL);
	samples[0].kind = 99;
	assert(imu_raw_encode(out, sizeof(out), &written, &meta, samples, 1) == -EINVAL);
	return 0;
}
