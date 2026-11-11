/* SPDX-License-Identifier: Apache-2.0 */
#include "imu_raw_packet.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

static void put_le(uint8_t *p, uint64_t v, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		p[i] = (uint8_t)(v >> (8 * i));
	}
}

int imu_raw_encode(uint8_t *out, size_t capacity, size_t *written,
		   const struct imu_raw_metadata *meta,
		   const struct lsm6dso_fifo_sample *samples, size_t count)
{
	size_t accel = 0, gyro = 0, bitmap_len, size;

	if (!written) {
		return -EINVAL;
	}
	*written = 0;
	if (!out || !meta || !samples || count == 0 || count > IMU_RAW_MAX_WORDS ||
	    (meta->flags & ~IMU_RAW_FLAGS_MASK)) {
		return -EINVAL;
	}
	for (size_t i = 0; i < count; i++) {
		if (samples[i].kind == LSM6DSO_FIFO_ACCEL) {
			accel++;
		} else if (samples[i].kind == LSM6DSO_FIFO_GYRO) {
			gyro++;
		} else {
			return -EINVAL;
		}
	}
	if ((accel == 0) != (meta->accel_first_index == UINT64_MAX) ||
	    (gyro == 0) != (meta->gyro_first_index == UINT64_MAX) ||
	    (accel && meta->accel_first_index > UINT64_MAX - accel) ||
	    (gyro && meta->gyro_first_index > UINT64_MAX - gyro) ||
	    ((!accel || !(meta->flags & IMU_RAW_ACCEL_TIME_VALID)) && meta->accel_first_tick) ||
	    ((!gyro || !(meta->flags & IMU_RAW_GYRO_TIME_VALID)) && meta->gyro_first_tick) ||
	    (!accel && (meta->flags & IMU_RAW_ACCEL_TIME_VALID)) ||
	    (!gyro && (meta->flags & IMU_RAW_GYRO_TIME_VALID)) ||
	    /* Unix time comes from the tick and is zero unless marked valid. */
	    ((meta->flags & IMU_RAW_ACCEL_UNIX_VALID) && !(meta->flags & IMU_RAW_ACCEL_TIME_VALID)) ||
	    ((meta->flags & IMU_RAW_GYRO_UNIX_VALID) && !(meta->flags & IMU_RAW_GYRO_TIME_VALID)) ||
	    (!(meta->flags & IMU_RAW_ACCEL_UNIX_VALID) && meta->accel_first_unix_us) ||
	    (!(meta->flags & IMU_RAW_GYRO_UNIX_VALID) && meta->gyro_first_unix_us)) {
		return -EINVAL;
	}
	bitmap_len = (count + 7) / 8;
	size = IMU_RAW_HEADER_SIZE + bitmap_len + count * 6;
	if (capacity < size) {
		return -ENOSPC;
	}
	memset(out, 0, size);
	memcpy(out, "IMU1", 4);
	out[4] = IMU_RAW_VERSION;
	out[5] = meta->flags;
	put_le(out + 6, IMU_RAW_HEADER_SIZE, 2);
	memcpy(out + 8, meta->session_id, 16);
	put_le(out + 24, meta->batch_seq, 8);
	put_le(out + 32, meta->accel_first_index, 8);
	put_le(out + 40, meta->gyro_first_index, 8);
	put_le(out + 48, meta->accel_first_tick, 8);
	put_le(out + 56, meta->gyro_first_tick, 8);
	put_le(out + 64, count, 2);
	put_le(out + 66, bitmap_len, 2);
	put_le(out + 68, meta->epoch_id, 4);
	put_le(out + 72, meta->time_map_id, 4);
	put_le(out + 80, meta->accel_first_unix_us, 8);
	put_le(out + 88, meta->gyro_first_unix_us, 8);
	for (size_t i = 0; i < count; i++) {
		if (samples[i].kind == LSM6DSO_FIFO_ACCEL) {
			out[IMU_RAW_HEADER_SIZE + i / 8] |= (uint8_t)(1U << (i % 8));
		}
		for (size_t axis = 0; axis < 3; axis++) {
			put_le(out + IMU_RAW_HEADER_SIZE + bitmap_len + i * 6 + axis * 2,
			       (uint16_t)samples[i].xyz[axis], 2);
		}
	}
	*written = size;
	return 0;
}
