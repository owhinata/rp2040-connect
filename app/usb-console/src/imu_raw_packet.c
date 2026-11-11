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

static int check(const struct imu_raw_metadata *meta, size_t count, size_t accel)
{
	size_t gyro = count - accel;

	if (!meta || count == 0 || count > IMU_RAW_MAX_WORDS || accel > count ||
	    (meta->flags & ~IMU_RAW_FLAGS_MASK)) {
		return -EINVAL;
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
	return 0;
}

static void put_header(uint8_t *out, const struct imu_raw_metadata *meta, size_t count)
{
	memset(out, 0, IMU_RAW_HEADER_SIZE);
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
	put_le(out + 66, (count + 7) / 8, 2);
	put_le(out + 68, meta->epoch_id, 4);
	put_le(out + 72, meta->time_map_id, 4);
	put_le(out + 80, meta->accel_first_unix_us, 8);
	put_le(out + 88, meta->gyro_first_unix_us, 8);
}

static void put_word(uint8_t *bitmap, uint8_t *raw, size_t index,
		     const struct lsm6dso_fifo_sample *sample)
{
	if (sample->kind == LSM6DSO_FIFO_ACCEL) {
		bitmap[index / 8] |= (uint8_t)(1U << (index % 8));
	}
	for (size_t axis = 0; axis < 3; axis++) {
		put_le(raw + index * 6 + axis * 2, (uint16_t)sample->xyz[axis], 2);
	}
}

int imu_raw_encode(uint8_t *out, size_t capacity, size_t *written,
		   const struct imu_raw_metadata *meta,
		   const struct lsm6dso_fifo_sample *samples, size_t count)
{
	size_t accel = 0, bitmap_len, size;
	int rc;

	if (!written) {
		return -EINVAL;
	}
	*written = 0;
	if (!out || !samples || count > IMU_RAW_MAX_WORDS) {
		return -EINVAL;
	}
	for (size_t i = 0; i < count; i++) {
		if (samples[i].kind == LSM6DSO_FIFO_ACCEL) {
			accel++;
		} else if (samples[i].kind != LSM6DSO_FIFO_GYRO) {
			return -EINVAL;
		}
	}
	rc = check(meta, count, accel);
	if (rc) {
		return rc;
	}
	bitmap_len = (count + 7) / 8;
	size = IMU_RAW_HEADER_SIZE + bitmap_len + count * 6;
	if (capacity < size) {
		return -ENOSPC;
	}
	put_header(out, meta, count);
	memset(out + IMU_RAW_HEADER_SIZE, 0, bitmap_len);
	for (size_t i = 0; i < count; i++) {
		put_word(out + IMU_RAW_HEADER_SIZE, out + IMU_RAW_HEADER_SIZE + bitmap_len, i,
			 &samples[i]);
	}
	*written = size;
	return 0;
}

/* While a payload is being built its raw values sit after a bitmap of the
 * maximum length; a batch that ends short is closed up when it is finished. */
#define STAGED_RAW (IMU_RAW_HEADER_SIZE + IMU_RAW_MAX_BITMAP)

void imu_raw_stage_reset(uint8_t *payload)
{
	memset(payload + IMU_RAW_HEADER_SIZE, 0, IMU_RAW_MAX_BITMAP);
}

void imu_raw_stage_put(uint8_t *payload, size_t index, const struct lsm6dso_fifo_sample *sample)
{
	put_word(payload + IMU_RAW_HEADER_SIZE, payload + STAGED_RAW, index, sample);
}

int imu_raw_stage_finish(uint8_t *payload, size_t *written, const struct imu_raw_metadata *meta,
			 size_t count, size_t accel)
{
	size_t bitmap_len = (count + 7) / 8;
	int rc;

	if (!payload || !written) {
		return -EINVAL;
	}
	*written = 0;
	rc = check(meta, count, accel);
	if (rc) {
		return rc;
	}
	if (bitmap_len != IMU_RAW_MAX_BITMAP) {
		memmove(payload + IMU_RAW_HEADER_SIZE + bitmap_len, payload + STAGED_RAW, count * 6);
	}
	put_header(payload, meta, count);
	*written = IMU_RAW_HEADER_SIZE + bitmap_len + count * 6;
	return 0;
}

void imu_raw_stage_reopen(uint8_t *payload, size_t count)
{
	size_t bitmap_len = (count + 7) / 8;

	if (bitmap_len != IMU_RAW_MAX_BITMAP) {
		memmove(payload + STAGED_RAW, payload + IMU_RAW_HEADER_SIZE + bitmap_len, count * 6);
		memset(payload + IMU_RAW_HEADER_SIZE + bitmap_len, 0, IMU_RAW_MAX_BITMAP - bitmap_len);
	}
}
