/* SPDX-License-Identifier: Apache-2.0 */
#include "imu_raw_batch.h"

#include <errno.h>
#include <string.h>

int imu_raw_batch_init(struct imu_raw_batch *batch, const uint8_t session_id[16],
		       uint32_t epoch_id, imu_raw_batch_sink sink, void *context)
{
	if (!batch || !session_id || !sink) {
		return -EINVAL;
	}
	memset(batch, 0, sizeof(*batch));
	memcpy(batch->meta.session_id, session_id, sizeof(batch->meta.session_id));
	batch->meta.epoch_id = epoch_id;
	batch->meta.accel_first_index = UINT64_MAX;
	batch->meta.gyro_first_index = UINT64_MAX;
	batch->sink = sink;
	batch->sink_context = context;
	return 0;
}

int imu_raw_batch_flush(struct imu_raw_batch *batch)
{
	size_t written, accel = 0, gyro = 0;
	int rc;

	if (!batch || !batch->sink) {
		return -EINVAL;
	}
	if (!batch->count) {
		return 0;
	}
	if (batch->next_batch == UINT64_MAX) {
		return -EOVERFLOW;
	}
	batch->meta.batch_seq = batch->next_batch;
	rc = imu_raw_encode(batch->payload, sizeof(batch->payload), &written,
			    &batch->meta, batch->samples, batch->count);
	if (rc) {
		return rc;
	}
	rc = batch->sink(batch->sink_context, batch->payload, written);
	if (rc && rc != -EAGAIN) {
		return rc;
	}
	if (rc == -EAGAIN) {
		for (size_t i = 0; i < batch->count; i++) {
			accel += batch->samples[i].kind == LSM6DSO_FIFO_ACCEL;
			gyro += batch->samples[i].kind == LSM6DSO_FIFO_GYRO;
		}
		batch->dropped_batches++;
		batch->dropped_accel += accel;
		batch->dropped_gyro += gyro;
	} else {
		batch->delivered_batches++;
	}
	batch->next_batch++;
	batch->count = 0;
	batch->meta.accel_first_index = UINT64_MAX;
	batch->meta.gyro_first_index = UINT64_MAX;
	return 0;
}

int imu_raw_batch_new_epoch(struct imu_raw_batch *batch)
{
	int rc;

	if (!batch || !batch->sink) {
		return -EINVAL;
	}
	if (batch->meta.epoch_id == UINT32_MAX) {
		return -EOVERFLOW;
	}
	rc = imu_raw_batch_flush(batch);
	if (rc) {
		return rc;
	}
	batch->meta.epoch_id++;
	return 0;
}

int imu_raw_batch_consume(struct imu_raw_batch *batch, const uint8_t *words,
			  size_t length, size_t *cursor)
{
	struct lsm6dso_fifo_entry entry;
	size_t next;
	int rc;

	if (!batch || !batch->sink || !cursor) {
		return -EINVAL;
	}
	for (;;) {
		next = *cursor;
		rc = lsm6dso_fifo_next_entry(words, length, &next, &entry);
		if (rc <= 0) {
			return rc;
		}
		if (entry.type == LSM6DSO_FIFO_ENTRY_OTHER) {
			return -EPROTO;
		}
		if (entry.type == LSM6DSO_FIFO_ENTRY_TIMESTAMP) {
			if (batch->timestamp_words == UINT64_MAX) {
				return -EOVERFLOW;
			}
			batch->timestamp_words++;
			*cursor = next;
			continue;
		}
		if (batch->count == IMU_RAW_MAX_WORDS) {
			rc = imu_raw_batch_flush(batch);
			if (rc) {
				return rc;
			}
		}
		if (entry.sample.kind == LSM6DSO_FIFO_ACCEL) {
			if (batch->next_accel == UINT64_MAX) {
				return -EOVERFLOW;
			}
			if (batch->meta.accel_first_index == UINT64_MAX) {
				batch->meta.accel_first_index = batch->next_accel;
			}
			batch->next_accel++;
		} else {
			if (batch->next_gyro == UINT64_MAX) {
				return -EOVERFLOW;
			}
			if (batch->meta.gyro_first_index == UINT64_MAX) {
				batch->meta.gyro_first_index = batch->next_gyro;
			}
			batch->next_gyro++;
		}
		batch->samples[batch->count++] = entry.sample;
		*cursor = next;
		if (batch->count == IMU_RAW_MAX_WORDS) {
			rc = imu_raw_batch_flush(batch);
			if (rc) {
				return rc;
			}
		}
	}
}
