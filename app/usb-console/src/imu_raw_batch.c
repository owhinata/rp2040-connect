/* SPDX-License-Identifier: Apache-2.0 */
#include "imu_raw_batch.h"

#include <errno.h>
#include <stdbool.h>
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
	imu_raw_time_init(&batch->time);
	imu_raw_stage_reset(batch->payload);
	return 0;
}

/* Keeps a noted tick only if its timestamp has been confirmed since and
 * nothing invalidated it, then asks the clock for server time. Run once per
 * flush attempt; a batch kept after a sink error is settled again. */
static void settle_one(struct imu_raw_batch *batch, bool noted, uint32_t generation,
		       uint64_t *tick, uint64_t *unix_us, uint8_t tick_flag, uint8_t unix_flag)
{
	batch->meta.flags &= (uint8_t)~(tick_flag | unix_flag);
	*unix_us = 0;
	if (!noted || generation != batch->time.generation ||
	    !imu_raw_time_confirmed(&batch->time)) {
		return;
	}
	batch->meta.flags |= tick_flag;
	if (batch->clock && batch->clock(batch->clock_context, *tick, unix_us)) {
		batch->meta.flags |= unix_flag;
	} else {
		*unix_us = 0;
	}
}

static void settle_times(struct imu_raw_batch *batch)
{
	struct imu_raw_metadata *m = &batch->meta;
	/* The clock wants ticks in non-decreasing order. */
	bool accel_first = !batch->gyro_tick_noted ||
			   (batch->accel_tick_noted &&
			    batch->accel_noted_tick <= batch->gyro_noted_tick);

	for (int pass = 0; pass < 2; pass++) {
		if ((pass == 0) == accel_first) {
			settle_one(batch, batch->accel_tick_noted, batch->accel_tick_generation,
				   &batch->accel_noted_tick, &m->accel_first_unix_us,
				   IMU_RAW_ACCEL_TIME_VALID,
				   IMU_RAW_ACCEL_UNIX_VALID);
		} else {
			settle_one(batch, batch->gyro_tick_noted, batch->gyro_tick_generation,
				   &batch->gyro_noted_tick, &m->gyro_first_unix_us,
				   IMU_RAW_GYRO_TIME_VALID,
				   IMU_RAW_GYRO_UNIX_VALID);
		}
	}
	/* The encoder requires a tick of zero unless it is marked valid. */
	m->accel_first_tick = (m->flags & IMU_RAW_ACCEL_TIME_VALID) ? batch->accel_noted_tick : 0;
	m->gyro_first_tick = (m->flags & IMU_RAW_GYRO_TIME_VALID) ? batch->gyro_noted_tick : 0;
}

int imu_raw_batch_flush(struct imu_raw_batch *batch)
{
	size_t written;
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
	settle_times(batch);
	rc = imu_raw_stage_finish(batch->payload, &written, &batch->meta, batch->count,
				  batch->accel_count);
	if (rc) {
		return rc;
	}
	rc = batch->sink(batch->sink_context, batch->payload, written);
	if (rc && rc != -EAGAIN) {
		/* Keep the batch so that a later flush can deliver it. */
		imu_raw_stage_reopen(batch->payload, batch->count);
		return rc;
	}
	if (rc == -EAGAIN) {
		batch->dropped_batches++;
		batch->dropped_accel += batch->accel_count;
		batch->dropped_gyro += batch->count - batch->accel_count;
	} else {
		batch->delivered_batches++;
	}
	batch->next_batch++;
	batch->count = 0;
	batch->accel_count = 0;
	imu_raw_stage_reset(batch->payload);
	batch->meta.accel_first_index = UINT64_MAX;
	batch->meta.gyro_first_index = UINT64_MAX;
	batch->meta.accel_first_tick = 0;
	batch->meta.gyro_first_tick = 0;
	batch->meta.accel_first_unix_us = 0;
	batch->meta.gyro_first_unix_us = 0;
	batch->meta.flags = 0;
	batch->accel_tick_noted = false;
	batch->gyro_tick_noted = false;
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
	imu_raw_time_new_epoch(&batch->time);
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
			imu_raw_time_timestamp(&batch->time, entry.payload, entry.counter);
			*cursor = next;
			continue;
		}
		if (batch->count == IMU_RAW_MAX_WORDS) {
			rc = imu_raw_batch_flush(batch);
			if (rc) {
				return rc;
			}
		}
		if ((entry.sample.kind == LSM6DSO_FIFO_ACCEL ? batch->next_accel :
		     batch->next_gyro) == UINT64_MAX) {
			return -EOVERFLOW;
		}
		/* Every sample advances the slot tracking, not only the first. */
		imu_raw_time_advance(&batch->time, entry.counter);
		if (entry.sample.kind == LSM6DSO_FIFO_ACCEL) {
			if (batch->meta.accel_first_index == UINT64_MAX) {
				batch->meta.accel_first_index = batch->next_accel;
				batch->accel_tick_noted =
					imu_raw_time_get(&batch->time, &batch->accel_noted_tick);
				batch->accel_tick_generation = batch->time.generation;
			}
			batch->next_accel++;
		} else {
			if (batch->meta.gyro_first_index == UINT64_MAX) {
				batch->meta.gyro_first_index = batch->next_gyro;
				batch->gyro_tick_noted =
					imu_raw_time_get(&batch->time, &batch->gyro_noted_tick);
				batch->gyro_tick_generation = batch->time.generation;
			}
			batch->next_gyro++;
		}
		imu_raw_stage_put(batch->payload, batch->count++, &entry.sample);
		batch->accel_count += entry.sample.kind == LSM6DSO_FIFO_ACCEL;
		*cursor = next;
		if (batch->count == IMU_RAW_MAX_WORDS) {
			rc = imu_raw_batch_flush(batch);
			if (rc) {
				return rc;
			}
		}
	}
}
