/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_RAW_BATCH_H_
#define IMU_RAW_BATCH_H_

#include "imu_raw_packet.h"

/* Runs in the FIFO consumer. The sink must not wait or retain payload; it
 * returns 0 after copying or -EAGAIN if it cannot accept the batch. */
typedef int (*imu_raw_batch_sink)(void *context, const uint8_t *payload, size_t length);

struct imu_raw_batch {
	struct imu_raw_metadata meta;
	struct lsm6dso_fifo_sample samples[IMU_RAW_MAX_WORDS];
	uint8_t payload[IMU_RAW_MAX_SIZE];
	uint64_t next_accel, next_gyro, next_batch;
	uint64_t delivered_batches, dropped_batches, dropped_accel, dropped_gyro;
	uint64_t timestamp_words;
	size_t count;
	imu_raw_batch_sink sink;
	void *sink_context;
};

/* The caller supplies a unique session ID; no random ID is generated here. */
int imu_raw_batch_init(struct imu_raw_batch *batch, const uint8_t session_id[16],
		       uint32_t epoch_id, imu_raw_batch_sink sink, void *context);
/* cursor is a byte offset into a FIFO word array. Each valid sample is
 * numbered before the sink is called. Timestamps are counted, not mapped.
 * Unsupported tags return -EPROTO without consuming the offending word.
 */
int imu_raw_batch_consume(struct imu_raw_batch *batch, const uint8_t *words,
			  size_t length, size_t *cursor);
/* Flush the incomplete batch at a deadline, stop, or before an epoch change. */
int imu_raw_batch_flush(struct imu_raw_batch *batch);
/* Call after an unknown number of samples was lost (FIFO overrun, restart,
 * unparsable word). Flushes what was collected and starts the next epoch.
 * Batch and sample numbers keep counting so they stay unique in the session;
 * consecutive numbers on both sides of an epoch change do not mean that the
 * samples are contiguous. On a flush error the epoch is left unchanged.
 */
int imu_raw_batch_new_epoch(struct imu_raw_batch *batch);
#endif
