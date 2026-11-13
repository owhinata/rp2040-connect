/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <errno.h>
#include <string.h>
#include "imu_raw_queue.h"
#include "imu_raw_batch.h"

static int queue_sink(void *context, const uint8_t *payload, size_t length)
{
	return imu_raw_queue_put(context, payload, length);
}

int main(void)
{
	static struct imu_raw_queue q;
	static struct imu_raw_queue_item item;
	uint8_t payload[IMU_RAW_MAX_SIZE];
	size_t length;
	struct imu_raw_metadata m = {.accel_first_index = 10, .gyro_first_index = 20};
	const struct lsm6dso_fifo_sample samples[] = {
		{LSM6DSO_FIFO_ACCEL, {1, 2, 3}}, {LSM6DSO_FIFO_GYRO, {4, 5, 6}},
		{LSM6DSO_FIFO_ACCEL, {7, 8, 9}},
	};
	imu_raw_queue_init(&q);
	assert(imu_raw_encode(payload, sizeof(payload), &length, &m, samples, 3) == 0);
	assert(imu_raw_queue_put(&q, payload, length) == -EAGAIN);
	assert(q.dropped_batches == 1 && q.dropped_accel == 2 && q.dropped_gyro == 1);
	imu_raw_queue_set_connected(&q, true);
	for (unsigned int i = 0; i < IMU_RAW_QUEUE_CAPACITY; i++) {
		payload[24] = i;
		assert(imu_raw_queue_put(&q, payload, length) == 0);
	}
	assert(q.high_watermark == IMU_RAW_QUEUE_CAPACITY &&
	       imu_raw_queue_put(&q, payload, length) == -EAGAIN);
	assert(imu_raw_queue_take(&q, &item) == 0 && item.payload[24] == 0);
	payload[24] = IMU_RAW_QUEUE_CAPACITY;
	assert(imu_raw_queue_put(&q, payload, length) == 0);
	for (unsigned int i = 1; i <= IMU_RAW_QUEUE_CAPACITY; i++) {
		assert(imu_raw_queue_take(&q, &item) == 0 && item.payload[24] == i);
	}
	assert(imu_raw_queue_take(&q, &item) == -EAGAIN);
	for (int i = 0; i < 3; i++) assert(imu_raw_queue_put(&q, payload, length) == 0);
	assert(imu_raw_queue_take(&q, &item) == 0);
	imu_raw_queue_set_connected(&q, false);
	assert(q.count == 0 && q.dropped_batches == 4);
	imu_raw_queue_discard(&q, &item);
	assert(q.dropped_batches == 5 && q.dropped_accel == 10 && q.dropped_gyro == 5);
	imu_raw_queue_set_connected(&q, true);
	assert(imu_raw_queue_take(&q, &item) == -EAGAIN);
	assert(imu_raw_queue_put(&q, payload, length - 1) == -EINVAL);
	assert(q.dropped_batches == 5);

	/* The producer numbers batches before the queue decides to drop them. */
	static struct imu_raw_batch batch;
	uint8_t session[16] = {1};
	uint8_t fifo[7] = {0x10, 0, 0, 0, 0, 0, 0};
	size_t cursor = 0;
	imu_raw_queue_init(&q);
	assert(imu_raw_batch_init(&batch, session, 0, queue_sink, &q) == 0);
	assert(imu_raw_batch_consume(&batch, fifo, sizeof(fifo), &cursor) == 0);
	assert(imu_raw_batch_flush(&batch) == 0);
	assert(batch.next_batch == 1 && batch.dropped_accel == 1 && q.dropped_accel == 1);
	imu_raw_queue_set_connected(&q, true);
	cursor = 0;
	assert(imu_raw_batch_consume(&batch, fifo, sizeof(fifo), &cursor) == 0);
	assert(imu_raw_batch_flush(&batch) == 0);
	assert(imu_raw_queue_take(&q, &item) == 0);
	assert(item.payload[24] == 1 && item.payload[32] == 1);
	return 0;
}
