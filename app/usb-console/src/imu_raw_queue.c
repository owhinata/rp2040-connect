/* SPDX-License-Identifier: Apache-2.0 */
#include "imu_raw_queue.h"

#include <errno.h>
#include <string.h>

static uint16_t le16(const uint8_t *p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

void imu_raw_queue_init(struct imu_raw_queue *queue)
{
	if (queue) {
		memset(queue, 0, sizeof(*queue));
	}
}

void imu_raw_queue_discard(struct imu_raw_queue *queue, const struct imu_raw_queue_item *item)
{
	if (!queue || !item) {
		return;
	}
	queue->dropped_batches++;
	queue->dropped_accel += item->accel_count;
	queue->dropped_gyro += item->gyro_count;
}

void imu_raw_queue_set_connected(struct imu_raw_queue *queue, bool connected)
{
	if (!queue) {
		return;
	}
	queue->connected = connected;
	if (!connected) {
		while (queue->count) {
			imu_raw_queue_discard(queue, &queue->items[queue->head]);
			queue->head = (queue->head + 1) % IMU_RAW_QUEUE_CAPACITY;
			queue->count--;
		}
	}
}

int imu_raw_queue_put(struct imu_raw_queue *queue, const uint8_t *payload, size_t length)
{
	struct imu_raw_queue_item *item;
	uint16_t count, bitmap, accel = 0;

	if (!queue || !payload || length < IMU_RAW_HEADER_SIZE ||
	    memcmp(payload, "IMU1", 4) || payload[4] != IMU_RAW_VERSION ||
	    le16(payload + 6) != IMU_RAW_HEADER_SIZE) {
		return -EINVAL;
	}
	count = le16(payload + 64);
	bitmap = le16(payload + 66);
	if (!count || count > IMU_RAW_MAX_WORDS || bitmap != (count + 7) / 8 ||
	    length != IMU_RAW_HEADER_SIZE + bitmap + (size_t)count * 6) {
		return -EINVAL;
	}
	for (size_t i = 0; i < count; i++) {
		accel += (payload[IMU_RAW_HEADER_SIZE + i / 8] >> (i % 8)) & 1U;
	}
	if (!queue->connected || queue->count == IMU_RAW_QUEUE_CAPACITY) {
		queue->dropped_batches++;
		queue->dropped_accel += accel;
		queue->dropped_gyro += count - accel;
		return -EAGAIN;
	}
	item = &queue->items[(queue->head + queue->count) % IMU_RAW_QUEUE_CAPACITY];
	item->length = length;
	item->accel_count = accel;
	item->gyro_count = count - accel;
	memcpy(item->payload, payload, length);
	queue->count++;
	if (queue->count > queue->high_watermark) {
		queue->high_watermark = queue->count;
	}
	return 0;
}

int imu_raw_queue_take(struct imu_raw_queue *queue, struct imu_raw_queue_item *item)
{
	if (!queue || !item) {
		return -EINVAL;
	}
	if (!queue->connected || !queue->count) {
		return -EAGAIN;
	}
	*item = queue->items[queue->head];
	queue->head = (queue->head + 1) % IMU_RAW_QUEUE_CAPACITY;
	queue->count--;
	return 0;
}
