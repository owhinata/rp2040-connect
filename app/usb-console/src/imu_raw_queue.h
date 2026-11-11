/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_RAW_QUEUE_H_
#define IMU_RAW_QUEUE_H_

#include "imu_raw_packet.h"
#include <stdbool.h>

#define IMU_RAW_QUEUE_CAPACITY 16U
struct imu_raw_queue_item {
	size_t length;
	uint16_t accel_count, gyro_count;
	uint8_t payload[IMU_RAW_MAX_SIZE];
};
struct imu_raw_queue {
	struct imu_raw_queue_item items[IMU_RAW_QUEUE_CAPACITY];
	size_t head, count, high_watermark;
	bool connected;
	uint64_t dropped_batches, dropped_accel, dropped_gyro;
};

/* Calls on the same queue must be serialized by the caller. Do not hold the
 * lock during MQTT operations. One consumer owns each item after take(). */
void imu_raw_queue_init(struct imu_raw_queue *queue);
void imu_raw_queue_set_connected(struct imu_raw_queue *queue, bool connected);
/* A valid but rejected batch is counted as dropped and returns -EAGAIN. */
int imu_raw_queue_put(struct imu_raw_queue *queue, const uint8_t *payload, size_t length);
int imu_raw_queue_take(struct imu_raw_queue *queue, struct imu_raw_queue_item *item);
/* Call exactly once for a taken item whose publish failed or was aborted. */
void imu_raw_queue_discard(struct imu_raw_queue *queue, const struct imu_raw_queue_item *item);
#endif
