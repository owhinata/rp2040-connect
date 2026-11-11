/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_RAW_PACKET_H_
#define IMU_RAW_PACKET_H_

#include <stddef.h>
#include <stdint.h>
#include "../drivers/lsm6dso/lsm6dso_fifo.h"

/* Experimental implementation of design/imu-raw-mqtt-design-ja.md §6. */
#define IMU_RAW_HEADER_SIZE 80U
#define IMU_RAW_MAX_WORDS 384U
#define IMU_RAW_MAX_SIZE (IMU_RAW_HEADER_SIZE + (IMU_RAW_MAX_WORDS + 7U) / 8U + 6U * IMU_RAW_MAX_WORDS)
#define IMU_RAW_ACCEL_TIME_VALID 1U
#define IMU_RAW_GYRO_TIME_VALID 2U

struct imu_raw_metadata {
	uint8_t session_id[16];
	uint64_t batch_seq;
	uint64_t accel_first_index, gyro_first_index;
	uint64_t accel_first_tick, gyro_first_tick;
	uint32_t epoch_id, time_map_id;
	uint8_t flags;
};

/* Encodes one complete payload. The caller owns numbering, timing, and transport.
 * An absent sensor must have index UINT64_MAX and tick 0; invalid time has tick 0.
 * On error, *written is set to zero and no output bytes are changed.
 */
int imu_raw_encode(uint8_t *out, size_t capacity, size_t *written,
		   const struct imu_raw_metadata *meta,
		   const struct lsm6dso_fifo_sample *samples, size_t count);
#endif
