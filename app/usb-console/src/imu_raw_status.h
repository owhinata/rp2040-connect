/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_RAW_STATUS_H_
#define IMU_RAW_STATUS_H_

#include <stddef.h>
#include <stdint.h>

/* Experimental JSON for imu/<device_id>/status and imu/<device_id>/info
 * (design/imu-raw-mqtt-design-ja.md §5). Time conversion is not implemented;
 * both messages say so explicitly instead of omitting it. */
enum imu_raw_state {
	IMU_RAW_STATE_IDLE,      /* session configured, no measurement yet */
	IMU_RAW_STATE_MEASURING,
	IMU_RAW_STATE_STOPPED,
};

struct imu_raw_status {
	/* Filled by the acquisition side. Numbers advance whether or not a
	 * batch was sent, so they are the reference for detecting loss. */
	uint8_t session_id[16];
	enum imu_raw_state state;
	uint32_t epoch_id;
	uint64_t uptime_ms;
	uint64_t accel_acquired, gyro_acquired, next_batch, timestamp_words;
	uint32_t fifo_overrun_observations, i2c_errors;
	int raw_error, measure_error;
	/* Filled by the transport. */
	uint64_t dropped_batches, dropped_accel, dropped_gyro, published;
	uint32_t connects, failures, queue_high_watermark;
};

struct imu_raw_info {
	uint8_t session_id[16];
	uint32_t epoch_id;
	uint16_t accel_odr_hz, gyro_odr_hz; /* configured, not measured */
	uint32_t accel_ug_per_lsb, gyro_udps_per_lsb;
	uint16_t fifo_watermark;
	uint8_t timestamp_decimation;
};

/* Both return the JSON length (without a terminator being counted) or
 * -ENOSPC/-EINVAL. The output is always NUL-terminated on success. */
int imu_raw_status_encode(char *out, size_t capacity, const struct imu_raw_status *status);
int imu_raw_info_encode(char *out, size_t capacity, const struct imu_raw_info *info,
			const char *device_id);
#endif
