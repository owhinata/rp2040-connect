/* SPDX-License-Identifier: Apache-2.0 */
#ifndef LSM6DSO_FIFO_DECODER_H_
#define LSM6DSO_FIFO_DECODER_H_

#include <zephyr/drivers/sensor.h>

/* Internal FIFO payload. Times are estimates of the first sample in each
 * channel's batch, in the sensor clock domain, not the host read time.
 * The producer must derive them before submitting a batch to the decoder.
 */
/* The marker shares the generic header's num_channels offset. Generic reads
 * cannot contain this many channels, which lets the combined decoder select
 * the format without relying on timestamps or random bytes.
 */
#define LSM6DSO_FIFO_PACKET_MARKER UINT32_MAX
struct __attribute__((packed, aligned(4))) lsm6dso_fifo_packet {
	uint64_t accel_first_timestamp_ns;
	uint32_t marker;
	uint64_t gyro_first_timestamp_ns;
	uint32_t accel_period_ns;
	uint32_t gyro_period_ns;
	uint32_t accel_gain_ug;
	uint32_t gyro_gain_udps;
	uint16_t words;
	uint8_t watermark;
	uint8_t data[];
};

extern const struct sensor_decoder_api lsm6dso_fifo_decoder;
extern const struct sensor_decoder_api lsm6dso_combined_decoder;

#endif
