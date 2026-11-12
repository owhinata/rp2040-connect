/* SPDX-License-Identifier: Apache-2.0 */
#ifndef LSM6DSO_FIFO_PACKET_H_
#define LSM6DSO_FIFO_PACKET_H_

#include <stdint.h>
#include "lsm6dso_cal.h"

/* Layout of a FIFO stream buffer. Private to the driver: only the code that
 * fills a buffer, the decoder and lsm6dso_fifo_buffer_words() may depend on
 * it. Everything else reads a buffer through those.
 */
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
	/* Conversions in effect when the batch was read. */
	struct lsm6dso_cal accel_cal;
	struct lsm6dso_cal gyro_cal;
	uint16_t words;
	uint8_t watermark;
	uint8_t data[];
};

#endif
