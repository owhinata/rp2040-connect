/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <string.h>

#include "imu_measure_packet.h"
#include "lsm6dso_fifo_decoder.h"

int main(void)
{
	union {
		uint64_t align;
		uint8_t bytes[sizeof(struct lsm6dso_fifo_packet) + 4 * 7];
	} storage = {0};
	struct lsm6dso_fifo_packet *packet = (void *)storage.bytes;
	struct imu_measure_counts counts = {0};
	const uint8_t words[] = {
		0x10, 1, 0, 0, 0, 0, 0,
		0x08, 2, 0, 0, 0, 0, 0,
		0x10, 3, 0, 0, 0, 0, 0,
		0x18, 4, 0, 0, 0, 0, 0,
	};
	const size_t needed = offsetof(struct lsm6dso_fifo_packet, data) + sizeof(words);

	packet->marker = LSM6DSO_FIFO_PACKET_MARKER;
	packet->words = 4;
	memcpy(packet->data, words, sizeof(words));
	assert(imu_measure_count_packet(storage.bytes, needed - 1, &counts) == -EINVAL);
	assert(counts.batches == 0 && counts.accel_samples == 0);
	assert(imu_measure_count_packet(storage.bytes, needed, &counts) == 0);
	assert(counts.batches == 1 && counts.accel_samples == 2 && counts.gyro_samples == 1 &&
	       counts.other_words == 1);
	assert(imu_measure_count_packet(storage.bytes, needed, &counts) == 0);
	assert(counts.batches == 2 && counts.accel_samples == 4 && counts.gyro_samples == 2 &&
	       counts.other_words == 2);
	assert(imu_measure_check_accounting(&counts, 8, 2, 1) == IMU_MEASURE_ACCOUNTING_MATCH);
	assert(imu_measure_check_accounting(&counts, 9, 2, 1) == IMU_MEASURE_ACCOUNTING_MISMATCH);
	assert(imu_measure_check_accounting(&counts, 8, 3, 1) == IMU_MEASURE_ACCOUNTING_MISMATCH);
	assert(imu_measure_check_accounting(&counts, 8, 2, 0) == IMU_MEASURE_ACCOUNTING_INCOMPLETE);
	counts.accel_samples = UINT64_MAX;
	assert(imu_measure_check_accounting(&counts, 0, 2, 1) == IMU_MEASURE_ACCOUNTING_MISMATCH);
	packet->marker = 0;
	assert(imu_measure_count_packet(storage.bytes, needed, &counts) == -EINVAL);
	packet->marker = LSM6DSO_FIFO_PACKET_MARKER;
	packet->words = 513;
	assert(imu_measure_count_packet(storage.bytes, sizeof(storage.bytes), &counts) == -EINVAL);
	assert(imu_measure_count_packet(NULL, 0, &counts) == -EINVAL);
	return 0;
}
