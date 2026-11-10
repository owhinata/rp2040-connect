/* SPDX-License-Identifier: Apache-2.0 */
#ifndef IMU_MEASURE_PACKET_H_
#define IMU_MEASURE_PACKET_H_

#include <stddef.h>
#include <stdint.h>

struct imu_measure_counts {
	uint64_t batches;
	uint64_t accel_samples;
	uint64_t gyro_samples;
	uint64_t other_words;
#if defined(CONFIG_NINA_SEND_PROFILE)
	uint32_t other_tags[32];
	uint8_t first_other[7];
	uint32_t first_other_index, first_other_packet_words;
	uint32_t batches_with_other, batches_valid_after_other;
#endif
};

enum imu_measure_accounting {
	IMU_MEASURE_ACCOUNTING_INCOMPLETE,
	IMU_MEASURE_ACCOUNTING_MATCH,
	IMU_MEASURE_ACCOUNTING_MISMATCH,
};

int imu_measure_count_packet(const uint8_t *buffer, size_t length,
			     struct imu_measure_counts *counts);
enum imu_measure_accounting imu_measure_check_accounting(const struct imu_measure_counts *counts,
							 uint64_t fifo_words, uint64_t fifo_transfers,
							 int complete);
#endif
