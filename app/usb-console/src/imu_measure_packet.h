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
