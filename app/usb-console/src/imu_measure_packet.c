/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stddef.h>
#include "imu_measure_packet.h"
#include "../drivers/lsm6dso/lsm6dso_fifo.h"
#include "../drivers/lsm6dso/lsm6dso_fifo_decoder.h"

int imu_measure_count_packet(const uint8_t *buffer, size_t length,
			     struct imu_measure_counts *counts)
{
	const struct lsm6dso_fifo_packet *packet = (const void *)buffer;
	struct lsm6dso_fifo_sample sample;
	size_t cursor = 0;
	uint64_t accel = 0, gyro = 0, other = 0;
	int ret;

	if (!buffer || !counts || length < offsetof(struct lsm6dso_fifo_packet, data) ||
	    packet->marker != LSM6DSO_FIFO_PACKET_MARKER ||
	    packet->words > LSM6DSO_FIFO_MAX_WORDS ||
	    length - offsetof(struct lsm6dso_fifo_packet, data) <
		(size_t)packet->words * LSM6DSO_FIFO_WORD_SIZE) {
		return -EINVAL;
	}
	/* Count unknown tags too; silently discarding them could hide a format
	 * mismatch or a sensor configuration change during a measurement.
	 */
	for (size_t i = 0; i < packet->words; i++) {
		ret = lsm6dso_fifo_decode_word(packet->data + cursor, &sample);
		if (ret == 0) {
			accel += sample.kind == LSM6DSO_FIFO_ACCEL;
			gyro += sample.kind == LSM6DSO_FIFO_GYRO;
		} else if (ret == -ENOTSUP) {
			other++;
		} else {
			return ret;
		}
		cursor += LSM6DSO_FIFO_WORD_SIZE;
	}
	counts->batches++;
	counts->accel_samples += accel;
	counts->gyro_samples += gyro;
	counts->other_words += other;
	return 0;
}

enum imu_measure_accounting imu_measure_check_accounting(const struct imu_measure_counts *counts,
							 uint64_t fifo_words, uint64_t fifo_transfers,
							 int complete)
{
	uint64_t words;

	if (!counts || !complete) {
		return IMU_MEASURE_ACCOUNTING_INCOMPLETE;
	}
	if (counts->accel_samples > UINT64_MAX - counts->gyro_samples) {
		return IMU_MEASURE_ACCOUNTING_MISMATCH;
	}
	words = counts->accel_samples + counts->gyro_samples;
	if (words > UINT64_MAX - counts->other_words) {
		return IMU_MEASURE_ACCOUNTING_MISMATCH;
	}
	words += counts->other_words;
	return words == fifo_words && counts->batches == fifo_transfers ?
		IMU_MEASURE_ACCOUNTING_MATCH : IMU_MEASURE_ACCOUNTING_MISMATCH;
}
