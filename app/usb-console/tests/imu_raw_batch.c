/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "imu_raw_batch.h"

static uint8_t received[IMU_RAW_MAX_SIZE];
static size_t received_len;
static int sink_result;
static int sink(void *context, const uint8_t *payload, size_t length)
{
	(void)context;
	if (sink_result) {
		return sink_result;
	}
	memcpy(received, payload, length);
	received_len = length;
	return 0;
}
static bool clock_ready = true;
static bool test_clock(void *context, uint64_t tick, uint64_t *unix_us)
{
	(void)context;
	*unix_us = 1000000 + tick * 25;
	return clock_ready;
}
static uint64_t le64(const uint8_t *p)
{
	uint64_t n = 0;
	for (size_t i = 0; i < 8; i++) {
		n |= (uint64_t)p[i] << (8 * i);
	}
	return n;
}

int main(void)
{
	static struct imu_raw_batch b;
	static uint8_t words[LSM6DSO_FIFO_MAX_WORDS * LSM6DSO_FIFO_WORD_SIZE];
	const uint8_t session[16] = {0x42};
	size_t cursor = 0;

	for (size_t i = 0; i < LSM6DSO_FIFO_MAX_WORDS; i++) {
		words[i * 7] = i % 2 ? 0x10 : 0x08;
		words[i * 7 + 1] = (uint8_t)i;
	}
	assert(imu_raw_batch_init(&b, session, 3, sink, NULL) == 0);
	sink_result = -EAGAIN;
	assert(imu_raw_batch_consume(&b, words, sizeof(words), &cursor) == 0);
	assert(cursor == sizeof(words) && b.next_accel == 256 && b.next_gyro == 256);
	assert(b.next_batch == 1 && b.dropped_batches == 1 && b.dropped_accel == 192 &&
	       b.dropped_gyro == 192 && b.count == 128);
	sink_result = 0;
	assert(imu_raw_batch_flush(&b) == 0);
	assert(b.delivered_batches == 1 && b.next_batch == 2 && received_len == 880);
	assert(!memcmp(received, "IMU1", 4) && received[8] == 0x42 && received[68] == 3);
	assert(le64(received + 24) == 1 && le64(received + 32) == 192 &&
	       le64(received + 40) == 192 && received[5] == 0);
	assert(received[96] == 0xaa && received[112] == 128 && received[118] == 129);
	assert(imu_raw_batch_flush(&b) == 0 && b.next_batch == 2);
	cursor = 0;
	assert(imu_raw_batch_consume(&b, words, 7, &cursor) == 0);
	sink_result = -EIO;
	assert(imu_raw_batch_flush(&b) == -EIO);
	assert(b.next_batch == 2 && b.count == 1 && b.delivered_batches == 1);
	sink_result = 0;
	assert(imu_raw_batch_flush(&b) == 0);
	assert(le64(received + 24) == 2 && le64(received + 32) == UINT64_MAX &&
	       le64(received + 40) == 256);
	const uint8_t extra[] = {0x20, 1, 2, 3, 4, 0, 0, 0x18, 0, 0, 0, 0, 0, 0};
	cursor = 0;
	assert(imu_raw_batch_consume(&b, extra, sizeof(extra), &cursor) == -EPROTO);
	assert(cursor == 7 && b.timestamp_words == 1);
	assert(imu_raw_batch_consume(&b, extra, sizeof(extra) - 1, &cursor) == -EINVAL);
	cursor = 0;
	b.next_accel = UINT64_MAX;
	assert(imu_raw_batch_consume(&b, words + 7, 7, &cursor) == -EOVERFLOW);
	assert(cursor == 0);
	assert(imu_raw_batch_init(&b, NULL, 0, sink, NULL) == -EINVAL);

	/* An epoch change flushes first, then keeps numbering in the new epoch. */
	assert(imu_raw_batch_init(&b, session, 7, sink, NULL) == 0);
	cursor = 0;
	assert(imu_raw_batch_consume(&b, words, 14, &cursor) == 0 && b.count == 2);
	assert(imu_raw_batch_new_epoch(&b) == 0 && b.count == 0 && b.next_batch == 1);
	assert(received[68] == 7 && le64(received + 24) == 0 && le64(received + 32) == 0);
	cursor = 0;
	assert(imu_raw_batch_consume(&b, words, 14, &cursor) == 0);
	assert(imu_raw_batch_flush(&b) == 0);
	assert(received[68] == 8 && le64(received + 24) == 1 && le64(received + 32) == 1 &&
	       le64(received + 40) == 1);
	/* Nothing pending: only the epoch advances. A failed flush keeps it. */
	assert(imu_raw_batch_new_epoch(&b) == 0 && b.meta.epoch_id == 9 && b.next_batch == 2);
	cursor = 0;
	assert(imu_raw_batch_consume(&b, words, 7, &cursor) == 0);
	sink_result = -EIO;
	assert(imu_raw_batch_new_epoch(&b) == -EIO && b.meta.epoch_id == 9 && b.count == 1);
	sink_result = 0;
	b.meta.epoch_id = UINT32_MAX;
	assert(imu_raw_batch_new_epoch(&b) == -EOVERFLOW && b.count == 1);
	assert(imu_raw_batch_new_epoch(NULL) == -EINVAL);

	/* Sensor ticks: the first sample of each kind carries its slot's tick. */
	static uint8_t timed[3 * 17 * 7];
	size_t n = 0;

	for (unsigned int slot = 0; slot < 24; slot++) {
		uint8_t counter = (uint8_t)((slot & 3U) << 1);

		if (slot % 8 == 0) {
			uint32_t raw = 5000 + slot * 6;

			timed[n] = 0x20 | counter;
			timed[n + 1] = (uint8_t)raw;
			timed[n + 2] = (uint8_t)(raw >> 8);
			n += 7;
		}
		timed[n] = 0x08 | counter;
		n += 7;
		timed[n] = 0x10 | counter;
		n += 7;
	}
	assert(n == sizeof(timed));
	assert(imu_raw_batch_init(&b, session, 0, sink, NULL) == 0);
	cursor = 0;
	/* Timestamp, one full slot and the gyro of the next slot. */
	assert(imu_raw_batch_consume(&b, timed, 4 * 7, &cursor) == 0);
	assert(imu_raw_batch_flush(&b) == 0);
	assert(received[5] == 3 && le64(received + 48) == 5000 && le64(received + 56) == 5000);
	/* The period is not known yet: slot 1 has no tick, so the accel of this
	 * batch is untimed. Its gyro is absent from the batch start (slot 1's gyro
	 * was sent above), so the first gyro here is slot 2's. */
	assert(imu_raw_batch_consume(&b, timed, 6 * 7, &cursor) == 0);
	assert(imu_raw_batch_flush(&b) == 0);
	assert(received[5] == 0 && le64(received + 48) == 0 && le64(received + 56) == 0 &&
	       le64(received + 32) == 1 && le64(received + 40) == 2);
	/* After the second timestamp (slot 8) later slots are extrapolated. */
	assert(imu_raw_batch_consume(&b, timed, (17 + 1 + 2 * 2 + 1) * 7, &cursor) == 0);
	assert(imu_raw_batch_flush(&b) == 0);
	assert(imu_raw_batch_consume(&b, timed, (17 + 1 + 2 * 4) * 7, &cursor) == 0);
	assert(imu_raw_batch_flush(&b) == 0);
	/* Batch: accel of slot 10 (index 10), gyro of slot 11 (index 11). */
	assert(received[5] == 3 && le64(received + 32) == 10 && le64(received + 40) == 11 &&
	       le64(received + 48) == 5000 + 10 * 6 && le64(received + 56) == 5000 + 11 * 6);
	/* A new epoch forgets the slot position: no tick until the next timestamp. */
	assert(imu_raw_batch_new_epoch(&b) == 0);
	assert(imu_raw_batch_consume(&b, timed, (17 + 1 + 2 * 5) * 7, &cursor) == 0);
	assert(imu_raw_batch_flush(&b) == 0 && received[5] == 0 && received[68] == 1);
	assert(imu_raw_batch_consume(&b, timed, sizeof(timed), &cursor) == 0);
	assert(imu_raw_batch_flush(&b) == 0 && b.time.anchors == 3);

	/* With a clock, timed first samples also carry Unix time; a clock that
	 * has no answer leaves it invalid and zero. */
	assert(imu_raw_batch_init(&b, session, 0, sink, NULL) == 0);
	b.clock = test_clock;
	cursor = 0;
	assert(imu_raw_batch_consume(&b, timed, 4 * 7, &cursor) == 0);
	assert(imu_raw_batch_flush(&b) == 0);
	assert(received[5] == 15 && le64(received + 80) == 1000000 + 5000 * 25 &&
	       le64(received + 88) == 1000000 + 5000 * 25);
	clock_ready = false;
	assert(imu_raw_batch_consume(&b, timed, (17 + 3) * 7, &cursor) == 0);
	assert(imu_raw_batch_flush(&b) == 0);
	assert(imu_raw_batch_consume(&b, timed, (17 + 5) * 7, &cursor) == 0);
	assert(imu_raw_batch_flush(&b) == 0);
	assert(received[5] == 3 && le64(received + 80) == 0 && le64(received + 88) == 0);
	return 0;
}
