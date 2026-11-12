/* SPDX-License-Identifier: Apache-2.0 */

#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <string.h>

#include "lsm6dso_fifo.h"
/* The layout is the driver's; a test of the driver may build a buffer by hand. */
#include "lsm6dso_fifo_packet.h"

int main(void)
{
	const uint8_t words[] = {
		0x0f, 0x00, 0x80, 0xff, 0x7f, 0xff, 0xff, /* gyro, tag counter */
		0x18, 0, 0, 0, 0, 0, 0, /* unsupported temperature tag */
		0x10, 0x34, 0x12, 0x00, 0x00, 0x01, 0x80, /* accel */
	};
	struct lsm6dso_fifo_sample sample;
	struct lsm6dso_fifo_entry entry;
	size_t cursor = 0;

	assert(lsm6dso_fifo_next(words, sizeof(words), &cursor, &sample) == 1);
	assert(sample.kind == LSM6DSO_FIFO_GYRO);
	assert(sample.xyz[0] == -32768 && sample.xyz[1] == 32767 && sample.xyz[2] == -1);
	assert(lsm6dso_fifo_next(words, sizeof(words), &cursor, &sample) == 1);
	assert(sample.kind == LSM6DSO_FIFO_ACCEL);
	assert(sample.xyz[0] == 0x1234 && sample.xyz[1] == 0 && sample.xyz[2] == -32767);
	assert(lsm6dso_fifo_next(words, sizeof(words), &cursor, &sample) == 0);
	assert(lsm6dso_fifo_next(words, sizeof(words) - 1, &cursor, &sample) == -EINVAL);
	cursor = 1;
	assert(lsm6dso_fifo_next(words, sizeof(words), &cursor, &sample) == -EINVAL);
	cursor = 0;
	assert(lsm6dso_fifo_next(NULL, 0, &cursor, &sample) == 0);
	assert(lsm6dso_fifo_next(NULL, 7, &cursor, &sample) == -EINVAL);
	assert(lsm6dso_fifo_decode_word(words + 7, &sample) == -ENOTSUP);
	cursor = 0;
	assert(lsm6dso_fifo_next_entry(words, sizeof(words), &cursor, &entry) == 1);
	assert(entry.type == LSM6DSO_FIFO_ENTRY_SAMPLE && entry.sample.xyz[0] == -32768);
	assert(entry.counter == 3);
	assert(lsm6dso_fifo_next_entry(words, sizeof(words), &cursor, &entry) == 1);
	assert(entry.type == LSM6DSO_FIFO_ENTRY_OTHER && entry.tag == 3);
	assert(lsm6dso_fifo_next_entry(words, sizeof(words), &cursor, &entry) == 1);
	assert(entry.type == LSM6DSO_FIFO_ENTRY_SAMPLE && entry.sample.kind == LSM6DSO_FIFO_ACCEL);
	assert(lsm6dso_fifo_next_entry(words, sizeof(words), &cursor, &entry) == 0);
	const uint8_t timestamp[] = {0x20, 1, 2, 3, 4, 5, 6};
	cursor = 0;
	assert(lsm6dso_fifo_next_entry(timestamp, sizeof(timestamp), &cursor, &entry) == 1);
	assert(entry.type == LSM6DSO_FIFO_ENTRY_TIMESTAMP && entry.payload[3] == 4 &&
	       entry.counter == 0);
	/* The counter is the first four bytes, little-endian; the rest is not part of it. */
	assert(entry.timestamp == 0x04030201);
	assert(lsm6dso_fifo_next_entry(timestamp, 6, &cursor, &entry) == -EINVAL);

	/* Reading the words out of a stream buffer without knowing its header. */
	union { uint64_t align; uint8_t bytes[offsetof(struct lsm6dso_fifo_packet, data) + 14]; } buffer = {0};
	struct lsm6dso_fifo_packet *packet = (void *)buffer.bytes;
	const uint8_t *found;
	size_t size;

	packet->marker = LSM6DSO_FIFO_PACKET_MARKER;
	packet->words = 2;
	memcpy(packet->data, words, 14);
	assert(lsm6dso_fifo_buffer_words(buffer.bytes, sizeof(buffer.bytes), &found, &size) == 0);
	assert(found == packet->data && size == 14 && memcmp(found, words, 14) == 0);
	/* Truncated, not a FIFO buffer, an impossible word count, bad arguments. */
	assert(lsm6dso_fifo_buffer_words(buffer.bytes, sizeof(buffer.bytes) - 1, &found, &size) ==
	       -EBADMSG);
	assert(lsm6dso_fifo_buffer_words(buffer.bytes, 8, &found, &size) == -EBADMSG);
	packet->marker = 3;
	assert(lsm6dso_fifo_buffer_words(buffer.bytes, sizeof(buffer.bytes), &found, &size) ==
	       -EBADMSG);
	packet->marker = LSM6DSO_FIFO_PACKET_MARKER;
	packet->words = LSM6DSO_FIFO_MAX_WORDS + 1;
	assert(lsm6dso_fifo_buffer_words(buffer.bytes, sizeof(buffer.bytes), &found, &size) ==
	       -EBADMSG);
	assert(lsm6dso_fifo_buffer_words(NULL, 64, &found, &size) == -EBADMSG);
	packet->words = 0;
	assert(lsm6dso_fifo_buffer_words(buffer.bytes, sizeof(buffer.bytes), &found, &size) == 0 &&
	       size == 0);
	return 0;
}
