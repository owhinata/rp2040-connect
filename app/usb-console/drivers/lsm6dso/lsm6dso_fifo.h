/* SPDX-License-Identifier: Apache-2.0 */

#ifndef LSM6DSO_FIFO_H_
#define LSM6DSO_FIFO_H_

#include <stddef.h>
#include <stdint.h>

#define LSM6DSO_FIFO_WORD_SIZE 7U
#define LSM6DSO_FIFO_MAX_WORDS 512U

enum lsm6dso_fifo_kind {
	LSM6DSO_FIFO_GYRO = 1,
	LSM6DSO_FIFO_ACCEL = 2,
};

struct lsm6dso_fifo_sample {
	enum lsm6dso_fifo_kind kind;
	int16_t xyz[3];
};

enum lsm6dso_fifo_entry_type {
	LSM6DSO_FIFO_ENTRY_SAMPLE,
	LSM6DSO_FIFO_ENTRY_TIMESTAMP,
	LSM6DSO_FIFO_ENTRY_OTHER,
};

struct lsm6dso_fifo_entry {
	enum lsm6dso_fifo_entry_type type;
	uint8_t tag;
	uint8_t counter; /* 2-bit tag counter shared by the words of one sampling instant. */
	uint8_t payload[6]; /* Original bytes, including timestamp payload. */
	struct lsm6dso_fifo_sample sample; /* Valid only for SAMPLE. */
};

/* Returns every FIFO word in order, including timestamp and unknown tags.
 * Returns 1 for a word, 0 at end, or -EINVAL for invalid input.
 * This only identifies timestamp words; it does not assign sample times or
 * validate tag parity or sensor FIFO integrity.
 */
int lsm6dso_fifo_next_entry(const uint8_t *bytes, size_t length, size_t *cursor,
			   struct lsm6dso_fifo_entry *entry);

/* Decode a single uncompressed word. Returns -ENOTSUP for other tags. */
int lsm6dso_fifo_decode_word(const uint8_t word[LSM6DSO_FIFO_WORD_SIZE],
			     struct lsm6dso_fifo_sample *sample);

/* Iterate over supported words, skipping other tags. Cursor is a byte offset.
 * Returns 1 for a sample, 0 at end, or a negative errno for invalid input.
 */
int lsm6dso_fifo_next(const uint8_t *bytes, size_t length, size_t *cursor,
		      struct lsm6dso_fifo_sample *sample);

#endif
