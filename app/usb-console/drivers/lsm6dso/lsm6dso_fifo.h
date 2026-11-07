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

/* Decode a single uncompressed word. Returns -ENOTSUP for other tags. */
int lsm6dso_fifo_decode_word(const uint8_t word[LSM6DSO_FIFO_WORD_SIZE],
			     struct lsm6dso_fifo_sample *sample);

/* Iterate over supported words, skipping other tags. Cursor is a byte offset.
 * Returns 1 for a sample, 0 at end, or a negative errno for invalid input.
 */
int lsm6dso_fifo_next(const uint8_t *bytes, size_t length, size_t *cursor,
		      struct lsm6dso_fifo_sample *sample);

#endif
