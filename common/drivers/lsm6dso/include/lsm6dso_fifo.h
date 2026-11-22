/* SPDX-License-Identifier: Apache-2.0 */
/* The LSM6DSO's FIFO: the format of its words and their parser
 * (lsm6dso_fifo.c), and the counters the driver keeps of reading it.  No OS:
 * the driver's core and its users on any OS read the words with these.
 */
#ifndef LSM6DSO_FIFO_H_
#define LSM6DSO_FIFO_H_

#include <stdbool.h>
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
	uint8_t payload[6]; /* Original bytes. */
	struct lsm6dso_fifo_sample sample; /* Valid only for SAMPLE. */
	uint32_t timestamp; /* Sensor timestamp counter. Valid only for TIMESTAMP. */
};

/* Returns every FIFO word in order, including timestamp and unknown tags.
 * Returns 1 for a word, 0 at end, or -EINVAL for invalid input.
 * A timestamp word yields the counter value it carries; this does not assign
 * sample times or validate tag parity or sensor FIFO integrity.
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

struct lsm6dso_fifo_stats {
	uint32_t generation;
	uint64_t words_read;
	uint64_t transfer_time_ns;
	uint64_t max_transfer_time_ns;
	uint32_t transfers;
	uint32_t i2c_errors;
	uint32_t buffer_too_small;
	/* Observations, not a count of lost samples or distinct overruns. */
	uint32_t overrun_observations;
	int stop_error;
	uint16_t accel_hz;
	uint16_t gyro_hz;
	/* Sensitivity in effect when the FIFO was started. */
	uint32_t accel_ug_per_lsb;
	uint32_t gyro_udps_per_lsb;
	/* INTERNAL_FREQ_FINE read when the FIFO was started; 0 if unreadable. */
	int8_t freq_fine;
	bool active;
	bool fault;
};

#endif /* LSM6DSO_FIFO_H_ */
