/* SPDX-License-Identifier: Apache-2.0 */

#include "lsm6dso_fifo.h"

#include <errno.h>
#include <string.h>

int lsm6dso_fifo_decode_word(const uint8_t word[LSM6DSO_FIFO_WORD_SIZE],
			     struct lsm6dso_fifo_sample *sample)
{
	uint8_t tag;

	if (word == NULL || sample == NULL) {
		return -EINVAL;
	}

	tag = word[0] >> 3;
	if (tag != LSM6DSO_FIFO_GYRO && tag != LSM6DSO_FIFO_ACCEL) {
		return -ENOTSUP;
	}

	sample->kind = (enum lsm6dso_fifo_kind)tag;
	for (size_t axis = 0; axis < 3; axis++) {
		uint16_t raw = (uint16_t)word[1 + axis * 2] |
			       ((uint16_t)word[2 + axis * 2] << 8);

		/* Avoid relying on implementation-defined unsigned-to-signed conversion. */
		sample->xyz[axis] = raw < 0x8000U ? (int16_t)raw :
				    (int16_t)((int32_t)raw - 65536);
	}
	return 0;
}

int lsm6dso_fifo_next(const uint8_t *bytes, size_t length, size_t *cursor,
		      struct lsm6dso_fifo_sample *sample)
{
	if (cursor == NULL || sample == NULL || (bytes == NULL && length != 0) ||
	    length > LSM6DSO_FIFO_MAX_WORDS * LSM6DSO_FIFO_WORD_SIZE ||
	    length % LSM6DSO_FIFO_WORD_SIZE != 0 || *cursor > length ||
	    *cursor % LSM6DSO_FIFO_WORD_SIZE != 0) {
		return -EINVAL;
	}

	while (*cursor < length) {
		int ret = lsm6dso_fifo_decode_word(bytes + *cursor, sample);

		*cursor += LSM6DSO_FIFO_WORD_SIZE;
		if (ret == 0) {
			return 1;
		}
		if (ret != -ENOTSUP) {
			return ret;
		}
	}
	return 0;
}

int lsm6dso_fifo_next_entry(const uint8_t *bytes, size_t length, size_t *cursor,
			   struct lsm6dso_fifo_entry *entry)
{
	const uint8_t *word;

	if (!cursor || !entry || (!bytes && length) ||
	    length > LSM6DSO_FIFO_MAX_WORDS * LSM6DSO_FIFO_WORD_SIZE ||
	    length % LSM6DSO_FIFO_WORD_SIZE || *cursor > length ||
	    *cursor % LSM6DSO_FIFO_WORD_SIZE) {
		return -EINVAL;
	}
	if (*cursor == length) {
		return 0;
	}
	word = bytes + *cursor;
	entry->tag = word[0] >> 3;
	entry->counter = (word[0] >> 1) & 3U;
	memcpy(entry->payload, word + 1, sizeof(entry->payload));
	if (lsm6dso_fifo_decode_word(word, &entry->sample) == 0) {
		entry->type = LSM6DSO_FIFO_ENTRY_SAMPLE;
	} else if (entry->tag == 4) { /* LSM6DSO timestamp tag */
		entry->type = LSM6DSO_FIFO_ENTRY_TIMESTAMP;
	} else {
		entry->type = LSM6DSO_FIFO_ENTRY_OTHER;
	}
	*cursor += LSM6DSO_FIFO_WORD_SIZE;
	return 1;
}
