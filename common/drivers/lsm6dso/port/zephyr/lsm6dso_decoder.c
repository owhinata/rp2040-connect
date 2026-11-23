/* SPDX-License-Identifier: Apache-2.0 */

#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>
#include "lsm6dso_cal.h"
#include "lsm6dso_decoder.h"

/* The buffers of the FIFO stream (lsm6dso_fifo_packet.h); the words in them
 * are parsed by lsm6dso_fifo.c.
 */

int lsm6dso_fifo_buffer_words(const uint8_t *buffer, size_t length, const uint8_t **words,
			      size_t *size)
{
	const struct lsm6dso_fifo_packet *packet = (const void *)buffer;
	const size_t header = offsetof(struct lsm6dso_fifo_packet, data);

	if (!buffer || !words || !size || length < header ||
	    packet->marker != LSM6DSO_FIFO_PACKET_MARKER ||
	    packet->words > LSM6DSO_FIFO_MAX_WORDS ||
	    length - header < (size_t)packet->words * LSM6DSO_FIFO_WORD_SIZE) {
		return -EBADMSG;
	}
	*words = packet->data;
	*size = (size_t)packet->words * LSM6DSO_FIFO_WORD_SIZE;
	return 0;
}

/* The decoders of the sensor API. */

/* Q31 with shift 9 covers the maximum LSM6DSO32 acceleration range. */
#define FIFO_Q31_SHIFT LSM6DSO_CAL_Q31_SHIFT

_Static_assert(offsetof(struct lsm6dso_fifo_packet, marker) == sizeof(uint64_t),
	       "FIFO marker must occupy the generic channel count offset");

static int channel_kind(struct sensor_chan_spec channel)
{
	if (channel.chan_idx != 0) {
		return -ENOTSUP;
	}
	switch (channel.chan_type) {
	case SENSOR_CHAN_ACCEL_XYZ:
		return LSM6DSO_FIFO_ACCEL;
	case SENSOR_CHAN_GYRO_XYZ:
		return LSM6DSO_FIFO_GYRO;
	default:
		return -ENOTSUP;
	}
}

static int frame_count(const uint8_t *buffer, struct sensor_chan_spec channel, uint16_t *count)
{
	const struct lsm6dso_fifo_packet *packet = (const void *)buffer;
	struct lsm6dso_fifo_sample sample;
	size_t cursor = 0;
	int kind = channel_kind(channel);
	int ret;

	if (!buffer || !count || kind < 0 || packet->words > LSM6DSO_FIFO_MAX_WORDS) {
		return -EINVAL;
	}
	*count = 0;
	while ((ret = lsm6dso_fifo_next(packet->data, packet->words * LSM6DSO_FIFO_WORD_SIZE,
					&cursor, &sample)) > 0) {
		if ((int)sample.kind == kind) {
			(*count)++;
		}
	}
	return ret;
}

static int size_info(struct sensor_chan_spec channel, size_t *base, size_t *frame)
{
	if (channel_kind(channel) < 0) {
		return -ENOTSUP;
	}
	if (!base || !frame) {
		return -EINVAL;
	}
	*base = sizeof(struct sensor_three_axis_data);
	*frame = sizeof(struct sensor_three_axis_sample_data);
	return 0;
}

/* The frame iterator: the position in the words, in bytes, and above it
 * the number of this channel's words before that position, which gives the
 * next frame's timestamp without counting them again on every call (the
 * sensor shell decodes one frame per call).  The caller starts with 0 and
 * passes it back unchanged.
 */
#define FIT_CURSOR(fit)         ((fit) & 0xffffU)
#define FIT_BEFORE(fit)         ((fit) >> 16)
#define FIT(cursor, before)     ((uint32_t)(cursor) | ((uint32_t)(before) << 16))

static int decode(const uint8_t *buffer, struct sensor_chan_spec channel, uint32_t *fit,
		  uint16_t max_count, void *out)
{
	const struct lsm6dso_fifo_packet *packet = (const void *)buffer;
	struct sensor_three_axis_data *decoded = out;
	struct lsm6dso_fifo_sample sample;
	size_t cursor;
	uint32_t before;
	uint32_t period;
	uint64_t base;
	int kind = channel_kind(channel), ret;
	uint16_t count = 0;
	struct lsm6dso_cal cal_copy;
	const struct lsm6dso_cal *cal = &cal_copy;
	int32_t values[3];

	if (!buffer || !fit || !out || kind < 0 || packet->words > LSM6DSO_FIFO_MAX_WORDS ||
	    FIT_CURSOR(*fit) > packet->words * LSM6DSO_FIFO_WORD_SIZE ||
	    FIT_CURSOR(*fit) % LSM6DSO_FIFO_WORD_SIZE != 0 ||
	    FIT_BEFORE(*fit) > FIT_CURSOR(*fit) / LSM6DSO_FIFO_WORD_SIZE) {
		return -EINVAL;
	}
	if (max_count == 0) {
		return 0;
	}
	/* The packet is packed; copy the conversion to an aligned object. */
	memcpy(&cal_copy, kind == LSM6DSO_FIFO_ACCEL ? &packet->accel_cal : &packet->gyro_cal,
	       sizeof(cal_copy));
	period = kind == LSM6DSO_FIFO_ACCEL ? packet->accel_period_ns : packet->gyro_period_ns;
	base = kind == LSM6DSO_FIFO_ACCEL ? packet->accel_first_timestamp_ns :
						    packet->gyro_first_timestamp_ns;
	cursor = FIT_CURSOR(*fit);
	before = FIT_BEFORE(*fit);
	decoded->header.base_timestamp_ns = base + (uint64_t)before * period;
	decoded->shift = FIFO_Q31_SHIFT;
	while (count < max_count && (ret = lsm6dso_fifo_next(packet->data,
				packet->words * LSM6DSO_FIFO_WORD_SIZE, &cursor, &sample)) > 0) {
		if ((int)sample.kind != kind) {
			continue;
		}
		if ((uint64_t)count * period > UINT32_MAX) {
			/* Leave this word for the next call. */
			cursor -= LSM6DSO_FIFO_WORD_SIZE;
			break;
		}
		decoded->readings[count].timestamp_delta = count * period;
		lsm6dso_cal_apply(cal, sample.xyz, values);
		for (int axis = 0; axis < 3; axis++) {
			decoded->readings[count].values[axis] = values[axis];
		}
		count++;
	}
	if (ret < 0) {
		return ret;
	}
	*fit = FIT(cursor, before + count);
	decoded->header.reading_count = count;
	return count;
}

static bool has_trigger(const uint8_t *buffer, enum sensor_trigger_type trigger)
{
	const struct lsm6dso_fifo_packet *packet = (const void *)buffer;

	return buffer && trigger == SENSOR_TRIG_FIFO_WATERMARK && packet->watermark;
}

const struct sensor_decoder_api lsm6dso_fifo_decoder = {
	.get_frame_count = frame_count,
	.get_size_info = size_info,
	.decode = decode,
	.has_trigger = has_trigger,
};

static bool is_fifo_packet(const uint8_t *buffer)
{
	uint32_t marker;

	if (!buffer) {
		return false;
	}
	memcpy(&marker, buffer + sizeof(uint64_t), sizeof(marker));
	return marker == LSM6DSO_FIFO_PACKET_MARKER;
}

static int combined_frame_count(const uint8_t *buffer, struct sensor_chan_spec channel,
				uint16_t *count)
{
	const struct sensor_decoder_api *decoder = is_fifo_packet(buffer) ? &lsm6dso_fifo_decoder :
								  &__sensor_default_decoder;

	return decoder->get_frame_count(buffer, channel, count);
}

static int combined_decode(const uint8_t *buffer, struct sensor_chan_spec channel, uint32_t *fit,
			   uint16_t max_count, void *out)
{
	const struct sensor_decoder_api *decoder = is_fifo_packet(buffer) ? &lsm6dso_fifo_decoder :
								  &__sensor_default_decoder;

	return decoder->decode(buffer, channel, fit, max_count, out);
}

static bool combined_has_trigger(const uint8_t *buffer, enum sensor_trigger_type trigger)
{
	return is_fifo_packet(buffer) && lsm6dso_fifo_decoder.has_trigger(buffer, trigger);
}

const struct sensor_decoder_api lsm6dso_combined_decoder = {
	.get_frame_count = combined_frame_count,
	/* Only the size of a channel's decoded representation depends on the
	 * channel, not on the input buffer format.
	 */
	.get_size_info = sensor_natively_supported_channel_size_info,
	.decode = combined_decode,
	.has_trigger = combined_has_trigger,
};
