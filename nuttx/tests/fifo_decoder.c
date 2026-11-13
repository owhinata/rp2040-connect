/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <errno.h>
#include <string.h>
#include "lsm6dso_fifo_decoder.h"

static int default_count(const uint8_t *buf, struct sensor_chan_spec channel, uint16_t *count)
{
	(void)buf;
	(void)channel;
	*count = 42;
	return 0;
}
static int default_decode(const uint8_t *buf, struct sensor_chan_spec channel, uint32_t *fit,
			  uint16_t count, void *out)
{
	(void)buf; (void)channel; (void)fit; (void)count; (void)out;
	return 42;
}
int sensor_natively_supported_channel_size_info(struct sensor_chan_spec channel, size_t *base,
						size_t *frame)
{
	return lsm6dso_fifo_decoder.get_size_info(channel, base, frame);
}
const struct sensor_decoder_api __sensor_default_decoder = {
	.get_frame_count = default_count,
	.decode = default_decode,
};

int main(void)
{
	union { uint64_t align; uint8_t bytes[sizeof(struct lsm6dso_fifo_packet) + 21]; } storage = {0};
	union { uint64_t align; uint8_t bytes[sizeof(struct sensor_three_axis_data) +
						 sizeof(struct sensor_three_axis_sample_data)]; } output;
	struct lsm6dso_fifo_packet *packet = (void *)storage.bytes;
	struct sensor_three_axis_data *data = (void *)output.bytes;
	struct sensor_chan_spec accel = { SENSOR_CHAN_ACCEL_XYZ, 0 };
	struct sensor_chan_spec gyro = { SENSOR_CHAN_GYRO_XYZ, 0 };
	const uint8_t words[] = {
		0x10, 0xe8, 0x03, 0, 0, 0, 0, /* accel X = 1000 */
		0x08, 0, 0, 0, 0, 0, 0,        /* gyro */
		0x10, 0x18, 0xfc, 0, 0, 0, 0, /* accel X = -1000 */
	};
	struct lsm6dso_cal nominal;
	uint16_t count;
	uint32_t fit = 0;
	size_t base, frame;

	packet->words = 3;
	packet->marker = LSM6DSO_FIFO_PACKET_MARKER;
	packet->accel_first_timestamp_ns = 1000;
	packet->accel_period_ns = 100;
	packet->accel_gain_ug = 1000;
	lsm6dso_cal_nominal(&nominal, LSM6DSO_CAL_ACCEL, 1000);
	memcpy(&packet->accel_cal, &nominal, sizeof(nominal));
	packet->watermark = 1;
	memcpy(packet->data, words, sizeof(words));
	assert(lsm6dso_fifo_decoder.get_frame_count(storage.bytes, accel, &count) == 0 && count == 2);
	assert(lsm6dso_fifo_decoder.get_frame_count(storage.bytes, gyro, &count) == 0 && count == 1);
	assert(lsm6dso_combined_decoder.get_frame_count(storage.bytes, accel, &count) == 0 && count == 2);
	assert(lsm6dso_fifo_decoder.get_size_info(accel, &base, &frame) == 0);
	assert(base == sizeof(*data) && frame == sizeof(struct sensor_three_axis_sample_data));
	assert(lsm6dso_fifo_decoder.has_trigger(storage.bytes, SENSOR_TRIG_FIFO_WATERMARK));
	assert(lsm6dso_fifo_decoder.decode(storage.bytes, accel, &fit, 1, data) == 1);
	assert(fit == 7 && data->header.base_timestamp_ns == 1000 && data->shift == 9);
	assert(data->readings[0].values[0] == 41132071);
	assert(lsm6dso_fifo_decoder.decode(storage.bytes, accel, &fit, 1, data) == 1);
	assert(fit == 21 && data->header.base_timestamp_ns == 1100);
	assert(data->readings[0].values[0] == -41132071);
	assert(lsm6dso_fifo_decoder.decode(storage.bytes, accel, &fit, 1, data) == 0);
	fit = 1;
	assert(lsm6dso_fifo_decoder.decode(storage.bytes, accel, &fit, 1, data) == -EINVAL);
	packet->marker = 6; /* generic fallback buffer */
	assert(lsm6dso_combined_decoder.get_frame_count(storage.bytes, accel, &count) == 0 && count == 42);
	assert(lsm6dso_combined_decoder.decode(storage.bytes, accel, &fit, 1, data) == 42);
	return 0;
}
