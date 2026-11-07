/* SPDX-License-Identifier: Apache-2.0 */
#ifndef TEST_FAKE_SENSOR_H_
#define TEST_FAKE_SENSOR_H_
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef int32_t q31_t;
enum sensor_channel { SENSOR_CHAN_ACCEL_XYZ = 3, SENSOR_CHAN_GYRO_XYZ = 7 };
enum sensor_trigger_type { SENSOR_TRIG_FIFO_WATERMARK = 1 };
struct sensor_chan_spec { uint16_t chan_type, chan_idx; };
struct sensor_data_header { uint64_t base_timestamp_ns; uint16_t reading_count; };
struct sensor_three_axis_data {
	struct sensor_data_header header;
	int8_t shift;
	struct sensor_three_axis_sample_data { uint32_t timestamp_delta; q31_t values[3]; } readings[1];
};
struct sensor_decoder_api {
	int (*get_frame_count)(const uint8_t *, struct sensor_chan_spec, uint16_t *);
	int (*get_size_info)(struct sensor_chan_spec, size_t *, size_t *);
	int (*decode)(const uint8_t *, struct sensor_chan_spec, uint32_t *, uint16_t, void *);
	bool (*has_trigger)(const uint8_t *, enum sensor_trigger_type);
};
extern const struct sensor_decoder_api __sensor_default_decoder;
int sensor_natively_supported_channel_size_info(struct sensor_chan_spec, size_t *, size_t *);
#define SENSOR_G 9806650LL
#define SENSOR_PI 3141592LL
#define CLAMP(v, lo, hi) ((v) < (lo) ? (lo) : (v) > (hi) ? (hi) : (v))
#endif
