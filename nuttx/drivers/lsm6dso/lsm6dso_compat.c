/* Portions copied from Zephyr v4.5.0 drivers/sensor/default_rtio_sensor.c.
 * Copyright (c) 2023 Google LLC.
 * Copyright (c) 2024 Croxel Inc.
 * SPDX-License-Identifier: Apache-2.0
 */
/* The Zephyr build's drivers/lsm6dso/lsm6dso_compat.c with only the system
 * parts replaced: <zephyr/logging/log.h> and CONFIG_SENSOR_LOG_LEVEL (3)
 * become app_log.h and LSM6DSO_COMPAT_LOG_LEVEL (3); <inttypes.h>, and
 * PRIu32/PRId32 for two uint32_t/q31_t log arguments (long on NuttX's arm).  The Zephyr names it uses come from the port's compatibility
 * layer (nuttx/drivers/compat).
 */
#ifdef __NuttX__
#include <nuttx/config.h>
#endif
#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/dsp/types.h>
/* CONFIG_SENSOR_LOG_LEVEL of the Zephyr build (3, info). */
#define LSM6DSO_COMPAT_LOG_LEVEL 3
#include "app_log.h"
#include <zephyr/rtio/work.h>
LOG_MODULE_REGISTER(lsm6dso_compat, LSM6DSO_COMPAT_LOG_LEVEL);
/**
 * @brief Compute the number of samples needed for the given channels
 *
 * @param[in] channels Array of channels requested
 * @param[in] num_channels Number of channels on the @p channels array
 * @return The number of samples required to read the given channels
 */
static inline int compute_num_samples(const struct sensor_chan_spec *const channels,
				      size_t num_channels)
{
	int num_samples = 0;

	for (size_t i = 0; i < num_channels; ++i) {
		num_samples += SENSOR_CHANNEL_3_AXIS(channels[i].chan_type) ? 3 : 1;
	}

	return num_samples;
}

/**
 * @brief Compute the required header size
 *
 * This function takes into account alignment of the q31 values that will follow the header.
 *
 * @param[in] num_output_samples The number of samples to represent
 * @return The number of bytes needed for this sample frame's header
 */
static inline uint32_t compute_header_size(int num_output_samples)
{
	uint32_t size = sizeof(struct sensor_data_generic_header) +
			(num_output_samples * sizeof(struct sensor_chan_spec));
	return (size + 3) & ~0x3;
}

/**
 * @brief Compute the minimum number of bytes needed
 *
 * @param[in] num_output_samples The number of samples to represent
 * @return The number of bytes needed for this sample frame
 */
static inline uint32_t compute_min_buf_len(int num_output_samples)
{
	return compute_header_size(num_output_samples) + (num_output_samples * sizeof(q31_t));
}

/**
 * @brief Checks if the header already contains a given channel
 *
 * @param[in] header The header to scan
 * @param[in] channel The channel to search for
 * @param[in] num_channels The number of valid channels in the header so far
 * @return Index of the @p channel if found or negative if not found
 */
static inline int check_header_contains_channel(const struct sensor_data_generic_header *header,
						struct sensor_chan_spec chan_spec, int num_channels)
{
	__ASSERT_NO_MSG(!SENSOR_CHANNEL_3_AXIS(chan_spec.chan_type));

	for (int i = 0; i < num_channels; ++i) {
		if (sensor_chan_spec_eq(header->channels[i], chan_spec)) {
			return i;
		}
	}
	return -1;
}

/**
 * @brief Fallback function for retrofiting old drivers to rtio (sync)
 *
 * @param[in] iodev_sqe The read submission queue event
 */
static void sensor_submit_fallback_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct device *dev = cfg->sensor;
	const struct sensor_chan_spec *const channels = cfg->channels;
	const int num_output_samples = compute_num_samples(channels, cfg->count);
	uint32_t min_buf_len = compute_min_buf_len(num_output_samples);
	uint64_t cycles;
	int rc;

	rc = sensor_clock_get_cycles(&cycles);
	if (rc != 0) {
		LOG_ERR("Failed to get sensor clock cycles");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	uint64_t timestamp_ns = sensor_clock_cycles_to_ns(cycles);
	uint8_t *buf;
	uint32_t buf_len;

	rc = sensor_sample_fetch(dev);
	/* Check that the fetch succeeded */
	if (rc != 0) {
		LOG_WRN("Failed to fetch samples");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	/* Get the buffer for the frame, it may be allocated dynamically by the rtio context */
	rc = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
	if (rc != 0) {
		LOG_WRN("Failed to get a read buffer of size %" PRIu32 " bytes", min_buf_len);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	/* Set the timestamp and num_channels */
	struct sensor_data_generic_header *header = (struct sensor_data_generic_header *)buf;

	header->timestamp_ns = timestamp_ns;
	header->num_channels = num_output_samples;
	header->shift = 0;

	q31_t *q = (q31_t *)(buf + compute_header_size(num_output_samples));

	/* Populate values, update shift, and set channels */
	for (size_t i = 0, sample_idx = 0; i < cfg->count; ++i) {
		struct sensor_value value[3];
		const int num_samples = SENSOR_CHANNEL_3_AXIS(channels[i].chan_type) ? 3 : 1;

		/* Get the current channel requested by the user */
		rc = sensor_channel_get(dev, channels[i].chan_type, value);

		if (num_samples == 3) {
			header->channels[sample_idx++] = (struct sensor_chan_spec) {
				rc == 0 ? channels[i].chan_type - 3 : SENSOR_CHAN_MAX,
				0
			};
			header->channels[sample_idx++] = (struct sensor_chan_spec) {
				rc == 0 ? channels[i].chan_type - 2 : SENSOR_CHAN_MAX,
				0
			};
			header->channels[sample_idx++] = (struct sensor_chan_spec) {
				rc == 0 ? channels[i].chan_type - 1 : SENSOR_CHAN_MAX,
				0
			};
		} else {
			header->channels[sample_idx++] = (struct sensor_chan_spec) {
				rc == 0 ? channels[i].chan_type : SENSOR_CHAN_MAX,
				0
			};
		}

		if (rc != 0) {
			LOG_DBG("Failed to get channel (type: %d, index %d), skipping",
				channels[i].chan_type, channels[i].chan_idx);
			continue;
		}

		/* Get the largest absolute value reading to set the scale for the channel */
		uint32_t header_scale = 0;

		for (int sample = 0; sample < num_samples; ++sample) {
			/*
			 * The scale is the ceil(abs(sample)).
			 * Since we are using fractional values, it's easier to assume that .val2
			 *   is non 0 and convert this to abs(sample.val1) + 1 (removing a branch).
			 * Since it's possible that val1 (int32_t) is saturated (INT32_MAX) we need
			 *   to upcast it to 64 bit int first, then take the abs() of that 64 bit
			 *   int before we '+ 1'. Once that's done, we can safely cast back down
			 *   to uint32_t because the min value is 0 and max is INT32_MAX + 1 which
			 *   is less than UINT32_MAX.
			 */
			uint32_t scale = (uint32_t)llabs((int64_t)value[sample].val1) + 1;

			header_scale = MAX(header_scale, scale);
		}

		int8_t new_shift = ilog2(header_scale - 1) + 1;

		/* Reset sample_idx */
		sample_idx -= num_samples;
		if (header->shift < new_shift) {
			/*
			 * Shift was updated, need to convert all the existing q values. This could
			 * be optimized by calling zdsp_scale_q31() but that would force a
			 * dependency between sensors and the zDSP subsystem.
			 */
			for (int q_idx = 0; q_idx < sample_idx; ++q_idx) {
				q[q_idx] = q[q_idx] >> (new_shift - header->shift);
			}
			header->shift = new_shift;
		}

		/*
		 * Spread the q31 values. This is needed because some channels are 3D. If
		 * the user specified one of those then num_samples will be 3; and we need to
		 * produce 3 separate readings.
		 */
		for (int sample = 0; sample < num_samples; ++sample) {
			/* Check if the channel is already in the buffer */
			int prev_computed_value_idx = check_header_contains_channel(
				header, header->channels[sample_idx + sample], sample_idx + sample);

			if (prev_computed_value_idx >= 0 &&
			    prev_computed_value_idx != sample_idx + sample) {
				LOG_DBG("value[%d] previously computed at q[%d]@%p", sample,
					prev_computed_value_idx,
					(void *)&q[prev_computed_value_idx]);
				q[sample_idx + sample] = q[prev_computed_value_idx];
				continue;
			}

			/* Convert the value to micro-units */
			int64_t value_u = sensor_value_to_micro(&value[sample]);

			/* Convert to q31 using the shift */
			q[sample_idx + sample] =
				((value_u * (INT64_C(1) << 31)) / 1000000) >> header->shift;

			LOG_DBG("value[%d]=%s%d.%06d, q[%d]@%p=%" PRId32 ", shift: %d",
				sample, value_u < 0 ? "-" : "",
				abs((int)value[sample].val1), abs((int)value[sample].val2),
				(int)(sample_idx + sample), (void *)&q[sample_idx + sample],
				q[sample_idx + sample], header->shift);
		}
		sample_idx += num_samples;
	}
	LOG_DBG("Total channels in header: %" PRIu32, header->num_channels);
	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

/**
 * @brief Fallback function for retrofiting old drivers to rtio
 *
 * @param[in] dev The sensor device to read
 * @param[in] iodev_sqe The read submission queue event
 */
void lsm6dso_submit_fallback(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct rtio_work_req *req = rtio_work_req_alloc();

	if (req == NULL) {
		LOG_ERR("RTIO work item allocation failed. Consider to increase "
			"CONFIG_RTIO_WORKQ_POOL_ITEMS.");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	rtio_work_req_submit(req, iodev_sqe, sensor_submit_fallback_sync);
}
