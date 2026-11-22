/* ST Microelectronics LSM6DSO 6-axis IMU sensor driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 */

/* The driver's device and sensor API on the chip's core (lsm6dso_chip.c):
 * attributes, single reads and their channels, the decoder and RTIO
 * submissions (lsm6dso_rtio.c, lsm6dso_rtio_stream.c), and the devices.
 */

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "lsm6dso.h"
#include "lsm6dso_decoder.h"

LOG_MODULE_REGISTER(LSM6DSO, CONFIG_SENSOR_LOG_LEVEL);

static int lsm6dso_accel_config(const struct device *dev,
				enum sensor_channel chan,
				enum sensor_attribute attr,
				const struct sensor_value *val)
{
	struct lsm6dso_data *data = dev->data;

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return lsm6dso_chip_accel_range_set(&data->chip, sensor_ms2_to_g(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return lsm6dso_chip_accel_odr_set(&data->chip, val->val1);
	default:
		LOG_DBG("Accel attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6dso_gyro_config(const struct device *dev,
			       enum sensor_channel chan,
			       enum sensor_attribute attr,
			       const struct sensor_value *val)
{
	struct lsm6dso_data *data = dev->data;

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return lsm6dso_chip_gyro_range_set(&data->chip, sensor_rad_to_degrees(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return lsm6dso_chip_gyro_odr_set(&data->chip, val->val1);
	default:
		LOG_DBG("Gyro attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6dso_attr_set_unlocked(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		return lsm6dso_accel_config(dev, chan, attr, val);
	case SENSOR_CHAN_GYRO_XYZ:
		return lsm6dso_gyro_config(dev, chan, attr, val);
	default:
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6dso_sample_fetch_unlocked(const struct device *dev,
				enum sensor_channel chan)
{
	struct lsm6dso_data *data = dev->data;
	int ret = 0;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		ret = lsm6dso_chip_fetch_accel(&data->chip);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		ret = lsm6dso_chip_fetch_gyro(&data->chip);
		break;
	case SENSOR_CHAN_ALL:
		ret = lsm6dso_chip_fetch_accel(&data->chip);
		if (ret != 0) {
			break;
		}
		ret = lsm6dso_chip_fetch_gyro(&data->chip);
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}

void lsm6dso_cal_reload(const struct device *dev)
{
	struct lsm6dso_data *data = dev->data;

	lsm6dso_chip_cal_reload(&data->chip);
}

void lsm6dso_cal_set_lookup(const struct device *dev, lsm6dso_cal_lookup_t lookup)
{
	struct lsm6dso_data *data = dev->data;

	lsm6dso_chip_cal_set_lookup(&data->chip, lookup);
}

void lsm6dso_cal_status(const struct device *dev, struct lsm6dso_cal_status *status)
{
	struct lsm6dso_data *data = dev->data;

	lsm6dso_chip_cal_status(&data->chip, status);
}

/* The last fetched sample in the units of the sensor API, converted as the
 * FIFO decoder converts (lsm6dso_chip_convert()).
 */
static int lsm6dso_converted_channel_get(const struct device *dev, enum sensor_channel chan,
					 struct sensor_value *val, bool accel)
{
	struct lsm6dso_data *data = dev->data;
	int32_t out[3];
	int first, last;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_GYRO_X:
		first = last = 0;
		break;
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_GYRO_Y:
		first = last = 1;
		break;
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_GYRO_Z:
		first = last = 2;
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_GYRO_XYZ:
		first = 0;
		last = 2;
		break;
	default:
		return -ENOTSUP;
	}
	lsm6dso_chip_convert(&data->chip, accel ? LSM6DSO_CAL_ACCEL : LSM6DSO_CAL_GYRO, out);
	for (int axis = first; axis <= last; axis++) {
		int64_t micro = lsm6dso_cal_to_micro(out[axis]);

		val->val1 = (int32_t)(micro / 1000000);
		val->val2 = (int32_t)(micro % 1000000);
		val++;
	}
	return 0;
}

static int lsm6dso_channel_get_unlocked(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		return lsm6dso_converted_channel_get(dev, chan, val, true);
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		return lsm6dso_converted_channel_get(dev, chan, val, false);
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6dso_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	struct lsm6dso_data *data = dev->data;
	int ret;

#ifdef CONFIG_SENSOR_ASYNC_API
	if (attr == (enum sensor_attribute)SENSOR_ATTR_LSM6DSO_FIFO_TIMESTAMP_DECIMATION) {
		if (chan != SENSOR_CHAN_ALL || val->val2 != 0 || val->val1 < 0) {
			return -EINVAL;
		}
		return lsm6dso_fifo_set_timestamp_decimation(dev, (unsigned int)val->val1);
	}
	if (attr == (enum sensor_attribute)SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK) {
		if (chan != SENSOR_CHAN_ALL || val->val2 != 0 || val->val1 < 1 ||
		    val->val1 > LSM6DSO_FIFO_MAX_WATERMARK) {
			return -EINVAL;
		}
		return lsm6dso_fifo_set_watermark(dev, (uint16_t)val->val1);
	}
#endif
	ret = lsm6dso_chip_acquire(&data->chip);
	if (ret == 0) {
		ret = lsm6dso_attr_set_unlocked(dev, chan, attr, val);
		lsm6dso_chip_release(&data->chip);
	}
	return ret;
}

#ifdef CONFIG_SENSOR_ASYNC_API
static int lsm6dso_attr_get(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}
	if (attr == (enum sensor_attribute)SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK) {
		val->val1 = lsm6dso_fifo_get_watermark(dev);
	} else if (attr == (enum sensor_attribute)SENSOR_ATTR_LSM6DSO_FIFO_TIMESTAMP_DECIMATION) {
		val->val1 = (int32_t)lsm6dso_fifo_get_timestamp_decimation(dev);
	} else {
		return -ENOTSUP;
	}
	val->val2 = 0;
	return 0;
}
#endif

static int lsm6dso_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct lsm6dso_data *data = dev->data;
	int ret;

	ret = lsm6dso_chip_acquire(&data->chip);
	if (ret == 0) {
		ret = lsm6dso_sample_fetch_unlocked(dev, chan);
		lsm6dso_chip_release(&data->chip);
	}
	return ret;
}

int lsm6dso_cal_zero(const struct device *dev)
{
	struct lsm6dso_data *data = dev->data;

	return lsm6dso_chip_cal_zero(&data->chip);
}

static int lsm6dso_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct lsm6dso_data *data = dev->data;
	int ret;

	ret = lsm6dso_chip_acquire(&data->chip);
	if (ret == 0) {
		ret = lsm6dso_channel_get_unlocked(dev, chan, val);
		lsm6dso_chip_release(&data->chip);
	}
	return ret;
}

int lsm6dso_timestamp_read(const struct device *dev, int64_t (*clock_us)(void), uint32_t *tick,
			   int64_t *before_us, int64_t *after_us)
{
	struct lsm6dso_data *data = dev->data;

	return lsm6dso_chip_timestamp_read(&data->chip, clock_us, tick, before_us, after_us);
}

void lsm6dso_fifo_get_stats(const struct device *dev, struct lsm6dso_fifo_stats *stats)
{
	struct lsm6dso_data *data = dev->data;

	lsm6dso_chip_fifo_get_stats(&data->chip, stats);
}

#ifdef CONFIG_SENSOR_ASYNC_API
static int lsm6dso_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &lsm6dso_combined_decoder;
	return 0;
}
#endif

static DEVICE_API(sensor, lsm6dso_driver_api) = {
	.attr_set = lsm6dso_attr_set,
#ifdef CONFIG_SENSOR_ASYNC_API
	.attr_get = lsm6dso_attr_get,
#endif
	.sample_fetch = lsm6dso_sample_fetch,
	.channel_get = lsm6dso_channel_get,
#ifdef CONFIG_SENSOR_ASYNC_API
	.get_decoder = lsm6dso_get_decoder,
	.submit = lsm6dso_submit,
#endif
};

static int lsm6dso_init(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;
	const struct lsm6dso_chip_config chip_cfg = {
		.ctx = &cfg->ctx,
		.accel_pm = cfg->accel_pm,
		.accel_odr = cfg->accel_odr,
		.accel_lp_filter = cfg->accel_lp_filter,
		.accel_range = cfg->accel_range & ACCEL_RANGE_MASK,
		.accel_range_double = cfg->accel_range & ACCEL_RANGE_DOUBLE,
		.gyro_pm = cfg->gyro_pm,
		.gyro_odr = cfg->gyro_odr,
		.gyro_range = cfg->gyro_range,
	};

	LOG_INF("Initialize device %s", dev->name);
	data->dev = dev;
#ifdef CONFIG_SENSOR_ASYNC_API
	lsm6dso_stream_init(dev);
#endif

	if (lsm6dso_chip_init(&data->chip, &chip_cfg) < 0) {
		LOG_ERR("failed to initialize chip");
		return -EIO;
	}

	return 0;
}

/* The devices: lsm6dso_instance.h of the OS's build, port/<OS> on its include
 * path before this directory (this directory's: the devicetree instances of
 * the Zephyr build; the NuttX and FreeRTOS builds' one device).
 */
#include <lsm6dso_instance.h>
