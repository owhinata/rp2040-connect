/* ST Microelectronics LSM6DSO 6-axis IMU sensor driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

#include "lsm6dso.h"
#include "lsm6dso_fifo.h"
#include "lsm6dso_attributes.h"
#include "lsm6dso_fifo_decoder.h"
#include "lsm6dso_fifo_hw.h"

LOG_MODULE_REGISTER(LSM6DSO, CONFIG_SENSOR_LOG_LEVEL);

static const uint16_t lsm6dso_odr_map[] = {0, 12, 26, 52, 104, 208, 417, 833,
					1667, 3333, 6667};
static const uint8_t lsm6dso_lp_filter_map[] = {0,
						LSM6DSO_LP_ODR_DIV_10,
						LSM6DSO_LP_ODR_DIV_20,
						LSM6DSO_LP_ODR_DIV_45,
						LSM6DSO_LP_ODR_DIV_100,
						LSM6DSO_LP_ODR_DIV_200,
						LSM6DSO_LP_ODR_DIV_400,
						LSM6DSO_LP_ODR_DIV_800};

static int lsm6dso_freq_to_odr_val(uint16_t freq)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(lsm6dso_odr_map); i++) {
		if (freq <= lsm6dso_odr_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static int lsm6dso_odr_to_freq_val(uint16_t odr)
{
	/* for valid index, return value from map */
	if (odr < ARRAY_SIZE(lsm6dso_odr_map)) {
		return lsm6dso_odr_map[odr];
	}

	/* invalid index, return last entry */
	return lsm6dso_odr_map[ARRAY_SIZE(lsm6dso_odr_map) - 1];
}

static const uint16_t lsm6dso_accel_fs_map[] = {2, 16, 4, 8};

static int lsm6dso_accel_range_to_fs_val(int32_t range, bool double_range)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(lsm6dso_accel_fs_map); i++) {
		if (range == (lsm6dso_accel_fs_map[i] << double_range)) {
			return i;
		}
	}

	return -EINVAL;
}

static int lsm6dso_accel_fs_val_to_gain(int fs, bool double_range)
{
	/* Range of ±2G has a LSB of GAIN_UNIT_XL, thus divide by 2 */
	return double_range ?
		lsm6dso_accel_fs_map[fs] * GAIN_UNIT_XL :
		lsm6dso_accel_fs_map[fs] * GAIN_UNIT_XL / 2;
}

static const uint16_t lsm6dso_gyro_fs_map[] = {250, 125, 500, 0, 1000, 0, 2000};
static const uint16_t lsm6dso_gyro_fs_sens[] = {2, 1, 4, 0, 8, 0, 16};

static int lsm6dso_gyro_range_to_fs_val(int32_t range)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(lsm6dso_gyro_fs_map); i++) {
		if (range == lsm6dso_gyro_fs_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static int lsm6dso_accel_set_fs_raw(const struct device *dev, uint8_t fs)
{
	const struct lsm6dso_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6dso_data *data = dev->data;

	if (lsm6dso_xl_full_scale_set(ctx, fs) < 0) {
		return -EIO;
	}

	data->accel_fs = fs;

	return 0;
}

static int lsm6dso_accel_set_odr_raw(const struct device *dev, uint8_t odr)
{
	const struct lsm6dso_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6dso_data *data = dev->data;

	if (lsm6dso_xl_data_rate_set(ctx, odr) < 0) {
		return -EIO;
	}

	data->accel_freq = lsm6dso_odr_to_freq_val(odr);

	return 0;
}

static int lsm6dso_gyro_set_fs_raw(const struct device *dev, uint8_t fs)
{
	const struct lsm6dso_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	if (lsm6dso_gy_full_scale_set(ctx, fs) < 0) {
		return -EIO;
	}

	return 0;
}

static int lsm6dso_gyro_set_odr_raw(const struct device *dev, uint8_t odr)
{
	const struct lsm6dso_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6dso_data *data = dev->data;

	if (lsm6dso_gy_data_rate_set(ctx, odr) < 0) {
		return -EIO;
	}
	data->gyro_freq = lsm6dso_odr_to_freq_val(odr);

	return 0;
}

static int lsm6dso_accel_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;

	odr = lsm6dso_freq_to_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	if (lsm6dso_accel_set_odr_raw(dev, odr) < 0) {
		LOG_DBG("failed to set accelerometer sampling rate");
		return -EIO;
	}

	return 0;
}

static int lsm6dso_accel_range_set(const struct device *dev, int32_t range)
{
	int fs;
	struct lsm6dso_data *data = dev->data;
	const struct lsm6dso_config *cfg = dev->config;
	bool range_double = !!(cfg->accel_range & ACCEL_RANGE_DOUBLE);

	fs = lsm6dso_accel_range_to_fs_val(range, range_double);
	if (fs < 0) {
		return fs;
	}

	if (lsm6dso_accel_set_fs_raw(dev, fs) < 0) {
		LOG_DBG("failed to set accelerometer full-scale");
		return -EIO;
	}

	data->acc_gain = lsm6dso_accel_fs_val_to_gain(fs, range_double);
	return 0;
}

static int lsm6dso_accel_config(const struct device *dev,
				enum sensor_channel chan,
				enum sensor_attribute attr,
				const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return lsm6dso_accel_range_set(dev, sensor_ms2_to_g(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return lsm6dso_accel_odr_set(dev, val->val1);
	default:
		LOG_DBG("Accel attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6dso_gyro_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;

	odr = lsm6dso_freq_to_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	if (lsm6dso_gyro_set_odr_raw(dev, odr) < 0) {
		LOG_DBG("failed to set gyroscope sampling rate");
		return -EIO;
	}

	return 0;
}

static int lsm6dso_gyro_range_set(const struct device *dev, int32_t range)
{
	int fs;
	struct lsm6dso_data *data = dev->data;

	fs = lsm6dso_gyro_range_to_fs_val(range);
	if (fs < 0) {
		return fs;
	}

	if (lsm6dso_gyro_set_fs_raw(dev, fs) < 0) {
		LOG_DBG("failed to set gyroscope full-scale");
		return -EIO;
	}

	data->gyro_gain = (lsm6dso_gyro_fs_sens[fs] * GAIN_UNIT_G);
	return 0;
}

static int lsm6dso_gyro_config(const struct device *dev,
			       enum sensor_channel chan,
			       enum sensor_attribute attr,
			       const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return lsm6dso_gyro_range_set(dev, sensor_rad_to_degrees(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return lsm6dso_gyro_odr_set(dev, val->val1);
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
#if defined(CONFIG_LSM6DSO_SENSORHUB)
	struct lsm6dso_data *data = dev->data;
#endif /* CONFIG_LSM6DSO_SENSORHUB */

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		return lsm6dso_accel_config(dev, chan, attr, val);
	case SENSOR_CHAN_GYRO_XYZ:
		return lsm6dso_gyro_config(dev, chan, attr, val);
#if defined(CONFIG_LSM6DSO_SENSORHUB)
	case SENSOR_CHAN_MAGN_XYZ:
	case SENSOR_CHAN_PRESS:
	case SENSOR_CHAN_HUMIDITY:
		if (!data->shub_inited) {
			LOG_ERR("shub not inited.");
			return -ENOTSUP;
		}

		return lsm6dso_shub_config(dev, chan, attr, val);
#endif /* CONFIG_LSM6DSO_SENSORHUB */
	default:
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6dso_sample_fetch_accel(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6dso_data *data = dev->data;

	if (lsm6dso_acceleration_raw_get(ctx, data->acc) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	return 0;
}

static int lsm6dso_sample_fetch_gyro(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6dso_data *data = dev->data;

	if (lsm6dso_angular_rate_raw_get(ctx, data->gyro) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_LSM6DSO_ENABLE_TEMP)
static int lsm6dso_sample_fetch_temp(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6dso_data *data = dev->data;

	if (lsm6dso_temperature_raw_get(ctx, &data->temp_sample) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	return 0;
}
#endif

#if defined(CONFIG_LSM6DSO_SENSORHUB)
static int lsm6dso_sample_fetch_shub(const struct device *dev)
{
	if (lsm6dso_shub_fetch_external_devs(dev) < 0) {
		LOG_DBG("failed to read ext shub devices");
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_LSM6DSO_SENSORHUB */

static int lsm6dso_sample_fetch_unlocked(const struct device *dev,
				enum sensor_channel chan)
{
#if defined(CONFIG_LSM6DSO_SENSORHUB)
	struct lsm6dso_data *data = dev->data;
#endif /* CONFIG_LSM6DSO_SENSORHUB */
	int ret = 0;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		ret = lsm6dso_sample_fetch_accel(dev);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		ret = lsm6dso_sample_fetch_gyro(dev);
		break;
#if defined(CONFIG_LSM6DSO_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		ret = lsm6dso_sample_fetch_temp(dev);
		break;
#endif
	case SENSOR_CHAN_ALL:
		ret = lsm6dso_sample_fetch_accel(dev);
		if (ret != 0) {
			break;
		}
		ret = lsm6dso_sample_fetch_gyro(dev);
		if (ret != 0) {
			break;
		}
#if defined(CONFIG_LSM6DSO_ENABLE_TEMP)
		ret = lsm6dso_sample_fetch_temp(dev);
		if (ret != 0) {
			break;
		}
#endif
#if defined(CONFIG_LSM6DSO_SENSORHUB)
		if (data->shub_inited) {
			ret = lsm6dso_sample_fetch_shub(dev);
		}
#endif
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}

static inline void lsm6dso_accel_convert(struct sensor_value *val, int raw_val,
					 uint32_t sensitivity)
{
	int64_t dval;

	/* Sensitivity is exposed in ug/LSB */
	/* Convert to m/s^2 */
	dval = (int64_t)(raw_val) * sensitivity;
	sensor_ug_to_ms2(dval, val);
}

static inline int lsm6dso_accel_get_channel(enum sensor_channel chan,
					    struct sensor_value *val,
					    struct lsm6dso_data *data,
					    uint32_t sensitivity)
{
	uint8_t i;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		lsm6dso_accel_convert(val, data->acc[0], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		lsm6dso_accel_convert(val, data->acc[1], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		lsm6dso_accel_convert(val, data->acc[2], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		for (i = 0; i < 3; i++) {
			lsm6dso_accel_convert(val++, data->acc[i], sensitivity);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6dso_accel_channel_get(enum sensor_channel chan,
				     struct sensor_value *val,
				     struct lsm6dso_data *data)
{
	return lsm6dso_accel_get_channel(chan, val, data, data->acc_gain);
}

static inline void lsm6dso_gyro_convert(struct sensor_value *val, int raw_val,
					uint32_t sensitivity)
{
	int64_t dval;

	/* Sensitivity is exposed in udps/LSB */
	/* So, calculate value in 10 udps unit and then to rad/s */
	dval = (int64_t)(raw_val) * sensitivity / 10;
	sensor_10udegrees_to_rad(dval, val);
}

static inline int lsm6dso_gyro_get_channel(enum sensor_channel chan,
					   struct sensor_value *val,
					   struct lsm6dso_data *data,
					   uint32_t sensitivity)
{
	uint8_t i;

	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
		lsm6dso_gyro_convert(val, data->gyro[0], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Y:
		lsm6dso_gyro_convert(val, data->gyro[1], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Z:
		lsm6dso_gyro_convert(val, data->gyro[2], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		for (i = 0; i < 3; i++) {
			lsm6dso_gyro_convert(val++, data->gyro[i], sensitivity);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int lsm6dso_gyro_channel_get(enum sensor_channel chan,
				    struct sensor_value *val,
				    struct lsm6dso_data *data)
{
	return lsm6dso_gyro_get_channel(chan, val, data, data->gyro_gain);
}

#if defined(CONFIG_LSM6DSO_ENABLE_TEMP)
static void lsm6dso_gyro_channel_get_temp(struct sensor_value *val,
					  struct lsm6dso_data *data)
{
	/* val = temp_sample / 256 + 25 */
	val->val1 = data->temp_sample / 256 + 25;
	val->val2 = (data->temp_sample % 256) * (1000000 / 256);
}
#endif

#if defined(CONFIG_LSM6DSO_SENSORHUB)
static inline void lsm6dso_magn_convert(struct sensor_value *val, int raw_val,
					uint16_t sensitivity)
{
	double dval;

	/* Sensitivity is exposed in ugauss/LSB */
	dval = (double)(raw_val * sensitivity);
	val->val1 = (int32_t)dval / 1000000;
	val->val2 = (int32_t)dval % 1000000;
}

static inline int lsm6dso_magn_get_channel(enum sensor_channel chan,
					   struct sensor_value *val,
					   struct lsm6dso_data *data)
{
	int16_t sample[3];
	int idx;

	idx = lsm6dso_shub_get_idx(data->dev, SENSOR_CHAN_MAGN_XYZ);
	if (idx < 0) {
		LOG_DBG("external magn not supported");
		return -ENOTSUP;
	}


	sample[0] = (int16_t)(data->ext_data[idx][0] |
			     (data->ext_data[idx][1] << 8));
	sample[1] = (int16_t)(data->ext_data[idx][2] |
			     (data->ext_data[idx][3] << 8));
	sample[2] = (int16_t)(data->ext_data[idx][4] |
			     (data->ext_data[idx][5] << 8));

	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
		lsm6dso_magn_convert(val, sample[0], data->magn_gain);
		break;
	case SENSOR_CHAN_MAGN_Y:
		lsm6dso_magn_convert(val, sample[1], data->magn_gain);
		break;
	case SENSOR_CHAN_MAGN_Z:
		lsm6dso_magn_convert(val, sample[2], data->magn_gain);
		break;
	case SENSOR_CHAN_MAGN_XYZ:
		lsm6dso_magn_convert(val, sample[0], data->magn_gain);
		lsm6dso_magn_convert(val + 1, sample[1], data->magn_gain);
		lsm6dso_magn_convert(val + 2, sample[2], data->magn_gain);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline void lsm6dso_hum_convert(struct sensor_value *val,
				       struct lsm6dso_data *data)
{
	float rh;
	int16_t raw_val;
	struct hts221_data *ht = &data->hts221;
	int idx;

	idx = lsm6dso_shub_get_idx(data->dev, SENSOR_CHAN_HUMIDITY);
	if (idx < 0) {
		LOG_DBG("external press/temp not supported");
		return;
	}

	raw_val = (int16_t)(data->ext_data[idx][0] |
			   (data->ext_data[idx][1] << 8));

	/* find relative humidty by linear interpolation */
	rh = (ht->y1 - ht->y0) * raw_val + ht->x1 * ht->y0 - ht->x0 * ht->y1;
	rh /= (ht->x1 - ht->x0);

	/* convert humidity to integer and fractional part */
	sensor_value_from_float(val, rh);
}

static inline void lsm6dso_press_convert(struct sensor_value *val,
					 struct lsm6dso_data *data)
{
	int32_t raw_val;
	int idx;

	idx = lsm6dso_shub_get_idx(data->dev, SENSOR_CHAN_PRESS);
	if (idx < 0) {
		LOG_DBG("external press/temp not supported");
		return;
	}

	raw_val = (int32_t)(data->ext_data[idx][0] |
			   (data->ext_data[idx][1] << 8) |
			   (data->ext_data[idx][2] << 16));

	/* Pressure sensitivity is 4096 LSB/hPa */
	/* Convert raw_val to val in kPa */
	val->val1 = (raw_val >> 12) / 10;
	val->val2 = (raw_val >> 12) % 10 * 100000 +
		(((int32_t)((raw_val) & 0x0FFF) * 100000L) >> 12);
}

static inline void lsm6dso_temp_convert(struct sensor_value *val,
					struct lsm6dso_data *data)
{
	int16_t raw_val;
	int idx;

	idx = lsm6dso_shub_get_idx(data->dev, SENSOR_CHAN_PRESS);
	if (idx < 0) {
		LOG_DBG("external press/temp not supported");
		return;
	}

	raw_val = (int16_t)(data->ext_data[idx][3] |
			   (data->ext_data[idx][4] << 8));

	/* Temperature sensitivity is 100 LSB/deg C */
	val->val1 = raw_val / 100;
	val->val2 = (int32_t)raw_val % 100 * (10000);
}
#endif

static int lsm6dso_channel_get_unlocked(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct lsm6dso_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		lsm6dso_accel_channel_get(chan, val, data);
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		lsm6dso_gyro_channel_get(chan, val, data);
		break;
#if defined(CONFIG_LSM6DSO_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		lsm6dso_gyro_channel_get_temp(val, data);
		break;
#endif
#if defined(CONFIG_LSM6DSO_SENSORHUB)
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
	case SENSOR_CHAN_MAGN_XYZ:
		if (!data->shub_inited) {
			LOG_ERR("attr_set() shub not inited.");
			return -ENOTSUP;
		}

		lsm6dso_magn_get_channel(chan, val, data);
		break;

	case SENSOR_CHAN_HUMIDITY:
		if (!data->shub_inited) {
			LOG_ERR("attr_set() shub not inited.");
			return -ENOTSUP;
		}

		lsm6dso_hum_convert(val, data);
		break;

	case SENSOR_CHAN_PRESS:
		if (!data->shub_inited) {
			LOG_ERR("attr_set() shub not inited.");
			return -ENOTSUP;
		}

		lsm6dso_press_convert(val, data);
		break;

	case SENSOR_CHAN_AMBIENT_TEMP:
		if (!data->shub_inited) {
			LOG_ERR("attr_set() shub not inited.");
			return -ENOTSUP;
		}

		lsm6dso_temp_convert(val, data);
		break;
#endif
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
	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	ret = data->fifo_owned ? -EBUSY : lsm6dso_attr_set_unlocked(dev, chan, attr, val);
	k_mutex_unlock(&data->fifo_lock);
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

	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	ret = data->fifo_owned ? -EBUSY : lsm6dso_sample_fetch_unlocked(dev, chan);
	k_mutex_unlock(&data->fifo_lock);
	return ret;
}

static int lsm6dso_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct lsm6dso_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	ret = data->fifo_owned ? -EBUSY : lsm6dso_channel_get_unlocked(dev, chan, val);
	k_mutex_unlock(&data->fifo_lock);
	return ret;
}

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, i2c) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, i2c)
int lsm6dso_fifo_start_locked_driver(const struct device *dev, uint16_t watermark,
				     bool accel, bool gyro)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;
	int xl, gy, ret;

	/* Other trigger and sensor-hub accesses do not yet use fifo_lock. */
	if (IS_ENABLED(CONFIG_LSM6DSO_TRIGGER) || IS_ENABLED(CONFIG_LSM6DSO_SENSORHUB) ||
	    cfg->ctx.read_reg != (stmdev_read_ptr)stmemsc_i2c_read) {
		return -ENOTSUP;
	}
	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	if (data->fifo_owned) {
		ret = -EBUSY;
		goto out;
	}
	xl = accel ? lsm6dso_freq_to_odr_val(data->accel_freq) : 0;
	gy = gyro ? lsm6dso_freq_to_odr_val(data->gyro_freq) : 0;
	if ((!xl && accel) || (!gy && gyro) || xl < 0 || gy < 0 || (!xl && !gy) ||
	    watermark == 0 || watermark > 511) {
		ret = -EINVAL;
		goto out;
	}
	data->fifo_owned = true;
	memset(&data->fifo_stats, 0, sizeof(data->fifo_stats));
	data->fifo_stats.generation = ++data->fifo_generation;
	data->fifo_stats.accel_hz = accel ? data->accel_freq : 0;
	data->fifo_stats.gyro_hz = gyro ? data->gyro_freq : 0;
	data->fifo_stats.accel_ug_per_lsb = data->acc_gain;
	data->fifo_stats.gyro_udps_per_lsb = data->gyro_gain;
	{
		uint8_t freq_fine = 0;

		if (lsm6dso_odr_cal_reg_get(&cfg->ctx, &freq_fine) == 0) {
			data->fifo_stats.freq_fine = (int8_t)freq_fine;
		}
	}
	ret = lsm6dso_fifo_configure(&cfg->ctx, watermark, xl, gy,
				     data->timestamp_decimation
				     );
	if (ret != 0) {
		data->fifo_stats.i2c_errors++;
	}
	data->fifo_fault = ret != 0;
out:
	k_mutex_unlock(&data->fifo_lock);
	return ret;
}

int lsm6dso_fifo_pause_batch_locked_driver(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	if (!data->fifo_owned || data->fifo_fault) {
		ret = -EIO;
	} else if (lsm6dso_fifo_pause_batch(&cfg->ctx) != 0) {
		data->fifo_stats.i2c_errors++;
		data->fifo_fault = true;
		ret = -EIO;
	}
	k_mutex_unlock(&data->fifo_lock);
	return ret;
}

int lsm6dso_fifo_stop_locked_driver(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	if (data->fifo_owned) {
		ret = lsm6dso_fifo_stop(&cfg->ctx);
		if (ret != 0) {
			data->fifo_stats.i2c_errors++;
		}
		data->fifo_owned = ret != 0;
		data->fifo_fault = ret != 0;
	}
	k_mutex_unlock(&data->fifo_lock);
	return ret;
}

int lsm6dso_fifo_read_locked_driver(const struct device *dev, uint8_t *buffer,
				    size_t capacity_words, struct lsm6dso_fifo_status *status)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;
	int ret;

	if (!status || (!buffer && capacity_words)) {
		return -EINVAL;
	}
	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	if (!data->fifo_owned || data->fifo_fault) {
		ret = -EIO;
		goto out;
	}
	ret = lsm6dso_fifo_status_read(&cfg->stmemsc_cfg.i2c, status);
	if (ret != 0) {
		data->fifo_stats.i2c_errors++;
		data->fifo_fault = true;
		goto out;
	}
	if (status->overrun || status->overrun_latched) {
		data->fifo_stats.overrun_observations++;
	}
	if (status->words > LSM6DSO_FIFO_MAX_WORDS) {
		data->fifo_fault = true;
		ret = -EIO;
	} else if (status->words > capacity_words) {
		data->fifo_stats.buffer_too_small++;
		ret = -ENOBUFS;
	} else if (status->words) {
		uint32_t start = k_cycle_get_32();
		uint64_t duration;

		ret = lsm6dso_fifo_burst_read(&cfg->stmemsc_cfg.i2c, buffer, status->words);
		duration = k_cyc_to_ns_floor64((uint32_t)(k_cycle_get_32() - start));
		data->fifo_stats.transfers++;
		data->fifo_stats.transfer_time_ns += duration;
		data->fifo_stats.max_transfer_time_ns = MAX(data->fifo_stats.max_transfer_time_ns,
							   duration);
		if (ret != 0) {
			data->fifo_stats.i2c_errors++;
			data->fifo_fault = true;
		} else {
			data->fifo_stats.words_read += status->words;
		}
	}
out:
	k_mutex_unlock(&data->fifo_lock);
	return ret;
}

int lsm6dso_fifo_status_locked_driver(const struct device *dev,
				      struct lsm6dso_fifo_status *status)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;
	int ret;

	if (!status) {
		return -EINVAL;
	}
	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	if (!data->fifo_owned || data->fifo_fault) {
		ret = -EIO;
	} else {
		ret = lsm6dso_fifo_status_read(&cfg->stmemsc_cfg.i2c, status);
		if (ret != 0) {
			data->fifo_stats.i2c_errors++;
			data->fifo_fault = true;
		} else if (status->overrun || status->overrun_latched) {
			data->fifo_stats.overrun_observations++;
		}
	}
	k_mutex_unlock(&data->fifo_lock);
	return ret;
}

int lsm6dso_timestamp_read(const struct device *dev, int64_t (*clock_us)(void), uint32_t *tick,
			   int64_t *before_us, int64_t *after_us)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;
	uint8_t first[4], second[4];
	uint32_t a, b;
	int ret;

	if (!clock_us || !tick || !before_us || !after_us) {
		return -EINVAL;
	}
	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	ret = i2c_burst_read_dt(&cfg->stmemsc_cfg.i2c, LSM6DSO_TIMESTAMP0, first, sizeof(first));
	if (ret == 0) {
		*before_us = clock_us();
		ret = i2c_burst_read_dt(&cfg->stmemsc_cfg.i2c, LSM6DSO_TIMESTAMP0, second,
					sizeof(second));
		*after_us = clock_us();
	}
	k_mutex_unlock(&data->fifo_lock);
	if (ret != 0) {
		return ret;
	}
	a = sys_get_le32(first);
	b = sys_get_le32(second);
	/* 400 ticks are 10 ms: far more than two reads take, far less than a
	 * wrong carry into the second byte or above would produce. */
	if ((uint32_t)(b - a) > 400U) {
		return -EAGAIN;
	}
	*tick = b;
	return 0;
}

void lsm6dso_fifo_get_stats(const struct device *dev, struct lsm6dso_fifo_stats *stats)
{
	struct lsm6dso_data *data = dev->data;

	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	*stats = data->fifo_stats;
	stats->active = data->fifo_owned;
	stats->fault = data->fifo_fault;
	k_mutex_unlock(&data->fifo_lock);
}

void lsm6dso_fifo_record_stop_error(const struct device *dev, int error)
{
	struct lsm6dso_data *data = dev->data;

	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	if (error != 0 && data->fifo_stats.stop_error == 0) {
		data->fifo_stats.stop_error = error;
	}
	k_mutex_unlock(&data->fifo_lock);
}
#endif

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
#if CONFIG_LSM6DSO_TRIGGER
	.trigger_set = lsm6dso_trigger_set,
#endif
	.sample_fetch = lsm6dso_sample_fetch,
	.channel_get = lsm6dso_channel_get,
#ifdef CONFIG_SENSOR_ASYNC_API
	.get_decoder = lsm6dso_get_decoder,
	.submit = lsm6dso_submit,
#endif
};

static int lsm6dso_init_chip(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct lsm6dso_data *lsm6dso = dev->data;
	uint8_t chip_id, master_on;
	uint8_t odr, fs;

	/* All registers except 0x01 are different between banks, including the WHO_AM_I
	 * register and the register used for a SW reset.  If the lsm6dso wasn't on the user
	 * bank when it reset, then both the chip id check and the sw reset will fail unless we
	 * set the bank now.
	 */
	if (lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK) < 0) {
		LOG_ERR("Failed to set user bank");
		return -EIO;
	}

	if (lsm6dso_device_id_get(ctx, &chip_id) < 0) {
		LOG_ERR("Failed reading chip id");
		return -EIO;
	}

	LOG_INF("chip id 0x%x", chip_id);

	if (chip_id != LSM6DSO_ID) {
		LOG_ERR("Invalid chip id 0x%x", chip_id);
		return -EIO;
	}

	/* I3C disable stay preserved after s/w reset */
	if (lsm6dso_i3c_disable_set(ctx, LSM6DSO_I3C_DISABLE) < 0) {
		LOG_ERR("Failed to disable I3C");
		return -EIO;
	}

	/* Per AN5192 §7.2.1, "… when applying the software reset procedure, the I2C master
	 * must be disabled, followed by a 300 μs wait."
	 */
	if (lsm6dso_sh_master_get(ctx, &master_on) < 0) {
		LOG_ERR("Failed to get I2C_MASTER status");
		return -EIO;
	}
	if (master_on) {
		LOG_DBG("Disable shub before reset");
		lsm6dso_sh_master_set(ctx, 0);
		k_busy_wait(300);
	}

	/* reset device */
	if (lsm6dso_reset_set(ctx, 1) < 0) {
		LOG_ERR("Failed to reset device");
		return -EIO;
	}

	k_busy_wait(100);

	/* set accel power mode */
	LOG_DBG("accel pm is %d", cfg->accel_pm);
	switch (cfg->accel_pm) {
	default:
	case 0:
		lsm6dso_xl_power_mode_set(ctx, LSM6DSO_HIGH_PERFORMANCE_MD);
		break;
	case 1:
		lsm6dso_xl_power_mode_set(ctx, LSM6DSO_LOW_NORMAL_POWER_MD);
		break;
	case 2:
		lsm6dso_xl_power_mode_set(ctx, LSM6DSO_ULTRA_LOW_POWER_MD);
		break;
	}

	fs = cfg->accel_range & ACCEL_RANGE_MASK;
	LOG_DBG("accel range is %d", fs);
	if (lsm6dso_accel_set_fs_raw(dev, fs) < 0) {
		LOG_ERR("failed to set accelerometer range %d", fs);
		return -EIO;
	}
	lsm6dso->acc_gain = lsm6dso_accel_fs_val_to_gain(fs, cfg->accel_range & ACCEL_RANGE_DOUBLE);

	odr = cfg->accel_odr;
	LOG_DBG("accel odr is %d", odr);
	lsm6dso->accel_freq = lsm6dso_odr_to_freq_val(odr);
	if (lsm6dso_accel_set_odr_raw(dev, odr) < 0) {
		LOG_ERR("failed to set accelerometer odr %d", odr);
		return -EIO;
	}

	/* set gyro power mode */
	LOG_DBG("gyro pm is %d", cfg->gyro_pm);
	switch (cfg->gyro_pm) {
	default:
	case 0:
		lsm6dso_gy_power_mode_set(ctx, LSM6DSO_GY_HIGH_PERFORMANCE);
		break;
	case 1:
		lsm6dso_gy_power_mode_set(ctx, LSM6DSO_GY_NORMAL);
		break;
	}

	fs = cfg->gyro_range;
	LOG_DBG("gyro range is %d", fs);
	if (lsm6dso_gyro_set_fs_raw(dev, fs) < 0) {
		LOG_ERR("failed to set gyroscope range %d", fs);
		return -EIO;
	}
	lsm6dso->gyro_gain = (lsm6dso_gyro_fs_sens[fs] * GAIN_UNIT_G);

	odr = cfg->gyro_odr;
	LOG_DBG("gyro odr is %d", odr);
	lsm6dso->gyro_freq = lsm6dso_odr_to_freq_val(odr);
	if (lsm6dso_gyro_set_odr_raw(dev, odr) < 0) {
		LOG_ERR("failed to set gyroscope odr %d", odr);
		return -EIO;
	}

	/* Set FIFO bypass mode */
	if (lsm6dso_fifo_mode_set(ctx, LSM6DSO_BYPASS_MODE) < 0) {
		LOG_ERR("failed to set FIFO mode");
		return -EIO;
	}

	if (lsm6dso_block_data_update_set(ctx, 1) < 0) {
		LOG_ERR("failed to set BDU mode");
		return -EIO;
	}

	if (cfg->accel_lp_filter) {
		if (lsm6dso_xl_filter_lp2_set(ctx, 1)) {
			LOG_ERR("failed to enable low pass filter (LPF2)");
			return -EIO;
		}

		if (lsm6dso_xl_hp_path_on_out_set(ctx,
				lsm6dso_lp_filter_map[cfg->accel_lp_filter])) {
			LOG_ERR("failed to configure low pass filter (LPF2)");
			return -EIO;
		}
	}

	return 0;
}

static int lsm6dso_init(const struct device *dev)
{
#ifdef CONFIG_LSM6DSO_TRIGGER
	const struct lsm6dso_config *cfg = dev->config;
#endif
	struct lsm6dso_data *data = dev->data;

	LOG_INF("Initialize device %s", dev->name);
	data->dev = dev;
	k_mutex_init(&data->fifo_lock);
#ifdef CONFIG_SENSOR_ASYNC_API
	lsm6dso_stream_init(dev);
#endif

	if (lsm6dso_init_chip(dev) < 0) {
		LOG_ERR("failed to initialize chip");
		return -EIO;
	}

#ifdef CONFIG_LSM6DSO_TRIGGER
	if (cfg->trig_enabled) {
		if (lsm6dso_init_interrupt(dev) < 0) {
			LOG_ERR("Failed to initialize interrupt.");
			return -EIO;
		}
	}
#endif

#ifdef CONFIG_LSM6DSO_SENSORHUB
	data->shub_inited = true;
	if (lsm6dso_shub_init(dev) < 0) {
		LOG_INF("shub: no external chips found");
		data->shub_inited = false;
	}
#endif

	return 0;
}

/*
 * Device creation macro, shared by LSM6DSO_DEFINE_SPI() and
 * LSM6DSO_DEFINE_I2C().
 */

#define LSM6DSO_DEVICE_INIT(inst, model)				\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			    lsm6dso_init,				\
			    NULL,					\
			    &model##_data_##inst,			\
			    &model##_config_##inst,			\
			    POST_KERNEL,				\
			    CONFIG_SENSOR_INIT_PRIORITY,		\
			    &lsm6dso_driver_api);

/*
 * Instantiation macros used when a device is on a SPI bus.
 */

#ifdef CONFIG_LSM6DSO_TAP
#define LSM6DSO_CONFIG_TAP(inst)					\
	.tap_mode = DT_INST_PROP(inst, tap_mode),			\
	.tap_threshold = DT_INST_PROP(inst, tap_threshold),		\
	.tap_shock = DT_INST_PROP(inst, tap_shock),			\
	.tap_latency = DT_INST_PROP(inst, tap_latency),			\
	.tap_quiet = DT_INST_PROP(inst, tap_quiet),
#else
#define LSM6DSO_CONFIG_TAP(inst)
#endif /* CONFIG_LSM6DSO_TAP */

#ifdef CONFIG_LSM6DSO_TRIGGER
#define LSM6DSO_CFG_IRQ(inst)						\
	.trig_enabled = true,						\
	.gpio_drdy = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),		\
	.int_pin = DT_INST_PROP(inst, int_pin)
#else
#define LSM6DSO_CFG_IRQ(inst)
#endif /* CONFIG_LSM6DSO_TRIGGER */

#define LSM6DSO_SPI_OP  (SPI_WORD_SET(8) |				\
			 SPI_OP_MODE_CONTROLLER |			\
			 SPI_MODE_CPOL |				\
			 SPI_MODE_CPHA)					\

#define LSM6DSO_CONFIG_COMMON(inst)					\
	.accel_pm = DT_INST_PROP(inst, accel_pm),			\
	.accel_odr = DT_INST_PROP(inst, accel_odr),			\
	.accel_lp_filter = DT_INST_PROP(inst, accel_lp_filter),		\
	.accel_range = DT_INST_PROP(inst, accel_range) |		\
		(DT_INST_NODE_HAS_COMPAT(inst, st_lsm6dso32) ?	        \
			ACCEL_RANGE_DOUBLE : 0),			\
	.gyro_pm = DT_INST_PROP(inst, gyro_pm),				\
	.gyro_odr = DT_INST_PROP(inst, gyro_odr),			\
	.gyro_range = DT_INST_PROP(inst, gyro_range),			\
	.drdy_pulsed = DT_INST_PROP(inst, drdy_pulsed),			\
	IF_ENABLED(CONFIG_SENSOR_ASYNC_API, (.fifo_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, {0}),)) \
	LSM6DSO_CONFIG_TAP(inst)					\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, irq_gpios),		\
		(LSM6DSO_CFG_IRQ(inst)), ())

#define LSM6DSO_CONFIG_SPI(inst, model)					\
	{								\
		STMEMSC_CTX_SPI(&model##_config_##inst.stmemsc_cfg),	\
		.stmemsc_cfg = {					\
			.spi = SPI_DT_SPEC_INST_GET(inst,		\
					   LSM6DSO_SPI_OP),		\
		},							\
		LSM6DSO_CONFIG_COMMON(inst)				\
	}

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define LSM6DSO_CONFIG_I2C(inst, model)					\
	{								\
		STMEMSC_CTX_I2C(&model##_config_##inst.stmemsc_cfg),	\
		.stmemsc_cfg = {					\
			.i2c = I2C_DT_SPEC_INST_GET(inst),		\
		},							\
		LSM6DSO_CONFIG_COMMON(inst)				\
	}

/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define LSM6DSO_DEFINE(inst, model)					\
	static struct lsm6dso_data model##_data_##inst;			\
	static const struct lsm6dso_config model##_config_##inst =	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),			\
			(LSM6DSO_CONFIG_SPI(inst, model)),		\
			(LSM6DSO_CONFIG_I2C(inst, model)));		\
	LSM6DSO_DEVICE_INIT(inst, model)

#define DT_DRV_COMPAT st_lsm6dso
DT_INST_FOREACH_STATUS_OKAY_VARGS(LSM6DSO_DEFINE, lsm6dso)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT st_lsm6dso32
DT_INST_FOREACH_STATUS_OKAY_VARGS(LSM6DSO_DEFINE, lsm6dso32)
#undef DT_DRV_COMPAT
