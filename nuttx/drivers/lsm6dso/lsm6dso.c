/* ST Microelectronics LSM6DSO 6-axis IMU sensor driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 */

/* The Zephyr build's drivers/lsm6dso/lsm6dso.c (Zephyr's driver with the
 * FIFO, calibration and zero-rate changes of zephyr/app/usb-console), with
 * the same functions, branches, order, register operations, conversions,
 * error values and log lines.  Only the calls into the system are
 * replaced:
 *
 *   Zephyr                              here
 *   <zephyr/logging/log.h>              app_log.h (the same LOG_* lines in
 *                                       the RAM log, "dmesg")
 *   CONFIG_SENSOR_LOG_LEVEL (3)         LSM6DSO_LOG_LEVEL (3)
 *   k_mutex_init/_lock/_unlock          lsm6dso_os_mutex_init/_lock/_unlock
 *   k_spin_lock/_unlock                 lsm6dso_os_spin_lock/_unlock
 *   k_busy_wait, k_msleep, k_usleep     lsm6dso_os_busy_wait_us, _sleep_ms,
 *                                       _sleep_us
 *   k_cycle_get_32, k_cyc_to_ns_floor64 lsm6dso_os_cycle_get_32 (TIMER,
 *                                       1 MHz), _cyc_to_ns_floor64
 *   sys_get_le32()                      the same, written out below
 *   CONFIG_SENSOR_ASYNC_API             CONFIG_NANO_LSM6DSO_STREAM (T11),
 *                                       also around the include of
 *                                       lsm6dso_fifo_decoder.h
 *   #if CONFIG_LSM6DSO_TRIGGER          #ifdef (-Wundef)
 *   DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()  LSM6DSO_BUS_I2C (lsm6dso.h)
 *   the devicetree instance macros      one DEVICE_DT_DEFINE() at the end,
 *                                       with the board's values
 *
 * The device model, the sensor API and the I2C calls keep Zephyr's names:
 * struct device, struct sensor_value, i2c_burst_read_dt() and so on come
 * from the port's compatibility layer (nuttx/drivers/compat), and
 * stmemsc_i2c_read()/_write() from stmemsc_i2c.c.  The board initializes
 * the device in its bring-up (boards/.../src/nano_lsm6dso.c, device_init()),
 * where Zephyr does it before main() (POST_KERNEL,
 * CONFIG_SENSOR_INIT_PRIORITY 90).
 */

#include <nuttx/config.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <arch/board/board.h>

/* CONFIG_SENSOR_LOG_LEVEL of the Zephyr build (3, info). */
#define LSM6DSO_LOG_LEVEL 3
#include "app_log.h"

#include "lsm6dso.h"
#include "lsm6dso_fifo.h"
#include "lsm6dso_attributes.h"
#ifdef CONFIG_NANO_LSM6DSO_STREAM
#include "lsm6dso_fifo_decoder.h"
#endif
#include "lsm6dso_fifo_hw.h"

LOG_MODULE_REGISTER(LSM6DSO, LSM6DSO_LOG_LEVEL);

/* zephyr/sys/byteorder.h */
static inline uint32_t sys_get_le32(const uint8_t src[4])
{
	return ((uint32_t)src[3] << 24) | ((uint32_t)src[2] << 16) |
	       ((uint32_t)src[1] << 8) | src[0];
}

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

static void lsm6dso_cal_refresh(const struct device *dev, enum lsm6dso_cal_kind kind);
static void lsm6dso_zero_restart(const struct device *dev);

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
	lsm6dso_cal_refresh(dev, LSM6DSO_CAL_ACCEL);
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
	lsm6dso_cal_refresh(dev, LSM6DSO_CAL_GYRO);
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

/* Selects the conversion for the current sensitivity: the stored calibration
 * if the lookup has one, the nominal sensitivity otherwise.
 */
static void lsm6dso_cal_refresh(const struct device *dev, enum lsm6dso_cal_kind kind)
{
	struct lsm6dso_data *data = dev->data;
	uint32_t gain = kind == LSM6DSO_CAL_ACCEL ? data->acc_gain : data->gyro_gain;
	struct lsm6dso_cal cal;
	bool calibrated = data->cal_lookup != NULL && data->cal_lookup(kind, gain, &cal);
	lsm6dso_spinlock_key_t key;

	if (!calibrated) {
		lsm6dso_cal_nominal(&cal, kind, gain);
	}
	key = lsm6dso_os_spin_lock(&data->cal_lock);
	if (kind == LSM6DSO_CAL_ACCEL) {
		data->accel_cal = cal;
		data->accel_calibrated = calibrated;
	} else {
		data->gyro_cal = cal;
		data->gyro_calibrated = calibrated;
	}
	lsm6dso_os_spin_unlock(&data->cal_lock, key);
	lsm6dso_zero_restart(dev);
}

/* Forgets the tracked gyro bias. Its limits are in LSB and its reference is
 * the stored bias, so it starts over when either sensitivity or the stored
 * calibration changes.
 */
static void lsm6dso_zero_restart(const struct device *dev)
{
	struct lsm6dso_data *data = dev->data;
	struct lsm6dso_zero_limits limits;
	struct lsm6dso_cal accel, gyro;

	lsm6dso_cal_snapshot(dev, &accel, &gyro);
	lsm6dso_zero_limits(&limits, data->gyro_gain, data->acc_gain, data->gyro_calibrated);
	lsm6dso_os_mutex_lock(&data->zero_lock);
	lsm6dso_zero_reset(&data->zero, &limits, data->gyro_calibrated ? gyro.bias : NULL);
	lsm6dso_os_mutex_unlock(&data->zero_lock);
}

/* Puts a newly tracked bias into the gyro conversion. Call with zero_lock held. */
static void lsm6dso_zero_apply(const struct device *dev)
{
	struct lsm6dso_data *data = dev->data;
	lsm6dso_spinlock_key_t key = lsm6dso_os_spin_lock(&data->cal_lock);

	memcpy(data->gyro_cal.bias, data->zero.bias, sizeof(data->gyro_cal.bias));
	lsm6dso_os_spin_unlock(&data->cal_lock, key);
}

void lsm6dso_cal_track(const struct device *dev, const uint8_t *words, size_t len)
{
	struct lsm6dso_data *data = dev->data;
	struct lsm6dso_fifo_sample sample;
	size_t cursor = 0;
	bool changed = false;

	lsm6dso_os_mutex_lock(&data->zero_lock);
	lsm6dso_zero_set_rate(&data->zero, data->gyro_freq);
	while (lsm6dso_fifo_next(words, len, &cursor, &sample) > 0) {
		if (sample.kind == LSM6DSO_FIFO_ACCEL) {
			lsm6dso_zero_accel(&data->zero, sample.xyz);
		} else if (lsm6dso_zero_gyro(&data->zero, sample.xyz)) {
			changed = true;
		}
	}
	if (changed) {
		lsm6dso_zero_apply(dev);
	}
	lsm6dso_os_mutex_unlock(&data->zero_lock);
}

void lsm6dso_cal_snapshot(const struct device *dev, struct lsm6dso_cal *accel,
			  struct lsm6dso_cal *gyro)
{
	struct lsm6dso_data *data = dev->data;
	lsm6dso_spinlock_key_t key = lsm6dso_os_spin_lock(&data->cal_lock);

	*accel = data->accel_cal;
	*gyro = data->gyro_cal;
	lsm6dso_os_spin_unlock(&data->cal_lock, key);
}

void lsm6dso_cal_reload(const struct device *dev)
{
	lsm6dso_cal_refresh(dev, LSM6DSO_CAL_ACCEL);
	lsm6dso_cal_refresh(dev, LSM6DSO_CAL_GYRO);
}

void lsm6dso_cal_set_lookup(const struct device *dev, lsm6dso_cal_lookup_t lookup)
{
	struct lsm6dso_data *data = dev->data;

	data->cal_lookup = lookup;
	lsm6dso_cal_reload(dev);
}

void lsm6dso_cal_status(const struct device *dev, struct lsm6dso_cal_status *status)
{
	struct lsm6dso_data *data = dev->data;
	lsm6dso_spinlock_key_t key = lsm6dso_os_spin_lock(&data->cal_lock);

	status->accel_gain = data->acc_gain;
	status->gyro_gain = data->gyro_gain;
	status->accel_calibrated = data->accel_calibrated;
	status->gyro_calibrated = data->gyro_calibrated;
	memcpy(status->gyro_bias, data->gyro_cal.bias, sizeof(status->gyro_bias));
	lsm6dso_os_spin_unlock(&data->cal_lock, key);
	lsm6dso_os_mutex_lock(&data->zero_lock);
	status->gyro_bias_updates = data->zero.updates;
	status->gyro_bias_rejected = data->zero.rejected;
	lsm6dso_os_mutex_unlock(&data->zero_lock);
}

/* Converts the last fetched sample with the same function the FIFO decoder
 * uses.  A single axis still needs all three raw values, because the
 * calibration mixes the axes.
 */
static int lsm6dso_converted_channel_get(const struct device *dev, enum sensor_channel chan,
					 struct sensor_value *val, bool accel)
{
	struct lsm6dso_data *data = dev->data;
	struct lsm6dso_cal accel_cal, gyro_cal;
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
	lsm6dso_cal_snapshot(dev, &accel_cal, &gyro_cal);
	lsm6dso_cal_apply(accel ? &accel_cal : &gyro_cal, accel ? data->acc : data->gyro, out);
	for (int axis = first; axis <= last; axis++) {
		int64_t micro = lsm6dso_cal_to_micro(out[axis]);

		val->val1 = (int32_t)(micro / 1000000);
		val->val2 = (int32_t)(micro % 1000000);
		val++;
	}
	return 0;
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
	__maybe_unused struct lsm6dso_data *data = dev->data;

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

#ifdef CONFIG_NANO_LSM6DSO_STREAM
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
	lsm6dso_os_mutex_lock(&data->fifo_lock);
	ret = data->fifo_owned ? -EBUSY : lsm6dso_attr_set_unlocked(dev, chan, attr, val);
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
	return ret;
}

#ifdef CONFIG_NANO_LSM6DSO_STREAM
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

	lsm6dso_os_mutex_lock(&data->fifo_lock);
	ret = data->fifo_owned ? -EBUSY : lsm6dso_sample_fetch_unlocked(dev, chan);
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
	return ret;
}

/* Rate and duration of the on-demand bias reading. */
#define ZERO_POLL_HZ      104
#define ZERO_SETTLE_MS    200
#define ZERO_SECONDS      3

int lsm6dso_cal_zero(const struct device *dev)
{
	struct lsm6dso_data *data = dev->data;
	uint16_t accel_freq, gyro_freq;
	uint32_t updates, rejected;
	int ret = 0;

	lsm6dso_os_mutex_lock(&data->fifo_lock);
	if (data->fifo_owned) {
		lsm6dso_os_mutex_unlock(&data->fifo_lock);
		return -EBUSY;
	}
	accel_freq = data->accel_freq;
	gyro_freq = data->gyro_freq;
	/* A sensor that is off is run for the reading and switched off again; one
	 * that is on keeps its rate, and is read at the polling rate.
	 */
	if (accel_freq == 0) {
		ret = lsm6dso_accel_set_odr_raw(dev, lsm6dso_freq_to_odr_val(ZERO_POLL_HZ));
	}
	if (ret == 0 && gyro_freq == 0) {
		ret = lsm6dso_gyro_set_odr_raw(dev, lsm6dso_freq_to_odr_val(ZERO_POLL_HZ));
	}
	if (ret == 0) {
		lsm6dso_os_sleep_ms(ZERO_SETTLE_MS);
		lsm6dso_os_mutex_lock(&data->zero_lock);
		lsm6dso_zero_set_rate(&data->zero, 0);
		lsm6dso_zero_set_rate(&data->zero, ZERO_POLL_HZ);
		updates = data->zero.updates;
		rejected = data->zero.rejected;
		for (int i = 0; i < ZERO_SECONDS * ZERO_POLL_HZ; i++) {
			ret = lsm6dso_sample_fetch_accel(dev);
			if (ret == 0) {
				ret = lsm6dso_sample_fetch_gyro(dev);
			}
			if (ret != 0) {
				break;
			}
			lsm6dso_zero_accel(&data->zero, data->acc);
			if (lsm6dso_zero_gyro(&data->zero, data->gyro)) {
				lsm6dso_zero_apply(dev);
				break;
			}
			lsm6dso_os_sleep_us(USEC_PER_SEC / ZERO_POLL_HZ);
		}
		if (ret == 0 && data->zero.updates == updates) {
			ret = data->zero.rejected != rejected ? -ERANGE : -EAGAIN;
		}
		/* The next stream sets its own rate and starts a fresh window. */
		lsm6dso_zero_set_rate(&data->zero, 0);
		lsm6dso_os_mutex_unlock(&data->zero_lock);
	}
	if (accel_freq == 0) {
		(void)lsm6dso_accel_set_odr_raw(dev, 0);
	}
	if (gyro_freq == 0) {
		(void)lsm6dso_gyro_set_odr_raw(dev, 0);
	}
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
	return ret;
}

static int lsm6dso_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct lsm6dso_data *data = dev->data;
	int ret;

	lsm6dso_os_mutex_lock(&data->fifo_lock);
	ret = data->fifo_owned ? -EBUSY : lsm6dso_channel_get_unlocked(dev, chan, val);
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
	return ret;
}

#if LSM6DSO_BUS_I2C
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
	lsm6dso_os_mutex_lock(&data->fifo_lock);
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
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
	return ret;
}

int lsm6dso_fifo_pause_batch_locked_driver(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;
	int ret = 0;

	lsm6dso_os_mutex_lock(&data->fifo_lock);
	if (!data->fifo_owned || data->fifo_fault) {
		ret = -EIO;
	} else if (lsm6dso_fifo_pause_batch(&cfg->ctx) != 0) {
		data->fifo_stats.i2c_errors++;
		data->fifo_fault = true;
		ret = -EIO;
	}
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
	return ret;
}

int lsm6dso_fifo_stop_locked_driver(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;
	int ret = 0;

	lsm6dso_os_mutex_lock(&data->fifo_lock);
	if (data->fifo_owned) {
		ret = lsm6dso_fifo_stop(&cfg->ctx);
		if (ret != 0) {
			data->fifo_stats.i2c_errors++;
		}
		data->fifo_owned = ret != 0;
		data->fifo_fault = ret != 0;
	}
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
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
	lsm6dso_os_mutex_lock(&data->fifo_lock);
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
		uint32_t start = lsm6dso_os_cycle_get_32();
		uint64_t duration;

		ret = lsm6dso_fifo_burst_read(&cfg->stmemsc_cfg.i2c, buffer, status->words);
		duration = lsm6dso_os_cyc_to_ns_floor64((uint32_t)(lsm6dso_os_cycle_get_32() - start));
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
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
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
	lsm6dso_os_mutex_lock(&data->fifo_lock);
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
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
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
	lsm6dso_os_mutex_lock(&data->fifo_lock);
	ret = i2c_burst_read_dt(&cfg->stmemsc_cfg.i2c, LSM6DSO_TIMESTAMP0, first, sizeof(first));
	if (ret == 0) {
		*before_us = clock_us();
		ret = i2c_burst_read_dt(&cfg->stmemsc_cfg.i2c, LSM6DSO_TIMESTAMP0, second,
					sizeof(second));
		*after_us = clock_us();
	}
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
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

	lsm6dso_os_mutex_lock(&data->fifo_lock);
	*stats = data->fifo_stats;
	stats->active = data->fifo_owned;
	stats->fault = data->fifo_fault;
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
}

void lsm6dso_fifo_record_stop_error(const struct device *dev, int error)
{
	struct lsm6dso_data *data = dev->data;

	lsm6dso_os_mutex_lock(&data->fifo_lock);
	if (error != 0 && data->fifo_stats.stop_error == 0) {
		data->fifo_stats.stop_error = error;
	}
	lsm6dso_os_mutex_unlock(&data->fifo_lock);
}
#endif

#ifdef CONFIG_NANO_LSM6DSO_STREAM
static int lsm6dso_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &lsm6dso_combined_decoder;
	return 0;
}
#endif

static DEVICE_API(sensor, lsm6dso_driver_api) = {
	.attr_set = lsm6dso_attr_set,
#ifdef CONFIG_NANO_LSM6DSO_STREAM
	.attr_get = lsm6dso_attr_get,
#endif
#ifdef CONFIG_LSM6DSO_TRIGGER
	.trigger_set = lsm6dso_trigger_set,
#endif
	.sample_fetch = lsm6dso_sample_fetch,
	.channel_get = lsm6dso_channel_get,
#ifdef CONFIG_NANO_LSM6DSO_STREAM
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
		lsm6dso_os_busy_wait_us(300);
	}

	/* reset device */
	if (lsm6dso_reset_set(ctx, 1) < 0) {
		LOG_ERR("Failed to reset device");
		return -EIO;
	}

	lsm6dso_os_busy_wait_us(100);

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
	lsm6dso_cal_refresh(dev, LSM6DSO_CAL_ACCEL);

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
	lsm6dso_cal_refresh(dev, LSM6DSO_CAL_GYRO);

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
	lsm6dso_os_mutex_init(&data->fifo_lock);
	lsm6dso_os_mutex_init(&data->zero_lock);
#ifdef CONFIG_NANO_LSM6DSO_STREAM
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
 * The device: the devicetree node lsm6dsox@6a of the Zephyr build
 * (zephyr/build/usb-console/zephyr/zephyr.dts: the board's
 * arduino_nano_connect.dts with app/usb-console/app.overlay), compatible
 * "st,lsm6dso" (not st,lsm6dso32: no ACCEL_RANGE_DOUBLE), on i2c0 at 0x6a.
 * The board's <arch/board/board.h> carries the property values; the rest
 * are the binding's defaults (st,lsm6dso-common.yaml).  Zephyr's
 * LSM6DSO_CONFIG_COMMON() and LSM6DSO_CONFIG_I2C() for that node:
 */

static struct lsm6dso_data lsm6dso_data_0;
static const struct lsm6dso_config lsm6dso_config_0 = {
	STMEMSC_CTX_I2C(&lsm6dso_config_0.stmemsc_cfg),
	.stmemsc_cfg = {
		.i2c = {
			.bus = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm6dsox))),
			.addr = BOARD_LSM6DSOX_I2C_ADDR,
		},
	},
	.accel_pm = BOARD_LSM6DSOX_ACCEL_PM,
	.accel_odr = BOARD_LSM6DSOX_ACCEL_ODR,
	.accel_lp_filter = BOARD_LSM6DSOX_ACCEL_LP_FILTER,
	.accel_range = BOARD_LSM6DSOX_ACCEL_RANGE,
	.gyro_pm = BOARD_LSM6DSOX_GYRO_PM,
	.gyro_odr = BOARD_LSM6DSOX_GYRO_ODR,
	.gyro_range = BOARD_LSM6DSOX_GYRO_RANGE,
	.drdy_pulsed = BOARD_LSM6DSOX_DRDY_PULSED,
	/* CONFIG_NANO_LSM6DSO_STREAM: .fifo_gpio, irq-gpios (the stream, T11) */
};

/* SENSOR_DEVICE_DT_INST_DEFINE(): the init function, data, config and API;
 * POST_KERNEL and CONFIG_SENSOR_INIT_PRIORITY are the board's order of
 * calls.
 */
DEVICE_DT_DEFINE(lsm6dsox, BOARD_LSM6DSOX_NODE_NAME, lsm6dso_init,
		 &lsm6dso_data_0, &lsm6dso_config_0, &lsm6dso_driver_api);
