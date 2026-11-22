/* ST Microelectronics LSM6DSO 6-axis IMU sensor driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 */

/* The driver's core (lsm6dso_chip.h): the part of Zephyr's lsm6dso.c, with
 * this driver's FIFO and calibration, that operates the chip, on a struct
 * lsm6dso_chip in place of the device.  The binding of each OS
 * (port/<OS>/lsm6dso.c) keeps the device and the sensor API.
 */

#include "lsm6dso_os.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "lsm6dso_chip.h"

LOG_MODULE_DECLARE(LSM6DSO, LSM6DSO_OS_LOG_LEVEL);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif

/* Accel sensor sensitivity grain is 61 ug/LSB */
#define GAIN_UNIT_XL				(61LL)

/* Gyro sensor sensitivity grain is 4.375 udps/LSB */
#define GAIN_UNIT_G				(4375LL)

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

static void lsm6dso_cal_refresh(struct lsm6dso_chip *chip, enum lsm6dso_cal_kind kind);
static void lsm6dso_zero_restart(struct lsm6dso_chip *chip);

static int lsm6dso_accel_set_fs_raw(struct lsm6dso_chip *chip, uint8_t fs)
{
	if (lsm6dso_xl_full_scale_set(chip->ctx, fs) < 0) {
		return -EIO;
	}

	chip->accel_fs = fs;

	return 0;
}

static int lsm6dso_accel_set_odr_raw(struct lsm6dso_chip *chip, uint8_t odr)
{
	if (lsm6dso_xl_data_rate_set(chip->ctx, odr) < 0) {
		return -EIO;
	}

	chip->accel_freq = lsm6dso_odr_to_freq_val(odr);

	return 0;
}

static int lsm6dso_gyro_set_fs_raw(struct lsm6dso_chip *chip, uint8_t fs)
{
	if (lsm6dso_gy_full_scale_set(chip->ctx, fs) < 0) {
		return -EIO;
	}

	return 0;
}

static int lsm6dso_gyro_set_odr_raw(struct lsm6dso_chip *chip, uint8_t odr)
{
	if (lsm6dso_gy_data_rate_set(chip->ctx, odr) < 0) {
		return -EIO;
	}
	chip->gyro_freq = lsm6dso_odr_to_freq_val(odr);

	return 0;
}

int lsm6dso_chip_accel_odr_set(struct lsm6dso_chip *chip, uint16_t freq)
{
	int odr;

	odr = lsm6dso_freq_to_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	if (lsm6dso_accel_set_odr_raw(chip, odr) < 0) {
		LOG_DBG("failed to set accelerometer sampling rate");
		return -EIO;
	}

	return 0;
}

int lsm6dso_chip_accel_range_set(struct lsm6dso_chip *chip, int32_t range)
{
	int fs;
	bool range_double = chip->accel_range_double;

	fs = lsm6dso_accel_range_to_fs_val(range, range_double);
	if (fs < 0) {
		return fs;
	}

	if (lsm6dso_accel_set_fs_raw(chip, fs) < 0) {
		LOG_DBG("failed to set accelerometer full-scale");
		return -EIO;
	}

	chip->acc_gain = lsm6dso_accel_fs_val_to_gain(fs, range_double);
	lsm6dso_cal_refresh(chip, LSM6DSO_CAL_ACCEL);
	return 0;
}

int lsm6dso_chip_gyro_odr_set(struct lsm6dso_chip *chip, uint16_t freq)
{
	int odr;

	odr = lsm6dso_freq_to_odr_val(freq);
	if (odr < 0) {
		return odr;
	}

	if (lsm6dso_gyro_set_odr_raw(chip, odr) < 0) {
		LOG_DBG("failed to set gyroscope sampling rate");
		return -EIO;
	}

	return 0;
}

int lsm6dso_chip_gyro_range_set(struct lsm6dso_chip *chip, int32_t range)
{
	int fs;

	fs = lsm6dso_gyro_range_to_fs_val(range);
	if (fs < 0) {
		return fs;
	}

	if (lsm6dso_gyro_set_fs_raw(chip, fs) < 0) {
		LOG_DBG("failed to set gyroscope full-scale");
		return -EIO;
	}

	chip->gyro_gain = (lsm6dso_gyro_fs_sens[fs] * GAIN_UNIT_G);
	lsm6dso_cal_refresh(chip, LSM6DSO_CAL_GYRO);
	return 0;
}

int lsm6dso_chip_fetch_accel(struct lsm6dso_chip *chip)
{
	if (lsm6dso_acceleration_raw_get(chip->ctx, chip->acc) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	return 0;
}

int lsm6dso_chip_fetch_gyro(struct lsm6dso_chip *chip)
{
	if (lsm6dso_angular_rate_raw_get(chip->ctx, chip->gyro) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	return 0;
}

/* Selects the conversion for the current sensitivity: the stored calibration
 * if the lookup has one, the nominal sensitivity otherwise.
 */
static void lsm6dso_cal_refresh(struct lsm6dso_chip *chip, enum lsm6dso_cal_kind kind)
{
	uint32_t gain = kind == LSM6DSO_CAL_ACCEL ? chip->acc_gain : chip->gyro_gain;
	struct lsm6dso_cal cal;
	bool calibrated = chip->cal_lookup != NULL && chip->cal_lookup(kind, gain, &cal);
	lsm6dso_os_spinlock_key_t key;

	if (!calibrated) {
		lsm6dso_cal_nominal(&cal, kind, gain);
	}
	key = lsm6dso_os_spin_lock(&chip->cal_lock);
	if (kind == LSM6DSO_CAL_ACCEL) {
		chip->accel_cal = cal;
		chip->accel_calibrated = calibrated;
	} else {
		chip->gyro_cal = cal;
		chip->gyro_calibrated = calibrated;
	}
	lsm6dso_os_spin_unlock(&chip->cal_lock, key);
	lsm6dso_zero_restart(chip);
}

/* Forgets the tracked gyro bias. Its limits are in LSB and its reference is
 * the stored bias, so it starts over when either sensitivity or the stored
 * calibration changes.
 */
static void lsm6dso_zero_restart(struct lsm6dso_chip *chip)
{
	struct lsm6dso_zero_limits limits;
	struct lsm6dso_cal accel, gyro;

	lsm6dso_chip_cal_snapshot(chip, &accel, &gyro);
	lsm6dso_zero_limits(&limits, chip->gyro_gain, chip->acc_gain, chip->gyro_calibrated);
	lsm6dso_os_mutex_lock(&chip->zero_lock);
	lsm6dso_zero_reset(&chip->zero, &limits, chip->gyro_calibrated ? gyro.bias : NULL);
	lsm6dso_os_mutex_unlock(&chip->zero_lock);
}

/* Puts a newly tracked bias into the gyro conversion. Call with zero_lock held. */
static void lsm6dso_zero_apply(struct lsm6dso_chip *chip)
{
	lsm6dso_os_spinlock_key_t key = lsm6dso_os_spin_lock(&chip->cal_lock);

	memcpy(chip->gyro_cal.bias, chip->zero.bias, sizeof(chip->gyro_cal.bias));
	lsm6dso_os_spin_unlock(&chip->cal_lock, key);
}

void lsm6dso_chip_cal_track(struct lsm6dso_chip *chip, const uint8_t *words, size_t len)
{
	struct lsm6dso_fifo_sample sample;
	size_t cursor = 0;
	bool changed = false;

	lsm6dso_os_mutex_lock(&chip->zero_lock);
	lsm6dso_zero_set_rate(&chip->zero, chip->gyro_freq);
	while (lsm6dso_fifo_next(words, len, &cursor, &sample) > 0) {
		if (sample.kind == LSM6DSO_FIFO_ACCEL) {
			lsm6dso_zero_accel(&chip->zero, sample.xyz);
		} else if (lsm6dso_zero_gyro(&chip->zero, sample.xyz)) {
			changed = true;
		}
	}
	if (changed) {
		lsm6dso_zero_apply(chip);
	}
	lsm6dso_os_mutex_unlock(&chip->zero_lock);
}

void lsm6dso_chip_cal_snapshot(struct lsm6dso_chip *chip, struct lsm6dso_cal *accel,
			       struct lsm6dso_cal *gyro)
{
	lsm6dso_os_spinlock_key_t key = lsm6dso_os_spin_lock(&chip->cal_lock);

	*accel = chip->accel_cal;
	*gyro = chip->gyro_cal;
	lsm6dso_os_spin_unlock(&chip->cal_lock, key);
}

void lsm6dso_chip_cal_reload(struct lsm6dso_chip *chip)
{
	lsm6dso_cal_refresh(chip, LSM6DSO_CAL_ACCEL);
	lsm6dso_cal_refresh(chip, LSM6DSO_CAL_GYRO);
}

void lsm6dso_chip_cal_set_lookup(struct lsm6dso_chip *chip, lsm6dso_cal_lookup_t lookup)
{
	chip->cal_lookup = lookup;
	lsm6dso_chip_cal_reload(chip);
}

void lsm6dso_chip_cal_status(struct lsm6dso_chip *chip, struct lsm6dso_cal_status *status)
{
	lsm6dso_os_spinlock_key_t key = lsm6dso_os_spin_lock(&chip->cal_lock);

	status->accel_gain = chip->acc_gain;
	status->gyro_gain = chip->gyro_gain;
	status->accel_calibrated = chip->accel_calibrated;
	status->gyro_calibrated = chip->gyro_calibrated;
	memcpy(status->gyro_bias, chip->gyro_cal.bias, sizeof(status->gyro_bias));
	lsm6dso_os_spin_unlock(&chip->cal_lock, key);
	lsm6dso_os_mutex_lock(&chip->zero_lock);
	status->gyro_bias_updates = chip->zero.updates;
	status->gyro_bias_rejected = chip->zero.rejected;
	lsm6dso_os_mutex_unlock(&chip->zero_lock);
}

/* Converts the last fetched sample with the same function the FIFO decoder
 * uses.  A single axis still needs all three raw values, because the
 * calibration mixes the axes.
 */
void lsm6dso_chip_convert(struct lsm6dso_chip *chip, enum lsm6dso_cal_kind kind,
			  int32_t out[3])
{
	struct lsm6dso_cal accel_cal, gyro_cal;
	bool accel = kind == LSM6DSO_CAL_ACCEL;

	lsm6dso_chip_cal_snapshot(chip, &accel_cal, &gyro_cal);
	lsm6dso_cal_apply(accel ? &accel_cal : &gyro_cal, accel ? chip->acc : chip->gyro, out);
}

/* Rate and duration of the on-demand bias reading. */
#define ZERO_POLL_HZ      104
#define ZERO_SETTLE_MS    200
#define ZERO_SECONDS      3
#define ZERO_POLL_US      (1000000 / ZERO_POLL_HZ)

int lsm6dso_chip_cal_zero(struct lsm6dso_chip *chip)
{
	uint16_t accel_freq, gyro_freq;
	uint32_t updates, rejected;
	int ret = 0;

	lsm6dso_os_mutex_lock(&chip->fifo_lock);
	if (chip->fifo_owned) {
		lsm6dso_os_mutex_unlock(&chip->fifo_lock);
		return -EBUSY;
	}
	accel_freq = chip->accel_freq;
	gyro_freq = chip->gyro_freq;
	/* A sensor that is off is run for the reading and switched off again; one
	 * that is on keeps its rate, and is read at the polling rate.
	 */
	if (accel_freq == 0) {
		ret = lsm6dso_accel_set_odr_raw(chip, lsm6dso_freq_to_odr_val(ZERO_POLL_HZ));
	}
	if (ret == 0 && gyro_freq == 0) {
		ret = lsm6dso_gyro_set_odr_raw(chip, lsm6dso_freq_to_odr_val(ZERO_POLL_HZ));
	}
	if (ret == 0) {
		lsm6dso_os_msleep(ZERO_SETTLE_MS);
		lsm6dso_os_mutex_lock(&chip->zero_lock);
		lsm6dso_zero_set_rate(&chip->zero, 0);
		lsm6dso_zero_set_rate(&chip->zero, ZERO_POLL_HZ);
		updates = chip->zero.updates;
		rejected = chip->zero.rejected;
		for (int i = 0; i < ZERO_SECONDS * ZERO_POLL_HZ; i++) {
			ret = lsm6dso_chip_fetch_accel(chip);
			if (ret == 0) {
				ret = lsm6dso_chip_fetch_gyro(chip);
			}
			if (ret != 0) {
				break;
			}
			lsm6dso_zero_accel(&chip->zero, chip->acc);
			if (lsm6dso_zero_gyro(&chip->zero, chip->gyro)) {
				lsm6dso_zero_apply(chip);
				break;
			}
			lsm6dso_os_usleep(ZERO_POLL_US);
		}
		if (ret == 0 && chip->zero.updates == updates) {
			ret = chip->zero.rejected != rejected ? -ERANGE : -EAGAIN;
		}
		/* The next stream sets its own rate and starts a fresh window. */
		lsm6dso_zero_set_rate(&chip->zero, 0);
		lsm6dso_os_mutex_unlock(&chip->zero_lock);
	}
	if (accel_freq == 0) {
		(void)lsm6dso_accel_set_odr_raw(chip, 0);
	}
	if (gyro_freq == 0) {
		(void)lsm6dso_gyro_set_odr_raw(chip, 0);
	}
	lsm6dso_os_mutex_unlock(&chip->fifo_lock);
	return ret;
}

/* Internal primitives. The caller must serialize all access to the IMU, set
 * the sensor ODRs first, and arrange interrupt routing and error recovery.
 * Batch values are the ST register encodings (0 disables that sensor).
 */
static int lsm6dso_fifo_stop(const stmdev_ctx_t *ctx)
{
	if (ctx == NULL) {
		return -EINVAL;
	}
	return lsm6dso_fifo_mode_set(ctx, LSM6DSO_BYPASS_MODE) == 0 ? 0 : -EIO;
}

static int lsm6dso_fifo_pause_batch(const stmdev_ctx_t *ctx)
{
	if (ctx == NULL) {
		return -EINVAL;
	}
	if (lsm6dso_fifo_xl_batch_set(ctx, LSM6DSO_XL_NOT_BATCHED) != 0 ||
	    lsm6dso_fifo_gy_batch_set(ctx, LSM6DSO_GY_NOT_BATCHED) != 0) {
		return -EIO;
	}
	return 0;
}

static int lsm6dso_fifo_configure(const stmdev_ctx_t *ctx, uint16_t watermark,
				  uint8_t accel_batch, uint8_t gyro_batch, uint8_t timestamp_batch)
{
	if (ctx == NULL || watermark == 0 || watermark > 511 || accel_batch > 10 ||
	    gyro_batch > 10 || timestamp_batch > 3 || (accel_batch == 0 && gyro_batch == 0)) {
		return -EINVAL;
	}

	/* Always enter bypass before changing FIFO configuration. The
	 * timestamp counter is left running once enabled: it is the time base
	 * of the FIFO timestamp words across stream restarts. */
	if (lsm6dso_fifo_stop(ctx) != 0 ||
	    lsm6dso_timestamp_set(ctx, 1) != 0 ||
	    lsm6dso_fifo_xl_batch_set(ctx, (lsm6dso_bdr_xl_t)accel_batch) != 0 ||
	    lsm6dso_fifo_gy_batch_set(ctx, (lsm6dso_bdr_gy_t)gyro_batch) != 0 ||
	    lsm6dso_fifo_temp_batch_set(ctx, LSM6DSO_TEMP_NOT_BATCHED) != 0 ||
	    lsm6dso_fifo_timestamp_decimation_set(ctx, (lsm6dso_odr_ts_batch_t)timestamp_batch) != 0 ||
	    lsm6dso_fifo_stop_on_wtm_set(ctx, 0) != 0 ||
	    lsm6dso_compression_algo_set(ctx, LSM6DSO_CMP_DISABLE) != 0 ||
	    lsm6dso_fifo_watermark_set(ctx, watermark) != 0 ||
	    lsm6dso_fifo_mode_set(ctx, LSM6DSO_STREAM_MODE) != 0) {
		/* Failure of this best-effort rollback must be handled by the caller. */
		(void)lsm6dso_fifo_stop(ctx);
		return -EIO;
	}
	return 0;
}

static int lsm6dso_fifo_status_read(const stmdev_ctx_t *ctx, struct lsm6dso_fifo_status *status)
{
	uint8_t raw[2];
	int ret;

	if (ctx == NULL || status == NULL) {
		return -EINVAL;
	}
	ret = lsm6dso_os_bus_read(ctx, LSM6DSO_FIFO_STATUS1, raw, sizeof(raw));
	if (ret != 0) {
		return ret;
	}
	status->words = raw[0] | ((uint16_t)(raw[1] & 0x03U) << 8);
	status->watermark = (raw[1] & 0x80U) != 0;
	status->full = (raw[1] & 0x20U) != 0;
	status->overrun = (raw[1] & 0x40U) != 0;
	status->overrun_latched = (raw[1] & 0x08U) != 0;
	return 0;
}

static int lsm6dso_fifo_burst_read(const stmdev_ctx_t *ctx, uint8_t *buffer, size_t words)
{
	if (ctx == NULL || buffer == NULL || words == 0 || words > LSM6DSO_FIFO_MAX_WORDS) {
		return -EINVAL;
	}
	return lsm6dso_os_bus_read(ctx, LSM6DSO_FIFO_DATA_OUT_TAG, buffer,
				   words * LSM6DSO_FIFO_WORD_SIZE);
}

/* INT1: the FIFO threshold on the sensor's pin. */
static int lsm6dso_fifo_route_int1(const stmdev_ctx_t *ctx, bool enabled)
{
	lsm6dso_pin_int1_route_t route;

	if (ctx == NULL) {
		return -EINVAL;
	}
	if (lsm6dso_pin_int1_route_get(ctx, &route) != 0) {
		return -EIO;
	}
	route.fifo_th = enabled;
	return lsm6dso_pin_int1_route_set(ctx, route) == 0 ? 0 : -EIO;
}

int lsm6dso_chip_fifo_route_int1(struct lsm6dso_chip *chip, bool enabled)
{
	return lsm6dso_fifo_route_int1(chip->ctx, enabled);
}

int lsm6dso_chip_fifo_start(struct lsm6dso_chip *chip, uint16_t watermark, bool accel,
			    bool gyro)
{
	int xl, gy, ret;

	lsm6dso_os_mutex_lock(&chip->fifo_lock);
	if (chip->fifo_owned) {
		ret = -EBUSY;
		goto out;
	}
	xl = accel ? lsm6dso_freq_to_odr_val(chip->accel_freq) : 0;
	gy = gyro ? lsm6dso_freq_to_odr_val(chip->gyro_freq) : 0;
	if ((!xl && accel) || (!gy && gyro) || xl < 0 || gy < 0 || (!xl && !gy) ||
	    watermark == 0 || watermark > 511) {
		ret = -EINVAL;
		goto out;
	}
	chip->fifo_owned = true;
	memset(&chip->fifo_stats, 0, sizeof(chip->fifo_stats));
	chip->fifo_stats.generation = ++chip->fifo_generation;
	chip->fifo_stats.accel_hz = accel ? chip->accel_freq : 0;
	chip->fifo_stats.gyro_hz = gyro ? chip->gyro_freq : 0;
	chip->fifo_stats.accel_ug_per_lsb = chip->acc_gain;
	chip->fifo_stats.gyro_udps_per_lsb = chip->gyro_gain;
	{
		uint8_t freq_fine = 0;

		if (lsm6dso_odr_cal_reg_get(chip->ctx, &freq_fine) == 0) {
			chip->fifo_stats.freq_fine = (int8_t)freq_fine;
		}
	}
	ret = lsm6dso_fifo_configure(chip->ctx, watermark, xl, gy, chip->timestamp_decimation);
	if (ret != 0) {
		chip->fifo_stats.i2c_errors++;
	}
	chip->fifo_fault = ret != 0;
out:
	lsm6dso_os_mutex_unlock(&chip->fifo_lock);
	return ret;
}

int lsm6dso_chip_fifo_pause_batch(struct lsm6dso_chip *chip)
{
	int ret = 0;

	lsm6dso_os_mutex_lock(&chip->fifo_lock);
	if (!chip->fifo_owned || chip->fifo_fault) {
		ret = -EIO;
	} else if (lsm6dso_fifo_pause_batch(chip->ctx) != 0) {
		chip->fifo_stats.i2c_errors++;
		chip->fifo_fault = true;
		ret = -EIO;
	}
	lsm6dso_os_mutex_unlock(&chip->fifo_lock);
	return ret;
}

int lsm6dso_chip_fifo_stop(struct lsm6dso_chip *chip)
{
	int ret = 0;

	lsm6dso_os_mutex_lock(&chip->fifo_lock);
	if (chip->fifo_owned) {
		ret = lsm6dso_fifo_stop(chip->ctx);
		if (ret != 0) {
			chip->fifo_stats.i2c_errors++;
		}
		chip->fifo_owned = ret != 0;
		chip->fifo_fault = ret != 0;
	}
	lsm6dso_os_mutex_unlock(&chip->fifo_lock);
	return ret;
}

int lsm6dso_chip_fifo_read(struct lsm6dso_chip *chip, uint8_t *buffer, size_t capacity_words,
			   struct lsm6dso_fifo_status *status)
{
	int ret;

	if (!status || (!buffer && capacity_words)) {
		return -EINVAL;
	}
	lsm6dso_os_mutex_lock(&chip->fifo_lock);
	if (!chip->fifo_owned || chip->fifo_fault) {
		ret = -EIO;
		goto out;
	}
	ret = lsm6dso_fifo_status_read(chip->ctx, status);
	if (ret != 0) {
		chip->fifo_stats.i2c_errors++;
		chip->fifo_fault = true;
		goto out;
	}
	if (status->overrun || status->overrun_latched) {
		chip->fifo_stats.overrun_observations++;
	}
	if (status->words > LSM6DSO_FIFO_MAX_WORDS) {
		chip->fifo_fault = true;
		ret = -EIO;
	} else if (status->words > capacity_words) {
		chip->fifo_stats.buffer_too_small++;
		ret = -ENOBUFS;
	} else if (status->words) {
		uint32_t start = lsm6dso_os_cycle_get_32();
		uint64_t duration;

		ret = lsm6dso_fifo_burst_read(chip->ctx, buffer, status->words);
		duration = lsm6dso_os_cycles_to_ns((uint32_t)(lsm6dso_os_cycle_get_32() - start));
		chip->fifo_stats.transfers++;
		chip->fifo_stats.transfer_time_ns += duration;
		if (duration > chip->fifo_stats.max_transfer_time_ns) {
			chip->fifo_stats.max_transfer_time_ns = duration;
		}
		if (ret != 0) {
			chip->fifo_stats.i2c_errors++;
			chip->fifo_fault = true;
		} else {
			chip->fifo_stats.words_read += status->words;
		}
	}
out:
	lsm6dso_os_mutex_unlock(&chip->fifo_lock);
	return ret;
}

int lsm6dso_chip_fifo_status(struct lsm6dso_chip *chip, struct lsm6dso_fifo_status *status)
{
	int ret;

	if (!status) {
		return -EINVAL;
	}
	lsm6dso_os_mutex_lock(&chip->fifo_lock);
	if (!chip->fifo_owned || chip->fifo_fault) {
		ret = -EIO;
	} else {
		ret = lsm6dso_fifo_status_read(chip->ctx, status);
		if (ret != 0) {
			chip->fifo_stats.i2c_errors++;
			chip->fifo_fault = true;
		} else if (status->overrun || status->overrun_latched) {
			chip->fifo_stats.overrun_observations++;
		}
	}
	lsm6dso_os_mutex_unlock(&chip->fifo_lock);
	return ret;
}

int lsm6dso_chip_timestamp_read(struct lsm6dso_chip *chip, int64_t (*clock_us)(void),
				uint32_t *tick, int64_t *before_us, int64_t *after_us)
{
	uint8_t first[4], second[4];
	uint32_t a, b;
	int ret;

	if (!clock_us || !tick || !before_us || !after_us) {
		return -EINVAL;
	}
	lsm6dso_os_mutex_lock(&chip->fifo_lock);
	ret = lsm6dso_os_bus_read(chip->ctx, LSM6DSO_TIMESTAMP0, first, sizeof(first));
	if (ret == 0) {
		*before_us = clock_us();
		ret = lsm6dso_os_bus_read(chip->ctx, LSM6DSO_TIMESTAMP0, second, sizeof(second));
		*after_us = clock_us();
	}
	lsm6dso_os_mutex_unlock(&chip->fifo_lock);
	if (ret != 0) {
		return ret;
	}
	/* Little-endian. */
	a = (uint32_t)first[0] | (uint32_t)first[1] << 8 | (uint32_t)first[2] << 16 |
	    (uint32_t)first[3] << 24;
	b = (uint32_t)second[0] | (uint32_t)second[1] << 8 | (uint32_t)second[2] << 16 |
	    (uint32_t)second[3] << 24;
	/* 400 ticks are 10 ms: far more than two reads take, far less than a
	 * wrong carry into the second byte or above would produce. */
	if ((uint32_t)(b - a) > 400U) {
		return -EAGAIN;
	}
	*tick = b;
	return 0;
}

void lsm6dso_chip_fifo_get_stats(struct lsm6dso_chip *chip, struct lsm6dso_fifo_stats *stats)
{
	lsm6dso_os_mutex_lock(&chip->fifo_lock);
	*stats = chip->fifo_stats;
	stats->active = chip->fifo_owned;
	stats->fault = chip->fifo_fault;
	lsm6dso_os_mutex_unlock(&chip->fifo_lock);
}

void lsm6dso_chip_fifo_record_stop_error(struct lsm6dso_chip *chip, int error)
{
	lsm6dso_os_mutex_lock(&chip->fifo_lock);
	if (error != 0 && chip->fifo_stats.stop_error == 0) {
		chip->fifo_stats.stop_error = error;
	}
	lsm6dso_os_mutex_unlock(&chip->fifo_lock);
}

int lsm6dso_chip_fifo_set_timestamp_decimation(struct lsm6dso_chip *chip,
					       unsigned int decimation)
{
	uint8_t encoded;
	int ret = 0;

	switch (decimation) {
	case 0: encoded = LSM6DSO_NO_DECIMATION; break;
	case 1: encoded = LSM6DSO_DEC_1; break;
	case 8: encoded = LSM6DSO_DEC_8; break;
	case 32: encoded = LSM6DSO_DEC_32; break;
	default: return -EINVAL;
	}
	lsm6dso_os_mutex_lock(&chip->fifo_lock);
	if (chip->fifo_owned) {
		ret = -EBUSY;
	} else {
		chip->timestamp_decimation = encoded;
	}
	lsm6dso_os_mutex_unlock(&chip->fifo_lock);
	return ret;
}

unsigned int lsm6dso_chip_fifo_get_timestamp_decimation(struct lsm6dso_chip *chip)
{
	static const unsigned int values[] = {0, 1, 8, 32};
	unsigned int value;

	lsm6dso_os_mutex_lock(&chip->fifo_lock);
	value = values[chip->timestamp_decimation];
	lsm6dso_os_mutex_unlock(&chip->fifo_lock);
	return value;
}

static int lsm6dso_init_chip(struct lsm6dso_chip *chip, const struct lsm6dso_chip_config *cfg)
{
	const stmdev_ctx_t *ctx = cfg->ctx;
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

	fs = cfg->accel_range;
	LOG_DBG("accel range is %d", fs);
	if (lsm6dso_accel_set_fs_raw(chip, fs) < 0) {
		LOG_ERR("failed to set accelerometer range %d", fs);
		return -EIO;
	}
	chip->acc_gain = lsm6dso_accel_fs_val_to_gain(fs, cfg->accel_range_double);
	lsm6dso_cal_refresh(chip, LSM6DSO_CAL_ACCEL);

	odr = cfg->accel_odr;
	LOG_DBG("accel odr is %d", odr);
	chip->accel_freq = lsm6dso_odr_to_freq_val(odr);
	if (lsm6dso_accel_set_odr_raw(chip, odr) < 0) {
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
	if (lsm6dso_gyro_set_fs_raw(chip, fs) < 0) {
		LOG_ERR("failed to set gyroscope range %d", fs);
		return -EIO;
	}
	chip->gyro_gain = (lsm6dso_gyro_fs_sens[fs] * GAIN_UNIT_G);
	lsm6dso_cal_refresh(chip, LSM6DSO_CAL_GYRO);

	odr = cfg->gyro_odr;
	LOG_DBG("gyro odr is %d", odr);
	chip->gyro_freq = lsm6dso_odr_to_freq_val(odr);
	if (lsm6dso_gyro_set_odr_raw(chip, odr) < 0) {
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

int lsm6dso_chip_init(struct lsm6dso_chip *chip, const struct lsm6dso_chip_config *config)
{
	lsm6dso_os_mutex_init(&chip->fifo_lock);
	lsm6dso_os_mutex_init(&chip->zero_lock);
	chip->ctx = config->ctx;
	chip->accel_range_double = config->accel_range_double;
	chip->timestamp_decimation = LSM6DSO_DEC_8;

	return lsm6dso_init_chip(chip, config);
}
