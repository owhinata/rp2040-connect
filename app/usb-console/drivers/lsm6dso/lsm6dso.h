/* ST Microelectronics LSM6DSO 6-axis IMU sensor driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LSM6DSO_LSM6DSO_H_
#define ZEPHYR_DRIVERS_SENSOR_LSM6DSO_LSM6DSO_H_

#include <zephyr/drivers/sensor.h>
#include "lsm6dso_cal.h"
#include "lsm6dso_zero.h"
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <stmemsc.h>
#include "lsm6dso_reg.h"
#include "lsm6dso_fifo_stats.h"
#ifdef CONFIG_SENSOR_ASYNC_API
#include "lsm6dso_fifo_irq.h"
#endif

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, spi) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, spi)
#include <zephyr/drivers/spi.h>
#endif

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, i2c) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, i2c)
#include <zephyr/drivers/i2c.h>
#endif

#define LSM6DSO_EN_BIT					0x01
#define LSM6DSO_DIS_BIT					0x00

/* Accel sensor sensitivity grain is 61 ug/LSB */
#define GAIN_UNIT_XL				(61LL)

/* Gyro sensor sensitivity grain is 4.375 udps/LSB */
#define GAIN_UNIT_G				(4375LL)

struct lsm6dso_config {
	stmdev_ctx_t ctx;
	union {
#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, i2c) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, i2c)
		const struct i2c_dt_spec i2c;
#endif
#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, spi) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, spi)
		const struct spi_dt_spec spi;
#endif
	} stmemsc_cfg;
	uint8_t accel_pm;
	uint8_t accel_odr;
	uint8_t accel_lp_filter;
#define ACCEL_RANGE_DOUBLE	BIT(7)
#define ACCEL_RANGE_MASK	BIT_MASK(6)
	uint8_t accel_range;
	uint8_t gyro_pm;
	uint8_t gyro_odr;
	uint8_t gyro_range;
	uint8_t drdy_pulsed;
#ifdef CONFIG_SENSOR_ASYNC_API
	struct gpio_dt_spec fifo_gpio;
#endif
#ifdef CONFIG_LSM6DSO_TRIGGER
	const struct gpio_dt_spec gpio_drdy;
	uint8_t int_pin;
	bool trig_enabled;
#ifdef CONFIG_LSM6DSO_TAP
	uint8_t tap_mode;
	uint8_t tap_threshold[3];
	uint8_t tap_shock;
	uint8_t tap_latency;
	uint8_t tap_quiet;
#endif /* CONFIG_LSM6DSO_TAP */
#endif /* CONFIG_LSM6DSO_TRIGGER */
};

#define LSM6DSO_SHUB_MAX_NUM_TARGETS			3

struct lsm6dso_data {
	const struct device *dev;
	struct k_mutex fifo_lock;
	/* Nonzero also covers an error state until bypass has been restored. */
	bool fifo_owned;
	bool fifo_fault;
	uint32_t fifo_generation;
	struct lsm6dso_fifo_stats fifo_stats;
#ifdef CONFIG_SENSOR_ASYNC_API
	struct lsm6dso_fifo_irq fifo_irq;
	struct k_work_delayable stream_work;
	struct k_spinlock stream_lock;
	uint16_t stream_watermark;
	uint8_t timestamp_decimation;
	struct rtio_iodev_sqe *stream_sqe;
	struct rtio *stream_rtio;
	rtio_sqe_handle_t stream_handle;
	bool stream_active;
	bool stream_irq_active;
	bool stream_stop_requested;
	bool stream_batch_paused;
#endif
	int16_t acc[3];
	uint32_t acc_gain;
	int16_t gyro[3];
	uint32_t gyro_gain;
	/* Conversions to physical units; guarded by cal_lock. */
	struct k_spinlock cal_lock;
	struct lsm6dso_cal accel_cal;
	struct lsm6dso_cal gyro_cal;
	bool accel_calibrated;
	bool gyro_calibrated;
	lsm6dso_cal_lookup_t cal_lookup;
	/* Gyro bias tracked at rest; guarded by zero_lock. */
	struct k_mutex zero_lock;
	struct lsm6dso_zero zero;
#if defined(CONFIG_LSM6DSO_ENABLE_TEMP)
	int16_t temp_sample;
#endif
#if defined(CONFIG_LSM6DSO_SENSORHUB)
	uint8_t ext_data[LSM6DSO_SHUB_MAX_NUM_TARGETS][6];
	uint16_t magn_gain;

	struct hts221_data {
		int16_t x0;
		int16_t x1;
		int16_t y0;
		int16_t y1;
	} hts221;
	bool shub_inited;
	uint8_t num_ext_dev;
	uint8_t shub_ext[LSM6DSO_SHUB_MAX_NUM_TARGETS];
#endif /* CONFIG_LSM6DSO_SENSORHUB */

	uint16_t accel_freq;
	uint8_t accel_fs;
	uint16_t gyro_freq;
	uint8_t gyro_fs;

#ifdef CONFIG_LSM6DSO_TRIGGER
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t handler_drdy_acc;
	const struct sensor_trigger *trig_drdy_acc;
	sensor_trigger_handler_t handler_drdy_gyr;
	const struct sensor_trigger *trig_drdy_gyr;
	sensor_trigger_handler_t handler_drdy_temp;
	const struct sensor_trigger *trig_drdy_temp;

#ifdef CONFIG_LSM6DSO_TILT
	sensor_trigger_handler_t handler_tilt;
	const struct sensor_trigger *trig_tilt;
#endif /* CONFIG_LSM6DSO_TILT */

#ifdef CONFIG_LSM6DSO_TAP
	sensor_trigger_handler_t handler_tap;
	const struct sensor_trigger *trig_tap;
	sensor_trigger_handler_t handler_double_tap;
	const struct sensor_trigger *trig_double_tap;
#endif /* CONFIG_LSM6DSO_TAP */

#if defined(CONFIG_LSM6DSO_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_LSM6DSO_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_LSM6DSO_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_LSM6DSO_TRIGGER */
};

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, i2c) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, i2c)
struct lsm6dso_fifo_status;
int lsm6dso_fifo_start_locked_driver(const struct device *dev, uint16_t watermark,
				     bool accel, bool gyro);
int lsm6dso_fifo_stop_locked_driver(const struct device *dev);
int lsm6dso_fifo_read_locked_driver(const struct device *dev, uint8_t *buffer,
				    size_t capacity_words, struct lsm6dso_fifo_status *status);
int lsm6dso_fifo_status_locked_driver(const struct device *dev,
				      struct lsm6dso_fifo_status *status);
int lsm6dso_fifo_pause_batch_locked_driver(const struct device *dev);
void lsm6dso_fifo_record_stop_error(const struct device *dev, int error);
#endif

#ifdef CONFIG_SENSOR_ASYNC_API
void lsm6dso_stream_init(const struct device *dev);
void lsm6dso_submit(const struct device *dev, struct rtio_iodev_sqe *sqe);
#endif

#if defined(CONFIG_LSM6DSO_SENSORHUB)
int lsm6dso_shub_init(const struct device *dev);
int lsm6dso_shub_fetch_external_devs(const struct device *dev);
int lsm6dso_shub_get_idx(const struct device *dev, enum sensor_channel type);
int lsm6dso_shub_config(const struct device *dev, enum sensor_channel chan,
			enum sensor_attribute attr,
			const struct sensor_value *val);
#endif /* CONFIG_LSM6DSO_SENSORHUB */

#ifdef CONFIG_LSM6DSO_TRIGGER
int lsm6dso_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int lsm6dso_init_interrupt(const struct device *dev);
#endif

/* Feeds FIFO words to the gyro bias tracker. */
void lsm6dso_cal_track(const struct device *dev, const uint8_t *words, size_t len);
/* Copies the conversions currently in effect. */
void lsm6dso_cal_snapshot(const struct device *dev, struct lsm6dso_cal *accel,
			  struct lsm6dso_cal *gyro);

#endif /* ZEPHYR_DRIVERS_SENSOR_LSM6DSO_LSM6DSO_H_ */
