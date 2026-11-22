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
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <stmemsc.h>
#include "lsm6dso_reg.h"
#include "lsm6dso_chip.h"
#include <zephyr/drivers/sensor/lsm6dso.h>
#ifdef CONFIG_SENSOR_ASYNC_API
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#endif

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, spi) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, spi)
#include <zephyr/drivers/spi.h>
#endif

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, i2c) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, i2c)
#include <zephyr/drivers/i2c.h>
#endif

#ifdef CONFIG_SENSOR_ASYNC_API
/* The FIFO's INT1 line while a stream runs (lsm6dso_rtio_stream.c). The
 * worker callback must drain below the watermark, or return an error. It
 * runs on the system workqueue, never in the GPIO interrupt handler.
 */
struct lsm6dso_fifo_irq {
	struct gpio_dt_spec gpio;
	struct gpio_callback callback;
	struct k_work work;
	atomic_t enabled;
	int (*handle)(void *context);
	void *context;
};
#endif

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
#ifdef CONFIG_SENSOR_ASYNC_API
	struct gpio_dt_spec fifo_gpio;
#endif
};

struct lsm6dso_data {
	const struct device *dev;
	/* The chip's state: rates, ranges, samples, FIFO, calibration. */
	struct lsm6dso_chip chip;
#ifdef CONFIG_SENSOR_ASYNC_API
	struct lsm6dso_fifo_irq fifo_irq;
	struct k_work_delayable stream_work;
	struct k_spinlock stream_lock;
	uint16_t stream_watermark;
	struct rtio_iodev_sqe *stream_sqe;
	struct rtio *stream_rtio;
	rtio_sqe_handle_t stream_handle;
	bool stream_active;
	bool stream_irq_active;
	bool stream_stop_requested;
	bool stream_batch_paused;
#endif
};

#ifdef CONFIG_SENSOR_ASYNC_API
void lsm6dso_stream_init(const struct device *dev);
void lsm6dso_submit(const struct device *dev, struct rtio_iodev_sqe *sqe);
void lsm6dso_submit_stream(const struct device *dev, struct rtio_iodev_sqe *sqe);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_LSM6DSO_LSM6DSO_H_ */
