/* ST Microelectronics LSM6DSO 6-axis IMU sensor driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 */

#ifndef LSM6DSO_SENSOR_H_
#define LSM6DSO_SENSOR_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <stmemsc.h>
#include "lsm6dso_reg.h"
#include "lsm6dso_core.h"
#include <zephyr/drivers/sensor/lsm6dso.h>
#ifdef CONFIG_SENSOR_ASYNC_API
#include "lsm6dso_stream.h"
#endif

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, spi) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, spi)
#include <zephyr/drivers/spi.h>
#endif

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, i2c) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, i2c)
#include <zephyr/drivers/i2c.h>
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
	/* The core's state: rates, ranges, samples, FIFO, calibration. */
	struct lsm6dso core;
#ifdef CONFIG_SENSOR_ASYNC_API
	/* The FIFO stream (lsm6dso_rtio_stream.c), on the line fifo_gpio. */
	struct lsm6dso_stream stream;
#endif
};

#ifdef CONFIG_SENSOR_ASYNC_API
void lsm6dso_sensor_stream_init(const struct device *dev);
void lsm6dso_sensor_submit(const struct device *dev, struct rtio_iodev_sqe *sqe);
void lsm6dso_sensor_submit_stream(const struct device *dev, struct rtio_iodev_sqe *sqe);
#endif

#endif /* LSM6DSO_SENSOR_H_ */
