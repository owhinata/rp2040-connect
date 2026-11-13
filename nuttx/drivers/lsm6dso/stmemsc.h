/* ST Microelectronics STMEMS hal i/f
 *
 * Copyright (c) 2021 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * zephyrproject-rtos/modules/hal/st/sensor/stmemsc/
 */

/* The I2C part of Zephyr's drivers/sensor/st/stmemsc/stmemsc.h (Zephyr
 * v4.5.0): the glue between ST's register drivers (stmdev_ctx_t of
 * lsm6dso_reg.h) and the Zephyr I2C API (here the compatibility layer,
 * nuttx/drivers/compat).  The I3C and SPI parts are left out (the LSM6DSOX
 * of this board is on I2C0).
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_STMEMSC_STMEMSC_H_
#define ZEPHYR_DRIVERS_SENSOR_STMEMSC_STMEMSC_H_

#include <stdint.h>
#include <zephyr/drivers/i2c.h>

/* CONFIG_STMEMSC_I3C_I2C_WRITE_BUFFER_SIZE of the Zephyr build (16). */

#define CONFIG_STMEMSC_I3C_I2C_WRITE_BUFFER_SIZE 16

void stmemsc_mdelay(uint32_t millisec);

/*
 * Populate the stmdev_ctx_t structure pointed by stmdev_ctx_ptr with
 * standard stmemsc i2c APIs.
 */
#define STMEMSC_CTX_I2C(stmdev_ctx_ptr)					\
	.ctx = {							\
		.read_reg = (stmdev_read_ptr)stmemsc_i2c_read,		\
		.write_reg = (stmdev_write_ptr)stmemsc_i2c_write,	\
		.mdelay = (stmdev_mdelay_ptr)stmemsc_mdelay,		\
		.handle = (void *)stmdev_ctx_ptr			\
	}

int stmemsc_i2c_read(const struct i2c_dt_spec *stmemsc,
		     uint8_t reg_addr, uint8_t *value, uint8_t len);
int stmemsc_i2c_write(const struct i2c_dt_spec *stmemsc,
		      uint8_t reg_addr, uint8_t *value, uint8_t len);

#endif /* ZEPHYR_DRIVERS_SENSOR_STMEMSC_STMEMSC_H_ */
