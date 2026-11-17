/* ST Microelectronics STMEMS hal i/f
 *
 * Copyright (c) 2021 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * zephyrproject-rtos/modules/hal/st/sensor/stmemsc/
 */

/* stmemsc_i2c_read() and stmemsc_i2c_write() of Zephyr's
 * drivers/sensor/st/stmemsc/stmemsc_i2c.c (Zephyr v4.5.0), unchanged.  The
 * assertion is the compatibility layer's __ASSERT_NO_MSG() (off in the
 * Zephyr build, where CONFIG_ASSERT is not set; NuttX's DEBUGASSERT(), off
 * unless CONFIG_DEBUG_ASSERTIONS; nothing on FreeRTOS).  The _incr variants
 * are left out (not used by the LSM6DSO driver).  Shared by the NuttX and
 * FreeRTOS builds.
 */

#include <string.h>

#include <zephyr/sys/__assert.h>

#include "stmemsc.h"

int stmemsc_i2c_read(const struct i2c_dt_spec *stmemsc,
		     uint8_t reg_addr, uint8_t *value, uint8_t len)
{
	return i2c_burst_read_dt(stmemsc, reg_addr, value, len);
}

int stmemsc_i2c_write(const struct i2c_dt_spec *stmemsc,
		      uint8_t reg_addr, uint8_t *value, uint8_t len)
{
	uint8_t buf[CONFIG_STMEMSC_I3C_I2C_WRITE_BUFFER_SIZE];

	__ASSERT_NO_MSG(len <= sizeof(buf) - 1);

	buf[0] = reg_addr;
	memcpy(&buf[1], value, len);

	return i2c_write_dt(stmemsc, buf, len + 1);
}
