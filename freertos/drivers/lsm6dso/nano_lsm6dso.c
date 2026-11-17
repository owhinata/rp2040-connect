/* SPDX-License-Identifier: MIT */
/* The start of the LSM6DSOX on FreeRTOS (nano_lsm6dso.h): the device that
 * lsm6dso_instance.h defines at the end of the shared driver
 * (common/drivers/lsm6dso/lsm6dso.c), on the device i2c0 of the I2C0 core
 * (freertos/drivers/i2c_dw).  The counterpart of the NuttX build's board
 * src/nano_lsm6dso.c (there i2c0 is bound to a NuttX I2C master first; here
 * it is the core's own device, ready after nano_i2c0_initialize()).
 */

#include <errno.h>

#include <zephyr/device.h>

#include "nano_lsm6dso.h"

int nano_lsm6dso_initialize(void)
{
	if (!device_is_ready(DEVICE_DT_GET(DT_NODELABEL(i2c0)))) {
		return -ENODEV;
	}

	return device_init(DEVICE_DT_GET(DT_NODELABEL(lsm6dsox)));
}
