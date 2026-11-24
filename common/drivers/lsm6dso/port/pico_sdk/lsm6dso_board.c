/* SPDX-License-Identifier: MIT */
/* The start of the LSM6DSOX on the Pico SDK (lsm6dso_board.h): the device that
 * lsm6dso_instance.h next to this file
 * defines at the end of the shared driver's binding
 * (common/drivers/lsm6dso/port/zephyr/lsm6dso_sensor.c), on the device i2c0 of the I2C core
 * (common/drivers/i2c_dw/port/pico_sdk).  The counterpart of the NuttX build's board
 * src/rp2040_lsm6dso.c (there i2c0 is bound to a NuttX I2C master first; here
 * it is the core's own device, ready after i2c_dw_i2c0_initialize()).
 */

#include <errno.h>

#include <zephyr/device.h>

#include "lsm6dso_board.h"

int lsm6dso_board_initialize(void)
{
	if (!device_is_ready(DEVICE_DT_GET(DT_NODELABEL(i2c0)))) {
		return -ENODEV;
	}

	return device_init(DEVICE_DT_GET(DT_NODELABEL(lsm6dsox)));
}
