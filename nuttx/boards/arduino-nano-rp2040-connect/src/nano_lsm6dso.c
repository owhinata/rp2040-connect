/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_lsm6dso.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/* The devices of the Zephyr build's devicetree that the ported LSM6DSO
 * driver uses (<zephyr/devicetree.h> of nuttx/drivers/compat):
 *
 *   i2c0      /soc/i2c@40044000: the I2C0 master of nano_i2c.c, as a bus
 *             device of the compatibility layer, at the devicetree's
 *             clock-frequency (100 kHz) until i2c_configure() changes it
 *   lsm6dsox  /soc/i2c@40044000/lsm6dsox@6a: defined by
 *             nuttx/drivers/lsm6dso/lsm6dso.c, initialized here
 *
 * Zephyr initializes i2c0 (POST_KERNEL, CONFIG_I2C_INIT_PRIORITY 50) and
 * then the sensor (POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY 90) before
 * main(), which mounts the key-value store; nano_bringup() calls this after
 * nano_i2c0_initialize() and before nano_flash_kv_initialize() for the
 * same order.  A failed sensor initialization leaves the device not ready
 * (device_is_ready() false), as in Zephyr, and boot goes on.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stddef.h>

#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#include "nano_lsm6dso.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct i2c_compat_bus g_i2c0_bus;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* No init function: the board's nano_i2c0_initialize() has set up the
 * controller; i2c_compat_bind() makes the device ready.
 */

DEVICE_DT_DEFINE(i2c0, "i2c@40044000", NULL, &g_i2c0_bus, NULL, NULL);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nano_lsm6dso_initialize(struct i2c_master_s *i2c0)
{
  int ret;

  ret = i2c_compat_bind(DEVICE_DT_GET(DT_NODELABEL(i2c0)), i2c0,
                        BOARD_I2C0_FREQUENCY);
  if (ret < 0)
    {
      return ret;
    }

  return device_init(DEVICE_DT_GET(DT_NODELABEL(lsm6dsox)));
}
