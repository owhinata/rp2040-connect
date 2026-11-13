/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_i2c.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_I2C_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_I2C_H

#include <nuttx/config.h>

struct i2c_master_s;

/****************************************************************************
 * Name: nano_i2c0_initialize
 *
 * Description:
 *   Bring up I2C0 (GPIO12 SDA, GPIO13 SCL: the LSM6DSOX and the ATECC608A)
 *   the way the Zephyr build's driver does at boot, and return it as a
 *   NuttX I2C master (I2C_TRANSFER(), and I2C_RESET() with CONFIG_I2C_RESET).
 *   Later calls return the same instance.  Returns NULL if the controller
 *   does not answer as a DesignWare I2C block.  From nano_bringup().
 *
 ****************************************************************************/

struct i2c_master_s *nano_i2c0_initialize(void);

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_I2C_H */
