/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_lsm6dso.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_LSM6DSO_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_LSM6DSO_H

#include <nuttx/config.h>

struct i2c_master_s;

/****************************************************************************
 * Name: nano_lsm6dso_initialize
 *
 * Description:
 *   Make the i2c0 device of the Zephyr compatibility layer ready on the
 *   I2C0 master (100 kHz) and initialize the LSM6DSOX (lsm6dsox@6a) with
 *   the ported driver, as Zephyr does before main().  Returns 0, or the
 *   negative errno of the driver's init (the device is then not ready,
 *   and the driver has logged why).  From nano_bringup(), once.
 *
 ****************************************************************************/

int nano_lsm6dso_initialize(struct i2c_master_s *i2c0);

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_LSM6DSO_H */
