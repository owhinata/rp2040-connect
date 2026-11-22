/****************************************************************************
 * common/drivers/i2c_dw/port/nuttx/i2c_dw_controllers_rp2040.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __COMMON_DRIVERS_I2C_DW_PORT_NUTTX_I2C_DW_CONTROLLERS_RP2040_H
#define __COMMON_DRIVERS_I2C_DW_PORT_NUTTX_I2C_DW_CONTROLLERS_RP2040_H

#include <nuttx/config.h>

struct i2c_master_s;

/****************************************************************************
 * Name: i2c_dw_i2c0_initialize
 *
 * Description:
 *   Bring up I2C0 (GPIO12 SDA, GPIO13 SCL: the LSM6DSOX and the ATECC608A)
 *   the way the Zephyr build's driver does at boot, and return it as a
 *   NuttX I2C master (I2C_TRANSFER(), and I2C_RESET() with CONFIG_I2C_RESET).
 *   Later calls return the same instance.  Returns NULL if the controller
 *   does not answer as a DesignWare I2C block.  From rp2040_bringup().
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_DW_I2C0
struct i2c_master_s *i2c_dw_i2c0_initialize(void);
#endif

/****************************************************************************
 * Name: i2c_dw_i2c1_initialize
 *
 * Description:
 *   The same for I2C1 (BOARD_I2C1_SDA_GPIO, BOARD_I2C1_SCL_GPIO: GPIO18,
 *   D6, and GPIO19, D7, with the internal pull-ups), a second instance of
 *   the same driver, without the DMA reads.  From rp2040_bringup(), after
 *   I2C0.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_DW_I2C1
struct i2c_master_s *i2c_dw_i2c1_initialize(void);
#endif

#endif /* __COMMON_DRIVERS_I2C_DW_PORT_NUTTX_I2C_DW_CONTROLLERS_RP2040_H */
