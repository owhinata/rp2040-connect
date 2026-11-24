/* SPDX-License-Identifier: MIT */
/* The I2C controllers of the builds on the Pico SDK (the core's binding,
 * common/drivers/i2c_dw/port/pico_sdk with port/<OS>): their start-up.  Each bus is then a
 * device of the Zephyr compatibility layer, DEVICE_DT_GET(DT_NODELABEL(i2c0))
 * and, with CONFIG_I2C_DW_I2C1 (the CMake option I2C_DW_I2C1, i2c_dw.cmake),
 * DEVICE_DT_GET(DT_NODELABEL(i2c1)), used with Zephyr's I2C API
 * (<zephyr/drivers/i2c.h>).  The DMA reads (i2c0's only) have their state in
 * i2c_dw_rp2040_dma.h of common/drivers/i2c_dw/include ("i2c_dma").
 */

#ifndef I2C_DW_CONTROLLERS_RP2040_H_
#define I2C_DW_CONTROLLERS_RP2040_H_

#ifdef CONFIG_I2C_DW_I2C0
/* Resets I2C0, sets up GPIO12 (SDA) and GPIO13 (SCL), checks the block
 * (IC_COMP_TYPE), writes the controller's registers for 100 kHz (the core's
 * start; the first transfer sets I2C0_SPEED) and enables its interrupt; the device i2c0
 * is ready after it.  Before, the shared handler of the DMA interrupt for
 * the DMA reads (i2c_dw_dma_os_rp2040.c), as the Zephyr build initializes its
 * DMA device (POST_KERNEL 40) before i2c0 (POST_KERNEL 50).  Once, before the
 * scheduler starts.
 * Returns 0, or a negative errno (-EIO: not a DesignWare I2C block); i2c0
 * then stays not ready and its calls return -ENODEV.
 */
int i2c_dw_i2c0_initialize(void);
#endif

#ifdef CONFIG_I2C_DW_I2C1
/* The same for I2C1 on its pins (I2C1_SDA_GPIO, I2C1_SCL_GPIO of
 * i2c_dw_instance.h: GPIO18, D6, and GPIO19, D7) with the internal
 * pull-ups, without DMA reads; the device i2c1.  After
 * i2c_dw_i2c0_initialize(), as Zephyr initializes i2c1 after i2c0.  A
 * failure leaves i2c1 not ready and does not touch i2c0.
 */
int i2c_dw_i2c1_initialize(void);
#endif

#endif /* I2C_DW_CONTROLLERS_RP2040_H_ */
