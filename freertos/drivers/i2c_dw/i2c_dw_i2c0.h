/* SPDX-License-Identifier: MIT */
/* I2C0 of the FreeRTOS build (freertos/drivers/i2c_dw): the start-up of the
 * controller.  The bus is then the device i2c0 of the Zephyr compatibility
 * layer, DEVICE_DT_GET(DT_NODELABEL(i2c0)), used with Zephyr's I2C API
 * (<zephyr/drivers/i2c.h>).  The DMA reads' state is
 * i2c_dw_rp2040_dma.h of common/drivers/i2c_dw ("i2c_dma").
 */

#ifndef FREERTOS_DRIVERS_I2C_DW_I2C_DW_I2C0_H
#define FREERTOS_DRIVERS_I2C_DW_I2C_DW_I2C0_H

/* Resets I2C0, sets up GPIO12 (SDA) and GPIO13 (SCL), checks the block
 * (IC_COMP_TYPE), writes the controller's registers for 100 kHz (the
 * devicetree's clock-frequency) and enables its interrupt; the device i2c0
 * is ready after it.  Once, before the scheduler starts, after
 * dma_rpi_pico_initialize() (common/drivers/i2c_dw/dma_rpi_pico.h): the
 * Zephyr build initializes its DMA device (POST_KERNEL 40) before i2c0
 * (POST_KERNEL 50).
 * Returns 0, or a negative errno (-EIO: not a DesignWare I2C block); i2c0
 * then stays not ready and its calls return -ENODEV.
 */
int i2c_dw_i2c0_initialize(void);

#endif /* FREERTOS_DRIVERS_I2C_DW_I2C_DW_I2C0_H */
