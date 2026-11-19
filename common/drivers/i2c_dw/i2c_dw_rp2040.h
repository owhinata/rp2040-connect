/*
 * SPDX-License-Identifier: Apache-2.0
 */

/* The RP2040's I2C controllers in the DesignWare driver, as the Zephyr
 * build's drivers/i2c/i2c_dw_rp2040.h (local patch 0003) has them: the depth
 * of their FIFOs, which IC_COMP_PARAM_1 does not give, and the RX batch that
 * i2c_dw_data_ask(), i2c_dw_data_read(), i2c_dw_isr_handler() and
 * i2c_dw_transfer_msgs() use for the RX threshold.  Zephyr's file also has
 * i2c_dw_rp2040_controller(), the check that a device is one of the
 * RP2040's controllers: the core drives only i2c0, so its calls are left
 * out (i2c_dw.c).
 *
 * Included once, by i2c_dw.c.  No OS header.
 */

#ifndef COMMON_DRIVERS_I2C_DW_I2C_DW_RP2040_H_
#define COMMON_DRIVERS_I2C_DW_I2C_DW_RP2040_H_

/* i2c_dw_rp2040.h:20-25 */

#define RP2040_I2C_FIFO_DEPTH 16U
#define RP2040_I2C_RX_BATCH 4U

#endif /* COMMON_DRIVERS_I2C_DW_I2C_DW_RP2040_H_ */
