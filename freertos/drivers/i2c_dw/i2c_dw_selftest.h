/* SPDX-License-Identifier: MIT */
/* The board's check of I2C0, and of I2C1 when it is built
 * (i2c_dw_selftest.c), part of the "selftest" command (a test feature,
 * built with -DNANO_SELFTEST=ON only).
 */

#ifndef FREERTOS_DRIVERS_I2C_DW_I2C_DW_SELFTEST_H
#define FREERTOS_DRIVERS_I2C_DW_I2C_DW_SELFTEST_H

/* Runs the checks on the board's LSM6DSOX (and on I2C1 with nothing
 * connected); returns the number of checks that failed.  From a task, while
 * nothing else uses I2C0 or I2C1.
 */
int i2c_dw_selftest_run(void);

#endif /* FREERTOS_DRIVERS_I2C_DW_I2C_DW_SELFTEST_H */
