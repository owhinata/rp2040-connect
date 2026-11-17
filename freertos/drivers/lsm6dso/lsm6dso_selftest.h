/* SPDX-License-Identifier: MIT */
/* The board's check of the LSM6DSOX (lsm6dso_selftest.c), part of the
 * "selftest" command (a test feature, built with -DNANO_SELFTEST=ON only).
 */

#ifndef FREERTOS_DRIVERS_LSM6DSO_LSM6DSO_SELFTEST_H
#define FREERTOS_DRIVERS_LSM6DSO_LSM6DSO_SELFTEST_H

/* Runs the checks on the board's LSM6DSOX; returns the number of checks
 * that failed.  From a task, while nothing else uses the sensor (no stream)
 * and with the board at rest.
 */
int lsm6dso_selftest_run(void);

#endif /* FREERTOS_DRIVERS_LSM6DSO_LSM6DSO_SELFTEST_H */
