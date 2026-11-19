/* SPDX-License-Identifier: MIT */
/* The LSM6DSOX of the FreeRTOS build (freertos/drivers/lsm6dso): its start.
 * The sensor is then the device lsm6dsox of the Zephyr compatibility layer,
 * DEVICE_DT_GET(DT_NODELABEL(lsm6dsox)), used with Zephyr's sensor API
 * (<zephyr/drivers/sensor.h>) and the driver's own calls (lsm6dso.h,
 * lsm6dso_fifo_stats.h).  Like zephyr_compat.h, this header includes none of
 * the layer's Zephyr headers.
 */

#ifndef FREERTOS_DRIVERS_LSM6DSO_NANO_LSM6DSO_H
#define FREERTOS_DRIVERS_LSM6DSO_NANO_LSM6DSO_H

/* Initializes the LSM6DSOX (lsm6dsox@6a) with the shared driver, as Zephyr
 * does before main() (POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY 90, after
 * i2c0's 50): the log lines "Initialize device lsm6dsox@6a" and
 * "chip id 0x6c", the FIFO stream's work queue ("imu_fifo_worker"), the
 * chip's reset and its configuration (ranges 8 g and 2000 dps, both off,
 * FIFO in bypass mode).  From a task (the driver sleeps and waits for I2C0),
 * once, after zephyr_compat_initialize() (gpio0, the work queues) and
 * i2c_dw_i2c0_initialize().  Returns 0; -ENODEV without touching the sensor
 * when i2c0 is not ready (the NuttX build skips the sensor then too); or
 * the driver's -EIO, after which the device is not ready (its calls fail,
 * device_is_ready() is false, as in Zephyr) and the driver has logged why.
 * The start goes on in every case.
 */
int nano_lsm6dso_initialize(void);

#endif /* FREERTOS_DRIVERS_LSM6DSO_NANO_LSM6DSO_H */
