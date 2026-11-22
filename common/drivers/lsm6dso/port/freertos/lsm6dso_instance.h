/* ST Microelectronics LSM6DSO 6-axis IMU sensor driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 */

/* The end of the driver on FreeRTOS (port/zephyr/lsm6dso_sensor.c, the Zephyr
 * binding, which includes this file last, so lsm6dso_sensor_init and
 * lsm6dso_driver_api stay static): in place of the Zephyr build's devicetree
 * instance macros (port/zephyr/lsm6dso_instance.h), the one device with the
 * devicetree's values, as the NuttX build's port/nuttx/lsm6dso_instance.h
 * defines it from its board header.  The application initializes it (nano_lsm6dso_initialize(),
 * nano_lsm6dso.h) where Zephyr does before main() (POST_KERNEL,
 * CONFIG_SENSOR_INIT_PRIORITY 90).
 */

/*
 * The device: the devicetree node lsm6dsox@6a of the Zephyr build
 * (zephyr/build/usb-console/zephyr/zephyr.dts: the board's
 * arduino_nano_connect.dts with app/usb-console/app.overlay), compatible
 * "st,lsm6dso" (not st,lsm6dso32: no ACCEL_RANGE_DOUBLE), on i2c0 at 0x6a.
 * The values are the binding's (zephyr/dt-bindings/sensor/lsm6dso.h); the
 * properties that the node does not set are the defaults of
 * st,lsm6dso-common.yaml.  The same values as the NuttX build's
 * (BOARD_LSM6DSOX_* of nuttx/boards/arduino-nano-rp2040-connect/include/
 * board.h).
 */

#define LSM6DSOX_NODE_NAME       "lsm6dsox@6a"
#define LSM6DSOX_I2C_ADDR        0x6a /* reg = <0x6a> */
#define LSM6DSOX_ACCEL_PM        0    /* default: high performance */
#define LSM6DSOX_ACCEL_ODR       0    /* default: off */
#define LSM6DSOX_ACCEL_LP_FILTER 0    /* default: LPF2 off */
#define LSM6DSOX_ACCEL_RANGE     3    /* app.overlay: LSM6DSO_DT_FS_8G */
#define LSM6DSOX_GYRO_PM         0    /* default: high performance */
#define LSM6DSOX_GYRO_ODR        0    /* default: off */
#define LSM6DSOX_GYRO_RANGE      6    /* app.overlay: LSM6DSO_DT_FS_2000DPS */
/* irq-gpios = <&gpio0 24 0>: GPIO24, active high, no pull. */
#define LSM6DSOX_INT1_GPIO       24
#define LSM6DSOX_INT1_FLAGS      0

/* Zephyr's LSM6DSO_CONFIG_COMMON() and LSM6DSO_CONFIG_I2C() for that node: */

static struct lsm6dso_data lsm6dso_data_0;
static const struct lsm6dso_config lsm6dso_config_0 = {
	STMEMSC_CTX_I2C(&lsm6dso_config_0.stmemsc_cfg),
	.stmemsc_cfg.i2c.bus = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm6dsox))),
	.stmemsc_cfg.i2c.addr = LSM6DSOX_I2C_ADDR,
	.accel_pm = LSM6DSOX_ACCEL_PM,
	.accel_odr = LSM6DSOX_ACCEL_ODR,
	.accel_lp_filter = LSM6DSOX_ACCEL_LP_FILTER,
	.accel_range = LSM6DSOX_ACCEL_RANGE,
	.gyro_pm = LSM6DSOX_GYRO_PM,
	.gyro_odr = LSM6DSOX_GYRO_ODR,
	.gyro_range = LSM6DSOX_GYRO_RANGE,
#ifdef CONFIG_SENSOR_ASYNC_API
	/* GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, {0}) */
	.fifo_gpio.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),
	.fifo_gpio.pin = LSM6DSOX_INT1_GPIO,
	.fifo_gpio.dt_flags = LSM6DSOX_INT1_FLAGS,
#endif
};

/* SENSOR_DEVICE_DT_INST_DEFINE(): the init function, data, config and API;
 * POST_KERNEL and CONFIG_SENSOR_INIT_PRIORITY are the application's order of
 * calls.
 */
DEVICE_DT_DEFINE(lsm6dsox, LSM6DSOX_NODE_NAME, lsm6dso_sensor_init, &lsm6dso_data_0,
		 &lsm6dso_config_0, &lsm6dso_driver_api);
