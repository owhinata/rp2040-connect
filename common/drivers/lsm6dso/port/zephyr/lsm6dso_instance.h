/* ST Microelectronics LSM6DSO 6-axis IMU sensor driver
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lsm6dso.pdf
 */

/* The end of the driver on Zephyr (lsm6dso.c, which includes this file last,
 * so lsm6dso_init and lsm6dso_driver_api stay static): the devices of the
 * devicetree's st,lsm6dso and st,lsm6dso32 nodes (Zephyr's binding of
 * upstream's driver), upstream's macros with the FIFO's INT1 line
 * (fifo_gpio, the node's irq-gpios).  The NuttX and FreeRTOS builds have
 * their own lsm6dso_instance.h in port/nuttx and port/freertos.
 */

/*
 * Device creation macro, shared by LSM6DSO_DEFINE_SPI() and
 * LSM6DSO_DEFINE_I2C().
 */

#define LSM6DSO_DEVICE_INIT(inst, model)				\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			    lsm6dso_init,				\
			    NULL,					\
			    &model##_data_##inst,			\
			    &model##_config_##inst,			\
			    POST_KERNEL,				\
			    CONFIG_SENSOR_INIT_PRIORITY,		\
			    &lsm6dso_driver_api);

/*
 * Instantiation macros used when a device is on a SPI bus.
 */

#define LSM6DSO_SPI_OP  (SPI_WORD_SET(8) |				\
			 SPI_OP_MODE_CONTROLLER |			\
			 SPI_MODE_CPOL |				\
			 SPI_MODE_CPHA)					\

#define LSM6DSO_CONFIG_COMMON(inst)					\
	.accel_pm = DT_INST_PROP(inst, accel_pm),			\
	.accel_odr = DT_INST_PROP(inst, accel_odr),			\
	.accel_lp_filter = DT_INST_PROP(inst, accel_lp_filter),		\
	.accel_range = DT_INST_PROP(inst, accel_range) |		\
		(DT_INST_NODE_HAS_COMPAT(inst, st_lsm6dso32) ?	        \
			ACCEL_RANGE_DOUBLE : 0),			\
	.gyro_pm = DT_INST_PROP(inst, gyro_pm),				\
	.gyro_odr = DT_INST_PROP(inst, gyro_odr),			\
	.gyro_range = DT_INST_PROP(inst, gyro_range),			\
	IF_ENABLED(CONFIG_SENSOR_ASYNC_API,				\
		   (.fifo_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, {0}),))

#define LSM6DSO_CONFIG_SPI(inst, model)					\
	{								\
		STMEMSC_CTX_SPI(&model##_config_##inst.stmemsc_cfg),	\
		.stmemsc_cfg = {					\
			.spi = SPI_DT_SPEC_INST_GET(inst,		\
					   LSM6DSO_SPI_OP),		\
		},							\
		LSM6DSO_CONFIG_COMMON(inst)				\
	}

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define LSM6DSO_CONFIG_I2C(inst, model)					\
	{								\
		STMEMSC_CTX_I2C(&model##_config_##inst.stmemsc_cfg),	\
		.stmemsc_cfg = {					\
			.i2c = I2C_DT_SPEC_INST_GET(inst),		\
		},							\
		LSM6DSO_CONFIG_COMMON(inst)				\
	}

/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define LSM6DSO_DEFINE(inst, model)					\
	static struct lsm6dso_data model##_data_##inst;			\
	static const struct lsm6dso_config model##_config_##inst =	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),			\
			(LSM6DSO_CONFIG_SPI(inst, model)),		\
			(LSM6DSO_CONFIG_I2C(inst, model)));		\
	LSM6DSO_DEVICE_INIT(inst, model)

#define DT_DRV_COMPAT st_lsm6dso
DT_INST_FOREACH_STATUS_OKAY_VARGS(LSM6DSO_DEFINE, lsm6dso)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT st_lsm6dso32
DT_INST_FOREACH_STATUS_OKAY_VARGS(LSM6DSO_DEFINE, lsm6dso32)
#undef DT_DRV_COMPAT
