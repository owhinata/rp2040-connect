/* SPDX-License-Identifier: Apache-2.0 */
#ifndef LSM6DSO_ATTRIBUTES_H_
#define LSM6DSO_ATTRIBUTES_H_

#include <zephyr/drivers/sensor.h>

/* Device-wide FIFO threshold, in FIFO words, for SENSOR_CHAN_ALL. */
enum lsm6dso_sensor_attribute {
	SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK = SENSOR_ATTR_PRIV_START,
};

#endif
