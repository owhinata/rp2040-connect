/* SPDX-License-Identifier: Apache-2.0 */
#ifndef LSM6DSO_ATTRIBUTES_H_
#define LSM6DSO_ATTRIBUTES_H_

#include <zephyr/drivers/sensor.h>

/* Device-wide FIFO settings for SENSOR_CHAN_ALL. Both take effect at the next
 * stream start and cannot be changed while a stream is running. */
enum lsm6dso_sensor_attribute {
	/* FIFO threshold in FIFO words. */
	SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK = SENSOR_ATTR_PRIV_START,
	/* One timestamp word per this many sampling instants: 0 (none), 1, 8 or 32. */
	SENSOR_ATTR_LSM6DSO_FIFO_TIMESTAMP_DECIMATION,
};

#endif
