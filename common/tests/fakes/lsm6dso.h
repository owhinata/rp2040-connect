/* SPDX-License-Identifier: MIT */
/* Stands in for the driver's lsm6dso.h, which each build keeps in its own driver directory
 * (zephyr/app/usb-console/drivers/lsm6dso, nuttx/drivers/lsm6dso).  lsm6dso_stream.c includes
 * it; stream_stop.c defines what the stream needs, and this guard, before it includes
 * lsm6dso_stream.c, so nothing of the real header is used. */
#ifndef ZEPHYR_DRIVERS_SENSOR_LSM6DSO_LSM6DSO_H_
#define ZEPHYR_DRIVERS_SENSOR_LSM6DSO_LSM6DSO_H_
#endif
