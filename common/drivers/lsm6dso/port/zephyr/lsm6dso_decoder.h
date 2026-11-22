/* SPDX-License-Identifier: Apache-2.0 */
/* The decoders of the sensor API for the buffers the driver produces: the
 * FIFO stream's (lsm6dso_fifo_packet.h) and, through the combined decoder,
 * the single reads' generic format.
 */
#ifndef LSM6DSO_DECODER_H_
#define LSM6DSO_DECODER_H_

#include <stdint.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/lsm6dso.h>

#include "lsm6dso_fifo_packet.h"

extern const struct sensor_decoder_api lsm6dso_fifo_decoder;
extern const struct sensor_decoder_api lsm6dso_combined_decoder;

#endif
