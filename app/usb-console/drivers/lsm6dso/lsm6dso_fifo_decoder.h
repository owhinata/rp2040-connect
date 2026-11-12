/* SPDX-License-Identifier: Apache-2.0 */
#ifndef LSM6DSO_FIFO_DECODER_H_
#define LSM6DSO_FIFO_DECODER_H_

#include <zephyr/drivers/sensor.h>
#include "lsm6dso_fifo_packet.h"

/* Decoders for the buffers the driver produces. */
extern const struct sensor_decoder_api lsm6dso_fifo_decoder;
extern const struct sensor_decoder_api lsm6dso_combined_decoder;

#endif
