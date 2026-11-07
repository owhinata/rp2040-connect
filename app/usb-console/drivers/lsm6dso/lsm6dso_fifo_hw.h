/* SPDX-License-Identifier: Apache-2.0 */

#ifndef LSM6DSO_FIFO_HW_H_
#define LSM6DSO_FIFO_HW_H_

#include <zephyr/drivers/i2c.h>
#include "lsm6dso_reg.h"

struct lsm6dso_fifo_status {
	uint16_t words;
	bool watermark;
	bool full;
	bool overrun;
	bool overrun_latched;
};

/* Internal primitives. The caller must serialize all access to the IMU, set
 * the sensor ODRs first, and arrange interrupt routing and error recovery.
 * Batch values are the ST register encodings (0 disables that sensor).
 */
int lsm6dso_fifo_configure(const stmdev_ctx_t *ctx, uint16_t watermark,
			   uint8_t accel_batch, uint8_t gyro_batch);
int lsm6dso_fifo_stop(const stmdev_ctx_t *ctx);
/* Stop adding both sensors while retaining the unread FIFO contents. */
int lsm6dso_fifo_pause_batch(const stmdev_ctx_t *ctx);
int lsm6dso_fifo_route_int1(const stmdev_ctx_t *ctx, bool enabled);
int lsm6dso_fifo_status_read(const struct i2c_dt_spec *bus,
			     struct lsm6dso_fifo_status *status);
int lsm6dso_fifo_burst_read(const struct i2c_dt_spec *bus, uint8_t *buffer,
			    size_t words);

#endif
