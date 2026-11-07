/* SPDX-License-Identifier: Apache-2.0 */

#include "lsm6dso_fifo_hw.h"
#include "lsm6dso_fifo.h"

#include <errno.h>

int lsm6dso_fifo_route_int1(const stmdev_ctx_t *ctx, bool enabled)
{
	lsm6dso_pin_int1_route_t route;

	if (ctx == NULL) {
		return -EINVAL;
	}
	if (lsm6dso_pin_int1_route_get(ctx, &route) != 0) {
		return -EIO;
	}
	route.fifo_th = enabled;
	return lsm6dso_pin_int1_route_set(ctx, route) == 0 ? 0 : -EIO;
}

int lsm6dso_fifo_stop(const stmdev_ctx_t *ctx)
{
	if (ctx == NULL) {
		return -EINVAL;
	}
	return lsm6dso_fifo_mode_set(ctx, LSM6DSO_BYPASS_MODE) == 0 ? 0 : -EIO;
}

int lsm6dso_fifo_pause_batch(const stmdev_ctx_t *ctx)
{
	if (ctx == NULL) {
		return -EINVAL;
	}
	if (lsm6dso_fifo_xl_batch_set(ctx, LSM6DSO_XL_NOT_BATCHED) != 0 ||
	    lsm6dso_fifo_gy_batch_set(ctx, LSM6DSO_GY_NOT_BATCHED) != 0) {
		return -EIO;
	}
	return 0;
}

int lsm6dso_fifo_configure(const stmdev_ctx_t *ctx, uint16_t watermark,
			   uint8_t accel_batch, uint8_t gyro_batch)
{
	if (ctx == NULL || watermark == 0 || watermark > 511 || accel_batch > 10 ||
	    gyro_batch > 10 || (accel_batch == 0 && gyro_batch == 0)) {
		return -EINVAL;
	}

	/* Always enter bypass before changing FIFO configuration. */
	if (lsm6dso_fifo_stop(ctx) != 0 ||
	    lsm6dso_fifo_xl_batch_set(ctx, (lsm6dso_bdr_xl_t)accel_batch) != 0 ||
	    lsm6dso_fifo_gy_batch_set(ctx, (lsm6dso_bdr_gy_t)gyro_batch) != 0 ||
	    lsm6dso_fifo_temp_batch_set(ctx, LSM6DSO_TEMP_NOT_BATCHED) != 0 ||
	    lsm6dso_fifo_timestamp_decimation_set(ctx, LSM6DSO_NO_DECIMATION) != 0 ||
	    lsm6dso_fifo_stop_on_wtm_set(ctx, 0) != 0 ||
	    lsm6dso_compression_algo_set(ctx, LSM6DSO_CMP_DISABLE) != 0 ||
	    lsm6dso_fifo_watermark_set(ctx, watermark) != 0 ||
	    lsm6dso_fifo_mode_set(ctx, LSM6DSO_STREAM_MODE) != 0) {
		/* Failure of this best-effort rollback must be handled by the caller. */
		(void)lsm6dso_fifo_stop(ctx);
		return -EIO;
	}
	return 0;
}

int lsm6dso_fifo_status_read(const struct i2c_dt_spec *bus,
			     struct lsm6dso_fifo_status *status)
{
	uint8_t raw[2];
	int ret;

	if (bus == NULL || status == NULL) {
		return -EINVAL;
	}
	ret = i2c_burst_read_dt(bus, LSM6DSO_FIFO_STATUS1, raw, sizeof(raw));
	if (ret != 0) {
		return ret;
	}
	status->words = raw[0] | ((uint16_t)(raw[1] & 0x03U) << 8);
	status->watermark = (raw[1] & 0x80U) != 0;
	status->full = (raw[1] & 0x20U) != 0;
	status->overrun = (raw[1] & 0x40U) != 0;
	status->overrun_latched = (raw[1] & 0x08U) != 0;
	return 0;
}

int lsm6dso_fifo_burst_read(const struct i2c_dt_spec *bus, uint8_t *buffer,
			    size_t words)
{
	if (bus == NULL || buffer == NULL || words == 0 || words > LSM6DSO_FIFO_MAX_WORDS) {
		return -EINVAL;
	}
	return i2c_burst_read_dt(bus, LSM6DSO_FIFO_DATA_OUT_TAG, buffer,
				 words * LSM6DSO_FIFO_WORD_SIZE);
}
