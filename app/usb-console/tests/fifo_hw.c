/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <errno.h>
#include <string.h>
#include "lsm6dso_fifo_hw.h"
#include "lsm6dso_fifo.h"

static int calls, fail_call, last_mode, last_xl, last_gyro, last_timestamp;
static int i2c_result;
static uint8_t status_bytes[2], last_reg;
static size_t last_length;
static lsm6dso_pin_int1_route_t int1_route;

static int fake_set(void)
{
	return ++calls == fail_call ? -1 : 0;
}
int32_t lsm6dso_fifo_mode_set(const stmdev_ctx_t *ctx, int value)
{
	(void)ctx;
	last_mode = value;
	return fake_set();
}
int32_t lsm6dso_pin_int1_route_get(const stmdev_ctx_t *ctx, lsm6dso_pin_int1_route_t *route)
{
	(void)ctx;
	int result = fake_set();
	if (!result) { *route = int1_route; }
	return result;
}
int32_t lsm6dso_pin_int1_route_set(const stmdev_ctx_t *ctx, lsm6dso_pin_int1_route_t route)
{
	(void)ctx;
	int result = fake_set();
	if (!result) { int1_route = route; }
	return result;
}
#define FAKE_SET(name, type) \
	int32_t name(const stmdev_ctx_t *ctx, type value) \
	{ (void)ctx; (void)value; return fake_set(); }
int32_t lsm6dso_fifo_xl_batch_set(const stmdev_ctx_t *ctx, int value)
{
	(void)ctx;
	last_xl = value;
	return fake_set();
}
int32_t lsm6dso_fifo_gy_batch_set(const stmdev_ctx_t *ctx, int value)
{
	(void)ctx;
	last_gyro = value;
	return fake_set();
}
FAKE_SET(lsm6dso_fifo_temp_batch_set, int)
int32_t lsm6dso_fifo_timestamp_decimation_set(const stmdev_ctx_t *ctx, int value)
{
	(void)ctx;
	last_timestamp = value;
	return fake_set();
}
FAKE_SET(lsm6dso_fifo_stop_on_wtm_set, uint8_t)
FAKE_SET(lsm6dso_compression_algo_set, int)
FAKE_SET(lsm6dso_fifo_watermark_set, uint16_t)

int i2c_burst_read_dt(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t *buf, size_t len)
{
	(void)bus;
	last_reg = reg;
	last_length = len;
	if (i2c_result == 0 && reg == LSM6DSO_FIFO_STATUS1) {
		assert(len == sizeof(status_bytes));
		memcpy(buf, status_bytes, len);
	}
	return i2c_result;
}

int main(void)
{
	stmdev_ctx_t ctx = {0};
	struct i2c_dt_spec bus = {0};
	struct lsm6dso_fifo_status status = {0};
	uint8_t buf[LSM6DSO_FIFO_MAX_WORDS * LSM6DSO_FIFO_WORD_SIZE];

	assert(lsm6dso_fifo_configure(&ctx, 0, 1, 1, 0) == -EINVAL);
	assert(lsm6dso_fifo_configure(&ctx, 512, 1, 1, 0) == -EINVAL);
	assert(lsm6dso_fifo_configure(&ctx, 10, 0, 0, 0) == -EINVAL);
	assert(lsm6dso_fifo_configure(&ctx, 10, 11, 0, 0) == -EINVAL);
	assert(lsm6dso_fifo_configure(&ctx, 10, 1, 1, 4) == -EINVAL);
	assert(calls == 0);
	assert(lsm6dso_fifo_configure(&ctx, 32, 4, 4, 0) == 0);
	assert(calls == 9 && last_mode == LSM6DSO_STREAM_MODE && last_timestamp == 0);
	for (int decimation = 1; decimation <= 3; decimation++) {
		calls = 0;
		assert(lsm6dso_fifo_configure(&ctx, 32, 4, 4, decimation) == 0);
		assert(last_timestamp == decimation);
	}
	/* Every individual failed configuration write attempts a bypass rollback. */
	for (int failed = 1; failed <= 9; failed++) {
		calls = 0;
		fail_call = failed;
		last_mode = -1;
		assert(lsm6dso_fifo_configure(&ctx, 32, 4, 4, 0) == -EIO);
		assert(calls == failed + 1 && last_mode == LSM6DSO_BYPASS_MODE);
	}
	fail_call = 0;
	calls = 0;
	last_mode = LSM6DSO_STREAM_MODE;
	last_xl = last_gyro = -1;
	assert(lsm6dso_fifo_pause_batch(&ctx) == 0);
	assert(calls == 2 && last_xl == 0 && last_gyro == 0);
	assert(last_mode == LSM6DSO_STREAM_MODE); /* Unread FIFO must remain intact. */
	for (int failed = 1; failed <= 2; failed++) {
		calls = 0;
		fail_call = failed;
		assert(lsm6dso_fifo_pause_batch(&ctx) == -EIO);
		assert(calls == failed && last_mode == LSM6DSO_STREAM_MODE);
	}
	fail_call = 0;
	calls = 0;
	int1_route.drdy_xl = 1;
	assert(lsm6dso_fifo_route_int1(&ctx, true) == 0);
	assert(int1_route.fifo_th == 1 && int1_route.drdy_xl == 1);
	assert(lsm6dso_fifo_route_int1(&ctx, false) == 0);
	assert(int1_route.fifo_th == 0 && int1_route.drdy_xl == 1);
	calls = 0;
	fail_call = 1;
	assert(lsm6dso_fifo_route_int1(&ctx, true) == -EIO && calls == 1);
	calls = 0;
	fail_call = 2;
	assert(lsm6dso_fifo_route_int1(&ctx, true) == -EIO && int1_route.fifo_th == 0);
	fail_call = 0;
	status_bytes[0] = 0;
	status_bytes[1] = 0xea; /* 512 words; watermark, full, overrun, latched */
	assert(lsm6dso_fifo_status_read(&bus, &status) == 0);
	assert(status.words == 512 && status.watermark && status.full && status.overrun &&
	       status.overrun_latched);
	assert(lsm6dso_fifo_burst_read(&bus, buf, 512) == 0);
	assert(last_reg == LSM6DSO_FIFO_DATA_OUT_TAG && last_length == sizeof(buf));
	assert(lsm6dso_fifo_burst_read(&bus, buf, 513) == -EINVAL);
	assert(lsm6dso_fifo_burst_read(&bus, buf, 0) == -EINVAL);
	i2c_result = -EIO;
	assert(lsm6dso_fifo_status_read(&bus, &status) == -EIO);
	assert(lsm6dso_fifo_burst_read(&bus, buf, 1) == -EIO);
	return 0;
}
