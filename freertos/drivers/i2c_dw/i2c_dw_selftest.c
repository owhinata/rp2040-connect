/* SPDX-License-Identifier: MIT */
/* The board's check of I2C0 for the "selftest" command (a test feature:
 * built with -DNANO_SELFTEST=ON only, freertos/drivers/i2c_dw/i2c_dw.cmake),
 * through Zephyr's I2C API on the device i2c0 and the board's LSM6DSOX at
 * 0x6a, with the controller's and the DMA's registers read back:
 *
 *   ready      i2c0 is ready after the start
 *   standard   i2c_configure(standard); WHO_AM_I (0x0f) reads 0x6c on the
 *              interrupt path; IC_TAR 0x6a, IC_SS_SCL_HCNT 500,
 *              IC_SS_SCL_LCNT 625 (125 MHz), IC_FS_SPKLEN 1, IC_CON 0x363
 *              (with the DMA reads on: master, standard, restart, slave
 *              off, TX_EMPTY_CTRL, RX_FIFO_FULL_HLD_CTRL)
 *   fast_plus  i2c_configure(fast plus) as the IMU service sets it;
 *              WHO_AM_I; IC_FS_SCL_HCNT 31, IC_FS_SCL_LCNT 31, IC_CON 0x365
 *   refused    i2c_configure() of high speed and of 10-bit addresses,
 *              a transfer to 0x80 and one with a length and no buffer:
 *              -EINVAL, and the next WHO_AM_I still reads 0x6c
 *   nack       a read from 0x10 (nothing there): -EIO ("NACK on
 *              i2c@40044000" and "IO Fail on i2c@40044000" in dmesg, rate
 *              limited), then WHO_AM_I reads 0x6c
 *   dma        a 35-byte read (5 FIFO words from FIFO_DATA_OUT_TAG, 0x78,
 *              as the FIFO stream reads; the FIFO is in bypass mode while
 *              idle, so the bytes are not checked) with the DMA reads on:
 *              0, dma_transfers +1, dma_bytes +35, no error or fallback;
 *              DMA channels 0 (TX, on DMA_IRQ_0) and 1 (RX, on DMA_IRQ_1)
 *              with Zephyr's CTRL values (0x00100015: 16 bits, read
 *              increment, DREQ 32, chained to itself, enabled; 0x00108821:
 *              8 bits, write increment, DREQ 33, chained to itself,
 *              enabled), IC_DMA_TDLR 8, IC_DMA_RDLR 0, IC_DMA_CR 0 after,
 *              neither channel's interrupt left enabled; then the same read
 *              with the DMA reads off: 0, the counters unchanged, IC_CON
 *              without RX_FIFO_FULL_HLD_CTRL (0x165)
 *
 * It ends with the DMA reads as they were and the bus at the standard speed
 * (the class at the start of the board).  Run it while the IMU is idle (no
 * stream: the dma check reads the FIFO).  Not a host test: it needs the
 * board.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/structs/dma.h"
#include "hardware/structs/i2c.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#include "i2c_dw_selftest.h"
#include "nano_i2c_dma.h"

#define LSM6DSOX_ADDR      0x6a
#define LSM6DSOX_WHO_AM_I  0x0f
#define LSM6DSOX_ID        0x6c
#define LSM6DSOX_FIFO_DATA 0x78
#define ABSENT_ADDR        0x10
#define DMA_READ_LEN       35 /* 5 FIFO words; the DMA path takes 32 bytes and more */

#define IC_CON_DMA_STANDARD  0x363u
#define IC_CON_DMA_FAST_PLUS 0x365u
#define IC_CON_FAST_PLUS     0x165u
#define DMA_CTRL_TX          0x00100015u
#define DMA_CTRL_RX          0x00108821u

static const struct i2c_dt_spec imu = {
	.bus = DEVICE_DT_GET(DT_NODELABEL(i2c0)),
	.addr = LSM6DSOX_ADDR,
};

static int failures;

static void check_eq(long long got, long long want, int line, const char *what)
{
	if (got != want) {
		printf("  FAIL line %d: %s: %lld (0x%llx), expected %lld (0x%llx)\n", line, what,
		       got, (unsigned long long)got, want, (unsigned long long)want);
		failures++;
	}
}

#define CHECK(cond)         check_eq((cond) ? 1 : 0, 1, __LINE__, #cond)
#define CHECK_EQ(got, want) check_eq((long long)(got), (long long)(want), __LINE__, #got)

static int who_am_i(void)
{
	uint8_t id = 0;
	int ret = i2c_burst_read_dt(&imu, LSM6DSOX_WHO_AM_I, &id, 1);

	return ret != 0 ? ret : id;
}

static int configure(uint32_t speed)
{
	return i2c_configure(imu.bus, I2C_MODE_CONTROLLER | I2C_SPEED_SET(speed));
}

static void test_ready(void)
{
	CHECK(i2c_is_ready_dt(&imu));
}

static void test_standard(void)
{
	CHECK_EQ(configure(I2C_SPEED_STANDARD), 0);
	CHECK_EQ(who_am_i(), LSM6DSOX_ID);
	CHECK_EQ(i2c0_hw->tar & 0x3ff, LSM6DSOX_ADDR);
	CHECK_EQ(i2c0_hw->ss_scl_hcnt, 500);
	CHECK_EQ(i2c0_hw->ss_scl_lcnt, 625);
	CHECK_EQ(i2c0_hw->fs_spklen, 1);
	if (nano_i2c_dma_enabled()) {
		CHECK_EQ(i2c0_hw->con, IC_CON_DMA_STANDARD);
	}
}

static void test_fast_plus(void)
{
	CHECK_EQ(configure(I2C_SPEED_FAST_PLUS), 0);
	CHECK_EQ(who_am_i(), LSM6DSOX_ID);
	CHECK_EQ(i2c0_hw->fs_scl_hcnt, 31);
	CHECK_EQ(i2c0_hw->fs_scl_lcnt, 31);
	if (nano_i2c_dma_enabled()) {
		CHECK_EQ(i2c0_hw->con, IC_CON_DMA_FAST_PLUS);
	}
}

static void test_refused(void)
{
	struct i2c_msg msg = {.buf = NULL, .len = 1, .flags = I2C_MSG_WRITE};
	uint8_t byte = 0;

	CHECK_EQ(configure(I2C_SPEED_HIGH), -EINVAL);
	CHECK_EQ(i2c_configure(imu.bus, I2C_MODE_CONTROLLER | I2C_ADDR_10_BITS |
						I2C_SPEED_SET(I2C_SPEED_FAST_PLUS)),
		 -EINVAL);
	CHECK_EQ(i2c_read(imu.bus, &byte, 1, 0x80), -EINVAL);
	CHECK_EQ(i2c_transfer(imu.bus, &msg, 1, LSM6DSOX_ADDR), -EINVAL);
	CHECK_EQ(who_am_i(), LSM6DSOX_ID);
	CHECK_EQ(i2c0_hw->fs_scl_hcnt, 31);
}

static void test_nack(void)
{
	uint8_t byte = 0;

	CHECK_EQ(i2c_read(imu.bus, &byte, 1, ABSENT_ADDR), -EIO);
	CHECK_EQ(who_am_i(), LSM6DSOX_ID);
}

static void test_dma(void)
{
	bool was_enabled = nano_i2c_dma_enabled();
	struct nano_i2c_dma_stats before, after;
	uint8_t buf[DMA_READ_LEN];

	nano_i2c_dma_set_enabled(true);
	nano_i2c_dma_get_stats(&before);
	CHECK_EQ(i2c_burst_read_dt(&imu, LSM6DSOX_FIFO_DATA, buf, sizeof(buf)), 0);
	nano_i2c_dma_get_stats(&after);
	CHECK_EQ(after.transfers - before.transfers, 1);
	CHECK_EQ(after.bytes - before.bytes, DMA_READ_LEN);
	CHECK_EQ(after.errors - before.errors, 0);
	CHECK_EQ(after.fallbacks - before.fallbacks, 0);
	CHECK_EQ(dma_hw->ch[0].al1_ctrl, DMA_CTRL_TX);
	CHECK_EQ(dma_hw->ch[1].al1_ctrl, DMA_CTRL_RX);
	CHECK_EQ(dma_hw->ch[0].transfer_count, 0);
	CHECK_EQ(dma_hw->ch[1].transfer_count, 0);
	CHECK_EQ(dma_hw->inte0 & 0x3u, 0);
	CHECK_EQ(dma_hw->inte1 & 0x3u, 0);
	CHECK_EQ(i2c0_hw->dma_tdlr, 8);
	CHECK_EQ(i2c0_hw->dma_rdlr, 0);
	CHECK_EQ(i2c0_hw->dma_cr, 0);

	nano_i2c_dma_set_enabled(false);
	CHECK_EQ(i2c_burst_read_dt(&imu, LSM6DSOX_FIFO_DATA, buf, sizeof(buf)), 0);
	nano_i2c_dma_get_stats(&before);
	CHECK_EQ(before.transfers, after.transfers);
	CHECK_EQ(before.bytes, after.bytes);
	CHECK_EQ(i2c0_hw->con, IC_CON_FAST_PLUS);
	CHECK_EQ(who_am_i(), LSM6DSOX_ID);

	nano_i2c_dma_set_enabled(was_enabled);
}

static const struct {
	const char *name;
	void (*fn)(void);
} tests[] = {
	/* clang-format off */
	{"ready", test_ready},
	{"standard", test_standard},
	{"fast_plus", test_fast_plus},
	{"refused", test_refused},
	{"nack", test_nack},
	{"dma", test_dma},
	/* clang-format on */
};

int i2c_dw_selftest_run(void)
{
	size_t count = sizeof(tests) / sizeof(tests[0]);
	int failed = 0;

	for (size_t i = 0; i < count; i++) {
		failures = 0;
		printf("i2c %s\n", tests[i].name);
		if (i > 0 && !i2c_is_ready_dt(&imu)) {
			printf("  skipped (i2c0 not ready)\n");
			failed++;
			continue;
		}
		tests[i].fn();
		printf("  %s\n", failures == 0 ? "ok" : "FAILED");
		failed += failures != 0;
	}
	(void)configure(I2C_SPEED_STANDARD);
	printf("selftest: i2c: %d of %u tests failed\n", failed, (unsigned int)count);
	return failed;
}
