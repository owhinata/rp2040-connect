/* SPDX-License-Identifier: MIT */
/* The board's check of I2C0, and of I2C1 when it is built, for the
 * "selftest" command (a test feature:
 * built with -DSELFTEST=ON only, freertos/drivers/i2c_dw/i2c_dw.cmake),
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
 *              -EINVAL; a write of no bytes to 0x6a (as Zephyr's "i2c
 *              scan" sends): -ENOTSUP; the next WHO_AM_I still reads 0x6c
 *   nack       a read from 0x10 (nothing there): -EIO ("NACK on
 *              i2c@40044000" and "IO Fail on i2c@40044000" in dmesg, rate
 *              limited), then WHO_AM_I reads 0x6c
 *   dma        a 35-byte read (5 FIFO words from FIFO_DATA_OUT_TAG, 0x78,
 *              as the FIFO stream reads; the FIFO is in bypass mode while
 *              idle, so the bytes are not checked) with the DMA reads on:
 *              0, dma_transfers +1, dma_bytes +35, no error or fallback;
 *              DMA channels 0 (TX) and 1 (RX: the lowest unclaimed in the
 *              Pico SDK's record, as nothing else of this build claims one)
 *              with Zephyr's CTRL values (0x00100015: 16 bits, read
 *              increment, DREQ 32, chained to itself, enabled; 0x00108821:
 *              8 bits, write increment, DREQ 33, chained to itself,
 *              enabled), IC_DMA_TDLR 8, IC_DMA_RDLR 0, IC_DMA_CR 0 after,
 *              neither channel's interrupt left enabled and both unclaimed
 *              again (i2c_dw_dma_os_rp2040.c); then the same read
 *              with the DMA reads off: 0, the counters unchanged, IC_CON
 *              without RX_FIFO_FULL_HLD_CTRL (0x165)
 *   i2c1       with CONFIG_I2C_DW_I2C1 only (-DI2C_DW_I2C1=ON), nothing
 *              connected to I2C1: i2c1 is ready; GPIO18 and GPIO19 are
 *              FUNCSEL I2C (3) with the pads IE, 4 mA, SCHMITT and the
 *              pull-up (0x5a); a 1-byte read from 0x10 at 100 kHz ends
 *              with -EIO (the NACK of an unanswered address; "NACK on
 *              i2c@40048000" and "IO Fail on i2c@40048000" in dmesg) in
 *              under 100 ms (not the 1 s timeout); IC_TAR 0x10, IC_CON
 *              0x163 (no RX_FIFO_FULL_HLD_CTRL: no DMA reads on i2c1); a
 *              40-byte read from 0x10, which the DMA path would take on
 *              i2c0: -EIO, the DMA counters unchanged; then WHO_AM_I on
 *              i2c0 still reads 0x6c.  An SDA or SCL held low on the pins
 *              gives -ETIME or -EIO instead (a FAIL)
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

#include "hardware/dma.h"
#include "hardware/structs/dma.h"
#include "hardware/structs/i2c.h"
#ifdef CONFIG_I2C_DW_I2C1
#include "hardware/structs/io_bank0.h"
#include "hardware/structs/pads_bank0.h"
#endif

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/i2c/i2c_dw_rp2040_dma.h>

#include "i2c_dw_selftest.h"

#define LSM6DSOX_ADDR      0x6a
#define LSM6DSOX_WHO_AM_I  0x0f
#define LSM6DSOX_ID        0x6c
#define LSM6DSOX_FIFO_DATA 0x78
#define ABSENT_ADDR        0x10
#define DMA_READ_LEN       35 /* 5 FIFO words; the DMA path takes 32 bytes and more */

#define IC_CON_DMA_STANDARD  0x363u
#define IC_CON_DMA_FAST_PLUS 0x365u
#define IC_CON_FAST_PLUS     0x165u
#define IC_CON_STANDARD      0x163u
#define PAD_I2C_PULL_UP      0x5au /* IE, DRIVE 4 mA, PUE, SCHMITT */
#define I2C1_READ_LEN        40
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
	if (i2c_dw_rp2040_dma_enabled()) {
		CHECK_EQ(i2c0_hw->con, IC_CON_DMA_STANDARD);
	}
}

static void test_fast_plus(void)
{
	CHECK_EQ(configure(I2C_SPEED_FAST_PLUS), 0);
	CHECK_EQ(who_am_i(), LSM6DSOX_ID);
	CHECK_EQ(i2c0_hw->fs_scl_hcnt, 31);
	CHECK_EQ(i2c0_hw->fs_scl_lcnt, 31);
	if (i2c_dw_rp2040_dma_enabled()) {
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
	msg.buf = &byte;
	msg.len = 0;
	CHECK_EQ(i2c_transfer(imu.bus, &msg, 1, LSM6DSOX_ADDR), -ENOTSUP);
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
	bool was_enabled = i2c_dw_rp2040_dma_enabled();
	struct i2c_dw_rp2040_dma_stats before, after;
	uint8_t buf[DMA_READ_LEN];
	/* The read takes the two lowest free channels, TX first, and gives them
	 * back (others, the NINA's SPI, keep theirs).
	 */
	unsigned int tx = 0, rx;

	while (tx < NUM_DMA_CHANNELS && dma_channel_is_claimed(tx)) {
		tx++;
	}
	rx = tx + 1;
	while (rx < NUM_DMA_CHANNELS && dma_channel_is_claimed(rx)) {
		rx++;
	}
	CHECK(rx < NUM_DMA_CHANNELS);
	if (rx >= NUM_DMA_CHANNELS) {
		return;
	}
	i2c_dw_rp2040_dma_set_enabled(true);
	i2c_dw_rp2040_dma_get_stats(&before);
	CHECK_EQ(i2c_burst_read_dt(&imu, LSM6DSOX_FIFO_DATA, buf, sizeof(buf)), 0);
	i2c_dw_rp2040_dma_get_stats(&after);
	CHECK_EQ(after.transfers - before.transfers, 1);
	CHECK_EQ(after.bytes - before.bytes, DMA_READ_LEN);
	CHECK_EQ(after.errors - before.errors, 0);
	CHECK_EQ(after.fallbacks - before.fallbacks, 0);
	/* CHAIN_TO (bits 11..14) holds the channel's own number: no chain. */
	CHECK_EQ(dma_hw->ch[tx].al1_ctrl, (DMA_CTRL_TX & ~0x7800u) | (tx << 11));
	CHECK_EQ(dma_hw->ch[rx].al1_ctrl, (DMA_CTRL_RX & ~0x7800u) | (rx << 11));
	CHECK_EQ(dma_hw->ch[tx].transfer_count, 0);
	CHECK_EQ(dma_hw->ch[rx].transfer_count, 0);
	CHECK_EQ(dma_hw->inte0 & (BIT(tx) | BIT(rx)), 0);
	CHECK_EQ(dma_hw->inte1 & (BIT(tx) | BIT(rx)), 0);
	CHECK_EQ(dma_channel_is_claimed(tx), false);
	CHECK_EQ(dma_channel_is_claimed(rx), false);
	CHECK_EQ(i2c0_hw->dma_tdlr, 8);
	CHECK_EQ(i2c0_hw->dma_rdlr, 0);
	CHECK_EQ(i2c0_hw->dma_cr, 0);

	i2c_dw_rp2040_dma_set_enabled(false);
	CHECK_EQ(i2c_burst_read_dt(&imu, LSM6DSOX_FIFO_DATA, buf, sizeof(buf)), 0);
	i2c_dw_rp2040_dma_get_stats(&before);
	CHECK_EQ(before.transfers, after.transfers);
	CHECK_EQ(before.bytes, after.bytes);
	CHECK_EQ(i2c0_hw->con, IC_CON_FAST_PLUS);
	CHECK_EQ(who_am_i(), LSM6DSOX_ID);

	i2c_dw_rp2040_dma_set_enabled(was_enabled);
}

#ifdef CONFIG_I2C_DW_I2C1
static void test_i2c1(void)
{
	const struct device *const bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	struct i2c_dw_rp2040_dma_stats before, after;
	uint8_t buf[I2C1_READ_LEN];
	int64_t start, elapsed;

	CHECK(device_is_ready(bus));
	if (!device_is_ready(bus)) {
		return;
	}
	for (unsigned int gpio = 18; gpio <= 19; gpio++) {
		CHECK_EQ(io_bank0_hw->io[gpio].ctrl & 0x1f, 3);
		CHECK_EQ(pads_bank0_hw->io[gpio] & 0xff, PAD_I2C_PULL_UP);
	}
	CHECK_EQ(i2c_configure(bus, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD)), 0);
	start = k_uptime_get();
	CHECK_EQ(i2c_read(bus, buf, 1, ABSENT_ADDR), -EIO);
	elapsed = k_uptime_get() - start;
	CHECK(elapsed < 100);
	CHECK_EQ(i2c1_hw->tar & 0x3ff, ABSENT_ADDR);
	CHECK_EQ(i2c1_hw->ss_scl_hcnt, 500);
	CHECK_EQ(i2c1_hw->con, IC_CON_STANDARD);

	i2c_dw_rp2040_dma_get_stats(&before);
	CHECK_EQ(i2c_read(bus, buf, sizeof(buf), ABSENT_ADDR), -EIO);
	i2c_dw_rp2040_dma_get_stats(&after);
	CHECK_EQ(after.transfers - before.transfers, 0);
	CHECK_EQ(after.fallbacks - before.fallbacks, 0);
	CHECK_EQ(after.errors - before.errors, 0);
	CHECK_EQ(i2c1_hw->dma_cr, 0);

	CHECK_EQ(who_am_i(), LSM6DSOX_ID);
}
#endif

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
#ifdef CONFIG_I2C_DW_I2C1
	{"i2c1", test_i2c1},
#endif
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
