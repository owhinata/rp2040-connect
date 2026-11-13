/* SPDX-License-Identifier: Apache-2.0 */
/* boards/arduino-nano-rp2040-connect/src/nano_i2c_dw.h: the OS-independent
 * pieces of the I2C0 driver (nano_i2c.c) against the Zephyr build's
 * driver (zephyr/app/usb-console/drivers/i2c_dw/i2c_dw.c, i2c_dw.h) and
 * Zephyr's i2c.h, written out below as the reference:
 *
 * - SCL counts of i2c_dw_configure() for CONFIG_I2C_DW_CLOCK_SPEED=125 (and
 *   other clocks, for the minimum clamps);
 * - the frequencies the port takes (i2c_map_dt_bitrate());
 * - the words written to IC_DATA_CMD for a whole transfer, for the message
 *   lists that NuttX users send (the i2c tool, a register read and write
 *   of a sensor driver) and the Zephyr calls that do the same
 *   (i2c_write_read(), i2c_burst_write(), i2c_read(), i2c_write()), across
 *   consecutive transfers (the RESTART that a change of direction from the
 *   previous transfer adds).
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nano_i2c_dw.h"

/* ---- Reference: Zephyr's i2c_dw.h:82-133 with CONFIG_I2C_DW_CLOCK_SPEED
 * as a variable and IC_CLK_FREQ_OPTIMIZATION off; i2c_dw_configure()
 * (i2c_dw.c:1397-1435) with offsets 0 and spike lengths 1.
 */

static int ref_configure(uint32_t speed, uint32_t mhz, bool hs, uint32_t *h, uint32_t *l)
{
	uint32_t lcnt_val = 0, hcnt_val = 0;
	int rc = 0;
	uint8_t fs_spk_len = 1, hs_spk_len = 1;

	switch (speed) {
	case 1: lcnt_val = mhz * 5; hcnt_val = mhz * 4; break;
	case 2: lcnt_val = (mhz * 7) / 8; hcnt_val = (mhz * 6) / 8; break;
	case 3: lcnt_val = (mhz * 2) / 8; hcnt_val = (mhz * 2) / 8; break;
	case 4:
		if (hs) { lcnt_val = (mhz * 7) / 8; hcnt_val = (mhz * 6) / 8; }
		else { rc = -EINVAL; }
		break;
	default: rc = -EINVAL;
	}
	if (speed == 4) {
		lcnt_val = lcnt_val < hs_spk_len + 8u ? hs_spk_len + 8u : lcnt_val;
		hcnt_val = hcnt_val < hs_spk_len + 6u ? hs_spk_len + 6u : hcnt_val;
	} else {
		lcnt_val = lcnt_val < fs_spk_len + 8u ? fs_spk_len + 8u : lcnt_val;
		hcnt_val = hcnt_val < fs_spk_len + 6u ? fs_spk_len + 6u : hcnt_val;
	}
	*h = hcnt_val;
	*l = lcnt_val;
	return rc;
}

static void test_scl_counts(void)
{
	uint16_t h, l;
	uint32_t rh, rl;
	uint32_t mhz, speed;
	int hs;

	/* The values the Zephyr build writes (125 MHz clk_peri). */
	assert(nano_i2c_dw_scl_counts(NANO_I2C_SPEED_STANDARD, 125, false, &h, &l) == 0);
	assert(h == 500 && l == 625);
	assert(nano_i2c_dw_scl_counts(NANO_I2C_SPEED_FAST, 125, false, &h, &l) == 0);
	assert(h == 93 && l == 109);
	assert(nano_i2c_dw_scl_counts(NANO_I2C_SPEED_FAST_PLUS, 125, false, &h, &l) == 0);
	assert(h == 31 && l == 31);
	/* RP2040 has no high speed (IC_CON resets to FAST). */
	assert(nano_i2c_dw_scl_counts(NANO_I2C_SPEED_HIGH, 125, false, &h, &l) == -EINVAL);
	assert(h == 7 && l == 9);
	assert(nano_i2c_dw_scl_counts(NANO_I2C_SPEED_ULTRA, 125, false, &h, &l) == -EINVAL);
	assert(nano_i2c_dw_scl_counts(0, 125, false, &h, &l) == -EINVAL);

	for (mhz = 1; mhz <= 400; mhz++) {
		for (speed = 0; speed <= 6; speed++) {
			for (hs = 0; hs <= 1; hs++) {
				int rc = nano_i2c_dw_scl_counts(speed, mhz, hs, &h, &l);
				int rrc = ref_configure(speed, mhz, hs, &rh, &rl);

				assert(rc == rrc && h == rh && l == rl);
			}
		}
	}
}

static void test_speed(void)
{
	assert(nano_i2c_dw_speed(100000) == NANO_I2C_SPEED_STANDARD);
	assert(nano_i2c_dw_speed(400000) == NANO_I2C_SPEED_FAST);
	assert(nano_i2c_dw_speed(1000000) == NANO_I2C_SPEED_FAST_PLUS);
	assert(nano_i2c_dw_speed(3400000) == NANO_I2C_SPEED_HIGH);
	assert(nano_i2c_dw_speed(5000000) == NANO_I2C_SPEED_ULTRA);
	assert(nano_i2c_dw_speed(0) == 0);
	assert(nano_i2c_dw_speed(50000) == 0);
	assert(nano_i2c_dw_speed(99999) == 0);
	assert(nano_i2c_dw_speed(1000001) == 0);
}

/* ---- The words a transfer writes to IC_DATA_CMD (FIFO splits aside). */

struct msg {
	uint16_t flags;   /* NuttX I2C_M_* or Zephyr I2C_MSG_* */
	const uint8_t *buf;
	uint32_t len;
};

#define MAXW 64

/* The port: nano_i2c.c's i2c_dw_transfer() loop with the helpers that
 * i2c_dw_data_send() and i2c_dw_data_ask() call.  *xfr_flags carries over
 * between transfers as dw->xfr_flags does.
 */

static int port_words(uint8_t *xfr_flags, const struct msg *m, int count, uint32_t *w)
{
	int n = 0;

	for (int i = 0; i < count; i++) {
		uint8_t f = nano_i2c_dw_msg_flags(i > 0 ? m[i - 1].flags : 0, m[i].flags,
						  i + 1 < count ? m[i + 1].flags : 0, i == 0,
						  i == count - 1);
		uint32_t len = m[i].len;
		uint8_t pflags = *xfr_flags;

		if (len == 0 && m[i].buf != NULL) {
			len = 1;
		}
		*xfr_flags = nano_i2c_dw_next_flags(pflags, f);
		if ((*xfr_flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			for (uint32_t k = 0; k < len; k++) {
				w[n++] = nano_i2c_dw_write_cmd(xfr_flags, m[i].buf[k], len - k);
			}
		} else {
			for (uint32_t k = 0; k < len; k++) {
				w[n++] = nano_i2c_dw_read_cmd(xfr_flags, len - k);
			}
		}
	}
	return n;
}

/* Reference: Zephyr.  z_impl_i2c_transfer() (i2c.h:1012-1018) sets STOP on
 * the last message; i2c_dw_transfer() (i2c_dw.c:1262-1272) adds RESTART on
 * a change of direction; i2c_dw_data_send() (:518-529) and
 * i2c_dw_data_ask() (:430-441) write the words.  Written out here, not
 * through nano_i2c_dw.h.
 */

static int zephyr_words(uint8_t *xfr_flags, struct msg *m, int num, uint32_t *w)
{
	int n = 0;

	if (!num) {
		return 0;
	}
	m[num - 1].flags |= BIT(1);
	for (int i = 0; i < num; i++) {
		uint8_t pflags = *xfr_flags;
		uint32_t len = m[i].len;

		if (len == 0 && m[i].buf != NULL) {
			len = 1;
		}
		*xfr_flags = (uint8_t)m[i].flags;
		if ((pflags & BIT(0)) != (*xfr_flags & BIT(0))) {
			*xfr_flags |= BIT(2);
		}
		for (uint32_t k = 0; k < len; k++) {
			uint32_t data = (*xfr_flags & BIT(0)) ? BIT(8) : m[i].buf[k];

			if (*xfr_flags & BIT(2)) {
				data |= BIT(10);
				*xfr_flags &= ~BIT(2);
			}
			if ((*xfr_flags & BIT(1)) && (len - k) == 1) {
				data |= BIT(9);
			}
			w[n++] = data;
		}
	}
	return n;
}

/* Zephyr flags (zephyr/include/zephyr/drivers/i2c.h:204-223) */
#define ZW 0u
#define ZR BIT(0)
#define ZS BIT(1)
#define ZRS BIT(2)
/* NuttX flags */
#define NR NANO_I2C_M_READ
#define NNS NANO_I2C_M_NOSTOP
#define NNST NANO_I2C_M_NOSTART

static uint8_t g_port_flags, g_zephyr_flags;

/* One transfer through both, with the carried flags; the words must be
 * the same and, when given, equal to want.
 */
static void same(const struct msg *nuttx, int nn, struct msg *zephyr, int zn,
		 const uint32_t *want, int wn)
{
	uint32_t a[MAXW], b[MAXW];
	int na = port_words(&g_port_flags, nuttx, nn, a);
	int nb = zephyr_words(&g_zephyr_flags, zephyr, zn, b);

	assert(na == nb);
	assert(memcmp(a, b, na * sizeof(a[0])) == 0);
	assert(g_port_flags == g_zephyr_flags);
	if (want) {
		assert(na == wn && memcmp(a, want, wn * sizeof(a[0])) == 0);
	}
}

static void test_words(void)
{
	static const uint8_t reg0f = 0x0f, reg07 = 0x07;
	static const uint8_t data2[2] = {0x5a, 0x00};
	static const uint8_t regdata[3] = {0x07, 0x5a, 0x00};
	uint8_t rx[16];
	const uint32_t RD = IC_DATA_CMD_CMD, ST = IC_DATA_CMD_STOP, RS = IC_DATA_CMD_RESTART;

	rx[0] = 0x33;
	/* After boot dw->xfr_flags is 0 (a write). */
	g_port_flags = g_zephyr_flags = 0;

	/* WHO_AM_I: the i2c tool's "get -r 0f" and a driver's register read
	 * (NOSTOP, READ) = Zephyr's i2c_write_read() (WRITE, RESTART|READ|STOP).
	 */
	{
		struct msg n[] = {{NNS, &reg0f, 1}, {NR, rx, 1}};
		struct msg z[] = {{ZW, &reg0f, 1}, {ZRS | ZR | ZS, rx, 1}};
		const uint32_t want[] = {0x0f, RD | RS | ST};

		same(n, 2, z, 2, want, 2);
	}
	/* The next transfer starts with a write after a read: RESTART on its
	 * first word in both.  Burst write: register (NOSTOP), data (NOSTART)
	 * = Zephyr's i2c_burst_write() (WRITE, WRITE|STOP).
	 */
	{
		struct msg n[] = {{NNS, &reg07, 1}, {NNST, data2, 2}};
		struct msg z[] = {{ZW, &reg07, 1}, {ZW | ZS, data2, 2}};
		const uint32_t want[] = {0x07 | RS, 0x5a, 0x00 | ST};

		same(n, 2, z, 2, want, 3);
	}
	/* One message with register and data (i2c_write()); no RESTART now. */
	{
		struct msg n[] = {{0, regdata, 3}};
		struct msg z[] = {{ZW, regdata, 3}};
		const uint32_t want[] = {0x07, 0x5a, 0x00 | ST};

		same(n, 1, z, 1, want, 3);
	}
	/* A 6-byte register read. */
	{
		struct msg n[] = {{NNS, &reg0f, 1}, {NR, rx, 6}};
		struct msg z[] = {{ZW, &reg0f, 1}, {ZRS | ZR | ZS, rx, 6}};
		const uint32_t want[] = {0x0f, RD | RS, RD, RD, RD, RD, RD | ST};

		same(n, 2, z, 2, want, 7);
	}
	/* Read with no register (i2c_read(), the i2c tool without -r):
	 * after a read, no RESTART.
	 */
	{
		struct msg n[] = {{NR, rx, 2}};
		struct msg z[] = {{ZR | ZS, rx, 2}};
		const uint32_t want[] = {RD, RD | ST};

		same(n, 1, z, 1, want, 2);
	}
	/* The i2c tool's "dev" probe: a 1-byte read; then Zephyr's scan, a
	 * zero-length write with a buffer (sent as one byte).
	 */
	{
		struct msg n[] = {{NR, rx, 1}};
		struct msg z[] = {{ZR | ZS, rx, 1}};
		const uint32_t want[] = {RD | ST};

		same(n, 1, z, 1, want, 1);
	}
	{
		struct msg n[] = {{0, rx, 0}};
		struct msg z[] = {{ZW | ZS, rx, 0}};
		const uint32_t want[] = {rx[0] | RS | ST};

		same(n, 1, z, 1, want, 1);
	}
	/* The i2c tool's "set -s": register (NOSTOP), data (no NOSTART) is a
	 * repeated start into a second write = Zephyr WRITE, RESTART|WRITE|STOP.
	 */
	{
		struct msg n[] = {{NNS, &reg07, 1}, {0, data2, 1}};
		struct msg z[] = {{ZW, &reg07, 1}, {ZRS | ZW | ZS, data2, 1}};
		const uint32_t want[] = {0x07, 0x5a | RS | ST};

		same(n, 2, z, 2, want, 2);
	}
	/* Two messages without NOSTOP: STOP after each, the second starts with
	 * the controller's own START (Zephyr: WRITE|STOP, READ|STOP; the change
	 * of direction still sets RESTART on the read).
	 */
	{
		struct msg n[] = {{0, &reg0f, 1}, {NR, rx, 1}};
		struct msg z[] = {{ZW | ZS, &reg0f, 1}, {ZR | ZS, rx, 1}};
		const uint32_t want[] = {0x0f | ST, RD | RS | ST};

		same(n, 2, z, 2, want, 2);
	}
	/* NOSTOP on the last message: STOP all the same (Zephyr sets it). */
	{
		struct msg n[] = {{NNS | NR, rx, 2}};
		struct msg z[] = {{ZR, rx, 2}};
		const uint32_t want[] = {RD, RD | ST};

		same(n, 1, z, 1, want, 2);
	}
	/* Three messages: write, continuation, then a repeated-start read. */
	{
		struct msg n[] = {{NNS, &reg07, 1}, {NNS | NNST, data2, 2}, {NR, rx, 2}};
		struct msg z[] = {{ZW, &reg07, 1}, {ZW, data2, 2}, {ZRS | ZR | ZS, rx, 2}};
		const uint32_t want[] = {0x07 | RS, 0x5a, 0x00, RD | RS, RD | ST};

		same(n, 3, z, 3, want, 5);
	}
}

static void test_msg_flags(void)
{
	/* Single message: STOP, no RESTART. */
	assert(nano_i2c_dw_msg_flags(0, 0, 0, true, true) == (I2C_MSG_WRITE | I2C_MSG_STOP));
	assert(nano_i2c_dw_msg_flags(0, NR, 0, true, true) == (I2C_MSG_READ | I2C_MSG_STOP));
	/* NOSTART on the first message is ignored. */
	assert(nano_i2c_dw_msg_flags(0, NNST, 0, true, true) == I2C_MSG_STOP);
	/* NOSTOP / next NOSTART suppress STOP before the last message. */
	assert(nano_i2c_dw_msg_flags(0, NNS, NR, true, false) == I2C_MSG_WRITE);
	assert(nano_i2c_dw_msg_flags(0, 0, NNST, true, false) == I2C_MSG_WRITE);
	assert(nano_i2c_dw_msg_flags(0, 0, 0, true, false) == I2C_MSG_STOP);
	/* RESTART only after a message without STOP and without NOSTART. */
	assert(nano_i2c_dw_msg_flags(NNS, NR, 0, false, true) ==
	       (I2C_MSG_READ | I2C_MSG_STOP | I2C_MSG_RESTART));
	assert(nano_i2c_dw_msg_flags(NNS, NNST, 0, false, true) == I2C_MSG_STOP);
	assert(nano_i2c_dw_msg_flags(0, NR, 0, false, true) == (I2C_MSG_READ | I2C_MSG_STOP));
	/* Direction change adds RESTART; same direction keeps the flags. */
	assert(nano_i2c_dw_next_flags(I2C_MSG_WRITE, I2C_MSG_READ) ==
	       (I2C_MSG_READ | I2C_MSG_RESTART));
	assert(nano_i2c_dw_next_flags(I2C_MSG_READ | I2C_MSG_STOP, I2C_MSG_READ) == I2C_MSG_READ);
}

/* The register map copy keeps Zephyr's layout. */
static void test_registers(void)
{
	union ic_con_register con = {.raw = 0};

	con.bits.master_mode = 1;
	con.bits.speed = I2C_DW_SPEED_STANDARD;
	con.bits.restart_en = 1;
	con.bits.slave_disable = 1;
	con.bits.tx_empty_ctl = 1;
	con.bits.bus_clear = 1;
	/* With RX_FIFO_FULL_HLD_CTRL (bit 9): 0x0b63 written; RP2040 has no
	 * bit 11 and reads 0x0363 back.
	 */
	assert((con.raw | BIT(9)) == 0x0b63);
	con.bits.speed = I2C_DW_SPEED_FAST;
	assert((con.raw | BIT(9)) == 0x0b65);
	assert(DW_IC_REG_DATA_CMD == 0x10 && DW_IC_REG_SS_SCL_HCNT == 0x14 &&
	       DW_IC_REG_FS_SCL_LCNT == 0x20 && DW_IC_REG_SDAHOLD == 0x7c &&
	       DW_IC_REG_FS_SPKLEN == 0xa0 && DW_IC_REG_COMP_TYPE == 0xfc);
	assert(DW_ENABLE_TX_INT_I2C_CONTROLLER == 0x258 && DW_ENABLE_RX_INT_I2C_CONTROLLER == 0x207);
}

int main(void)
{
	test_scl_counts();
	test_speed();
	test_msg_flags();
	test_words();
	test_registers();
	printf("nano_i2c_dw: ok\n");
	return 0;
}
