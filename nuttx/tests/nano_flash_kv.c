/* SPDX-License-Identifier: Apache-2.0 */
/* boards/arduino-nano-rp2040-connect/src/nano_flash_kv.h: the range checks
 * done before every flash erase and program, the split of writes into
 * pages, and the check of the area's contents before NuttX's NVS mounts
 * it (NuttX ATEs accepted, the Zephyr build's NVS rejected).
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "nano_flash_kv.h"

/* include/board.h */
#define KV_LO 0x00ffe000u
#define KV_HI 0x01000000u

/* NuttX's crc8part() (libs/libc/misc/lib_crc8.c, lib_crc8table.c): the
 * table of the reflected polynomial 0xB2, input and output inverted.
 */
static uint8_t crc_tab[256];

static void crc_init(void)
{
	for (int y = 0; y < 256; y++) {
		uint8_t crc = (uint8_t)y;

		for (int x = 0; x < 8; x++) {
			crc = (crc & 1) ? 0xb2 ^ (crc >> 1) : crc >> 1;
		}
		crc_tab[y] = crc;
	}
	/* Spot checks against crc8_tab[] of lib_crc8.c. */
	assert(crc_tab[1] == 0x3e && crc_tab[8] == 0x95);
	assert(crc_tab[0x80] == 0xb2 && crc_tab[255] == 0x15);
}

static uint8_t crc8part(const uint8_t *src, size_t len, uint8_t crc)
{
	crc ^= 0xff;
	for (size_t i = 0; i < len; i++) {
		crc = crc_tab[crc ^ src[i]];
	}
	return crc ^ 0xff;
}

/* An ATE as mtd_config_nvs.c writes it with a 1-byte program unit. */
static void nuttx_ate(uint8_t s[16], uint32_t id, uint16_t offset,
		      uint16_t len, uint16_t key_len, uint8_t data_crc8)
{
	memset(s, 0xff, 16);
	s[0] = id;
	s[1] = id >> 8;
	s[2] = id >> 16;
	s[3] = id >> 24;
	s[4] = offset;
	s[5] = offset >> 8;
	s[6] = len;
	s[7] = len >> 8;
	s[8] = key_len;
	s[9] = key_len >> 8;
	s[10] = data_crc8;
	s[11] = crc8part(s, 11, 0xff);
}

static int plausible(const uint8_t s[16], int close)
{
	return nano_kv_ate_plausible(s, crc8part(s, NANO_KV_ATE_CRC_LEN, 0xff),
				     NANO_KV_SECTOR_SIZE, close);
}

/* A fake flash for nano_kv_write_pages(): programming clears bits. */
static uint8_t flash[KV_HI - KV_LO];
static int programs;

static int program(void *arg, uint32_t page_offset, const uint8_t *page)
{
	(void)arg;
	if (!nano_kv_range_ok(page_offset, NANO_KV_PAGE_SIZE, KV_LO, KV_HI,
			      NANO_KV_PAGE_SIZE)) {
		return -22;
	}
	for (uint32_t i = 0; i < NANO_KV_PAGE_SIZE; i++) {
		flash[page_offset - KV_LO + i] &= page[i];
	}
	programs++;
	return 0;
}

static void test_ranges(void)
{
	/* Erase: one or both sectors, nothing else. */
	assert(nano_kv_range_ok(KV_LO, 4096, KV_LO, KV_HI, 4096));
	assert(nano_kv_range_ok(KV_LO + 4096, 4096, KV_LO, KV_HI, 4096));
	assert(nano_kv_range_ok(KV_LO, 8192, KV_LO, KV_HI, 4096));
	assert(!nano_kv_range_ok(KV_LO - 4096, 4096, KV_LO, KV_HI, 4096));
	assert(!nano_kv_range_ok(KV_LO - 4096, 8192, KV_LO, KV_HI, 4096));
	assert(!nano_kv_range_ok(KV_HI, 4096, KV_LO, KV_HI, 4096));
	assert(!nano_kv_range_ok(KV_LO + 4096, 8192, KV_LO, KV_HI, 4096));
	assert(!nano_kv_range_ok(KV_LO, 12288, KV_LO, KV_HI, 4096));
	assert(!nano_kv_range_ok(KV_LO + 256, 4096, KV_LO, KV_HI, 4096));
	assert(!nano_kv_range_ok(KV_LO, 0, KV_LO, KV_HI, 4096));
	assert(!nano_kv_range_ok(0, 4096, KV_LO, KV_HI, 4096));
	/* No wrap-around: a huge length or an offset near 2^32. */
	assert(!nano_kv_range_ok(KV_LO, 0xfffff000u, KV_LO, KV_HI, 4096));
	assert(!nano_kv_range_ok(0xfffff000u, 0x2000, KV_LO, KV_HI, 4096));
	/* Pages. */
	assert(nano_kv_range_ok(KV_LO, 256, KV_LO, KV_HI, 256));
	assert(nano_kv_range_ok(KV_HI - 256, 256, KV_LO, KV_HI, 256));
	assert(!nano_kv_range_ok(KV_HI - 128, 256, KV_LO, KV_HI, 256));
	assert(!nano_kv_range_ok(KV_LO - 256, 256, KV_LO, KV_HI, 256));
	assert(!nano_kv_range_ok(KV_LO + 1, 256, KV_LO, KV_HI, 256));
	/* Bytes. */
	assert(nano_kv_range_ok(KV_HI - 1, 1, KV_LO, KV_HI, 1));
	assert(!nano_kv_range_ok(KV_HI - 1, 2, KV_LO, KV_HI, 1));
	assert(!nano_kv_range_ok(KV_LO - 1, 1, KV_LO, KV_HI, 1));
	assert(!nano_kv_range_ok(KV_LO - 1, 2, KV_LO, KV_HI, 1));
	/* Bad parameters: no area, alignment 0. */
	assert(!nano_kv_range_ok(KV_LO, 256, 0, 0, 256));
	assert(!nano_kv_range_ok(KV_LO, 256, KV_HI, KV_LO, 256));
	assert(!nano_kv_range_ok(KV_LO, 256, KV_LO, KV_HI, 0));

	/* Every page and sector of the first and last MiB of the flash:
	 * accepted exactly when inside the area.
	 */
	for (uint32_t a = 0; a < 0x01000000u; a += 256) {
		int inside;

		if (a == 0x00100000u) {
			a = 0x00f00000u;
		}
		inside = a >= KV_LO && a < KV_HI;
		assert(nano_kv_range_ok(a, 256, KV_LO, KV_HI, 256) == inside);
		if (a % 4096 == 0) {
			assert(nano_kv_range_ok(a, 4096, KV_LO, KV_HI, 4096) ==
			       inside);
		}
	}
}

static void test_pages(void)
{
	uint8_t page[NANO_KV_PAGE_SIZE];
	uint8_t data[300];
	uint8_t expect[sizeof(flash)];

	assert(nano_kv_page_chunk(0, 10) == 10);
	assert(nano_kv_page_chunk(250, 10) == 6);
	assert(nano_kv_page_chunk(256, 300) == 256);
	assert(nano_kv_page_chunk(255, 1) == 1);
	assert(nano_kv_page_chunk(511, 300) == 1);

	for (size_t i = 0; i < sizeof(data); i++) {
		data[i] = (uint8_t)(i * 7 + 1);
	}

	/* Writes of every length at offsets around page boundaries change
	 * exactly the bytes written, in as many pages as they touch.
	 */
	const uint32_t offsets[] = { 0, 1, 16, 240, 255, 256, 4095, 4096,
				     8192 - 300, 8192 - 1 };

	for (size_t o = 0; o < sizeof(offsets) / sizeof(offsets[0]); o++) {
		for (uint32_t len = 1; len <= 300; len++) {
			uint32_t off = offsets[o];

			if (off + len > sizeof(flash)) {
				continue;
			}
			memset(flash, 0xff, sizeof(flash));
			memset(expect, 0xff, sizeof(expect));
			memcpy(expect + off, data, len);
			programs = 0;
			assert(nano_kv_write_pages(KV_LO + off, data, len, page,
						   program, NULL) == 0);
			assert(memcmp(flash, expect, sizeof(flash)) == 0);
			assert(programs == (int)((off + len - 1) / 256 -
						 off / 256 + 1));
		}
	}

	/* A second write into a programmed page leaves the bytes around it
	 * (0xff in the page) as they are: the NVS "expired" byte.
	 */
	memset(flash, 0xff, sizeof(flash));
	assert(nano_kv_write_pages(KV_LO + 32, data, 16, page, program,
				   NULL) == 0);
	assert(nano_kv_write_pages(KV_LO + 44, (const uint8_t *)"\0", 1, page,
				   program, NULL) == 0);
	assert(memcmp(flash + 32, data, 12) == 0 && flash[44] == 0 &&
	       memcmp(flash + 45, data + 13, 3) == 0);

	/* A write running past the area stops with the program() error
	 * after the pages inside.
	 */
	memset(flash, 0xff, sizeof(flash));
	programs = 0;
	assert(nano_kv_write_pages(KV_HI - 10, data, 20, page, program,
				   NULL) == -22);
	assert(programs == 1);
}

static void test_layout(void)
{
	uint8_t s[16];
	uint8_t z[16];

	/* NuttX: a data entry (key 16 bytes with NUL, value 5 bytes). */
	nuttx_ate(s, 0x12345678, 0x0100, 5, 16, 0x5a);
	assert(plausible(s, 0) && !plausible(s, 1));
	/* ... replaced or deleted later (expired byte cleared). */
	s[12] = 0x00;
	assert(plausible(s, 0));
	/* ... or half cleared by a power loss. */
	s[12] = 0x7c;
	assert(plausible(s, 0));
	/* A gc-done ATE as the first entry of a block. */
	nuttx_ate(s, 0xffffffff, 0x0000, 0, 0, 0xff);
	assert(plausible(s, 0));
	/* A close ATE pointing at the last entry. */
	nuttx_ate(s, 0xffffffff, 4096 - 16 * 10, 0, 0, 0xff);
	assert(plausible(s, 1) && plausible(s, 0));
	nuttx_ate(s, 0xffffffff, 4096 - 16 * 10 + 4, 0, 0, 0xff);
	assert(!plausible(s, 1));
	/* Damaged: CRC, offset beyond the block, data entry without a key,
	 * the padding after the expired byte.
	 */
	nuttx_ate(s, 0x12345678, 0x0100, 5, 16, 0x5a);
	s[11] ^= 1;
	assert(!plausible(s, 0));
	nuttx_ate(s, 0x12345678, 4080, 5, 16, 0x5a);
	assert(!plausible(s, 0));
	nuttx_ate(s, 0x12345678, 0x0100, 5, 0, 0x5a);
	assert(!plausible(s, 0));
	nuttx_ate(s, 0x12345678, 0x0100, 5, 16, 0x5a);
	s[14] = 0;
	assert(!plausible(s, 0));
	/* All erased. */
	memset(s, 0xff, sizeof(s));
	assert(!plausible(s, 0) && !plausible(s, 1));

	/* The Zephyr build's NVS: 8-byte ATEs (id u16, offset u16, len u16,
	 * part 0xff, crc8), two to a 16-byte slot.  Even when the bytes
	 * happen to pass NuttX's CRC, the second ATE's len and part sit where
	 * NuttX keeps erased padding.
	 */
	int crc_hits = 0;

	for (uint32_t v = 0; v < 65536; v++) {
		memset(z, 0xff, sizeof(z));
		/* ATE: id 0x4001 (a value), offset 0x0123, len 9 */
		z[0] = 0x01; z[1] = 0x40; z[2] = 0x23; z[3] = 0x01;
		z[4] = 9; z[5] = 0; z[6] = 0xff; z[7] = 0x37;
		/* ATE: id 0x0001 (its name), offset v, len 18 */
		z[8] = 0x01; z[9] = 0x00; z[10] = v; z[11] = v >> 8;
		z[12] = 18; z[13] = 0; z[14] = 0xff; z[15] = 0x91;
		if (crc8part(z, 11, 0xff) == z[11]) {
			crc_hits++;
		}
		assert(!plausible(z, 0) && !plausible(z, 1));
	}
	assert(crc_hits > 0);
	/* Zephyr's gc-done ATE (id 0xffff, offset 0) with a close ATE after
	 * it, and the same with the close slot still erased.
	 */
	memset(z, 0xff, sizeof(z));
	z[0] = 0xff; z[1] = 0xff; z[2] = 0; z[3] = 0;
	z[4] = 0; z[5] = 0; z[6] = 0xff; z[7] = 0x12;
	assert(!plausible(z, 0) && !plausible(z, 1));
	z[8] = 0xff; z[9] = 0xff; z[10] = 0x50; z[11] = 0x0d;
	z[12] = 0; z[13] = 0; z[14] = 0xff; z[15] = 0x34;
	assert(!plausible(z, 0) && !plausible(z, 1));

	assert(nano_kv_layout(1, false) == NANO_KV_LAYOUT_NUTTX);
	assert(nano_kv_layout(2, false) == NANO_KV_LAYOUT_NUTTX);
	assert(nano_kv_layout(1, true) == NANO_KV_LAYOUT_NUTTX);
	assert(nano_kv_layout(0, true) == NANO_KV_LAYOUT_EMPTY);
	assert(nano_kv_layout(0, false) == NANO_KV_LAYOUT_FOREIGN);
}

int main(void)
{
	crc_init();
	test_ranges();
	test_pages();
	test_layout();
	puts("nano_flash_kv ok");
	return 0;
}
