/* SPDX-License-Identifier: Apache-2.0 */
/* apps/kv/kv_hex.c: hex2bin() and bin2hex() of Zephyr's lib/utils/hex.c as
 * "kv sethex" and "kv get" use them.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "kv_hex.h"

int main(void)
{
	uint8_t buf[128];
	char hex[2 * 128 + 1];

	/* Upper and lower case. */
	assert(kv_hex2bin("00ff7fA5aB", 10, buf, sizeof(buf)) == 5);
	assert(buf[0] == 0x00 && buf[1] == 0xff && buf[2] == 0x7f &&
	       buf[3] == 0xa5 && buf[4] == 0xab);
	/* Odd length: leading zero nibble (cmd_sethex rejects it before). */
	assert(kv_hex2bin("abc", 3, buf, sizeof(buf)) == 2);
	assert(buf[0] == 0x0a && buf[1] == 0xbc);
	/* Any other character gives 0. */
	assert(kv_hex2bin("0g", 2, buf, sizeof(buf)) == 0);
	assert(kv_hex2bin("g0", 2, buf, sizeof(buf)) == 0);
	assert(kv_hex2bin("0 ", 2, buf, sizeof(buf)) == 0);
	assert(kv_hex2bin("0x12", 4, buf, sizeof(buf)) == 0);
	assert(kv_hex2bin("-1", 2, buf, sizeof(buf)) == 0);
	assert(kv_hex2bin("g", 1, buf, sizeof(buf)) == 0);
	/* Too small a buffer gives 0. */
	assert(kv_hex2bin("0102", 4, buf, 1) == 0);
	assert(kv_hex2bin("", 0, buf, sizeof(buf)) == 0);
	/* 128 bytes. */
	memset(hex, 'F', 256);
	assert(kv_hex2bin(hex, 256, buf, sizeof(buf)) == 128);
	assert(buf[0] == 0xff && buf[127] == 0xff);

	/* Lower case out, NUL terminated, 0 if it does not fit. */
	buf[0] = 0x31; buf[1] = 0x38; buf[2] = 0xab; buf[3] = 0x0F;
	assert(kv_bin2hex(buf, 4, hex, sizeof(hex)) == 8);
	assert(strcmp(hex, "3138ab0f") == 0);
	assert(kv_bin2hex(buf, 4, hex, 8) == 0);
	assert(kv_bin2hex(buf, 4, hex, 9) == 8);
	assert(kv_bin2hex(buf, 0, hex, 1) == 0 && hex[0] == '\0');
	memset(buf, 0, sizeof(buf));
	assert(kv_bin2hex(buf, 128, hex, sizeof(hex)) == 256);
	assert(strlen(hex) == 256);
	puts("kv_hex ok");
	return 0;
}
