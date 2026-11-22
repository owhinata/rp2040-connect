/* SPDX-License-Identifier: MIT */
/* inet_pton() and inet_ntop() of <arpa/inet.h> (common/compat/libc_net/
 * include), for AF_INET only, which newlib does not have.  The accepted
 * text is POSIX's for inet_pton(AF_INET): "ddd.ddd.ddd.ddd", each part a
 * decimal number 0..255 of one to three digits (POSIX leaves leading zeros
 * open; accepted here, read as decimal, as NuttX and Zephyr's
 * zsock_inet_pton() read them).
 * Part of the POSIX network headers for a C library that has none
 * (common/compat/libc_net), which the FreeRTOS build uses with newlib.
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <arpa/inet.h>

int inet_pton(int af, const char *src, void *dst)
{
	uint8_t bytes[4];
	const char *p = src;

	if (af != AF_INET) {
		errno = EAFNOSUPPORT;
		return -1;
	}
	for (int i = 0; i < 4; i++) {
		unsigned int value = 0;
		int digits = 0;

		if (i > 0) {
			if (*p != '.') {
				return 0;
			}
			p++;
		}
		while (*p >= '0' && *p <= '9') {
			if (++digits > 3) {
				return 0;
			}
			value = value * 10u + (unsigned int)(*p - '0');
			p++;
		}
		if (digits == 0 || value > 255u) {
			return 0;
		}
		bytes[i] = (uint8_t)value;
	}
	if (*p != '\0') {
		return 0;
	}
	memcpy(dst, bytes, sizeof(bytes));
	return 1;
}

const char *inet_ntop(int af, const void *src, char *dst, socklen_t size)
{
	const uint8_t *b = src;
	char text[INET_ADDRSTRLEN];
	int n;

	if (af != AF_INET) {
		errno = EAFNOSUPPORT;
		return NULL;
	}
	n = snprintf(text, sizeof(text), "%u.%u.%u.%u", b[0], b[1], b[2], b[3]);
	if (n < 0 || (socklen_t)n >= size) {
		errno = ENOSPC;
		return NULL;
	}
	memcpy(dst, text, (size_t)n + 1);
	return dst;
}
