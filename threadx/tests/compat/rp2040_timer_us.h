/* SPDX-License-Identifier: MIT */
/* The RP2040 TIMER of the test port: the shared
 * common/chip/rp2040/include/rp2040_timer_us.h (read by its path, so that
 * no include path can find it before this file), whose register read
 * RP2040_TIMER_READ() returns here the high and low words of the host's
 * CLOCK_MONOTONIC in microseconds, so that rp2040_timer_us() and the
 * layer's clocks (z_os_clock_us()) are one clock, as on the board.
 */

#ifndef THREADX_TESTS_COMPAT_RP2040_TIMER_US_H
#define THREADX_TESTS_COMPAT_RP2040_TIMER_US_H

#include <stdint.h>
#include <stdlib.h>
#include <time.h>

static inline uint32_t test_host_timer_read(uint32_t offset)
{
	struct timespec t;
	uint64_t now;

	clock_gettime(CLOCK_MONOTONIC, &t);
	now = (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000u;
	if (offset == 0x24) {
		return (uint32_t)(now >> 32);
	}
	if (offset == 0x28) {
		return (uint32_t)now;
	}
	abort();
}

#define RP2040_TIMER_READ(offset) test_host_timer_read(offset)
#include "../../../common/chip/rp2040/include/rp2040_timer_us.h"

#endif /* THREADX_TESTS_COMPAT_RP2040_TIMER_US_H */
