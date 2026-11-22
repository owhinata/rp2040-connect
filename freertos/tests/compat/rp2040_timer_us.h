/* SPDX-License-Identifier: MIT */
/* The RP2040 TIMER of the test port: the shared
 * common/chip/rp2040/include/rp2040_timer_us.h (read by its path, so that
 * no include path can find it before this file), whose register read
 * RP2040_TIMER_READ() returns here the high and low words of the host's
 * CLOCK_MONOTONIC in microseconds, so that
 * rp2040_timer_us(), the layer's clocks (z_os_clock_us()) and
 * time_sync_mono_us() are one clock, as on the board.  A test that defines
 * test_timer_read() (freertos/tests/time_sync) sees each read, 'H' for
 * TIMERAWH and 'L' for TIMERAWL; any other register fails the test.
 */

#ifndef FREERTOS_TESTS_COMPAT_RP2040_TIMER_US_H
#define FREERTOS_TESTS_COMPAT_RP2040_TIMER_US_H

#include <stdint.h>
#include <stdlib.h>
#include <time.h>

__attribute__((weak)) void test_timer_read(char reg);

static inline uint32_t test_host_timer_read(uint32_t offset)
{
	struct timespec t;
	uint64_t now;

	clock_gettime(CLOCK_MONOTONIC, &t);
	now = (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000u;
	if (offset == 0x24) {
		if (test_timer_read != NULL) {
			test_timer_read('H');
		}
		return (uint32_t)(now >> 32);
	}
	if (offset == 0x28) {
		if (test_timer_read != NULL) {
			test_timer_read('L');
		}
		return (uint32_t)now;
	}
	abort();
}

#define RP2040_TIMER_READ(offset) test_host_timer_read(offset)
#include "../../../common/chip/rp2040/include/rp2040_timer_us.h"

#endif /* FREERTOS_TESTS_COMPAT_RP2040_TIMER_US_H */
