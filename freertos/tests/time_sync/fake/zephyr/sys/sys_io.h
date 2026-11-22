/* SPDX-License-Identifier: MIT */
/* Stand-in for the compatibility layer's <zephyr/sys/sys_io.h>
 * (common/compat/zephyr/sys/sys_io.h) in the host test of freertos/tests/
 * time_sync: sys_read32() of the RP2040 TIMER's raw registers, TIMERAWH
 * (0x40054024) and TIMERAWL (0x40054028), returns the high and low words of
 * the test port's rp2040_timer_us() (CLOCK_MONOTONIC, freertos/tests/compat/
 * zephyr_os_port.h), so that time_sync_mono_us() and the layer's clocks are
 * one clock, as on the board (the TIMER).  Each read is appended to
 * test_timer_reads ('H' or 'L'); any other address fails the test.
 */

#ifndef FREERTOS_TESTS_TIME_SYNC_ZEPHYR_SYS_SYS_IO_H
#define FREERTOS_TESTS_TIME_SYNC_ZEPHYR_SYS_SYS_IO_H

#include <stdint.h>
#include <stdlib.h>

#include "zephyr_os_port.h"

typedef uintptr_t mem_addr_t;

void test_timer_read(char reg);

static inline uint32_t sys_read32(mem_addr_t addr)
{
	uint64_t now = rp2040_timer_us();

	if (addr == 0x40054024) {
		test_timer_read('H');
		return (uint32_t)(now >> 32);
	}
	if (addr == 0x40054028) {
		test_timer_read('L');
		return (uint32_t)now;
	}
	abort();
}

#endif /* FREERTOS_TESTS_TIME_SYNC_ZEPHYR_SYS_SYS_IO_H */
