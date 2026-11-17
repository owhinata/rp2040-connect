/* SPDX-License-Identifier: MIT */
/* Stand-in for the compatibility layer's <zephyr/sys/sys_io.h> (the host
 * test time_sync): sys_read32() of the RP2040 TIMER's raw registers reads
 * the test's clock.  Each read of TIMERAWL moves it on by 1 us, so each
 * time_sync_mono_us() (TIMERAWH, TIMERAWL, TIMERAWH) is one read that the
 * test can tell apart; TIMERAWH is its high word.
 */
#ifndef TEST_FAKE_ZEPHYR_SYS_SYS_IO_H
#define TEST_FAKE_ZEPHYR_SYS_SYS_IO_H
#include <assert.h>
#include <stdint.h>

typedef uintptr_t mem_addr_t;

extern int64_t test_now_us;

static inline uint32_t sys_read32(mem_addr_t addr)
{
	if (addr == 0x40054028) { /* TIMERAWL */
		return (uint32_t)test_now_us++;
	}
	assert(addr == 0x40054024); /* TIMERAWH */
	return (uint32_t)((uint64_t)test_now_us >> 32);
}
#endif
