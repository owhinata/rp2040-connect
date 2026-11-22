/* SPDX-License-Identifier: MIT */
/* The RP2040 TIMER's registers for the host test time_sync: the shared
 * common/chip/rp2040/include/rp2040_timer_us.h (read by its path) with its
 * register read RP2040_TIMER_READ() on the test's clock.  Each
 * read of TIMERAWL moves it on by 1 us, so each time_sync_mono_us()
 * (TIMERAWH, TIMERAWL, TIMERAWH) is one read that the test can tell apart;
 * TIMERAWH is its high word.
 */
#ifndef TEST_FAKE_RP2040_TIMER_US_H
#define TEST_FAKE_RP2040_TIMER_US_H
#include <assert.h>
#include <stdint.h>

extern int64_t test_now_us;

static inline uint32_t test_timer_read(uint32_t offset)
{
	if (offset == 0x28) { /* TIMERAWL */
		return (uint32_t)test_now_us++;
	}
	assert(offset == 0x24); /* TIMERAWH */
	return (uint32_t)((uint64_t)test_now_us >> 32);
}

#define RP2040_TIMER_READ(offset) test_timer_read(offset)
#include "../../../common/chip/rp2040/include/rp2040_timer_us.h"
#endif
