/* SPDX-License-Identifier: Apache-2.0 */
/* Stand-in for the board's <arch/board/nano_timer.h> (the host test
 * time_sync): nano_mono_us() reads the test's clock, which every read
 * moves on by 1 us so that the test can tell the reads apart.
 */
#ifndef TEST_FAKE_NANO_TIMER_H
#define TEST_FAKE_NANO_TIMER_H
#include <stdint.h>
extern int64_t test_now_us;
static inline uint64_t nano_mono_us(void)
{
	return (uint64_t)test_now_us++;
}
#endif
