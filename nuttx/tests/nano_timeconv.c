/* SPDX-License-Identifier: Apache-2.0 */
/* boards/arduino-nano-rp2040-connect/src/nano_timeconv.h against plain
 * division and against the generic ONESHOT_COUNT conversions of NuttX
 * 13.1.0 (include/nuttx/timers/oneshot.h and clkcnt.h, written out below for
 * a 1 MHz counter and 100 us ticks).
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "nano_timeconv.h"

#define FREQ          1000000u   /* lower->frequency */
#define TICK_PER_SEC  10000u
#define NSEC_PER_TICK 100000u
#define NSEC_PER_SEC  1000000000u
#define CNT2NSEC_MULT 524288000u /* clkcnt_best_multshift(10^6, 10^9) */
#define CNT2NSEC_SH   19u

/* oneshot_tick_current() */
static uint64_t generic_tick(uint64_t cnt)
{
	uint64_t sec = cnt / FREQ;
	uint32_t nsec;

	cnt -= sec * FREQ;
	nsec = (uint32_t)(cnt * CNT2NSEC_MULT >> CNT2NSEC_SH);
	return sec * TICK_PER_SEC + nsec / NSEC_PER_TICK;
}

/* oneshot_current() */
static void generic_time(uint64_t cnt, uint64_t *sec, uint32_t *nsec)
{
	*sec = cnt / FREQ;
	cnt -= *sec * FREQ;
	*nsec = (uint32_t)(cnt * CNT2NSEC_MULT >> CNT2NSEC_SH);
}

/* clkcnt_tick2cnt() with a run-time frequency (clkcnt_time2cnt()). */
static uint64_t generic_tick2cnt(uint64_t tick)
{
	uint64_t sec = tick / TICK_PER_SEC;

	tick -= sec * TICK_PER_SEC;
	return sec * FREQ + (tick * FREQ + (TICK_PER_SEC - 1)) / TICK_PER_SEC;
}

static void check(uint64_t x)
{
	uint32_t rem, nsec, gnsec;
	uint64_t q, sec, gsec;

	q = nano_div100(x, &rem);
	assert(q == x / 100 && rem == x % 100);
	assert(nano_count_to_tick(x) == x / 100);
	assert(nano_count_to_tick(x) == generic_tick(x));

	nano_count_to_time(x, &sec, &nsec);
	assert(sec == x / FREQ && nsec == (uint32_t)(x % FREQ) * 1000u);
	generic_time(x, &gsec, &gnsec);
	assert(sec == gsec && nsec == gnsec && nsec < NSEC_PER_SEC);

	assert(nano_tick_to_count(x) == x * 100u);
	assert(nano_tick_to_count(x) == generic_tick2cnt(x));
	/* A tick starts at its count and the count maps back to the tick. */
	if (x <= (UINT64_MAX - 99) / 100) {
		assert(nano_count_to_tick(nano_tick_to_count(x)) == x);
		assert(nano_count_to_tick(nano_tick_to_count(x) + 99) == x);
	}
}

static uint64_t rng = 0x9e3779b97f4a7c15u;

static uint64_t next(void)
{
	rng ^= rng << 13;
	rng ^= rng >> 7;
	rng ^= rng << 17;
	return rng;
}

int main(void)
{
	static const uint64_t edges[] = {
		0, 1, 99, 100, 101, 199, 200, 255, 256, 25599, 25600, 43689, 43690,
		999999, 1000000, 1000001,
		UINT32_MAX - 100, UINT32_MAX - 1, UINT32_MAX, (uint64_t)UINT32_MAX + 1,
		(uint64_t)UINT32_MAX + 2, (uint64_t)UINT32_MAX + 100,
		(uint64_t)UINT32_MAX * 100 - 1, (uint64_t)UINT32_MAX * 100,
		(uint64_t)UINT32_MAX * 100 + 1, ((uint64_t)1 << 32) * 100 - 1,
		((uint64_t)1 << 32) * 100, ((uint64_t)1 << 32) * 100 + 1,
		((uint64_t)1 << 32) * 99 + 0xffffffffu,
		UINT64_MAX / 100 - 1, UINT64_MAX / 100, UINT64_MAX / 100 + 1,
		(uint64_t)1 << 63, ((uint64_t)1 << 63) - 1, ((uint64_t)1 << 63) + 1,
		UINT64_MAX - 100, UINT64_MAX - 1, UINT64_MAX,
	};
	uint64_t n, i;

	/* The one-multiply division is exact over the whole range it is
	 * claimed for, and the steps stay inside it. */
	for (n = 0; n < 43699; n++) {
		assert(((uint32_t)n * (uint64_t)NANO_DIV100_MUL >> NANO_DIV100_SHIFT) == n / 100);
	}
	assert((43699u * (uint64_t)NANO_DIV100_MUL >> NANO_DIV100_SHIFT) != 43699 / 100);
	assert(25599u * (uint64_t)NANO_DIV100_MUL <= UINT32_MAX);

	for (i = 0; i < sizeof(edges) / sizeof(edges[0]); i++) {
		for (n = 0; n < 300; n++) {
			check(edges[i] + n);
			check(edges[i] - n);
		}
	}

	/* Every count of the first 10^7 us and around every 32-bit wrap of
	 * the first 2^8 wraps. */
	for (n = 0; n < 10000000; n++) {
		check(n);
	}
	for (i = 1; i <= 256; i++) {
		for (n = 0; n < 2000; n++) {
			check((i << 32) + n);
			check((i << 32) - n);
		}
	}

	/* Random values, uniform and with a random number of significant bits. */
	for (i = 0; i < 2000000; i++) {
		uint64_t r = next();

		check(r);
		check(r >> (next() % 64));
	}

	printf("nano_timeconv: ok\n");
	return 0;
}
