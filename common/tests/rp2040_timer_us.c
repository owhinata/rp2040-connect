/* SPDX-License-Identifier: MIT */
/* rp2040_timer_us() of common/chip/rp2040/include/rp2040_timer_us.h on
 * scripted registers: the reads in order (TIMERAWH, TIMERAWL, TIMERAWH, and
 * again while the two high words differ) and the value returned when the low
 * word carries into the high word before, between or after the reads.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* One read: the register expected and the value it returns. */
struct reg_read {
	uint32_t offset;
	uint32_t value;
};

static const struct reg_read *script;
static unsigned int script_len, script_pos;

static uint32_t script_read(uint32_t offset)
{
	assert(script_pos < script_len);
	assert(script[script_pos].offset == offset);
	return script[script_pos++].value;
}

#define RP2040_TIMER_READ(offset) script_read(offset)
#include "rp2040_timer_us.h"

#define H RP2040_TIMER_TIMERAWH_OFFSET
#define L RP2040_TIMER_TIMERAWL_OFFSET

static uint64_t run(const struct reg_read *reads, unsigned int n)
{
	uint64_t us;

	script = reads;
	script_len = n;
	script_pos = 0;
	us = rp2040_timer_us();
	assert(script_pos == n); /* every scripted read made, no more */
	return us;
}

#define RUN(...)                                                                                   \
	run((const struct reg_read[]){__VA_ARGS__},                                                \
	    sizeof((const struct reg_read[]){__VA_ARGS__}) / sizeof(struct reg_read))

int main(void)
{
	assert(RP2040_TIMER_BASE == 0x40054000 && H == 0x24 && L == 0x28);

	/* No carry: three reads. */
	assert(RUN({H, 5}, {L, 100}, {H, 5}) == ((uint64_t)5 << 32 | 100));

	/* The low word carries after it was read: the pair (5, 0xffffffff) is
	 * not mixed with the new high word; read again.
	 */
	assert(RUN({H, 5}, {L, 0xffffffffu}, {H, 6}, {L, 2}, {H, 6}) == ((uint64_t)6 << 32 | 2));

	/* It carried between the first high word and the low word: (5, 1)
	 * would be 2^32 us (71 min) early; read again.
	 */
	assert(RUN({H, 5}, {L, 1}, {H, 6}, {L, 3}, {H, 6}) == ((uint64_t)6 << 32 | 3));

	/* Two carries in a row (the reads interrupted for long). */
	assert(RUN({H, 5}, {L, 9}, {H, 6}, {L, 8}, {H, 7}, {L, 4}, {H, 7}) ==
	       ((uint64_t)7 << 32 | 4));

	/* The full range of both words. */
	assert(RUN({H, 0xffffffffu}, {L, 0xffffffffu}, {H, 0xffffffffu}) == UINT64_MAX);
	assert(RUN({H, 0}, {L, 0}, {H, 0}) == 0);

	puts("rp2040_timer_us: ok");
	return 0;
}
