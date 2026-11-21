/* SPDX-License-Identifier: MIT */
/* Host test of the channel allocation of NuttX's RP2040 DMA driver,
 * arch/arm/src/rp2040/rp2040_dmac.c with the local patches
 * nuttx/patches/nuttx/0011 and 0012 (the file itself, included below), on a
 * model of the DMA's registers and fakes of the kernel's lock and count
 * (tests/rp2040_dmac_fakes):
 *
 * - rp2040_dmachannel(), rp2040_dmachannel_trywait() and rp2040_dmafree()
 *   clear the flag of their own channel only: another channel's pending
 *   completion interrupt stays.  DMA_INTS0 clears the bits written as 1, so
 *   a write through its set or clear alias, which the model takes as the
 *   hardware does, clears the other channels' flags too.
 * - rp2040_dmachannel_trywait() never waits: NULL when no channel is free,
 *   and NULL with the count given back and no register written when
 *   another task holds the allocator's lock; it gets the channel afterwards.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

bool fake_mutex_held;

#include "../nuttx/arch/arm/src/rp2040/rp2040_dmac.c"

/* The registers: INTR (the raw flags), INTE0, and the rest as plain words.
 * INTS0 reads INTR & INTE0 and clears the INTR bits written as 1.
 */

#define NREGS 0x1000
static uint32_t regs[NREGS / 4];
static unsigned int writes;

#define REG(addr) regs[((addr) - RP2040_DMA_BASE) / 4]

uint32_t getreg32(uintptr_t addr)
{
	if (addr == RP2040_DMA_INTS0) {
		return REG(RP2040_DMA_INTR) & REG(RP2040_DMA_INTE0);
	}
	return REG(addr);
}

void putreg32(uint32_t value, uintptr_t addr)
{
	writes++;
	if (addr == RP2040_DMA_INTS0) {
		REG(RP2040_DMA_INTR) &= ~value;
	} else if (addr == RP2040_DMA_CHAN_ABORT) {
		/* The aborts end at once here. */
	} else {
		REG(addr) = value;
	}
}

/* The aliases write the register's value with the bits set or cleared. */

void setbits_reg32(uint32_t value, uintptr_t addr)
{
	putreg32(getreg32(addr) | value, addr);
}

void clrbits_reg32(uint32_t value, uintptr_t addr)
{
	putreg32(getreg32(addr) & ~value, addr);
}

void modbits_reg32(uint32_t value, uint32_t mask, uintptr_t addr)
{
	putreg32((getreg32(addr) & ~mask) | (value & mask), addr);
}

int irq_attach(int irq, xcpt_t isr, void *arg)
{
	(void)irq;
	(void)isr;
	(void)arg;
	return 0;
}

void up_enable_irq(int irq)
{
	(void)irq;
}

/* Another driver's channel OTHER with its completion interrupt pending. */
#define OTHER 4

static void other_pending(void)
{
	REG(RP2040_DMA_INTE0) |= 1u << OTHER;
	REG(RP2040_DMA_INTR) |= 1u << OTHER;
}

static bool other_still_pending(void)
{
	return getreg32(RP2040_DMA_INTS0) & (1u << OTHER);
}

static void test_flags(void)
{
	DMA_HANDLE h;

	/* The channel's own flag from an earlier use is cleared, the other's
	 * stays, with each of the three functions.
	 */
	other_pending();
	REG(RP2040_DMA_INTR) |= 1u << 0;
	h = rp2040_dmachannel();
	assert(h == &g_dmach[0] && other_still_pending());
	assert(!(REG(RP2040_DMA_INTR) & 1u) && (REG(RP2040_DMA_INTE0) & 1u));

	REG(RP2040_DMA_INTR) |= 1u << 0;
	rp2040_dmafree(h);
	assert(other_still_pending());
	assert(!(REG(RP2040_DMA_INTR) & 1u) && !(REG(RP2040_DMA_INTE0) & 1u));

	REG(RP2040_DMA_INTR) |= 1u << 0;
	h = rp2040_dmachannel_trywait();
	assert(h == &g_dmach[0] && other_still_pending());
	assert(!(REG(RP2040_DMA_INTR) & 1u) && (REG(RP2040_DMA_INTE0) & 1u));
	rp2040_dmafree(h);
	assert(other_still_pending());
	assert(g_dmac.chansem.count == RP2040_DMA_NCHANNELS);
}

static void test_trywait(void)
{
	DMA_HANDLE h[RP2040_DMA_NCHANNELS];
	unsigned int before;

	/* All but one channel taken, the allocator's lock held by another
	 * task: no channel, the count back, nothing written.
	 */
	for (int i = 0; i < RP2040_DMA_NCHANNELS - 1; i++) {
		h[i] = rp2040_dmachannel();
		assert(h[i] == &g_dmach[i]);
	}
	assert(g_dmac.chansem.count == 1);
	fake_mutex_held = true;
	before = writes;
	assert(rp2040_dmachannel_trywait() == NULL);
	assert(g_dmac.chansem.count == 1 && writes == before);
	assert(!g_dmach[RP2040_DMA_NCHANNELS - 1].inuse);

	/* The lock free again: the last channel. */
	fake_mutex_held = false;
	h[RP2040_DMA_NCHANNELS - 1] = rp2040_dmachannel_trywait();
	assert(h[RP2040_DMA_NCHANNELS - 1] == &g_dmach[RP2040_DMA_NCHANNELS - 1]);
	assert(g_dmac.chansem.count == 0);

	/* None free: NULL at once, nothing written. */
	before = writes;
	assert(rp2040_dmachannel_trywait() == NULL);
	assert(g_dmac.chansem.count == 0 && writes == before);

	for (int i = 0; i < RP2040_DMA_NCHANNELS; i++) {
		rp2040_dmafree(h[i]);
	}
	assert(g_dmac.chansem.count == RP2040_DMA_NCHANNELS);
}

int main(void)
{
	arm_dma_initialize();
	test_flags();
	test_trywait();
	return 0;
}
