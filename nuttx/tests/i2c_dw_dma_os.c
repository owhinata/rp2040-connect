/* SPDX-License-Identifier: MIT */
/* Host test of the DMA part of the I2C0 reads (i2c_dw_dma_os_rp2040.c of
 * the I2C core's NuttX binding, common/drivers/i2c_dw/port/nuttx, the
 * boundary of common/drivers/i2c_dw/port/i2c_dw_dma_os.h) on a fake of
 * NuttX's rp2040_dmac.c
 * (tests/i2c_dw_dma_os_fakes: its header, and here its functions and its
 * interrupt handler, written as NuttX 13.1.0 has them, on an array in place
 * of the DMA's registers; the calls and the writes of INTE0, INTS0 and
 * CHAN_ABORT are recorded).  The file is included, so that each case starts
 * from its initial state.
 *
 * - The start: two channels from rp2040_dmachannel_trywait() (the local
 *   patch nuttx/patches/nuttx/0011; the lowest free, after another user's),
 *   their interrupts off until a read starts them; once.  Exactly two free:
 *   both taken.  Exactly one free: it is taken and freed again, so another
 *   user gets it.  None free: nothing taken.  Without both: -EBUSY at
 *   once, not ready, no channel (-ENODEV), and the same result on a second
 *   call.  The allocator busy at the second try (the fake's
 *   rp2040_dmachannel_trywait() returns NULL; the patched function itself
 *   is not under test): the same, the first channel given back.
 * - The channels per read: the two, then -EBUSY without waiting; back after
 *   the release; others refused (-EINVAL).
 * - The setup: rp2040_txdmasetup() with DREQ 32, halfwords, 2 bytes per word
 *   and the read address incrementing; rp2040_rxdmasetup() with DREQ 33,
 *   bytes, the write address incrementing; the registers as NuttX writes
 *   them; refused while busy (-EBUSY).
 * - The start: the old flag cleared, the interrupt enabled, then
 *   rp2040_dmastart() (RX before TX, as the core calls them).
 * - The interrupt (NuttX's handler): the end of a channel gives 0; another
 *   user's channel goes to its own callback; a bus error gives -EIO, and the
 *   channel, which NuttX's handler starts again by writing its error flags
 *   back to CTRL_TRIG, is aborted at once; NuttX's status of an earlier
 *   channel of the same interrupt carries over to ours (-EIO).
 * - The stop: interrupt off and CHAN_ABORT, never rp2040_dmastop() (it waits
 *   without a bound); busy until BUSY and the abort are clear; no callback
 *   after it; the acknowledge clears the late flag.
 * - The release: interrupt off, no callback after it.
 */

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "../../common/drivers/i2c_dw/port/nuttx/i2c_dw_dma_os_rp2040.c"

int fake_irq_saved;
uint32_t fake_dma_regs[0x460 / 4];

#define REG(addr) fake_dma_regs[((addr) - RP2040_DMA_BASE) / 4]

/* In NuttX's interrupt handler (dmac_interrupt()): its own write of INTS0 is
 * not recorded.
 */
static int isr_depth;

/* The calls and the writes -------------------------------------------------*/

static char calls[2048];

static void record(const char *fmt, ...)
{
	size_t used = strlen(calls);
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(calls + used, sizeof(calls) - used, fmt, ap);
	va_end(ap);
	strcat(calls, " ");
}

static void expect_calls(const char *expected)
{
	if (strcmp(calls, expected) != 0) {
		printf("expected: %s\n     got: %s\n", expected, calls);
		fflush(stdout);
		assert(0);
	}
	calls[0] = '\0';
	assert(fake_irq_saved == 0);
}

/* The registers: a write of CTRL_TRIG clears the error flags written with 1
 * and, with EN, starts the channel (a trigger register); INTS0 is INTR and
 * INTE0, written with 1 to clear; a 1 written to CHAN_ABORT stays until the
 * test ends the abort.
 */

#define ERRORS (RP2040_DMA_CTRL_TRIG_READ_ERROR | RP2040_DMA_CTRL_TRIG_WRITE_ERROR)

static int ctrl_channel(uintptr_t addr)
{
	for (int ch = 0; ch < RP2040_DMA_NCHANNELS; ch++) {
		if (addr == RP2040_DMA_CTRL_TRIG(ch)) {
			return ch;
		}
	}
	return -1;
}

static void ctrl_write(int ch, uint32_t value)
{
	uint32_t old = REG(RP2040_DMA_CTRL_TRIG(ch));
	uint32_t errors = old & ERRORS & ~(value & ERRORS);
	uint32_t v = (value & ~(ERRORS | RP2040_DMA_CTRL_TRIG_AHB_ERROR | RP2040_DMA_CTRL_TRIG_BUSY)) |
		     errors | (old & RP2040_DMA_CTRL_TRIG_BUSY);

	if (errors) {
		v |= RP2040_DMA_CTRL_TRIG_AHB_ERROR;
	}
	if (v & RP2040_DMA_CTRL_TRIG_EN) {
		v |= RP2040_DMA_CTRL_TRIG_BUSY;
	}
	REG(RP2040_DMA_CTRL_TRIG(ch)) = v;
}

uint32_t getreg32(uintptr_t addr)
{
	if (addr == RP2040_DMA_INTS0) {
		return REG(RP2040_DMA_INTR) & REG(RP2040_DMA_INTE0);
	}
	return REG(addr);
}

void putreg32(uint32_t value, uintptr_t addr)
{
	int ch = ctrl_channel(addr);

	if (addr == RP2040_DMA_INTS0) {
		REG(RP2040_DMA_INTR) &= ~value;
		if (isr_depth == 0) {
			record("ints0(0x%x)", (unsigned int)value);
		}
	} else if (addr == RP2040_DMA_CHAN_ABORT) {
		REG(addr) |= value;
		record("abort(0x%x)", (unsigned int)value);
	} else if (ch >= 0) {
		ctrl_write(ch, value);
	} else {
		REG(addr) = value;
	}
}

void setbits_reg32(uint32_t value, uintptr_t addr)
{
	int ch = ctrl_channel(addr);

	if (addr == RP2040_DMA_INTS0) {
		/* The set alias writes INTS0 | value: every pending flag is cleared. */
		REG(RP2040_DMA_INTR) &= ~(getreg32(addr) | value);
	} else if (ch >= 0) {
		ctrl_write(ch, REG(addr) | value);
	} else {
		REG(addr) |= value;
		if (addr == RP2040_DMA_INTE0) {
			record("set(inte0,0x%x)", (unsigned int)value);
		}
	}
}

void clrbits_reg32(uint32_t value, uintptr_t addr)
{
	REG(addr) &= ~value;
	if (addr == RP2040_DMA_INTE0) {
		record("clr(inte0,0x%x)", (unsigned int)value);
	}
}

/* NuttX's rp2040_dmac.c ------------------------------------------------------*/

struct fake_channel {
	uint8_t chan;
	bool inuse;
	dma_callback_t callback;
	void *arg;
	uintptr_t read_addr; /* the addresses as given (the array keeps 32 bits) */
	uintptr_t write_addr;
};

static struct fake_channel fake_ch[RP2040_DMA_NCHANNELS];

/* The lowest free channel, its flag cleared and its interrupt enabled, or
 * NULL when none is free.
 */
static struct fake_channel *take_channel(const char *name)
{
	for (int ch = 0; ch < RP2040_DMA_NCHANNELS; ch++) {
		if (!fake_ch[ch].inuse) {
			fake_ch[ch].inuse = true;
			setbits_reg32(1u << ch, RP2040_DMA_INTS0);
			REG(RP2040_DMA_INTE0) |= 1u << ch;
			record("%s()=%d", name, ch);
			return &fake_ch[ch];
		}
	}
	record("%s()=NULL", name);
	return NULL;
}

/* Other users' channels (the binding never calls it). */
DMA_HANDLE rp2040_dmachannel(void)
{
	DMA_HANDLE handle = take_channel("dmachannel");

	assert(handle != NULL); /* NuttX waits here: the test never gets this far */
	return handle;
}

/* The number of calls of rp2040_dmachannel_trywait() after which it finds
 * the allocator's lock held by another task (-1: never).
 */
static int fake_busy_after = -1;

/* The local patch nuttx/patches/nuttx/0011: NULL at once when no channel is
 * free, or when the allocator's lock is held (the count it took given back,
 * so no channel is taken). The function's inside is not under test here.
 */
DMA_HANDLE rp2040_dmachannel_trywait(void)
{
	if (fake_busy_after == 0) {
		record("trywait()=busy");
		return NULL;
	}
	if (fake_busy_after > 0) {
		fake_busy_after--;
	}
	return take_channel("trywait");
}

void rp2040_dmafree(DMA_HANDLE handle)
{
	struct fake_channel *c = handle;

	assert(c->inuse);
	REG(RP2040_DMA_CHAN_ABORT) &= ~(1u << c->chan);
	REG(RP2040_DMA_CTRL_TRIG(c->chan)) = 0;
	REG(RP2040_DMA_INTE0) &= ~(1u << c->chan);
	c->inuse = false;
	record("dmafree(%d)", c->chan);
}

static void dmasetup(struct fake_channel *c, uintptr_t read, uintptr_t write, size_t nbytes,
		     dma_config_t config, uint32_t incr)
{
	uint32_t ch = c->chan;
	uint32_t ctrl;

	assert(c->inuse);
	assert(config.size <= RP2040_DMA_SIZE_WORD);
	assert((nbytes >> config.size) > 0);
	c->read_addr = read;
	c->write_addr = write;
	REG(RP2040_DMA_TRANS_COUNT(ch)) = (uint32_t)(nbytes >> config.size);
	ctrl = ERRORS |
	       (((uint32_t)config.dreq << RP2040_DMA_CTRL_TRIG_TREQ_SEL_SHIFT) &
		RP2040_DMA_CTRL_TRIG_TREQ_SEL_MASK) |
	       ((ch << RP2040_DMA_CTRL_TRIG_CHAIN_TO_SHIFT) & RP2040_DMA_CTRL_TRIG_CHAIN_TO_MASK) |
	       ((uint32_t)config.size << RP2040_DMA_CTRL_TRIG_DATA_SIZE_SHIFT);
	if (!config.noincr) {
		ctrl |= incr;
	}
	putreg32(ctrl, RP2040_DMA_CTRL_TRIG(ch));
}

void rp2040_rxdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr, size_t nbytes,
		       dma_config_t config)
{
	struct fake_channel *c = handle;

	dmasetup(c, paddr, maddr, nbytes, config, RP2040_DMA_CTRL_TRIG_INCR_WRITE);
	record("rxsetup(%d,dreq %u,size %u,noincr %u,%zu)", c->chan, config.dreq, config.size,
	       config.noincr, nbytes);
}

void rp2040_txdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr, size_t nbytes,
		       dma_config_t config)
{
	struct fake_channel *c = handle;

	dmasetup(c, maddr, paddr, nbytes, config, RP2040_DMA_CTRL_TRIG_INCR_READ);
	record("txsetup(%d,dreq %u,size %u,noincr %u,%zu)", c->chan, config.dreq, config.size,
	       config.noincr, nbytes);
}

void rp2040_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg)
{
	struct fake_channel *c = handle;

	assert(c->inuse);
	c->callback = callback;
	c->arg = arg;
	setbits_reg32(RP2040_DMA_CTRL_TRIG_EN, RP2040_DMA_CTRL_TRIG(c->chan));
	record("dmastart(%d)", c->chan);
}

void rp2040_dmastop(DMA_HANDLE handle)
{
	(void)handle;
	assert(0); /* waits on CHAN_ABORT without a bound: not to be used */
}

uintptr_t rp2040_dma_register(DMA_HANDLE handle, uint16_t offset)
{
	struct fake_channel *c = handle;

	assert(c->inuse);
	return RP2040_DMA_CH(c->chan) + offset;
}

/* rp2040_dmac_interrupt(), as NuttX 13.1.0 has it (DMA_IRQ_0). */
static void dmac_interrupt(void)
{
	int result = 0;
	uint32_t stat = getreg32(RP2040_DMA_INTS0) & RP2040_DMA_INTS0_MASK;

	isr_depth++;
	putreg32(stat, RP2040_DMA_INTS0);
	while (stat != 0) {
		int ch = ffs((int)stat) - 1;
		uint32_t ctrl = getreg32(RP2040_DMA_CTRL_TRIG(ch));

		stat &= ~(1u << ch);
		if (ctrl & RP2040_DMA_CTRL_TRIG_AHB_ERROR) {
			setbits_reg32(ERRORS, RP2040_DMA_CTRL_TRIG(ch));
			result = EIO;
		}
		if (fake_ch[ch].callback) {
			fake_ch[ch].callback(&fake_ch[ch], (uint8_t)result, fake_ch[ch].arg);
			fake_ch[ch].callback = NULL;
		}
		fake_ch[ch].arg = NULL;
	}
	isr_depth--;
}

/* The DMA's side: a channel ends, or stops on a bus error, and flags it. */

static void dma_end(int ch)
{
	REG(RP2040_DMA_CTRL_TRIG(ch)) &= ~RP2040_DMA_CTRL_TRIG_BUSY;
	REG(RP2040_DMA_INTR) |= 1u << ch;
}

static void dma_fail(int ch, uint32_t error)
{
	REG(RP2040_DMA_CTRL_TRIG(ch)) = (REG(RP2040_DMA_CTRL_TRIG(ch)) & ~RP2040_DMA_CTRL_TRIG_BUSY) |
					error | RP2040_DMA_CTRL_TRIG_AHB_ERROR;
	REG(RP2040_DMA_INTR) |= 1u << ch;
}

static void abort_ends(int ch)
{
	REG(RP2040_DMA_CTRL_TRIG(ch)) &= ~RP2040_DMA_CTRL_TRIG_BUSY;
	REG(RP2040_DMA_CHAN_ABORT) &= ~(1u << ch);
}

/* The core's callback, and another user's. */

static char cbs[512];

static void core_cb(void *user_data, uint32_t channel, int status)
{
	size_t used = strlen(cbs);

	assert(isr_depth > 0);
	snprintf(cbs + used, sizeof(cbs) - used, "cb(%s,%u,%d) ", (const char *)user_data,
		 (unsigned int)channel, status);
}

static void other_cb(DMA_HANDLE handle, uint8_t status, void *arg)
{
	size_t used = strlen(cbs);

	(void)arg;
	snprintf(cbs + used, sizeof(cbs) - used, "other(%d,%u) ",
		 ((struct fake_channel *)handle)->chan, status);
}

static void expect_cbs(const char *expected)
{
	if (strcmp(cbs, expected) != 0) {
		printf("expected callbacks: %s\n     got callbacks: %s\n", expected, cbs);
		fflush(stdout);
		assert(0);
	}
	cbs[0] = '\0';
}

static void reset(void)
{
	memset(fake_dma_regs, 0, sizeof(fake_dma_regs));
	memset(fake_ch, 0, sizeof(fake_ch));
	for (int ch = 0; ch < RP2040_DMA_NCHANNELS; ch++) {
		fake_ch[ch].chan = (uint8_t)ch;
	}
	memset(g_slots, 0, sizeof(g_slots));
	g_init_result = 1;
	fake_busy_after = -1;
	calls[0] = '\0';
	cbs[0] = '\0';
}

/* The cases -----------------------------------------------------------------*/

static const uint16_t words[40];
static uint8_t bytes[40];
#define DATA_CMD ((uintptr_t)0x40044010u)

static void test_start(void)
{
	DMA_HANDLE other;

	reset();
	other = rp2040_dmachannel();
	expect_calls("dmachannel()=0 ");
	assert(!i2c_dw_dma_os_is_ready());
	assert(i2c_dw_dma_os_request_channel() == -ENODEV);
	assert(i2c_dw_dma_os_initialize() == 0);
	expect_calls("trywait()=1 clr(inte0,0x2) trywait()=2 clr(inte0,0x4) ");
	assert(REG(RP2040_DMA_INTE0) == 0x1);
	assert(i2c_dw_dma_os_is_ready());
	assert(i2c_dw_dma_os_initialize() == 0);
	expect_calls("");
	assert(i2c_dw_dma_os_request_channel() == 1);
	assert(i2c_dw_dma_os_request_channel() == 2);
	(void)other;

	/* Exactly two free (the last two): both taken, ready. */
	reset();
	for (int ch = 0; ch < RP2040_DMA_NCHANNELS - 2; ch++) {
		(void)rp2040_dmachannel();
	}
	calls[0] = '\0';
	assert(i2c_dw_dma_os_initialize() == 0);
	expect_calls("trywait()=10 clr(inte0,0x400) trywait()=11 clr(inte0,0x800) ");
	assert(i2c_dw_dma_os_is_ready());
	assert(i2c_dw_dma_os_request_channel() == 10);
	assert(i2c_dw_dma_os_request_channel() == 11);

	/* Exactly one free: taken without waiting, given back, nothing ready;
	 * the channel is free again for other users.
	 */
	reset();
	for (int ch = 0; ch < RP2040_DMA_NCHANNELS - 1; ch++) {
		(void)rp2040_dmachannel();
	}
	calls[0] = '\0';
	assert(i2c_dw_dma_os_initialize() == -EBUSY);
	expect_calls("trywait()=11 clr(inte0,0x800) trywait()=NULL dmafree(11) ");
	assert(!fake_ch[11].inuse);
	assert(!i2c_dw_dma_os_is_ready());
	assert(i2c_dw_dma_os_request_channel() == -ENODEV);
	assert(i2c_dw_dma_os_initialize() == -EBUSY);
	assert(i2c_dw_dma_os_config_tx(11, words, DATA_CMD, 40, core_cb, "tx") == -EINVAL);
	assert(!i2c_dw_dma_os_is_busy(11));
	expect_calls("");
	other = rp2040_dmachannel();
	expect_calls("dmachannel()=11 ");
	assert(other == &fake_ch[11]);

	/* The allocator busy at the second try: the first is given back, nothing
	 * ready, no wait, and no channel is lost for other users.
	 */
	reset();
	fake_busy_after = 1;
	assert(i2c_dw_dma_os_initialize() == -EBUSY);
	expect_calls("trywait()=0 clr(inte0,0x1) trywait()=busy dmafree(0) ");
	assert(!fake_ch[0].inuse);
	assert(!i2c_dw_dma_os_is_ready());
	assert(i2c_dw_dma_os_request_channel() == -ENODEV);
	assert(i2c_dw_dma_os_initialize() == -EBUSY);
	expect_calls("");
	fake_busy_after = -1;
	other = rp2040_dmachannel();
	expect_calls("dmachannel()=0 ");

	/* None free: no channel taken, nothing ready, no wait. */
	reset();
	for (int ch = 0; ch < RP2040_DMA_NCHANNELS; ch++) {
		(void)rp2040_dmachannel();
	}
	calls[0] = '\0';
	assert(i2c_dw_dma_os_initialize() == -EBUSY);
	expect_calls("trywait()=NULL ");
	assert(!i2c_dw_dma_os_is_ready());
	assert(i2c_dw_dma_os_request_channel() == -ENODEV);
	assert(i2c_dw_dma_os_initialize() == -EBUSY);
	expect_calls("");
}

static void test_channels(void)
{
	reset();
	assert(i2c_dw_dma_os_initialize() == 0);
	expect_calls("trywait()=0 clr(inte0,0x1) trywait()=1 clr(inte0,0x2) ");
	assert(i2c_dw_dma_os_request_channel() == 0);
	assert(i2c_dw_dma_os_request_channel() == 1);
	assert(i2c_dw_dma_os_request_channel() == -EBUSY);
	i2c_dw_dma_os_release_channel(1);
	expect_calls("clr(inte0,0x2) ");
	i2c_dw_dma_os_release_channel(1);
	i2c_dw_dma_os_release_channel(7);
	expect_calls("");
	assert(i2c_dw_dma_os_request_channel() == 1);
	assert(i2c_dw_dma_os_config_rx(7, DATA_CMD, bytes, 40, core_cb, "x") == -EINVAL);
	assert(i2c_dw_dma_os_start(7) == -EINVAL);
	assert(i2c_dw_dma_os_stop(7) == -EINVAL);
	assert(!i2c_dw_dma_os_is_busy(7));
	i2c_dw_dma_os_acknowledge(7);
	expect_calls("");
}

static void test_read(void)
{
	DMA_HANDLE other;

	reset();
	assert(i2c_dw_dma_os_initialize() == 0);
	other = rp2040_dmachannel(); /* channel 2, another user's */
	assert(i2c_dw_dma_os_request_channel() == 0);
	assert(i2c_dw_dma_os_request_channel() == 1);
	calls[0] = '\0';

	/* The setup of both sides, not started. */
	assert(i2c_dw_dma_os_config_tx(0, words, DATA_CMD, 40, core_cb, "tx") == 0);
	assert(i2c_dw_dma_os_config_rx(1, DATA_CMD, bytes, 40, core_cb, "rx") == 0);
	expect_calls("txsetup(0,dreq 32,size 1,noincr 0,80) rxsetup(1,dreq 33,size 0,noincr 0,40) ");
	assert(fake_ch[0].read_addr == (uintptr_t)words && fake_ch[0].write_addr == DATA_CMD);
	assert(fake_ch[1].read_addr == DATA_CMD && fake_ch[1].write_addr == (uintptr_t)bytes);
	assert(REG(RP2040_DMA_TRANS_COUNT(0)) == 40 && REG(RP2040_DMA_TRANS_COUNT(1)) == 40);
	/* Zephyr's CTRL values (0x00100015, 0x00108821) without EN. */
	assert(REG(RP2040_DMA_CTRL_TRIG(0)) == 0x00100014u);
	assert(REG(RP2040_DMA_CTRL_TRIG(1)) == 0x00108820u);

	/* The start: RX, then TX; an old flag cleared first. */
	REG(RP2040_DMA_INTR) = 0x3;
	assert(i2c_dw_dma_os_start(1) == 0);
	assert(i2c_dw_dma_os_start(0) == 0);
	expect_calls("ints0(0x2) set(inte0,0x2) dmastart(1) ints0(0x1) set(inte0,0x1) dmastart(0) ");
	assert(REG(RP2040_DMA_CTRL_TRIG(0)) == (0x00100015u | RP2040_DMA_CTRL_TRIG_BUSY));
	assert(REG(RP2040_DMA_CTRL_TRIG(1)) == (0x00108821u | RP2040_DMA_CTRL_TRIG_BUSY));
	assert(REG(RP2040_DMA_INTR) == 0);
	assert(i2c_dw_dma_os_is_busy(0) && i2c_dw_dma_os_is_busy(1));
	assert(i2c_dw_dma_os_start(0) == -EBUSY);
	assert(i2c_dw_dma_os_config_tx(0, words, DATA_CMD, 40, core_cb, "tx") == -EBUSY);
	expect_calls("");

	/* TX ends together with another user's channel: each to its callback. */
	rp2040_dmastart(other, other_cb, NULL);
	calls[0] = '\0';
	dma_end(0);
	dma_end(2);
	dmac_interrupt();
	expect_cbs("cb(tx,0,0) other(2,0) ");
	assert(!i2c_dw_dma_os_is_busy(0));

	/* RX stops on a bus error: NuttX's handler writes the error flags back
	 * to CTRL_TRIG, which starts the channel again; this file aborts it.
	 */
	dma_fail(1, RP2040_DMA_CTRL_TRIG_WRITE_ERROR);
	dmac_interrupt();
	expect_cbs("cb(rx,1,-5) ");
	expect_calls("abort(0x2) ");
	assert((REG(RP2040_DMA_CTRL_TRIG(1)) & (ERRORS | RP2040_DMA_CTRL_TRIG_AHB_ERROR)) == 0);
	assert(i2c_dw_dma_os_is_busy(1));
	abort_ends(1);
	assert(!i2c_dw_dma_os_is_busy(1));

	/* The stop: interrupt off, CHAN_ABORT, no wait; busy until both clear;
	 * a late end calls nothing; the acknowledge clears it.
	 */
	assert(i2c_dw_dma_os_config_tx(0, words, DATA_CMD, 40, core_cb, "tx") == 0);
	assert(i2c_dw_dma_os_start(0) == 0);
	calls[0] = '\0';
	assert(i2c_dw_dma_os_stop(0) == 0);
	expect_calls("clr(inte0,0x1) abort(0x1) ");
	assert(i2c_dw_dma_os_is_busy(0));
	REG(RP2040_DMA_CTRL_TRIG(0)) &= ~RP2040_DMA_CTRL_TRIG_BUSY;
	assert(i2c_dw_dma_os_is_busy(0));
	REG(RP2040_DMA_CHAN_ABORT) = 0;
	assert(!i2c_dw_dma_os_is_busy(0));
	REG(RP2040_DMA_INTR) |= 1u << 0;
	dmac_interrupt();
	expect_cbs("");
	assert(REG(RP2040_DMA_INTR) == 1u << 0);
	i2c_dw_dma_os_acknowledge(0);
	expect_calls("ints0(0x1) ");
	assert(REG(RP2040_DMA_INTR) == 0);

	/* The release: interrupt off; NuttX's handler may still call this
	 * file's callback (rp2040_dmastart() set it), which passes nothing on.
	 */
	assert(i2c_dw_dma_os_config_rx(1, DATA_CMD, bytes, 40, core_cb, "rx") == 0);
	assert(i2c_dw_dma_os_start(1) == 0);
	calls[0] = '\0';
	i2c_dw_dma_os_release_channel(1);
	expect_calls("clr(inte0,0x2) ");
	REG(RP2040_DMA_INTE0) |= 1u << 1;
	dma_end(1);
	dmac_interrupt();
	expect_cbs("");
	expect_calls("");
}

/* NuttX's handler keeps the status of an earlier channel for the later ones
 * of the same interrupt: another user's bus error on channel 0 and the end
 * of our channel 1 give our read -EIO.
 */
static void test_status_carried(void)
{
	DMA_HANDLE other;

	reset();
	other = rp2040_dmachannel();
	assert(i2c_dw_dma_os_initialize() == 0);
	assert(i2c_dw_dma_os_request_channel() == 1);
	assert(i2c_dw_dma_os_config_tx(1, words, DATA_CMD, 40, core_cb, "tx") == 0);
	assert(i2c_dw_dma_os_start(1) == 0);
	rp2040_dmastart(other, other_cb, NULL);
	calls[0] = '\0';
	dma_fail(0, RP2040_DMA_CTRL_TRIG_READ_ERROR);
	dma_end(1);
	dmac_interrupt();
	expect_cbs("other(0,5) cb(tx,1,-5) ");
	expect_calls("abort(0x2) ");
}

int main(void)
{
	test_start();
	test_channels();
	test_read();
	test_status_carried();
	printf("i2c_dw_dma_os: all tests passed\n");
	return 0;
}
