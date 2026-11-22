/* SPDX-License-Identifier: MIT */
/* The host test of the I2C core's FreeRTOS binding on the FreeRTOS POSIX
 * port: common/drivers/i2c_dw/port/freertos (the boundary i2c_dw_os_impl.h
 * on FreeRTOS's task notifications and delays and the TIMER, the devices i2c0 and
 * i2c1 and their Zephyr I2C API in i2c_dw_instance.h, the DMA reads'
 * channels in i2c_dw_dma_os_rp2040.c) and
 * freertos/drivers/compat/zephyr/drivers/i2c.h, under the shared core
 * (common/drivers/i2c_dw/i2c_dw.c, through i2c_dw_core.c here, with the
 * prelude, as freertos/drivers/i2c_dw/i2c_dw.cmake builds it), on fakes of
 * the Pico SDK's GPIO, reset, IRQ, sync, TIMER and DMA functions
 * (pico_fake/, implemented here: they record the calls), on arrays in place of I2C0's and I2C1's
 * registers and a structure in place of the DMA's, which a task of this test
 * plays the controllers, the DMA, the LSM6DSOX on i2c0 and a device at 0x50
 * on i2c1 on (it calls the controllers' handlers and the DMA line's shared
 * handlers that the driver installed, as interrupt handlers).  Built twice
 * (freertos/script/host_tests.sh): i2c0 only, as the default build, and
 * with CONFIG_I2C_DW_I2C1, which adds the tests of i2c1 (the last item).
 *
 * - Before the start: i2c0 is not ready; its calls return -ENODEV; the DMA
 *   part is not ready and gives no channel (-ENODEV).
 * - A block that is not a DesignWare I2C (IC_COMP_TYPE): -EIO, i2c0 not
 *   ready, its init_res EIO; the DMA part's shared handler on DMA_IRQ_1
 *   (the default order priority) and the line enabled before, once.
 * - The start: I2C0's reset, GPIO12 then GPIO13 as Zephyr's pinctrl sets
 *   them (in its order), the I2C0 line; the controller off, the spike
 *   lengths 1, SDA hold kept; i2c0 ready.  A second call does nothing.
 * - WHO_AM_I through i2c_burst_read_dt() at 100 kHz: the commands (0x0f;
 *   then READ|STOP|RESTART), the byte, IC_TAR, the SCL counts 500/625,
 *   IC_CON with the DMA reads' RX_FIFO_FULL_HLD_CTRL, and without it once
 *   the DMA reads are off.
 * - i2c_configure(): fast plus (31/31) and fast (93/109) counts and IC_CON;
 *   high speed and 10-bit addresses refused, nothing changed.
 * - Transfers refused before the core (-EINVAL; no message: 0), touching no
 *   register.
 * - A NACK: -EIO, the core's two log lines, and the next transfer works.
 * - No interrupt: -ETIMEDOUT after the core's 1000 ms (1001 ticks, then 2),
 *   the abort requested, the interrupts masked, and the next transfer
 *   works (the semaphore was reset).
 * - The DMA part (i2c_dw_dma_os_rp2040.c) on its own: the lowest unclaimed channel
 *   of the SDK's record, -EBUSY when none is left; the setup of each side
 *   (addresses, count, CTRL: data size, increments, DREQ 32 or 33, chained
 *   to itself, enabled, normal priority; no trigger), refused on a busy or
 *   not held channel; the start (flag cleared, interrupt enabled on
 *   DMA_IRQ_1, the SDK's start); the handler: the end of a channel gives 0,
 *   a bus error -EIO with the channel's error flags cleared, its flag
 *   cleared and its interrupt disabled; another user's channel on the
 *   shared line is left as it is, and so is DMA_IRQ_0's enable; the stop
 *   (interrupt disabled, CHAN_ABORT, no wait), busy until both BUSY and the
 *   abort are clear, no callback after it, the acknowledge; the release
 *   (unclaimed, no callback after it).
 * - The DMA reads of the core on it: a 40-byte read (the command words, the
 *   setup of both channels, RX started before TX, the bytes, the
 *   thresholds, IC_DMA_CR and the interrupt mask after, both channels
 *   unclaimed); 32 bytes take the DMA, 31 and the 1-byte WHO_AM_I the
 *   interrupts (no channel claimed); no channel, or only one: a fallback,
 *   on the interrupts; a bus error of the TX or the RX channel: -EIO, one
 *   error, both channels stopped and their late flags (RP2040-E13)
 *   cleared, the controller's abort waited for, the channels unclaimed, and
 *   the next transfer works; no end: -ETIMEDOUT after the core's 1000 ms,
 *   one error; the DMA reads off: the interrupts, no channel; a stop that
 *   does not end: -EIO, one error, the core's log line, the channels kept
 *   (never unclaimed) and the controller quarantined (-EIO for the next
 *   transfer).  The counters (transfers, bytes, errors, fallbacks) after
 *   each.
 * - The boundary's semaphore and time functions: K_MSEC() rounding plus
 *   one tick, the deadline's last try without waiting, a give from an
 *   interrupt handler and from a task, a give between a wait's look at the
 *   count and its block (not lost), a notification left on the index (the
 *   wait goes on for the rest of its time), the limit, the reset, the bus
 *   lock (binary, its waiters by priority), the sleep's ticks, the clock in
 *   ms of the TIMER.
 * - The command words of the DMA reads (i2c_dw_rp2040_read_commands() of
 *   the core's i2c_dw_rp2040_dma_priv.h, through i2c_dw_core.c): READ on
 *   each, RESTART on the first when asked, STOP on the last; no buffer, no
 *   length or one longer than the buffer: -EINVAL.
 * - With CONFIG_I2C_DW_I2C1: i2c1 not ready before its start; a block that
 *   is not a DesignWare I2C: -EIO, i2c1 not ready, i2c0 ready; the start:
 *   I2C1's reset, GPIO18 then GPIO19 with the pull-ups, I2C1_IRQ (24), no
 *   DMA handler, the name i2c@40048000, the same API as i2c0's and its own
 *   data, i2c0's registers untouched, a second call does nothing; a write
 *   and a read on its registers (IC_TAR, the counts, IC_CON without
 *   RX_FIFO_FULL_HLD_CTRL); i2c_configure() of i2c1 changes i2c1 only; an
 *   unanswered address: -EIO at once with i2c1's name in the log, and the
 *   next transfer works; reads of 40 bytes as the last message take the
 *   interrupts (no channel, no counter); i2c0's DMA read still works; 50
 *   transfers on each bus from two tasks at once, each bus seeing only its
 *   own commands.
 *
 * Built and run by freertos/script/host_tests.sh.
 */

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/i2c/i2c_dw_rp2040_dma.h>

#include "pico_i2c_fake.h"

#include "i2c_dw_os_impl.h"
#include "i2c_dw_dma_os.h"
#include "i2c_dw_controllers_rp2040.h"
#include "priorities.h"

#define RUNNER_STACK 16384
#define HW_STACK     16384
#define LOG_SIZE     4096

_Thread_local int z_host_isr_depth;

/* tick.c's vApplicationTickHook(), counted as an interrupt handler. */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

/* The calls and the log ------------------------------------------------------*/

static char calls[LOG_SIZE];

static void record(const char *fmt, ...)
{
	size_t used = strlen(calls);
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(calls + used, sizeof(calls) - used, fmt, ap);
	va_end(ap);
	assert(strlen(calls) + 1 < sizeof(calls));
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
}

static char log_text[LOG_SIZE];

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	size_t used = strlen(log_text);

	(void)func;
	snprintf(log_text + used, sizeof(log_text) - used, "<%d> %s: ", level, module);
	used = strlen(log_text);
	vsnprintf(log_text + used, sizeof(log_text) - used, fmt, ap);
	used = strlen(log_text);
	snprintf(log_text + used, sizeof(log_text) - used, "\n");
}

void app_log_printf(int level, const char *module, const char *func, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	app_log_vprintf(level, module, func, fmt, ap);
	va_end(ap);
}

bool app_log_ratelimit_pass(uint32_t *last_ms, uint32_t *skipped, uint32_t rate_ms,
			    uint32_t *skipped_out)
{
	(void)last_ms;
	(void)skipped;
	(void)rate_ms;
	*skipped_out = 0;
	return true;
}

static void expect_log(const char *expected)
{
	if (strcmp(log_text, expected) != 0) {
		printf("expected log: %s\n     got log: %s\n", expected, log_text);
		fflush(stdout);
		assert(0);
	}
	log_text[0] = '\0';
}

/* The fakes of the Pico SDK (pico_fake/pico_i2c_fake.h) ----------------------*/

volatile uint32_t fake_i2c0_regs[64];
volatile uint32_t fake_i2c1_regs[64];

static irq_handler_t handlers[32];
static int irq_masked;

uint __get_current_exception(void)
{
	return z_host_isr_depth > 0 ? 16u + 23u : 0u;
}

uint64_t time_us_64(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000u;
}

void gpio_init(uint gpio)
{
	record("init(%u)", gpio);
}

void gpio_set_function(uint gpio, gpio_function_t fn)
{
	record("func(%u,%s)", gpio, fn == GPIO_FUNC_I2C ? "i2c" : "?");
}

void gpio_set_pulls(uint gpio, bool up, bool down)
{
	record("pulls(%u,%d,%d)", gpio, up, down);
}

void gpio_set_drive_strength(uint gpio, enum gpio_drive_strength drive)
{
	record("drive(%u,%s)", gpio, drive == GPIO_DRIVE_STRENGTH_4MA ? "4mA" : "?");
}

void gpio_set_slew_rate(uint gpio, enum gpio_slew_rate slew)
{
	record("slew(%u,%s)", gpio, slew == GPIO_SLEW_RATE_SLOW ? "slow" : "fast");
}

void gpio_set_input_hysteresis_enabled(uint gpio, bool enabled)
{
	record("schmitt(%u,%d)", gpio, enabled);
}

void gpio_set_input_enabled(uint gpio, bool enabled)
{
	record("ie(%u,%d)", gpio, enabled);
}

void gpio_set_oeover(uint gpio, uint value)
{
	record("oeover(%u,%u)", gpio, value);
}

void gpio_set_outover(uint gpio, uint value)
{
	record("outover(%u,%u)", gpio, value);
}

void gpio_set_inover(uint gpio, uint value)
{
	record("inover(%u,%u)", gpio, value);
}

void gpio_set_irqover(uint gpio, uint value)
{
	record("irqover(%u,%u)", gpio, value);
}

static const char *block_name(uint32_t block)
{
	return block == RESET_I2C0 ? "i2c0" : block == RESET_I2C1 ? "i2c1" : "?";
}

void reset_block_num(uint32_t block_num)
{
	record("reset(%s)", block_name(block_num));
}

void unreset_block_num_wait_blocking(uint32_t block_num)
{
	record("unreset(%s)", block_name(block_num));
}

void irq_set_exclusive_handler(uint num, irq_handler_t handler)
{
	assert(num < 32 && handlers[num] == NULL);
	handlers[num] = handler;
	record("handler(%u)", num);
}

/* The shared handlers of each line, in the order of their order priorities
 * (higher first, as the SDK runs them).
 */
#define SHARED_MAX 4

static struct {
	irq_handler_t handler;
	uint8_t order;
} shared[32][SHARED_MAX];

void irq_add_shared_handler(uint num, irq_handler_t handler, uint8_t order_priority)
{
	int i = SHARED_MAX - 1;

	assert(num < 32 && handlers[num] == NULL && shared[num][i].handler == NULL);
	while (i > 0 &&
	       (shared[num][i - 1].handler == NULL || shared[num][i - 1].order < order_priority)) {
		shared[num][i] = shared[num][i - 1];
		i--;
	}
	shared[num][i].handler = handler;
	shared[num][i].order = order_priority;
	record("shared(%u,%u)", num, order_priority);
}

void irq_set_enabled(uint num, bool enabled)
{
	record("irq(%u,%d)", num, enabled);
}

uint32_t save_and_disable_interrupts(void)
{
	return (uint32_t)irq_masked++;
}

void restore_interrupts(uint32_t status)
{
	irq_masked = (int)status;
}

/* The DMA's registers, and the SDK's record of the claimed channels (the
 * test claims some for "another user").
 */
dma_hw_t fake_dma_hw;
static uint32_t fake_claimed;

int dma_claim_unused_channel(bool required)
{
	assert(!required);
	for (int ch = 0; ch < (int)NUM_DMA_CHANNELS; ch++) {
		if (!(fake_claimed & (1u << ch))) {
			fake_claimed |= 1u << ch;
			record("claim(%d)", ch);
			return ch;
		}
	}
	record("claim(-1)");
	return -1;
}

void dma_channel_unclaim(uint channel)
{
	assert(fake_claimed & (1u << channel));
	fake_claimed &= ~(1u << channel);
	record("unclaim(%u)", channel);
}

void dma_channel_configure(uint channel, const dma_channel_config_t *config,
			   volatile void *write_addr, const volatile void *read_addr,
			   uint32_t encoded_transfer_count, bool trigger)
{
	dma_channel_hw_t *ch = &fake_dma_hw.ch[channel];

	assert(!trigger && !(ch->al1_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS));
	ch->read_addr = (uintptr_t)read_addr;
	ch->write_addr = (uintptr_t)write_addr;
	ch->transfer_count = encoded_transfer_count;
	ch->al1_ctrl = config->ctrl;
	record("configure(%u)", channel);
}

void dma_channel_start(uint channel)
{
	fake_dma_hw.ch[channel].al1_ctrl |= DMA_CH0_CTRL_TRIG_BUSY_BITS;
	record("start(%u)", channel);
}

#define DMA_ERROR_FLAGS (DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS)

void hw_set_bits(io_rw_32 *addr, uint32_t mask)
{
	for (uint32_t i = 0; i < NUM_DMA_CHANNELS; i++) {
		if (addr == &fake_dma_hw.ch[i].al1_ctrl) {
			/* The error flags are write 1 to clear; AHB_ERROR is their OR. */
			uint32_t v =
				(*addr | (mask & ~DMA_ERROR_FLAGS)) & ~(mask & DMA_ERROR_FLAGS);

			v &= ~DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS;
			*addr = v | (v & DMA_ERROR_FLAGS ? DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS : 0);
			record("setbits(al1_ctrl %u,0x%x)", i, (unsigned int)mask);
			return;
		}
	}
	assert(0);
}

/* CHAN_ABORT: a write of 1 aborts that channel, 0 changes nothing, and the
 * bit reads 1 until the abort has ended.  The structure keeps only the last
 * value written, so the stop is wrapped (-Wl,--wrap, host_tests.sh) to keep
 * the bits of each write until the test's DMA ends them (hw_dma()).
 */
static uint32_t fake_aborts;

int __real_i2c_dw_dma_os_stop(uint32_t channel);
int __wrap_i2c_dw_dma_os_stop(uint32_t channel);

int __wrap_i2c_dw_dma_os_stop(uint32_t channel)
{
	int ret = __real_i2c_dw_dma_os_stop(channel);

	fake_aborts |= fake_dma_hw.abort;
	fake_dma_hw.abort = fake_aborts;
	return ret;
}

/* A channel stopped by a bus error: its error flag, AHB_ERROR, its end flag. */
static void dma_fail(int ch, uint32_t error)
{
	fake_dma_hw.ch[ch].al1_ctrl = (fake_dma_hw.ch[ch].al1_ctrl & ~DMA_CH0_CTRL_TRIG_BUSY_BITS) |
				      error | DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS;
	fake_dma_hw.irq_ctrl[0].intr |= 1u << ch;
}

/* The end of a channel's transfers: not busy, its flag. */
static void dma_end(int ch)
{
	fake_dma_hw.ch[ch].al1_ctrl &= ~DMA_CH0_CTRL_TRIG_BUSY_BITS;
	fake_dma_hw.irq_ctrl[0].intr |= 1u << ch;
}

/* DMA_IRQ_1 (or 0) asserted: its shared handlers, as interrupt handlers. */
static void dma_interrupt(uint num)
{
	z_host_isr_depth++;
	for (int i = 0; i < SHARED_MAX && shared[num][i].handler != NULL; i++) {
		shared[num][i].handler();
	}
	z_host_isr_depth--;
}

/* I2C0's registers and the bus ----------------------------------------------*/

#define REG(off) fake_i2c0_regs[(off) / 4]

#define IC_CON         0x00
#define IC_TAR         0x04
#define IC_DATA_CMD    0x10
#define IC_SS_SCL_HCNT 0x14
#define IC_SS_SCL_LCNT 0x18
#define IC_FS_SCL_HCNT 0x1c
#define IC_FS_SCL_LCNT 0x20
#define IC_INTR_STAT   0x2c
#define IC_INTR_MASK   0x30
#define IC_RX_TL       0x38
#define IC_ENABLE      0x6c
#define IC_STATUS      0x70
#define IC_SDA_HOLD    0x7c
#define IC_TX_ABRT_SRC 0x80
#define IC_DMA_CR      0x88
#define IC_DMA_TDLR    0x8c
#define IC_DMA_RDLR    0x90
#define IC_FS_SPKLEN   0xa0
#define IC_HS_SPKLEN   0xa4
#define IC_COMP_TYPE   0xfc
#define DW_MAGIC       0x44570140u
#define STAT_TX_EMPTY  (1u << 4)
#define STAT_RX_FULL   (1u << 2)
#define STAT_TX_ABRT   (1u << 6)
#define STAT_STOP_DET  (1u << 9)
#define STATUS_TFNF    (1u << 1)
#define STATUS_RFNE    (1u << 3)
#define CMD_READ       (1u << 8)
#define CMD_STOP       (1u << 9)
#define CMD_RESTART    (1u << 10)
#define CMD_NONE       0xdead0000u
/* IC_CON as the core writes it (bit 11, BUS_CLEAR_FEATURE_CTRL, is not in
 * the RP2040's register and reads 0 there; the array keeps it).
 */
#define CON_STD_DMA    0xb63u
#define CON_STD        0x963u
#define CON_FAST_DMA   0xb65u

#define LSM6DSOX_ADDR     0x6a
#define LSM6DSOX_WHO_AM_I 0x0f
#define LSM6DSOX_ID       0x6c

/* The task that plays the controller's interrupts, the DMA and the
 * LSM6DSOX.  In the DMA modes the LSM6DSOX answers the interrupt path's
 * messages as in HW_DEVICE, the controller ends an abort (IC_ENABLE.ABORT)
 * and the DMA an abort of a channel (CHAN_ABORT; then, as RP2040-E13 allows,
 * the channel flags its end late), except in HW_DMA_STUCK.
 */
enum hw_mode {
	HW_OFF,          /* no interrupt at all (a dead bus) */
	HW_DEVICE,       /* the LSM6DSOX answers */
	HW_NACK,         /* nothing at the address: the address byte is not acked */
	HW_DMA,          /* the DMA read ends: both channels, then STOP_DET */
	HW_DMA_TX_ERROR, /* the TX channel stops on a read bus error */
	HW_DMA_RX_ERROR, /* the RX channel stops on a write bus error */
	HW_DMA_SILENT,   /* the DMA read never ends */
	HW_DMA_STUCK,    /* never ends, and the channels' aborts never end */
};

static volatile enum hw_mode hw_mode = HW_OFF;
static char hw_log[LOG_SIZE];
static uint8_t device_reg;

static void hw_record(const char *fmt, ...)
{
	size_t used = strlen(hw_log);
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(hw_log + used, sizeof(hw_log) - used, fmt, ap);
	va_end(ap);
	strcat(hw_log, " ");
}

static void expect_hw(const char *expected)
{
	if (strcmp(hw_log, expected) != 0) {
		printf("expected bus: %s\n     got bus: %s\n", expected, hw_log);
		fflush(stdout);
		assert(0);
	}
	hw_log[0] = '\0';
}

/* A controller's interrupt (I2C0's: interrupt()), as its handler sees it. */
static void bus_interrupt(volatile uint32_t *regs, uint irq, uint32_t intr_stat, uint32_t status)
{
	regs[IC_INTR_STAT / 4] = intr_stat;
	regs[IC_STATUS / 4] = status;
	z_host_isr_depth++;
	handlers[irq]();
	z_host_isr_depth--;
	regs[IC_INTR_STAT / 4] = 0;
}

static void interrupt(uint32_t intr_stat, uint32_t status)
{
	bus_interrupt(fake_i2c0_regs, I2C0_IRQ, intr_stat, status);
}

/* One message, armed by the core (its interrupt mask set): the controller
 * asks for the first command (TX_EMPTY); for a read, the byte arrives
 * (RX_FULL): answer(the register last written); STOP_DET after a command
 * with STOP.  Messages of one byte.  In mode HW_NACK nothing acks the
 * address.  The commands go to the log through note().
 */
static void bus_message(volatile uint32_t *regs, uint irq, enum hw_mode mode, uint8_t *reg,
			uint8_t (*answer)(uint8_t reg), void (*note)(const char *fmt, ...))
{
	uint32_t cmd;

	if (mode == HW_NACK) {
		regs[IC_TX_ABRT_SRC / 4] = 1; /* ADDR7BNACK */
		bus_interrupt(regs, irq, STAT_TX_ABRT, STATUS_TFNF);
		regs[IC_TX_ABRT_SRC / 4] = 0;
		note("nack");
		return;
	}
	regs[IC_DATA_CMD / 4] = CMD_NONE;
	bus_interrupt(regs, irq, STAT_TX_EMPTY, STATUS_TFNF);
	cmd = regs[IC_DATA_CMD / 4];
	note("0x%x", (unsigned int)cmd);
	if (cmd == CMD_NONE) {
		return;
	}
	if (cmd & CMD_READ) {
		regs[IC_DATA_CMD / 4] = answer(*reg);
		bus_interrupt(regs, irq, STAT_RX_FULL, STATUS_TFNF | STATUS_RFNE);
	} else {
		*reg = (uint8_t)cmd;
	}
	if (cmd & CMD_STOP) {
		bus_interrupt(regs, irq, STAT_STOP_DET, STATUS_TFNF);
	}
}

static uint8_t lsm6dsox_answer(uint8_t reg)
{
	return reg == LSM6DSOX_WHO_AM_I ? LSM6DSOX_ID : 0;
}

/* I2C0's message, the LSM6DSOX answering. */
static void hw_message(void)
{
	bus_message(fake_i2c0_regs, I2C0_IRQ, hw_mode, &device_reg, lsm6dsox_answer, hw_record);
}

/* The DMA read seen by the bus: the TX channel's command words (copied when
 * it ends) and the RX channel's bytes (0xa0, 0xa1, ...).
 */
static uint16_t dma_words[4096];
static uint32_t dma_word_count;

/* The started channel (BUSY) paced by dreq, or -1. */
static int dma_busy_channel(uint32_t dreq)
{
	for (int ch = 0; ch < (int)NUM_DMA_CHANNELS; ch++) {
		uint32_t ctrl = fake_dma_hw.ch[ch].al1_ctrl;

		if ((ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS) &&
		    ((ctrl & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) >> DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB) ==
			    dreq) {
			return ch;
		}
	}
	return -1;
}

static void hw_dma(void)
{
	uint32_t abort = fake_dma_hw.abort;
	int tx, rx;

	if (abort && hw_mode != HW_DMA_STUCK) {
		for (int ch = 0; ch < (int)NUM_DMA_CHANNELS; ch++) {
			if (abort & (1u << ch)) {
				dma_end(ch);
			}
		}
		fake_aborts = 0;
		fake_dma_hw.abort = 0;
		hw_record("abort(0x%x)", (unsigned int)abort);
	}
	if ((REG(IC_ENABLE) & 2) && hw_mode != HW_DMA_STUCK) {
		REG(IC_ENABLE) &= ~2u;
		hw_record("i2c-abort");
	}
	tx = dma_busy_channel(DREQ_I2C0_TX);
	rx = dma_busy_channel(DREQ_I2C0_RX);
	if (REG(IC_DMA_CR) != 3 || tx < 0 || rx < 0) {
		return;
	}
	switch (hw_mode) {
	case HW_DMA: {
		const dma_channel_hw_t *t = &fake_dma_hw.ch[tx];
		const dma_channel_hw_t *r = &fake_dma_hw.ch[rx];

		assert(t->transfer_count <= ARRAY_SIZE(dma_words));
		dma_word_count = t->transfer_count;
		memcpy(dma_words, (const void *)t->read_addr, dma_word_count * sizeof(uint16_t));
		for (uint32_t i = 0; i < r->transfer_count; i++) {
			((uint8_t *)r->write_addr)[i] = (uint8_t)(0xa0 + i);
		}
		hw_record("dma(%u,%u)", (unsigned int)t->transfer_count,
			  (unsigned int)r->transfer_count);
		dma_end(tx);
		dma_end(rx);
		dma_interrupt(DMA_IRQ_1);
		interrupt(STAT_STOP_DET, STATUS_TFNF);
		break;
	}
	case HW_DMA_TX_ERROR:
		dma_fail(tx, DMA_CH0_CTRL_TRIG_READ_ERROR_BITS);
		hw_record("tx-error");
		dma_interrupt(DMA_IRQ_1);
		break;
	case HW_DMA_RX_ERROR:
		dma_fail(rx, DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS);
		hw_record("rx-error");
		dma_interrupt(DMA_IRQ_1);
		break;
	default:
		break;
	}
}

#ifdef CONFIG_I2C_DW_I2C1
static void hw1_message(void);
#endif

static void hw_main(void *arg)
{
	(void)arg;

	for (;;) {
		vTaskDelay(1);
		if (hw_mode >= HW_DMA) {
			hw_dma();
		}
		/* A message of the interrupt path (TX_EMPTY on; the DMA reads'
		 * mask has no TX_EMPTY).
		 */
		if (hw_mode != HW_OFF && (REG(IC_INTR_MASK) & STAT_TX_EMPTY)) {
			hw_message();
		}
#ifdef CONFIG_I2C_DW_I2C1
		hw1_message();
#endif
	}
}

/* The tests ------------------------------------------------------------------*/

static const struct device *const i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static const struct i2c_dt_spec imu = {
	.bus = DEVICE_DT_GET(DT_NODELABEL(i2c0)),
	.addr = LSM6DSOX_ADDR,
};

/* The commands of a WHO_AM_I read: the register address, then one read
 * with RESTART and STOP.  After a transfer that ended with a read, the
 * address also has RESTART: the core (Zephyr's i2c_dw_transfer()) adds one
 * on a change of direction from the last message of the previous transfer
 * (the controller makes a START of it on an idle bus).
 */
#define WHO_AM_I_BUS            "0xf 0x700 "
#define WHO_AM_I_BUS_AFTER_READ "0x40f 0x700 "

static int who_am_i(void)
{
	uint8_t id = 0;
	int ret = i2c_burst_read_dt(&imu, LSM6DSOX_WHO_AM_I, &id, 1);

	return ret != 0 ? ret : id;
}

static void test_before_start(void)
{
	uint8_t byte;

	assert(!i2c_is_ready_dt(&imu));
	assert(i2c_burst_read_dt(&imu, LSM6DSOX_WHO_AM_I, &byte, 1) == -ENODEV);
	assert(i2c_configure(i2c0, I2C_SPEED_SET(I2C_SPEED_FAST_PLUS)) == -ENODEV);
	assert(!i2c_dw_dma_os_is_ready());
	assert(i2c_dw_dma_os_request_channel() == -ENODEV);
#ifdef CONFIG_I2C_DW_I2C1
	assert(!device_is_ready(DEVICE_DT_GET(DT_NODELABEL(i2c1))));
	assert(i2c_read(DEVICE_DT_GET(DT_NODELABEL(i2c1)), &byte, 1, 0x50) == -ENODEV);
#endif
	expect_calls("");
}

#define PINS(n)                                                                                    \
	"init(" #n ") func(" #n ",i2c) pulls(" #n ",0,0) drive(" #n ",4mA) slew(" #n ",slow) "     \
	"schmitt(" #n ",1) ie(" #n ",1) oeover(" #n ",0) outover(" #n ",0) inover(" #n ",0) "      \
	"irqover(" #n ",0) "

static void test_start(void)
{
	/* The reset values of the RP2040's I2C0 that the start reads. */
	REG(IC_COMP_TYPE) = 0;
	REG(IC_CON) = 0x65;
	REG(IC_SDA_HOLD) = 1;
	REG(IC_FS_SPKLEN) = 7;
	REG(IC_ENABLE) = 1;

	/* Not a DesignWare block: refused after the reset and the pins.  The
	 * DMA part's handler first (the DMA device before i2c0), once.
	 */
	assert(i2c_dw_i2c0_initialize() == -EIO);
	expect_calls("shared(12,128) irq(12,1) reset(i2c0) unreset(i2c0) " PINS(12) PINS(13));
	assert(!device_is_ready(i2c0));
	assert(i2c0->state->init_res == EIO);
	assert(i2c_dw_dma_os_is_ready());

	REG(IC_COMP_TYPE) = DW_MAGIC;
	assert(i2c_dw_i2c0_initialize() == 0);
	expect_calls("reset(i2c0) unreset(i2c0) " PINS(12) PINS(13) "handler(23) irq(23,1) ");
	assert(device_is_ready(i2c0));
	assert(i2c0->state->init_res == 0);
	assert(strcmp(i2c0->name, "i2c@40044000") == 0);
	assert((REG(IC_ENABLE) & 1) == 0);
	assert(REG(IC_FS_SPKLEN) == 1);
	assert(REG(IC_HS_SPKLEN) == 1);
	assert(REG(IC_SDA_HOLD) == 1);
	assert(REG(IC_SS_SCL_HCNT) == 0); /* written at the first transfer */

	assert(i2c_dw_dma_os_initialize() == 0);
	assert(i2c_dw_i2c0_initialize() == 0);
	expect_calls("");
	assert(i2c_dw_rp2040_dma_enabled());
}

static void test_who_am_i(void)
{
	hw_mode = HW_DEVICE;
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS);
	assert((REG(IC_TAR) & 0x3ff) == LSM6DSOX_ADDR);
	assert(REG(IC_SS_SCL_HCNT) == 500);
	assert(REG(IC_SS_SCL_LCNT) == 625);
	assert(REG(IC_CON) == CON_STD_DMA);
	assert(REG(IC_RX_TL) == 0);
	assert(REG(IC_INTR_MASK) == 0);
	assert((REG(IC_ENABLE) & 1) == 1);

	/* Two in a row: the semaphores are back at 0. */
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);

	/* The DMA reads off: IC_CON without RX_FIFO_FULL_HLD_CTRL. */
	i2c_dw_rp2040_dma_set_enabled(false);
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
	assert(REG(IC_CON) == CON_STD);
	i2c_dw_rp2040_dma_set_enabled(true);

	/* A write of two bytes: the last command carries STOP (the first, which
	 * the array does not keep, has RESTART after the read).
	 */
	static const uint8_t ctrl[2] = {0x12, 0x44};

	assert(i2c_write_dt(&imu, ctrl, sizeof(ctrl)) == 0);
	expect_hw("0x244 ");
	expect_log("");
}

static void test_configure(void)
{
	assert(i2c_configure(i2c0, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_FAST_PLUS)) == 0);
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS);
	assert(REG(IC_FS_SCL_HCNT) == 31);
	assert(REG(IC_FS_SCL_LCNT) == 31);
	assert(REG(IC_CON) == CON_FAST_DMA);

	/* Refused: nothing changes. */
	assert(i2c_configure(i2c0, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_HIGH)) == -EINVAL);
	assert(i2c_configure(i2c0, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_ULTRA)) ==
	       -EINVAL);
	assert(i2c_configure(i2c0, I2C_MODE_CONTROLLER | I2C_ADDR_10_BITS |
					   I2C_SPEED_SET(I2C_SPEED_FAST)) == -EINVAL);
	REG(IC_FS_SCL_HCNT) = 0;
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
	assert(REG(IC_FS_SCL_HCNT) == 31);

	assert(i2c_configure(i2c0, I2C_SPEED_SET(I2C_SPEED_FAST)) == 0);
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
	assert(REG(IC_FS_SCL_HCNT) == 93);
	assert(REG(IC_FS_SCL_LCNT) == 109);

	assert(i2c_configure(i2c0, I2C_MODE_CONTROLLER | I2C_SPEED_SET(I2C_SPEED_STANDARD)) == 0);
	REG(IC_SS_SCL_HCNT) = 0;
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
	assert(REG(IC_SS_SCL_HCNT) == 500);
	assert(REG(IC_CON) == CON_STD_DMA);
}

static void test_refused(void)
{
	const struct i2c_driver_api *api = i2c0->api;
	struct i2c_msg msg = {.buf = NULL, .len = 1, .flags = I2C_MSG_WRITE};
	uint8_t byte = 0;

	REG(IC_TAR) = 0;
	assert(i2c_read(i2c0, &byte, 1, 0x80) == -EINVAL);
	assert(i2c_transfer(i2c0, &msg, 1, LSM6DSOX_ADDR) == -EINVAL);
	msg.buf = &byte;
	msg.flags = I2C_MSG_READ | I2C_MSG_ADDR_10_BITS;
	assert(i2c_transfer(i2c0, &msg, 1, LSM6DSOX_ADDR) == -EINVAL);
	assert(i2c_transfer(i2c0, &msg, 0, LSM6DSOX_ADDR) == 0);
	assert(api->transfer(i2c0, NULL, 1, LSM6DSOX_ADDR) == -EINVAL);
	assert(api->transfer(i2c0, &msg, 0, LSM6DSOX_ADDR) == 0);
	vTaskDelay(5);
	expect_hw("");
	assert(REG(IC_TAR) == 0);

	/* A message of no bytes, with or without a buffer, alone or after a
	 * message of one byte: -ENOTSUP before anything is sent.
	 */
	{
		struct i2c_msg empty[2] = {
			{.buf = &byte, .len = 1, .flags = I2C_MSG_WRITE},
			{.buf = &byte, .len = 0, .flags = I2C_MSG_WRITE},
		};

		assert(i2c_transfer(i2c0, &empty[1], 1, LSM6DSOX_ADDR) == -ENOTSUP);
		assert(i2c_write(i2c0, NULL, 0, LSM6DSOX_ADDR) == -ENOTSUP);
		assert(i2c_transfer(i2c0, empty, 2, LSM6DSOX_ADDR) == -ENOTSUP);
		assert(api->transfer(i2c0, empty, 2, LSM6DSOX_ADDR) == -ENOTSUP);
		vTaskDelay(5);
		expect_hw("");
		assert(REG(IC_TAR) == 0);
		assert(empty[1].len == 0);
	}
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
}

static void test_nack(void)
{
	uint8_t byte = 0;

	hw_mode = HW_NACK;
	assert(i2c_read(i2c0, &byte, 1, 0x10) == -EIO);
	expect_hw("nack ");
	expect_log("<1> i2c_dw: NACK on i2c@40044000\n<1> i2c_dw: IO Fail on i2c@40044000\n");
	hw_mode = HW_DEVICE;
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
}

static void test_timeout(void)
{
	TickType_t start, ticks;
	int ret;

	hw_mode = HW_OFF;
	start = xTaskGetTickCount();
	ret = who_am_i();
	ticks = xTaskGetTickCount() - start;
	assert(ret == -ETIMEDOUT);
	/* 1000 ms in 1 ms ticks: 1000 + 1, then the 1 ms wait for the abort
	 * (1 + 1); a few ticks more for the host.
	 */
	assert(ticks >= 1003 && ticks <= 1020);
	assert(REG(IC_ENABLE) & 2); /* ABORT */
	assert(REG(IC_INTR_MASK) == 0);
	expect_hw("");
	expect_log("");

	REG(IC_ENABLE) &= ~2u; /* the controller has ended the abort */
	hw_mode = HW_DEVICE;
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS);
}

/* The DMA part (i2c_dw_dma_os_rp2040.c) --------------------------------------*/

static char dma_cb_log[LOG_SIZE];

static void dma_cb(void *user_data, uint32_t channel, int status)
{
	size_t used = strlen(dma_cb_log);

	assert(z_host_isr_depth > 0);
	snprintf(dma_cb_log + used, sizeof(dma_cb_log) - used, "cb(%s,%u,%d) ",
		 (const char *)user_data, (unsigned int)channel, status);
}

static void expect_cb(const char *expected)
{
	if (strcmp(dma_cb_log, expected) != 0) {
		printf("expected callbacks: %s\n     got callbacks: %s\n", expected, dma_cb_log);
		fflush(stdout);
		assert(0);
	}
	dma_cb_log[0] = '\0';
}

/* CTRL as Zephyr's driver writes it for the two sides on channels 0 and 1:
 * 16 bits, read increment, DREQ 32, chained to 0, enabled; 8 bits, write
 * increment, DREQ 33, chained to 1, enabled.
 */
#define CTRL_TX0 0x00100015u
#define CTRL_RX1 0x00108821u

#define INTE1 fake_dma_hw.irq_ctrl[1].inte
#define INTE0 fake_dma_hw.irq_ctrl[0].inte
#define INTR  fake_dma_hw.irq_ctrl[0].intr

static void test_dma_glue(void)
{
	static const uint16_t words[40];
	static uint8_t bytes[40];
	const uintptr_t data_cmd = I2C0_BASE + IC_DATA_CMD;
	int tx, rx;

	/* The lowest unclaimed channels; none left: -EBUSY, without waiting. */
	fake_claimed = 0;
	tx = i2c_dw_dma_os_request_channel();
	rx = i2c_dw_dma_os_request_channel();
	assert(tx == 0 && rx == 1);
	fake_claimed = 0xfff;
	assert(i2c_dw_dma_os_request_channel() == -EBUSY);
	fake_claimed = 0x3;
	expect_calls("claim(0) claim(1) claim(-1) ");

	/* The two sides' setup, without trigger. */
	assert(i2c_dw_dma_os_config_tx(tx, words, data_cmd, 40, dma_cb, "tx") == 0);
	assert(i2c_dw_dma_os_config_rx(rx, data_cmd, bytes, 40, dma_cb, "rx") == 0);
	expect_calls("configure(0) configure(1) ");
	assert(fake_dma_hw.ch[0].read_addr == (uintptr_t)words);
	assert(fake_dma_hw.ch[0].write_addr == data_cmd);
	assert(fake_dma_hw.ch[0].transfer_count == 40);
	assert(fake_dma_hw.ch[0].al1_ctrl == CTRL_TX0);
	assert(fake_dma_hw.ch[1].read_addr == data_cmd);
	assert(fake_dma_hw.ch[1].write_addr == (uintptr_t)bytes);
	assert(fake_dma_hw.ch[1].transfer_count == 40);
	assert(fake_dma_hw.ch[1].al1_ctrl == CTRL_RX1);
	/* A channel not held: refused, nothing written. */
	assert(i2c_dw_dma_os_config_tx(5, words, data_cmd, 40, dma_cb, "x") == -EINVAL);
	assert(i2c_dw_dma_os_start(5) == -EINVAL);
	assert(i2c_dw_dma_os_stop(5) == -EINVAL);
	expect_calls("");

	/* The start: an old flag cleared, the interrupt on DMA_IRQ_1, the SDK's
	 * start; RX first, as the core starts them.
	 */
	INTR = 0x3;
	assert(i2c_dw_dma_os_start(rx) == 0);
	assert(i2c_dw_dma_os_start(tx) == 0);
	expect_calls("start(1) start(0) ");
	assert(INTR == 0 && INTE1 == 0x3 && INTE0 == 0);
	assert(i2c_dw_dma_os_is_busy(tx) && i2c_dw_dma_os_is_busy(rx));
	/* Busy: no new setup, no second start. */
	assert(i2c_dw_dma_os_config_tx(tx, words, data_cmd, 40, dma_cb, "tx") == -EBUSY);
	assert(i2c_dw_dma_os_start(tx) == -EBUSY);
	expect_calls("");

	/* Another user's channel 5 on the shared line: left with its flag and
	 * enable.  Channel 0 also enabled on DMA_IRQ_0 (as if by someone else):
	 * the handler of DMA_IRQ_1 leaves that enable alone.
	 */
	INTE1 |= 1u << 5;
	INTE0 |= 1u << 0;
	INTR |= 1u << 5;
	dma_interrupt(DMA_IRQ_1);
	expect_cb("");
	dma_end(tx);
	dma_interrupt(DMA_IRQ_1);
	expect_cb("cb(tx,0,0) ");
	assert(INTR == 1u << 5 && INTE1 == ((1u << 5) | 0x2) && INTE0 == 1u << 0);
	INTE0 = 0;
	INTE1 &= ~(1u << 5);
	INTR = 0;

	/* A bus error: -EIO, the error flags cleared through the set alias. */
	dma_fail(rx, DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS);
	dma_interrupt(DMA_IRQ_1);
	expect_cb("cb(rx,1,-5) ");
	expect_calls("setbits(al1_ctrl 1,0x60000000) ");
	assert((fake_dma_hw.ch[1].al1_ctrl & 0xe0000000u) == 0);
	assert(INTR == 0 && INTE1 == 0);
	assert(!i2c_dw_dma_os_is_busy(tx) && !i2c_dw_dma_os_is_busy(rx));

	/* The stop: interrupt off, CHAN_ABORT, no wait; busy until BUSY and
	 * the abort are clear; a late end calls nothing, the acknowledge clears
	 * it.
	 */
	assert(i2c_dw_dma_os_config_tx(tx, words, data_cmd, 40, dma_cb, "tx") == 0);
	assert(i2c_dw_dma_os_start(tx) == 0);
	expect_calls("configure(0) start(0) ");
	assert(i2c_dw_dma_os_stop(tx) == 0);
	assert(fake_dma_hw.abort == 1u << 0 && INTE1 == 0);
	assert(i2c_dw_dma_os_is_busy(tx));
	dma_end(tx);
	assert(i2c_dw_dma_os_is_busy(tx));
	fake_aborts = 0;
	fake_dma_hw.abort = 0;
	assert(!i2c_dw_dma_os_is_busy(tx));
	dma_interrupt(DMA_IRQ_1);
	expect_cb("");
	assert(INTR == 1u << 0);
	i2c_dw_dma_os_acknowledge(tx);
	assert(INTR == 0);

	/* The release: unclaimed, no callback after it, a second release does
	 * nothing.
	 */
	assert(i2c_dw_dma_os_config_rx(rx, data_cmd, bytes, 40, dma_cb, "rx") == 0);
	assert(i2c_dw_dma_os_start(rx) == 0);
	i2c_dw_dma_os_release_channel(tx);
	i2c_dw_dma_os_release_channel(rx);
	i2c_dw_dma_os_release_channel(rx);
	expect_calls("configure(1) start(1) unclaim(0) unclaim(1) ");
	assert(INTE1 == 0);
	dma_end(rx);
	INTE1 = 0x2;
	dma_interrupt(DMA_IRQ_1);
	expect_cb("");
	assert(i2c_dw_dma_os_start(rx) == -EINVAL);
	INTE1 = 0;
	INTR = 0;
	assert(fake_claimed == 0);
	expect_calls("");
}

/* The DMA reads of the core ------------------------------------------------*/

/* i2c_dw_core.c */
void i2c_dw_test_clear_quarantine(void);

static struct i2c_dw_rp2040_dma_stats stats_before;

static void stats_mark(void)
{
	i2c_dw_rp2040_dma_get_stats(&stats_before);
}

/* The counters' changes since stats_mark(). */
static void expect_stats(uint32_t transfers, uint32_t bytes, uint32_t errors, uint32_t fallbacks)
{
	struct i2c_dw_rp2040_dma_stats now;

	i2c_dw_rp2040_dma_get_stats(&now);
	if (now.transfers - stats_before.transfers != transfers ||
	    now.bytes - stats_before.bytes != bytes || now.errors - stats_before.errors != errors ||
	    now.fallbacks - stats_before.fallbacks != fallbacks) {
		printf("expected +%u/%u/%u/%u, got +%u/%u/%u/%u\n", (unsigned int)transfers,
		       (unsigned int)bytes, (unsigned int)errors, (unsigned int)fallbacks,
		       (unsigned int)(now.transfers - stats_before.transfers),
		       (unsigned int)(now.bytes - stats_before.bytes),
		       (unsigned int)(now.errors - stats_before.errors),
		       (unsigned int)(now.fallbacks - stats_before.fallbacks));
		fflush(stdout);
		assert(0);
	}
	stats_mark();
}

/* After a read, DMA or not: the controller's DMA off, its interrupts
 * masked, no channel's interrupt or flag left, no abort pending.
 */
static void expect_dma_idle(void)
{
	assert(REG(IC_DMA_CR) == 0);
	assert(REG(IC_INTR_MASK) == 0);
	assert(INTE1 == 0 && INTE0 == 0 && INTR == 0);
	assert(fake_dma_hw.abort == 0);
}

#define FIFO_DATA_OUT_TAG 0x78

static void test_dma_reads(void)
{
	static uint8_t buf[64];
	TickType_t start, ticks;
	int ret;

	stats_mark();

	/* A DMA read of 40 bytes (after the register address on the
	 * interrupts): the commands (RESTART on the first, STOP on the last),
	 * both sides set up, RX started before TX, the bytes, the controller's
	 * thresholds, the channels released.
	 */
	hw_mode = HW_DMA;
	memset(buf, 0, sizeof(buf));
	assert(i2c_burst_read_dt(&imu, FIFO_DATA_OUT_TAG, buf, 40) == 0);
	expect_hw("0x478 dma(40,40) ");
	expect_calls("claim(0) claim(1) configure(0) configure(1) start(1) start(0) unclaim(0) "
		     "unclaim(1) ");
	expect_stats(1, 40, 0, 0);
	for (int i = 0; i < 40; i++) {
		assert(buf[i] == 0xa0 + i);
	}
	assert(buf[40] == 0);
	assert(dma_word_count == 40);
	assert(dma_words[0] == (CMD_READ | CMD_RESTART));
	for (int i = 1; i < 39; i++) {
		assert(dma_words[i] == CMD_READ);
	}
	assert(dma_words[39] == (CMD_READ | CMD_STOP));
	assert(fake_dma_hw.ch[0].al1_ctrl == CTRL_TX0);
	assert(fake_dma_hw.ch[0].write_addr == I2C0_BASE + IC_DATA_CMD);
	assert(fake_dma_hw.ch[1].al1_ctrl == CTRL_RX1);
	assert(fake_dma_hw.ch[1].read_addr == I2C0_BASE + IC_DATA_CMD);
	assert(REG(IC_DMA_TDLR) == 8 && REG(IC_DMA_RDLR) == 0);
	expect_dma_idle();
	assert(fake_claimed == 0);
	expect_log("");

	/* 32 bytes take the DMA; 31, and WHO_AM_I's one, the interrupts (no
	 * claim: a NACK ends the 31-byte read at once).
	 */
	assert(i2c_burst_read_dt(&imu, FIFO_DATA_OUT_TAG, buf, 32) == 0);
	expect_hw("0x478 dma(32,32) ");
	expect_calls("claim(0) claim(1) configure(0) configure(1) start(1) start(0) unclaim(0) "
		     "unclaim(1) ");
	expect_stats(1, 32, 0, 0);
	hw_mode = HW_DEVICE;
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
	hw_mode = HW_NACK;
	assert(i2c_read(i2c0, buf, 31, 0x10) == -EIO);
	expect_hw("nack ");
	expect_log("<1> i2c_dw: NACK on i2c@40044000\n<1> i2c_dw: IO Fail on i2c@40044000\n");
	expect_calls("");
	expect_stats(0, 0, 0, 0);

	/* No channel left (another user holds them all), or only one: a
	 * fallback each, then the interrupts.
	 */
	fake_claimed = 0xfff;
	assert(i2c_read(i2c0, buf, 40, 0x10) == -EIO);
	expect_hw("nack ");
	expect_calls("claim(-1) ");
	fake_claimed = 0xffe;
	assert(i2c_read(i2c0, buf, 40, 0x10) == -EIO);
	expect_hw("nack ");
	expect_calls("claim(0) claim(-1) unclaim(0) ");
	fake_claimed = 0;
	expect_log("<1> i2c_dw: NACK on i2c@40044000\n<1> i2c_dw: IO Fail on i2c@40044000\n"
		   "<1> i2c_dw: NACK on i2c@40044000\n<1> i2c_dw: IO Fail on i2c@40044000\n");
	expect_stats(0, 0, 0, 2);

	/* The DMA reads off: the interrupts, no channel. */
	i2c_dw_rp2040_dma_set_enabled(false);
	assert(i2c_read(i2c0, buf, 40, 0x10) == -EIO);
	expect_hw("nack ");
	expect_calls("");
	expect_log("<1> i2c_dw: NACK on i2c@40044000\n<1> i2c_dw: IO Fail on i2c@40044000\n");
	expect_stats(0, 0, 0, 0);
	i2c_dw_rp2040_dma_set_enabled(true);

	/* A bus error of either channel: -EIO and one error; both stopped, the
	 * controller's abort waited for, the late flags cleared, both
	 * released; the next transfer works.
	 */
	for (int side = 0; side < 2; side++) {
		hw_mode = side == 0 ? HW_DMA_TX_ERROR : HW_DMA_RX_ERROR;
		assert(i2c_burst_read_dt(&imu, FIFO_DATA_OUT_TAG, buf, 40) == -EIO);
		expect_calls(
			side == 0 ? "claim(0) claim(1) configure(0) configure(1) start(1) start(0) "
				    "setbits(al1_ctrl 0,0x60000000) unclaim(0) unclaim(1) "
				  : "claim(0) claim(1) configure(0) configure(1) start(1) start(0) "
				    "setbits(al1_ctrl 1,0x60000000) unclaim(0) unclaim(1) ");
		expect_stats(0, 0, 1, 0);
		expect_dma_idle();
		assert(!i2c_dw_dma_os_is_busy(0) && !i2c_dw_dma_os_is_busy(1));
		assert((REG(IC_ENABLE) & 2) == 0);
		hw_log[0] = '\0';
		expect_log("");
		hw_mode = HW_DEVICE;
		assert(who_am_i() == LSM6DSOX_ID);
		expect_hw(WHO_AM_I_BUS_AFTER_READ);
	}

	/* No end: -ETIMEDOUT after the core's 1000 ms, one error, released. */
	hw_mode = HW_DMA_SILENT;
	start = xTaskGetTickCount();
	ret = i2c_burst_read_dt(&imu, FIFO_DATA_OUT_TAG, buf, 40);
	ticks = xTaskGetTickCount() - start;
	assert(ret == -ETIMEDOUT);
	assert(ticks >= 1001 && ticks <= 1030);
	expect_calls("claim(0) claim(1) configure(0) configure(1) start(1) start(0) unclaim(0) "
		     "unclaim(1) ");
	expect_stats(0, 0, 1, 0);
	expect_dma_idle();
	hw_log[0] = '\0';
	expect_log("");

	/* Aborts that never end: after the timeout and the 100 ms wait, -EIO,
	 * one error, the log line, the channels kept, the controller
	 * quarantined.
	 */
	hw_mode = HW_DMA_STUCK;
	start = xTaskGetTickCount();
	ret = i2c_burst_read_dt(&imu, FIFO_DATA_OUT_TAG, buf, 40);
	ticks = xTaskGetTickCount() - start;
	assert(ret == -EIO);
	/* 1000 ms in ticks, then 100 ms of i2c_dw_os_uptime_ms() (the TIMER,
	 * here the host's clock, which the POSIX port's ticks follow loosely).
	 */
	assert(ticks >= 1080 && ticks <= 1160);
	expect_calls("claim(0) claim(1) configure(0) configure(1) start(1) start(0) ");
	expect_stats(0, 0, 1, 0);
	expect_log("<1> i2c_dw: DMA channels did not stop; I2C controller quarantined\n");
	assert(fake_claimed == 0x3);
	assert(i2c_dw_dma_os_is_busy(0) && i2c_dw_dma_os_is_busy(1));
	hw_mode = HW_DEVICE;
	assert(who_am_i() == -EIO);
	assert(i2c_configure(i2c0, I2C_SPEED_SET(I2C_SPEED_STANDARD)) == -EIO);
	hw_log[0] = '\0';
	expect_calls("");

	/* For the tests after this one: the controller back, the DMA idle (the
	 * two channels stay held, as on the board).
	 */
	i2c_dw_test_clear_quarantine();
	fake_dma_hw.ch[0].al1_ctrl &= ~DMA_CH0_CTRL_TRIG_BUSY_BITS;
	fake_dma_hw.ch[1].al1_ctrl &= ~DMA_CH0_CTRL_TRIG_BUSY_BITS;
	fake_aborts = 0;
	fake_dma_hw.abort = 0;
	REG(IC_ENABLE) &= ~2u;
	assert(who_am_i() == LSM6DSOX_ID);
	hw_log[0] = '\0';
	expect_log("");
}

#ifdef CONFIG_I2C_DW_I2C1
/* I2C1 (CONFIG_I2C_DW_I2C1) ---------------------------------------------------*/

#define REG1(off) fake_i2c1_regs[(off) / 4]

/* IC_CON of i2c1: never RX_FIFO_FULL_HLD_CTRL (the DMA reads are i2c0's). */
#define CON1_STD  0x963u
#define CON1_FAST 0x965u

/* A device at 0x50 on i2c1 that answers a read with its register + 0x30. */
#define DEV1_ADDR 0x50

static const struct device *const i2c1 = DEVICE_DT_GET(DT_NODELABEL(i2c1));

static volatile enum hw_mode hw1_mode = HW_OFF;
static char hw1_log[LOG_SIZE];
static uint8_t device1_reg;

static void hw1_record(const char *fmt, ...)
{
	size_t used = strlen(hw1_log);
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(hw1_log + used, sizeof(hw1_log) - used, fmt, ap);
	va_end(ap);
	strcat(hw1_log, " ");
}

static void expect_hw1(const char *expected)
{
	if (strcmp(hw1_log, expected) != 0) {
		printf("expected bus 1: %s\n     got bus 1: %s\n", expected, hw1_log);
		fflush(stdout);
		assert(0);
	}
	hw1_log[0] = '\0';
}

static uint8_t dev1_answer(uint8_t reg)
{
	return (uint8_t)(reg + 0x30);
}

/* From hw_main(): I2C1's message of the interrupt path. */
static void hw1_message(void)
{
	if (hw1_mode != HW_OFF && (REG1(IC_INTR_MASK) & STAT_TX_EMPTY)) {
		bus_message(fake_i2c1_regs, I2C1_IRQ, hw1_mode, &device1_reg, dev1_answer,
			    hw1_record);
	}
}

static int dev1_read(uint8_t reg)
{
	uint8_t value = 0;
	int ret = i2c_write_read(i2c1, DEV1_ADDR, &reg, 1, &value, 1);

	return ret != 0 ? ret : value;
}

#define PINS_PULL_UP(n)                                                                            \
	"init(" #n ") func(" #n ",i2c) pulls(" #n ",1,0) drive(" #n ",4mA) slew(" #n ",slow) "     \
	"schmitt(" #n ",1) ie(" #n ",1) oeover(" #n ",0) outover(" #n ",0) inover(" #n ",0) "      \
	"irqover(" #n ",0) "

/* The start of I2C1, after i2c0's: its reset, GPIO18 and GPIO19 with the
 * pull-ups, I2C1_IRQ (no DMA handler); a block that is not a DesignWare
 * I2C: -EIO, i2c1 not ready, i2c0 untouched.
 */
static void test_i2c1_start(void)
{
	uint32_t i2c0_before[64];

	for (int i = 0; i < 64; i++) {
		i2c0_before[i] = fake_i2c0_regs[i];
	}
	REG1(IC_COMP_TYPE) = 0;
	REG1(IC_CON) = 0x65;
	REG1(IC_SDA_HOLD) = 1;
	REG1(IC_FS_SPKLEN) = 7;
	REG1(IC_ENABLE) = 1;

	assert(i2c_dw_i2c1_initialize() == -EIO);
	expect_calls("reset(i2c1) unreset(i2c1) " PINS_PULL_UP(18) PINS_PULL_UP(19));
	assert(!device_is_ready(i2c1));
	assert(i2c1->state->init_res == EIO);
	assert(device_is_ready(i2c0));

	REG1(IC_COMP_TYPE) = DW_MAGIC;
	assert(i2c_dw_i2c1_initialize() == 0);
	expect_calls("reset(i2c1) unreset(i2c1) " PINS_PULL_UP(18)
			     PINS_PULL_UP(19) "handler(24) irq(24,1) ");
	assert(device_is_ready(i2c1));
	assert(i2c1->state->init_res == 0);
	assert(strcmp(i2c1->name, "i2c@40048000") == 0);
	assert(i2c1->api == i2c0->api);
	assert(i2c1->data != i2c0->data);
	assert((REG1(IC_ENABLE) & 1) == 0);
	assert(REG1(IC_FS_SPKLEN) == 1 && REG1(IC_HS_SPKLEN) == 1 && REG1(IC_SDA_HOLD) == 1);
	assert(i2c_dw_i2c1_initialize() == 0);
	expect_calls("");
	for (int i = 0; i < 64; i++) {
		assert(fake_i2c0_regs[i] == i2c0_before[i]);
	}
	expect_log("");
}

/* Transfers on i2c1: a write and a read through the fake device, IC_TAR,
 * the counts and IC_CON without RX_FIFO_FULL_HLD_CTRL; i2c_configure() of
 * i2c1 alone; an unanswered address (-EIO, the log with i2c1's name, the
 * next transfer works); a long last read never takes the DMA path; i2c0
 * unaffected throughout.
 */
static void test_i2c1_transfers(void)
{
	static const uint8_t write[2] = {0x12, 0x44};
	static uint8_t buf[64];
	uint32_t i2c0_con = REG(IC_CON);

	stats_mark();
	hw1_mode = HW_DEVICE;
	hw_mode = HW_DEVICE;
	assert(dev1_read(0x05) == 0x35);
	expect_hw1("0x5 0x700 ");
	assert((REG1(IC_TAR) & 0x3ff) == DEV1_ADDR);
	assert(REG1(IC_SS_SCL_HCNT) == 500 && REG1(IC_SS_SCL_LCNT) == 625);
	assert(REG1(IC_CON) == CON1_STD);
	assert(i2c_write(i2c1, write, sizeof(write), DEV1_ADDR) == 0);
	expect_hw1("0x244 ");
	expect_hw("");
	assert(REG(IC_CON) == i2c0_con);

	/* The speed class of i2c1 only. */
	assert(i2c_configure(i2c1, I2C_SPEED_SET(I2C_SPEED_FAST)) == 0);
	assert(dev1_read(0x06) == 0x36);
	expect_hw1("0x6 0x700 ");
	assert(REG1(IC_FS_SCL_HCNT) == 93 && REG1(IC_FS_SCL_LCNT) == 109);
	assert(REG1(IC_CON) == CON1_FAST);
	assert(who_am_i() == LSM6DSOX_ID);
	expect_hw(WHO_AM_I_BUS_AFTER_READ);
	assert(REG(IC_CON) == CON_STD_DMA);
	assert(i2c_configure(i2c1, I2C_SPEED_SET(I2C_SPEED_STANDARD)) == 0);

	/* An address that nothing answers: -EIO at once, not a timeout. */
	TickType_t start = xTaskGetTickCount();

	hw1_mode = HW_NACK;
	assert(i2c_read(i2c1, buf, 1, 0x10) == -EIO);
	assert(xTaskGetTickCount() - start < 100);
	expect_hw1("nack ");
	expect_log("<1> i2c_dw: NACK on i2c@40048000\n<1> i2c_dw: IO Fail on i2c@40048000\n");

	/* A read of 40 bytes, the last message (the DMA path's on i2c0): the
	 * interrupts, no channel, no counter.
	 */
	assert(i2c_read(i2c1, buf, 40, 0x10) == -EIO);
	expect_hw1("nack ");
	expect_log("<1> i2c_dw: NACK on i2c@40048000\n<1> i2c_dw: IO Fail on i2c@40048000\n");
	assert(i2c_burst_read(i2c1, 0x10, 0x78, buf, 40) == -EIO);
	expect_hw1("nack ");
	expect_log("<1> i2c_dw: NACK on i2c@40048000\n<1> i2c_dw: IO Fail on i2c@40048000\n");
	expect_calls("");
	expect_stats(0, 0, 0, 0);
	assert(REG1(IC_DMA_CR) == 0 && REG1(IC_INTR_MASK) == 0);
	assert(REG1(IC_CON) == CON1_STD);

	/* The next transfer works (no RESTART on its first command: the last
	 * message, the burst read's register address, was a write).
	 */
	hw1_mode = HW_DEVICE;
	assert(dev1_read(0x07) == 0x37);
	expect_hw1("0x7 0x700 ");

	/* i2c0's DMA reads still work (and stay on i2c0's registers; channels
	 * 0 and 1 are still held since the quarantine of test_dma_reads()).
	 */
	hw_mode = HW_DMA;
	assert(i2c_burst_read_dt(&imu, FIFO_DATA_OUT_TAG, buf, 40) == 0);
	expect_hw("0x478 dma(40,40) ");
	expect_calls("claim(2) claim(3) configure(2) configure(3) start(3) start(2) unclaim(2) "
		     "unclaim(3) ");
	expect_stats(1, 40, 0, 0);
	assert(REG(IC_CON) == CON_STD_DMA);
	hw_mode = HW_DEVICE;
	expect_hw1("");
	expect_log("");
}

/* Transfers on both buses from two tasks at once: each bus has its own lock
 * and semaphores.
 */
#define INTERLEAVED 50

/* A bus log of count pairs "0x<first> 0x700 ", the first commands from lo
 * to hi in order (all lo when lo == hi).
 */
static void expect_pairs(const char *log, int count, unsigned int lo, unsigned int hi)
{
	const char *p = log;

	for (int i = 0; i < count; i++) {
		unsigned int first, read;
		int used = 0;

		assert(sscanf(p, "0x%x 0x%x %n", &first, &read, &used) == 2 && used > 0);
		assert(first == (lo == hi ? lo : lo + (unsigned int)i) && first <= hi);
		assert(read == (CMD_READ | CMD_STOP | CMD_RESTART));
		p += used;
	}
	assert(*p == '\0');
}

static volatile int bus1_done;
static volatile int bus1_failures;

static void bus1_main(void *arg)
{
	(void)arg;
	for (int i = 0; i < INTERLEAVED; i++) {
		if (dev1_read((uint8_t)i) != ((i + 0x30) & 0xff)) {
			bus1_failures++;
		}
	}
	bus1_done = 1;
	vTaskDelete(NULL);
}

static void test_i2c1_interleaved(void)
{
	int failures = 0;

	hw_mode = HW_DEVICE;
	hw1_mode = HW_DEVICE;
	bus1_done = 0;
	bus1_failures = 0;
	assert(xTaskCreate(bus1_main, "bus1", HW_STACK / sizeof(StackType_t), NULL, APP_PRIO_SHELL,
			   NULL) == pdPASS);
	for (int i = 0; i < INTERLEAVED; i++) {
		if (who_am_i() != LSM6DSOX_ID) {
			failures++;
		}
	}
	while (!bus1_done) {
		vTaskDelay(1);
	}
	assert(failures == 0 && bus1_failures == 0);
	/* Each bus saw only its own commands, in pairs: WHO_AM_I's on i2c0
	 * (0x40f, then a read), the device's registers 0 to 49 on i2c1 (0x4nn,
	 * then a read).
	 */
	expect_pairs(hw_log, INTERLEAVED, 0x40f, 0x40f);
	expect_pairs(hw1_log, INTERLEAVED, 0x400, 0x400 + INTERLEAVED - 1);
	hw_log[0] = '\0';
	hw1_log[0] = '\0';
	expect_log("");
	expect_calls("");
	hw1_mode = HW_OFF;
}
#endif /* CONFIG_I2C_DW_I2C1 */

static void give_from_isr(void *arg)
{
	z_host_isr_depth++;
	i2c_dw_os_sem_give(arg);
	z_host_isr_depth--;
}

static void giver_main(void *arg)
{
	vTaskDelay(20);
	give_from_isr(arg);
	vTaskDelete(NULL);
}

/* The window between a wait's look at the count and its block: the wait
 * reads the tick count there (the only FreeRTOS call between the two), so
 * the wrapped xTaskGetTickCount() (-Wl,--wrap, host_tests.sh) gives the
 * armed semaphore once, as an interrupt handler would at that moment, while
 * the waiter is on its list.
 */
static i2c_dw_os_sem_t *race_sem;

TickType_t __real_xTaskGetTickCount(void);
TickType_t __wrap_xTaskGetTickCount(void);

TickType_t __wrap_xTaskGetTickCount(void)
{
	i2c_dw_os_sem_t *sem = race_sem;

	if (sem != NULL && sem->waiters != NULL &&
	    sem->waiters->task == xTaskGetCurrentTaskHandle()) {
		race_sem = NULL;
		give_from_isr(sem);
	}
	return __real_xTaskGetTickCount();
}

/* The bus lock's waiters: each takes the lock, notes its name, gives it
 * back.
 */
static i2c_dw_os_sem_t lock_sem;
static char lock_order[32];

static void locker_main(void *arg)
{
	assert(i2c_dw_os_bus_lock(&lock_sem) == 0);
	strcat(lock_order, arg);
	i2c_dw_os_bus_unlock(&lock_sem);
	vTaskDelete(NULL);
}

static void test_boundary(void)
{
	static i2c_dw_os_sem_t sem;
	TickType_t start, end, ticks;
	int64_t ms;

	i2c_dw_os_sem_init(&sem, 0, I2C_DW_OS_SEM_MAX_LIMIT);

	/* The ticks of the waits, as the layer's k_sem_take(K_MSEC(ms)) and
	 * k_msleep() had them: ms rounded up to ticks, plus one; 0 does not
	 * block.
	 */
	assert(i2c_dw_os_ms_to_ticks(0) == 0);
	assert(i2c_dw_os_ms_to_ticks(1) == 1);
	assert(i2c_dw_os_ms_to_ticks(1000) == 1000);
	assert(i2c_dw_os_block_ticks(0) == 0);
	assert(i2c_dw_os_block_ticks(1) == 2);
	assert(i2c_dw_os_block_ticks(1000) == 1001);
	assert(i2c_dw_os_block_ticks(UINT64_MAX) == portMAX_DELAY - 1);
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_ms(&sem, 0) == -EBUSY);
	assert(xTaskGetTickCount() - start <= 1);
	start = xTaskGetTickCount();
	i2c_dw_os_msleep(5);
	ticks = xTaskGetTickCount() - start;
	assert(ticks >= 6 && ticks <= 8);

	/* K_MSEC(ms) in ticks (1 ms, as on the board), plus one. */
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_ms(&sem, 10) == -EAGAIN);
	ticks = xTaskGetTickCount() - start;
	assert(ticks >= 11 && ticks <= 13);

	/* The deadline: now + the ticks (at least one); once passed, one try
	 * without waiting.
	 */
	start = xTaskGetTickCount();
	end = i2c_dw_os_timepoint_calc(10);
	assert(end - start >= 10 && end - start <= 11);
	assert(i2c_dw_os_timepoint_calc(0) - xTaskGetTickCount() <= 1);
	end = xTaskGetTickCount() - 5;
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_until(&sem, end) != 0);
	assert(xTaskGetTickCount() - start <= 1);
	give_from_isr(&sem);
	assert(i2c_dw_os_sem_take_until(&sem, end) == 0);

	/* A give from an interrupt handler wakes the waiting task. */
	assert(xTaskCreate(giver_main, "giver", HW_STACK / sizeof(StackType_t), &sem,
			   APP_PRIO_SHELL + 1, NULL) == pdPASS);
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_until(&sem, i2c_dw_os_timepoint_calc(1000)) == 0);
	ticks = xTaskGetTickCount() - start;
	assert(ticks >= 19 && ticks <= 30);

	assert(sem.count == 0 && sem.waiters == NULL);

	/* A give between the look at the count and the block is not lost: the
	 * wait returns 0 at once.
	 */
	race_sem = &sem;
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_ms(&sem, 1000) == 0);
	assert(race_sem == NULL);
	assert(xTaskGetTickCount() - start <= 1);
	assert(sem.count == 0 && sem.waiters == NULL);

	/* A notification left on the index (another wait's, or a stray one)
	 * does not end a wait: it waits for the rest of its time, -EAGAIN, or
	 * until a give.
	 */
	(void)xTaskNotifyGiveIndexed(xTaskGetCurrentTaskHandle(), I2C_DW_OS_NOTIFY_INDEX);
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_ms(&sem, 10) == -EAGAIN);
	ticks = xTaskGetTickCount() - start;
	assert(ticks >= 11 && ticks <= 13);
	assert(sem.waiters == NULL);
	(void)xTaskNotifyGiveIndexed(xTaskGetCurrentTaskHandle(), I2C_DW_OS_NOTIFY_INDEX);
	assert(xTaskCreate(giver_main, "giver", HW_STACK / sizeof(StackType_t), &sem,
			   APP_PRIO_SHELL + 1, NULL) == pdPASS);
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_ms(&sem, 1000) == 0);
	ticks = xTaskGetTickCount() - start;
	assert(ticks >= 19 && ticks <= 30);
	assert(sem.count == 0 && sem.waiters == NULL);

	/* A give from a task; the limit; the reset of a count of two. */
	i2c_dw_os_sem_give(&sem);
	give_from_isr(&sem);
	assert(sem.count == 2);
	i2c_dw_os_sem_reset(&sem);
	assert(sem.count == 0);
	assert(i2c_dw_os_sem_take_until(&sem, xTaskGetTickCount()) == -EBUSY);
	i2c_dw_os_sem_init(&sem, 0, 2);
	for (int i = 0; i < 3; i++) {
		give_from_isr(&sem);
	}
	assert(sem.count == 2);

	/* The lock: binary, a second unlock leaves it at 1. */
	i2c_dw_os_sem_init(&lock_sem, 1, 1);
	assert(i2c_dw_os_bus_lock(&lock_sem) == 0);
	assert(lock_sem.count == 0);
	i2c_dw_os_bus_unlock(&lock_sem);
	i2c_dw_os_bus_unlock(&lock_sem);
	assert(lock_sem.count == 1);

	/* Its waiters by priority, not by arrival: held here, a low task then a
	 * high one wait; the unlock hands the lock to the high one, its unlock
	 * to the low one, and the count is back at 1.
	 */
	lock_order[0] = '\0';
	assert(i2c_dw_os_bus_lock(&lock_sem) == 0);
	assert(xTaskCreate(locker_main, "low", HW_STACK / sizeof(StackType_t), "low ",
			   APP_PRIO_SHELL + 1, NULL) == pdPASS);
	assert(xTaskCreate(locker_main, "high", HW_STACK / sizeof(StackType_t), "high ",
			   APP_PRIO_SHELL + 2, NULL) == pdPASS);
	vTaskDelay(5);
	assert(lock_order[0] == '\0');
	i2c_dw_os_bus_unlock(&lock_sem);
	vTaskDelay(5);
	assert(strcmp(lock_order, "high low ") == 0);
	assert(lock_sem.count == 1 && lock_sem.waiters == NULL);

	/* The clock: the TIMER in ms. */
	ms = i2c_dw_os_uptime_ms();
	assert(ms == (int64_t)(time_us_64() / 1000u) || ms + 1 == (int64_t)(time_us_64() / 1000u));

	/* The interrupt lock nests. */
	uint32_t outer = i2c_dw_os_irq_lock();
	uint32_t inner = i2c_dw_os_irq_lock();

	assert(irq_masked == 2);
	i2c_dw_os_irq_unlock(inner);
	assert(irq_masked == 1);
	i2c_dw_os_irq_unlock(outer);
	assert(irq_masked == 0);
	assert(I2C_DW_CLOCK_SPEED == 125);
}

/* i2c_dw_core.c */
int i2c_dw_test_read_commands(uint16_t *commands, size_t capacity, size_t length, bool restart);

static void test_read_commands(void)
{
	static uint16_t commands[3585];

	assert(i2c_dw_test_read_commands(NULL, 4, 1, false) == -EINVAL);
	assert(i2c_dw_test_read_commands(commands, 4, 0, false) == -EINVAL);
	assert(i2c_dw_test_read_commands(commands, 4, 5, false) == -EINVAL);
	assert(i2c_dw_test_read_commands(commands, 4, 1, true) == 0);
	assert(commands[0] == 0x700);
	assert(i2c_dw_test_read_commands(commands, 4, 4, false) == 0);
	assert(commands[0] == 0x100 && commands[1] == 0x100 && commands[2] == 0x100 &&
	       commands[3] == 0x300);
	assert(i2c_dw_test_read_commands(commands, 3584, 3584, true) == 0);
	assert(commands[0] == 0x500 && commands[3583] == 0x300 && commands[3584] == 0);
	for (size_t i = 1; i < 3583; i++) {
		assert(commands[i] == 0x100);
	}
}

static void runner(void *arg)
{
	(void)arg;

	test_before_start();
	test_start();
	test_who_am_i();
	test_configure();
	test_refused();
	test_nack();
	test_timeout();
	test_dma_glue();
	test_dma_reads();
#ifdef CONFIG_I2C_DW_I2C1
	test_i2c1_start();
	test_i2c1_transfers();
	test_i2c1_interleaved();
#endif
	test_boundary();
	test_read_commands();
	assert(irq_masked == 0);
#ifdef CONFIG_I2C_DW_I2C1
	printf("i2c_dw: all tests passed (with i2c1)\n");
#else
	printf("i2c_dw: all tests passed\n");
#endif
	fflush(stdout);
	_exit(0);
}

int main(void)
{
	/* printf() must not allocate in a task (the port can switch threads
	 * while the C library holds its locks).
	 */
	static char out[1 << 12];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, APP_PRIO_SHELL,
			NULL) != pdPASS ||
	    xTaskCreate(hw_main, "hw", HW_STACK / sizeof(StackType_t), NULL, APP_PRIO_SHELL + 4,
			NULL) != pdPASS) {
		return 1;
	}
	vTaskStartScheduler();
	return 1;
}
