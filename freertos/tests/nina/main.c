/* SPDX-License-Identifier: MIT */
/* The host test of the boundaries of the NINA core on the Pico SDK and a
 * kernel, on that kernel's host port:
 * common/drivers/nina/port/pico_sdk/nina_bus_rp2040.c and the core's binding
 * of the kernel (common/drivers/nina/port/freertos/nina_os.c on the FreeRTOS
 * POSIX port, port/threadx/nina_os.c on ThreadX's Linux port, each with its
 * nina_bus_dma_os_impl.h; on the kernel and the Pico SDK alone: no
 * compatibility layer is linked) on fakes of the Pico SDK's DMA, GPIO, GPIO
 * interrupt, IRQ, reset, SPI and TIMER functions (pico_fake/, implemented
 * here: they record the calls and play the module), and the shared core
 * over both (common/drivers/nina/internal/nina_proto.c), as the board
 * builds them (common/drivers/nina/port/pico_sdk/nina.cmake).  What the test
 * needs of the kernel is the kernel's nina_test_os.c (nina_test_os.h: this
 * directory's for FreeRTOS, threadx/tests/nina's for ThreadX), which also
 * has the tests of what the binding's header leaves to the kernel (the
 * Wi-Fi thread's wake-up before its first wait, among others).
 * freertos/script/host_tests.sh runs it twice: with the board's 1 ms tick,
 * and with a 4 ms one (configTICK_RATE_HZ 250); threadx/script/host_tests.sh
 * once, at ThreadX's 1 ms; each time the kernel's tick count starts 5 s
 * before it wraps, and the fake TIMER 4.5 s before its low 32 bits wrap.
 *
 * - nina_bus_initialize(), which the core calls at its first request (a
 *   second call does nothing): the reset of SPI1, the three SPI pins as
 *   Zephyr's pinctrl sets them (in its order), 8 MHz, mode 0, 8 bits, MSB
 *   first, SSE, then CS as an output at its inactive (high) level.
 * - nina_bus_pin_configure() for each mode, as Zephyr's
 *   gpio_rpi_configure(); nina_bus_pin_set()/_get() with each line's
 *   polarity.
 * - "nina version" (0x37) through the core: the reset of the first request
 *   (RESET active, 10 ms, inactive, 750 ms, IRQ low and an input) with the
 *   binding's sleeps (the time rounded up to ticks, plus one tick), the
 *   frame on SPI, the answer; the second request does not reset again.
 * - A module that stops answering: the request ends with -ETIMEDOUT after
 *   the core's 2000 ms (the binding's clock), CS untouched.
 * - nina_os_lock(): recursive and with priority inheritance; the DNS lock
 *   is a separate mutex, and so is the Wi-Fi operations' lock
 *   (nina_os_wifi_lock(), with priority inheritance).
 * - A module that gets ready while a request waits: the request goes on
 *   within a few ticks of it.
 * - The sleeps' and waits' ticks: the time rounded up to ticks, plus one
 *   tick (0: a yield, or no wait); a sleep of UINT32_MAX ms (clamped to
 *   INT32_MAX) stays asleep; nina_os_uptime_ms() is the TIMER in ms,
 *   rounded down, and goes on across the wrap of the tick count and of the
 *   TIMER's low word.
 *
 * The long transfers by DMA (nina_bus_dma_rp2040.c, its messages through
 * log_fake/app_log.h), on a fake of the SDK's DMA functions and registers
 * whose engine (a task above the runner) moves the bytes a tick after the
 * TX channel's start and then raises DMA_IRQ_1's shared handlers, one of
 * them a stand-in for the I2C0 reads' with a channel of its own:
 * - nina_bus_initialize() claims two channels, adds the handler to
 *   DMA_IRQ_1 and enables the RX channel's interrupt there;
 * - the threshold: 63 bytes polled, 64 by DMA; the same bytes on the wire
 *   and back as by polling, with and without an RX buffer (the sink, no
 *   increment); 8-bit transfers, DREQ_SPI1_RX and _TX, SSPDR without
 *   increment, the RX channel started before the TX channel, SSPDMACR set
 *   only while the DMA moves the bytes (the polled fakes check it is 0),
 *   the calling task blocked meanwhile, the overrun flag cleared after;
 * - a wake-up of the task that is not the transfer's (on FreeRTOS a
 *   notification left on the NINA's index) does not end the wait early;
 *   the transfer's end between its look at the end flag and its wait does
 *   (nina_test_os_dma_window());
 * - each handler looks at and acknowledges its own channels only, also
 *   when both flags are raised at once;
 * - a transfer that never ends: -EIO after the wait (2 ms + LEN / 256 ms in
 *   ticks, plus one), the RX channel's interrupt off, both channels
 *   aborted, the flag the abort raises acknowledged, the interrupt on
 *   again, one rate-limited error logged; the next transfer works;
 * - a lost byte (the overrun flag, the RX channel short): -EIO, logged as
 *   an overrun; a bus error of the TX channel: -EIO, logged, its error
 *   flags cleared; the next transfers work, and log nothing;
 * - channels that do not stop when aborted: logged once, and the DMA is not
 *   used again;
 * - no second free channel at the start: the first given back, logged, and
 *   every transfer polled.
 *
 * As on the board, nina_os_ready_start() succeeds (the core's first
 * request calls it), and the core waits with the binding's waits on
 * READY's edge interrupt, on fakes of the SDK's GPIO interrupt functions
 * that latch the edges and run the raw handlers of IO_IRQ_BANK0.  A handler
 * for GPIO24 stands for the board's GPIO bank 0 backend (gpio_os_rp2040.c, the
 * LSM6DSOX's INT1).  So the test also checks:
 * - the binding adds one raw handler, for GPIO10 and GPIO2 only, beside the
 *   backend's for GPIO24 (no pin in two masks), and each handler looks at
 *   and acknowledges only its own pins, also when both have an edge;
 *   READY's edge is enabled only during a wait (falling towards ready,
 *   rising towards busy) and off after each request;
 * - a wait that times out reads READY once per 10 ms (the binding's
 *   slices), a wait that the interrupt ends early twice;
 * - a change right after the core's read of READY (before it waits) is not
 *   lost: the wait returns at once; so is one between the wait's start and
 *   its block (nina_test_os_ready_window());
 * - the arm drops a wake-up left from an earlier wait, and an edge latched
 *   while READY's event was off does not fire once it is on.
 *
 * The binding's waits for the socket events on the IRQ line (GPIO2):
 * nina_os_event_start() enables its rising edge; each edge counts one and
 * ends the waits of every task that waits (two at once here); a wait
 * without an edge ends after its time (0: at once); one whose count has
 * moved returns at once, and so does one with an edge between its join of
 * the waiters and its block (nina_test_os_event_window()); a wake-up of the
 * task that is not the wait's does not end it; a waiter that timed out is
 * not woken by a later edge.  A start
 * that fails cannot happen here (the binding's starts return 0); the
 * fallback after a failed start is common/tests/nina_wait.c's and
 * nina_proto.c's.
 */

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <time.h>

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/resets.h"
#include "hardware/spi.h"
#include "hardware/timer.h"

#include "nina_bus.h"
#include "nina_bus_dma_rp2040.h"
#include "nina_os.h"
#include "nina_proto.h"
#include "nina_test_os.h"

#define LOG_SIZE 4096
#define NUM_GPIO 30

#define GPIO_IRQ   2
#define GPIO_RESET 3
#define GPIO_CS    9
#define GPIO_READY 10
#define GPIO_INT1  24

/* The tick in ms, rounded up (the slack of the timing checks). */
#define TICK_MS ((int64_t)((1000 + nina_test_os_tick_hz() - 1) / nina_test_os_tick_hz()))

/* MS in ticks, rounded down (FreeRTOS's pdMS_TO_TICKS()). */
#define MS_TO_TICKS(ms) ((uint32_t)((uint64_t)(ms) * nina_test_os_tick_hz() / 1000u))

/* Where the fake TIMER starts (host_tests.sh: 4.5 s before its low word
 * wraps).
 */
#ifndef TEST_TIMER_START_US
#define TEST_TIMER_START_US 0
#endif

/* The fake TIMER: the host's monotonic clock from TEST_TIMER_START_US. */
static uint64_t host_start_us;

static uint64_t host_us(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000u;
}

uint64_t time_us_64(void)
{
	return host_us() - host_start_us + TEST_TIMER_START_US;
}

/* The calls -------------------------------------------------------------*/

static char calls[LOG_SIZE];
static bool recording = true;

static void record(const char *fmt, ...)
{
	size_t used = strlen(calls);
	va_list ap;

	if (!recording) {
		return;
	}
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

/* The messages of nina_bus_dma_rp2040.c (log_fake/app_log.h). */
static char logged[512];

void test_log(int level, const char *module, int ratelimited, const char *fmt, ...)
{
	size_t used = strlen(logged);
	va_list ap;

	assert(level == 1 && strcmp(module, "nina_bus") == 0);
	snprintf(logged + used, sizeof(logged) - used, "<err> %s%s: ", ratelimited ? "rl " : "",
		 module);
	used = strlen(logged);
	va_start(ap, fmt);
	vsnprintf(logged + used, sizeof(logged) - used, fmt, ap);
	va_end(ap);
	assert(strlen(logged) + 2 < sizeof(logged));
	strcat(logged, "\n");
}

static void expect_log(const char *expected)
{
	if (strcmp(logged, expected) != 0) {
		printf("expected log: %s\n     got log: %s\n", expected, logged);
		fflush(stdout);
		assert(0);
	}
	logged[0] = '\0';
}

/* The lines and the module -------------------------------------------------*/

struct pin {
	bool out;   /* direction */
	bool level; /* output level */
};

static struct pin pins[NUM_GPIO];

/* The module: answers while alive and out of reset.  READY is low (ready)
 * while CS is high, and the module answers CS going low by releasing READY
 * (pulled up).  The first CS period after ready takes the command, the next
 * one sends the answer.
 */
static bool module_alive = true;
static unsigned int module_resets;
static uint8_t command[64], answer[64];
static size_t command_len, answer_len, answer_pos;
static bool answering;
static unsigned int spi_calls, spi_bytes;

/* The DMA tests' module: answers each byte with the byte XOR 0x5a and keeps
 * what it received.
 */
static bool loopback;
static uint8_t wire[4096];
static size_t wire_len;

static bool module_running(void)
{
	return module_alive && pins[GPIO_RESET].out && pins[GPIO_RESET].level;
}

static bool cs_asserted(void)
{
	return pins[GPIO_CS].out && !pins[GPIO_CS].level;
}

/* The end of a CS period. */
static void cs_released(void)
{
	if (answering) {
		answering = false;
		command_len = 0;
		return;
	}
	if (command_len < 4 || command[0] != 0xe0 || command[1] != 0x37) {
		command_len = 0;
		return;
	}
	/* The version: START, the reply, one parameter, its length, END. */
	static const char version[] = "1.5.0";

	answer_len = 0;
	answer[answer_len++] = 0xe0;
	answer[answer_len++] = 0x37 | 0x80;
	answer[answer_len++] = 1;
	answer[answer_len++] = sizeof(version) - 1;
	memcpy(answer + answer_len, version, sizeof(version) - 1);
	answer_len += sizeof(version) - 1;
	answer[answer_len++] = 0xee;
	answer_pos = 0;
	answering = true;
}

static uint8_t exchange(uint8_t tx)
{
	assert(cs_asserted() && module_running());
	spi_bytes++;
	if (loopback) {
		assert(wire_len < sizeof(wire));
		wire[wire_len++] = tx;
		return tx ^ 0x5a;
	}
	if (answering) {
		return answer_pos < answer_len ? answer[answer_pos++] : 0xff;
	}
	assert(command_len < sizeof(command));
	command[command_len++] = tx;
	return 0xff;
}

/* READY's level and edges -------------------------------------------------*/

/* The physical level (high: busy or no module), counted reads of it, and a
 * test's hook: after that many more reads, the module comes alive just
 * after the read returns (as if it got ready between the core's read and
 * its wait).
 */
static bool ready_high = true;
static unsigned int ready_reads;
static int alive_after_reads = -1;

/* GPIO10's latched edges (INTR) and enabled events (PROC0_INTE), GPIO2's
 * and GPIO24's, the raw handlers of IO_IRQ_BANK0 and their masks, its
 * enable.
 */
static uint32_t ready_intr, ready_inte;
static uint32_t irq_intr, irq_inte;   /* GPIO2's */
static uint32_t int1_intr, int1_inte; /* GPIO24's (the backend's pin) */

#define MAX_HANDLERS 4

static irq_handler_t handlers[MAX_HANDLERS];
static uint32_t handler_masks[MAX_HANDLERS];
static unsigned int num_handlers;
static uint32_t raw_mask;
static bool bank_enabled;
static unsigned int bank_irqs;

/* The mask of the handler that runs (0: none), which the fakes check. */
static uint32_t running_mask;

static uint32_t ready_events(void)
{
	return ready_intr | (ready_high ? GPIO_IRQ_LEVEL_HIGH : GPIO_IRQ_LEVEL_LOW);
}

/* The NVIC: IO_IRQ_BANK0's raw handlers run in turn, as the SDK's shared
 * handlers, if an enabled event of a pin is pending.
 */
static void bank_handlers(void *arg)
{
	(void)arg;
	bank_irqs++;
	for (unsigned int i = 0; i < num_handlers; i++) {
		running_mask = handler_masks[i];
		handlers[i]();
		running_mask = 0;
	}
}

static void bank_irq(void)
{
	if (!bank_enabled || ((ready_events() & ready_inte) == 0 && (irq_intr & irq_inte) == 0 &&
			      (int1_intr & int1_inte) == 0)) {
		return;
	}
	nina_test_os_isr(bank_handlers, NULL);
}

/* A handler may only look at and acknowledge the pins of its mask. */
static void check_own_pin(uint gpio)
{
	assert(running_mask == 0 || (running_mask & (1u << gpio)) != 0);
}

/* The board's backend for GPIO24 (gpio_os_rp2040.c's handler). */
static unsigned int int1_handled;

static void backend_isr(void)
{
	if (gpio_get_irq_event_mask(GPIO_INT1) != 0) {
		gpio_acknowledge_irq(GPIO_INT1, 0xfu);
		int1_handled++;
	}
}

static bool module_running(void);
static bool cs_asserted(void);

/* After any change of the module or the lines: READY follows them, an edge
 * is latched and may interrupt.
 */
static void ready_update(void)
{
	bool high = !(module_running() && !cs_asserted());

	if (high == ready_high) {
		return;
	}
	ready_high = high;
	ready_intr |= high ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL;
	bank_irq();
}

static void set_module_alive(bool alive)
{
	module_alive = alive;
	ready_update();
}

/* Ticks at the reset sequence's edges (each valid once its flag is set). */
static uint32_t tick_reset_active, tick_reset_inactive, tick_irq_low;
static bool seen_reset_active, seen_reset_inactive, seen_irq_low;

/* The fakes of the Pico SDK (pico_fake/pico_nina_fake.h) ---------------------*/

void gpio_init(uint gpio)
{
	record("init(%u)", gpio);
	pins[gpio].out = false;
	pins[gpio].level = false;
}

void gpio_set_function(uint gpio, gpio_function_t fn)
{
	record("func(%u,%s)", gpio,
	       fn == GPIO_FUNC_SPI   ? "spi"
	       : fn == GPIO_FUNC_SIO ? "sio"
				     : "?");
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

static void line_changed(uint gpio, bool was_cs)
{
	if (gpio == GPIO_CS && was_cs && !cs_asserted()) {
		cs_released();
	}
	if (gpio == GPIO_RESET && pins[gpio].out) {
		if (!pins[gpio].level && !seen_reset_active) {
			tick_reset_active = nina_test_os_ticks();
			seen_reset_active = true;
		} else if (pins[gpio].level && seen_reset_active && !seen_reset_inactive) {
			tick_reset_inactive = nina_test_os_ticks();
			seen_reset_inactive = true;
			module_resets++;
		}
	}
	if (gpio == 2 && pins[gpio].out && !pins[gpio].level && !seen_irq_low) {
		tick_irq_low = nina_test_os_ticks();
		seen_irq_low = true;
	}
	ready_update();
}

void gpio_set_dir(uint gpio, bool out)
{
	bool was_cs = cs_asserted();

	record("dir(%u,%s)", gpio, out ? "out" : "in");
	pins[gpio].out = out;
	line_changed(gpio, was_cs);
}

void gpio_put(uint gpio, bool value)
{
	bool was_cs = cs_asserted();

	record("put(%u,%d)", gpio, value);
	pins[gpio].level = value;
	line_changed(gpio, was_cs);
}

bool gpio_get(uint gpio)
{
	bool high;

	assert(gpio == GPIO_READY && !pins[gpio].out);
	/* Pulled up unless the running module pulls it low. */
	high = !(module_running() && !cs_asserted());
	assert(high == ready_high);
	ready_reads++;
	if (alive_after_reads > 0 && --alive_after_reads == 0) {
		alive_after_reads = -1;
		set_module_alive(true);
	}
	return high;
}

uint32_t gpio_get_all(void)
{
	return ready_high ? 1u << GPIO_READY : 0;
}

/* A rising edge of the IRQ line (the module's socket event). */
static void irq_rise(void)
{
	irq_intr |= GPIO_IRQ_EDGE_RISE;
	bank_irq();
}

void nina_test_irq_rise(void)
{
	irq_rise();
}

void nina_test_module_alive(bool alive)
{
	set_module_alive(alive);
}

/* The SDK's: clear the events (INTR), then set or clear them in INTE.  Only
 * the binding calls it here (for its two pins), and not in its handler.
 */
void gpio_set_irq_enabled(uint gpio, uint32_t event_mask, bool enabled)
{
	assert(running_mask == 0);
	if (enabled) {
		/* The SDK asserts that the bank has a handler. */
		assert(num_handlers > 0);
	}
	record("irq(%u,%#x,%d)", gpio, (unsigned int)event_mask, enabled);
	if (gpio == GPIO_IRQ) {
		irq_intr &= ~(event_mask & 0xcu);
		irq_inte = enabled ? irq_inte | event_mask : irq_inte & ~event_mask;
		return;
	}
	assert(gpio == GPIO_READY);
	ready_intr &= ~(event_mask & 0xcu);
	if (enabled) {
		ready_inte |= event_mask;
	} else {
		ready_inte &= ~event_mask;
	}
}

uint32_t gpio_get_irq_event_mask(uint gpio)
{
	check_own_pin(gpio);
	if (gpio == GPIO_IRQ) {
		return irq_intr & irq_inte;
	}
	if (gpio == GPIO_INT1) {
		return int1_intr & int1_inte;
	}
	return gpio == GPIO_READY ? ready_events() & ready_inte : 0;
}

void gpio_acknowledge_irq(uint gpio, uint32_t event_mask)
{
	check_own_pin(gpio);
	if (gpio == GPIO_IRQ) {
		irq_intr &= ~(event_mask & 0xcu);
		return;
	}
	if (gpio == GPIO_INT1) {
		int1_intr &= ~(event_mask & 0xcu);
		return;
	}
	assert(gpio == GPIO_READY);
	ready_intr &= ~(event_mask & 0xcu);
}

/* The SDK's: hard_assert() that no pin is claimed twice, then a shared
 * handler of IO_IRQ_BANK0.
 */
void gpio_add_raw_irq_handler_masked(uint32_t gpio_mask, irq_handler_t handler)
{
	assert((raw_mask & gpio_mask) == 0 && num_handlers < MAX_HANDLERS);
	for (unsigned int i = 0; i < num_handlers; i++) {
		assert(handlers[i] != handler);
	}
	record("raw(%#x)", (unsigned int)gpio_mask);
	raw_mask |= gpio_mask;
	handler_masks[num_handlers] = gpio_mask;
	handlers[num_handlers++] = handler;
}

static bool dma_line_enabled;

void irq_set_enabled(uint num, bool enabled)
{
	if (num == DMA_IRQ_1) {
		record("dmairq(%u,%d)", num, enabled);
		dma_line_enabled = enabled;
		return;
	}
	assert(num == IO_IRQ_BANK0);
	bank_enabled = enabled;
}

/* DMA_IRQ_1's shared handlers, in the order added (the same order
 * priority).
 */
#define DMA_HANDLERS 4

static irq_handler_t dma_handlers[DMA_HANDLERS];
static unsigned int num_dma_handlers;

void irq_add_shared_handler(uint num, irq_handler_t handler, uint8_t order_priority)
{
	assert(num == DMA_IRQ_1 &&
	       order_priority == PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
	assert(num_dma_handlers < DMA_HANDLERS);
	for (unsigned int i = 0; i < num_dma_handlers; i++) {
		assert(dma_handlers[i] != handler);
	}
	record("shared(%u)", num);
	dma_handlers[num_dma_handlers++] = handler;
}

void reset_block_num(reset_num_t block_num)
{
	record("reset(%d)", (int)block_num);
}

void unreset_block_num_wait_blocking(reset_num_t block_num)
{
	record("unreset(%d)", (int)block_num);
}

struct spi_inst {
	spi_hw_t hw;
};

static struct spi_inst spi1_inst;
spi_inst_t *const fake_spi1 = &spi1_inst;

spi_hw_t *spi_get_hw(spi_inst_t *spi)
{
	return &spi->hw;
}

/* The DMA ---------------------------------------------------------------*/

#define DMA_ERROR_FLAGS (DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS)

/* The stand-in for the I2C0 reads: its channel and handler (main()). */
#define OTHER_CHANNEL 0

/* The NINA's channels: the next two free ones, TX first (test_initialize()). */
#define NINA_TX_CHANNEL 1
#define NINA_RX_CHANNEL 2

dma_hw_t fake_dma_hw;
static uint32_t dma_claimed;
static uint32_t dma_intr;    /* INTR, the channels' raw flags */
static uint32_t dma_inte[2]; /* INTE0, INTE1 */
static struct {
	volatile void *write;
	const volatile void *read;
	uint32_t count;
} dma_ch[NUM_DMA_CHANNELS];

/* What the engine does with the next transfer. */
enum dma_mode {
	DMA_RUN,   /* moves the bytes, both channels end */
	DMA_STALL, /* nothing: the channels stay busy until aborted */
	DMA_LOSE,  /* one byte lost: the overrun flag, the RX channel short */
	DMA_FAIL,  /* a read error of the TX channel */
	DMA_STUCK, /* as DMA_STALL, and an abort never ends */
};

static enum dma_mode dma_mode;
static bool dma_other_too; /* the engine raises the stand-in's flag too */
static int dma_rx_ch = -1, dma_tx_ch = -1;
static unsigned int dma_irqs, other_handled;
static uint32_t last_abort;

/* The channels of the handler that runs (0: none), which the fakes check. */
static uint32_t dma_handler_mask;

static void check_own_channel(uint channel)
{
	assert(dma_handler_mask == 0 || (dma_handler_mask & (1u << channel)) != 0);
}

static uint32_t nina_channels(void)
{
	return (1u << NINA_TX_CHANNEL) | (1u << NINA_RX_CHANNEL);
}

static void other_isr(void)
{
	if (dma_irqn_get_channel_status(1, OTHER_CHANNEL)) {
		dma_irqn_acknowledge_channel(1, OTHER_CHANNEL);
		other_handled++;
	}
}

/* DMA_IRQ_1, a level interrupt: its handlers run while an enabled flag is
 * raised; each takes its own, so none is left.
 */
static void dma_line_handlers(void *arg)
{
	(void)arg;
	dma_irqs++;
	for (unsigned int i = 0; i < num_dma_handlers; i++) {
		dma_handler_mask =
			dma_handlers[i] == other_isr ? 1u << OTHER_CHANNEL : nina_channels();
		dma_handlers[i]();
		dma_handler_mask = 0;
	}
	assert((dma_intr & dma_inte[1]) == 0);
}

static void dma_interrupt(void)
{
	if (!dma_line_enabled || (dma_intr & dma_inte[1]) == 0) {
		return;
	}
	nina_test_os_isr(dma_line_handlers, NULL);
}

int dma_claim_unused_channel(bool required)
{
	assert(!required);
	for (int ch = 0; ch < (int)NUM_DMA_CHANNELS; ch++) {
		if (!(dma_claimed & (1u << ch))) {
			dma_claimed |= 1u << ch;
			record("claim(%d)", ch);
			return ch;
		}
	}
	record("claim(-1)");
	return -1;
}

void dma_channel_unclaim(uint channel)
{
	assert(dma_claimed & (1u << channel));
	dma_claimed &= ~(1u << channel);
	record("unclaim(%u)", channel);
}

void dma_irqn_set_channel_enabled(uint irq_index, uint channel, bool enabled)
{
	assert(irq_index == 1 && dma_handler_mask == 0);
	record("inte(%u,%d)", channel, enabled);
	dma_inte[1] = enabled ? dma_inte[1] | (1u << channel) : dma_inte[1] & ~(1u << channel);
}

bool dma_irqn_get_channel_status(uint irq_index, uint channel)
{
	assert(irq_index == 1);
	check_own_channel(channel);
	return (dma_intr & dma_inte[1] & (1u << channel)) != 0;
}

/* A write of the channel's bit to INTS1: its raw flag cleared. */
void dma_irqn_acknowledge_channel(uint irq_index, uint channel)
{
	assert(irq_index == 1);
	check_own_channel(channel);
	dma_intr &= ~(1u << channel);
}

/* Also ends the aborts written to CHAN_ABORT (unless DMA_STUCK): the
 * channels idle, each with its flag raised (RP2040-E13).
 */
bool dma_channel_is_busy(uint channel)
{
	uint32_t abort = fake_dma_hw.abort;

	if (abort != 0 && abort != last_abort) {
		record("abort(%#x)", (unsigned int)abort);
		last_abort = abort;
	}
	if (abort != 0 && dma_mode != DMA_STUCK) {
		for (uint32_t ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
			if (abort & (1u << ch)) {
				fake_dma_hw.ch[ch].al1_ctrl &= ~DMA_CH0_CTRL_TRIG_BUSY_BITS;
				dma_intr |= 1u << ch;
			}
		}
		fake_dma_hw.abort = 0;
		last_abort = 0;
	}
	return (fake_dma_hw.ch[channel].al1_ctrl & DMA_CH0_CTRL_TRIG_BUSY_BITS) != 0;
}

void dma_channel_configure(uint channel, const dma_channel_config_t *config,
			   volatile void *write_addr, const volatile void *read_addr,
			   uint32_t encoded_transfer_count, bool trigger)
{
	uint32_t ctrl = config->ctrl;
	uint32_t dreq = (ctrl & DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) >> DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB;
	volatile uint32_t *al1 = &fake_dma_hw.ch[channel].al1_ctrl;

	assert(trigger && (dma_claimed & (1u << channel)) && !dma_channel_is_busy(channel));
	assert(((ctrl & DMA_CH0_CTRL_TRIG_DATA_SIZE_BITS) >> DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB) ==
	       DMA_SIZE_8);
	assert((ctrl & DMA_CH0_CTRL_TRIG_EN_BITS) && ((ctrl & DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS) >>
						      DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB) == channel);
	assert(encoded_transfer_count > 0);
	dma_ch[channel].write = write_addr;
	dma_ch[channel].read = read_addr;
	dma_ch[channel].count = encoded_transfer_count;
	/* The error flags stay until written with 1. */
	*al1 = ctrl | DMA_CH0_CTRL_TRIG_BUSY_BITS |
	       (*al1 & (DMA_ERROR_FLAGS | DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS));
	if (dreq == DREQ_SPI1_RX) {
		assert(read_addr == &spi1_inst.hw.dr && !(ctrl & DMA_CH0_CTRL_TRIG_INCR_READ_BITS));
		dma_rx_ch = (int)channel;
		dma_tx_ch = -1;
		record("rx(%u,%u,%d)", channel, (unsigned int)encoded_transfer_count,
		       (ctrl & DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS) != 0);
		return;
	}
	/* TX: after RX, with SSPDMACR's requests on; the engine moves the bytes. */
	assert(dreq == DREQ_SPI1_TX && write_addr == &spi1_inst.hw.dr);
	assert((ctrl & DMA_CH0_CTRL_TRIG_INCR_READ_BITS) &&
	       !(ctrl & DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS));
	assert(dma_rx_ch >= 0 && dma_channel_is_busy((uint)dma_rx_ch));
	assert(dma_ch[dma_rx_ch].count == encoded_transfer_count);
	assert(spi1_inst.hw.dmacr == (SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS));
	dma_tx_ch = (int)channel;
	record("tx(%u,%u)", channel, (unsigned int)encoded_transfer_count);
	nina_test_os_dma_kick();
}

static void dma_end(int ch)
{
	fake_dma_hw.ch[ch].al1_ctrl &= ~DMA_CH0_CTRL_TRIG_BUSY_BITS;
	dma_intr |= 1u << ch;
}

/* The bytes of the started transfer through the module, as the two
 * channels and SPI1 move them, while the caller sleeps (the kernel's DMA
 * engine calls it a tick after the TX channel's start).
 */
void nina_test_dma_move(void)
{
	int tx = dma_tx_ch, rx = dma_rx_ch;
	uint32_t n = dma_ch[tx].count;
	bool rx_incr = (fake_dma_hw.ch[rx].al1_ctrl & DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS) != 0;
	const volatile uint8_t *src = dma_ch[tx].read;
	volatile uint8_t *dst = dma_ch[rx].write;

	assert(nina_test_os_dma_caller_waits());
	assert(spi1_inst.hw.dmacr == (SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS));
	assert(spi1_inst.hw.cr1 & SPI_SSPCR1_SSE_BITS);
	if (dma_mode == DMA_STALL || dma_mode == DMA_STUCK) {
		return;
	}
	if (dma_mode == DMA_FAIL) {
		fake_dma_hw.ch[tx].al1_ctrl =
			(fake_dma_hw.ch[tx].al1_ctrl & ~DMA_CH0_CTRL_TRIG_BUSY_BITS) |
			DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS;
		dma_intr |= 1u << tx;
		dma_interrupt();
		return;
	}
	for (uint32_t i = 0; i < n; i++) {
		uint8_t r = exchange(src[i]);

		if (dma_mode == DMA_LOSE && i == n / 2) {
			spi1_inst.hw.ris |= SPI_SSPRIS_RORRIS_BITS;
			continue;
		}
		*dst = r;
		if (rx_incr) {
			dst++;
		}
	}
	dma_end(tx);
	if (dma_mode != DMA_LOSE) {
		dma_end(rx);
	}
	if (dma_other_too) {
		dma_intr |= 1u << OTHER_CHANNEL;
	}
	dma_interrupt();
}

void hw_set_bits(io_rw_32 *addr, uint32_t mask)
{
	if (addr == &spi1_inst.hw.cr1) {
		record("cr1|=%#x", (unsigned int)mask);
		*addr |= mask;
		return;
	}
	for (uint32_t i = 0; i < NUM_DMA_CHANNELS; i++) {
		if (addr == &fake_dma_hw.ch[i].al1_ctrl) {
			/* The error flags: write 1 to clear; AHB_ERROR is their OR. */
			uint32_t v = *addr & ~(mask & DMA_ERROR_FLAGS);

			assert(mask == DMA_ERROR_FLAGS);
			if (!(v & DMA_ERROR_FLAGS)) {
				v &= ~DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS;
			}
			*addr = v;
			return;
		}
	}
	assert(0);
}

/* The SDK's result for 8 MHz from clk_peri 125 MHz: CPSDVSR 2, SCR 7. */
uint spi_set_baudrate(spi_inst_t *spi, uint baudrate)
{
	assert(spi == spi1);
	record("baud(%u)", baudrate);
	return 7812500;
}

void spi_set_format(spi_inst_t *spi, uint data_bits, spi_cpol_t cpol, spi_cpha_t cpha,
		    spi_order_t order)
{
	assert(spi == spi1);
	record("format(%u,%d,%d,%s)", data_bits, (int)cpol, (int)cpha,
	       order == SPI_MSB_FIRST ? "msb" : "lsb");
}

int spi_write_read_blocking(spi_inst_t *spi, const uint8_t *src, uint8_t *dst, size_t len)
{
	assert(spi == spi1 && (spi1_inst.hw.cr1 & SPI_SSPCR1_SSE_BITS) && len > 0);
	assert(spi1_inst.hw.dmacr == 0);
	spi_calls++;
	for (size_t i = 0; i < len; i++) {
		dst[i] = exchange(src[i]);
	}
	return (int)len;
}

int spi_write_blocking(spi_inst_t *spi, const uint8_t *src, size_t len)
{
	assert(spi == spi1 && (spi1_inst.hw.cr1 & SPI_SSPCR1_SSE_BITS) && len > 0);
	assert(spi1_inst.hw.dmacr == 0);
	spi_calls++;
	for (size_t i = 0; i < len; i++) {
		(void)exchange(src[i]);
	}
	return (int)len;
}

/* The tests ----------------------------------------------------------------*/

/* The binding's sleeps and waits: the time rounded up to ticks, plus one
 * tick (none for 0).
 */
static uint32_t sleep_ticks(uint64_t us)
{
	uint64_t ticks = (us * nina_test_os_tick_hz() + 999999u) / 1000000u;

	return ticks == 0 ? 0 : (uint32_t)(ticks + 1);
}

/* A sleep of N ticks ends at the N-th tick; the test allows a few ticks of
 * the POSIX port's lateness.
 */
static void expect_ticks(uint32_t got, uint32_t expected)
{
	if (got < expected || got > expected + 5) {
		printf("ticks: expected %lu, got %lu\n", (unsigned long)expected,
		       (unsigned long)got);
		fflush(stdout);
		assert(0);
	}
}

static int64_t now_ms(void)
{
	return (int64_t)(time_us_64() / 1000u);
}

int64_t nina_test_now_ms(void)
{
	return now_ms();
}

static int version(char *out, size_t cap)
{
	size_t len;
	int ret = nina_proto_request(0x37, NULL, 0, false, false, (uint8_t *)out, cap - 1, &len);

	if (ret == 0) {
		out[len] = '\0';
	}
	return ret;
}

static void test_initialize(void)
{
	assert(nina_bus_initialize() == 0);
	expect_calls(
		"reset(17) unreset(17) "
		"init(14) func(14,spi) pulls(14,0,0) drive(14,4mA) slew(14,slow) schmitt(14,0) "
		"ie(14,0) oeover(14,0) outover(14,0) inover(14,0) irqover(14,0) "
		"init(11) func(11,spi) pulls(11,0,0) drive(11,4mA) slew(11,slow) schmitt(11,0) "
		"ie(11,0) oeover(11,0) outover(11,0) inover(11,0) irqover(11,0) "
		"init(8) func(8,spi) pulls(8,0,0) drive(8,4mA) slew(8,slow) schmitt(8,0) "
		"ie(8,1) oeover(8,0) outover(8,0) inover(8,0) irqover(8,0) "
		"baud(8000000) format(8,0,0,msb) cr1|=0x2 "
		"claim(1) claim(2) shared(12) dmairq(12,1) inte(2,1) "
		"pulls(9,0,0) func(9,sio) ie(9,0) put(9,1) dir(9,out) ");
	assert(spi1_inst.hw.dmacr == 0);
	assert(num_dma_handlers == 2 && dma_handlers[0] == other_isr && dma_line_enabled);
	assert(dma_inte[1] == ((1u << OTHER_CHANNEL) | (1u << 2)));
	assert(pins[GPIO_CS].out && pins[GPIO_CS].level);
	/* Again, as from the core's first request: nothing more. */
	assert(nina_bus_initialize() == 0);
	expect_calls("");
}

static void test_lines(void)
{
	/* gpio_rpi_configure() with the devicetree flags of each line. */
	assert(nina_bus_pin_configure(NINA_BUS_PIN_READY, NINA_BUS_PIN_INPUT) == 0);
	expect_calls("pulls(10,1,0) func(10,sio) dir(10,in) ");
	assert(nina_bus_pin_configure(NINA_BUS_PIN_IRQ, NINA_BUS_PIN_OUTPUT_HIGH) == 0);
	expect_calls("pulls(2,0,1) func(2,sio) ie(2,0) put(2,1) dir(2,out) ");
	assert(nina_bus_pin_configure(NINA_BUS_PIN_IRQ, NINA_BUS_PIN_OUTPUT_INACTIVE) == 0);
	expect_calls("pulls(2,0,1) func(2,sio) ie(2,0) put(2,0) dir(2,out) ");
	assert(nina_bus_pin_configure(NINA_BUS_PIN_CS, NINA_BUS_PIN_OUTPUT_ACTIVE) == 0);
	expect_calls("pulls(9,0,0) func(9,sio) ie(9,0) put(9,0) dir(9,out) ");
	assert(nina_bus_pin_configure(NINA_BUS_PIN_CS, NINA_BUS_PIN_OUTPUT_HIGH) == 0);
	expect_calls("pulls(9,0,0) func(9,sio) ie(9,0) put(9,1) dir(9,out) ");
	assert(nina_bus_pin_configure(4, NINA_BUS_PIN_INPUT) == -EINVAL);
	assert(nina_bus_pin_configure(NINA_BUS_PIN_CS, 4) == -EINVAL);
	expect_calls("");

	/* Logical values: CS, READY and RESET are active low, IRQ high. */
	assert(nina_bus_pin_set(NINA_BUS_PIN_CS, 1) == 0);
	assert(nina_bus_pin_set(NINA_BUS_PIN_CS, 0) == 0);
	assert(nina_bus_pin_set(NINA_BUS_PIN_IRQ, 1) == 0);
	assert(nina_bus_pin_set(NINA_BUS_PIN_IRQ, 0) == 0);
	expect_calls("put(9,0) put(9,1) put(2,1) put(2,0) ");
	/* The module is held in reset (GPIO3 not driven yet): READY high. */
	assert(nina_bus_pin_get(NINA_BUS_PIN_READY) == 0);
	/* The lines as nina_proto.c finds them before its first request. */
	pins[2].out = false;
	pins[GPIO_READY].out = false;
}

static void test_version(void)
{
	char v[16];
	unsigned int resets = module_resets;

	/* test_lines() drove IRQ low; the edges of the reset are measured from
	 * here.
	 */
	seen_reset_active = seen_reset_inactive = seen_irq_low = false;
	recording = false;
	assert(version(v, sizeof(v)) == 0);
	assert(strcmp(v, "1.5.0") == 0);
	assert(module_resets == resets + 1 && seen_irq_low);
	/* RESET active for 10 ms, then 750 ms before IRQ goes low. */
	expect_ticks(tick_reset_inactive - tick_reset_active, sleep_ticks(10000));
	expect_ticks(tick_irq_low - tick_reset_inactive, sleep_ticks(750000));
	/* 0x37 with no parameter is START, cmd, 0, END: four bytes, sent in
	 * single transfers; the answer is ten bytes read in single and bulk
	 * transfers.
	 */
	assert(command_len == 0 && !answering);
	assert(!cs_asserted() && pins[GPIO_CS].level);

	/* The second request does not reset the module again. */
	assert(version(v, sizeof(v)) == 0);
	assert(strcmp(v, "1.5.0") == 0);
	assert(module_resets == resets + 1);
	recording = true;
	calls[0] = '\0';
	printf("nina: version \"%s\", %u SPI calls, %u bytes\n", v, spi_calls, spi_bytes);
}

static void test_no_answer(void)
{
	char v[16];
	unsigned int calls_before = spi_calls;
	unsigned int reads;
	int64_t start;
	int64_t took;

	set_module_alive(false);
	recording = false;
	reads = ready_reads;
	start = now_ms();
	assert(version(v, sizeof(v)) == -ETIMEDOUT);
	took = now_ms() - start;
	reads = ready_reads - reads;
	recording = true;
	calls[0] = '\0';
	/* wait_ready(1, 2000): waits of 10 ms (the binding's
	 * READY_WAIT_SLICE_MS) until the clock passes the deadline, READY read
	 * before the first and after each.
	 */
	assert(took >= 2000 && took < 2000 + 50 + 4 * TICK_MS);
	assert(spi_calls == calls_before && !cs_asserted());
	assert(reads >= 2 && reads <= 2000 / 10 + 1 && ready_inte == 0);
	set_module_alive(true);
	printf("nina: no answer, -ETIMEDOUT after %lld ms, %u reads of READY\n", (long long)took,
	       reads);
}

static void wake_helper(void *arg)
{
	nina_test_os_delay(MS_TO_TICKS((uintptr_t)arg));
	set_module_alive(true);
	nina_test_os_thread_exit();
}

/* The module gets ready while the request waits for it (a helper task
 * above the runner brings it up after 50 ms): the request goes on within a
 * few ticks.  The first wait reads READY once before its first 10 ms slice
 * and once after each (the last ended by the interrupt), the other three
 * once each.
 */
static void test_late_ready(void)
{
	char v[16];
	unsigned int reads;
	int64_t start;
	int64_t took;

	set_module_alive(false);
	recording = false;
	(void)nina_test_os_thread_start(wake_helper, (void *)50, 1);
	reads = ready_reads;
	start = now_ms();
	assert(version(v, sizeof(v)) == 0);
	took = now_ms() - start;
	reads = ready_reads - reads;
	recording = true;
	calls[0] = '\0';
	assert(strcmp(v, "1.5.0") == 0);
	assert(took >= 50 - TICK_MS && took < 50 + 20 + 2 * TICK_MS);
	assert(reads >= 5 && reads <= 1 + 50 / 10 + 1 + 3 && ready_inte == 0);
	printf("nina: ready after %lld ms, %u reads of READY\n", (long long)took, reads);
}

/* The binding's raw handler claims GPIO10 and GPIO2, beside the backend's
 * for GPIO24; READY's edge is off between waits.
 */
static void test_irq_setup(void)
{
	assert(num_handlers == 2 && handlers[0] == backend_isr);
	assert(handler_masks[0] == (1u << GPIO_INT1));
	assert(handler_masks[1] == ((1u << GPIO_READY) | (1u << GPIO_IRQ)));
	assert(raw_mask == ((1u << GPIO_INT1) | (1u << GPIO_READY) | (1u << GPIO_IRQ)));
	assert(irq_inte == 0);
	assert(bank_enabled && ready_inte == 0);
}

/* The module gets ready right after the core's first read of READY, before
 * it waits: the edge was enabled before the read, so the interrupt gives
 * the semaphore and the wait returns at once instead of after 2000 ms.
 */
static void test_ready_between_read_and_wait(void)
{
	char v[16];
	unsigned int reads;
	unsigned int irqs;
	int64_t start;
	int64_t took;

	set_module_alive(false);
	recording = false;
	alive_after_reads = 1;
	reads = ready_reads;
	irqs = bank_irqs;
	start = now_ms();
	assert(version(v, sizeof(v)) == 0);
	took = now_ms() - start;
	reads = ready_reads - reads;
	irqs = bank_irqs - irqs;
	recording = true;
	calls[0] = '\0';
	assert(strcmp(v, "1.5.0") == 0 && alive_after_reads == -1);
	assert(took < 20 && reads == 5 && irqs >= 1 && ready_inte == 0);
	printf("nina: ready between the read and the wait: %lld ms, %u interrupts\n",
	       (long long)took, irqs);
}

/* The READY waits' calls and ticks, directly (as the core makes them, one
 * waiter): the arm's events, the slice, a wake-up left from before and an
 * edge latched while the event was off.
 */
static void test_ready_waits(void)
{
	uint32_t t;
	unsigned int irqs;

	set_module_alive(false);
	assert(ready_high);

	/* Towards ready: all events off, then the falling edge; the disarm. */
	nina_os_ready_arm(1);
	expect_calls("irq(10,0xf,0) irq(10,0x4,1) ");
	nina_os_ready_disarm();
	expect_calls("irq(10,0xf,0) ");
	/* Towards busy: the rising edge. */
	nina_os_ready_arm(0);
	expect_calls("irq(10,0xf,0) irq(10,0x8,1) ");
	nina_os_ready_disarm();
	calls[0] = '\0';

	/* No edge: the time rounded up to ticks, plus one; at most 10 ms; 0 at
	 * once.
	 */
	nina_os_ready_arm(1);
	t = nina_test_os_ticks();
	nina_os_ready_wait(3);
	expect_ticks(nina_test_os_ticks() - t, sleep_ticks(3000));
	t = nina_test_os_ticks();
	nina_os_ready_wait(25);
	expect_ticks(nina_test_os_ticks() - t, sleep_ticks(10000));
	t = nina_test_os_ticks();
	nina_os_ready_wait(0);
	assert(nina_test_os_ticks() - t <= 1);

	/* An edge before the wait: the wait returns at once. */
	irqs = bank_irqs;
	set_module_alive(true);
	assert(bank_irqs == irqs + 1);
	t = nina_test_os_ticks();
	nina_os_ready_wait(10);
	assert(nina_test_os_ticks() - t <= 1);

	/* An edge between the wait's start and its block (the kernel's
	 * nina_test_os_ready_window()): the wait returns at once.
	 */
	set_module_alive(false); /* rising: its event is off */
	irqs = bank_irqs;
	nina_test_os_ready_window();
	t = nina_test_os_ticks();
	nina_os_ready_wait(10);
	assert(nina_test_os_ticks() - t <= 1 && bank_irqs == irqs + 1 && !ready_high);

	/* A wake-up left from before the arm is dropped: the edge gave the
	 * semaphore, nobody took it before the disarm.
	 */
	set_module_alive(false); /* rising: its event is off */
	set_module_alive(true);  /* falling: gives */
	nina_os_ready_disarm();
	/* Edges latched while the events are off. */
	set_module_alive(false);
	set_module_alive(true);
	set_module_alive(false);
	assert((ready_intr & (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)) != 0);
	irqs = bank_irqs;
	nina_os_ready_arm(1);
	/* The latched falling edge was cleared by the enable: no interrupt. */
	assert(bank_irqs == irqs && (ready_intr & GPIO_IRQ_EDGE_FALL) == 0);
	t = nina_test_os_ticks();
	nina_os_ready_wait(10);
	expect_ticks(nina_test_os_ticks() - t, sleep_ticks(10000));
	nina_os_ready_disarm();
	set_module_alive(true);
	ready_intr = 0;
	calls[0] = '\0';
	printf("nina: READY waits: arm, slice, stale wake-up, latched edge\n");
}

/* A task that waits for the next edge (ARG: its slot) and notes when. */
static volatile int64_t event_woke[3];
static volatile uint32_t event_mark;
static volatile uint32_t event_wait_ms;

static void event_helper(void *arg)
{
	int slot = (int)(intptr_t)arg;

	nina_os_event_wait(event_mark, event_wait_ms);
	event_woke[slot] = now_ms();
	nina_test_os_thread_exit();
}

static void start_event_helper(int slot, int above)
{
	(void)nina_test_os_thread_start(event_helper, (void *)(intptr_t)slot, above);
}

static void test_events(void)
{
	uint32_t seq;
	int64_t start, took;
	uint32_t t;

	/* The start: GPIO2's events off, then its rising edge, nothing else of
	 * the line; the handler is not added again.
	 */
	assert(irq_inte == 0);
	assert(nina_os_event_start() == 0);
	expect_calls("irq(2,0xf,0) irq(2,0x8,1) ");
	assert(irq_inte == GPIO_IRQ_EDGE_RISE && ready_inte == 0 && num_handlers == 2);

	/* Two tasks wait; one edge 30 ms later ends both waits (a
	 * broadcast), and counts one.
	 */
	seq = nina_os_event_seq();
	event_mark = seq;
	event_wait_ms = 1000;
	event_woke[0] = event_woke[1] = -1;
	start_event_helper(0, 1);
	start_event_helper(1, 1);
	nina_test_os_delay(MS_TO_TICKS(30));
	assert(event_woke[0] == -1 && event_woke[1] == -1);
	start = now_ms();
	irq_rise();
	nina_test_os_delay(MS_TO_TICKS(5));
	assert(nina_os_event_seq() == seq + 1);
	assert(event_woke[0] >= start && event_woke[0] < start + 5);
	assert(event_woke[1] >= start && event_woke[1] < start + 5);

	/* The count moved: at once. */
	start = now_ms();
	nina_os_event_wait(seq, 100);
	assert(now_ms() - start <= 1);

	/* No edge: the time (100 ms), rounded up to ticks, plus one. */
	seq = nina_os_event_seq();
	t = nina_test_os_ticks();
	start = now_ms();
	nina_os_event_wait(seq, 100);
	took = now_ms() - start;
	expect_ticks(nina_test_os_ticks() - t, sleep_ticks(100000));
	/* The POSIX port's ticks lag its clock by a few ms. */
	assert(nina_os_event_seq() == seq && took >= 100 && took <= 120 + 2 * TICK_MS);

	/* 0 ms: at once. */
	t = nina_test_os_ticks();
	nina_os_event_wait(seq, 0);
	assert(nina_test_os_ticks() - t <= 1);

	/* An edge before the wait (after the caller read the count): at once. */
	seq = nina_os_event_seq();
	irq_rise();
	start = now_ms();
	nina_os_event_wait(seq, 100);
	assert(now_ms() - start <= 1 && nina_os_event_seq() == seq + 1);

	/* An edge between the wait's join and its block (the kernel's
	 * nina_test_os_event_window()): at once.
	 */
	seq = nina_os_event_seq();
	nina_test_os_event_window();
	t = nina_test_os_ticks();
	nina_os_event_wait(seq, 1000);
	assert(nina_test_os_ticks() - t <= 1 && nina_os_event_seq() == seq + 1);

	/* A wake-up that is not the wait's (on FreeRTOS a notification left on
	 * the binding's index by a waiter that timed out as it was woken) does
	 * not end the next wait early.
	 */
	seq = nina_os_event_seq();
	nina_test_os_stray_wake();
	t = nina_test_os_ticks();
	nina_os_event_wait(seq, 20);
	expect_ticks(nina_test_os_ticks() - t, sleep_ticks(20000));

	/* A waiter that timed out has left the list: a later edge wakes only
	 * the one that still waits.
	 */
	seq = nina_os_event_seq();
	event_mark = seq;
	event_wait_ms = 10;
	event_woke[2] = -1;
	start_event_helper(2, 1);
	nina_test_os_delay(MS_TO_TICKS(40));
	assert(event_woke[2] != -1);
	event_wait_ms = 1000;
	event_woke[0] = -1;
	start_event_helper(0, 1);
	start = now_ms();
	irq_rise();
	assert(event_woke[0] >= start && nina_os_event_seq() == seq + 1);
	printf("nina: socket events, two waiters woken by one edge\n");
}

/* The backend's pin and the binding's interrupt at once: each handler
 * acknowledges its own pins only (check_own_pin()).
 */
static void test_shared_bank(void)
{
	uint32_t seq = nina_os_event_seq();
	unsigned int handled = int1_handled;

	int1_inte = GPIO_IRQ_EDGE_RISE;
	int1_intr = GPIO_IRQ_EDGE_RISE;
	irq_rise();
	assert(int1_handled == handled + 1 && int1_intr == 0);
	assert(nina_os_event_seq() == seq + 1 && irq_intr == 0);
	/* GPIO24 alone: the binding's handler leaves it and counts nothing. */
	int1_intr = GPIO_IRQ_EDGE_RISE;
	bank_irq();
	assert(int1_handled == handled + 2 && nina_os_event_seq() == seq + 1);
	int1_inte = 0;
	printf("nina: IO_IRQ_BANK0 shared with the backend's handler\n");
}

/* The DMA tests' data and checks. */
static uint8_t dma_tx[3000], dma_rx[3000];

/* LEN bytes of dma_tx through nina_bus_transceive() (into dma_rx unless
 * NO_RX): RET, and on success the bytes on the wire and back.  SPI1's end
 * state either way: no DMA requests, the overrun flag cleared (written 1).
 */
static void dma_xfer(size_t len, bool no_rx, int ret)
{
	wire_len = 0;
	memset(dma_rx, 0, sizeof(dma_rx));
	spi1_inst.hw.icr = 0;
	assert(nina_bus_transceive(dma_tx, no_rx ? NULL : dma_rx, len) == ret);
	assert(spi1_inst.hw.dmacr == 0);
	if (ret != 0) {
		return;
	}
	assert(wire_len == len && memcmp(wire, dma_tx, len) == 0);
	for (size_t i = 0; i < len && !no_rx; i++) {
		assert(dma_rx[i] == (dma_tx[i] ^ 0x5a));
	}
}

/* The calls of one transfer by DMA of LEN bytes (RX with increment unless
 * NO_RX), then AFTER.
 */
static void expect_dma_calls(size_t len, bool no_rx, const char *after)
{
	char expected[256];

	snprintf(expected, sizeof(expected), "rx(%u,%u,%d) tx(%u,%u) %s", NINA_RX_CHANNEL,
		 (unsigned int)len, !no_rx, NINA_TX_CHANNEL, (unsigned int)len, after);
	expect_calls(expected);
	assert(spi1_inst.hw.icr == SPI_SSPICR_RORIC_BITS);
}

/* A transfer of LEN bytes by polling: one SDK call, no DMA. */
static void expect_polled(size_t len)
{
	unsigned int calls_before = spi_calls;

	dma_xfer(len, false, 0);
	assert(spi_calls == calls_before + 1);
	expect_calls("");
}

/* The wait's ticks for LEN bytes: 2 ms + LEN / 256 ms in ticks, plus one. */
static uint32_t dma_wait_ticks(size_t len)
{
	return (uint32_t)(((2 + len / 256) * nina_test_os_tick_hz() + 999) / 1000 + 1);
}

static void test_dma(void)
{
	uint32_t rx_bit = 1u << NINA_RX_CHANNEL;
	unsigned int calls_before, handled;
	char expected[128];
	uint32_t t, took;

	for (size_t i = 0; i < sizeof(dma_tx); i++) {
		dma_tx[i] = (uint8_t)(i * 7 + 3);
	}
	loopback = true;
	assert(nina_bus_select() == 0);
	calls[0] = '\0';
	logged[0] = '\0';

	/* The threshold: 63 bytes polled, 64 by DMA. */
	expect_polled(63);
	calls_before = spi_calls;
	dma_xfer(64, false, 0);
	assert(spi_calls == calls_before);
	expect_dma_calls(64, false, "");
	assert((dma_intr & nina_channels()) == 0 && (dma_inte[1] & rx_bit));

	/* No RX buffer: the received bytes into the sink, without increment. */
	dma_xfer(2400, true, 0);
	expect_dma_calls(2400, true, "");

	/* A wake-up that is not the transfer's (on FreeRTOS a notification left
	 * on the NINA's index): the wait still ends with the transfer.
	 */
	nina_test_os_stray_wake();
	dma_xfer(200, false, 0);
	expect_dma_calls(200, false, "");

	/* The transfer's end between its look at the end flag and its wait
	 * (the kernel's nina_test_os_dma_window()): the wait returns at once.
	 */
	t = nina_test_os_ticks();
	nina_test_os_dma_window();
	dma_xfer(300, false, 0);
	expect_dma_calls(300, false, "");
	assert(nina_test_os_ticks() - t <= 1);

	/* The stand-in's flag alone: its handler takes it, the NINA's leaves
	 * it (check_own_channel()); both at once: each takes its own.
	 */
	handled = other_handled;
	dma_intr |= 1u << OTHER_CHANNEL;
	dma_interrupt();
	assert(other_handled == handled + 1 && dma_intr == 0);
	dma_other_too = true;
	dma_xfer(500, false, 0);
	dma_other_too = false;
	expect_dma_calls(500, false, "");
	assert(other_handled == handled + 2 && dma_intr == 0);
	expect_log("");

	/* A transfer that never ends: -EIO after the wait, both channels
	 * aborted, the flag of the abort taken, the interrupt on again, one
	 * error logged; the next transfer works.
	 */
	dma_mode = DMA_STALL;
	t = nina_test_os_ticks();
	dma_xfer(2400, false, -EIO);
	took = nina_test_os_ticks() - t;
	snprintf(expected, sizeof(expected), "inte(%u,0) abort(%#x) inte(%u,1) ", NINA_RX_CHANNEL,
		 (unsigned int)nina_channels(), NINA_RX_CHANNEL);
	expect_dma_calls(2400, false, expected);
	assert(took >= dma_wait_ticks(2400) && took <= dma_wait_ticks(2400) + 5);
	assert(!dma_channel_is_busy(NINA_TX_CHANNEL) && !dma_channel_is_busy(NINA_RX_CHANNEL));
	assert((dma_intr & nina_channels()) == 0 && (dma_inte[1] & rx_bit));
	{
		char line[96];

		snprintf(line, sizeof(line),
			 "<err> rl nina_bus: SPI1 DMA did not end in %u ticks (2400 bytes)\n",
			 (unsigned int)dma_wait_ticks(2400));
		expect_log(line);
	}
	dma_mode = DMA_RUN;
	dma_xfer(2400, false, 0);
	expect_dma_calls(2400, false, "");
	expect_log("");

	/* A lost byte: the RX channel short, the overrun flag; -EIO. */
	dma_mode = DMA_LOSE;
	dma_xfer(300, false, -EIO);
	expect_dma_calls(300, false, expected);
	expect_log("<err> rl nina_bus: SPI1 RX overrun by DMA (300 bytes)\n");
	spi1_inst.hw.ris = 0;
	dma_mode = DMA_RUN;
	dma_xfer(300, false, 0);
	expect_dma_calls(300, false, "");

	/* A bus error of the TX channel: -EIO, its error flags cleared. */
	dma_mode = DMA_FAIL;
	dma_xfer(300, false, -EIO);
	expect_dma_calls(300, false, expected);
	assert(!(fake_dma_hw.ch[NINA_TX_CHANNEL].al1_ctrl &
		 (DMA_ERROR_FLAGS | DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS)));
	expect_log("<err> rl nina_bus: SPI1 DMA bus error (300 bytes)\n");
	dma_mode = DMA_RUN;
	dma_xfer(300, false, 0);
	expect_dma_calls(300, false, "");
	expect_log("");

	/* Channels that do not stop: the abort is given up after its bound,
	 * the interrupt stays off, and the DMA is not used again.
	 */
	dma_mode = DMA_STUCK;
	dma_xfer(300, false, -EIO);
	snprintf(expected, sizeof(expected), "inte(%u,0) abort(%#x) ", NINA_RX_CHANNEL,
		 (unsigned int)nina_channels());
	expect_dma_calls(300, false, expected);
	assert(!(dma_inte[1] & rx_bit));
	{
		char lines[192];

		snprintf(lines, sizeof(lines),
			 "<err> nina_bus: SPI1 DMA channels 1 and 2 do not stop: every transfer "
			 "polled\n"
			 "<err> rl nina_bus: SPI1 DMA did not end in %u ticks (300 bytes)\n",
			 (unsigned int)dma_wait_ticks(300));
		expect_log(lines);
	}
	expect_polled(3000);

	/* A start without a second free channel: the first given back, an
	 * error, every transfer polled.
	 */
	dma_claimed = ((1u << NUM_DMA_CHANNELS) - 1) & ~(1u << 5);
	nina_bus_dma_initialize();
	expect_calls("claim(5) claim(-1) unclaim(5) ");
	expect_log("<err> nina_bus: No DMA channels for SPI1: every transfer polled\n");
	assert(dma_claimed == (((1u << NUM_DMA_CHANNELS) - 1) & ~(1u << 5)));
	expect_polled(3000);
	expect_log("");

	nina_bus_deselect();
	loopback = false;
	calls[0] = '\0';
	printf("nina: SPI DMA: threshold, sink, end state, shared DMA_IRQ_1, timeout, lost byte, "
	       "bus error, stuck abort, no channels, errors logged (%u interrupts)\n",
	       dma_irqs);
}

static void test_sleeps(void)
{
	uint32_t t;

	t = nina_test_os_ticks();
	nina_os_sleep_us(100);
	expect_ticks(nina_test_os_ticks() - t, sleep_ticks(100));
	t = nina_test_os_ticks();
	nina_os_sleep_us(1500);
	expect_ticks(nina_test_os_ticks() - t, sleep_ticks(1500));
	t = nina_test_os_ticks();
	nina_os_sleep_ms(10);
	expect_ticks(nina_test_os_ticks() - t, sleep_ticks(10000));
	t = nina_test_os_ticks();
	nina_os_sleep_ms(7);
	expect_ticks(nina_test_os_ticks() - t, sleep_ticks(7000));
	/* 0: a yield, no tick. */
	t = nina_test_os_ticks();
	nina_os_sleep_us(0);
	nina_os_sleep_ms(0);
	assert(nina_test_os_ticks() - t <= 1);
}

/* A task that sleeps for UINT32_MAX ms (clamped to INT32_MAX ms: about
 * 24.8 days); the test checks that it is still asleep, then deletes it.
 */
static volatile bool long_sleep_done;

static void long_sleep_helper(void *arg)
{
	(void)arg;
	nina_os_sleep_ms(UINT32_MAX);
	long_sleep_done = true;
	nina_test_os_thread_exit();
}

static void test_long_sleep(void)
{
	nina_test_os_thread_t task;

	long_sleep_done = false;
	task = nina_test_os_thread_start(long_sleep_helper, NULL, 1);
	nina_test_os_delay(MS_TO_TICKS(20));
	assert(!long_sleep_done && nina_test_os_thread_blocked(task));
	nina_test_os_thread_delete(task);
}

/* The clock: the TIMER in ms, rounded down, also across the wrap of the
 * tick count (host_tests.sh starts it 5 s before) and of the TIMER's low
 * word (4.5 s): sampled at every tick until both have wrapped.
 */
static void test_uptime(void)
{
	uint32_t last_tick = nina_test_os_ticks();
	int64_t last = nina_os_uptime_ms();
	bool tick_wrapped = false;
	int64_t start = last;

	while (!tick_wrapped || time_us_64() <= (UINT64_C(1) << 32) + 1000) {
		uint64_t before = time_us_64();
		int64_t ms = nina_os_uptime_ms();
		uint64_t after = time_us_64();
		uint32_t tick = nina_test_os_ticks();

		assert(ms >= (int64_t)(before / 1000) && ms <= (int64_t)(after / 1000));
		assert(ms >= last);
		if (tick < last_tick) {
			tick_wrapped = true;
		}
		last_tick = tick;
		last = ms;
		assert(last - start < 10000);
		nina_test_os_delay(1);
	}
	printf("nina: uptime %lld ms across the wrap of the tick and of the TIMER's low word\n",
	       (long long)last);
}

static volatile int helper_state;

/* Takes the Wi-Fi operations' lock. */
static void wifi_lock_helper(void *arg)
{
	(void)arg;
	helper_state = 1;
	nina_os_wifi_lock();
	nina_os_wifi_unlock();
	helper_state = 4;
	nina_test_os_thread_exit();
}

/* Takes nina_lock, then the DNS lock (ARG non-NULL) or nina_lock only. */
static void lock_helper(void *arg)
{
	helper_state = 1;
	assert(nina_os_lock() == 0);
	helper_state = 2;
	nina_os_unlock();
	if (arg != NULL) {
		nina_os_dns_lock();
		helper_state = 3;
		nina_os_dns_unlock();
	}
	helper_state = 4;
	nina_test_os_thread_exit();
}

static void start_helper(int above, void *arg)
{
	helper_state = 0;
	(void)nina_test_os_thread_start(lock_helper, arg, above);
}

static void test_locks(void)
{
	int mine = nina_test_os_priority();

	/* Recursive: the owner takes it twice.  The helper (higher) runs at once
	 * and waits; this task has its priority until the last unlock, which
	 * lets the helper run.
	 */
	assert(nina_os_lock() == 0);
	assert(nina_os_lock() == 0);
	start_helper(mine + 2, NULL);
	assert(helper_state == 1);
	assert(nina_test_os_priority() == mine + 2);
	nina_os_unlock();
	assert(helper_state == 1);
	assert(nina_test_os_priority() == mine + 2);
	nina_os_unlock();
	assert(helper_state == 4);
	assert(nina_test_os_priority() == mine);

	/* An unlock by a task that does not hold it changes nothing. */
	nina_os_unlock();
	assert(nina_os_lock() == 0);
	nina_os_unlock();

	/* The DNS lock is another mutex: held here, it does not keep the
	 * helper from nina_lock, only from itself.
	 */
	nina_os_dns_lock();
	start_helper(mine + 2, "dns");
	assert(helper_state == 2);
	assert(nina_test_os_priority() == mine + 2);
	nina_os_dns_unlock();
	assert(helper_state == 4);
	assert(nina_test_os_priority() == mine);

	/* The Wi-Fi lock is a third one, with priority inheritance. */
	nina_os_wifi_lock();
	start_helper(mine + 2, "dns");
	assert(helper_state == 4);
	helper_state = 0;
	(void)nina_test_os_thread_start(wifi_lock_helper, NULL, mine + 2);
	assert(helper_state == 1);
	assert(nina_test_os_priority() == mine + 2);
	nina_os_wifi_unlock();
	assert(helper_state == 4);
	assert(nina_test_os_priority() == mine);
}

void nina_test_run(void)
{
	test_initialize();
	test_lines();
	test_version();
	test_no_answer();
	test_late_ready();
	test_irq_setup();
	test_ready_between_read_and_wait();
	test_ready_waits();
	test_events();
	test_shared_bank();
	test_dma();
	test_sleeps();
	test_long_sleep();
	test_locks();
	nina_test_os_tests();
	test_uptime();
	printf("nina: tick %u Hz\n", nina_test_os_tick_hz());
	fflush(stdout);
}

void nina_test_setup(void)
{
	host_start_us = host_us();
	/* The backend's handler for GPIO24, as gpio_os_rp2040.c adds it when the
	 * LSM6DSOX's interrupt is first enabled.
	 */
	gpio_add_raw_irq_handler_masked(1u << GPIO_INT1, backend_isr);
	bank_enabled = true;
	/* The stand-in for the I2C0 reads on DMA_IRQ_1, with its channel, as
	 * i2c_dw_dma_os_rp2040.c at the start.
	 */
	dma_claimed = 1u << OTHER_CHANNEL;
	irq_add_shared_handler(DMA_IRQ_1, other_isr,
			       PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
	dma_line_enabled = true;
	dma_inte[1] = 1u << OTHER_CHANNEL;
	calls[0] = '\0';
}
