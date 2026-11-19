/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS build's boundaries of the NINA core on the
 * FreeRTOS POSIX port: freertos/drivers/nina/nina_bus.c on fakes of the
 * Pico SDK's GPIO, reset and SPI functions (pico_fake/, implemented here:
 * they record the calls and play the module), freertos/drivers/nina/
 * nina_os.c on the FreeRTOS part of the compatibility layer, and the
 * shared core over both (common/drivers/nina/nina_spi.c), as the board
 * builds them (freertos/drivers/nina/nina.cmake).
 *
 * - Before nina_bus_initialize(): a request fails with -ENODEV and touches
 *   no line (the board's start makes no request).
 * - nina_bus_initialize(): the reset of SPI1, the three SPI pins as
 *   Zephyr's pinctrl sets them (in its order), 8 MHz, mode 0, 8 bits, MSB
 *   first, SSE, then CS as an output at its inactive (high) level.
 * - nina_pin_configure() for each mode, as Zephyr's
 *   gpio_rpi_configure(); nina_pin_set()/_get() with each line's
 *   polarity.
 * - "nina version" (0x37) through the core: the reset of the first request
 *   (RESET active, 10 ms, inactive, 750 ms, IRQ low and an input) with the
 *   sleeps of the layer (the time rounded up to ticks, plus one tick), the
 *   frame on SPI, the answer; the second request does not reset again.
 * - A module that stops answering: the request ends with -ETIMEDOUT after
 *   the core's 2000 ms (the layer's clock), CS untouched.
 * - nina_os_lock(): recursive and with priority inheritance; the DNS lock
 *   is a separate mutex.
 * - A module that gets ready while a request waits: the request goes on
 *   within a few ticks of it.
 *
 * As on the board, nina_os_ready_start() succeeds (the core's first
 * request calls it), and the core waits with nina_os.c's waits on READY's
 * edge interrupt, through drivers/compat/rp2040/nano_gpio.c
 * and the layer's gpio0 (common/compat/zephyr_gpio.c), on fakes of the
 * SDK's GPIO interrupt functions that latch READY's edges and run the
 * bank's handler as an interrupt handler.  So the test also checks:
 * - the bank's handler claims GPIO24 and GPIO10; READY's edge is enabled
 *   only during a wait (falling towards ready, rising towards busy) and
 *   off after each request;
 * - a wait that times out reads READY once per 10 ms (nina_os.c's
 *   slices), a wait that the interrupt ends early twice;
 * - a change right after the core's read of READY (before it waits) is not
 *   lost: the wait returns at once.
 *
 * nina_os.c's waits for the socket
 * events on the IRQ line (GPIO2, also claimed by the bank's handler):
 * nina_os_event_start() enables its rising edge; each edge counts one and
 * ends the waits of every task that waits (two at once here); a wait
 * without an edge ends after its time; one whose count has moved returns
 * at once.
 */

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/kernel.h>

#include "hardware/gpio.h"
#include "hardware/resets.h"
#include "hardware/spi.h"

#include "nina_bus.h"
#include "nina_os.h"
#include "nina_proto.h"
#include "priorities.h"
#include "zephyr_compat.h"

#define RUNNER_STACK 16384
#define HELPER_STACK 16384
#define LOG_SIZE     4096
#define NUM_GPIO     30

#define GPIO_IRQ   2
#define GPIO_RESET 3
#define GPIO_CS    9
#define GPIO_READY 10

_Thread_local int z_host_isr_depth;

/* tick.c's vApplicationTickHook(), counted as an interrupt handler. */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
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

/* GPIO10's latched edges (INTR) and enabled events (PROC0_INTE), the bank's
 * handler and its mask, IO_IRQ_BANK0's enable.
 */
static uint32_t ready_intr, ready_inte;
static uint32_t irq_intr, irq_inte; /* GPIO2's */
static uint32_t raw_mask;
static irq_handler_t bank_handler;
static bool bank_enabled;
static unsigned int ready_irqs;

static uint32_t ready_events(void)
{
	return ready_intr | (ready_high ? GPIO_IRQ_LEVEL_HIGH : GPIO_IRQ_LEVEL_LOW);
}

/* The NVIC: the bank's handler runs, as an interrupt handler, if an enabled
 * event of GPIO10 is pending.
 */
static void bank_irq(void)
{
	if (bank_enabled && bank_handler != NULL &&
	    ((ready_events() & ready_inte) != 0 || (irq_intr & irq_inte) != 0)) {
		ready_irqs++;
		z_host_isr_depth++;
		bank_handler();
		z_host_isr_depth--;
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
static TickType_t tick_reset_active, tick_reset_inactive, tick_irq_low;
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
			tick_reset_active = xTaskGetTickCount();
			seen_reset_active = true;
		} else if (pins[gpio].level && seen_reset_active && !seen_reset_inactive) {
			tick_reset_inactive = xTaskGetTickCount();
			seen_reset_inactive = true;
			module_resets++;
		}
	}
	if (gpio == 2 && pins[gpio].out && !pins[gpio].level && !seen_irq_low) {
		tick_irq_low = xTaskGetTickCount();
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

/* The SDK's: clear the events (INTR), then set or clear them in INTE. */
void gpio_set_irq_enabled(uint gpio, uint32_t event_mask, bool enabled)
{
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
	if (gpio == GPIO_IRQ) {
		return irq_intr & irq_inte;
	}
	return gpio == GPIO_READY ? ready_events() & ready_inte : 0;
}

void gpio_acknowledge_irq(uint gpio, uint32_t event_mask)
{
	if (gpio == GPIO_IRQ) {
		irq_intr &= ~(event_mask & 0xcu);
		return;
	}
	assert(gpio == GPIO_READY);
	ready_intr &= ~(event_mask & 0xcu);
}

void gpio_add_raw_irq_handler_masked(uint32_t gpio_mask, irq_handler_t handler)
{
	assert(raw_mask == 0 && bank_handler == NULL);
	raw_mask = gpio_mask;
	bank_handler = handler;
}

void irq_set_enabled(uint num, bool enabled)
{
	assert(num == IO_IRQ_BANK0);
	bank_enabled = enabled;
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

void hw_set_bits(volatile uint32_t *addr, uint32_t mask)
{
	assert(addr == &spi1_inst.hw.cr1);
	record("cr1|=%#x", (unsigned int)mask);
	*addr |= mask;
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
	spi_calls++;
	for (size_t i = 0; i < len; i++) {
		dst[i] = exchange(src[i]);
	}
	return (int)len;
}

int spi_write_blocking(spi_inst_t *spi, const uint8_t *src, size_t len)
{
	assert(spi == spi1 && (spi1_inst.hw.cr1 & SPI_SSPCR1_SSE_BITS) && len > 0);
	spi_calls++;
	for (size_t i = 0; i < len; i++) {
		(void)exchange(src[i]);
	}
	return (int)len;
}

/* The tests ----------------------------------------------------------------*/

/* The layer's sleeps: the time rounded up to ticks, plus one tick. */
static TickType_t sleep_ticks(uint32_t us)
{
	return (TickType_t)((us + Z_USEC_PER_TICK - 1) / Z_USEC_PER_TICK + 1);
}

/* A sleep of N ticks ends at the N-th tick; the test allows a few ticks of
 * the POSIX port's lateness.
 */
static void expect_ticks(TickType_t got, TickType_t expected)
{
	if (got < expected || got > expected + 5) {
		printf("ticks: expected %lu, got %lu\n", (unsigned long)expected,
		       (unsigned long)got);
		fflush(stdout);
		assert(0);
	}
}

static int version(char *out, size_t cap)
{
	size_t len;
	int ret = nina_request(0x37, NULL, 0, false, false, (uint8_t *)out, cap - 1, &len);

	if (ret == 0) {
		out[len] = '\0';
	}
	return ret;
}

static void test_before_initialize(void)
{
	char v[16];

	assert(!nina_bus_ready());
	assert(version(v, sizeof(v)) == -ENODEV);
	expect_calls("");
}

static void test_initialize(void)
{
	assert(nina_bus_initialize() == 0);
	assert(nina_bus_ready());
	expect_calls(
		"reset(17) unreset(17) "
		"init(14) func(14,spi) pulls(14,0,0) drive(14,4mA) slew(14,slow) schmitt(14,0) "
		"ie(14,0) oeover(14,0) outover(14,0) inover(14,0) irqover(14,0) "
		"init(11) func(11,spi) pulls(11,0,0) drive(11,4mA) slew(11,slow) schmitt(11,0) "
		"ie(11,0) oeover(11,0) outover(11,0) inover(11,0) irqover(11,0) "
		"init(8) func(8,spi) pulls(8,0,0) drive(8,4mA) slew(8,slow) schmitt(8,0) "
		"ie(8,1) oeover(8,0) outover(8,0) inover(8,0) irqover(8,0) "
		"baud(8000000) format(8,0,0,msb) cr1|=0x2 "
		"pulls(9,0,0) func(9,sio) ie(9,0) put(9,1) dir(9,out) ");
	assert(spi1_inst.hw.dmacr == 0);
	assert(pins[GPIO_CS].out && pins[GPIO_CS].level);
}

static void test_lines(void)
{
	/* gpio_rpi_configure() with the devicetree flags of each line. */
	assert(nina_pin_configure(NINA_PIN_READY, NINA_PIN_INPUT) == 0);
	expect_calls("pulls(10,1,0) func(10,sio) dir(10,in) ");
	assert(nina_pin_configure(NINA_PIN_IRQ, NINA_PIN_OUTPUT_HIGH) == 0);
	expect_calls("pulls(2,0,1) func(2,sio) ie(2,0) put(2,1) dir(2,out) ");
	assert(nina_pin_configure(NINA_PIN_IRQ, NINA_PIN_OUTPUT_INACTIVE) == 0);
	expect_calls("pulls(2,0,1) func(2,sio) ie(2,0) put(2,0) dir(2,out) ");
	assert(nina_pin_configure(NINA_PIN_CS, NINA_PIN_OUTPUT_ACTIVE) == 0);
	expect_calls("pulls(9,0,0) func(9,sio) ie(9,0) put(9,0) dir(9,out) ");
	assert(nina_pin_configure(NINA_PIN_CS, NINA_PIN_OUTPUT_HIGH) == 0);
	expect_calls("pulls(9,0,0) func(9,sio) ie(9,0) put(9,1) dir(9,out) ");
	assert(nina_pin_configure(4, NINA_PIN_INPUT) == -EINVAL);
	assert(nina_pin_configure(NINA_PIN_CS, 4) == -EINVAL);
	expect_calls("");

	/* Logical values: CS, READY and RESET are active low, IRQ high. */
	assert(nina_pin_set(NINA_PIN_CS, 1) == 0);
	assert(nina_pin_set(NINA_PIN_CS, 0) == 0);
	assert(nina_pin_set(NINA_PIN_IRQ, 1) == 0);
	assert(nina_pin_set(NINA_PIN_IRQ, 0) == 0);
	expect_calls("put(9,0) put(9,1) put(2,1) put(2,0) ");
	/* The module is held in reset (GPIO3 not driven yet): READY high. */
	assert(nina_pin_get(NINA_PIN_READY) == 0);
	/* The lines as nina_spi.c finds them before its first request. */
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
	start = k_uptime_get();
	assert(version(v, sizeof(v)) == -ETIMEDOUT);
	took = k_uptime_get() - start;
	reads = ready_reads - reads;
	recording = true;
	calls[0] = '\0';
	/* wait_ready(1, 2000): waits of 10 ms (nina_os.c READY_WAIT_SLICE_MS)
	 * until the clock passes the deadline, READY read before the first and
	 * after each.
	 */
	assert(took >= 2000 && took < 2000 + 50);
	assert(spi_calls == calls_before && !cs_asserted());
	assert(reads >= 2 && reads <= 2000 / 10 + 1 && ready_inte == 0);
	set_module_alive(true);
	printf("nina: no answer, -ETIMEDOUT after %lld ms, %u reads of READY\n", (long long)took,
	       reads);
}

static void wake_helper(void *arg)
{
	vTaskDelay(pdMS_TO_TICKS((uintptr_t)arg));
	set_module_alive(true);
	vTaskDelete(NULL);
}

/* The module gets ready while the request waits for it (a helper task
 * above the runner brings it up after 50 ms): the request goes on within a
 * few ticks.  The first wait reads READY once before
 * its first 10 ms slice and once after each (the last ended by the
 * interrupt), the other three once each.
 */
static void test_late_ready(void)
{
	char v[16];
	unsigned int reads;
	int64_t start;
	int64_t took;

	set_module_alive(false);
	recording = false;
	assert(xTaskCreate(wake_helper, "wake", HELPER_STACK / sizeof(StackType_t), (void *)50,
			   uxTaskPriorityGet(NULL) + 1, NULL) == pdPASS);
	reads = ready_reads;
	start = k_uptime_get();
	assert(version(v, sizeof(v)) == 0);
	took = k_uptime_get() - start;
	reads = ready_reads - reads;
	recording = true;
	calls[0] = '\0';
	assert(strcmp(v, "1.5.0") == 0);
	assert(took >= 50 && took < 50 + 20);
	assert(reads >= 5 && reads <= 1 + 50 / 10 + 1 + 3 && ready_inte == 0);
	printf("nina: ready after %lld ms, %u reads of READY\n", (long long)took, reads);
}

/* The bank's handler claims the three pins; READY's edge is off between
 * waits. */
static void test_irq_setup(void)
{
	assert(raw_mask == ((1u << 24) | (1u << GPIO_READY) | (1u << GPIO_IRQ)));
	assert(irq_inte == 0);
	assert(bank_enabled && bank_handler != NULL && ready_inte == 0);
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
	irqs = ready_irqs;
	start = k_uptime_get();
	assert(version(v, sizeof(v)) == 0);
	took = k_uptime_get() - start;
	reads = ready_reads - reads;
	irqs = ready_irqs - irqs;
	recording = true;
	calls[0] = '\0';
	assert(strcmp(v, "1.5.0") == 0 && alive_after_reads == -1);
	assert(took < 20 && reads == 5 && irqs >= 1 && ready_inte == 0);
	printf("nina: ready between the read and the wait: %lld ms, %u interrupts\n",
	       (long long)took, irqs);
}

/* A task that waits for the next edge (ARG: its slot) and notes when. */
static volatile int64_t event_woke[2];
static volatile uint32_t event_mark;

static void event_helper(void *arg)
{
	int slot = (int)(intptr_t)arg;

	nina_os_event_wait(event_mark, 1000);
	event_woke[slot] = k_uptime_get();
	vTaskDelete(NULL);
}

static void test_events(void)
{
	UBaseType_t mine = uxTaskPriorityGet(NULL);
	uint32_t seq;
	int64_t start, took;

	/* The start: GPIO2's rising edge, nothing else of the line. */
	assert(irq_inte == 0);
	assert(nina_os_event_start() == 0);
	assert(irq_inte == GPIO_IRQ_EDGE_RISE && ready_inte == 0);

	/* Two tasks wait; one edge 30 ms later ends both waits (a
	 * broadcast), and counts one. */
	seq = nina_os_event_seq();
	event_mark = seq;
	event_woke[0] = event_woke[1] = -1;
	assert(xTaskCreate(event_helper, "ev0", HELPER_STACK / sizeof(StackType_t), (void *)0,
			   mine + 1, NULL) == pdPASS);
	assert(xTaskCreate(event_helper, "ev1", HELPER_STACK / sizeof(StackType_t), (void *)1,
			   mine + 1, NULL) == pdPASS);
	k_msleep(30);
	assert(event_woke[0] == -1 && event_woke[1] == -1);
	start = k_uptime_get();
	irq_rise();
	k_msleep(5);
	assert(nina_os_event_seq() == seq + 1);
	assert(event_woke[0] >= start && event_woke[0] < start + 5);
	assert(event_woke[1] >= start && event_woke[1] < start + 5);

	/* The count moved: at once. */
	start = k_uptime_get();
	nina_os_event_wait(seq, 100);
	assert(k_uptime_get() - start <= 1);

	/* No edge: the time (100 ms), the count left by the edge taken on the
	 * way. */
	seq = nina_os_event_seq();
	start = k_uptime_get();
	nina_os_event_wait(seq, 100);
	took = k_uptime_get() - start;
	/* The POSIX port's ticks lag its clock by a few ms. */
	assert(nina_os_event_seq() == seq && took >= 100 && took <= 120);

	/* An edge before the wait (after the caller read the count): at once. */
	seq = nina_os_event_seq();
	irq_rise();
	start = k_uptime_get();
	nina_os_event_wait(seq, 100);
	assert(k_uptime_get() - start <= 1 && nina_os_event_seq() == seq + 1);
	printf("nina: socket events, two waiters woken by one edge\n");
}

static void test_sleeps(void)
{
	TickType_t t;

	t = xTaskGetTickCount();
	nina_os_sleep_us(100);
	expect_ticks(xTaskGetTickCount() - t, sleep_ticks(100));
	t = xTaskGetTickCount();
	nina_os_sleep_ms(10);
	expect_ticks(xTaskGetTickCount() - t, sleep_ticks(10000));
	assert(nina_os_uptime_ms() == k_uptime_get());
}

static volatile int helper_state;

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
	vTaskDelete(NULL);
}

static void start_helper(UBaseType_t priority, void *arg)
{
	helper_state = 0;
	assert(xTaskCreate(lock_helper, "helper", HELPER_STACK / sizeof(StackType_t), arg, priority,
			   NULL) == pdPASS);
}

static void test_locks(void)
{
	UBaseType_t mine = uxTaskPriorityGet(NULL);

	/* Recursive: the owner takes it twice.  The helper (higher) runs at once
	 * and waits; this task has its priority until the last unlock, which
	 * lets the helper run.
	 */
	assert(nina_os_lock() == 0);
	assert(nina_os_lock() == 0);
	start_helper(mine + 2, NULL);
	assert(helper_state == 1);
	assert(uxTaskPriorityGet(NULL) == mine + 2);
	nina_os_unlock();
	assert(helper_state == 1);
	assert(uxTaskPriorityGet(NULL) == mine + 2);
	nina_os_unlock();
	assert(helper_state == 4);
	assert(uxTaskPriorityGet(NULL) == mine);

	/* The DNS lock is another mutex: held here, it does not keep the
	 * helper from nina_lock, only from itself.
	 */
	nina_os_dns_lock();
	start_helper(mine + 2, "dns");
	assert(helper_state == 2);
	assert(uxTaskPriorityGet(NULL) == mine + 2);
	nina_os_dns_unlock();
	assert(helper_state == 4);
	assert(uxTaskPriorityGet(NULL) == mine);
}

static void runner(void *arg)
{
	(void)arg;
	test_before_initialize();
	test_initialize();
	test_lines();
	test_version();
	test_no_answer();
	test_late_ready();
	test_irq_setup();
	test_ready_between_read_and_wait();
	test_events();
	test_sleeps();
	test_locks();
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
	if (zephyr_compat_initialize() != 0) {
		return 2;
	}
	/* The shell's priority: "nina version" runs in the console's task. */
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, NANO_PRIO_SHELL,
			NULL) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
