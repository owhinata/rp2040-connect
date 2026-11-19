/* SPDX-License-Identifier: MIT */
/* The host test of the FreeRTOS build's GPIO boundary on the FreeRTOS POSIX
 * port: freertos/drivers/compat/rp2040/nano_gpio.c (common/compat/
 * nano_gpio.h on the Pico SDK's hardware_gpio) on fakes of the SDK's GPIO
 * and IRQ functions (pico_fake/, implemented here: they record the calls and
 * play GPIO bank 0's levels, latched edges and interrupt enables), under
 * the layer's gpio0 device (common/compat/zephyr_gpio.c, with
 * zephyr_device.c and the gpio0 start of zephyr_compat_initialize()) and the
 * LSM6DSO's FIFO interrupt (common/drivers/lsm6dso/lsm6dso_fifo_irq.c), with
 * the prelude and CONFIG_NANO_LSM6DSO_STREAM, as
 * freertos/drivers/lsm6dso/lsm6dso.cmake builds them.
 *
 * - The start: gpio0 is not ready before zephyr_compat_initialize() and is
 *   after it, which touches no register and adds no handler.
 * - gpio_pin_configure() of an input, as Zephyr's gpio_rpi_configure():
 *   gpio_set_pulls() (none, up, down), gpio_set_function(SIO),
 *   gpio_set_dir(in); outputs and disconnected pins refused (-ENOTSUP), a
 *   pin above 29 too (-EINVAL), with no call.  gpio_pin_get(): the level,
 *   inverted for GPIO_ACTIVE_LOW.
 * - gpio_pin_interrupt_configure(), as Zephyr's
 *   gpio_rpi_pin_interrupt_configure(): all events off (and cleared), then
 *   the mode's events on (rising, falling, both edges, the active level,
 *   inverted for an active-low pin); the bank's handler is added for
 *   GPIO24 and IO_IRQ_BANK0 enabled at the first enable only; another pin's
 *   interrupt (also the NINA's READY and IRQ line, which have their own
 *   handler) is refused (-ENOTSUP) but may be disabled; an edge latched
 *   while its event was off does not fire once it is on.
 * - The bank's interrupt, as Zephyr's gpio_rpi_isr(): GPIO24's events are
 *   acknowledged, then its callbacks run in the interrupt handler with the
 *   pin; a pin the handler does not own is left alone; callbacks added and
 *   removed (twice: -EINVAL).
 * - lsm6dso_fifo_irq: the start configures GPIO24 and its rising edge; an
 *   edge submits the work, which runs the handler in the system work
 *   queue's task; a level already high at the start is not lost; a handler
 *   that fails turns the interrupt off; the stop turns it off and removes
 *   the callback.
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
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "pico_gpio_fake.h"

#include "lsm6dso_fifo_irq.h"
#include "priorities.h"
#include "zephyr_compat.h"

#define RUNNER_STACK 16384
#define LOG_SIZE     4096
#define INT1         24
#define OTHER        5
/* The NINA's READY and IRQ line: the NINA binding's own handler's pins. */
#define READY        10
#define NINA_IRQ     2

_Thread_local int z_host_isr_depth;

/* tick.c's vApplicationTickHook(), counted as an interrupt handler. */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

/* The other devices of zephyr_device.c's list (no init function: ready at
 * once is not needed here).
 */
DEVICE_DT_DEFINE(i2c0, "i2c@40044000", NULL, NULL, NULL, NULL);
DEVICE_DT_DEFINE(lsm6dsox, "lsm6dsox@6a", NULL, NULL, NULL, NULL);

/* The calls ------------------------------------------------------------------*/

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

/* GPIO bank 0 ------------------------------------------------------------------*/

static uint32_t level;   /* GPIO_IN */
static uint32_t intr[4]; /* INTR: the latched edges, 4 bits per pin */
static uint32_t inte[4]; /* PROC0_INTE */
static uint32_t raw_mask;
static irq_handler_t bank_handler;
static bool bank_enabled;

#define SHIFT(pin) (4 * ((pin) % 8))

static uint32_t pin_events(uint pin)
{
	uint32_t events = (intr[pin / 8] >> SHIFT(pin)) & 0xcu;

	events |= (level & (1u << pin)) != 0 ? GPIO_IRQ_LEVEL_HIGH : GPIO_IRQ_LEVEL_LOW;
	return events;
}

static void clear_edges(uint pin, uint32_t events)
{
	intr[pin / 8] &= ~((events & 0xcu) << SHIFT(pin));
}

void gpio_set_pulls(uint gpio, bool up, bool down)
{
	record("pulls(%u,%d,%d)", gpio, up, down);
}

void gpio_set_function(uint gpio, gpio_function_t fn)
{
	record("func(%u,%d)", gpio, fn);
}

void gpio_set_dir(uint gpio, bool out)
{
	record("dir(%u,%d)", gpio, out);
}

uint32_t gpio_get_all(void)
{
	return level;
}

/* The SDK's: clear the events (INTR), then set or clear them in INTE. */
void gpio_set_irq_enabled(uint gpio, uint32_t event_mask, bool enabled)
{
	record("irq(%u,0x%x,%d)", gpio, (unsigned int)event_mask, enabled);
	clear_edges(gpio, event_mask);
	if (enabled) {
		inte[gpio / 8] |= event_mask << SHIFT(gpio);
	} else {
		inte[gpio / 8] &= ~(event_mask << SHIFT(gpio));
	}
}

uint32_t gpio_get_irq_event_mask(uint gpio)
{
	return pin_events(gpio) & ((inte[gpio / 8] >> SHIFT(gpio)) & 0xfu);
}

void gpio_acknowledge_irq(uint gpio, uint32_t event_mask)
{
	record("ack(%u,0x%x)", gpio, (unsigned int)event_mask);
	clear_edges(gpio, event_mask);
}

void gpio_add_raw_irq_handler_masked(uint32_t gpio_mask, irq_handler_t handler)
{
	record("raw(0x%x)", (unsigned int)gpio_mask);
	assert((raw_mask & gpio_mask) == 0); /* the SDK's hard_assert() */
	raw_mask |= gpio_mask;
	bank_handler = handler;
}

void irq_set_enabled(uint num, bool enabled)
{
	record("irq_en(%u,%d)", num, enabled);
	assert(num == IO_IRQ_BANK0);
	bank_enabled = enabled;
}

static bool bank_pending(void)
{
	for (uint pin = 0; pin < 30; pin++) {
		if (gpio_get_irq_event_mask(pin) != 0) {
			return true;
		}
	}
	return false;
}

/* The NVIC: the bank's handler runs once, as an interrupt handler, if an
 * enabled event is pending.
 */
static void bank_irq(void)
{
	if (bank_enabled && bank_handler != NULL && bank_pending()) {
		z_host_isr_depth++;
		bank_handler();
		z_host_isr_depth--;
	}
}

static void set_level(uint pin, bool high)
{
	bool was = (level & (1u << pin)) != 0;

	if (high && !was) {
		intr[pin / 8] |= (uint32_t)GPIO_IRQ_EDGE_RISE << SHIFT(pin);
	} else if (!high && was) {
		intr[pin / 8] |= (uint32_t)GPIO_IRQ_EDGE_FALL << SHIFT(pin);
	}
	level = high ? level | (1u << pin) : level & ~(1u << pin);
	bank_irq();
}

/* The tests ---------------------------------------------------------------------*/

static const struct device *const gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));

static void test_start(void)
{
	assert(!device_is_ready(gpio0));
	assert(zephyr_compat_initialize() == 0);
	assert(device_is_ready(gpio0));
	expect_calls("");
	assert(bank_handler == NULL && !bank_enabled);
}

static void test_configure(void)
{
	assert(gpio_pin_configure(gpio0, INT1, GPIO_INPUT) == 0);
	expect_calls("pulls(24,0,0) func(24,5) dir(24,0) ");
	assert(gpio_pin_configure(gpio0, OTHER, GPIO_INPUT | GPIO_PULL_UP) == 0);
	expect_calls("pulls(5,1,0) func(5,5) dir(5,0) ");
	assert(gpio_pin_configure(gpio0, OTHER, GPIO_INPUT | GPIO_PULL_DOWN) == 0);
	expect_calls("pulls(5,0,1) func(5,5) dir(5,0) ");
	assert(gpio_pin_configure(gpio0, OTHER, GPIO_OUTPUT) == -ENOTSUP);
	assert(gpio_pin_configure(gpio0, OTHER, GPIO_DISCONNECTED) == -ENOTSUP);
	assert(gpio_pin_configure(gpio0, 30, GPIO_INPUT) == -EINVAL);
	expect_calls("");

	level = 1u << INT1;
	assert(gpio_pin_get(gpio0, INT1) == 1);
	assert(gpio_pin_get(gpio0, OTHER) == 0);
	assert(gpio_pin_get(gpio0, 30) == -EINVAL);
	assert(gpio_pin_configure(gpio0, INT1, GPIO_INPUT | GPIO_ACTIVE_LOW) == 0);
	assert(gpio_pin_get(gpio0, INT1) == 0);
	level = 0;
	assert(gpio_pin_get(gpio0, INT1) == 1);
	assert(gpio_pin_configure(gpio0, INT1, GPIO_INPUT) == 0);
	assert(gpio_pin_get(gpio0, INT1) == 0);
	expect_calls("pulls(24,0,0) func(24,5) dir(24,0) pulls(24,0,0) func(24,5) dir(24,0) ");
}

struct counted {
	struct gpio_callback cb;
	const char *name;
	int calls;
};

static void counted_handler(const struct device *port, struct gpio_callback *cb,
			    gpio_port_pins_t pins)
{
	struct counted *c = CONTAINER_OF(cb, struct counted, cb);

	assert(port == gpio0);
	c->calls++;
	record("%s(0x%x,isr=%d)", c->name, (unsigned int)pins, z_host_isr_depth > 0);
}

static void test_interrupts(void)
{
	struct counted a = {.name = "a"}, b = {.name = "b"};

	/* Only GPIO24 may interrupt; any pin may be turned off. */
	assert(gpio_pin_interrupt_configure(gpio0, OTHER, GPIO_INT_EDGE_RISING) == -ENOTSUP);
	assert(gpio_pin_interrupt_configure(gpio0, READY, GPIO_INT_EDGE_RISING) == -ENOTSUP);
	assert(gpio_pin_interrupt_configure(gpio0, NINA_IRQ, GPIO_INT_EDGE_RISING) == -ENOTSUP);
	assert(gpio_pin_interrupt_configure(gpio0, 30, GPIO_INT_DISABLE) == -EINVAL);
	expect_calls("");
	assert(gpio_pin_interrupt_configure(gpio0, OTHER, GPIO_INT_DISABLE) == 0);
	expect_calls("irq(5,0xf,0) ");

	/* The first enable adds the handler for the board's pins. */
	assert(gpio_pin_interrupt_configure(gpio0, INT1, GPIO_INT_EDGE_TO_ACTIVE) == 0);
	expect_calls("raw(0x1000000) irq_en(13,1) irq(24,0xf,0) irq(24,0x8,1) ");
	assert(gpio_pin_interrupt_configure(gpio0, INT1, GPIO_INT_EDGE_BOTH) == 0);
	expect_calls("irq(24,0xf,0) irq(24,0xc,1) ");
	assert(gpio_pin_interrupt_configure(gpio0, INT1, GPIO_INT_EDGE_FALLING) == 0);
	expect_calls("irq(24,0xf,0) irq(24,0x4,1) ");
	assert(gpio_pin_interrupt_configure(gpio0, INT1, GPIO_INT_LEVEL_ACTIVE) == 0);
	expect_calls("irq(24,0xf,0) irq(24,0x2,1) ");
	assert(gpio_pin_configure(gpio0, INT1, GPIO_INPUT | GPIO_ACTIVE_LOW) == 0);
	assert(gpio_pin_interrupt_configure(gpio0, INT1, GPIO_INT_EDGE_TO_ACTIVE) == 0);
	assert(gpio_pin_interrupt_configure(gpio0, INT1, GPIO_INT_LEVEL_ACTIVE) == 0);
	expect_calls("pulls(24,0,0) func(24,5) dir(24,0) irq(24,0xf,0) irq(24,0x4,1) "
		     "irq(24,0xf,0) irq(24,0x1,1) ");
	assert(gpio_pin_configure(gpio0, INT1, GPIO_INPUT) == 0);
	expect_calls("pulls(24,0,0) func(24,5) dir(24,0) ");

	/* An edge: acknowledged, then the callbacks (the last added first). */
	gpio_init_callback(&a.cb, counted_handler, BIT(INT1));
	gpio_init_callback(&b.cb, counted_handler, BIT(INT1));
	assert(gpio_add_callback(gpio0, &a.cb) == 0);
	assert(gpio_add_callback(gpio0, &b.cb) == 0);
	assert(gpio_pin_interrupt_configure(gpio0, INT1, GPIO_INT_EDGE_TO_ACTIVE) == 0);
	expect_calls("irq(24,0xf,0) irq(24,0x8,1) ");
	set_level(INT1, true);
	expect_calls("ack(24,0xf) b(0x1000000,isr=1) a(0x1000000,isr=1) ");
	assert(intr[INT1 / 8] == 0);
	set_level(INT1, false);
	expect_calls("");

	/* The falling edge latched while it was off does not fire. */
	assert((intr[INT1 / 8] >> SHIFT(INT1)) == GPIO_IRQ_EDGE_FALL);
	assert(gpio_pin_interrupt_configure(gpio0, INT1, GPIO_INT_EDGE_BOTH) == 0);
	bank_irq();
	expect_calls("irq(24,0xf,0) irq(24,0xc,1) ");
	assert(a.calls == 1 && b.calls == 1);

	/* A removed callback is not called; removing it again is -EINVAL. */
	assert(gpio_remove_callback(gpio0, &b.cb) == 0);
	assert(gpio_remove_callback(gpio0, &b.cb) == -EINVAL);
	set_level(INT1, true);
	expect_calls("ack(24,0xf) a(0x1000000,isr=1) ");

	/* A pin of another handler is left alone. */
	inte[OTHER / 8] |= (uint32_t)GPIO_IRQ_EDGE_RISE << SHIFT(OTHER);
	set_level(OTHER, true);
	set_level(INT1, false);
	expect_calls("ack(24,0xf) a(0x1000000,isr=1) ");
	assert(((intr[OTHER / 8] >> SHIFT(OTHER)) & 0xfu) == GPIO_IRQ_EDGE_RISE);
	inte[OTHER / 8] = 0;
	intr[OTHER / 8] = 0;
	level &= ~(1u << OTHER);

	/* Off: no more calls. */
	assert(gpio_pin_interrupt_configure(gpio0, INT1, GPIO_INT_DISABLE) == 0);
	expect_calls("irq(24,0xf,0) ");
	set_level(INT1, true);
	set_level(INT1, false);
	expect_calls("");
	assert(gpio_remove_callback(gpio0, &a.cb) == 0);
	assert(a.calls == 3 && b.calls == 1);
	assert(raw_mask == (1u << INT1));
}

/* lsm6dso_fifo_irq ------------------------------------------------------------*/

static int handled;
static int handle_result;
static char handler_task[configMAX_TASK_NAME_LEN];

static int fifo_handle(void *context)
{
	assert(context == &handled);
	handled++;
	strcpy(handler_task, pcTaskGetName(NULL));
	return handle_result;
}

static void test_fifo_irq(void)
{
	static struct lsm6dso_fifo_irq irq;
	const struct gpio_dt_spec spec = {.port = gpio0, .pin = INT1, .dt_flags = 0};

	/* The start: the pin as an input, then its rising edge. */
	assert(lsm6dso_fifo_irq_start(&irq, &spec, fifo_handle, &handled) == 0);
	expect_calls("pulls(24,0,0) func(24,5) dir(24,0) irq(24,0xf,0) irq(24,0x8,1) ");
	k_msleep(5);
	assert(handled == 0);

	/* An edge: the work runs the handler in the system work queue. */
	set_level(INT1, true);
	expect_calls("ack(24,0xf) ");
	k_msleep(5);
	assert(handled == 1);
	assert(strcmp(handler_task, "sysworkq") == 0);
	set_level(INT1, false);
	set_level(INT1, true);
	k_msleep(5);
	assert(handled == 2);
	expect_calls("ack(24,0xf) ");

	/* The stop: off, and the callback is gone. */
	assert(lsm6dso_fifo_irq_stop(&irq) == 0);
	expect_calls("irq(24,0xf,0) ");
	assert(gpio_remove_callback(gpio0, &irq.callback) == -EINVAL);

	/* A level already high at the start is not lost. */
	assert(lsm6dso_fifo_irq_start(&irq, &spec, fifo_handle, &handled) == 0);
	expect_calls("pulls(24,0,0) func(24,5) dir(24,0) irq(24,0xf,0) irq(24,0x8,1) ");
	k_msleep(5);
	assert(handled == 3);

	/* A handler that fails turns the interrupt off. */
	handle_result = -EIO;
	set_level(INT1, false);
	set_level(INT1, true);
	k_msleep(5);
	assert(handled == 4);
	expect_calls("ack(24,0xf) irq(24,0xf,0) ");
	set_level(INT1, false);
	set_level(INT1, true);
	k_msleep(5);
	assert(handled == 4);
	expect_calls("");
	assert(lsm6dso_fifo_irq_stop(&irq) == 0);
	expect_calls("irq(24,0xf,0) ");
	handle_result = 0;
	set_level(INT1, false);
}

static void runner(void *arg)
{
	(void)arg;

	test_start();
	test_configure();
	test_interrupts();
	test_fifo_irq();
	printf("gpio: all tests passed\n");
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
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, NANO_PRIO_SHELL,
			NULL) != pdPASS) {
		return 1;
	}
	vTaskStartScheduler();
	return 1;
}
