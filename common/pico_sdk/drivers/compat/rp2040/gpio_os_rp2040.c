/* SPDX-License-Identifier: MIT */
/* GPIO bank 0 for the Zephyr compatibility layer's gpio0 device
 * (common/compat/port/gpio_os.h, used by common/compat/zephyr_gpio.c) on the
 * Pico SDK's hardware_gpio, as Zephyr's drivers/gpio/gpio_rpi_pico.c drives
 * it (that driver is written on the same SDK calls):
 *
 *   Zephyr (gpio_rpi_pico.c)               here
 *   gpio_rpi_configure() for an input:     the same calls in the same order:
 *   gpio_set_pulls(), gpio_set_function(   gpio_set_pulls(), gpio_set_function(
 *   GPIO_FUNC_SIO), gpio_set_dir(GPIO_IN)  GPIO_FUNC_SIO), gpio_set_dir(GPIO_IN)
 *   gpio_rpi_pin_interrupt_configure():    the same: gpio_set_irq_enabled(pin,
 *   gpio_set_irq_enabled(pin, all events,  all events, false), then
 *   false), then gpio_set_irq_enabled(     gpio_set_irq_enabled(pin, events,
 *   pin, events, true)                     true); each clears the events it
 *                                          names (INTR, write 1 to clear)
 *                                          before it changes PROC0_INTE, so an
 *                                          edge latched while the event was
 *                                          off does not fire
 *   gpio_rpi_bank_init() (POST_KERNEL 40): the handler is added the first
 *   IRQ_CONNECT() of IO_IRQ_BANK0 with     time a pin's interrupt is enabled,
 *   gpio_rpi_isr(), irq_enable()           as the NuttX build does
 *                                          (gpio_add_raw_irq_handler_masked(),
 *                                          then irq_set_enabled(IO_IRQ_BANK0))
 *   gpio_rpi_isr(): for each pin 0..29     the same, for the pins of
 *   with pending events (PROC0_INTS),      GPIO_OS_IRQ_PINS:
 *   gpio_acknowledge_irq(pin, all          gpio_get_irq_event_mask() (INTS),
 *   events), then the pin's callbacks      gpio_acknowledge_irq(pin, all
 *                                          events), gpio_os_irq_handler()
 *
 * Why a raw handler: the SDK's gpio_set_irq_callback() keeps one callback
 * per core for every pin, so a second user of it would take the place of
 * this one.  gpio_add_raw_irq_handler_masked() adds a shared handler of
 * IO_IRQ_BANK0 for the pins of its mask, which the SDK's default handler
 * (that of gpio_set_irq_callback()) then leaves alone.  The SDK keeps the
 * mask per core and asserts that no two handlers claim a pin; one handler
 * cannot be added twice for two masks, so the pins that may interrupt are
 * fixed here (GPIO_OS_IRQ_PINS: the LSM6DSOX's INT1, GPIO24; Zephyr's
 * driver takes any pin) and claimed together.  An interrupt on another pin
 * is refused with -ENOTSUP.  The NINA's READY (GPIO10) and IRQ line (GPIO2)
 * have a raw handler of their own, added the same way by the NINA core's
 * binding (common/drivers/nina/port/<OS>/nina_os.c); each handler
 * looks at and acknowledges only its own pins.
 *
 * The interrupt runs at the SDK's default priority (0x80) like every
 * peripheral interrupt of the build (I2C0, the DMA, USB; Zephyr gives them
 * all its lowest, 3).  gpio_os_irq_handler() runs the pin's callbacks there;
 * the LSM6DSO's submits a work item (k_work_submit(): the wake-up of the
 * work queue's thread from the interrupt and a switch at the end of the
 * handler).  The core's
 * IO_BANK0 registers are the calling core's (core 0, the only one running).
 *
 * Part of the Zephyr compatibility layer for an OS on the Pico SDK with the
 * generic kernel part (common/pico_sdk/drivers/compat/rp2040; NuttX's is
 * the board's src/rp2040_gpio_os.c).
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"

#include "zephyr_kernel_os.h"

#include "gpio_os.h"

/* The pin whose interrupt goes through gpio0: GPIO24, the LSM6DSOX's INT1
 * (irq-gpios = <&gpio0 24 0> of the Zephyr build's devicetree).
 */
#define GPIO_OS_IRQ_PINS (1u << 24)

/* gpio_os.h's event bits are the SDK's (and the registers'). */
_Static_assert(GPIO_OS_IRQ_LEVEL_LOW == GPIO_IRQ_LEVEL_LOW, "GPIO_IRQ_LEVEL_LOW");
_Static_assert(GPIO_OS_IRQ_LEVEL_HIGH == GPIO_IRQ_LEVEL_HIGH, "GPIO_IRQ_LEVEL_HIGH");
_Static_assert(GPIO_OS_IRQ_EDGE_FALL == GPIO_IRQ_EDGE_FALL, "GPIO_IRQ_EDGE_FALL");
_Static_assert(GPIO_OS_IRQ_EDGE_RISE == GPIO_IRQ_EDGE_RISE, "GPIO_IRQ_EDGE_RISE");

static bool attached;

static void gpio_os_isr(void)
{
	for (unsigned int pin = 0; pin < GPIO_OS_NUM; pin++) {
		if ((GPIO_OS_IRQ_PINS & (1u << pin)) == 0) {
			continue;
		}
		if (gpio_get_irq_event_mask(pin) != 0) {
			gpio_acknowledge_irq(pin, GPIO_OS_IRQ_ALL);
			gpio_os_irq_handler(pin);
		}
	}
}

int gpio_os_configure_input(unsigned int pin, bool pull_up, bool pull_down)
{
	if (pin >= GPIO_OS_NUM) {
		return -EINVAL;
	}

	gpio_set_pulls(pin, pull_up, pull_down);
	gpio_set_function(pin, GPIO_FUNC_SIO);
	gpio_set_dir(pin, GPIO_IN);
	return 0;
}

uint32_t gpio_os_get_all(void)
{
	return gpio_get_all();
}

int gpio_os_irq_configure(unsigned int pin, uint32_t events)
{
	if (pin >= GPIO_OS_NUM || (events & ~GPIO_OS_IRQ_ALL) != 0) {
		return -EINVAL;
	}
	if (events != 0 && (GPIO_OS_IRQ_PINS & (1u << pin)) == 0) {
		return -ENOTSUP;
	}

	if (events != 0) {
		/* With the pin's events still off (INTE clear since the reset or
		 * the last disable).  Callers are threads (the LSM6DSO's work queue
		 * and the stream's start); the lock (z_os_task_lock() of the
		 * generic kernel part, zephyr_kernel_os.h) keeps two of them
		 * from adding the handler twice.  The pins' enables are set and
		 * cleared through the registers' atomic aliases, so two pins (also
		 * the NINA binding's) may be configured at once.
		 */
		z_os_task_lock();
		if (!attached) {
			gpio_add_raw_irq_handler_masked(GPIO_OS_IRQ_PINS, gpio_os_isr);
			irq_set_enabled(IO_IRQ_BANK0, true);
			attached = true;
		}
		z_os_task_unlock();
	}

	gpio_set_irq_enabled(pin, GPIO_OS_IRQ_ALL, false);
	if (events != 0) {
		gpio_set_irq_enabled(pin, events, true);
	}
	return 0;
}
