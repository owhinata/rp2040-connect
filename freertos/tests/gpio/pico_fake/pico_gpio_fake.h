/* SPDX-License-Identifier: MIT */
/* The part of the Pico SDK that freertos/drivers/compat/rp2040/nano_gpio.c
 * uses, for its host test (freertos/tests/gpio/main.c, which implements the
 * functions: they record the calls and play GPIO bank 0's interrupt
 * registers).  hardware/gpio.h and hardware/irq.h next to this one include
 * it.  Names and values as in the SDK (hardware_gpio, hardware_irq,
 * hardware/regs/intctrl.h).
 */

#ifndef FREERTOS_TESTS_GPIO_PICO_FAKE_PICO_GPIO_FAKE_H
#define FREERTOS_TESTS_GPIO_PICO_FAKE_PICO_GPIO_FAKE_H

#include <stdbool.h>
#include <stdint.h>

typedef unsigned int uint;

/* hardware_gpio */
typedef enum gpio_function {
	GPIO_FUNC_SIO = 5,
} gpio_function_t;

enum gpio_dir {
	GPIO_IN = 0,
	GPIO_OUT = 1,
};

enum gpio_irq_level {
	GPIO_IRQ_LEVEL_LOW = 0x1u,
	GPIO_IRQ_LEVEL_HIGH = 0x2u,
	GPIO_IRQ_EDGE_FALL = 0x4u,
	GPIO_IRQ_EDGE_RISE = 0x8u,
};

void gpio_set_pulls(uint gpio, bool up, bool down);
void gpio_set_function(uint gpio, gpio_function_t fn);
void gpio_set_dir(uint gpio, bool out);
uint32_t gpio_get_all(void);
void gpio_set_irq_enabled(uint gpio, uint32_t event_mask, bool enabled);
uint32_t gpio_get_irq_event_mask(uint gpio);
void gpio_acknowledge_irq(uint gpio, uint32_t event_mask);

/* hardware/regs/intctrl.h and hardware_irq */
#define IO_IRQ_BANK0 13

typedef void (*irq_handler_t)(void);

void gpio_add_raw_irq_handler_masked(uint32_t gpio_mask, irq_handler_t handler);
void irq_set_enabled(uint num, bool enabled);

#endif /* FREERTOS_TESTS_GPIO_PICO_FAKE_PICO_GPIO_FAKE_H */
