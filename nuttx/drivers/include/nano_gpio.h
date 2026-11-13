/* SPDX-License-Identifier: Apache-2.0 */
/* The board's GPIO bank 0 (GPIO0..29) for the Zephyr compatibility layer's
 * gpio0 device (nuttx/drivers/compat/zephyr_gpio.c), which the LSM6DSOX's
 * INT1 line (GPIO24, BOARD_LSM6DSOX_INT1_GPIO) uses.  The board implements
 * it in src/nano_gpio.c as Zephyr's RP2040 GPIO driver does
 * (drivers/gpio/gpio_rpi_pico.c, on the pico-sdk's hardware_gpio).  Shared
 * by the board code and nuttx/drivers ($(TOPDIR)/external/include).
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_NANO_GPIO_H
#define __NUTTX_DRIVERS_INCLUDE_NANO_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#define NANO_GPIO_NUM 30

/* The RP2040's interrupt events of a pin (pico-sdk GPIO_IRQ_*). */

#define NANO_GPIO_IRQ_LEVEL_LOW  0x1U
#define NANO_GPIO_IRQ_LEVEL_HIGH 0x2U
#define NANO_GPIO_IRQ_EDGE_FALL  0x4U
#define NANO_GPIO_IRQ_EDGE_RISE  0x8U
#define NANO_GPIO_IRQ_ALL        0xfU

/* gpio_rpi_configure() for an input: the pulls, the SIO function (input
 * enable on, output disable off, no overrides), direction in.  0 or
 * -EINVAL.
 */

int nano_gpio_configure_input(unsigned int pin, bool pull_up,
                              bool pull_down);

/* The levels of all pins (SIO GPIO_IN). */

uint32_t nano_gpio_get_all(void);

/* gpio_rpi_pin_interrupt_configure(): the pin's events are disabled and
 * cleared, then the given ones (NANO_GPIO_IRQ_*, 0 for none) are cleared
 * and enabled.  On an enabled event the bank's interrupt handler clears it
 * and calls nano_gpio_irq_handler() with the pin.  0 or -EINVAL.
 */

int nano_gpio_irq_configure(unsigned int pin, uint32_t events);

/* Provided by the compatibility layer: called in the interrupt handler. */

void nano_gpio_irq_handler(unsigned int pin);

#endif /* __NUTTX_DRIVERS_INCLUDE_NANO_GPIO_H */
