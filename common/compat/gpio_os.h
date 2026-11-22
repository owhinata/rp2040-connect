/* SPDX-License-Identifier: Apache-2.0 */
/* The board's GPIO bank 0 (GPIO0..29) for the Zephyr compatibility layer's
 * gpio0 device (zephyr_gpio.c here), which the LSM6DSOX's INT1 line
 * (GPIO24) and, on NuttX, the NINA's READY line (GPIO10, the waits of
 * nina_os.c) and IRQ line (GPIO2, its socket events, nina_os.c) use.
 * Each OS implements it as Zephyr's RP2040 GPIO driver does
 * (drivers/gpio/gpio_rpi_pico.c, on the pico-sdk's hardware_gpio): on
 * NuttX the board's src/rp2040_gpio_os.c (BOARD_LSM6DSOX_INT1_GPIO), on
 * FreeRTOS freertos/drivers/compat/rp2040/gpio_os_rp2040.c.  Shared by the
 * board code and the layer (common/compat).
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_GPIO_OS_H
#define __NUTTX_DRIVERS_INCLUDE_GPIO_OS_H

#include <stdbool.h>
#include <stdint.h>

#define GPIO_OS_NUM 30

/* The RP2040's interrupt events of a pin (pico-sdk GPIO_IRQ_*). */

#define GPIO_OS_IRQ_LEVEL_LOW  0x1U
#define GPIO_OS_IRQ_LEVEL_HIGH 0x2U
#define GPIO_OS_IRQ_EDGE_FALL  0x4U
#define GPIO_OS_IRQ_EDGE_RISE  0x8U
#define GPIO_OS_IRQ_ALL        0xfU

/* gpio_rpi_configure() for an input: the pulls, the SIO function (input
 * enable on, output disable off, no overrides), direction in.  0 or
 * -EINVAL.
 */

int gpio_os_configure_input(unsigned int pin, bool pull_up,
                              bool pull_down);

/* The levels of all pins (SIO GPIO_IN). */

uint32_t gpio_os_get_all(void);

/* gpio_rpi_pin_interrupt_configure(): the pin's events are disabled and
 * cleared, then the given ones (GPIO_OS_IRQ_*, 0 for none) are cleared
 * and enabled.  On an enabled event the bank's interrupt handler clears it
 * and calls gpio_os_irq_handler() with the pin.  0 or -EINVAL.
 */

int gpio_os_irq_configure(unsigned int pin, uint32_t events);

/* Provided by the compatibility layer: called in the interrupt handler. */

void gpio_os_irq_handler(unsigned int pin);

#endif /* __NUTTX_DRIVERS_INCLUDE_GPIO_OS_H */
