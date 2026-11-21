/* SPDX-License-Identifier: Apache-2.0 */
/* The board's side of the NINA-W102 SPI link: the SPI1 bus and the
 * module's four GPIO lines, in the terms of the Zephyr calls (the
 * devicetree's spi1, cs-gpios, ready-gpios, reset-gpios and irq-gpios with
 * their flags, and the Zephyr GPIO and SPI calls on them).  nina_proto.c is
 * written against this interface.  Each build implements it:
 * zephyr/nina_bus.c next to this file (on the devicetree),
 * nuttx/nina_bus_rp2040.c and freertos/nina_bus_rp2040.c (on the
 * RP2040's SPI1 and GPIO); the host test (common/tests/nina_proto.c)
 * replaces it with fakes.
 *
 * Values are logical, as gpio_pin_set_dt()/gpio_pin_get_dt() in Zephyr:
 * 1 is "active".  The board knows each line's polarity (devicetree flags
 * of arduino_nano_connect.dts):
 *
 *   CS     GPIO9   active low                  (spi1 cs-gpios, 0x1)
 *   READY  GPIO10  active low, pull-up         (ready-gpios, 0x11)
 *   RESET  GPIO3   active low                  (reset-gpios, 0x1)
 *   IRQ    GPIO2   active high, pull-down      (irq-gpios, 0x20)
 */

#ifndef NINA_BUS_H_
#define NINA_BUS_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum nina_pin {
	NINA_PIN_CS,
	NINA_PIN_READY,
	NINA_PIN_RESET,
	NINA_PIN_IRQ,
};

/* Zephyr's gpio_pin_configure_dt() flags used by nina_proto.c. */
enum nina_pin_mode {
	NINA_PIN_OUTPUT_INACTIVE, /* GPIO_OUTPUT_INACTIVE */
	NINA_PIN_OUTPUT_ACTIVE,   /* GPIO_OUTPUT_ACTIVE */
	NINA_PIN_OUTPUT_HIGH,     /* GPIO_OUTPUT_HIGH (physical level) */
	NINA_PIN_INPUT,           /* GPIO_INPUT */
};

/* Called once at boot by the NuttX and FreeRTOS builds (Zephyr initializes
 * spi1 and configures its cs-gpios at boot, before main(); the Zephyr build
 * has no such function).  Sets up SPI1 and its pins and drives CS
 * inactive.  Leaves READY, RESET and IRQ alone: nina_proto.c configures them
 * on the first request.
 */
int nina_bus_initialize(void);

/* device_is_ready(spi) && gpio_is_ready_dt() of the four lines. */
bool nina_bus_ready(void);

/* gpio_pin_configure_dt(): the line's devicetree flags (polarity, pulls)
 * combined with MODE, applied as Zephyr's RP2040 GPIO driver does.
 */
int nina_pin_configure(enum nina_pin pin, enum nina_pin_mode mode);

/* gpio_pin_set_dt() and gpio_pin_get_dt(): logical values. */
int nina_pin_set(enum nina_pin pin, int value);
int nina_pin_get(enum nina_pin pin);

/* spi_transceive() with one TX and one RX buffer of LEN bytes, or
 * spi_write() when RX is NULL: SPI mode 0, 8-bit words, MSB first, at
 * CONFIG_NINA_SPI_FREQUENCY, by polling (Zephyr: its SPI driver's).  Does
 * not touch CS (nina_proto.c drives it).
 */
int nina_bus_transceive(const uint8_t *tx, uint8_t *rx, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* NINA_BUS_H_ */
