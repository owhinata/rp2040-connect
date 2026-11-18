/* SPDX-License-Identifier: Apache-2.0 */
/* The board's side of the NINA-W102 SPI link: the SPI1 bus and the
 * module's four GPIO lines, in the terms of the Zephyr calls (the
 * devicetree's spi1, cs-gpios, ready-gpios, reset-gpios and irq-gpios with
 * their flags, and the Zephyr GPIO and SPI calls on them).  nina_spi.c is
 * written against this interface.  Each build implements it:
 * zephyr/app/usb-console/drivers/nina/nano_nina.c (on the devicetree),
 * nuttx/boards/arduino-nano-rp2040-connect/src/nano_nina.c,
 * freertos/drivers/nina/nano_nina.c; the host test (common/tests/nina_spi.c)
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

#ifndef NANO_NINA_H_
#define NANO_NINA_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

enum nano_nina_pin
{
  NANO_NINA_CS,
  NANO_NINA_READY,
  NANO_NINA_RESET,
  NANO_NINA_IRQ,
};

/* Zephyr's gpio_pin_configure_dt() flags used by nina_spi.c. */

enum nano_nina_mode
{
  NANO_NINA_OUTPUT_INACTIVE,   /* GPIO_OUTPUT_INACTIVE */
  NANO_NINA_OUTPUT_ACTIVE,     /* GPIO_OUTPUT_ACTIVE */
  NANO_NINA_OUTPUT_HIGH,       /* GPIO_OUTPUT_HIGH (physical level) */
  NANO_NINA_INPUT,             /* GPIO_INPUT */
};

/* Called once at boot by the NuttX and FreeRTOS builds (Zephyr initializes
 * spi1 and configures its cs-gpios at boot, before main(); the Zephyr build
 * has no such function).  Sets up SPI1 and its pins and drives CS
 * inactive.  Leaves READY, RESET and IRQ alone: nina_spi.c configures them
 * on the first request.
 */

int nano_nina_initialize(void);

/* device_is_ready(spi) && gpio_is_ready_dt() of the four lines. */

bool nano_nina_bus_ready(void);

/* gpio_pin_configure_dt(): the line's devicetree flags (polarity, pulls)
 * combined with MODE, applied as Zephyr's RP2040 GPIO driver does.
 */

int nano_nina_pin_configure(enum nano_nina_pin pin, enum nano_nina_mode mode);

/* gpio_pin_set_dt() and gpio_pin_get_dt(): logical values. */

int nano_nina_pin_set(enum nano_nina_pin pin, int value);
int nano_nina_pin_get(enum nano_nina_pin pin);

/* spi_transceive() with one TX and one RX buffer of LEN bytes, or
 * spi_write() when RX is NULL: SPI mode 0, 8-bit words, MSB first, at
 * CONFIG_NANO_NINA_SPI_FREQUENCY (Zephyr: CONFIG_NINA_SPI_FREQUENCY), by
 * polling (Zephyr: its SPI driver's).  Does not touch CS (nina_spi.c
 * drives it).
 */

int nano_nina_transceive(const uint8_t *tx, uint8_t *rx, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* NANO_NINA_H_ */
