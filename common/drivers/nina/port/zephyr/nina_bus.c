/* SPDX-License-Identifier: MIT */
/* The NINA-W102 SPI link on Zephyr (nina_bus.h of the core, next to this
 * binding): the devicetree's spi1, its cs-gpios
 * and the nina_w102 node's ready-gpios, reset-gpios and irq-gpios (with
 * their flags: polarity and pulls; the board's devicetree), and the
 * Zephyr GPIO and SPI calls that nina_proto.c's table names:
 *
 *   nina_bus_initialize()  device_is_ready(spi1) and gpio_is_ready_dt()
 *                               of the four lines, in that order
 *   nina_bus_pin_configure()   gpio_pin_configure_dt() with GPIO_OUTPUT_
 *                               INACTIVE, _ACTIVE, _HIGH or GPIO_INPUT
 *   nina_bus_pin_set(), _get() gpio_pin_set_dt(), gpio_pin_get_dt()
 *                               (logical values)
 *   nina_bus_transceive()      spi_transceive() with one TX and one RX
 *                               buffer, or spi_write() without RX: mode 0,
 *                               8-bit words, MSB first, at
 *                               CONFIG_NINA_SPI_FREQUENCY
 *
 * Zephyr initializes spi1 and its cs-gpios at boot, so nina_bus_initialize()
 * only looks whether they are ready; nina_proto.c configures the other
 * lines on the first request.
 *
 * With CONFIG_NINA_PASSTHROUGH, while nina_passthrough_active() is true
 * (the NINA passthrough mode, nina_passthrough.h, where the module's lines
 * and the SPI pins belong to the mode) the bus is never ready, so every
 * request fails with -ENODEV before any line is touched.  The host test
 * (zephyr/app/usb-console/tests/nina_boundary.c) builds this file on fakes
 * of the Zephyr calls.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "nina_bus.h"
#ifdef CONFIG_NINA_PASSTHROUGH
#include "nina_passthrough.h"
#endif

#define NINA_NODE DT_NODELABEL(nina_w102)

static const struct device *const spi = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct gpio_dt_spec ready = GPIO_DT_SPEC_GET(NINA_NODE, ready_gpios);
static const struct gpio_dt_spec reset = GPIO_DT_SPEC_GET(NINA_NODE, reset_gpios);
static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(NINA_NODE, irq_gpios);
static const struct gpio_dt_spec cs = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi1), cs_gpios, 0);
static const struct spi_config config = {.frequency = CONFIG_NINA_SPI_FREQUENCY,
					 .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB};

static const struct gpio_dt_spec *line(enum nina_bus_pin pin)
{
	switch (pin) {
	case NINA_BUS_PIN_CS:
		return &cs;
	case NINA_BUS_PIN_READY:
		return &ready;
	case NINA_BUS_PIN_RESET:
		return &reset;
	case NINA_BUS_PIN_IRQ:
		return &irq;
	default:
		return NULL;
	}
}

int nina_bus_initialize(void)
{
#ifdef CONFIG_NINA_PASSTHROUGH
	if (nina_passthrough_active()) {
		return -ENODEV;
	}
#endif
	return device_is_ready(spi) && gpio_is_ready_dt(&ready) && gpio_is_ready_dt(&reset) &&
			       gpio_is_ready_dt(&irq) && gpio_is_ready_dt(&cs)
		       ? 0
		       : -ENODEV;
}

int nina_bus_pin_configure(enum nina_bus_pin pin, enum nina_bus_pin_mode mode)
{
	const struct gpio_dt_spec *spec = line(pin);
	gpio_flags_t flags;

	switch (mode) {
	case NINA_BUS_PIN_OUTPUT_INACTIVE:
		flags = GPIO_OUTPUT_INACTIVE;
		break;
	case NINA_BUS_PIN_OUTPUT_ACTIVE:
		flags = GPIO_OUTPUT_ACTIVE;
		break;
	case NINA_BUS_PIN_OUTPUT_HIGH:
		flags = GPIO_OUTPUT_HIGH;
		break;
	case NINA_BUS_PIN_INPUT:
		flags = GPIO_INPUT;
		break;
	default:
		return -EINVAL;
	}
	return spec ? gpio_pin_configure_dt(spec, flags) : -EINVAL;
}

int nina_bus_pin_set(enum nina_bus_pin pin, int value)
{
	const struct gpio_dt_spec *spec = line(pin);

	return spec ? gpio_pin_set_dt(spec, value) : -EINVAL;
}

int nina_bus_pin_get(enum nina_bus_pin pin)
{
	const struct gpio_dt_spec *spec = line(pin);

	return spec ? gpio_pin_get_dt(spec) : -EINVAL;
}

/* The caller keeps NINA's chip select asserted across these transfers. */
int nina_bus_transceive(const uint8_t *tx, uint8_t *rx, size_t len)
{
	struct spi_buf tb = {.buf = (void *)tx, .len = len};
	struct spi_buf rb = {.buf = rx, .len = len};
	struct spi_buf_set ts = {.buffers = &tb, .count = 1};
	struct spi_buf_set rs = {.buffers = &rb, .count = 1};

	return rx ? spi_transceive(spi, &config, &ts, &rs) : spi_write(spi, &config, &ts);
}
