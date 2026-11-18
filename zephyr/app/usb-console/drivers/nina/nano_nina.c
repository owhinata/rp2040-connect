/* SPDX-License-Identifier: MIT */
/* The board's side of the NINA-W102 SPI link on Zephyr (nano_nina.h of the
 * shared core, common/drivers/nina): the devicetree's spi1, its cs-gpios
 * and the nina_w102 node's ready-gpios, reset-gpios and irq-gpios (with
 * their flags: polarity and pulls, arduino_nano_connect.dts), and the
 * Zephyr GPIO and SPI calls that nina_spi.c's table names:
 *
 *   nano_nina_bus_ready()       device_is_ready(spi1) and gpio_is_ready_dt()
 *                               of the four lines, in that order
 *   nano_nina_pin_configure()   gpio_pin_configure_dt() with GPIO_OUTPUT_
 *                               INACTIVE, _ACTIVE, _HIGH or GPIO_INPUT
 *   nano_nina_pin_set(), _get() gpio_pin_set_dt(), gpio_pin_get_dt()
 *                               (logical values)
 *   nano_nina_transceive()      spi_transceive() with one TX and one RX
 *                               buffer, or spi_write() without RX: mode 0,
 *                               8-bit words, MSB first, at
 *                               CONFIG_NINA_SPI_FREQUENCY
 *
 * Zephyr initializes spi1 and its cs-gpios at boot, so this build has no
 * nano_nina_initialize(); nina_spi.c configures the other lines on the
 * first request.  In the NINA passthrough mode (nina_passthrough.h) the
 * bus is never ready: GPIO8 and GPIO9 are UART1's then, and the mode owns
 * the module's lines, so every request fails with -ENODEV before any of
 * them is touched.  The host test (tests/nina_boundary.c) builds this file
 * on fakes of the Zephyr calls.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "nano_nina.h"
#include "nina_passthrough.h"

#define NINA_NODE DT_NODELABEL(nina_w102)

static const struct device *const spi = DEVICE_DT_GET(DT_NODELABEL(spi1));
static const struct gpio_dt_spec ready = GPIO_DT_SPEC_GET(NINA_NODE, ready_gpios);
static const struct gpio_dt_spec reset = GPIO_DT_SPEC_GET(NINA_NODE, reset_gpios);
static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(NINA_NODE, irq_gpios);
static const struct gpio_dt_spec cs = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(spi1), cs_gpios, 0);
static const struct spi_config config = {.frequency = CONFIG_NINA_SPI_FREQUENCY,
					 .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB};

static const struct gpio_dt_spec *line(enum nano_nina_pin pin)
{
	switch (pin) {
	case NANO_NINA_CS:
		return &cs;
	case NANO_NINA_READY:
		return &ready;
	case NANO_NINA_RESET:
		return &reset;
	case NANO_NINA_IRQ:
		return &irq;
	default:
		return NULL;
	}
}

bool nano_nina_bus_ready(void)
{
	if (nina_passthrough_active()) {
		return false;
	}
	return device_is_ready(spi) && gpio_is_ready_dt(&ready) && gpio_is_ready_dt(&reset) &&
	       gpio_is_ready_dt(&irq) && gpio_is_ready_dt(&cs);
}

int nano_nina_pin_configure(enum nano_nina_pin pin, enum nano_nina_mode mode)
{
	const struct gpio_dt_spec *spec = line(pin);
	gpio_flags_t flags;

	switch (mode) {
	case NANO_NINA_OUTPUT_INACTIVE:
		flags = GPIO_OUTPUT_INACTIVE;
		break;
	case NANO_NINA_OUTPUT_ACTIVE:
		flags = GPIO_OUTPUT_ACTIVE;
		break;
	case NANO_NINA_OUTPUT_HIGH:
		flags = GPIO_OUTPUT_HIGH;
		break;
	case NANO_NINA_INPUT:
		flags = GPIO_INPUT;
		break;
	default:
		return -EINVAL;
	}
	return spec ? gpio_pin_configure_dt(spec, flags) : -EINVAL;
}

int nano_nina_pin_set(enum nano_nina_pin pin, int value)
{
	const struct gpio_dt_spec *spec = line(pin);

	return spec ? gpio_pin_set_dt(spec, value) : -EINVAL;
}

int nano_nina_pin_get(enum nano_nina_pin pin)
{
	const struct gpio_dt_spec *spec = line(pin);

	return spec ? gpio_pin_get_dt(spec) : -EINVAL;
}

/* The caller keeps NINA's chip select asserted across these transfers. */
int nano_nina_transceive(const uint8_t *tx, uint8_t *rx, size_t len)
{
	struct spi_buf tb = {.buf = (void *)tx, .len = len};
	struct spi_buf rb = {.buf = rx, .len = len};
	struct spi_buf_set ts = {.buffers = &tb, .count = 1};
	struct spi_buf_set rs = {.buffers = &rb, .count = 1};

	return rx ? spi_transceive(spi, &config, &ts, &rs) : spi_write(spi, &config, &ts);
}
