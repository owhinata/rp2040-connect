/* SPDX-License-Identifier: MIT */
/* The NINA-W102 SPI link on Zephyr (nina_bus.h of the core, next to this
 * binding): the devicetree's node of the module (compatible
 * "arduino,nina-spi"), with its ready-gpios, reset-gpios and irq-gpios
 * (their flags: polarity and pulls), its spi-max-frequency, the SPI bus it
 * is on and that bus's cs-gpios entry of its reg, and the
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
 *                               the node's spi-max-frequency
 *
 * The bus's driver does the transfers: with CONFIG_SPI_PL022_DMA and the
 * bus's dmas (app.overlay), a transfer of CONFIG_SPI_PL022_DMA_MIN_LEN
 * bytes (64) or more goes through two DMA channels while the thread waits,
 * a shorter one (the core's single bytes) is polled; without them every
 * transfer is polled.  A DMA error or a transfer that does not end in time
 * is the driver's error, logged by the driver, which nina_bus_transceive()
 * returns.
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

/* The module's node, the bus it is on and its chip select there. */
#define NINA_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(arduino_nina_spi)
#define NINA_BUS  DT_BUS(NINA_NODE)

static const struct device *const spi = DEVICE_DT_GET(NINA_BUS);
static const struct gpio_dt_spec ready = GPIO_DT_SPEC_GET(NINA_NODE, ready_gpios);
static const struct gpio_dt_spec reset = GPIO_DT_SPEC_GET(NINA_NODE, reset_gpios);
static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(NINA_NODE, irq_gpios);
static const struct gpio_dt_spec cs =
	GPIO_DT_SPEC_GET_BY_IDX(NINA_BUS, cs_gpios, DT_REG_ADDR_RAW(NINA_NODE));
static const struct spi_config config = {.frequency = DT_PROP(NINA_NODE, spi_max_frequency),
					 .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
						      SPI_LOCK_ON};

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

/* The bus lock with SPI_LOCK_ON: the first transfer of the configuration
 * takes it and keeps it until spi_release().  A transfer without buffers
 * takes it before CS goes active.
 *
 * Zephyr has no call that only takes the lock, and what a transfer without
 * buffers does is the SPI driver's: the PL022 driver takes the lock first
 * and, polled, then finds nothing to move (spi_pl022.c
 * spi_pl022_transceive_impl()); others return before they take it
 * (spi_litex_litespi.c).  The PL022's interrupt mode would start a
 * transfer of nothing, and so would its DMA mode, unless
 * CONFIG_SPI_PL022_DMA_MIN_LEN (the local patch zephyr/0007) sends
 * transfers shorter than that, this one too, down the polled path: then
 * the DMA mode keeps the lock as the polled driver does, and the long
 * transfers between select and deselect go through the DMA under the same
 * lock (the driver's DMA path changes neither the lock nor CS, which this
 * file drives).  With CONFIG_SPI_ASYNC a synchronous transfer gives the
 * lock back when it ends, SPI_LOCK_ON or not (spi_context.h
 * spi_context_release()).  So this file builds only for the PL022, polled
 * or in its DMA mode with a threshold, without CONFIG_SPI_ASYNC; another
 * controller or configuration needs its own way to take the bus here.
 */
BUILD_ASSERT(DT_NODE_HAS_COMPAT(NINA_BUS, arm_pl022),
	     "nina_bus_select() takes the bus lock as the PL022 SPI driver does");
BUILD_ASSERT(!IS_ENABLED(CONFIG_SPI_PL022_INTERRUPT),
	     "nina_bus_select() needs the polled PL022 SPI driver, or its DMA mode");
#ifdef CONFIG_SPI_PL022_DMA
BUILD_ASSERT(CONFIG_SPI_PL022_DMA_MIN_LEN > 0,
	     "in the PL022's DMA mode nina_bus_select() needs CONFIG_SPI_PL022_DMA_MIN_LEN > 0");
#endif
BUILD_ASSERT(!IS_ENABLED(CONFIG_SPI_ASYNC),
	     "with CONFIG_SPI_ASYNC a transfer does not keep the bus lock for nina_bus_select()");

int nina_bus_select(void)
{
	int ret = spi_transceive(spi, &config, NULL, NULL);

	if (ret) {
		return ret;
	}
	ret = gpio_pin_set_dt(&cs, 1);
	if (ret) {
		(void)spi_release(spi, &config);
	}
	return ret;
}

void nina_bus_deselect(void)
{
	(void)gpio_pin_set_dt(&cs, 0);
	(void)spi_release(spi, &config);
}

/* Between nina_bus_select() and nina_bus_deselect(). */
int nina_bus_transceive(const uint8_t *tx, uint8_t *rx, size_t len)
{
	struct spi_buf tb = {.buf = (void *)tx, .len = len};
	struct spi_buf rb = {.buf = rx, .len = len};
	struct spi_buf_set ts = {.buffers = &tb, .count = 1};
	struct spi_buf_set rs = {.buffers = &rb, .count = 1};

	return rx ? spi_transceive(spi, &config, &ts, &rs) : spi_write(spi, &config, &ts);
}
