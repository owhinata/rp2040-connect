/* SPDX-License-Identifier: MIT */
/* The board's side of the NINA-W102 SPI link on the Pico SDK
 * (common/drivers/nina/port/nina_bus.h): SPI1 and the module's four lines, set
 * up and driven as the Zephyr build's drivers do it.  The Zephyr build's
 * RP2040 drivers are written on the same hardware_gpio functions, so the
 * calls below are theirs, in their order.
 *
 * Zephyr build (zephyr/boards/arduino/nano_connect/arduino_nano_connect.dts
 * 144-159 and -pinctrl.dtsi 40-50, app/usb-console/app.overlay, Kconfig
 * NINA_SPI_FREQUENCY, build/usb-console/zephyr/.config):
 * - spi1, the PL022 driver (drivers/spi/spi_pl022.c), polled:
 *   CONFIG_SPI_PL022_INTERRUPT and _DMA are not set.  At boot
 *   (spi_pl022_init(), SPI init priority 50, before main()): the reset line
 *   toggled, the pinctrl state, a first configuration, then the cs-gpios
 *   driven inactive (spi_context_cs_configure_all()).
 * - pinctrl spi1_default: GPIO14 (SCK) and GPIO11 (TX), then GPIO8 (RX)
 *   with input-enable; pinctrl_configure_pin() (drivers/pinctrl/
 *   pinctrl_rpi_pico.c) sets every pad property: no pulls, 4 mA, slow
 *   slew, Schmitt trigger off, input enable only where asked, no overrides.
 * - nina_w102: cs-gpios GPIO9 active low; ready-gpios GPIO10 active low with
 *   pull-up; reset-gpios GPIO3 active low; irq-gpios GPIO2 with pull-down.
 *   nina_proto.c configures READY, RESET and IRQ only at the first request
 *   (common/drivers/nina/internal/nina_proto.c, init_nina()).
 * - Each transfer: SPI_WORD_SET(8) | SPI_TRANSFER_MSB, so mode 0 (CPOL 0,
 *   CPHA 0), MSB first, at CONFIG_NINA_SPI_FREQUENCY (8 MHz), which the
 *   PL022 makes from clk_peri (125 MHz, clk_sys) as CPSDVSR 2, SCR 7:
 *   7.8125 MHz.  spi_pl022_xfer() polls the FIFOs in the caller's thread, at
 *   most 8 bytes in flight, until every byte has come back.
 *
 * The NuttX build (common/drivers/nina/port/nuttx/nina_bus_rp2040.c)
 * does the same on its own SPI driver.  Here:
 *
 * - nina_bus_initialize(): the reset of SPI1 (as spi_init() does), the
 *   pins as pinctrl does, the format and the rate at once (Zephyr's first
 *   configuration at boot is its lowest rate and its 8 MHz configuration
 *   comes with the first transfer; nothing is sent in between, so the
 *   module sees the same), SSE on, and CS inactive.  The SDK's
 *   spi_set_baudrate() picks the PL022's dividers as Zephyr's
 *   spi_pl022_configure() does (the smallest even prescale, then the largest
 *   post-divider at or below the rate): CPSDVSR 2, SCR 7 from the SDK's
 *   clk_peri of 125 MHz (SYS_CLK_HZ; "devmem 40040000" reads CR0 0x707,
 *   "devmem 40040010" CPSR 0x2).  Unlike spi_init(), the DMA requests
 *   (SSPDMACR) stay off, as in Zephyr: only a transfer by DMA sets them,
 *   for its length.  Then the two DMA channels of the long transfers
 *   (nina_bus_dma_initialize(), nina_bus_dma_rp2040.c).  The Pico SDK's
 *   start-up code leaves these pins at their reset state (the board header,
 *   pico-sdk/src/boards/include/boards/arduino_nano_rp2040_connect.h, names
 *   none of them; stdio is USB only); so do the Zephyr and NuttX builds
 *   until this point.
 * - nina_bus_transceive(): a transfer of at least NINA_BUS_DMA_MIN (64)
 *   bytes by DMA while the calling task sleeps (nina_bus_dma_rp2040.c: the
 *   same bytes on the wire, without the gaps of the polling, and the call
 *   returns once every byte has come back, or -EIO).  A shorter one, or
 *   any before the scheduler runs or without the two channels, by the SDK's
 *   spi_write_read_blocking() (both buffers) or spi_write_blocking() (no
 *   RX buffer: received bytes are dropped and it waits until the shifter is
 *   idle), which poll the FIFOs in the calling task with at most 8 bytes in
 *   flight, as spi_pl022_xfer() does: the CPU spins for the length of the
 *   transfer (about 1 us a byte at 7.8125 MHz), at the caller's priority,
 *   and interrupts are served meanwhile (the PL022 holds the clock when its
 *   TX FIFO runs empty).  The core sends mostly single bytes, for which the
 *   DMA would cost more than it saves; the Zephyr and NuttX builds poll
 *   every transfer.  It takes no lock of its
 *   own: Zephyr's spi_transceive() takes the bus's context lock, but SPI1
 *   has no other user and nina_proto.c calls this only with nina_lock held
 *   (nina_os.h).  CS is not touched (nina_proto.c drives it: the Zephyr
 *   build's spi_config has no CS).
 * - nina_bus_pin_configure(): gpio_rpi_configure() of Zephyr's RP2040 GPIO
 *   driver (drivers/gpio/gpio_rpi_pico.c) for a push-pull line after
 *   gpio_pin_configure_dt() has folded in the devicetree flags: the pulls,
 *   the SIO function (which sets input enable and clears output disable),
 *   input enable off unless an input, then the level and the direction.
 * - nina_bus_pin_set() / _get(): gpio_put() / gpio_get() with the line's
 *   polarity (gpio_pin_set_dt() / gpio_pin_get_dt()).  The pins are on SIO,
 *   so no other task's GPIO calls interfere: SIO's set and clear registers
 *   change only the bits written.
 *
 * No GPIO interrupt is used here: the core's FreeRTOS binding
 * (common/drivers/nina/port/freertos/nina_os.c) waits for READY's edges
 * (GPIO10, active low, as in lines[] below) on its own handler of the GPIO
 * bank's interrupt, as the Zephyr and NuttX builds wait on READY's
 * interrupt; the line's configuration here is the same as theirs.  IRQ is
 * only driven at the reset, as in both builds; afterwards the binding may
 * enable its rising edge (the module's socket events, the same handler).
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/resets.h"
#include "hardware/spi.h"

#include "nina_board_rp2040.h"
#include "nina_bus.h"
#include "nina_bus_dma_rp2040.h"

/* CONFIG_NINA_SPI_FREQUENCY of the Zephyr and NuttX builds (their Kconfig
 * default).
 */
#ifndef CONFIG_NINA_SPI_FREQUENCY
#define CONFIG_NINA_SPI_FREQUENCY 8000000
#endif

struct nina_line {
	uint8_t gpio;
	bool active_low;
	bool pull_up;
	bool pull_down;
};

static const struct nina_line lines[] = {
	[NINA_BUS_PIN_CS] = {NINA_CS_GPIO, true, false, false},
	[NINA_BUS_PIN_READY] = {NINA_READY_GPIO, true, true, false},
	[NINA_BUS_PIN_RESET] = {NINA_RESET_GPIO, true, false, false},
	[NINA_BUS_PIN_IRQ] = {NINA_IRQ_GPIO, false, false, true},
};

static bool bus_ready;

/* pinctrl_configure_pin() of Zephyr's RP2040 pinctrl for an SPI pin with
 * the devicetree's defaults.
 */
static void spi_pin(unsigned int gpio, bool input_enable)
{
	gpio_init(gpio);
	gpio_set_function(gpio, GPIO_FUNC_SPI);
	gpio_set_pulls(gpio, false, false);
	gpio_set_drive_strength(gpio, GPIO_DRIVE_STRENGTH_4MA);
	gpio_set_slew_rate(gpio, GPIO_SLEW_RATE_SLOW);
	gpio_set_input_hysteresis_enabled(gpio, false);
	gpio_set_input_enabled(gpio, input_enable);
	gpio_set_oeover(gpio, GPIO_OVERRIDE_NORMAL);
	gpio_set_outover(gpio, GPIO_OVERRIDE_NORMAL);
	gpio_set_inover(gpio, GPIO_OVERRIDE_NORMAL);
	gpio_set_irqover(gpio, GPIO_OVERRIDE_NORMAL);
}

int nina_bus_initialize(void)
{
	if (bus_ready) {
		return 0;
	}
	reset_block_num(NINA_SPI_RESET);
	unreset_block_num_wait_blocking(NINA_SPI_RESET);

	spi_pin(NINA_SPI_SCK_GPIO, false);
	spi_pin(NINA_SPI_TX_GPIO, false);
	spi_pin(NINA_SPI_RX_GPIO, true);

	(void)spi_set_baudrate(NINA_SPI, CONFIG_NINA_SPI_FREQUENCY);
	spi_set_format(NINA_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
	hw_set_bits(&spi_get_hw(NINA_SPI)->cr1, SPI_SSPCR1_SSE_BITS);
	nina_bus_dma_initialize();
	bus_ready = true;

	return nina_bus_pin_configure(NINA_BUS_PIN_CS, NINA_BUS_PIN_OUTPUT_INACTIVE);
}

int nina_bus_pin_configure(enum nina_bus_pin pin, enum nina_bus_pin_mode mode)
{
	const struct nina_line *line;
	bool level;

	if ((unsigned int)pin >= sizeof(lines) / sizeof(lines[0])) {
		return -EINVAL;
	}
	switch (mode) {
	case NINA_BUS_PIN_OUTPUT_INACTIVE:
		level = lines[pin].active_low;
		break;
	case NINA_BUS_PIN_OUTPUT_ACTIVE:
		level = !lines[pin].active_low;
		break;
	case NINA_BUS_PIN_OUTPUT_HIGH:
		level = true;
		break;
	case NINA_BUS_PIN_INPUT:
		level = false;
		break;
	default:
		return -EINVAL;
	}

	line = &lines[pin];
	gpio_set_pulls(line->gpio, line->pull_up, line->pull_down);
	gpio_set_function(line->gpio, GPIO_FUNC_SIO);
	if (mode == NINA_BUS_PIN_INPUT) {
		gpio_set_dir(line->gpio, GPIO_IN);
		return 0;
	}
	gpio_set_input_enabled(line->gpio, false);
	gpio_put(line->gpio, level);
	gpio_set_dir(line->gpio, GPIO_OUT);
	return 0;
}

int nina_bus_pin_set(enum nina_bus_pin pin, int value)
{
	const struct nina_line *line = &lines[pin];

	gpio_put(line->gpio, (value != 0) != line->active_low);
	return 0;
}

int nina_bus_pin_get(enum nina_bus_pin pin)
{
	const struct nina_line *line = &lines[pin];

	return gpio_get(line->gpio) != line->active_low;
}

/* SPI1 has no other user here and the Pico SDK no bus lock: selecting is
 * the chip select alone (nina_bus_initialize() set the mode and the rate).
 */
int nina_bus_select(void)
{
	return nina_bus_pin_set(NINA_BUS_PIN_CS, 1);
}

void nina_bus_deselect(void)
{
	(void)nina_bus_pin_set(NINA_BUS_PIN_CS, 0);
}

int nina_bus_transceive(const uint8_t *tx, uint8_t *rx, size_t len)
{
	if (len == 0) {
		return 0;
	}
	if (nina_bus_dma_wanted(len)) {
		return nina_bus_dma_transceive(tx, rx, len);
	}
	if (rx != NULL) {
		(void)spi_write_read_blocking(NINA_SPI, tx, rx, len);
	} else {
		(void)spi_write_blocking(NINA_SPI, tx, len);
	}
	return 0;
}
