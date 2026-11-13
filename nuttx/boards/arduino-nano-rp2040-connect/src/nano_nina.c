/* SPDX-License-Identifier: Apache-2.0 */
/* The board's side of the NINA-W102 SPI link (<nano_nina.h>): SPI1 and the
 * module's lines, set up as the Zephyr build sets them up.
 *
 * Zephyr build (zephyr/boards/arduino/nano_connect/arduino_nano_connect.dts
 * and -pinctrl.dtsi, app/usb-console/app.overlay, prj.conf):
 * - spi1 (PL022), compatible raspberrypi,pico-spi/arm,pl022, polled
 *   (CONFIG_SPI_PL022_INTERRUPT and _DMA are off).  Initialized at boot:
 *   reset, pinctrl, a first configuration, then spi_context_cs_configure_all()
 *   drives cs-gpios to its inactive level (drivers/spi/spi_pl022.c,
 *   spi_pl022_init()).
 * - pinctrl spi1_default: GPIO14 (SCK) and GPIO11 (TX) as SPI, GPIO8 (RX)
 *   as SPI with input-enable.  Zephyr's RP2040 pinctrl
 *   (drivers/pinctrl/pinctrl_rpi_pico.c) sets every pad property: no pulls,
 *   4 mA, slow slew, Schmitt trigger off, input enable only where asked,
 *   no overrides.
 * - nina_w102: cs-gpios GPIO9 active low; ready-gpios GPIO10 active low
 *   with pull-up; reset-gpios GPIO3 active low; irq-gpios GPIO2 with
 *   pull-down.  nina_wifi.c configures READY, RESET and IRQ only on the
 *   first request.
 * - The transfer: SPI_WORD_SET(8) | SPI_TRANSFER_MSB, so mode 0 (CPOL 0,
 *   CPHA 0), MSB first, at CONFIG_NINA_SPI_FREQUENCY (8 MHz).  The PL022
 *   divides clk_peri (125 MHz): Zephyr picks CPSDVSR 2, SCR 7, NuttX's
 *   rp2040_spi.c CPSDVSR 16, SCR 0; both give 125 MHz / 16 = 7.8125 MHz.
 *
 * Here: NuttX's SPI1 driver (arch/arm/src/rp2040/rp2040_spi.c, polled with
 * RP2040_SPI_DMA off).  boards/arm/rp2040/common sets the pin functions and
 * makes CONFIG_RP2040_SPI1_CS_GPIO an output at 1 before this runs
 * (rp2040_common_initialize()); nano_nina_initialize() then sets the pads
 * as Zephyr's pinctrl and GPIO drivers do.  Each nano_nina_transceive()
 * takes the bus lock (SPI_LOCK) like each Zephyr spi_transceive() takes its
 * context lock, and sets mode, word size and frequency the first time,
 * like Zephyr's spi_pl022_configure() does when the configuration changes
 * (no other user of SPI1).  SPI_SELECT is not used: nina_spi.c drives CS
 * itself, as nina_wifi.c does (its spi_config has no CS).
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rp2040_pads_bank0.h"
#include "rp2040_gpio.h"
#include "rp2040_spi.h"

#include "nano_nina.h"

#if CONFIG_RP2040_SPI1_SCK_GPIO != BOARD_NINA_SPI1_SCK_GPIO || \
    CONFIG_RP2040_SPI1_TX_GPIO != BOARD_NINA_SPI1_TX_GPIO || \
    CONFIG_RP2040_SPI1_RX_GPIO != BOARD_NINA_SPI1_RX_GPIO || \
    CONFIG_RP2040_SPI1_CS_GPIO != BOARD_NINA_CS_GPIO
#  error "CONFIG_RP2040_SPI1_*_GPIO must match BOARD_NINA_* in board.h"
#endif

struct nina_line
{
  uint8_t gpio;
  bool    active_low;
  bool    pull_up;
  bool    pull_down;
};

static const struct nina_line g_lines[] =
{
  [NANO_NINA_CS]    = { BOARD_NINA_CS_GPIO,    true,  false, false },
  [NANO_NINA_READY] = { BOARD_NINA_READY_GPIO, true,  true,  false },
  [NANO_NINA_RESET] = { BOARD_NINA_RESET_GPIO, true,  false, false },
  [NANO_NINA_IRQ]   = { BOARD_NINA_IRQ_GPIO,   false, false, true  },
};

static struct spi_dev_s *g_spi;
static bool g_configured;

static void set_input_enabled(uint32_t gpio, bool enabled)
{
  modbits_reg32(enabled ? RP2040_PADS_BANK0_GPIO_IE : 0,
                RP2040_PADS_BANK0_GPIO_IE, RP2040_PADS_BANK0_GPIO(gpio));
}

/* pinctrl_configure_pin() of Zephyr's RP2040 pinctrl for an SPI pin with
 * the devicetree's defaults (the overrides stay at 0: set_function()
 * writes the whole CTRL register).
 */

static void spi_pin(uint32_t gpio, bool input_enable)
{
  rp2040_gpio_init(gpio);
  rp2040_gpio_set_function(gpio, RP2040_GPIO_FUNC_SPI);
  rp2040_gpio_set_pulls(gpio, false, false);
  rp2040_gpio_set_drive_strength(gpio, RP2040_PADS_BANK0_GPIO_DRIVE_4MA);
  rp2040_gpio_set_slew_fast(gpio, false);
  rp2040_gpio_set_input_hysteresis_enabled(gpio, false);
  set_input_enabled(gpio, input_enable);
}

int nano_nina_initialize(void)
{
  spi_pin(BOARD_NINA_SPI1_RX_GPIO, true);
  spi_pin(BOARD_NINA_SPI1_TX_GPIO, false);
  spi_pin(BOARD_NINA_SPI1_SCK_GPIO, false);

  g_spi = rp2040_spibus_initialize(1);
  if (g_spi == NULL)
    {
      return -ENODEV;
    }

  return nano_nina_pin_configure(NANO_NINA_CS, NANO_NINA_OUTPUT_INACTIVE);
}

bool nano_nina_bus_ready(void)
{
  return g_spi != NULL;
}

/* gpio_rpi_configure() (Zephyr drivers/gpio/gpio_rpi_pico.c) for a
 * push-pull line, after gpio_pin_configure_dt() has folded the
 * devicetree flags in: pulls, SIO function (input enable on, output
 * disable off), input enable off unless an input, the initial level, then
 * the direction.
 */

int nano_nina_pin_configure(enum nano_nina_pin pin, enum nano_nina_mode mode)
{
  const struct nina_line *line;
  bool input = mode == NANO_NINA_INPUT;
  int level;

  if ((unsigned int)pin >= sizeof(g_lines) / sizeof(g_lines[0]))
    {
      return -EINVAL;
    }

  line = &g_lines[pin];
  rp2040_gpio_set_pulls(line->gpio, line->pull_up, line->pull_down);
  rp2040_gpio_set_function(line->gpio, RP2040_GPIO_FUNC_SIO);
  if (input)
    {
      rp2040_gpio_setdir(line->gpio, false);
      return 0;
    }

  set_input_enabled(line->gpio, false);
  switch (mode)
    {
      case NANO_NINA_OUTPUT_INACTIVE:
        level = line->active_low;
        break;
      case NANO_NINA_OUTPUT_ACTIVE:
        level = !line->active_low;
        break;
      case NANO_NINA_OUTPUT_HIGH:
        level = 1;
        break;
      default:
        return -EINVAL;
    }

  rp2040_gpio_put(line->gpio, level);
  rp2040_gpio_setdir(line->gpio, true);
  return 0;
}

int nano_nina_pin_set(enum nano_nina_pin pin, int value)
{
  const struct nina_line *line = &g_lines[pin];

  rp2040_gpio_put(line->gpio, (value != 0) != line->active_low);
  return 0;
}

int nano_nina_pin_get(enum nano_nina_pin pin)
{
  const struct nina_line *line = &g_lines[pin];

  return rp2040_gpio_get(line->gpio) != line->active_low;
}

int nano_nina_transceive(const uint8_t *tx, uint8_t *rx, size_t len)
{
  int ret;

  /* Wait for ever, as Zephyr's spi_context_lock() (K_FOREVER) does: a
   * signal ends the mutex wait with -EINTR.
   */

  do
    {
      ret = SPI_LOCK(g_spi, true);
    }
  while (ret == -EINTR);

  if (ret < 0)
    {
      return ret;
    }

  if (!g_configured)
    {
      SPI_SETMODE(g_spi, SPIDEV_MODE0);
      SPI_SETBITS(g_spi, 8);
      SPI_SETFREQUENCY(g_spi, CONFIG_NANO_NINA_SPI_FREQUENCY);
      g_configured = true;
    }

  SPI_EXCHANGE(g_spi, tx, rx, len);
  SPI_LOCK(g_spi, false);
  return 0;
}
