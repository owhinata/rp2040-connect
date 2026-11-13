/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_gpio.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/* GPIO bank 0 for the Zephyr compatibility layer's gpio0 device
 * (<nano_gpio.h>), as Zephyr's drivers/gpio/gpio_rpi_pico.c drives it on
 * the pico-sdk's hardware_gpio:
 *
 * - gpio_rpi_configure() for an input: gpio_set_pulls(), then
 *   gpio_set_function(GPIO_FUNC_SIO) (pad: input enable on, output disable
 *   off; CTRL: the function, no overrides), then gpio_set_dir(GPIO_IN).
 *   NuttX's rp2040_gpio_set_pulls(), rp2040_gpio_set_function() and
 *   rp2040_gpio_setdir() do the same register writes.
 * - gpio_rpi_pin_interrupt_configure(): gpio_set_irq_enabled(pin, all
 *   events, false), then gpio_set_irq_enabled(pin, events, true); each
 *   clears the events it names in INTR (write-1-to-clear) before changing
 *   PROC0_INTE, so an edge latched while the event was off does not fire.
 * - The bank's interrupt (IO_IRQ_BANK0): Zephyr's gpio_rpi_isr() clears
 *   the pin's events, then fires the callbacks of the pin.  Here NuttX's
 *   rp2040_gpio_interrupt() clears the pending (enabled) events of the
 *   bank, then calls the pin's handler, which calls
 *   nano_gpio_irq_handler().  All interrupts have the same priority on
 *   NuttX (Zephyr gives the GPIO bank priority 3, the lowest, as every
 *   peripheral of the build).
 *
 * The pin's handler is attached with rp2040_gpio_irq_attach() once, the
 * first time it is enabled (that call also attaches and enables
 * IO_IRQ_BANK0); INTE is written here, not through
 * rp2040_gpio_enable_irq()/_disable_irq(), so that events are cleared as
 * pico-sdk clears them.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rp2040_io_bank0.h"
#include "rp2040_gpio.h"

#include "nano_gpio.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_attached;   /* pins whose handler is attached */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int nano_gpio_isr(int irq, void *context, void *arg)
{
  (void)context;
  (void)arg;

  /* rp2040_gpio_interrupt() passes the pin number as irq. */

  nano_gpio_irq_handler((unsigned int)irq);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int nano_gpio_configure_input(unsigned int pin, bool pull_up,
                              bool pull_down)
{
  if (pin >= NANO_GPIO_NUM)
    {
      return -EINVAL;
    }

  rp2040_gpio_set_pulls(pin, pull_up, pull_down);
  rp2040_gpio_set_function(pin, RP2040_GPIO_FUNC_SIO);
  rp2040_gpio_setdir(pin, false);
  return 0;
}

uint32_t nano_gpio_get_all(void)
{
  return getreg32(RP2040_SIO_GPIO_IN);
}

int nano_gpio_irq_configure(unsigned int pin, uint32_t events)
{
  unsigned int shift = (pin % 8) * 4;
  irqstate_t flags;

  if (pin >= NANO_GPIO_NUM || (events & ~NANO_GPIO_IRQ_ALL) != 0)
    {
      return -EINVAL;
    }

  if (events != 0 && (g_attached & (1U << pin)) == 0)
    {
      /* With INTE still clear for the pin.  Clears the latched edges of the
       * pins 8n..8n+7 (setbits_reg32() on INTR); none of the others in
       * GPIO24's group has interrupts.
       */

      rp2040_gpio_irq_attach(pin, events, nano_gpio_isr, NULL);
      g_attached |= 1U << pin;
    }

  flags = enter_critical_section();

  /* gpio_set_irq_enabled(pin, ALL_EVENTS, false) */

  putreg32(NANO_GPIO_IRQ_ALL << shift, RP2040_IO_BANK0_INTR(pin));
  modifyreg32(RP2040_IO_BANK0_PROC_INTE(pin, 0), NANO_GPIO_IRQ_ALL << shift,
              0);

  /* gpio_set_irq_enabled(pin, events, true) */

  if (events != 0)
    {
      putreg32(events << shift, RP2040_IO_BANK0_INTR(pin));
      modifyreg32(RP2040_IO_BANK0_PROC_INTE(pin, 0), 0, events << shift);
    }

  leave_critical_section(flags);
  return 0;
}
