/****************************************************************************
 * common/drivers/nina/port/nuttx/nina_passthrough_bus_rp2040.c
 *
 * SPDX-License-Identifier: MIT
 *
 ****************************************************************************/

/* The module's side of the NINA passthrough mode on the board
 * (nina_passthrough_bus.h of common/drivers/nina/port): the watchdog's
 * scratch registers and its restart, the module's lines, UART1 and the LED,
 * on the RP2040's registers and NuttX's rp2040_gpio and rp2040_uart
 * functions.
 *
 * The board (the mbed core's variants/NANO_RP2040_CONNECT; board.h):
 *
 *   RP2040   UART1 in this mode                SPI link of the normal firmware
 *   GPIO8    TX  -> the module's UART0 RX      SPI1 RX (MISO)
 *   GPIO9    RX  <- the module's UART0 TX      CS, active low
 *   GPIO2    the module's GPIO0 (boot strap)   IRQ (driven only at the reset)
 *   GPIO3    the module's RESETN, active low   RESET
 *
 * In this mode nano_bringup() starts the USB device only; SPI1 keeps the
 * pin functions of rp2040_common_initialize() but is never started.  NuttX
 * itself does not use the scratch registers; the restart is
 * nano_watchdog_reboot() (src/nano_reset.c), which keeps them.  UART1's
 * interrupt comes at 4 bytes in the receive FIFO (RXIFLSEL 1/8) or after
 * 32 bit times without more; its handler runs the core's receive path,
 * with the critical sections of the kernel and the USB interrupt the only
 * others that run in this mode.  The binding's "nina_pt_tx" thread writes
 * the transmit FIFO and, when it is full, sleeps in
 * nina_passthrough_bus_uart_tx_wait() until the transmit interrupt (at 4
 * bytes or fewer, TXIFLSEL 1/8) comes; this build has no
 * nina_passthrough_bus_uart_tx_irq().
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>

#include <arch/board/board.h>
#include <arch/board/nano_reset.h>

#include "arm_internal.h"
#include "hardware/rp2040_memorymap.h"
#include "hardware/rp2040_resets.h"
#include "hardware/rp2040_uart01.h"
#include "hardware/rp2040_watchdog.h"
#include "rp2040_gpio.h"
#include "rp2040_uart.h"

#include "nina_passthrough.h"
#include "nina_passthrough_bus.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PT_UART_BASE         RP2040_UART1_BASE
#define PT_UART_IRQ          RP2040_UART1_IRQ
#define PT_UART(reg)         (PT_UART_BASE + RP2040_UART_##reg##_OFFSET)
#define PT_PIN_TX            BOARD_NINA_SPI1_RX_GPIO  /* 8, UART1 TX */
#define PT_PIN_RX            BOARD_NINA_CS_GPIO       /* 9, UART1 RX */
#define PT_PIN_GPIO0         BOARD_NINA_IRQ_GPIO      /* 2 */
#define PT_PIN_RESETN        BOARD_NINA_RESET_GPIO    /* 3 */
#define PT_PIN_LED           BOARD_GPIO_LED_PIN       /* 6 */

#define PT_TX_WAIT_MS        10u       /* bound of a wait for the TX FIFO */

#if PT_PIN_TX != 8 || PT_PIN_RX != 9
#  error "UART1 of the passthrough is on GPIO8 (TX) and GPIO9 (RX)"
#endif

#if RP2040_UART_UARTDR_FE != NINA_PASSTHROUGH_BUS_UART_FRAMING || \
    RP2040_UART_UARTDR_BE != NINA_PASSTHROUGH_BUS_UART_BREAK || \
    RP2040_UART_UARTDR_OE != NINA_PASSTHROUGH_BUS_UART_OVERRUN
#  error "a received word is UARTDR as read"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_tx_sem;            /* room in the TX FIFO for nina_pt_tx */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void wake(FAR sem_t *sem)
{
  int value;

  if (nxsem_get_value(sem, &value) >= 0 && value <= 0)
    {
      nxsem_post(sem);
    }
}

/* UART1's interrupt: the transmit interrupt (enabled by nina_pt_tx only
 * while it waits) wakes nina_pt_tx; the receive FIFO goes into the core's
 * buffer.
 */

static int pt_uart_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t mis = getreg32(PT_UART(UARTMIS));

  if ((mis & RP2040_UART_UARTMIS_TXMIS) != 0)
    {
      modifyreg32(PT_UART(UARTIMSC), RP2040_UART_UARTIMSC_TXIM, 0);
      wake(&g_tx_sem);
    }

  nina_passthrough_uart_isr();
  putreg32(RP2040_UART_UARTICR_RTIC, PT_UART(UARTICR));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nina_passthrough_bus_mark_get(
  uint32_t s[NINA_PASSTHROUGH_BUS_MARK_WORDS])
{
  int i;

  for (i = 0; i < NINA_PASSTHROUGH_BUS_MARK_WORDS; i++)
    {
      s[i] = getreg32(RP2040_WATCHDOG_SCRATCH(i));
    }
}

void nina_passthrough_bus_mark_set(
  const uint32_t s[NINA_PASSTHROUGH_BUS_MARK_WORDS])
{
  irqstate_t flags = enter_critical_section();
  int i;

  for (i = 0; i < NINA_PASSTHROUGH_BUS_MARK_WORDS; i++)
    {
      putreg32(s[i], RP2040_WATCHDOG_SCRATCH(i));
    }

  leave_critical_section(flags);
}

void nina_passthrough_bus_restart(void)
{
  nano_watchdog_reboot();
}

/* RESETN without pulls (Zephyr's reset-gpios), driven low; GPIO0 with the
 * pull-down (irq-gpios), as nina_bus_rp2040.c sets them.
 */

void nina_passthrough_bus_lines_init(void)
{
  rp2040_gpio_init(PT_PIN_RESETN);
  rp2040_gpio_set_pulls(PT_PIN_RESETN, false, false);
  rp2040_gpio_put(PT_PIN_RESETN, 0);
  rp2040_gpio_setdir(PT_PIN_RESETN, true);
  rp2040_gpio_init(PT_PIN_GPIO0);
  rp2040_gpio_set_pulls(PT_PIN_GPIO0, false, true);
}

void nina_passthrough_bus_resetn(bool released)
{
  rp2040_gpio_put(PT_PIN_RESETN, released);
}

void nina_passthrough_bus_gpio0(enum nina_passthrough_bus_gpio0 state)
{
  switch (state)
    {
      case NINA_PASSTHROUGH_BUS_GPIO0_LOW:
        rp2040_gpio_put(PT_PIN_GPIO0, 0);
        rp2040_gpio_setdir(PT_PIN_GPIO0, true);
        break;

      case NINA_PASSTHROUGH_BUS_GPIO0_HIGH:
        rp2040_gpio_put(PT_PIN_GPIO0, 1);
        rp2040_gpio_setdir(PT_PIN_GPIO0, true);
        break;

      case NINA_PASSTHROUGH_BUS_GPIO0_RELEASED:
        rp2040_gpio_put(PT_PIN_GPIO0, 0);
        rp2040_gpio_setdir(PT_PIN_GPIO0, false);
        break;
    }
}

/* GPIO8 to the UART, or a plain input (SIO with the direction in). */

void nina_passthrough_bus_uart_tx_connect(bool connected)
{
  rp2040_gpio_set_function(PT_PIN_TX, connected ? RP2040_GPIO_FUNC_UART :
                                                  RP2040_GPIO_FUNC_SIO);
}

/* UART1: reset, 8N1 at BAUD, FIFOs on, no flow control, the interrupt
 * levels at 1/8.  GPIO8 an SIO input until
 * nina_passthrough_bus_uart_tx_connect();
 * GPIO9 (CS of the normal firmware, an output of
 * rp2040_common_initialize()) the UART's receive pin with the pull-up, so
 * the line idles high while the module is in reset.
 */

void nina_passthrough_bus_uart_init(uint32_t baud)
{
  nxsem_init(&g_tx_sem, 0, 0);

  setbits_reg32(RP2040_RESETS_RESET_UART1, RP2040_RESETS_RESET);
  clrbits_reg32(RP2040_RESETS_RESET_UART1, RP2040_RESETS_RESET);
  while ((getreg32(RP2040_RESETS_RESET_DONE) &
          RP2040_RESETS_RESET_UART1) == 0)
    {
    }

  putreg32(RP2040_UART_LCR_H_WLEN(8) | RP2040_UART_UARTLCR_H_FEN,
           PT_UART(UARTLCR_H));
  rp2040_setbaud(PT_UART_BASE, BOARD_PERI_FREQ, baud);
  putreg32(0, PT_UART(UARTIFLS));
  putreg32(RP2040_UART_INTR_ALL, PT_UART(UARTICR));
  putreg32(RP2040_UART_UARTCR_UARTEN | RP2040_UART_UARTCR_TXE |
           RP2040_UART_UARTCR_RXE, PT_UART(UARTCR));

  rp2040_gpio_init(PT_PIN_TX);
  rp2040_gpio_set_pulls(PT_PIN_TX, false, false);
  rp2040_gpio_setdir(PT_PIN_RX, false);
  rp2040_gpio_set_function(PT_PIN_RX, RP2040_GPIO_FUNC_UART);
  rp2040_gpio_set_pulls(PT_PIN_RX, true, false);
}

void nina_passthrough_bus_uart_set_baud(uint32_t baud)
{
  rp2040_setbaud(PT_UART_BASE, BOARD_PERI_FREQ, baud);
}

void nina_passthrough_bus_uart_irq_on(void)
{
  irq_attach(PT_UART_IRQ, pt_uart_interrupt, NULL);
  putreg32(RP2040_UART_UARTIMSC_RXIM | RP2040_UART_UARTIMSC_RTIM,
           PT_UART(UARTIMSC));
  up_enable_irq(PT_UART_IRQ);
}

void nina_passthrough_bus_uart_irq_off(void)
{
  up_disable_irq(PT_UART_IRQ);
}

bool nina_passthrough_bus_uart_rx(uint32_t *word)
{
  if ((getreg32(PT_UART(UARTFR)) & RP2040_UART_UARTFR_RXFE) != 0)
    {
      return false;
    }

  *word = getreg32(PT_UART(UARTDR));
  return true;
}

bool nina_passthrough_bus_uart_tx(uint8_t c)
{
  if ((getreg32(PT_UART(UARTFR)) & RP2040_UART_UARTFR_TXFF) != 0)
    {
      return false;
    }

  putreg32(c, PT_UART(UARTDR));
  return true;
}

/* From nina_pt_tx while the TX FIFO is full.  The transmit interrupt
 * comes when the FIFO falls through its level: enabled while the FIFO is
 * full, it comes once the FIFO has drained to 4 bytes.  The wait is
 * bounded all the same.
 */

void nina_passthrough_bus_uart_tx_wait(void)
{
  irqstate_t flags = enter_critical_section();
  bool wait;

  nxsem_reset(&g_tx_sem, 0);
  putreg32(RP2040_UART_UARTICR_TXIC, PT_UART(UARTICR));
  wait = (getreg32(PT_UART(UARTFR)) & RP2040_UART_UARTFR_TXFF) != 0;
  if (wait)
    {
      modifyreg32(PT_UART(UARTIMSC), 0, RP2040_UART_UARTIMSC_TXIM);
    }

  leave_critical_section(flags);

  if (wait)
    {
      nxsem_tickwait_uninterruptible(&g_tx_sem, MSEC2TICK(PT_TX_WAIT_MS));
      modifyreg32(PT_UART(UARTIMSC), RP2040_UART_UARTIMSC_TXIM, 0);
    }
}

void nina_passthrough_bus_led_init(void)
{
  rp2040_gpio_init(PT_PIN_LED);
  rp2040_gpio_setdir(PT_PIN_LED, true);
}

void nina_passthrough_bus_led(bool on)
{
  rp2040_gpio_put(PT_PIN_LED, on);
}
