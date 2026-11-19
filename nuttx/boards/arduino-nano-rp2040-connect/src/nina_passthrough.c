/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nina_passthrough.c
 *
 * SPDX-License-Identifier: MIT
 *
 ****************************************************************************/

/* The NINA passthrough mode (<arch/board/nina_passthrough.h>): esptool
 * on the PC talks through the board's USB port to the ESP32 ROM bootloader
 * of the NINA-W102, as WiFiNINA's SerialNINAPassthrough sketch (mbed core)
 * does, without replacing this firmware or touching the flash.  The
 * FreeRTOS build's mode (freertos/drivers/nina/nina_passthrough.c) on
 * NuttX, with its decisions shared (nina_passthrough_logic.h of
 * common/drivers/nina) and the NuttX side of them in
 * nina_passthrough_glue.h.
 *
 * The board (the mbed core's variants/NANO_RP2040_CONNECT; board.h):
 *
 *   RP2040   UART1 in this mode                SPI link of the normal firmware
 *   GPIO8    TX  -> the module's UART0 RX      SPI1 RX (MISO)
 *   GPIO9    RX  <- the module's UART0 TX      CS, active low
 *   GPIO2    the module's GPIO0 (boot strap)   IRQ (driven only at the reset)
 *   GPIO3    the module's RESETN, active low   RESET
 *
 * GPIO8 and GPIO9 are both links at once, so the SPI link must not run:
 * this mode is a start of its own.  "nina passthrough confirm"
 * (nuttx/apps/nina) leaves the request (four words, nina_pt_mark()) in the
 * watchdog's scratch registers 0..3 and restarts through the watchdog
 * (nano_watchdog_reboot(), src/nano_reset.c, which keeps them).
 * rp2040_boardinitialize() calls nina_passthrough_boot() first thing:
 * it reads and clears the request, so the RESET button, any later restart
 * and a fatal error start the normal firmware again.  NuttX itself does not
 * use these registers.  Nothing is written to the flash; the state is in
 * RAM and the scratch registers only.
 *
 * With the request, nano_bringup() sets up the USB device (CDC ACM,
 * /dev/ttyACM0, the same descriptors and serial number) and nothing of the
 * NINA, I2C0, the sensor, the network device or the key-value flash; SPI1
 * keeps the pin functions of rp2040_common_initialize() but is never
 * started.  The init task calls nina_passthrough_run() instead of
 * starting nsh, Telnet, the Wi-Fi, MQTT and time threads.  The mode:
 *
 * - The lines: cdcacm calls pt_cdc_event() from the USB interrupt at each
 *   SET_CONTROL_LINE_STATE and SET_LINE_CODING (CAIOC_REGISTERCB).  It
 *   reads the new state with CAIOC_GETCTRLLINE and CAIOC_GETLINECODING (two
 *   copies out of cdcacm's own state, safe there) and applies it at once,
 *   as the FreeRTOS build's tud_cdc_line_state_cb(): RTS set releases
 *   RESETN, DTR set pulls GPIO0 low (nina_pt_lines_pins()), so every change
 *   of the host is seen in its order, however short.  The UART follows the
 *   line coding's rate (nina_pt_uart_baud(), 8N1 always).  The interrupt
 *   only notes a close and the way out; the init task, waiting on a
 *   semaphore, logs the close, releases GPIO0 750 ms after a start into the
 *   module's own firmware, blinks the LED (GPIO6) at 5 Hz and leaves.
 * - Module to host: UART1's receive interrupt moves every byte from the
 *   32-byte FIFO into a 16 KiB ring (from the heap, which nothing else uses
 *   in this mode; nina_pt_ring_*()).  The interrupt comes at 4 bytes in the
 *   FIFO (RXIFLSEL 1/8) or after 32 bit times without more, so 28 bytes of
 *   margin remain: 300 us at 921600 bps, for the critical sections of the
 *   kernel and the USB interrupt, the only others that run in this mode.
 *   The kernel thread "nina_pt_rx" writes the ring to /dev/ttyACM0 (a
 *   blocking write into the serial driver's buffer, which cdcacm sends in
 *   bulk IN packets) and sleeps on a semaphore that the interrupt posts
 *   when it waits.  The ring holds 178 ms at 921600 bps, for the times the
 *   host does not read; bytes are lost only when it stops reading for that
 *   long, and they are counted.  While the host has the port closed (DTR
 *   clear), cdcacm drops the output (nuttx/patches/nuttx/0002); while the
 *   device is not configured, the write fails with ENOTCONN and the bytes
 *   are counted as discarded.
 * - Host to module: the kernel thread "nina_pt_tx" reads /dev/ttyACM0 (a
 *   blocking read of the serial driver's 2048-byte receive buffer) and
 *   puts the bytes into UART1's transmit FIFO; when it is full, the thread
 *   enables the transmit interrupt (at 4 bytes or fewer, TXIFLSEL 1/8) and
 *   sleeps until it comes.  It reads the next bytes only when these are in
 *   the FIFO; until then the serial driver's buffer fills and cdcacm stops
 *   taking packets (the host waits), so nothing is lost at any rate.
 * - The device is raw both ways: /dev/ttyACM0 is not a console, so the
 *   serial driver translates nothing and does not echo (its termios flags
 *   are cleared here once more).
 * - Way out: closing the port at 300 bps (nina_pt_exit()), or the RESET
 *   button.  Then the module is reset into its own firmware as nina_spi.c's
 *   init_nina() does it (GPIO0 high, RESETN low for 10 ms, 750 ms, GPIO0
 *   released), and the watchdog restarts the board.
 * - The RAM log ("dmesg" after the way out) gets a line when the mode
 *   starts, one each time the port closes and one when the mode ends, with
 *   the counts.
 *
 * No signal is used: the waits are semaphores, sleeps and blocking reads
 * and writes.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/tioctl.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/cdcacm.h>

#include <arch/board/board.h>
#include <arch/board/nina_passthrough.h>
#include <arch/board/nano_reset.h>
#include <arch/board/nano_timer.h>

#include "arm_internal.h"
#include "hardware/rp2040_memorymap.h"
#include "hardware/rp2040_resets.h"
#include "hardware/rp2040_uart01.h"
#include "hardware/rp2040_watchdog.h"
#include "rp2040_gpio.h"
#include "rp2040_uart.h"

#include "app_log.h"
#include "priorities.h"
#include "nano_connect.h"
#include "nina_passthrough_glue.h"

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

#define PT_USB_PATH          "/dev/ttyACM0"
#define PT_RING_SIZE         16384u
#define PT_OUT_SIZE          256       /* bytes read from the host at once */
#define PT_LED_HALF_MS       100u
#define PT_USB_RETRY_MS      100u
#define PT_TX_WAIT_MS        10u       /* bound of a wait for the TX FIFO */

/* init_nina() (common/drivers/nina/nina_spi.c): the reset pulse. */

#define PT_RESET_MS          10u

/* Time for the USB device to send the command's last message, or the last
 * bytes of the mode, before a restart (the FreeRTOS build's
 * PT_RESTART_DELAY_MS).
 */

#define PT_RESTART_DELAY_MS  100u

#if PT_PIN_TX != 8 || PT_PIN_RX != 9
#  error "UART1 of the passthrough is on GPIO8 (TX) and GPIO9 (RX)"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

LOG_MODULE_REGISTER(nina_passthrough, LOG_LEVEL_INF);

static bool g_requested;          /* this start runs the mode */
static bool g_active;             /* pt_cdc_event() acts */
static volatile bool g_leaving;   /* leave() has begun */

static struct file g_usb;         /* /dev/ttyACM0, shared by the threads */
static struct nina_pt_host g_host;
static struct nina_pt_pins g_applied;
static bool g_applied_valid;
static uint32_t g_baud;

static struct nina_pt_ring g_ring;
static struct nina_pt_uart_counts g_counts;
static uint64_t g_to_nina;
static uint64_t g_from_nina;
static uint64_t g_discarded;      /* from the module while USB was down */

static sem_t g_line_sem;          /* the lines changed */
static sem_t g_rx_sem;            /* bytes in the ring for nina_pt_rx */
static sem_t g_tx_sem;            /* room in the TX FIFO for nina_pt_tx */
static volatile bool g_rx_waiting;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t now_ms(void)
{
  return (uint32_t)(nano_mono_us() / 1000u);
}

/* The lines ****************************************************************/

static void set_gpio0(enum nina_pt_gpio0 state)
{
  switch (state)
    {
      case NINA_PT_GPIO0_LOW:
        rp2040_gpio_put(PT_PIN_GPIO0, 0);
        rp2040_gpio_setdir(PT_PIN_GPIO0, true);
        break;

      case NINA_PT_GPIO0_HIGH:
        rp2040_gpio_put(PT_PIN_GPIO0, 1);
        rp2040_gpio_setdir(PT_PIN_GPIO0, true);
        break;

      case NINA_PT_GPIO0_RELEASED:
        rp2040_gpio_put(PT_PIN_GPIO0, 0);
        rp2040_gpio_setdir(PT_PIN_GPIO0, false);
        break;
    }
}

/* GPIO8 to the UART, or a plain input (SIO with the direction in). */

static void set_uart_tx(bool connected)
{
  rp2040_gpio_set_function(PT_PIN_TX, connected ? RP2040_GPIO_FUNC_UART :
                                                  RP2040_GPIO_FUNC_SIO);
}

/* The levels of P, in the order the module needs: GPIO8 and GPIO0 settle
 * before RESETN changes, so the strap is in place when it is released.
 * With interrupts masked (from pt_cdc_event() or in a critical section).
 */

static void apply_pins(struct nina_pt_pins p)
{
  if (!g_applied_valid || p.uart_tx != g_applied.uart_tx)
    {
      set_uart_tx(p.uart_tx);
    }

  if (!g_applied_valid || p.gpio0 != g_applied.gpio0)
    {
      set_gpio0(p.gpio0);
    }

  if (!g_applied_valid || p.reset_released != g_applied.reset_released)
    {
      rp2040_gpio_put(PT_PIN_RESETN, p.reset_released);
    }

  g_applied       = p;
  g_applied_valid = true;
}

static void set_baud(uint32_t baud)
{
  if (baud != g_baud)
    {
      g_baud = baud;
      rp2040_setbaud(PT_UART_BASE, BOARD_PERI_FREQ, baud);
    }
}

static void log_counts(const char *why)
{
  LOG_INF("%s: %llu bytes to the NINA, %llu from it, %llu discarded "
          "without USB, %lu lost (buffer full, peak %lu of %lu), UART: "
          "%lu overruns, %lu breaks, %lu framing errors, %lu bps",
          why, (unsigned long long)g_to_nina,
          (unsigned long long)g_from_nina,
          (unsigned long long)g_discarded, (unsigned long)g_ring.dropped,
          (unsigned long)g_ring.peak, (unsigned long)g_ring.size,
          (unsigned long)g_counts.overruns, (unsigned long)g_counts.breaks,
          (unsigned long)g_counts.framing, (unsigned long)g_baud);
}

static void wake(FAR sem_t *sem)
{
  int value;

  if (nxsem_get_value(sem, &value) >= 0 && value <= 0)
    {
      nxsem_post(sem);
    }
}

/* The host's new line coding and control lines: the USB interrupt
 * (cdcacm's callback), or the init task in a critical section at the
 * start.
 */

static void host_coding(void)
{
  struct cdc_linecoding_s coding;

  if (file_ioctl(&g_usb, CAIOC_GETLINECODING,
                 (unsigned long)(uintptr_t)&coding) >= 0)
    {
      set_baud(nina_pt_host_coding(&g_host, nina_pt_coding_rate(coding.baud),
                                   g_baud));
    }
}

static void host_ctrlline(void)
{
  uint32_t now = now_ms();
  int ctrlline;

  if (file_ioctl(&g_usb, CAIOC_GETCTRLLINE,
                 (unsigned long)(uintptr_t)&ctrlline) >= 0)
    {
      nina_pt_host_ctrlline(&g_host, ctrlline, now);
      apply_pins(nina_pt_lines_pins(&g_host.lines, now));
    }
}

static void pt_cdc_event(enum cdcacm_event_e event)
{
  if (!g_active || g_leaving)
    {
      return;
    }

  switch (event)
    {
      case CDCACM_EVENT_LINECODING:
        host_coding();
        break;

      case CDCACM_EVENT_CTRLLINE:
        host_ctrlline();
        break;

      default:
        return;
    }

  wake(&g_line_sem);
}

/* The data *****************************************************************/

/* UART1's interrupt: the receive FIFO into the ring; the transmit
 * interrupt (enabled by nina_pt_tx only while it waits) wakes nina_pt_tx.
 */

static int pt_uart_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t mis = getreg32(PT_UART(UARTMIS));

  if ((mis & RP2040_UART_UARTMIS_TXMIS) != 0)
    {
      modifyreg32(PT_UART(UARTIMSC), RP2040_UART_UARTIMSC_TXIM, 0);
      wake(&g_tx_sem);
    }

  while ((getreg32(PT_UART(UARTFR)) & RP2040_UART_UARTFR_RXFE) == 0)
    {
      nina_pt_uart_word(&g_ring, &g_counts, getreg32(PT_UART(UARTDR)));
    }

  putreg32(RP2040_UART_UARTICR_RTIC, PT_UART(UARTICR));

  if (g_rx_waiting && g_ring.head != g_ring.tail)
    {
      g_rx_waiting = false;
      nxsem_post(&g_rx_sem);
    }

  return OK;
}

/* "nina_pt_rx": module to host. */

static int pt_rx_main(int argc, FAR char *argv[])
{
  for (; ; )
    {
      FAR const uint8_t *data;
      uint32_t n = nina_pt_ring_peek(&g_ring, &data);
      ssize_t ret;

      if (n == 0)
        {
          /* Wait for the interrupt; it may come between the flag and the
           * check, then the wait just returns at once later.
           */

          g_rx_waiting = true;
          if (g_ring.head == g_ring.tail)
            {
              nxsem_wait_uninterruptible(&g_rx_sem);
            }

          g_rx_waiting = false;
          continue;
        }

      ret = file_write(&g_usb, data, n);
      if (ret > 0)
        {
          nina_pt_ring_consume(&g_ring, (uint32_t)ret);
          g_from_nina += (uint32_t)ret;
        }
      else
        {
          /* ENOTCONN: the device is not configured (or another error):
           * the bytes have no reader.
           */

          nina_pt_ring_consume(&g_ring, n);
          g_discarded += n;
        }
    }

  return 0;
}

/* Puts C into the TX FIFO, waiting for room. */

static void uart_put(uint8_t c)
{
  while ((getreg32(PT_UART(UARTFR)) & RP2040_UART_UARTFR_TXFF) != 0)
    {
      irqstate_t flags = enter_critical_section();
      bool wait;

      /* The transmit interrupt comes when the FIFO falls through its
       * level: enabled while the FIFO is full, it comes once the FIFO has
       * drained to 4 bytes.  The wait is bounded all the same.
       */

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
          nxsem_tickwait_uninterruptible(&g_tx_sem,
                                         MSEC2TICK(PT_TX_WAIT_MS));
          modifyreg32(PT_UART(UARTIMSC), RP2040_UART_UARTIMSC_TXIM, 0);
        }
    }

  putreg32(c, PT_UART(UARTDR));
}

/* "nina_pt_tx": host to module. */

static int pt_tx_main(int argc, FAR char *argv[])
{
  static uint8_t buf[PT_OUT_SIZE];

  for (; ; )
    {
      ssize_t ret = file_read(&g_usb, buf, sizeof(buf));
      ssize_t i;

      if (ret <= 0)
        {
          /* ENOTCONN: the device is not configured; or nothing. */

          nxsched_msleep(PT_USB_RETRY_MS);
          continue;
        }

      for (i = 0; i < ret; i++)
        {
          uart_put(buf[i]);
          g_to_nina++;
        }
    }

  return 0;
}

/* The way out: the module into its own firmware, as init_nina() does it,
 * then the watchdog's restart into the normal firmware.
 */

static void leave(void) noreturn_function;
static void leave(void)
{
  irqstate_t flags = enter_critical_section();

  g_leaving = true;
  up_disable_irq(PT_UART_IRQ);
  leave_critical_section(flags);

  log_counts("Leaving the NINA passthrough");
  set_uart_tx(false);
  set_gpio0(NINA_PT_GPIO0_HIGH);
  rp2040_gpio_put(PT_PIN_RESETN, 0);
  nxsched_msleep(PT_RESET_MS);
  rp2040_gpio_put(PT_PIN_RESETN, 1);
  nxsched_msleep(NINA_PT_GPIO0_HOLD_MS);
  set_gpio0(NINA_PT_GPIO0_RELEASED);
  rp2040_gpio_put(PT_PIN_LED, 0);
  nxsched_msleep(PT_RESTART_DELAY_MS);
  nano_watchdog_reboot();
}

/* UART1: reset, 8N1 at NINA_PT_BAUD_INIT, FIFOs on, no flow control, the
 * interrupt levels at 1/8; the pins as the port closed wants them.
 */

static void hw_initialize(void)
{
  /* The module's lines: held in reset, GPIO0 high (apply_pins() below).
   * RESETN without pulls (Zephyr's reset-gpios), GPIO0 with the pull-down
   * (irq-gpios), as src/nina_bus.c sets them.
   */

  rp2040_gpio_init(PT_PIN_RESETN);
  rp2040_gpio_set_pulls(PT_PIN_RESETN, false, false);
  rp2040_gpio_put(PT_PIN_RESETN, 0);
  rp2040_gpio_setdir(PT_PIN_RESETN, true);
  rp2040_gpio_init(PT_PIN_GPIO0);
  rp2040_gpio_set_pulls(PT_PIN_GPIO0, false, true);

  setbits_reg32(RP2040_RESETS_RESET_UART1, RP2040_RESETS_RESET);
  clrbits_reg32(RP2040_RESETS_RESET_UART1, RP2040_RESETS_RESET);
  while ((getreg32(RP2040_RESETS_RESET_DONE) &
          RP2040_RESETS_RESET_UART1) == 0)
    {
    }

  g_baud = NINA_PT_BAUD_INIT;
  putreg32(RP2040_UART_LCR_H_WLEN(8) | RP2040_UART_UARTLCR_H_FEN,
           PT_UART(UARTLCR_H));
  rp2040_setbaud(PT_UART_BASE, BOARD_PERI_FREQ, g_baud);
  putreg32(0, PT_UART(UARTIFLS));
  putreg32(RP2040_UART_INTR_ALL, PT_UART(UARTICR));
  putreg32(RP2040_UART_UARTCR_UARTEN | RP2040_UART_UARTCR_TXE |
           RP2040_UART_UARTCR_RXE, PT_UART(UARTCR));

  /* GPIO8 an SIO input until apply_pins() connects it; GPIO9 (CS of the
   * normal firmware, an output of rp2040_common_initialize()) the UART's
   * receive pin with the pull-up, so the line idles high while the module
   * is in reset.
   */

  rp2040_gpio_init(PT_PIN_TX);
  rp2040_gpio_set_pulls(PT_PIN_TX, false, false);
  rp2040_gpio_setdir(PT_PIN_RX, false);
  rp2040_gpio_set_function(PT_PIN_RX, RP2040_GPIO_FUNC_UART);
  rp2040_gpio_set_pulls(PT_PIN_RX, true, false);
  apply_pins(nina_pt_lines_pins(&g_host.lines, now_ms()));

  irq_attach(PT_UART_IRQ, pt_uart_interrupt, NULL);
  putreg32(RP2040_UART_UARTIMSC_RXIM | RP2040_UART_UARTIMSC_RTIM,
           PT_UART(UARTIMSC));
  up_enable_irq(PT_UART_IRQ);

  rp2040_gpio_init(PT_PIN_LED);
  rp2040_gpio_setdir(PT_PIN_LED, true);
}

/* /dev/ttyACM0: opened once the host has configured the device, raw,
 * with cdcacm's callback, and the host's state at that time.
 */

static void usb_initialize(void)
{
  struct termios tio;
  irqstate_t flags;

  while (file_open(&g_usb, PT_USB_PATH, O_RDWR) < 0)
    {
      nxsched_msleep(PT_USB_RETRY_MS);
    }

  if (file_ioctl(&g_usb, TCGETS, (unsigned long)(uintptr_t)&tio) >= 0)
    {
      tio.c_iflag = 0;
      tio.c_oflag = 0;
      tio.c_lflag = 0;
      file_ioctl(&g_usb, TCSETS, (unsigned long)(uintptr_t)&tio);
    }

  flags = enter_critical_section();
  g_active = true;
  file_ioctl(&g_usb, CAIOC_REGISTERCB, (unsigned long)(uintptr_t)pt_cdc_event);
  host_coding();
  host_ctrlline();
  leave_critical_section(flags);
}

/* How long the init task may sleep: until the LED's next change at
 * LED_AT, or until GPIO0 is to be released.
 */

static uint32_t wait_ms(uint32_t now, uint32_t led_at)
{
  uint32_t ms = (int32_t)(led_at - now) > 0 ? led_at - now : 1;
  const struct nina_pt_lines *l = &g_host.lines;

  if (l->rts && l->own_firmware && g_applied.gpio0 != NINA_PT_GPIO0_RELEASED)
    {
      uint32_t held = now - l->run_since;
      uint32_t left = held < NINA_PT_GPIO0_HOLD_MS ?
                      NINA_PT_GPIO0_HOLD_MS - held : 1;

      if (left < ms)
        {
          ms = left;
        }
    }

  return ms;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* From rp2040_boardinitialize(), before anything else uses the scratch
 * registers.  The request is cleared at once.
 */

void nina_passthrough_boot(void)
{
  uint32_t s[NINA_PT_MARK_WORDS];
  int i;

  for (i = 0; i < NINA_PT_MARK_WORDS; i++)
    {
      s[i] = getreg32(RP2040_WATCHDOG_SCRATCH(i));
    }

  g_requested = nina_pt_mark_valid(s);
  if (g_requested)
    {
      for (i = 0; i < NINA_PT_MARK_WORDS; i++)
        {
          putreg32(0, RP2040_WATCHDOG_SCRATCH(i));
        }
    }
}

bool nina_passthrough_active(void)
{
  return g_requested;
}

void nina_passthrough_request(void)
{
  uint32_t s[NINA_PT_MARK_WORDS];
  irqstate_t flags;
  int i;

  nina_pt_mark(s);
  flags = enter_critical_section();
  for (i = 0; i < NINA_PT_MARK_WORDS; i++)
    {
      putreg32(s[i], RP2040_WATCHDOG_SCRATCH(i));
    }

  leave_critical_section(flags);
  nano_watchdog_reboot();
}

void nina_passthrough_run(void)
{
  struct sched_param param;
  FAR uint8_t *buf;
  uint32_t closes = 0;
  uint32_t led_at;
  bool led = false;

  param.sched_priority = NANO_PRIO_NINA_PT;
  nxsched_set_param(0, &param);

  buf = kmm_malloc(PT_RING_SIZE);
  if (buf == NULL)
    {
      LOG_ERR("NINA passthrough: no buffer");
      nxsched_msleep(PT_RESTART_DELAY_MS);
      nano_watchdog_reboot();
    }

  nina_pt_ring_init(&g_ring, buf, PT_RING_SIZE);
  nina_pt_host_init(&g_host);
  nxsem_init(&g_line_sem, 0, 0);
  nxsem_init(&g_rx_sem, 0, 0);
  nxsem_init(&g_tx_sem, 0, 0);

  hw_initialize();

  LOG_INF("NINA passthrough: USB <-> UART1 (GPIO8 TX, GPIO9 RX) at %lu bps, "
          "DTR -> GPIO0 (GPIO2), RTS -> RESETN (GPIO3); close the port at "
          "%u bps or press RESET to leave",
          (unsigned long)g_baud, NINA_PT_EXIT_BAUD);

  usb_initialize();

  if (kthread_create("nina_pt_rx", NANO_PRIO_NINA_PT, NANO_STACK_NINA_PT,
                     pt_rx_main, NULL) < 0 ||
      kthread_create("nina_pt_tx", NANO_PRIO_NINA_PT, NANO_STACK_NINA_PT,
                     pt_tx_main, NULL) < 0)
    {
      LOG_ERR("NINA passthrough: no thread");
    }

  led_at = now_ms();
  for (; ; )
    {
      uint32_t now = now_ms();
      irqstate_t flags;
      bool exit;

      if ((int32_t)(now - led_at) >= 0)
        {
          led = !led;
          rp2040_gpio_put(PT_PIN_LED, led);
          led_at += PT_LED_HALF_MS;
          if ((int32_t)(now - led_at) >= 0)
            {
              led_at = now + PT_LED_HALF_MS;
            }
        }

      /* The release of GPIO0 750 ms after a start into the module's own
       * firmware; the rest of the lines is applied by pt_cdc_event().
       */

      flags = enter_critical_section();
      apply_pins(nina_pt_lines_pins(&g_host.lines, now));
      exit = g_host.exit;
      leave_critical_section(flags);

      if (g_host.closes != closes)
        {
          closes = g_host.closes;
          log_counts("Port closed");
        }

      if (exit)
        {
          leave();
        }

      nxsem_tickwait_uninterruptible(&g_line_sem,
                                     MSEC2TICK(wait_ms(now_ms(), led_at)));
    }
}
