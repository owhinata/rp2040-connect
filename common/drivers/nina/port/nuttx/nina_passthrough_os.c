/****************************************************************************
 * common/drivers/nina/port/nuttx/nina_passthrough_os.c
 *
 * SPDX-License-Identifier: MIT
 *
 ****************************************************************************/

/* The NINA passthrough mode on NuttX (nina_passthrough_os.h of the core,
 * next to this binding; the board's CONFIG_NINA_PASSTHROUGH): cdcacm's
 * /dev/ttyACM0 as the host's side of the core
 * (common/drivers/nina/nina_passthrough.c, which explains the mode).  The
 * module's side is nina_passthrough_bus_rp2040.c next to this file, built
 * by the board's src/Make.defs.
 *
 * rp2040_boardinitialize() calls nina_passthrough_check() first thing.
 * With the request, nano_bringup() sets up the USB device (CDC ACM,
 * /dev/ttyACM0, the same descriptors and serial number) and nothing of the
 * NINA, I2C0, the sensor, the network device or the key-value flash; the
 * init task (nuttx/apps/init) calls nina_passthrough_run() instead of
 * starting nsh, Telnet, the Wi-Fi, MQTT and time threads.  The mode:
 *
 * - The lines: cdcacm calls pt_cdc_event() from the USB interrupt at each
 *   SET_CONTROL_LINE_STATE and SET_LINE_CODING (CAIOC_REGISTERCB).  It
 *   reads the new state with CAIOC_GETCTRLLINE and CAIOC_GETLINECODING (two
 *   copies out of cdcacm's own state, safe there) and gives it to the core
 *   at once (nina_passthrough_host_lines() with the rate of the last line
 *   coding, nina_passthrough_host_rate()), so every change of the host is
 *   seen in its order, however short.  The interrupt only counts a close
 *   (the core notes the way out); the init task, waiting on a semaphore,
 *   logs the close, runs the core's tick (GPIO0's release 750 ms after a
 *   start into the module's own firmware, the LED) in a critical section
 *   and leaves.
 * - Module to host: the kernel thread "nina_pt_rx" writes the core's
 *   buffer (16 KiB from the heap, which nothing else uses in this mode) to
 *   /dev/ttyACM0 (a blocking write into the serial driver's buffer, which
 *   cdcacm sends in bulk IN packets) and sleeps on a semaphore that UART1's
 *   interrupt (nina_passthrough_os_uart_irq()) posts when it waits.  While
 *   the host has the port closed (DTR clear), cdcacm drops the output
 *   (nuttx/patches/nuttx/0002); while the device is not configured, the
 *   write fails with ENOTCONN and the bytes are counted as discarded.
 * - Host to module: the kernel thread "nina_pt_tx" reads /dev/ttyACM0 (a
 *   blocking read of the serial driver's 2048-byte receive buffer, up to
 *   256 bytes at once) and puts the bytes into UART1's transmit FIFO; when
 *   it is full, the thread sleeps in nina_passthrough_bus_uart_tx_wait()
 *   until the FIFO has room.  It reads the next bytes only when these are
 *   in the FIFO; until then the serial driver's buffer fills and cdcacm
 *   stops taking packets (the host waits).
 * - The device is raw both ways: /dev/ttyACM0 is not a console, so the
 *   serial driver translates nothing and does not echo (its termios flags
 *   are cleared here once more).
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
#include <nuttx/usb/usb.h>

#include <arch/board/nano_timer.h>

#include "app_log.h"
#include "priorities.h"
#include "nina_passthrough.h"
#include "nina_passthrough_bus.h"
#include "nina_passthrough_os.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PT_USB_PATH          "/dev/ttyACM0"
#define PT_OUT_SIZE          256       /* bytes read from the host at once */
#define PT_USB_RETRY_MS      100u

/* Time for the log to store its last line before a restart. */

#define PT_RESTART_DELAY_MS  100u

/****************************************************************************
 * Private Data
 ****************************************************************************/

LOG_MODULE_REGISTER(nina_passthrough, LOG_LEVEL_INF);

static bool g_active;             /* pt_cdc_event() acts */
static volatile bool g_leaving;   /* leave() has begun */

static struct file g_usb;         /* /dev/ttyACM0, shared by the threads */
static uint32_t g_rate;           /* the host's line coding, as last seen
                                   * (0 before the first: not the exit
                                   * rate) */
static volatile uint32_t g_closes; /* DTR set -> clear: the port closed */

static sem_t g_line_sem;          /* the lines changed */
static sem_t g_rx_sem;            /* bytes in the buffer for nina_pt_rx */
static volatile bool g_rx_waiting;

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
      g_rate = GETUINT32(coding.baud);
      nina_passthrough_host_rate(g_rate);
    }
}

static void host_ctrlline(void)
{
  int ctrlline;

  if (file_ioctl(&g_usb, CAIOC_GETCTRLLINE,
                 (unsigned long)(uintptr_t)&ctrlline) >= 0)
    {
      if (nina_passthrough_host_lines((ctrlline & CDCACM_UART_DTR) != 0,
                                      (ctrlline & CDCACM_UART_RTS) != 0,
                                      g_rate))
        {
          g_closes++;
        }
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

/* "nina_pt_rx": module to host. */

static int pt_rx_main(int argc, FAR char *argv[])
{
  for (; ; )
    {
      if (!nina_passthrough_waiting())
        {
          /* Wait for the interrupt; it may come between the flag and the
           * check, then the wait just returns at once later.
           */

          g_rx_waiting = true;
          if (!nina_passthrough_waiting())
            {
              nxsem_wait_uninterruptible(&g_rx_sem);
            }

          g_rx_waiting = false;
          continue;
        }

      nina_passthrough_to_host();
    }

  return 0;
}

/* "nina_pt_tx": host to module. */

static int pt_tx_main(int argc, FAR char *argv[])
{
  static uint8_t buf[PT_OUT_SIZE];

  for (; ; )
    {
      ssize_t ret = file_read(&g_usb, buf, sizeof(buf));
      uint32_t done = 0;

      if (ret <= 0)
        {
          /* ENOTCONN: the device is not configured; or nothing. */

          nxsched_msleep(PT_USB_RETRY_MS);
          continue;
        }

      for (; ; )
        {
          done += nina_passthrough_to_module(buf + done,
                                             (uint32_t)ret - done);
          if (done == (uint32_t)ret)
            {
              break;
            }

          nina_passthrough_bus_uart_tx_wait();
        }
    }

  return 0;
}

/* The way out, once the core has noted it. */

static void leave(void) noreturn_function;
static void leave(void)
{
  irqstate_t flags = enter_critical_section();

  g_leaving = true;
  nina_passthrough_stop();
  leave_critical_section(flags);

  nina_passthrough_leave();
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* nina_passthrough_os.h */

uint32_t nina_passthrough_os_now_ms(void)
{
  return (uint32_t)(nano_mono_us() / 1000u);
}

void nina_passthrough_os_sleep_ms(uint32_t ms)
{
  nxsched_msleep(ms);
}

int nina_passthrough_os_write(FAR const uint8_t *data, uint32_t len)
{
  ssize_t ret = file_write(&g_usb, data, len);

  /* ENOTCONN: the device is not configured (or another error): the bytes
   * have no reader.
   */

  if (ret > 0)
    {
      return (int)ret;
    }

  return ret < 0 ? (int)ret : -EIO;
}

/* UART1's interrupt: wakes nina_pt_rx when it waits for the buffer. */

void nina_passthrough_os_uart_irq(bool received)
{
  UNUSED(received);

  if (g_rx_waiting && nina_passthrough_waiting())
    {
      g_rx_waiting = false;
      nxsem_post(&g_rx_sem);
    }
}

void nina_passthrough_os_log_start(uint32_t baud)
{
  LOG_INF(NINA_PASSTHROUGH_OS_LOG_START,
          NINA_PASSTHROUGH_OS_LOG_START_ARGS(baud));
}

void nina_passthrough_os_log_counts(
  FAR const char *why, FAR const struct nina_passthrough_os_counts *c)
{
  LOG_INF(NINA_PASSTHROUGH_OS_LOG_COUNTS,
          NINA_PASSTHROUGH_OS_LOG_COUNTS_ARGS(why, c));
}

/* nina_passthrough.h: the mode, in the init task. */

void nina_passthrough_run(void)
{
  struct sched_param param;
  FAR uint8_t *buf;
  uint32_t closes = 0;

  param.sched_priority = NANO_PRIO_NINA_PT;
  nxsched_set_param(0, &param);

  buf = kmm_malloc(NINA_PASSTHROUGH_RING_SIZE);
  if (buf == NULL)
    {
      LOG_ERR("NINA passthrough: no buffer");
      nxsched_msleep(PT_RESTART_DELAY_MS);
      nina_passthrough_bus_restart();
    }

  nxsem_init(&g_line_sem, 0, 0);
  nxsem_init(&g_rx_sem, 0, 0);

  nina_passthrough_hold();
  nina_passthrough_begin(buf);

  usb_initialize();

  if (kthread_create("nina_pt_rx", NANO_PRIO_NINA_PT, NANO_STACK_NINA_PT,
                     pt_rx_main, NULL) < 0 ||
      kthread_create("nina_pt_tx", NANO_PRIO_NINA_PT, NANO_STACK_NINA_PT,
                     pt_tx_main, NULL) < 0)
    {
      LOG_ERR("NINA passthrough: no thread");
    }

  for (; ; )
    {
      irqstate_t flags;
      uint32_t ms;
      bool exit;

      /* The core's tick and the way out against pt_cdc_event(). */

      flags = enter_critical_section();
      ms = nina_passthrough_tick();
      exit = nina_passthrough_exit_requested();
      leave_critical_section(flags);

      if (g_closes != closes)
        {
          closes = g_closes;
          nina_passthrough_log_counts("Port closed");
        }

      if (exit)
        {
          leave();
        }

      nxsem_tickwait_uninterruptible(&g_line_sem, MSEC2TICK(ms));
    }
}
