/* SPDX-License-Identifier: Apache-2.0 */
/* The kernel services of nina_spi.c on NuttX (see nina_os.h).
 *
 * Zephyr build (drivers/nina/nina_wifi.c):
 * - K_MUTEX_DEFINE(nina_lock): recursive, priority inheritance.  Here an
 *   rmutex_t, whose mutex inherits priority with
 *   CONFIG_PRIORITY_INHERITANCE.  The wait is repeated when a signal ends
 *   it (-EINTR), so that it waits for ever as K_FOREVER does.
 * - k_sleep(K_USEC(100)), k_sleep(K_MSEC(10)), k_sleep(K_MSEC(750)): Zephyr
 *   rounds a relative timeout up to ticks and adds one tick
 *   (kernel/timeout.c, z_add_timeout()).  nxsched_usleep() and
 *   nxsched_msleep() round up (USEC2TICK, MSEC2TICK) and wake at
 *   clock_systime_ticks() + ticks + 1 (clock_delay2abstick()): the same
 *   with the same 100 us tick.  They are not ended by signals; neither is
 *   k_sleep() (only k_wakeup(), which this driver does not use).  The
 *   core of this build does not sleep 100 us (its waits for READY are the
 *   ones below); nina_os_sleep_us() stays for the boundary.
 * - K_MUTEX_DEFINE(nina_dns_lock) of nina_dns.c: a mutex_t, which also
 *   inherits priority.
 * - k_uptime_get(): ticks converted to milliseconds, rounded down
 *   (k_ticks_to_ms_floor64()).  TICK2MSEC() rounds down too.
 *
 * The core's waits for READY (nina_spi.c wait_ready(), compiled with
 * NINA_OS_WAIT_READY, nina/Make.defs) sleep on an edge interrupt of the
 * line instead of sampling it every 100 us as the Zephyr build does
 * (nina_os_ready_*() of nina_os.h):
 *
 *   nina_os_ready_arm(desired)  drops a wake-up left from before
 *                               (nxsem_reset() to 0), then enables the edge
 *                               of READY (GPIO10, active low: nano_nina.c)
 *                               towards DESIRED, falling for 1 (ready),
 *                               rising for 0, through the board's GPIO
 *                               bank 0 (nano_gpio_irq_configure(), the
 *                               board's src/nano_gpio.c: clears an edge
 *                               latched while the event was off, then
 *                               enables it); the first call adds the
 *                               callback to the compatibility layer's gpio0,
 *                               which the LSM6DSOX's INT1 (GPIO24) uses too
 *   the interrupt               IO_IRQ_BANK0: NuttX's rp2040_gpio_interrupt()
 *                               clears the edge and calls the board's
 *                               handler, which runs gpio0's callbacks for
 *                               the pin; ready_changed() posts the
 *                               semaphore (at most one count, as a k_sem
 *                               with limit 1)
 *   nina_os_ready_wait(ms)      nxsem_tickwait_uninterruptible() for ms
 *                               rounded up to ticks, plus one tick (as the
 *                               sleeps).  In one piece: the deadline of the
 *                               core (nina_os_uptime_ms()) and the wait's
 *                               timeout are on the same clock, the tickless
 *                               tick read from the TIMER (the board's
 *                               nano_oneshot.c), which does not fall behind
 *                               while interrupts are masked; the timeout's
 *                               alarm then fires as soon as they are
 *                               unmasked.  (The FreeRTOS build waits in
 *                               slices of 10 ms because its tick does fall
 *                               behind its TIMER deadline.)
 *   nina_os_ready_disarm()      disables the edge (no interrupts while CS
 *                               toggles READY during the transfers)
 *
 * Nothing is lost between the core's read of READY and its wait: the edge
 * is enabled before the read, so a change after the read posts the
 * semaphore, and the wait returns at once.  A change before the read is
 * seen by the read.  A count left from an earlier wait is dropped by the
 * arm (and could only wake a wait early; the core reads the line again
 * after every wake-up).  One waiter at a time: the core calls these with
 * nina_lock held, never from an interrupt handler.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "nano_gpio.h"
#include "nina_os.h"

#ifndef NINA_OS_WAIT_READY
#  error "the core waits for READY with nina_os_ready_*() (nina/Make.defs)"
#endif

/* READY: GPIO10, active low (nano_nina.c, BOARD_NINA_READY_GPIO). */

#define READY_GPIO 10

static rmutex_t g_nina_lock = NXRMUTEX_INITIALIZER;
static mutex_t g_dns_lock = NXMUTEX_INITIALIZER;

/* Posted by the interrupt; no priority inheritance (SEM_INITIALIZER). */

static sem_t g_ready_sem = SEM_INITIALIZER(0);
static struct gpio_callback g_ready_cb;
static bool g_ready_cb_added;

/* In the GPIO bank's interrupt handler. */

static void ready_changed(const struct device *port,
                          struct gpio_callback *cb, gpio_port_pins_t pins)
{
  int value;

  (void)port;
  (void)cb;
  (void)pins;

  if (nxsem_get_value(&g_ready_sem, &value) == 0 && value <= 0)
    {
      nxsem_post(&g_ready_sem);
    }
}

void nina_os_ready_arm(int desired)
{
  /* Active low: ready (1) is the falling edge. */

  uint32_t edge = desired ? NANO_GPIO_IRQ_EDGE_FALL : NANO_GPIO_IRQ_EDGE_RISE;

  if (!g_ready_cb_added)
    {
      gpio_init_callback(&g_ready_cb, ready_changed, BIT(READY_GPIO));
      gpio_add_callback(DEVICE_DT_GET(DT_NODELABEL(gpio0)), &g_ready_cb);
      g_ready_cb_added = true;
    }

  nxsem_reset(&g_ready_sem, 0);
  nano_gpio_irq_configure(READY_GPIO, edge);
}

void nina_os_ready_wait(uint32_t ms)
{
  nxsem_tickwait_uninterruptible(&g_ready_sem, MSEC2TICK(ms));
}

void nina_os_ready_disarm(void)
{
  nano_gpio_irq_configure(READY_GPIO, 0);
}

int nina_os_lock(void)
{
  int ret;

  do
    {
      ret = nxrmutex_lock(&g_nina_lock);
    }
  while (ret == -EINTR);

  return ret;
}

void nina_os_unlock(void)
{
  nxrmutex_unlock(&g_nina_lock);
}

void nina_os_dns_lock(void)
{
  while (nxmutex_lock(&g_dns_lock) == -EINTR)
    {
    }
}

void nina_os_dns_unlock(void)
{
  nxmutex_unlock(&g_dns_lock);
}

void nina_os_sleep_us(uint32_t us)
{
  nxsched_usleep(us);
}

void nina_os_sleep_ms(uint32_t ms)
{
  nxsched_msleep(ms);
}

int64_t nina_os_uptime_ms(void)
{
  return (int64_t)TICK2MSEC(clock_systime_ticks());
}
