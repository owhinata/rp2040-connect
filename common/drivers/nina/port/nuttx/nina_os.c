/* SPDX-License-Identifier: Apache-2.0 */
/* The kernel services of the shared NINA core on NuttX (see nina_os.h).
 *
 * Zephyr build (common/drivers/nina/port/zephyr/nina_os.c):
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
 *   core sleeps 100 us only when nina_os_ready_start() below fails (its
 *   waits for READY then sample the line).
 * - K_MUTEX_DEFINE(nina_dns_lock) of nina_dns.c: a mutex_t, which also
 *   inherits priority.
 * - k_uptime_get(): ticks converted to milliseconds, rounded down
 *   (k_ticks_to_ms_floor64()).  TICK2MSEC() rounds down too.
 * - K_MUTEX_DEFINE(connect_state_lock) of nina_wifi.c (with
 *   CONFIG_NINA_NET): an rmutex_t, as nina_lock.
 * - K_SEM_DEFINE(scan_request, 0, 1) and connect_request of nina_wifi.c:
 *   a sem_t each (SEM_INITIALIZER: no priority inheritance), posted only
 *   when no count is pending (the limit of 1), the check and the post in
 *   one critical section; the wait is nxsem_wait_uninterruptible().
 *
 * The core's waits for READY (nina_proto.c wait_ready()) sleep on an edge
 * interrupt of the line instead of sampling it every 100 us, as the other
 * builds do (nina_os_ready_*() of nina_os.h):
 *
 *   nina_os_ready_start()       adds the callback to the compatibility
 *                               layer's gpio0, which the LSM6DSOX's INT1
 *                               (GPIO24) uses too; its error (the core then
 *                               samples READY) is gpio_add_callback()'s
 *   nina_os_ready_arm(desired)  drops a wake-up left from before
 *                               (nxsem_reset() to 0), then enables the edge
 *                               of READY (GPIO10, active low: nina_bus_rp2040.c)
 *                               towards DESIRED, falling for 1 (ready),
 *                               rising for 0, through the board's GPIO
 *                               bank 0 (nano_gpio_irq_configure(), the
 *                               board's src/nano_gpio.c: clears an edge
 *                               latched while the event was off, then
 *                               enables it)
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
 *
 * The waits of the socket loops for the module's socket events on its IRQ line (nina_os_event_*() of
 * nina_os.h, common/drivers/nina/internal/nina_wait.c):
 *
 *   nina_os_event_start()       adds the callback to gpio0 and enables the
 *                               rising edge of GPIO2 (the IRQ line, active
 *                               high, BOARD_NINA_IRQ_GPIO) through the
 *                               board's GPIO bank 0; the line stays an input
 *                               with its pull-down.  The probe calls it
 *                               after the module's reset, during which the
 *                               line is driven
 *   the interrupt               the count goes up; g_event_sem is posted
 *                               until no thread waits on it (a broadcast),
 *                               then once more if it is at 0, for a thread
 *                               that read the old count but does not wait
 *                               yet (at most one count); the event thread
 *                               of the sockets is woken too
 *                               (nina_poller_edge(), nina_poller.c, which
 *                               waits on its own semaphore)
 *   nina_os_event_seq()         the count (written by the interrupt only)
 *   nina_os_event_wait()        while the count is SEQ and the time is not
 *                               up: nxsem_tickwait_uninterruptible() for the
 *                               time left.  A count left by an earlier edge
 *                               ends one wait early; the count is read again
 *                               then, so the wait goes on
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "nano_gpio.h"
#include "nina_os.h"
#ifdef CONFIG_NINA_USRSOCK
#  include "nina_usrsock.h"
#endif

/* READY: GPIO10, active low (nina_bus_rp2040.c, BOARD_NINA_READY_GPIO). */

#define READY_GPIO 10

static rmutex_t g_nina_lock = NXRMUTEX_INITIALIZER;
static mutex_t g_dns_lock = NXMUTEX_INITIALIZER;

/* Posted by the interrupt; no priority inheritance (SEM_INITIALIZER). */

static sem_t g_ready_sem = SEM_INITIALIZER(0);
static struct gpio_callback g_ready_cb;

/* The IRQ line: GPIO2, active high (nina_bus_rp2040.c,
 * BOARD_NINA_IRQ_GPIO).
 */

#define IRQ_GPIO 2

static sem_t g_event_sem = SEM_INITIALIZER(0);
static struct gpio_callback g_event_cb;
static volatile uint32_t g_event_count;

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

int nina_os_ready_start(void)
{
  gpio_init_callback(&g_ready_cb, ready_changed, BIT(READY_GPIO));
  return gpio_add_callback(DEVICE_DT_GET(DT_NODELABEL(gpio0)), &g_ready_cb);
}

void nina_os_ready_arm(int desired)
{
  /* Active low: ready (1) is the falling edge. */

  uint32_t edge = desired ? NANO_GPIO_IRQ_EDGE_FALL : NANO_GPIO_IRQ_EDGE_RISE;

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

/* In the GPIO bank's interrupt handler. */

static void event_edge(const struct device *port, struct gpio_callback *cb,
                       gpio_port_pins_t pins)
{
  int value = 0;

  (void)port;
  (void)cb;
  (void)pins;

  g_event_count++;
  while (nxsem_get_value(&g_event_sem, &value) == 0 && value < 0)
    {
      nxsem_post(&g_event_sem);
    }

  if (value == 0)
    {
      nxsem_post(&g_event_sem);
    }

#ifdef CONFIG_NINA_USRSOCK
  nina_poller_edge();
#endif
}

int nina_os_event_start(void)
{
  const struct device *port = DEVICE_DT_GET(DT_NODELABEL(gpio0));
  int ret;

  gpio_init_callback(&g_event_cb, event_edge, BIT(IRQ_GPIO));
  ret = gpio_add_callback(port, &g_event_cb);
  if (ret == 0)
    {
      ret = nano_gpio_irq_configure(IRQ_GPIO, NANO_GPIO_IRQ_EDGE_RISE);
      if (ret != 0)
        {
          gpio_remove_callback(port, &g_event_cb);
        }
    }

  return ret;
}

uint32_t nina_os_event_seq(void)
{
  return g_event_count;
}

void nina_os_event_wait(uint32_t seq, uint32_t ms)
{
  int64_t end = nina_os_uptime_ms() + ms;
  int64_t left = ms;

  while (g_event_count == seq && left > 0)
    {
      nxsem_tickwait_uninterruptible(&g_event_sem,
                                     MSEC2TICK((uint32_t)left));
      left = end - nina_os_uptime_ms();
    }
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

#ifdef CONFIG_NINA_NET
static rmutex_t g_wifi_lock = NXRMUTEX_INITIALIZER;
static sem_t g_wifi_request[2] =
{
  SEM_INITIALIZER(0),           /* NINA_OS_WIFI_SCAN */
  SEM_INITIALIZER(0),           /* NINA_OS_WIFI_CONNECT */
};

void nina_os_wifi_lock(void)
{
  while (nxrmutex_lock(&g_wifi_lock) == -EINTR)
    {
    }
}

void nina_os_wifi_unlock(void)
{
  nxrmutex_unlock(&g_wifi_lock);
}

void nina_os_wifi_post(unsigned int which)
{
  irqstate_t flags = enter_critical_section();
  int value;

  if (nxsem_get_value(&g_wifi_request[which], &value) == 0 && value <= 0)
    {
      nxsem_post(&g_wifi_request[which]);
    }

  leave_critical_section(flags);
}

void nina_os_wifi_wait(unsigned int which)
{
  nxsem_wait_uninterruptible(&g_wifi_request[which]);
}
#endif

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
