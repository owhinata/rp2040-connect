/* SPDX-License-Identifier: MIT */
/* The kernel services of the shared NINA core on FreeRTOS
 * (common/drivers/nina/nina_os.h), on the Zephyr compatibility layer
 * (freertos/drivers/compat), with the calls of the Zephyr build's
 * drivers/nina/nina_wifi.c and nina_dns.c:
 *
 *   nina_os_lock(), _unlock()      K_MUTEX_DEFINE(nina_lock),
 *                                  k_mutex_lock(K_FOREVER): recursive, with
 *                                  priority inheritance (one level: the
 *                                  layer's k_mutex, zephyr_mutex.c)
 *   nina_os_dns_lock(), _unlock()  K_MUTEX_DEFINE(nina_dns_lock), the same
 *   nina_os_sleep_us(us)           k_sleep(K_USEC(us)) = k_usleep(us): us
 *                                  rounded up to ticks of 1 ms, plus one
 *                                  tick (the core of this build does not
 *                                  call it: its waits for READY are the
 *                                  ones below)
 *   nina_os_sleep_ms(ms)           k_sleep(K_MSEC(ms)) = k_msleep(ms): the
 *                                  same rounding (10 ms: 11 ticks, 750 ms:
 *                                  751 ticks)
 *   nina_os_uptime_ms()            k_uptime_get(): the TIMER in ms, rounded
 *                                  down (Zephyr: the tick count in ms,
 *                                  rounded down; the same, except that this
 *                                  one goes on while interrupts are
 *                                  disabled)
 *
 * The layer's mutex is a FreeRTOS mutex made at its first lock (a mutex
 * left all zero is unlocked, as in Zephyr), so these work from the first
 * task on; not from an interrupt handler (nor does the core call them so).
 * K_FOREVER waits cannot fail: nina_os_lock() returns 0.
 *
 * The core's waits for READY (nina_spi.c wait_ready(), compiled with
 * NINA_OS_WAIT_READY, nina.cmake) sleep on an edge interrupt of the line
 * instead of sampling it (nina_os_ready_*() of nina_os.h; the Zephyr and
 * NuttX builds sample it every 100 us):
 *
 *   nina_os_ready_arm(desired)  drops a wake-up left from before
 *                               (k_sem_reset()), then enables the edge of
 *                               READY (GPIO10, active low: nano_nina.c)
 *                               towards DESIRED, falling for 1 (ready),
 *                               rising for 0, through the board's GPIO bank
 *                               0 (nano_gpio_irq_configure(): clears an edge
 *                               latched while the event was off, then
 *                               enables it; GPIO10 is one of
 *                               NANO_GPIO_IRQ_PINS,
 *                               freertos/drivers/compat/rp2040/nano_gpio.c);
 *                               the first call adds the callback to the
 *                               layer's gpio0
 *   the interrupt               the bank's handler acknowledges the edge and
 *                               runs the callback, which gives the
 *                               semaphore (k_sem_give() from the handler:
 *                               the waiting task runs at the handler's end)
 *   nina_os_ready_wait(ms)      k_sem_take(K_MSEC(ms)), at most
 *                               READY_WAIT_SLICE_MS at a time: the same
 *                               rounding as the sleeps.  The core's deadline
 *                               is on the TIMER (nina_os_uptime_ms()), the
 *                               wait on the tick, which stands still while
 *                               interrupts are masked (a flash erase) until
 *                               the tick hook catches up (tick.c); in
 *                               slices, as a sampling would (one read of
 *                               READY per slice while the module stays
 *                               busy)
 *   nina_os_ready_disarm()      disables the edge (no interrupts while CS
 *                               toggles READY during the transfers)
 *
 * Nothing is lost between the core's read of READY and its wait: the edge
 * is enabled before the read, so a change after the read gives the
 * semaphore, and the wait returns at once.  A change before the read is
 * seen by the read.  An edge left latched from an earlier wait could only
 * wake a wait early; the core reads the line again after every wake-up.
 * One waiter at a time: the core calls these with nina_lock held.
 */

#include <stdint.h>

#include <zephyr/kernel.h>

#include "nina_os.h"

#ifndef NINA_OS_WAIT_READY
#error "the core waits for READY with nina_os_ready_*() (nina.cmake)"
#endif

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "nano_gpio.h"

/* READY: GPIO10, active low (nano_nina.c, the devicetree's ready-gpios). */
#define READY_GPIO 10

/* The longest single wait (see above). */
#define READY_WAIT_SLICE_MS 10

static K_SEM_DEFINE(ready_sem, 0, 1);
static struct gpio_callback ready_cb;
static bool ready_cb_added;

/* In the GPIO bank's interrupt handler. */
static void ready_changed(const struct device *port, struct gpio_callback *cb,
			  gpio_port_pins_t pins)
{
	(void)port;
	(void)cb;
	(void)pins;
	k_sem_give(&ready_sem);
}

void nina_os_ready_arm(int desired)
{
	/* Active low: ready (1) is the falling edge. */
	uint32_t edge = desired ? NANO_GPIO_IRQ_EDGE_FALL : NANO_GPIO_IRQ_EDGE_RISE;

	if (!ready_cb_added) {
		gpio_init_callback(&ready_cb, ready_changed, BIT(READY_GPIO));
		(void)gpio_add_callback(DEVICE_DT_GET(DT_NODELABEL(gpio0)), &ready_cb);
		ready_cb_added = true;
	}
	k_sem_reset(&ready_sem);
	(void)nano_gpio_irq_configure(READY_GPIO, edge);
}

void nina_os_ready_wait(uint32_t ms)
{
	(void)k_sem_take(&ready_sem, K_MSEC(ms < READY_WAIT_SLICE_MS ? ms : READY_WAIT_SLICE_MS));
}

void nina_os_ready_disarm(void)
{
	(void)nano_gpio_irq_configure(READY_GPIO, 0);
}

static K_MUTEX_DEFINE(nina_lock);
static K_MUTEX_DEFINE(nina_dns_lock);

int nina_os_lock(void)
{
	return k_mutex_lock(&nina_lock, K_FOREVER);
}

void nina_os_unlock(void)
{
	(void)k_mutex_unlock(&nina_lock);
}

void nina_os_dns_lock(void)
{
	(void)k_mutex_lock(&nina_dns_lock, K_FOREVER);
}

void nina_os_dns_unlock(void)
{
	(void)k_mutex_unlock(&nina_dns_lock);
}

void nina_os_sleep_us(uint32_t us)
{
	(void)k_usleep(us > INT32_MAX ? INT32_MAX : (int32_t)us);
}

void nina_os_sleep_ms(uint32_t ms)
{
	(void)k_msleep(ms > INT32_MAX ? INT32_MAX : (int32_t)ms);
}

int64_t nina_os_uptime_ms(void)
{
	return k_uptime_get();
}
