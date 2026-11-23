/* SPDX-License-Identifier: MIT */
/* The kernel services of the shared NINA core on Zephyr
 * (common/drivers/nina/port/nina_os.h):
 *
 *   nina_os_lock(), _unlock()      K_MUTEX_DEFINE(nina_lock),
 *                                  k_mutex_lock(K_FOREVER): recursive, with
 *                                  priority inheritance; cannot fail
 *   nina_os_dns_lock(), _unlock()  K_MUTEX_DEFINE(nina_dns_lock), the same
 *   nina_os_wifi_lock(), _unlock() K_MUTEX_DEFINE(connect_state_lock), the
 *                                  same
 *   nina_os_wifi_post(),           K_SEM_DEFINE(wifi_request, 0, 1):
 *   nina_os_wifi_wait(ms)          k_sem_give(), k_sem_take(K_MSEC(ms)) (a
 *                                  give left from a post whose request the
 *                                  core has already taken ends a later
 *                                  wait early)
 *   nina_os_sleep_us(us)           k_sleep(K_USEC(us))
 *   nina_os_sleep_ms(ms)           k_sleep(K_MSEC(ms))
 *   nina_os_uptime_ms()            k_uptime_get()
 *
 * The core's waits for the module's READY line sleep until an edge
 * interrupt of the line instead of sampling it every 100 us
 * (nina_os_ready_*() of nina_os.h):
 *
 *   nina_os_ready_start()       adds the callback to the line's port
 *                               (gpio0); its error (the core then samples
 *                               READY) is gpio_add_callback_dt()'s
 *   nina_os_ready_arm(desired)  drops a wake-up left from before
 *                               (k_sem_reset()), then enables the edge of
 *                               READY towards DESIRED with
 *                               gpio_pin_interrupt_configure_dt():
 *                               GPIO_INT_EDGE_TO_ACTIVE for 1 (ready),
 *                               _TO_INACTIVE for 0; ready-gpios is active
 *                               low, so these are the falling and the
 *                               rising edge.  The RP2040 GPIO driver clears
 *                               an edge latched while the event was off
 *                               before it enables it
 *   the interrupt               the GPIO driver's handler (IO_IRQ_BANK0)
 *                               acknowledges the edge and runs the
 *                               callback, which gives the semaphore
 *   nina_os_ready_wait(ms)      k_sem_take(K_MSEC(ms)), at most
 *                               READY_WAIT_SLICE_MS at a time
 *   nina_os_ready_disarm()      GPIO_INT_DISABLE (no interrupts while CS
 *                               toggles READY during the transfers)
 *
 * Nothing is lost between the core's read of READY and its wait: the edge
 * is enabled before the read, so a change after the read gives the
 * semaphore and the wait returns at once; a change before the read is seen
 * by the read.  A wake-up from an earlier edge can only end a wait early;
 * the core reads the line again after every wake-up.  The slice bounds what
 * an edge that is not seen would cost (the line is read again at least
 * every READY_WAIT_SLICE_MS while the module stays busy); the core's
 * deadline and the wait are on the same kernel clock.  One waiter at a
 * time: the core calls these with nina_lock held, not from an interrupt.
 *
 * It also has the waits for the module's socket events on its IRQ line
 * (nina_os_event_*() of nina_os.h, at the end of this file).
 *
 * The host test (zephyr/app/usb-console/tests/nina_boundary.c) builds this
 * file on fakes of the Zephyr calls, whose registration of the callbacks
 * succeeds or fails.
 */

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "nina_os.h"

K_MUTEX_DEFINE(nina_lock);
K_MUTEX_DEFINE(nina_dns_lock);

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

K_MUTEX_DEFINE(connect_state_lock);
static K_SEM_DEFINE(wifi_request, 0, 1);

void nina_os_wifi_lock(void)
{
	(void)k_mutex_lock(&connect_state_lock, K_FOREVER);
}

void nina_os_wifi_unlock(void)
{
	(void)k_mutex_unlock(&connect_state_lock);
}

void nina_os_wifi_post(void)
{
	k_sem_give(&wifi_request);
}

void nina_os_wifi_wait(uint32_t ms)
{
	(void)k_sem_take(&wifi_request, K_MSEC(ms));
}

void nina_os_sleep_us(uint32_t us)
{
	(void)k_sleep(K_USEC(us));
}

void nina_os_sleep_ms(uint32_t ms)
{
	(void)k_sleep(K_MSEC(ms));
}

int64_t nina_os_uptime_ms(void)
{
	return k_uptime_get();
}

/* The longest single wait (see above). */
#define READY_WAIT_SLICE_MS 10

/* The module's node (nina_bus.c). */
#define NINA_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(arduino_nina_spi)

static const struct gpio_dt_spec ready = GPIO_DT_SPEC_GET(NINA_NODE, ready_gpios);
static K_SEM_DEFINE(ready_sem, 0, 1);
static struct gpio_callback ready_cb;

/* In the GPIO driver's interrupt handler. */
static void ready_changed(const struct device *port, struct gpio_callback *cb,
			  gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	k_sem_give(&ready_sem);
}

int nina_os_ready_start(void)
{
	gpio_init_callback(&ready_cb, ready_changed, BIT(ready.pin));
	return gpio_add_callback_dt(&ready, &ready_cb);
}

void nina_os_ready_arm(int desired)
{
	k_sem_reset(&ready_sem);
	(void)gpio_pin_interrupt_configure_dt(&ready, desired ? GPIO_INT_EDGE_TO_ACTIVE
							      : GPIO_INT_EDGE_TO_INACTIVE);
}

void nina_os_ready_wait(uint32_t ms)
{
	(void)k_sem_take(&ready_sem, K_MSEC(MIN(ms, READY_WAIT_SLICE_MS)));
}

void nina_os_ready_disarm(void)
{
	(void)gpio_pin_interrupt_configure_dt(&ready, GPIO_INT_DISABLE);
}

/* The module's socket events (nina_os.h):
 *
 *   nina_os_event_start()     adds the callback to the IRQ line's port
 *                             (irq-gpios: GPIO2, active high) and enables
 *                             its edge to active (rising) with
 *                             gpio_pin_interrupt_configure_dt(), which
 *                             clears an edge latched before.  The line
 *                             stays an input with its pull-down
 *                             (nina_proto.c init_nina()); the probe calls
 *                             this after the module's reset, during which
 *                             the line is driven
 *   the interrupt             the GPIO driver's handler runs the callback:
 *                             the count goes up, k_sem_reset() wakes every
 *                             thread in k_sem_take() of event_sem (-EAGAIN:
 *                             a broadcast), then k_sem_give() leaves one
 *                             count for a thread that read the old count
 *                             but has not reached k_sem_take() yet
 *   nina_os_event_seq()       the count (written by the interrupt only)
 *   nina_os_event_wait()      while the count is SEQ and the time is not
 *                             up: k_sem_take() for the time left.  A count
 *                             left by an earlier edge ends one k_sem_take()
 *                             early; the count is read again then, so the
 *                             wait goes on
 */

static const struct gpio_dt_spec irq = GPIO_DT_SPEC_GET(NINA_NODE, irq_gpios);
static K_SEM_DEFINE(event_sem, 0, 1);
static struct gpio_callback event_cb;
static volatile uint32_t event_count;

/* In the GPIO driver's interrupt handler. */
static void event_edge(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	event_count++;
	k_sem_reset(&event_sem);
	k_sem_give(&event_sem);
}

int nina_os_event_start(void)
{
	int ret;

	gpio_init_callback(&event_cb, event_edge, BIT(irq.pin));
	ret = gpio_add_callback_dt(&irq, &event_cb);
	if (!ret) {
		ret = gpio_pin_interrupt_configure_dt(&irq, GPIO_INT_EDGE_TO_ACTIVE);
		if (ret) {
			(void)gpio_remove_callback_dt(&irq, &event_cb);
		}
	}
	return ret;
}

uint32_t nina_os_event_seq(void)
{
	return event_count;
}

void nina_os_event_wait(uint32_t seq, uint32_t ms)
{
	int64_t end = k_uptime_get() + ms;
	int64_t left = ms;

	while (event_count == seq && left > 0) {
		(void)k_sem_take(&event_sem, K_MSEC(left));
		left = end - k_uptime_get();
	}
}
