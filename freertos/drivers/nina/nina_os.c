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
 *                                  rounded up to 100 us ticks, plus one tick
 *                                  (NINA_READY_POLL_US 100 sleeps 2 ticks)
 *   nina_os_sleep_ms(ms)           k_sleep(K_MSEC(ms)) = k_msleep(ms): the
 *                                  same rounding (10 ms: 101 ticks, 750 ms:
 *                                  7501 ticks)
 *   nina_os_uptime_ms()            k_uptime_get(): the TIMER in ms, rounded
 *                                  down (Zephyr: the tick count in ms,
 *                                  rounded down; the same with a 100 us
 *                                  tick, except that this one goes on while
 *                                  interrupts are disabled)
 *
 * The layer's mutex is a FreeRTOS mutex made at its first lock (a mutex
 * left all zero is unlocked, as in Zephyr), so these work from the first
 * task on; not from an interrupt handler (nor does the core call them so).
 * K_FOREVER waits cannot fail: nina_os_lock() returns 0.
 */

#include <stdint.h>

#include <zephyr/kernel.h>

#include "nina_os.h"

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
