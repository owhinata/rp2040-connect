/* SPDX-License-Identifier: Apache-2.0 */
/* The kernel services nina_spi.c uses, in the terms of the Zephyr build's
 * drivers/nina/nina_wifi.c.  nina_os.c implements them on NuttX; the host
 * test (nuttx/tests/nina_spi.c) replaces them with fakes.
 */

#ifndef __NUTTX_DRIVERS_NINA_NINA_OS_H
#define __NUTTX_DRIVERS_NINA_NINA_OS_H

#include <stdint.h>

/* k_mutex_lock(&nina_lock, K_FOREVER) / k_mutex_unlock(): a recursive
 * mutex with priority inheritance.  nina_os_lock() returns 0, or a
 * negative errno if the wait was cancelled (never: NuttX's mutex wait only
 * fails like that with pthread cancellation points, which this build does
 * not use; a signal does not end the wait).
 */

int nina_os_lock(void);
void nina_os_unlock(void);

/* K_MUTEX_DEFINE(nina_dns_lock) of drivers/nina/nina_dns.c: held across
 * the two resolver commands (nina_dns.c).  A mutex with priority
 * inheritance; the wait is not ended by signals.
 */

void nina_os_dns_lock(void);
void nina_os_dns_unlock(void);

/* k_sleep(K_USEC(us)) and k_sleep(K_MSEC(ms)): the time rounded up to
 * ticks, plus one tick.  Not ended by signals.
 */

void nina_os_sleep_us(uint32_t us);
void nina_os_sleep_ms(uint32_t ms);

/* k_uptime_get(): the kernel clock in milliseconds, rounded down. */

int64_t nina_os_uptime_ms(void);

/* Optional: waiting for the module's READY line on an event instead of
 * sampling it.  Without NINA_OS_WAIT_READY (the Zephyr and NuttX builds,
 * and the FreeRTOS build by default), nina_spi.c samples READY every
 * NINA_READY_POLL_US with nina_os_sleep_us(), as the Zephyr build's
 * nina_wifi.c does.  An OS that defines NINA_OS_WAIT_READY for the
 * compilation of nina_spi.c provides these three instead (the FreeRTOS
 * build with NANO_NINA_READY_IRQ: freertos/drivers/nina/nina_os.c, on a
 * GPIO edge interrupt), and nina_spi.c waits with them:
 *
 *   nina_os_ready_arm(DESIRED);
 *   until READY reads DESIRED or the deadline has passed:
 *       nina_os_ready_wait(the milliseconds left);
 *   nina_os_ready_disarm();
 *
 * nina_os_ready_arm(): from its return on, a change of READY to the
 * logical level DESIRED (1: active, the module is ready; as
 * nano_nina_pin_get()) ends the current or the next nina_os_ready_wait()
 * early; a wake-up left from before the call is dropped.  Armed before
 * READY is read, so a change between the read and the wait is not lost.
 *
 * nina_os_ready_wait(): returns after such a change, or after MS
 * milliseconds (rounded up to ticks, plus one tick, as the sleeps), or
 * earlier for no reason; the caller reads READY again in each case.
 *
 * nina_os_ready_disarm(): no wake-ups after it.
 *
 * Called with nina_os_lock() held (one waiter at a time), not from an
 * interrupt handler.
 */

#ifdef NINA_OS_WAIT_READY
void nina_os_ready_arm(int desired);
void nina_os_ready_wait(uint32_t ms);
void nina_os_ready_disarm(void);
#endif

#endif /* __NUTTX_DRIVERS_NINA_NINA_OS_H */
