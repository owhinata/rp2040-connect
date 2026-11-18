/* SPDX-License-Identifier: Apache-2.0 */
/* The kernel services the shared NINA core uses (nina_spi.c, nina_sock.c,
 * nina_dns.c), in the terms of Zephyr's calls, which the core had before it
 * was shared.  Each build implements them: zephyr/app/usb-console/drivers/
 * nina/nina_os.c, nuttx/drivers/nina/nina_os.c and
 * freertos/drivers/nina/nina_os.c; the host tests (common/tests/nina_*.c)
 * replace them with fakes.
 */

#ifndef NINA_OS_H_
#define NINA_OS_H_

#include <stdint.h>

/* k_mutex_lock(&nina_lock, K_FOREVER) / k_mutex_unlock(): a recursive
 * mutex with priority inheritance.  nina_os_lock() returns 0, or a
 * negative errno if the wait was cancelled (never on Zephyr and FreeRTOS;
 * on NuttX the mutex wait only fails like that with pthread cancellation
 * points, which that build does not use, and a signal does not end it).
 */

int nina_os_lock(void);
void nina_os_unlock(void);

/* K_MUTEX_DEFINE(nina_dns_lock): held across the two resolver commands
 * (nina_dns.c).  A mutex with priority inheritance; the wait is not ended
 * by signals.
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
 * sampling it.  Without NINA_OS_WAIT_READY, nina_spi.c samples READY every
 * NINA_READY_POLL_US with nina_os_sleep_us().  An OS that defines
 * NINA_OS_WAIT_READY for the compilation of nina_spi.c provides these
 * three instead (on a GPIO edge interrupt of the line), and nina_spi.c
 * waits with them:
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

/* Optional: the resolver's results and its registration.  Without
 * NINA_OS_DNS_OFFLOAD, nina_dns.c takes its results from malloc() and
 * free(), and its nina_dns_init() does nothing (the callers call
 * nina_getaddrinfo() themselves).  An OS that defines NINA_OS_DNS_OFFLOAD
 * for the compilation of nina_dns.c provides these three instead (the
 * Zephyr build: zephyr/app/usb-console/drivers/nina/nina_socket.c):
 *
 * nina_os_dns_alloc(), nina_os_dns_free(): a result of SIZE bytes, or
 * NULL when there is no room (nina_getaddrinfo() then returns EAI_MEMORY),
 * and its release.  Not from an interrupt handler.
 *
 * nina_os_dns_register(): called by nina_dns_init() (once, from
 * iface_init() of nina_net.c): makes nina_getaddrinfo() and
 * nina_freeaddrinfo() the resolver of the OS's sockets.
 */

#ifdef NINA_OS_DNS_OFFLOAD
#include <stddef.h>

void *nina_os_dns_alloc(size_t size);
void nina_os_dns_free(void *ptr);
void nina_os_dns_register(void);
#endif

#endif /* NINA_OS_H_ */
