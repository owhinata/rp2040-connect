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

/* Optional: the module's socket events on its IRQ line (NANO_NINA_IRQ,
 * GPIO2, active high; firmware/nina-fw/README.md, "Socket events on the IRQ
 * line"): the line rises when an event that a command armed happens, and
 * falls when the module begins the next command.  Without NINA_OS_EVENTS,
 * the socket loops sleep their poll intervals (nina_wait.c).  An OS that
 * defines NINA_OS_EVENTS for the compilation of nina_wait.c (and its
 * nina_os.c) provides these three, on a GPIO edge interrupt of the line:
 *
 * nina_os_event_start(): from its return on, every rising edge of the line
 * adds one to the count of nina_os_event_seq() and ends every
 * nina_os_event_wait() in progress.  Called once (nina_wait.c, from the
 * capability probe of nina_spi.c), after the module's reset: init_nina()
 * drives the line during the reset (a boot mode pin of the module), so no
 * edge is looked at before.  Not from an interrupt handler.  0, or a
 * negative errno: the events are then not used.
 *
 * nina_os_event_seq(): the count of rising edges since the start (wraps
 * around).  Any thread or interrupt handler.
 *
 * nina_os_event_wait(SEQ, MS): returns at once when the count differs from
 * SEQ, else after the next edge, or after MS milliseconds (rounded up to
 * ticks, plus one tick, as the sleeps; 0: at once), or earlier for no
 * reason; the caller asks the module again in each case.  Any number of
 * threads at once (a broadcast: one edge ends all their waits).  The
 * caller takes SEQ before the command that may arm an event, so an edge
 * after that command is never lost.  Not with nina_os_lock() held, not
 * from an interrupt handler.
 */

#ifdef NINA_OS_EVENTS
int nina_os_event_start(void);
uint32_t nina_os_event_seq(void);
void nina_os_event_wait(uint32_t seq, uint32_t ms);
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
