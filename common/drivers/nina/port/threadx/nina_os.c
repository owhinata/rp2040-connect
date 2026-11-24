/* SPDX-License-Identifier: MIT */
/* The kernel services of the shared NINA core on ThreadX (nina_os.h),
 * written on the ThreadX kernel and the Pico SDK.  The calls of the Zephyr
 * build (common/drivers/nina/port/zephyr/nina_os.c) and what stands for them
 * here:
 *
 *   K_MUTEX_DEFINE(nina_lock),      a ThreadX mutex with priority
 *   nina_dns_lock and               inheritance (tx_mutex_create(),
 *   connect_state_lock;             TX_INHERIT), taken with tx_mutex_get(
 *   k_mutex_lock(K_FOREVER)         TX_WAIT_FOREVER): recursive for its
 *                                   owner, with ThreadX's priority
 *                                   inheritance (the holder is raised to its
 *                                   highest waiter, one level, and drops back
 *                                   when it releases the mutex, to the
 *                                   highest waiter of the mutexes it still
 *                                   holds).  Before the kernel runs (no
 *                                   thread) the lock takes the mutex without
 *                                   a wait, which ThreadX refuses there; no
 *                                   other thread can hold it then.  A wait
 *                                   for ever ends only when a thread holds
 *                                   the mutex: nina_os_lock() returns 0
 *                                   (-EAGAIN if ThreadX refused, which this
 *                                   build never makes it do).  ThreadX
 *                                   refuses the unlock of a mutex the caller
 *                                   does not hold
 *   K_SEM_DEFINE(wifi_request,      the same: a ThreadX semaphore, put with
 *   0, 1), k_sem_give(),            the ceiling 1 (tx_semaphore_ceiling_put()),
 *   k_sem_take(K_MSEC(ms))          taken with tx_semaphore_get() for the time
 *                                   rounded up to ticks, plus one tick.  A
 *                                   post is kept until the next wait (also
 *                                   one before the first wait, and one while
 *                                   the thread is busy), at most one.  A
 *                                   wake-up left from a post whose request
 *                                   the core has already taken ends a later
 *                                   wait early
 *   k_sleep(K_USEC(us)),            tx_thread_sleep(ticks + 1) for the time
 *   k_sleep(K_MSEC(ms))             rounded up to ticks (Zephyr wakes a
 *                                   relative timeout at the current tick +
 *                                   ticks + 1, ThreadX a sleep of n at the
 *                                   current tick + n); 0 ticks yields
 *                                   (tx_thread_relinquish()).  With the
 *                                   board's 1 ms tick: 100 us 2 ticks, 10 ms
 *                                   11, 750 ms 751.  The argument is clamped
 *                                   to INT32_MAX, as k_msleep()/k_usleep()
 *                                   take an int32_t
 *   k_uptime_get()                  the RP2040 TIMER (time_us_64(), 64 bits
 *                                   of 1 us) in ms, rounded down: not the
 *                                   tick, which is 32 bits wide (it wraps
 *                                   after about 49.7 days) and stands still
 *                                   while interrupts are masked for longer
 *                                   than a tick (a flash erase) until the
 *                                   application's tick catches up
 *
 * Every timed wait below (READY, the socket events) blocks for the time
 * rounded up to ticks, plus one tick, as the sleeps; 0 does not block.
 *
 * The objects are static (no heap, no byte pool) and made at the first use
 * of any of them (objects() below), from a thread or before the kernel
 * runs.  They are never zeroed or deleted, so they stay in ThreadX's lists
 * of created objects (unlike the compatibility layer's k_mutex, which the
 * application may zero while it is unlocked).
 *
 * The critical sections (taskENTER_CRITICAL() on FreeRTOS) mask interrupts
 * (tx_interrupt_control(TX_INT_DISABLE): PRIMASK on the RP2040, in a
 * thread and in the interrupt handler).  Inside them no ThreadX service
 * runs that may make a thread ready, except in the interrupt handler: a
 * thread's service that readies a more urgent thread switches to it at
 * once, and ThreadX's Cortex-M0 port unmasks interrupts for an instant to
 * do it (_tx_thread_system_return()), which would end the critical section
 * early; in an interrupt handler it only pends PendSV.  The creation of
 * the objects (no thread becomes ready) is the one service inside a
 * thread's critical section (with interrupts masked, ThreadX's check for a
 * switch inside it finds none to make).
 *
 * The core's waits for READY (nina_proto.c wait_ready()) sleep on an edge
 * interrupt of the line instead of sampling it (nina_os_ready_*() of
 * nina_os.h; the Zephyr and NuttX builds wait on its interrupt in the same
 * way):
 *
 *   nina_os_ready_start()       adds this file's handler of IO_IRQ_BANK0
 *                               (below) and enables the interrupt; 0
 *   nina_os_ready_arm(desired)  drops a wake-up left from before (takes
 *                               the semaphore without waiting), then enables
 *                               the edge of READY (GPIO10, active low:
 *                               nina_bus_rp2040.c) towards DESIRED, falling for 1
 *                               (ready), rising for 0: all of the pin's
 *                               events off, then the edge on
 *                               (gpio_set_irq_enabled(), which clears the
 *                               events it names before it changes the
 *                               enable, so an edge latched while the event
 *                               was off does not fire)
 *   the interrupt               the handler acknowledges the edge and puts
 *                               the semaphore (ceiling 1); the waiting
 *                               thread runs at the handler's end (ThreadX
 *                               pends PendSV itself)
 *   nina_os_ready_wait(ms)      tx_semaphore_get() for at most
 *                               READY_WAIT_SLICE_MS at a time.  The core's
 *                               deadline is on the TIMER
 *                               (nina_os_uptime_ms()), the wait on the tick,
 *                               which stands still while interrupts are
 *                               masked; in slices, as a sampling would (one
 *                               read of READY per slice while the module
 *                               stays busy)
 *   nina_os_ready_disarm()      all of the pin's events off (no interrupts
 *                               while CS toggles READY during the transfers)
 *
 * Nothing is lost between the core's read of READY and its wait: the edge
 * is enabled before the read, so a change after the read puts the
 * semaphore, and the wait returns at once.  A change before the read is
 * seen by the read.  One waiter at a time: the core calls these with
 * nina_lock held.
 *
 * The waits of the socket loops for the module's socket events on its IRQ
 * line (nina_os_event_*() of nina_os.h, nina_wait.c):
 *
 *   nina_os_event_start()       adds the handler (if nina_os_ready_start()
 *                               has not) and enables the rising edge of
 *                               GPIO2 (the IRQ line, active high:
 *                               nina_bus_rp2040.c) as READY's; the line stays an
 *                               input with its pull-down.  The probe calls
 *                               it after the module's reset, during which
 *                               the line is driven; 0
 *   the interrupt               the count goes up and every thread that
 *                               waits is woken (a broadcast): each waiter is
 *                               on a list (on its stack) with a semaphore of
 *                               its own (on its stack too, made for the
 *                               wait), which the handler empties, putting
 *                               each waiter's semaphore (ceiling 1)
 *   nina_os_event_seq()         the count (written by the interrupt only)
 *   nina_os_event_wait()        while the count is SEQ and the time
 *                               (nina_os_uptime_ms(), the TIMER) is not up:
 *                               joins the list, in one critical section
 *                               with a last look at the count (an edge after
 *                               the caller read SEQ either is seen there or
 *                               puts the waiter's semaphore, which keeps
 *                               it), and waits on the semaphore for the
 *                               time left; leaves the list when it times out
 *
 * ThreadX has no wake-up of a thread's own that a driver may use (FreeRTOS's
 * task notification indices; the per-thread semaphore of the compatibility
 * layer is the layer's wake flag, and sharing it would let either side end
 * or eat the other's waits), so each socket event wait makes its own
 * semaphore for the call and deletes it at the end (a creation walks
 * ThreadX's list of created semaphores once; these waits are not on the
 * paths of every sample).  The wake-ups of the Wi-Fi thread and of the
 * SPI DMA (nina_bus_dma_os_impl.h) are semaphores of their own too.
 *
 * The handler shares IO_IRQ_BANK0 with the board's GPIO bank 0 backend
 * (common/compat/port/pico_sdk/gpio_os_rp2040.c, the LSM6DSOX's INT1, GPIO24)
 * the SDK's way: each adds a raw handler with
 * gpio_add_raw_irq_handler_masked() for its own pins (the SDK asserts that
 * no pin is in two masks), and each looks at and acknowledges only its own
 * pins.  One handler for GPIO10 and GPIO2 (the SDK has a few shared
 * handler slots for all interrupts).  It runs at the SDK's default priority
 * (0x80), as every peripheral interrupt of the build.  ThreadX's Cortex-M0
 * port masks all interrupts (PRIMASK) in its critical sections, so its
 * services may be called from an interrupt of any priority.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

#include "tx_api.h"

#include "nina_board_rp2040.h"
#include "nina_os.h"
#include "zephyr_os_port.h"

#if TX_TIMER_TICKS_PER_SECOND != 1000
#error "the NINA's waits are written for ThreadX's 1 ms tick (tx_user.h)"
#endif

/* READY, active low; the IRQ line, active high (nina_board_rp2040.h). */
#define READY_GPIO NINA_READY_GPIO
#define IRQ_GPIO   NINA_IRQ_GPIO

#define NINA_GPIO_IRQ_PINS ((1u << READY_GPIO) | (1u << IRQ_GPIO))

#define GPIO_EVENTS_ALL                                                                            \
	(GPIO_IRQ_LEVEL_LOW | GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)

/* The longest single wait for READY (see above). */
#define READY_WAIT_SLICE_MS 10

static TX_MUTEX nina_lock;
static TX_MUTEX nina_dns_lock;
static TX_MUTEX connect_state_lock;
static TX_SEMAPHORE ready_sem;
static TX_SEMAPHORE wifi_request;
/* The objects are made (set last, in the critical section). */
static volatile bool objects_made;

/* The critical section (see above). */
static inline UINT nina_os_irq_lock(void)
{
	return tx_interrupt_control(TX_INT_DISABLE);
}

static inline void nina_os_irq_unlock(UINT posture)
{
	(void)tx_interrupt_control(posture);
}

/* Makes the objects once: the first lock of any thread finds
 * objects_made still false.  The critical section keeps a second thread
 * from making them again.
 */
static void make_objects(void)
{
	UINT posture = nina_os_irq_lock();

	if (!objects_made) {
		(void)tx_mutex_create(&nina_lock, (CHAR *)"nina_lock", TX_INHERIT);
		(void)tx_mutex_create(&nina_dns_lock, (CHAR *)"nina_dns_lock", TX_INHERIT);
		(void)tx_mutex_create(&connect_state_lock, (CHAR *)"connect_state_lock",
				      TX_INHERIT);
		(void)tx_semaphore_create(&ready_sem, (CHAR *)"nina_ready", 0);
		(void)tx_semaphore_create(&wifi_request, (CHAR *)"wifi_request", 0);
		objects_made = true;
	}
	nina_os_irq_unlock(posture);
}

static inline void objects(void)
{
	if (!objects_made) {
		make_objects();
	}
}

/* Time ----------------------------------------------------------------------*/

/* US in ticks, rounded up (Zephyr's K_USEC() and K_MSEC():
 * k_us_to_ticks_ceil64()).  US is at most UINT32_MAX * 1000.
 */
static uint64_t ticks_ceil(uint64_t us)
{
	return (us * TX_TIMER_TICKS_PER_SECOND + 999999u) / 1000000u;
}

/* The wait option of a wait of TICKS: 0 does not block, else ticks + 1, at
 * most TX_WAIT_FOREVER - 1 (TX_WAIT_FOREVER would be for ever).
 */
static ULONG block_ticks(uint64_t ticks)
{
	if (ticks == 0) {
		return TX_NO_WAIT;
	}
	if (ticks >= (uint64_t)(TX_WAIT_FOREVER - 2)) {
		return TX_WAIT_FOREVER - 1;
	}
	return (ULONG)ticks + 1;
}

static void sleep_ticks(uint64_t ticks)
{
	if (ticks == 0) {
		/* Behind the other ready threads of the priority, as Zephyr's yield. */
		tx_thread_relinquish();
		return;
	}
	/* The current tick + ticks + 1, in steps that a sleep can hold. */
	for (ticks += 1; ticks > 0;) {
		ULONG step = ticks < (uint64_t)(TX_WAIT_FOREVER - 1) ? (ULONG)ticks
								     : TX_WAIT_FOREVER - 1;

		(void)tx_thread_sleep(step);
		ticks -= step;
	}
}

void nina_os_sleep_us(uint32_t us)
{
	sleep_ticks(ticks_ceil(us > INT32_MAX ? INT32_MAX : us));
}

void nina_os_sleep_ms(uint32_t ms)
{
	sleep_ticks(ticks_ceil((uint64_t)(ms > INT32_MAX ? INT32_MAX : ms) * 1000u));
}

int64_t nina_os_uptime_ms(void)
{
	return (int64_t)(time_us_64() / 1000u);
}

/* Locks and requests ----------------------------------------------------------*/

/* A wait for ever in a thread; none before the kernel runs (see above). */
static UINT mutex_take(TX_MUTEX *mutex)
{
	objects();
	return tx_mutex_get(mutex, tx_thread_identify() != TX_NULL ? TX_WAIT_FOREVER : TX_NO_WAIT);
}

int nina_os_lock(void)
{
	return mutex_take(&nina_lock) == TX_SUCCESS ? 0 : -EAGAIN;
}

void nina_os_unlock(void)
{
	(void)tx_mutex_put(&nina_lock);
}

void nina_os_dns_lock(void)
{
	(void)mutex_take(&nina_dns_lock);
}

void nina_os_dns_unlock(void)
{
	(void)tx_mutex_put(&nina_dns_lock);
}

void nina_os_wifi_lock(void)
{
	(void)mutex_take(&connect_state_lock);
}

void nina_os_wifi_unlock(void)
{
	(void)tx_mutex_put(&connect_state_lock);
}

void nina_os_wifi_post(void)
{
	objects();
	(void)tx_semaphore_ceiling_put(&wifi_request, 1);
}

void nina_os_wifi_wait(uint32_t ms)
{
	objects();
	(void)tx_semaphore_get(&wifi_request, block_ticks(ticks_ceil((uint64_t)ms * 1000u)));
}

/* The lines' interrupt ----------------------------------------------------------*/

/* A thread in nina_os_event_wait(), on its stack. */
struct event_waiter {
	struct event_waiter *next;
	TX_SEMAPHORE wake;
};

static volatile uint32_t event_count;
static struct event_waiter *event_waiters;
static bool attached;

/* In the interrupt handler: one more edge, every waiter woken.  The puts
 * inside the critical section only pend PendSV (an interrupt handler).
 */
static void event_edge(void)
{
	UINT posture = nina_os_irq_lock();
	struct event_waiter *w = event_waiters;

	event_count++;
	event_waiters = NULL;
	while (w != NULL) {
		struct event_waiter *next = w->next;

		w->next = NULL;
		(void)tx_semaphore_ceiling_put(&w->wake, 1);
		w = next;
	}
	nina_os_irq_unlock(posture);
}

/* IO_IRQ_BANK0, for the pins of NINA_GPIO_IRQ_PINS only. */
static void nina_os_gpio_isr(void)
{
	if (gpio_get_irq_event_mask(IRQ_GPIO) != 0) {
		gpio_acknowledge_irq(IRQ_GPIO, GPIO_EVENTS_ALL);
		event_edge();
	}
	if (gpio_get_irq_event_mask(READY_GPIO) != 0) {
		gpio_acknowledge_irq(READY_GPIO, GPIO_EVENTS_ALL);
		(void)tx_semaphore_ceiling_put(&ready_sem, 1);
	}
}

/* Adds the handler once (from a thread; the critical section keeps two
 * first callers from adding it twice).
 */
static void gpio_attach(void)
{
	UINT posture;

	Z_TX_ASSERT(objects_made);
	posture = nina_os_irq_lock();
	if (!attached) {
		gpio_add_raw_irq_handler_masked(NINA_GPIO_IRQ_PINS, nina_os_gpio_isr);
		irq_set_enabled(IO_IRQ_BANK0, true);
		attached = true;
	}
	nina_os_irq_unlock(posture);
}

/* All of PIN's events off, then EVENTS (0: none) on.  The pins' enables
 * are set and cleared through the registers' atomic aliases.
 */
static void gpio_events(unsigned int pin, uint32_t events)
{
	gpio_set_irq_enabled(pin, GPIO_EVENTS_ALL, false);
	if (events != 0) {
		gpio_set_irq_enabled(pin, events, true);
	}
}

/* READY ---------------------------------------------------------------------*/

int nina_os_ready_start(void)
{
	gpio_attach();
	return 0;
}

void nina_os_ready_arm(int desired)
{
	/* Active low: ready (1) is the falling edge. */
	uint32_t edge = desired ? GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE;

	(void)tx_semaphore_get(&ready_sem, TX_NO_WAIT);
	gpio_events(READY_GPIO, edge);
}

void nina_os_ready_wait(uint32_t ms)
{
	uint32_t slice = ms < READY_WAIT_SLICE_MS ? ms : READY_WAIT_SLICE_MS;

	(void)tx_semaphore_get(&ready_sem, block_ticks(ticks_ceil((uint64_t)slice * 1000u)));
}

void nina_os_ready_disarm(void)
{
	gpio_events(READY_GPIO, 0);
}

/* The socket events -------------------------------------------------------------*/

int nina_os_event_start(void)
{
	gpio_attach();
	gpio_events(IRQ_GPIO, GPIO_IRQ_EDGE_RISE);
	return 0;
}

uint32_t nina_os_event_seq(void)
{
	return event_count;
}

static void event_unlink(struct event_waiter *waiter)
{
	struct event_waiter **link;

	for (link = &event_waiters; *link != NULL; link = &(*link)->next) {
		if (*link == waiter) {
			*link = waiter->next;
			waiter->next = NULL;
			return;
		}
	}
}

void nina_os_event_wait(uint32_t seq, uint32_t ms)
{
	struct event_waiter waiter; /* the semaphore set by its creation */
	int64_t end = nina_os_uptime_ms() + ms;
	int64_t left = ms;
	UINT posture;

	if (event_count != seq || left <= 0) {
		return;
	}
	waiter.next = NULL;
	if (tx_semaphore_create(&waiter.wake, (CHAR *)"nina_event", 0) != TX_SUCCESS) {
		/* Earlier for no reason (nina_os.h): the caller asks again. */
		return;
	}
	while (event_count == seq && left > 0) {
		posture = nina_os_irq_lock();
		if (event_count != seq) {
			nina_os_irq_unlock(posture);
			break;
		}
		/* No put is left from an earlier round: the handler puts only
		 * the waiters on the list, and with the count it changes, which
		 * ends the loop.
		 */
		waiter.next = event_waiters;
		event_waiters = &waiter;
		nina_os_irq_unlock(posture);

		(void)tx_semaphore_get(&waiter.wake,
				       block_ticks(ticks_ceil((uint64_t)left * 1000u)));

		posture = nina_os_irq_lock();
		event_unlink(&waiter);
		nina_os_irq_unlock(posture);
		left = end - nina_os_uptime_ms();
	}
	(void)tx_semaphore_delete(&waiter.wake);
}
