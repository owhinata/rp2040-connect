/* SPDX-License-Identifier: MIT */
/* The waits for NINA's READY line on NuttX: drivers/nina/nina_os.c
 * (nina_os_ready_*(), on an edge interrupt) under the shared core's
 * wait_ready() (common/drivers/nina/nina_spi.c, built with
 * NINA_OS_WAIT_READY as nina/Make.defs builds it, and included here to
 * reach the static function), with the compatibility layer's gpio0
 * (common/compat/zephyr_gpio.c), on fakes of NuttX's semaphores, mutexes,
 * sleeps and clock (tests/nina_os_fakes) and of the board's GPIO bank 0
 * (<nano_gpio.h>, the board's src/nano_gpio.c on the target).
 *
 * The fake clock counts ticks of 100 us; only the semaphore's timed wait
 * and the sleeps run it.  READY (GPIO10, active low) changes at a planned
 * tick or right after a read; a change latches the pin's edge, and an
 * enabled edge runs the bank's handler (nano_gpio_irq_handler(), as the
 * interrupt does) at once.  Checked:
 * - READY already at the level: no wait;
 * - READY changes during the wait (towards ready: the falling edge; towards
 *   busy: the rising one): the wait ends at the change, one wait;
 * - no change: -ETIMEDOUT at the core's deadline, one wait of the whole
 *   time (MSEC2TICK(ms) ticks), two reads of READY;
 * - a change right after the core's read of READY, before its wait: the
 *   wait returns at once (not lost);
 * - a wake-up and a latched edge left from before the arm are dropped;
 * - the edge is enabled only during a wait, in the direction of the
 *   desired level, and the semaphore never counts more than one.
 *
 * Built a second time with NINA_OS_EVENTS (and CONFIG_NANO_NINA_USRSOCK),
 * as nina/Make.defs builds nina_os.c: the socket events on the IRQ line
 * (GPIO2): the start (the callback on gpio0, GPIO2's rising edge), the
 * count of the edges, the event thread woken by each
 * (nina_poller_edge()), a wait that ends at an edge, at once when the
 * count moved, after its time without an edge (MSEC2TICK(ms) ticks plus
 * one), an edge between the caller's read of the count and its wait (the
 * count left ends it at once), a count left from an earlier edge that
 * does not end a later wait.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>

#include "nano_gpio.h"

/* The core, with its static wait_ready(). */
#include "nina_spi.c"

#define READY_GPIO 10
#define IRQ_GPIO   2

#ifdef NINA_OS_EVENTS
static uint64_t irq_edge_at = UINT64_MAX; /* tick of a planned IRQ edge */
static bool irq_edge_in_wait;           /* an edge at the next wait's start */
static unsigned int poller_edges;
static void irq_edge(void);

/* nina_poller.c's: the interrupt wakes the event thread. */
void nina_poller_edge(void)
{
	poller_edges++;
}
#endif

/* Fake clock and kernel --------------------------------------------------*/

static uint64_t now;            /* ticks of 100 us */

static uint64_t change_at = UINT64_MAX; /* tick of the planned change */
static bool change_to_high;     /* the physical level it gives */
static bool change_after_read;  /* change right after the next read */

static unsigned int waits;      /* nxsem_tickwait_uninterruptible() calls */
static uint32_t last_delay;
static int max_count;

static void set_ready_high(bool high);

uint64_t clock_systime_ticks(void)
{
	return now;
}

static void advance(uint64_t ticks)
{
	uint64_t end = now + ticks;

	while (now < end) {
		now++;
		if (now == change_at) {
			change_at = UINT64_MAX;
			set_ready_high(change_to_high);
		}
#ifdef NINA_OS_EVENTS
		if (now == irq_edge_at) {
			irq_edge_at = UINT64_MAX;
			irq_edge();
		}
#endif
	}
}

void nxsched_usleep(unsigned int usec)
{
	advance(USEC2TICK(usec) + 1);
}

void nxsched_msleep(unsigned int msec)
{
	advance(MSEC2TICK(msec) + 1);
}

int nxrmutex_lock(rmutex_t *m)
{
	m->depth++;
	return 0;
}

int nxrmutex_unlock(rmutex_t *m)
{
	assert(m->depth > 0);
	m->depth--;
	return 0;
}

int nxmutex_lock(mutex_t *m)
{
	assert(!m->locked);
	m->locked = 1;
	return 0;
}

int nxmutex_unlock(mutex_t *m)
{
	assert(m->locked);
	m->locked = 0;
	return 0;
}

/* The semaphore the test waits on (a NuttX waiter: the count reads one
 * less, -1 at 0), and whether a post has handed it its count. */
static sem_t *waiting_sem;
static bool handed;

int nxsem_post(sem_t *sem)
{
	if (sem == waiting_sem && !handed && sem->count == 0) {
		handed = true;
		return 0;
	}
	sem->count++;
	if (sem->count > max_count) {
		max_count = sem->count;
	}
	return 0;
}

int nxsem_get_value(sem_t *sem, int *sval)
{
	*sval = sem->count - (sem == waiting_sem && !handed ? 1 : 0);
	return 0;
}

int nxsem_reset(sem_t *sem, int16_t count)
{
	sem->count = count;
	return 0;
}

/* NuttX: the watchdog fires at clock_systime_ticks() + delay + 1. */
int nxsem_tickwait_uninterruptible(sem_t *sem, uint32_t delay)
{
	uint64_t end = now + delay + 1;

	waits++;
	last_delay = delay;
#ifdef NINA_OS_EVENTS
	/* The edge between the caller's read of the count and its wait. */
	if (irq_edge_in_wait) {
		irq_edge_in_wait = false;
		irq_edge();
	}
#endif
	if (sem->count > 0) {
		sem->count--;
		return 0;
	}
	waiting_sem = sem;
	handed = false;
	for (;;) {
		if (handed) {
			waiting_sem = NULL;
			return 0;
		}
		if (now >= end) {
			waiting_sem = NULL;
			return -ETIMEDOUT;
		}
		advance(1);
	}
}

/* Fake GPIO bank 0 (the board's <nano_gpio.h>) ---------------------------*/

static bool ready_high = true;  /* physical level: high is busy */
static uint32_t ready_intr;     /* latched edges (INTR) */
static uint32_t ready_inte;     /* enabled events (PROC0_INTE) */
static unsigned int ready_irqs;
static unsigned int ready_reads;

int nano_gpio_configure_input(unsigned int pin, bool pull_up, bool pull_down)
{
	(void)pin;
	(void)pull_up;
	(void)pull_down;
	assert(0);
	return -EINVAL;
}

uint32_t nano_gpio_get_all(void)
{
	return ready_high ? 1U << READY_GPIO : 0;
}

#ifdef NINA_OS_EVENTS
static uint32_t irq_inte;       /* GPIO2's enabled events */
static unsigned int irq_irqs;

/* A rising edge of the IRQ line: the bank's interrupt if it is enabled. */
static void irq_edge(void)
{
	if (irq_inte & NANO_GPIO_IRQ_EDGE_RISE) {
		irq_irqs++;
		nano_gpio_irq_handler(IRQ_GPIO);
	}
}
#endif

/* As the board's: clears the pin's latched edges, then enables EVENTS. */
int nano_gpio_irq_configure(unsigned int pin, uint32_t events)
{
#ifdef NINA_OS_EVENTS
	if (pin == IRQ_GPIO) {
		assert(events == NANO_GPIO_IRQ_EDGE_RISE);
		irq_inte = events;
		return 0;
	}
#endif
	assert(pin == READY_GPIO);
	assert(events == 0 || events == NANO_GPIO_IRQ_EDGE_FALL ||
	       events == NANO_GPIO_IRQ_EDGE_RISE);
	ready_intr = 0;
	ready_inte = events;
	return 0;
}

/* The bank's interrupt, run at once when an enabled edge latches. */
static void bank_interrupt(void)
{
	uint32_t pending = ready_intr & ready_inte;

	if (pending != 0) {
		ready_intr &= ~pending;
		ready_irqs++;
		nano_gpio_irq_handler(READY_GPIO);
	}
}

static void set_ready_high(bool high)
{
	if (high == ready_high) {
		return;
	}
	ready_high = high;
	ready_intr |= high ? NANO_GPIO_IRQ_EDGE_RISE : NANO_GPIO_IRQ_EDGE_FALL;
	bank_interrupt();
}

/* Fake board side of the link (<nano_nina.h>) ----------------------------*/

int nano_nina_initialize(void)
{
	return 0;
}

bool nano_nina_bus_ready(void)
{
	return true;
}

int nano_nina_pin_configure(enum nano_nina_pin pin, enum nano_nina_mode mode)
{
	(void)pin;
	(void)mode;
	return 0;
}

int nano_nina_pin_set(enum nano_nina_pin pin, int value)
{
	(void)pin;
	(void)value;
	return 0;
}

int nano_nina_pin_get(enum nano_nina_pin pin)
{
	int value;

	assert(pin == NANO_NINA_READY);
	ready_reads++;
	value = !ready_high;
	if (change_after_read) {
		change_after_read = false;
		set_ready_high(!ready_high);
	}
	return value;
}

int nano_nina_transceive(const uint8_t *tx, uint8_t *rx, size_t len)
{
	(void)tx;
	(void)rx;
	(void)len;
	assert(0);
	return -EIO;
}

/* Tests --------------------------------------------------------------------*/

static uint64_t start;

static void reset(bool high)
{
	ready_high = high;
	ready_intr = 0;
	ready_irqs = 0;
	ready_reads = 0;
	waits = 0;
	last_delay = 0;
	change_at = UINT64_MAX;
	change_after_read = false;
	start = now;
}

static void plan_change(uint64_t ticks, bool high)
{
	change_at = now + ticks;
	change_to_high = high;
}

/* READY at the level already: no wait, the edge enabled and off again. */
static void test_at_level(void)
{
	reset(false);
	assert(wait_ready(1, 2000) == 0);
	assert(now == start && waits == 0 && ready_reads == 1 && ready_inte == 0);

	reset(true);
	assert(wait_ready(0, 20) == 0);
	assert(now == start && waits == 0 && ready_reads == 1 && ready_inte == 0);
}

/* The module gets ready 50 ms into the wait: the falling edge ends it. */
static void test_ready_during_wait(void)
{
	reset(true);
	plan_change(500, false);
	assert(wait_ready(1, 2000) == 0);
	assert(now - start == 500);
	assert(waits == 1 && last_delay == MSEC2TICK(2000));
	assert(ready_irqs == 1 && ready_reads == 2 && ready_inte == 0);
}

/* The module takes READY back 5 ms after CS: the rising edge ends it. */
static void test_busy_during_wait(void)
{
	reset(false);
	plan_change(50, true);
	assert(wait_ready(0, 20) == 0);
	assert(now - start == 50);
	assert(waits == 1 && ready_irqs == 1 && ready_reads == 2 && ready_inte == 0);
}

/* No change: -ETIMEDOUT at the deadline (the wait's 20000 ticks plus one,
 * as NuttX's timeout), after one wait and two reads.
 */
static void test_timeout(void)
{
	reset(true);
	assert(wait_ready(1, 2000) == -ETIMEDOUT);
	assert(now - start == MSEC2TICK(2000) + 1);
	assert(waits == 1 && last_delay == MSEC2TICK(2000));
	assert(ready_irqs == 0 && ready_reads == 2 && ready_inte == 0);

	reset(false);
	assert(wait_ready(0, 20) == -ETIMEDOUT);
	assert(now - start == MSEC2TICK(20) + 1);
	assert(waits == 1 && ready_irqs == 0 && ready_reads == 2 && ready_inte == 0);
}

/* READY changes right after the core read it and before it waits: the
 * edge, enabled before the read, has posted the semaphore; the wait
 * returns at once.
 */
static void test_change_after_read(void)
{
	reset(true);
	change_after_read = true;
	assert(wait_ready(1, 2000) == 0);
	assert(now == start);
	assert(waits == 1 && ready_irqs == 1 && ready_reads == 2 && ready_inte == 0);

	reset(false);
	change_after_read = true;
	assert(wait_ready(0, 20) == 0);
	assert(now == start);
	assert(waits == 1 && ready_irqs == 1 && ready_reads == 2 && ready_inte == 0);
}

/* A wake-up and an edge left from before (a change at the end of a
 * transfer, while the edge was off) do not end the next wait early.
 */
static void test_stale_wakeup(void)
{
	reset(true);
	nano_gpio_irq_handler(READY_GPIO);      /* the semaphore counts one */
	ready_intr = NANO_GPIO_IRQ_EDGE_FALL | NANO_GPIO_IRQ_EDGE_RISE;
	assert(wait_ready(1, 20) == -ETIMEDOUT);
	assert(now - start == MSEC2TICK(20) + 1);
	assert(waits == 1 && ready_irqs == 0 && ready_inte == 0);
}

/* After the wait the edge is off: changes of READY during the transfers
 * run no handler.  Two interrupts before the waiter runs count one (a
 * k_sem with limit 1, as in the FreeRTOS build).
 */
static void test_edges(void)
{
	reset(true);
	plan_change(10, false);
	assert(wait_ready(1, 20) == 0);
	assert(now - start == 10 && ready_irqs == 1 && ready_inte == 0);
	set_ready_high(true);
	set_ready_high(false);
	assert(ready_irqs == 1);

	max_count = 0;
	nano_gpio_irq_handler(READY_GPIO);
	nano_gpio_irq_handler(READY_GPIO);
	assert(max_count == 1);
}

#ifdef NINA_OS_EVENTS
static void test_events(void)
{
	uint32_t seq;

	/* Nothing before the start. */
	irq_edge();
	assert(irq_irqs == 0 && nina_os_event_seq() == 0);
	assert(nina_os_event_start() == 0 && irq_inte == NANO_GPIO_IRQ_EDGE_RISE);
	/* An edge: the count, the event thread woken, one count left on the
	 * semaphore (none waits). */
	max_count = 0;
	irq_edge();
	assert(irq_irqs == 1 && nina_os_event_seq() == 1 && poller_edges == 1 && max_count == 1);

	/* The count moved since SEQ: at once. */
	reset(true);
	nina_os_event_wait(0, 100);
	assert(waits == 0 && now == start);

	/* The next edge 7 ms (70 ticks) later: the count left is taken (one
	 * wait at once), then the wait ends at the edge. */
	seq = nina_os_event_seq();
	reset(true);
	irq_edge_at = now + 70;
	nina_os_event_wait(seq, 100);
	assert(nina_os_event_seq() == seq + 1 && now - start == 70 && waits == 2);
	assert(poller_edges == 2);

	/* No edge: MSEC2TICK(100) ticks plus one, after taking the count left
	 * by the last edge. */
	seq = nina_os_event_seq();
	reset(true);
	nina_os_event_wait(seq, 100);
	assert(nina_os_event_seq() == seq && waits >= 2);
	assert(now - start >= MSEC2TICK(100) && now - start <= MSEC2TICK(100) + 10);

	/* An edge after the caller read the count, before its wait: the
	 * count it leaves ends the wait at once. */
	seq = nina_os_event_seq();
	reset(true);
	irq_edge_in_wait = true;
	nina_os_event_wait(seq, 100);
	assert(nina_os_event_seq() == seq + 1 && now == start && waits == 1);

	/* 0 ms: no wait. */
	seq = nina_os_event_seq();
	reset(true);
	nina_os_event_wait(seq, 0);
	assert(waits == 0);
	/* Two edges before a waiter runs: one count. */
	max_count = 0;
	irq_edge();
	irq_edge();
	assert(max_count == 1);
}
#endif

int main(void)
{
	test_at_level();
	test_ready_during_wait();
	test_busy_during_wait();
	test_timeout();
	test_change_after_read();
	test_stale_wakeup();
	test_edges();
#ifdef NINA_OS_EVENTS
	test_events();
#endif
	printf("nina_os: ok\n");
	return 0;
}
