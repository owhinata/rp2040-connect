/* SPDX-License-Identifier: MIT */
/* The generic kernel part of the Zephyr compatibility layer, for an OS that
 * offers only the primitives of zephyr_kernel_os.h: semaphores, k_poll() on
 * semaphores, the start of a thread, the timers of delayable work items and
 * the sleeps, with Zephyr's rules.  The work queue logic, the work queues'
 * threads and the start of what Zephyr starts at boot are
 * common/compat/zephyr_work.c and zephyr_init.c; the mutexes are
 * zephyr_mutex.c; the types are in zephyr_os.h.  No OS call but the
 * primitives (those of the FreeRTOS and ThreadX builds are
 * common/compat/port/<OS>/zephyr_kernel_os_impl.h; NuttX
 * implements the Zephyr-level functions directly instead,
 * common/compat/port/nuttx/zephyr_kernel.c).
 *
 *   Zephyr                    here
 *   k_sem                     Zephyr's own rules (kernel/sem.c) on a count
 *                             of the layer: a give hands the count to the
 *                             first waiter (highest priority, then first
 *                             come) or adds one up to the limit and signals
 *                             the first k_poll() event; k_sem_reset() ends
 *                             the waits with -EAGAIN.  A waiting thread
 *                             sleeps on its wake flag (z_os_wait())
 *   k_poll (semaphores)       Zephyr's kernel/poll.c for semaphores in
 *                             K_POLL_MODE_NOTIFY_ONLY: the events are
 *                             registered on their semaphores by the priority
 *                             of their thread; a give signals one event and
 *                             wakes its thread (z_os_wake())
 *   from an interrupt         k_sem_give() (and so k_work_submit()) wakes
 *                             with z_os_wake_from_isr() and asks for a
 *                             switch at the end of the handler
 *                             (z_os_yield_from_isr()): the woken thread runs
 *                             right after the handler
 *   K_THREAD_DEFINE() thread  z_os_thread_start() on its static stack,
 *                             started by z_thread_static_start() (the start
 *                             of the application; the layer's own threads,
 *                             the work queues' among them, the same way);
 *                             it ends (z_os_thread_exit()) when entry returns
 *   _timeout of an item       an entry in the layer's list of timeouts,
 *                             sorted by deadline, run by the tick hook
 *                             (zephyr_compat_tick()) in the tick interrupt,
 *                             as Zephyr runs its timeouts in the system clock
 *                             interrupt (zephyr_work.c relies on a timer that
 *                             cannot run while interrupts are masked)
 *   k_thread_name_set()       the thread's name array (z_os_thread_name())
 *   k_sleep(), k_msleep(),    z_os_sleep(ticks + 1): Zephyr's wake-up at the
 *   k_usleep()                current tick + ticks + 1; K_NO_WAIT (0 or less)
 *                             yields (z_os_yield(): behind the other threads of
 *                             the priority); K_FOREVER suspends the
 *                             thread (z_os_suspend())
 *   k_busy_wait()             z_os_busy_wait_us()
 *
 * Every relative wait (k_sem_take(), k_mutex_lock(), k_poll(), the sleeps)
 * is the OS's wait of ticks + 1 (z_wait_ticks()), so it ends between ticks
 * and ticks + 1 ticks after the call, as in Zephyr.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "zephyr_compat_os.h"
#include "zephyr_kernel_os.h"
#include "zephyr_os_internal.h"

/* Waits ---------------------------------------------------------------------*/

z_os_ticks_t z_wait_ticks(k_timeout_t timeout)
{
	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
		return Z_OS_TICKS_FOREVER;
	}
	if (timeout.ticks <= 0) {
		return 0;
	}
	if ((uint64_t)timeout.ticks >= (uint64_t)(Z_OS_TICKS_FOREVER - 2)) {
		return Z_OS_TICKS_FOREVER - 1;
	}
	return (z_os_ticks_t)timeout.ticks + 1;
}

/* The lock of the semaphores, their waiters and their poll events (Zephyr:
 * the semaphores' and poll_lock).
 */
static struct k_spinlock sem_lock;

/* A thread waiting in k_sem_take(), on its stack. */
struct z_sem_waiter {
	struct z_sem_waiter *next;
	k_tid_t task;
	z_os_prio_t prio;
	int result;
	volatile bool done; /* set with result by k_sem_give(), k_sem_reset() */
};

/* A thread waiting in k_poll(), on its stack; the events point to it. */
struct z_poller {
	k_tid_t task;
	z_os_prio_t prio;
};

/* Before a thread waits (with sem_lock held): drops a wake left by an
 * earlier wait that was woken as it ended (the waker sends one; the thread
 * may have found the result and returned without taking it).
 */
static void wait_prepare(void)
{
	z_os_wait_prepare();
}

static void wait_notified(z_os_ticks_t ticks)
{
	z_os_wait(ticks);
}

/* Wakes a waiting thread (with sem_lock held).  In an interrupt handler,
 * sets *woken when the thread should run at the end of it; in a thread the
 * switch, if any, waits for the unlock.
 */
static void wake(k_tid_t task, z_os_woken_t *woken)
{
	if (z_os_in_isr()) {
		z_os_wake_from_isr(task, woken);
	} else {
		z_os_wake(task);
	}
}

static void unlock_and_switch(k_spinlock_key_t key, z_os_woken_t woken)
{
	k_spin_unlock(&sem_lock, key);
	if (z_os_in_isr()) {
		z_os_yield_from_isr(woken);
	}
}

/* Semaphores ----------------------------------------------------------------*/

int k_sem_init(struct k_sem *sem, unsigned int initial_count, unsigned int limit)
{
	if (limit == 0 || initial_count > limit) {
		return -EINVAL;
	}
	*sem = (struct k_sem)Z_SEM_INITIALIZER(*sem, initial_count, limit);
	return 0;
}

/* Higher priority (z_os_prio_t) first, first come first within one
 * (Zephyr's wait queue).
 */
static void waiter_insert(struct z_sem_waiter **link, struct z_sem_waiter *waiter)
{
	while (*link != NULL && (*link)->prio >= waiter->prio) {
		link = &(*link)->next;
	}
	waiter->next = *link;
	*link = waiter;
}

static void waiter_remove(struct z_sem_waiter **link, struct z_sem_waiter *waiter)
{
	while (*link != NULL) {
		if (*link == waiter) {
			*link = waiter->next;
			waiter->next = NULL;
			return;
		}
		link = &(*link)->next;
	}
}

int k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
	struct z_sem_waiter waiter = {
		.result = -EAGAIN,
		.done = false,
	};
	z_os_ticks_t ticks = z_wait_ticks(timeout);
	k_spinlock_key_t key;

	if (!z_os_in_isr()) {
		waiter.task = z_os_thread_self();
		waiter.prio = z_os_thread_prio_self();
	}

	key = k_spin_lock(&sem_lock);
	if (sem->count > 0) {
		sem->count--;
		k_spin_unlock(&sem_lock, key);
		return 0;
	}
	/* Zephyr: an interrupt handler may only take with K_NO_WAIT. */
	if (ticks == 0 || z_os_in_isr()) {
		k_spin_unlock(&sem_lock, key);
		return -EBUSY;
	}
	wait_prepare();
	waiter_insert(&sem->waiters, &waiter);
	k_spin_unlock(&sem_lock, key);

	do {
		wait_notified(ticks);
	} while (ticks == Z_OS_TICKS_FOREVER && !waiter.done);

	key = k_spin_lock(&sem_lock);
	if (!waiter.done) {
		waiter_remove(&sem->waiters, &waiter);
	}
	k_spin_unlock(&sem_lock, key);
	return waiter.done ? waiter.result : -EAGAIN;
}

/* kernel/poll.c z_handle_obj_poll_events(): the first registered event (of
 * the highest-priority poller) only.
 */
static void poll_signal(struct k_sem *sem, z_os_woken_t *woken)
{
	struct k_poll_event *event = sem->poll_events;
	struct z_poller *poller;

	if (event == NULL) {
		return;
	}
	sem->poll_events = event->next;
	event->next = NULL;
	poller = event->waiter;
	event->waiter = NULL;
	event->state |= K_POLL_STATE_SEM_AVAILABLE;
	if (poller != NULL) {
		wake(poller->task, woken);
	}
}

/* kernel/sem.c z_impl_k_sem_give(): a waiter takes the count at once; with
 * none, the count goes up (not past the limit) and one poll event is
 * signaled (also at the limit).
 */
void k_sem_give(struct k_sem *sem)
{
	z_os_woken_t woken = 0;
	k_spinlock_key_t key = k_spin_lock(&sem_lock);
	struct z_sem_waiter *waiter = sem->waiters;

	if (waiter != NULL) {
		sem->waiters = waiter->next;
		waiter->next = NULL;
		waiter->result = 0;
		waiter->done = true;
		wake(waiter->task, &woken);
	} else {
		if (sem->count != sem->limit) {
			sem->count++;
		}
		poll_signal(sem, &woken);
	}
	unlock_and_switch(key, woken);
}

/* kernel/sem.c z_impl_k_sem_reset(): the waiters return -EAGAIN. */
void k_sem_reset(struct k_sem *sem)
{
	z_os_woken_t woken = 0;
	k_spinlock_key_t key = k_spin_lock(&sem_lock);
	struct z_sem_waiter *waiter;

	while ((waiter = sem->waiters) != NULL) {
		sem->waiters = waiter->next;
		waiter->next = NULL;
		waiter->result = -EAGAIN;
		waiter->done = true;
		wake(waiter->task, &woken);
	}
	sem->count = 0;
	unlock_and_switch(key, woken);
}

unsigned int k_sem_count_get(struct k_sem *sem)
{
	return *(volatile unsigned int *)&sem->count;
}

/* Polling -------------------------------------------------------------------*/

void k_poll_event_init(struct k_poll_event *event, uint32_t type, int mode, void *obj)
{
	*event = (struct k_poll_event){
		.type = type,
		.mode = (uint32_t)mode,
		.state = K_POLL_STATE_NOT_READY,
		.sem = obj,
	};
}

/* Marks the events whose semaphore has a count; returns how many events are
 * ready.
 */
static int poll_ready(struct k_poll_event *events, int num_events)
{
	int ready = 0;

	for (int i = 0; i < num_events; i++) {
		if (events[i].type == K_POLL_TYPE_SEM_AVAILABLE && events[i].sem->count > 0) {
			events[i].state |= K_POLL_STATE_SEM_AVAILABLE;
		}
		ready += events[i].state != K_POLL_STATE_NOT_READY;
	}
	return ready;
}

static int poll_signaled(const struct k_poll_event *events, int num_events)
{
	int ready = 0;

	for (int i = 0; i < num_events; i++) {
		ready += events[i].state != K_POLL_STATE_NOT_READY;
	}
	return ready;
}

/* kernel/poll.c add_event(): by the priority of the polling thread, after
 * the events of the same priority.
 */
static void poll_register(struct k_poll_event *event, struct z_poller *poller)
{
	struct k_poll_event **link = &event->sem->poll_events;

	while (*link != NULL && ((struct z_poller *)(*link)->waiter)->prio >= poller->prio) {
		link = &(*link)->next;
	}
	event->waiter = poller;
	event->next = *link;
	*link = event;
}

static void poll_unregister(struct k_poll_event *event)
{
	struct k_poll_event **link = &event->sem->poll_events;

	while (*link != NULL) {
		if (*link == event) {
			*link = event->next;
			break;
		}
		link = &(*link)->next;
	}
	event->next = NULL;
	event->waiter = NULL;
}

int k_poll(struct k_poll_event *events, int num_events, k_timeout_t timeout)
{
	struct z_poller poller = {0};
	z_os_ticks_t ticks = z_wait_ticks(timeout);
	k_spinlock_key_t key;
	int ready;

	if (!z_os_in_isr()) {
		poller.task = z_os_thread_self();
		poller.prio = z_os_thread_prio_self();
	}

	key = k_spin_lock(&sem_lock);
	for (int i = 0; i < num_events; i++) {
		events[i].state = K_POLL_STATE_NOT_READY;
	}
	ready = poll_ready(events, num_events);
	if (ready == 0 && ticks != 0 && !z_os_in_isr()) {
		wait_prepare();
		for (int i = 0; i < num_events; i++) {
			if (events[i].type == K_POLL_TYPE_SEM_AVAILABLE) {
				poll_register(&events[i], &poller);
			}
		}
		do {
			k_spin_unlock(&sem_lock, key);
			wait_notified(ticks);
			key = k_spin_lock(&sem_lock);
			ready = poll_signaled(events, num_events);
		} while (ticks == Z_OS_TICKS_FOREVER && ready == 0);

		for (int i = 0; i < num_events; i++) {
			if (events[i].type == K_POLL_TYPE_SEM_AVAILABLE &&
			    events[i].waiter == &poller) {
				poll_unregister(&events[i]);
			}
		}
	}
	k_spin_unlock(&sem_lock, key);
	return ready > 0 ? 0 : -EAGAIN;
}

/* Threads -------------------------------------------------------------------*/

int k_thread_name_set(k_tid_t thread, const char *name)
{
	k_tid_t task = thread != NULL ? thread : z_os_thread_self();
	k_spinlock_key_t key;
	char *dst;
	size_t i;

	if (task == NULL || name == NULL) {
		return -EINVAL;
	}
	/* The thread's own array (Z_OS_THREAD_NAME_SIZE, 32 as Zephyr's
	 * CONFIG_THREAD_MAX_NAME_LEN in the FreeRTOS and ThreadX builds); a
	 * longer name is cut, as in Zephyr.
	 */
	dst = z_os_thread_name(task);
	key = k_spin_lock(&sem_lock);
	for (i = 0; i < Z_OS_THREAD_NAME_SIZE - 1 && name[i] != '\0'; i++) {
		dst[i] = name[i];
	}
	dst[i] = '\0';
	k_spin_unlock(&sem_lock, key);
	return 0;
}

static void static_thread_main(void *arg)
{
	const struct z_thread_static *thread = arg;

	thread->entry(thread->p1, thread->p2, thread->p3);
	z_os_thread_exit();
}

int z_thread_static_start(const struct z_thread_static *thread)
{
	return z_os_thread_start(static_thread_main, thread->name, thread->stack,
				 thread->stack_size, thread->prio, (void *)thread, thread->tid);
}

/* Time ----------------------------------------------------------------------*/

int32_t k_sleep(k_timeout_t timeout)
{
	uint64_t ticks;

	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
		/* Zephyr: k_thread_suspend(_current). */
		for (;;) {
			z_os_suspend();
		}
	}
	if (timeout.ticks <= 0) {
		z_os_yield();
		return 0;
	}

	/* The current tick + ticks + 1, in steps that a finite wait can hold. */
	for (ticks = (uint64_t)timeout.ticks + 1; ticks > 0;) {
		z_os_ticks_t step = ticks < (uint64_t)(Z_OS_TICKS_FOREVER - 1)
					    ? (z_os_ticks_t)ticks
					    : Z_OS_TICKS_FOREVER - 1;

		z_os_sleep(step);
		ticks -= step;
	}
	return 0;
}

int32_t k_msleep(int32_t ms)
{
	return k_sleep(K_MSEC(ms));
}

/* Zephyr: k_us_to_ticks_ceil64(us), then as k_sleep(). */
int32_t k_usleep(int32_t us)
{
	int64_t ticks = us > 0 ? ((int64_t)us + Z_USEC_PER_TICK - 1) / Z_USEC_PER_TICK : 0;

	return k_sleep(K_TICKS(ticks));
}

void k_busy_wait(uint32_t usec_to_wait)
{
	if (usec_to_wait > 0) {
		z_os_busy_wait_us(usec_to_wait);
	}
}

/* Timeouts of delayable items -----------------------------------------------*/

/* The longest timeout: the deadlines are compared as signed 32-bit
 * differences (about 24.8 days of 1 ms ticks).
 */
#define TIMEOUT_TICKS_MAX ((int64_t)INT32_MAX - 1)

/* Sorted by deadline, the earliest first; entries with the same deadline in
 * the order they were added.  timeout_tick counts the calls of
 * zephyr_compat_tick(), one per tick interrupt.
 */
static struct _timeout *timeout_head;
static volatile uint32_t timeout_tick;
static struct k_spinlock timeout_lock;

static void timeout_unlink(struct _timeout *to)
{
	struct _timeout **link = &timeout_head;

	while (*link != NULL) {
		if (*link == to) {
			*link = to->next;
			break;
		}
		link = &(*link)->next;
	}
	to->next = NULL;
	to->linked = false;
}

/* kernel/timeout.c z_add_timeout(): ends at the current tick + ticks + 1;
 * K_FOREVER adds nothing.
 */
void z_add_timeout(struct _timeout *to, _timeout_func_t fn, k_timeout_t timeout)
{
	k_spinlock_key_t key = k_spin_lock(&timeout_lock);
	struct _timeout **link = &timeout_head;
	int64_t ticks = timeout.ticks;
	uint32_t now = timeout_tick;
	uint32_t left;

	if (to->linked) {
		timeout_unlink(to);
	}
	if (K_TIMEOUT_EQ(timeout, K_FOREVER)) {
		k_spin_unlock(&timeout_lock, key);
		return;
	}
	ticks = ticks < 0 ? 0 : (ticks > TIMEOUT_TICKS_MAX ? TIMEOUT_TICKS_MAX : ticks);

	to->fn = fn;
	to->deadline = now + (uint32_t)ticks + 1;
	left = to->deadline - now;
	while (*link != NULL && (*link)->deadline - now <= left) {
		link = &(*link)->next;
	}
	to->next = *link;
	*link = to;
	to->linked = true;
	k_spin_unlock(&timeout_lock, key);
}

void z_abort_timeout(struct _timeout *to)
{
	k_spinlock_key_t key = k_spin_lock(&timeout_lock);

	if (to->linked) {
		timeout_unlink(to);
	}
	k_spin_unlock(&timeout_lock, key);
}

/* The tick interrupt (interrupts masked, see zephyr_compat_os.h).  A function
 * may add its timeout again (the list is consistent when it runs).
 */
void zephyr_compat_tick(void)
{
	k_spinlock_key_t key = k_spin_lock(&timeout_lock);
	uint32_t now = timeout_tick + 1;

	timeout_tick = now;
	while (timeout_head != NULL && (int32_t)(timeout_head->deadline - now) <= 0) {
		struct _timeout *to = timeout_head;

		timeout_head = to->next;
		to->next = NULL;
		to->linked = false;
		to->fn(to);
	}
	k_spin_unlock(&timeout_lock, key);
}
