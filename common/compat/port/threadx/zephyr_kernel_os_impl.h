/* SPDX-License-Identifier: MIT */
/* The ThreadX primitives of the generic kernel part of the Zephyr
 * compatibility layer (common/compat/port/zephyr_kernel_os.h,
 * which states what each must do; read only through it).  What depends on
 * the ThreadX port and the board (z_os_in_isr(), the interrupt mask
 * z_tx_irq_save() and z_tx_irq_restore(), z_os_busy_wait_us() and the
 * clock) is the port header "zephyr_os_port.h" (common/compat/port/threadx/rp2040
 * on the board, threadx/tests/compat on the host's ThreadX Linux port); the
 * rest is the same ThreadX calls on both:
 *
 *   thread               a TX_THREAD in the Z_OS_THREAD_RESERVED bytes after
 *                        the thread's stack (struct z_tx_thread, with the
 *                        wake flag and the name array)
 *   z_os_task_lock()     interrupts masked and ThreadX's preemption
 *                        disabled (_tx_thread_preempt_disable), switch at
 *                        the outermost unlock (_tx_thread_system_preempt_check())
 *   z_os_isr_lock()      interrupts masked (the port's mask, its state)
 *   wake flag            a binary semaphore of the thread's own
 *                        (tx_semaphore_ceiling_put(), ceiling 1: latched; a
 *                        put before the get makes the get return at once)
 *   z_os_thread_start()  tx_thread_create() unstarted, then
 *                        tx_thread_resume(); ThreadX's time slice for the
 *                        sliced priorities
 *   z_os_sleep()         tx_thread_sleep(): ThreadX's timeouts end at the
 *                        current tick + n
 *   z_os_thread_local_*()  the TX_THREAD_USER_EXTENSION member
 *                        tx_thread_zephyr_local
 *   z_os_suspend()       tx_thread_suspend() of the caller
 *   k_mutex              a ThreadX mutex with priority inheritance
 *                        (TX_INHERIT), recursive for its owner, created by
 *                        its first use, and kept out of ThreadX's list of
 *                        created mutexes
 *   Z_OS_ASSERT()        the port header's
 *
 * Every thread that waits through the layer (k_sem_take(), k_poll(), the
 * work queues) must have been started by z_os_thread_start(): the wake
 * flag is part of the storage it lays out after the stack.
 *
 * What the application provides: tx_user.h (TX_TIMER_TICKS_PER_SECOND
 * 1000, TX_THREAD_USER_EXTENSION with "VOID *tx_thread_zephyr_local;",
 * TX_TIMER_PROCESS_IN_ISR so that sleeps and timeouts end in the tick
 * interrupt), a tick that calls zephyr_compat_tick() of "zephyr_compat_os.h"
 * once per ThreadX tick from the tick interrupt, and what the port header
 * reads; priorities_os.h of this directory (common/compat/include/priorities.h;
 * also the time slices, APP_TIMESLICE_TICKS and APP_TIMESLICE_PRIO_MAX) is
 * the board's.  Part of the
 * ThreadX implementation of the layer (common/compat/port/threadx).
 */

#ifndef COMMON_COMPAT_PORT_THREADX_ZEPHYR_KERNEL_OS_IMPL_H
#define COMMON_COMPAT_PORT_THREADX_ZEPHYR_KERNEL_OS_IMPL_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "tx_api.h"
#include "tx_initialize.h"
#include "tx_thread.h"
#include "tx_mutex.h"
#include "tx_semaphore.h"

#include "priorities_os.h"
#include "zephyr_os_port.h"

/* Types and constants ------------------------------------------------------*/

typedef TX_THREAD *z_os_thread_t;
typedef UINT z_os_prio_t;
typedef ULONG z_os_ticks_t;
typedef UINT z_os_woken_t;

#define Z_OS_TICKS_FOREVER TX_WAIT_FOREVER
#define Z_OS_TICK_HZ       TX_TIMER_TICKS_PER_SECOND

#if Z_OS_TICK_HZ != 1000
#error "the compatibility layer runs on a 1 ms tick (tx_user.h: TX_TIMER_TICKS_PER_SECOND 1000)"
#endif

#ifndef TX_TIMER_PROCESS_IN_ISR
#error "the compatibility layer needs TX_TIMER_PROCESS_IN_ISR (tx_user.h)"
#endif

/* 32 bytes with the NUL, as Zephyr's CONFIG_THREAD_MAX_NAME_LEN. */
#define Z_OS_THREAD_NAME_SIZE 32

/* A layer priority (APP_PRIO_FROM_ZEPHYR(), larger more urgent) as
 * ThreadX's (0 most urgent), and back.  Layer priority 0 (Zephyr's idle,
 * ThreadX's lowest) is not given to a thread of the layer.
 */
#define Z_TX_PRIORITY(prio)       ((UINT)(TX_MAX_PRIORITIES - 1) - (UINT)(prio))
#define Z_TX_PRIO_FROM_TX(txprio) ((z_os_prio_t)(TX_MAX_PRIORITIES - 1) - (z_os_prio_t)(txprio))

/* The storage of a thread after its stack (z_os_thread_start()): its
 * control block (first, so that a z_os_thread_t is this structure), its
 * wake flag, the function it runs (ThreadX passes a ULONG to the entry,
 * too small for a pointer on the host) and the array of its name, to which
 * the control block's tx_thread_name points.
 */
struct z_tx_thread {
	TX_THREAD thread;
	TX_SEMAPHORE wake;
	void (*fn)(void *arg);
	void *arg;
	char name[Z_OS_THREAD_NAME_SIZE];
};

#define Z_OS_THREAD_RESERVED ((sizeof(struct z_tx_thread) + 7u) & ~(size_t)7u)

/* k_mutex: a ThreadX mutex in the structure's own memory.  tx_mutex_id is
 * TX_MUTEX_ID once it is created; a k_mutex left all zero is created by
 * its first k_mutex_lock() (z_tx_mutex_get()), so K_MUTEX_DEFINE() and a
 * never initialized mutex work as unlocked ones, as Zephyr's do.
 */
struct k_mutex {
	TX_MUTEX mutex;
};

#define Z_MUTEX_INITIALIZER(obj)                                                                   \
	{                                                                                          \
		.mutex = {.tx_mutex_id = 0 }                                                       \
	}

#define Z_OS_ASSERT(test) Z_TX_ASSERT(test)

/* As in the FreeRTOS build, the primitives that stand for one or two
 * ThreadX calls are always inlined.
 */
#define Z_OS_INLINE static inline __attribute__((always_inline))

/* Context and lock (z_os_in_isr(): the port header) -------------------------*/

Z_OS_INLINE bool z_os_pre_kernel(void)
{
	/* TX_INITIALIZE_IN_PROGRESS from the start of the program (its
	 * initial value), TX_INITIALIZE_ALMOST_DONE after
	 * tx_initialize_low_level(), 0 or the interrupt nesting once
	 * tx_application_define() has returned.
	 */
	return _tx_thread_system_state >= TX_INITIALIZE_IN_PROGRESS;
}

/* The thread's lock.  Masking interrupts alone is not enough on ThreadX: a
 * service that makes a more urgent thread ready switches to it at once
 * (_tx_thread_system_return(), which on Cortex-M pends PendSV and unmasks
 * interrupts for an instant to take it).  Each service checks
 * _tx_thread_preempt_disable before it switches
 * (TX_THREAD_SYSTEM_RETURN_CHECK), as ThreadX itself uses it around its own
 * critical sequences, so the lock raises it too; at the outermost unlock it
 * is lowered with interrupts still masked, and
 * _tx_thread_system_preempt_check() makes the switch the services left
 * (an interrupt between the unmask and the check may make it first; the
 * check then finds nothing to do).  A thread's lock never waits, so one
 * depth and one saved mask serve every thread.
 */
extern UINT z_tx_lock_depth;
extern UINT z_tx_lock_state;

Z_OS_INLINE void z_os_task_lock(void)
{
	UINT state = z_tx_irq_save();

	if (z_tx_lock_depth++ == 0) {
		z_tx_lock_state = state;
		_tx_thread_preempt_disable++;
	}
}

Z_OS_INLINE void z_os_task_unlock(void)
{
	if (--z_tx_lock_depth == 0) {
		_tx_thread_preempt_disable--;
		z_tx_irq_restore(z_tx_lock_state);
		_tx_thread_system_preempt_check();
	}
}

Z_OS_INLINE uint32_t z_os_isr_lock(void)
{
	return (uint32_t)z_tx_irq_save();
}

Z_OS_INLINE void z_os_isr_unlock(uint32_t state)
{
	z_tx_irq_restore((UINT)state);
}

Z_OS_INLINE void z_os_yield(void)
{
	tx_thread_relinquish();
}

/* Threads ------------------------------------------------------------------*/

Z_OS_INLINE z_os_thread_t z_os_thread_self(void)
{
	return _tx_thread_current_ptr;
}

/* tx_thread_priority is the priority the thread runs at, raised by a
 * mutex's priority inheritance (its own is tx_thread_user_priority).
 */
Z_OS_INLINE z_os_prio_t z_os_thread_prio_self(void)
{
	return Z_TX_PRIO_FROM_TX(_tx_thread_current_ptr->tx_thread_priority);
}

/* The entry of every thread of the layer: the function and its argument
 * are in the storage after the stack.  A thread whose function returns is
 * left COMPLETED by ThreadX (which releases its mutexes).
 */
static inline void z_tx_thread_entry(ULONG input)
{
	struct z_tx_thread *t = (struct z_tx_thread *)_tx_thread_current_ptr;

	(void)input;
	t->fn(t->arg);
}

/* The priority must be 1..TX_MAX_PRIORITIES - 1 and stack_size at least
 * TX_MINIMUM_STACK.  The control block, the wake flag and the name stay in
 * ThreadX's lists of created objects after the thread has ended
 * (TX_COMPLETED or TX_TERMINATED); a new start on the same stack deletes
 * them first, and one on the stack of a thread that has not ended is
 * refused (-EBUSY).  The thread is created unstarted and resumed once its
 * wake flag and *tid are set, so it can only run after that (at once if it
 * is more urgent than the caller).  From a thread or from
 * tx_application_define().
 */
static inline int z_os_thread_start(void (*fn)(void *arg), const char *name, void *stack,
				    size_t stack_size, int prio, void *arg, z_os_thread_t *tid)
{
	struct z_tx_thread *t;
	ULONG slice;

	if (prio < 1 || prio >= TX_MAX_PRIORITIES || stack == NULL ||
	    stack_size < TX_MINIMUM_STACK || stack_size % 8 != 0) {
		return -EINVAL;
	}
	t = (struct z_tx_thread *)((uint8_t *)stack + stack_size);

	if (t->thread.tx_thread_id == TX_THREAD_ID) {
		if (t->thread.tx_thread_state != TX_COMPLETED &&
		    t->thread.tx_thread_state != TX_TERMINATED) {
			return -EBUSY;
		}
		if (tx_thread_delete(&t->thread) != TX_SUCCESS) {
			return -EBUSY;
		}
	}
	if (t->wake.tx_semaphore_id == TX_SEMAPHORE_ID) {
		(void)tx_semaphore_delete(&t->wake);
	}

	t->fn = fn;
	t->arg = arg;
	if (name == NULL) {
		name = "";
	}
	strncpy(t->name, name, sizeof(t->name) - 1);
	t->name[sizeof(t->name) - 1] = '\0';
	if (tx_semaphore_create(&t->wake, t->name, 0) != TX_SUCCESS) {
		return -EINVAL;
	}

	slice = prio <= APP_TIMESLICE_PRIO_MAX ? APP_TIMESLICE_TICKS : TX_NO_TIME_SLICE;
	if (tx_thread_create(&t->thread, t->name, z_tx_thread_entry, 0, stack, stack_size,
			     Z_TX_PRIORITY(prio), Z_TX_PRIORITY(prio), slice,
			     TX_DONT_START) != TX_SUCCESS) {
		(void)tx_semaphore_delete(&t->wake);
		return -EINVAL;
	}
	*tid = &t->thread;
	(void)tx_thread_resume(&t->thread);
	return 0;
}

/* tx_thread_terminate() of the caller does not return.  Unlike FreeRTOS's
 * vTaskDelete(NULL) it leaves the thread in ThreadX's list of created
 * threads, TX_TERMINATED, with the mutexes it held released, until
 * z_os_thread_start() reuses its stack.
 */
Z_OS_INLINE void z_os_thread_exit(void)
{
	for (;;) {
		(void)tx_thread_terminate(_tx_thread_current_ptr);
	}
}

Z_OS_INLINE char *z_os_thread_name(z_os_thread_t tid)
{
	return tid->tx_thread_name;
}

Z_OS_INLINE void *z_os_thread_local_get(void)
{
	if (z_os_pre_kernel()) {
		return NULL;
	}
	return _tx_thread_current_ptr->tx_thread_zephyr_local;
}

Z_OS_INLINE void z_os_thread_local_set(void *value)
{
	_tx_thread_current_ptr->tx_thread_zephyr_local = value;
}

/* Waits and wakes ------------------------------------------------------------*/

/* The flag is the thread's semaphore, 0 or 1. */
#define Z_TX_WAKE(tid) (&((struct z_tx_thread *)(tid))->wake)

Z_OS_INLINE void z_os_wait_prepare(void)
{
	(void)tx_semaphore_get(Z_TX_WAKE(_tx_thread_current_ptr), TX_NO_WAIT);
}

Z_OS_INLINE void z_os_wait(z_os_ticks_t ticks)
{
	(void)tx_semaphore_get(Z_TX_WAKE(_tx_thread_current_ptr), ticks);
}

/* Inside the lock: the put readies a waiting tid but does not switch to it
 * (_tx_thread_preempt_disable), z_os_task_unlock() does.
 */
Z_OS_INLINE void z_os_wake(z_os_thread_t tid)
{
	(void)tx_semaphore_ceiling_put(Z_TX_WAKE(tid), 1);
}

/* ThreadX switches at the end of the interrupt by itself (PendSV on
 * Cortex-M, the context restore of the Linux port); *woken only reports
 * that tid is more urgent than the interrupted thread (or that none was
 * running).
 */
Z_OS_INLINE void z_os_wake_from_isr(z_os_thread_t tid, z_os_woken_t *woken)
{
	TX_THREAD *current = _tx_thread_current_ptr;

	(void)tx_semaphore_ceiling_put(Z_TX_WAKE(tid), 1);
	if (current == NULL || tid->tx_thread_priority < current->tx_thread_priority) {
		*woken = 1;
	}
}

Z_OS_INLINE void z_os_yield_from_isr(z_os_woken_t woken)
{
	(void)woken;
}

/* Time (z_os_busy_wait_us(), z_os_clock_us(): the port header) --------------*/

Z_OS_INLINE void z_os_sleep(z_os_ticks_t ticks)
{
	(void)tx_thread_sleep(ticks);
}

Z_OS_INLINE void z_os_suspend(void)
{
	(void)tx_thread_suspend(_tx_thread_current_ptr);
}

/* Mutexes --------------------------------------------------------------------*/

/* Priority inheritance is ThreadX's (TX_INHERIT): a waiter raises the
 * owner to its priority, one level (ThreadX does not follow a chain of
 * owners), and the owner returns to the highest priority of the waiters of
 * the mutexes it still holds, or its own, when it releases the mutex.
 * ThreadX leaves the owner raised when a waiter gives up (its timeout,
 * _tx_mutex_cleanup()) until that release; Zephyr lowers it then
 * (kernel/mutex.c), and so does z_tx_mutex_timed_out(), with the release's
 * own computation (_tx_mutex_put()) and ThreadX's change of an owner's
 * priority (_tx_mutex_priority_change()), inside the lock.
 */
static inline void z_tx_mutex_timed_out(TX_MUTEX *m)
{
	TX_THREAD *owner;
	TX_THREAD *waiter;
	TX_MUTEX *owned;
	UINT highest = TX_MAX_PRIORITIES;
	UINT inherit = TX_MAX_PRIORITIES;
	UINT prio;

	z_os_task_lock();
	owner = m->tx_mutex_owner;
	if (m->tx_mutex_inherit == TX_TRUE && m->tx_mutex_ownership_count != 0 &&
	    owner != TX_NULL) {
		waiter = m->tx_mutex_suspension_list;
		for (ULONG i = 0; i < m->tx_mutex_suspended_count; i++) {
			if (waiter->tx_thread_priority < highest) {
				highest = waiter->tx_thread_priority;
			}
			waiter = waiter->tx_thread_suspended_next;
		}
		m->tx_mutex_highest_priority_waiting = highest;

		owned = owner->tx_thread_owned_mutex_list;
		for (UINT i = 0; i < owner->tx_thread_owned_mutex_count; i++) {
			if (owned->tx_mutex_inherit == TX_TRUE &&
			    owned->tx_mutex_highest_priority_waiting < inherit) {
				inherit = owned->tx_mutex_highest_priority_waiting;
			}
			owned = owned->tx_mutex_owned_next;
		}
		owner->tx_thread_inherit_priority = inherit;
		prio = inherit < owner->tx_thread_user_priority ? inherit
								: owner->tx_thread_user_priority;
		if (owner->tx_thread_priority != prio) {
			_tx_mutex_priority_change(owner, prio);
		}
	}
	z_os_task_unlock();
}

/* Zephyr's k_mutex holds no reference while it is unlocked, and the
 * application relies on that: mqtt_client_init() (common/imu/mqtt_min.c)
 * zeroes the client with the mutex in it at every connection, and
 * k_mutex_init()s it again.  ThreadX links every created mutex into its
 * list of created mutexes (_tx_mutex_created_ptr, for tx_mutex_info_get()
 * and the duplicate check of tx_mutex_create()), which zeroing a created
 * mutex would break.  So once created, a k_mutex's ThreadX mutex is taken
 * out of that list again: ThreadX then refers to it only while a thread
 * owns it (the owner's list of owned mutexes) or waits on it, as Zephyr
 * does, and an unlocked one left all zero is again a mutex to be created.
 * It is never deleted (tx_mutex_delete() would unlink it from that list).
 * Inside the lock: two first users cannot both create it.
 */
static inline void z_tx_mutex_create(struct k_mutex *mutex)
{
	TX_MUTEX *m = &mutex->mutex;

	z_os_task_lock();
	if (m->tx_mutex_id != TX_MUTEX_ID &&
	    tx_mutex_create(m, (CHAR *)"k_mutex", TX_INHERIT) == TX_SUCCESS) {
		if (_tx_mutex_created_ptr == m && m->tx_mutex_created_next == m) {
			_tx_mutex_created_ptr = TX_NULL;
		} else {
			m->tx_mutex_created_next->tx_mutex_created_previous =
				m->tx_mutex_created_previous;
			m->tx_mutex_created_previous->tx_mutex_created_next =
				m->tx_mutex_created_next;
			if (_tx_mutex_created_ptr == m) {
				_tx_mutex_created_ptr = m->tx_mutex_created_next;
			}
		}
		_tx_mutex_created_count--;
		m->tx_mutex_created_next = TX_NULL;
		m->tx_mutex_created_previous = TX_NULL;
	}
	z_os_task_unlock();
}

Z_OS_INLINE TX_MUTEX *z_tx_mutex_get(struct k_mutex *mutex)
{
	if (*(ULONG volatile *)&mutex->mutex.tx_mutex_id != TX_MUTEX_ID) {
		z_tx_mutex_create(mutex);
	}
	return &mutex->mutex;
}

/* An unlocked mutex is left as it is (it is one already); one that a thread
 * holds or waits on is not initialized again.
 */
static inline int z_os_mutex_init(struct k_mutex *mutex)
{
	TX_MUTEX *m = &mutex->mutex;

	if (m->tx_mutex_id == TX_MUTEX_ID) {
		if (m->tx_mutex_ownership_count != 0 || m->tx_mutex_suspended_count != 0) {
			return -EINVAL;
		}
		return 0;
	}
	z_tx_mutex_create(mutex);
	return m->tx_mutex_id == TX_MUTEX_ID ? 0 : -EINVAL;
}

Z_OS_INLINE bool z_os_mutex_lock(struct k_mutex *mutex, z_os_ticks_t ticks)
{
	TX_MUTEX *m = z_tx_mutex_get(mutex);

	if (tx_mutex_get(m, ticks) == TX_SUCCESS) {
		return true;
	}
	if (ticks != TX_NO_WAIT) {
		z_tx_mutex_timed_out(m);
	}
	return false;
}

/* A mutex never created is held by nobody.  The owner is read without the
 * lock: only the calling thread can make it the caller or stop it being
 * the caller.
 */
static inline int z_os_mutex_unlock(struct k_mutex *mutex)
{
	TX_MUTEX *m = &mutex->mutex;

	if (m->tx_mutex_id != TX_MUTEX_ID || m->tx_mutex_ownership_count == 0) {
		return -EINVAL;
	}
	if (m->tx_mutex_owner != _tx_thread_current_ptr) {
		return -EPERM;
	}
	return tx_mutex_put(m) == TX_SUCCESS ? 0 : -EINVAL;
}

#endif /* COMMON_COMPAT_PORT_THREADX_ZEPHYR_KERNEL_OS_IMPL_H */
