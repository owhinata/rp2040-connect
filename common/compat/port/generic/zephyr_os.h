/* SPDX-License-Identifier: MIT */
/* The OS part of <zephyr/kernel.h> of the Zephyr compatibility layer
 * (common/compat/include/zephyr/kernel.h, which includes this header first and lists
 * what it must provide) for an OS that offers only the primitives of
 * zephyr_kernel_os.h: the tick, the objects (k_sem, the timer of a delayable
 * work item, k_tid_t; k_mutex is the OS's), the priorities, the calling
 * thread, k_yield() and the cycle counter.  The functions on them are
 * zephyr_kernel.c and zephyr_mutex.c.  Includes no OS header itself.
 */

#ifndef COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_H
#define COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_H

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h> /* for <zephyr/sys/printk.h> */

/* The OS's headers first (they must come before the layer's atomic names),
 * with struct k_mutex, Z_MUTEX_INITIALIZER() and Z_OS_ASSERT().
 */
#include "zephyr_kernel_os.h"
#include "priorities.h"

#include <zephyr/sys/clock.h> /* MSEC_PER_SEC, ..., as Zephyr's <zephyr/kernel.h> */

/* Timeouts are kernel ticks, as in Zephyr: the OS's tick (1 ms in the
 * FreeRTOS and ThreadX builds; CONFIG_SYS_CLOCK_TICKS_PER_SEC of the Zephyr
 * build is 10000).  K_MSEC(), K_SECONDS() and k_usleep() convert with
 * Z_USEC_PER_TICK, k_uptime_get() and the cycle counter are
 * z_os_clock_us() whatever the tick, and 1000 must be a multiple of
 * Z_USEC_PER_TICK (<zephyr/kernel.h>).  A relative wait ends at the current
 * tick + ticks + 1 in Zephyr (z_add_timeout()); the OS's waits end at the
 * current tick + n, so the layer waits ticks + 1 (zephyr_kernel.c).
 */
#define Z_USEC_PER_TICK (1000000 / Z_OS_TICK_HZ)

#if (1000000 % Z_OS_TICK_HZ) != 0
#error "the tick must be a whole number of microseconds"
#endif

/* The objects ----------------------------------------------------------------*/

struct k_poll_event;

/* k_sem: Zephyr's semaphore itself (kernel/sem.c) rather than one of the
 * OS, so that it has a static initializer and Zephyr's rules: a count
 * with an upper limit; the threads waiting in k_sem_take() by priority
 * (first come first served within one), woken through their wake flag
 * (z_os_wake()); the k_poll() events on it by the priority of their
 * thread.  The fields belong to zephyr_kernel.c.
 */
struct z_sem_waiter;

struct k_sem {
	unsigned int count;
	unsigned int limit;
	struct z_sem_waiter *waiters;
	struct k_poll_event *poll_events;
};

#define Z_SEM_INITIALIZER(obj, initial, lim)                                                       \
	{                                                                                          \
		.count = (initial), .limit = (lim), .waiters = NULL, .poll_events = NULL           \
	}
#define K_SEM_MAX_LIMIT UINT_MAX

/* The timer of a delayable work item (Zephyr's struct _timeout): an entry in
 * the layer's list of timeouts, which the kernel tick's hook runs
 * (zephyr_compat_tick()), in the tick interrupt, as Zephyr runs its
 * timeouts in the system clock interrupt.
 */
struct _timeout;
typedef void (*_timeout_func_t)(struct _timeout *t);

struct _timeout {
	struct _timeout *next;
	_timeout_func_t fn;
	uint32_t deadline; /* the tick of zephyr_compat_tick() that runs fn */
	bool linked;
};

/* A thread: the OS's handle, which is what K_THREAD_DEFINE() stores in its
 * k_tid_t (Zephyr: the thread's struct k_thread, a constant; here the
 * variable is set when the thread starts, as on NuttX).
 */
typedef z_os_thread_t k_tid_t;

/* The bytes K_THREAD_STACK_DEFINE() adds after each stack for the OS's
 * control block of the thread (<zephyr/kernel.h>; z_os_thread_start()).
 */
#define K_THREAD_STACK_RESERVED Z_OS_THREAD_RESERVED

/* K_PRIO_PREEMPT(p): the OS's priority of Zephyr's p (APP_PRIO_FROM_ZEPHYR,
 * priorities_os.h; 20 - p in the FreeRTOS build, 15 - p in the ThreadX one).
 */
#define K_PRIO_PREEMPT(x) APP_PRIO_FROM_ZEPHYR(x)

static inline k_tid_t k_current_get(void)
{
	return z_os_thread_self();
}

/* Zephyr's k_yield(): gives the turn to the next ready thread of the same
 * priority, or goes on if there is none.  Not from an interrupt handler.
 */
static inline void k_yield(void)
{
	z_os_yield();
}

/* Zephyr's k_is_in_isr() and k_is_pre_kernel(): in an interrupt handler;
 * before the kernel runs threads (Zephyr: before the main thread starts).
 */
static inline bool k_is_in_isr(void)
{
	return z_os_in_isr();
}

static inline bool k_is_pre_kernel(void)
{
	return z_os_pre_kernel();
}

/* Zephyr's custom data of a thread (CONFIG_THREAD_CUSTOM_DATA): one
 * pointer per thread, set and read by the thread itself; NULL until set,
 * and before the kernel runs threads.  Not from an interrupt handler.
 */
static inline void k_thread_custom_data_set(void *value)
{
	z_os_thread_local_set(value);
}

static inline void *k_thread_custom_data_get(void)
{
	return z_os_thread_local_get();
}

/* The cycle counter: the board's monotonic clock in microseconds (the RP2040
 * TIMER, 1 MHz from the crystal; see <zephyr/kernel.h>).
 */
static inline uint64_t k_cycle_get_64(void)
{
	return z_os_clock_us();
}

static inline uint32_t k_cycle_get_32(void)
{
	return (uint32_t)z_os_clock_us();
}

#endif /* COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_H */
