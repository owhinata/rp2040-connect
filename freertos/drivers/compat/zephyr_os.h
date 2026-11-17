/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of <zephyr/kernel.h> of the Zephyr compatibility layer
 * (common/compat/zephyr/kernel.h, which includes this header first and lists
 * what it must provide): the FreeRTOS headers, the tick, the OS objects
 * (k_mutex, k_sem, the timer of a delayable work item, k_tid_t), the
 * priorities, the calling thread and the cycle counter.  The functions on
 * them are zephyr_kernel.c and zephyr_mutex.c.  Part of the FreeRTOS
 * implementation of the layer (freertos/drivers/compat, the counterpart of
 * nuttx/drivers/compat; see freertos/drivers/README.md).
 *
 * What the application provides: FreeRTOSConfig.h (the tick, two task
 * notifications per task, mutexes and recursive mutexes, static and dynamic
 * allocation, configUSE_TICK_HOOK with a hook that calls
 * zephyr_compat_tick() of "zephyr_compat_os.h"), priorities_os.h
 * (common/compat/priorities.h) and the port header "zephyr_os_port.h"
 * (freertos/drivers/compat/rp2040 for the board).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_H

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h> /* for <zephyr/sys/printk.h> */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "zephyr_os_port.h"
#include "priorities.h"

/* Timeouts are kernel ticks, as in Zephyr: the FreeRTOS tick
 * (configTICK_RATE_HZ 10000 on the board, 100 us, the Zephyr build's
 * CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000).  A relative wait ends at the
 * current tick + ticks + 1 in Zephyr (z_add_timeout()); FreeRTOS's block
 * times end at the current tick + n, so the layer waits ticks + 1
 * (zephyr_kernel.c).
 */
#define Z_USEC_PER_TICK (1000000 / configTICK_RATE_HZ)

#if (1000000 % configTICK_RATE_HZ) != 0
#error "the tick must be a whole number of microseconds"
#endif

#if configTASK_NOTIFICATION_ARRAY_ENTRIES < 2
#error "the compatibility layer waits on task notification index 1"
#endif

/* The task notification that the layer's waits use (k_sem_take(), k_poll());
 * index 0 is left to the Pico SDK and the application.
 */
#define Z_NOTIFY_INDEX 1

/* The OS-specific objects -------------------------------------------------*/

struct k_poll_event;

/* k_mutex: a FreeRTOS recursive mutex (recursive for its owner, with
 * FreeRTOS's priority inheritance) in the structure's own memory.  FreeRTOS
 * has no static initializer for one, so a mutex is created by its first
 * k_mutex_lock() or k_mutex_unlock() when handle is NULL (zephyr_mutex.c):
 * K_MUTEX_DEFINE() and a k_mutex left all zero (never initialized) both
 * work as an unlocked mutex, as Zephyr's do (lock_count 0).
 */
struct k_mutex {
	StaticSemaphore_t buffer;
	SemaphoreHandle_t handle;
};

#define Z_MUTEX_INITIALIZER(obj)                                                                   \
	{                                                                                          \
		.handle = NULL                                                                     \
	}

/* k_sem: Zephyr's semaphore itself (kernel/sem.c) rather than a FreeRTOS
 * one, so that it has a static initializer and Zephyr's rules: a count
 * with an upper limit; the threads waiting in k_sem_take() by priority
 * (first come first served within one), woken through their task
 * notification Z_NOTIFY_INDEX; the k_poll() events on it by the priority of
 * their thread.  The fields belong to zephyr_kernel.c.
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

/* A thread: its FreeRTOS task handle, which is what K_THREAD_DEFINE()
 * stores in its k_tid_t (Zephyr: the thread's struct k_thread, a constant;
 * here the variable is set when the thread starts, as on NuttX).
 */
typedef TaskHandle_t k_tid_t;

/* Priorities: K_PRIO_PREEMPT(p) is the FreeRTOS priority 20 - p
 * (NANO_PRIO_FROM_ZEPHYR, priorities_os.h).
 */
#define K_PRIO_PREEMPT(x) NANO_PRIO_FROM_ZEPHYR(x)

static inline k_tid_t k_current_get(void)
{
	return xTaskGetCurrentTaskHandle();
}

/* The cycle counter: the board's monotonic clock, the RP2040 TIMER
 * (nano_mono_us(), 1 MHz from the crystal; see <zephyr/kernel.h>).
 */
static inline uint64_t k_cycle_get_64(void)
{
	return nano_mono_us();
}

static inline uint32_t k_cycle_get_32(void)
{
	return (uint32_t)nano_mono_us();
}

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_H */
