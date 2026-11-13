/* SPDX-License-Identifier: Apache-2.0 */
/* The OS objects of the Zephyr compatibility layer (nuttx/drivers/compat,
 * <zephyr/spinlock.h> and <zephyr/kernel.h>) for host tests: one thread,
 * no interrupts.  A spin lock checks that it is not taken twice; the
 * semaphores are counts; a timer only records that it is armed; the time
 * (k_uptime_get(), k_sleep()) is a counter of the test.  The
 * functions are in zephyr_kernel_host.c.
 */
#ifndef TEST_ZEPHYR_OS_HOST_H
#define TEST_ZEPHYR_OS_HOST_H
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct k_spinlock { int locked; };
typedef int k_spinlock_key_t;

static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *l)
{
	assert(!l->locked);
	l->locked = 1;
	return 7;
}

static inline void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t key)
{
	assert(l->locked && key == 7);
	l->locked = 0;
}

struct k_mutex { int depth; };
#define Z_MUTEX_INITIALIZER(obj) { .depth = 0 }

struct k_poll_event;
struct k_sem {
	unsigned int count;
	unsigned int limit;
	struct k_poll_event *poll_events;
};
#define Z_SEM_INITIALIZER(obj, initial, lim) { .count = (initial), .limit = (lim) }
#define K_SEM_MAX_LIMIT 32767U

struct _timeout;
typedef void (*_timeout_func_t)(struct _timeout *t);
struct _timeout {
	bool armed;
	int64_t ticks;
	_timeout_func_t fn;
};

typedef int k_tid_t;
#endif
