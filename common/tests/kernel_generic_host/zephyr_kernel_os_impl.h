/* SPDX-License-Identifier: MIT */
/* The smallest OS part of the generic kernel part of the Zephyr
 * compatibility layer (common/compat/kernel_generic/zephyr_kernel_os.h) for
 * the host checks of common/tests/run.sh: one thread, no interrupts, the
 * host's monotonic clock.  It only has to compile: run.sh compiles
 * <zephyr/kernel.h> and the sources of the generic kernel part on it with
 * nothing of the layer's network part on the include path
 * (kernel_generic_independent).  The OS part a port writes is
 * freertos/drivers/compat/zephyr_kernel_os_impl.h.
 */

#ifndef COMMON_TESTS_KERNEL_GENERIC_HOST_ZEPHYR_KERNEL_OS_IMPL_H
#define COMMON_TESTS_KERNEL_GENERIC_HOST_ZEPHYR_KERNEL_OS_IMPL_H

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

typedef void *z_os_thread_t; /* NULL is no thread (zephyr_kernel.c) */
typedef int z_os_prio_t;
typedef uint32_t z_os_ticks_t;
typedef int z_os_woken_t;

#define Z_OS_TICKS_FOREVER    UINT32_MAX
#define Z_OS_TICK_HZ          1000
#define Z_OS_THREAD_NAME_SIZE 32
#define Z_OS_THREAD_RESERVED  0 /* no thread starts here */

struct k_mutex {
	z_os_thread_t holder;
	unsigned int depth;
};

#define Z_MUTEX_INITIALIZER(obj)                                                                   \
	{                                                                                          \
		.holder = NULL, .depth = 0                                                         \
	}

#define Z_OS_ASSERT(test) assert(test)

static char z_host_name[Z_OS_THREAD_NAME_SIZE];
static char z_host_thread;
static void *z_host_local;
static bool z_host_woken;

static inline bool z_os_in_isr(void)
{
	return false;
}

static inline bool z_os_pre_kernel(void)
{
	return false;
}

static inline void z_os_task_lock(void)
{
}

static inline void z_os_task_unlock(void)
{
}

static inline uint32_t z_os_isr_lock(void)
{
	return 0;
}

static inline void z_os_isr_unlock(uint32_t state)
{
	(void)state;
}

static inline z_os_thread_t z_os_thread_self(void)
{
	return &z_host_thread;
}

static inline z_os_prio_t z_os_thread_prio_self(void)
{
	return 0;
}

static inline int z_os_thread_start(void (*fn)(void *arg), const char *name, void *stack,
				    size_t stack_size, int prio, void *arg, z_os_thread_t *tid)
{
	(void)fn;
	(void)name;
	(void)stack;
	(void)stack_size;
	(void)prio;
	(void)arg;
	*tid = NULL;
	return -ENOMEM;
}

static inline void z_os_thread_exit(void)
{
}

static inline char *z_os_thread_name(z_os_thread_t tid)
{
	(void)tid;
	return z_host_name;
}

static inline void z_os_yield(void)
{
}

static inline void *z_os_thread_local_get(void)
{
	return z_host_local;
}

static inline void z_os_thread_local_set(void *value)
{
	z_host_local = value;
}

static inline void z_os_wait_prepare(void)
{
	z_host_woken = false;
}

static inline void z_os_wait(z_os_ticks_t ticks)
{
	(void)ticks;
	z_host_woken = false;
}

static inline void z_os_wake(z_os_thread_t tid)
{
	(void)tid;
	z_host_woken = true;
}

static inline void z_os_wake_from_isr(z_os_thread_t tid, z_os_woken_t *woken)
{
	z_os_wake(tid);
	*woken = 1;
}

static inline void z_os_yield_from_isr(z_os_woken_t woken)
{
	(void)woken;
}

static inline void z_os_sleep(z_os_ticks_t ticks)
{
	(void)ticks;
}

static inline void z_os_suspend(void)
{
}

static inline uint64_t z_os_clock_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

static inline void z_os_busy_wait_us(uint32_t us)
{
	uint64_t end = z_os_clock_us() + us;

	while (z_os_clock_us() < end) {
	}
}

static inline int z_os_mutex_init(struct k_mutex *mutex)
{
	mutex->holder = NULL;
	mutex->depth = 0;
	return 0;
}

static inline bool z_os_mutex_lock(struct k_mutex *mutex, z_os_ticks_t ticks)
{
	(void)ticks;
	if (mutex->depth && mutex->holder != z_os_thread_self()) {
		return false;
	}
	mutex->holder = z_os_thread_self();
	mutex->depth++;
	return true;
}

static inline int z_os_mutex_unlock(struct k_mutex *mutex)
{
	if (!mutex->depth) {
		return -EINVAL;
	}
	if (mutex->holder != z_os_thread_self()) {
		return -EPERM;
	}
	mutex->depth--;
	return 0;
}

#endif /* COMMON_TESTS_KERNEL_GENERIC_HOST_ZEPHYR_KERNEL_OS_IMPL_H */
