/* SPDX-License-Identifier: MIT */
/* The ThreadX part of the test's OS boundary
 * (common/tests/zephyr_kernel/kernel_generic/zephyr_kernel_test_os.h,
 * which includes this header and defines the items that the primitives of
 * zephyr_kernel_os.h give): the helper threads start with the layer's
 * z_os_thread_start() on the stacks of a pool here (a thread that waits
 * through the layer needs the wake flag that z_os_thread_start() lays out
 * after its stack; ThreadX allocates nothing), the tick count is
 * tx_time_get(), the priority of another thread is its tx_thread_priority
 * in the layer's order, the calling thread is tx_thread_identify() (to
 * compare with k_current_get()), and the scheduler lock is the layer's
 * thread lock, z_os_task_lock(), under which a thread started does not run
 * before the unlock.  The Linux port's own parts are in zephyr_os_port.h
 * and the zephyr_kernel_test_port_*() functions of main.c.
 */

#ifndef THREADX_TESTS_COMPAT_ZEPHYR_KERNEL_TEST_OS_IMPL_H
#define THREADX_TESTS_COMPAT_ZEPHYR_KERNEL_TEST_OS_IMPL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "tx_api.h"

#include <zephyr/kernel.h>

/* At most 3 helpers run at once; a slot is taken again once its thread
 * has ended (z_os_thread_start() deletes the ended one).
 */
#define TEST_HELPER_SLOTS 8
#define TEST_HELPER_STACK 2048

static K_THREAD_STACK_DEFINE(test_helper_stacks[TEST_HELPER_SLOTS], TEST_HELPER_STACK);

static inline uint32_t zephyr_kernel_test_os_tick(void)
{
	return (uint32_t)tx_time_get();
}

static inline bool zephyr_kernel_test_os_thread_create(void (*fn)(void *), const char *name,
						       size_t stack_size, void *arg, int prio,
						       k_tid_t *tid)
{
	if (stack_size > TEST_HELPER_STACK) {
		return false;
	}
	for (int i = 0; i < TEST_HELPER_SLOTS; i++) {
		if (z_os_thread_start(fn, name, test_helper_stacks[i], TEST_HELPER_STACK, prio, arg,
				      tid) == 0) {
			return true;
		}
	}
	return false;
}

static inline zephyr_kernel_test_os_prio_t zephyr_kernel_test_os_prio_get(k_tid_t tid)
{
	return Z_TX_PRIO_FROM_TX(tid->tx_thread_priority);
}

static inline k_tid_t zephyr_kernel_test_os_current(void)
{
	return tx_thread_identify();
}

static inline void zephyr_kernel_test_os_sched_lock(void)
{
	z_os_task_lock();
}

static inline void zephyr_kernel_test_os_sched_unlock(void)
{
	z_os_task_unlock();
}

#endif /* THREADX_TESTS_COMPAT_ZEPHYR_KERNEL_TEST_OS_IMPL_H */
