/* SPDX-License-Identifier: MIT */
/* The test's OS boundary (listed in common/tests/zephyr_kernel/
 * zephyr_kernel_test.h) for an OS on the primitives of zephyr_kernel_os.h
 * (the generic kernel part, common/compat/kernel_generic): the items those
 * give are here; the OS's part, "zephyr_kernel_test_os_impl.h", included
 * below, defines the others (zephyr_kernel_test_os_tick(), _thread_create(),
 * _prio_get(), _current(), _sched_lock(), _sched_unlock(); the FreeRTOS
 * build's is freertos/tests/compat/zephyr_kernel_test_os_impl.h).  An OS
 * that implements the kernel part directly writes all of them in its own
 * zephyr_kernel_test_os.h instead.
 */

#ifndef COMMON_TESTS_ZEPHYR_KERNEL_KERNEL_GENERIC_ZEPHYR_KERNEL_TEST_OS_H
#define COMMON_TESTS_ZEPHYR_KERNEL_KERNEL_GENERIC_ZEPHYR_KERNEL_TEST_OS_H

#include <stdbool.h>

#include <zephyr/kernel.h>

#include "zephyr_kernel_os.h"

typedef z_os_prio_t zephyr_kernel_test_os_prio_t;

#define ZEPHYR_KERNEL_TEST_OS_NAME_MAX (Z_OS_THREAD_NAME_SIZE - 1)

#include "zephyr_kernel_test_os_impl.h"

static inline void zephyr_kernel_test_os_tick_sync(void)
{
	z_os_sleep(1);
}

static inline void zephyr_kernel_test_os_thread_exit(void)
{
	z_os_thread_exit();
}

static inline zephyr_kernel_test_os_prio_t zephyr_kernel_test_os_prio_self(void)
{
	return z_os_thread_prio_self();
}

static inline const char *zephyr_kernel_test_os_name_get(k_tid_t tid)
{
	return z_os_thread_name(tid);
}

static inline bool zephyr_kernel_test_os_in_isr(void)
{
	return z_os_in_isr();
}

#endif /* COMMON_TESTS_ZEPHYR_KERNEL_KERNEL_GENERIC_ZEPHYR_KERNEL_TEST_OS_H */
