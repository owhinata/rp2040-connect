/* SPDX-License-Identifier: MIT */
/* The self-test's OS boundary (listed in common/compat/selftest/
 * zephyr_selftest.h) for an OS on the primitives of zephyr_kernel_os.h: the
 * items those give are here; the OS's part, "zephyr_selftest_os_impl.h",
 * included below, defines the others (zephyr_selftest_os_tick(),
 * _thread_create(), _prio_get(), _current(), _sched_lock(),
 * _sched_unlock(); the FreeRTOS build's is
 * freertos/drivers/compat/zephyr_selftest_os_impl.h).  An OS that implements
 * the kernel part directly writes all of them in its own
 * zephyr_selftest_os.h instead.
 */

#ifndef COMMON_COMPAT_KERNEL_GENERIC_ZEPHYR_SELFTEST_OS_H
#define COMMON_COMPAT_KERNEL_GENERIC_ZEPHYR_SELFTEST_OS_H

#include <stdbool.h>

#include <zephyr/kernel.h>

#include "zephyr_kernel_os.h"

typedef z_os_prio_t zephyr_selftest_os_prio_t;

#define ZEPHYR_SELFTEST_OS_NAME_MAX (Z_OS_THREAD_NAME_SIZE - 1)

#include "zephyr_selftest_os_impl.h"

static inline void zephyr_selftest_os_tick_sync(void)
{
	z_os_sleep(1);
}

static inline void zephyr_selftest_os_thread_exit(void)
{
	z_os_thread_exit();
}

static inline zephyr_selftest_os_prio_t zephyr_selftest_os_prio_self(void)
{
	return z_os_thread_prio_self();
}

static inline const char *zephyr_selftest_os_name_get(k_tid_t tid)
{
	return z_os_thread_name(tid);
}

static inline bool zephyr_selftest_os_in_isr(void)
{
	return z_os_in_isr();
}

#endif /* COMMON_COMPAT_KERNEL_GENERIC_ZEPHYR_SELFTEST_OS_H */
