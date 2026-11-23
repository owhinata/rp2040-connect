/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the test's OS boundary
 * (common/tests/zephyr_kernel/kernel_generic/zephyr_kernel_test_os.h,
 * which includes this header and defines the items that the primitives of
 * zephyr_kernel_os.h give): the helper threads are dynamic FreeRTOS tasks,
 * the tick count is xTaskGetTickCount(), the priority of another task is
 * uxTaskPriorityGet(), the calling task is the OS's own
 * xTaskGetCurrentTaskHandle() (to compare with k_current_get()), and the
 * scheduler lock is vTaskSuspendAll().  The host's FreeRTOS POSIX port's
 * own parts are in zephyr_os_port.h and the zephyr_kernel_test_port_*()
 * functions of main.c.
 */

#ifndef FREERTOS_TESTS_COMPAT_ZEPHYR_KERNEL_TEST_OS_IMPL_H
#define FREERTOS_TESTS_COMPAT_ZEPHYR_KERNEL_TEST_OS_IMPL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/kernel.h>

static inline uint32_t zephyr_kernel_test_os_tick(void)
{
	return (uint32_t)xTaskGetTickCount();
}

static inline bool zephyr_kernel_test_os_thread_create(void (*fn)(void *), const char *name,
						       size_t stack_size, void *arg, int prio,
						       k_tid_t *tid)
{
	return xTaskCreate(fn, name, stack_size / sizeof(StackType_t), arg, prio, tid) == pdPASS;
}

static inline zephyr_kernel_test_os_prio_t zephyr_kernel_test_os_prio_get(k_tid_t tid)
{
	return uxTaskPriorityGet(tid);
}

static inline k_tid_t zephyr_kernel_test_os_current(void)
{
	return xTaskGetCurrentTaskHandle();
}

static inline void zephyr_kernel_test_os_sched_lock(void)
{
	vTaskSuspendAll();
}

static inline void zephyr_kernel_test_os_sched_unlock(void)
{
	(void)xTaskResumeAll();
}

#endif /* FREERTOS_TESTS_COMPAT_ZEPHYR_KERNEL_TEST_OS_IMPL_H */
