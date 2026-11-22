/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the layer's self-test (the boundary is listed in
 * common/compat/selftest/zephyr_selftest.h): the helper threads are FreeRTOS
 * tasks, the tick count is xTaskGetTickCount(), the priorities and the names
 * are the tasks', and the scheduler lock is vTaskSuspendAll().  The same on
 * the board and on the host's FreeRTOS POSIX port; what differs between them
 * is in zephyr_os_port.h (z_os_in_isr()) and the zephyr_selftest_port_*()
 * functions.
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_SELFTEST_OS_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_SELFTEST_OS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/kernel.h>

typedef UBaseType_t zephyr_selftest_os_prio_t;

#define ZEPHYR_SELFTEST_OS_NAME_MAX (configMAX_TASK_NAME_LEN - 1)

static inline uint32_t zephyr_selftest_os_tick(void)
{
	return (uint32_t)xTaskGetTickCount();
}

static inline void zephyr_selftest_os_tick_sync(void)
{
	vTaskDelay(1);
}

static inline bool zephyr_selftest_os_thread_create(void (*fn)(void *), const char *name,
						    size_t stack_size, void *arg, int prio,
						    k_tid_t *tid)
{
	return xTaskCreate(fn, name, stack_size / sizeof(StackType_t), arg, prio, tid) == pdPASS;
}

static inline void zephyr_selftest_os_thread_exit(void)
{
	vTaskDelete(NULL);
}

static inline zephyr_selftest_os_prio_t zephyr_selftest_os_prio_get(k_tid_t tid)
{
	return uxTaskPriorityGet(tid);
}

static inline zephyr_selftest_os_prio_t zephyr_selftest_os_prio_self(void)
{
	return uxTaskPriorityGet(NULL);
}

static inline const char *zephyr_selftest_os_name_get(k_tid_t tid)
{
	return pcTaskGetName(tid);
}

static inline k_tid_t zephyr_selftest_os_current(void)
{
	return xTaskGetCurrentTaskHandle();
}

static inline bool zephyr_selftest_os_in_isr(void)
{
	return z_os_in_isr();
}

static inline void zephyr_selftest_os_sched_lock(void)
{
	vTaskSuspendAll();
}

static inline void zephyr_selftest_os_sched_unlock(void)
{
	(void)xTaskResumeAll();
}

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_SELFTEST_OS_H */
