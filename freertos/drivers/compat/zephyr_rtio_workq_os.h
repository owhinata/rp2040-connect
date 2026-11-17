/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the RTIO work queue (common/compat/zephyr_rtio_workq.c,
 * which includes this file at its end): the thread "RTIO WQ 0", a task on a
 * static stack (NANO_PRIO_RTIO_WQ, NANO_STACK_RTIO_WQ), started by
 * rtio_workq_initialize() (zephyr_compat_initialize()).  Part of the FreeRTOS
 * implementation of the Zephyr compatibility layer (freertos/drivers/compat;
 * NuttX's is nuttx/drivers/compat/zephyr_rtio_workq_os.h).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_RTIO_WORKQ_OS_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_RTIO_WORKQ_OS_H

#include "FreeRTOS.h"
#include "task.h"

#include "priorities.h"
#include "zephyr_os_internal.h"

/* rtio_workq.c rtio_workq_thread_fn(): k_queue_get(.., K_FOREVER). */
static void rtio_workq_thread(void *arg)
{
	(void)arg;

	for (;;) {
		(void)k_sem_take(&rtio_workq_sem, K_FOREVER);
		(void)rtio_workq_process();
	}
}

static K_THREAD_STACK_DEFINE(rtio_workq_stack, NANO_STACK_RTIO_WQ);
static TaskHandle_t rtio_workq_task;

void rtio_workq_initialize(void)
{
	int ret = z_os_task_start(rtio_workq_thread, "RTIO WQ 0", rtio_workq_stack,
				  K_THREAD_STACK_SIZEOF(rtio_workq_stack), NANO_PRIO_RTIO_WQ, NULL,
				  &rtio_workq_task);

	configASSERT(ret == 0);
	(void)ret;
}

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_RTIO_WORKQ_OS_H */
