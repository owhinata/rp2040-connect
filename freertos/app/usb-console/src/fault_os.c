/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the fatal error exit (fault.h of
 * common/imu/port/pico_sdk): the running task's name from its
 * control block, and the kernel's stack overflow check as an entry.
 * configASSERT() (FreeRTOSConfig.h) calls fault.c's app_assert_failed(), and
 * a task function that returns ends in the port's prvTaskExitError(), which
 * calls panic().
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "fault.h"

/* The handle is the task's control block; a fault may come from a corrupted
 * kernel, so only read it inside the RAM.
 */
void fault_os_thread_name(char *name, size_t size)
{
	TaskHandle_t task;

	if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
		return;
	}
	task = xTaskGetCurrentTaskHandle();
	if (task != NULL && fault_record_in_range((uintptr_t)task, sizeof(StaticTask_t),
						  FAULT_RAM_START, FAULT_RAM_END)) {
		strncpy(name, pcTaskGetName(task), size - 1);
	}
}

/* configCHECK_FOR_STACK_OVERFLOW 2: the kernel checks the stack of the task
 * it switches out (in PendSV, exception 14) for its limit and the pattern at
 * its end.
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
	struct fault_record *r =
		fault_begin(FAULT_REASON_STACK_OVERFLOW, (uint32_t)__builtin_return_address(0));

	(void)task;
	if (r != NULL && name != NULL) {
		strncpy(r->task, name, sizeof(r->task) - 1);
	}
	fault_finish(r);
}
