/* SPDX-License-Identifier: MIT */
/* What the files of the FreeRTOS part of the Zephyr compatibility layer share
 * (zephyr_kernel.c, zephyr_mutex.c, zephyr_rtio_workq_os.h); not for the
 * layer's users.  Include after <zephyr/kernel.h>.  Part of the FreeRTOS
 * implementation of the layer (freertos/drivers/compat).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_INTERNAL_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_INTERNAL_H

#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

/* The FreeRTOS block time of a Zephyr timeout: K_NO_WAIT (and any count of 0
 * or less) 0, K_FOREVER portMAX_DELAY (for ever, INCLUDE_vTaskSuspend), else
 * ticks + 1, Zephyr's end of a relative wait (z_add_timeout(): the current
 * tick + ticks + 1; FreeRTOS ends a block time of n at the current tick + n),
 * at most portMAX_DELAY - 1 (about 49.7 days with the board's 32-bit tick of
 * 1 ms).
 */
TickType_t z_os_wait_ticks(k_timeout_t timeout);

/* Starts fn(arg) in a new task on the static stack (stack_size bytes) at the
 * FreeRTOS priority prio (1..configMAX_PRIORITIES - 1), named name.  The
 * task's control block comes from the FreeRTOS heap (FreeRTOS keeps no list
 * of them; it stays allocated if the task ends).  *handle is set before the
 * task can run (the scheduler is suspended around the creation), as Zephyr
 * starts its static threads with the scheduler locked.  0, -EINVAL (bad
 * priority or stack) or -ENOMEM.
 */
int z_os_task_start(TaskFunction_t fn, const char *name, k_thread_stack_t *stack, size_t stack_size,
		    int prio, void *arg, TaskHandle_t *handle);

/* Zephyr's k_yield(): gives the turn to the next ready task of the same
 * priority (FreeRTOS's taskYIELD()), or goes on if there is none.  Not from
 * an interrupt handler.
 */
void z_os_yield(void);

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_INTERNAL_H */
