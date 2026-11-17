/* SPDX-License-Identifier: MIT */
/* k_mutex of <zephyr/kernel.h> on FreeRTOS: a recursive mutex
 * (xSemaphoreCreateRecursiveMutexStatic()) in the k_mutex's own memory, with
 * FreeRTOS's priority inheritance.  Part of the small Zephyr compatibility
 * layer of the port (freertos/drivers/compat; NuttX's is
 * nuttx/drivers/compat/zephyr_mutex.c).
 *
 * Zephyr's k_mutex is unlocked when all zero: k_mutex_lock() takes it when
 * lock_count is 0 (kernel/mutex.c), so a mutex in a zeroed static structure
 * works before k_mutex_init().  Zephyr's MQTT library relies on it
 * (mqtt_abort() locks the client's mutex, which only mqtt_client_init()
 * initializes), and so do K_MUTEX_DEFINE() here (FreeRTOS has no static
 * initializer for a mutex) and the settings subsystem's
 * K_MUTEX_DEFINE(settings_lock).  A mutex whose handle is NULL is created by
 * its first lock or unlock, inside a critical section, so two first users
 * cannot both create it (creating gives the new mutex once without waiting,
 * which is allowed there).  Not from an interrupt handler, as in Zephyr.
 *
 * The results are Zephyr's (kernel/mutex.c): K_NO_WAIT on a mutex another
 * thread holds -EBUSY, a timeout -EAGAIN; unlocking a mutex that nobody
 * holds -EINVAL, one that another thread holds -EPERM.
 *
 * Priority inheritance (approximation, see freertos/README.md): FreeRTOS
 * raises the holder to the priority of the highest waiter, one level only
 * (Zephyr follows a chain of holders), and drops it back only when the holder
 * has released every mutex it holds (Zephyr adjusts it at each unlock); a
 * waiter that times out has it lowered again
 * (vTaskPriorityDisinheritAfterTimeout()).
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <zephyr/kernel.h>

#include "zephyr_os_internal.h"

static SemaphoreHandle_t mutex_handle(struct k_mutex *mutex)
{
	SemaphoreHandle_t handle = *(SemaphoreHandle_t volatile *)&mutex->handle;

	if (handle == NULL) {
		taskENTER_CRITICAL();
		if (mutex->handle == NULL) {
			mutex->handle = xSemaphoreCreateRecursiveMutexStatic(&mutex->buffer);
		}
		handle = mutex->handle;
		taskEXIT_CRITICAL();
	}
	return handle;
}

int k_mutex_init(struct k_mutex *mutex)
{
	mutex->handle = xSemaphoreCreateRecursiveMutexStatic(&mutex->buffer);
	return mutex->handle != NULL ? 0 : -EINVAL;
}

int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
	TickType_t ticks = z_os_wait_ticks(timeout);

	if (xSemaphoreTakeRecursive(mutex_handle(mutex), ticks) == pdPASS) {
		return 0;
	}
	return ticks == 0 ? -EBUSY : -EAGAIN;
}

int k_mutex_unlock(struct k_mutex *mutex)
{
	SemaphoreHandle_t handle = mutex_handle(mutex);
	TaskHandle_t holder = xSemaphoreGetMutexHolder(handle);

	if (holder == NULL) {
		return -EINVAL;
	}
	if (holder != xTaskGetCurrentTaskHandle()) {
		return -EPERM;
	}
	return xSemaphoreGiveRecursive(handle) == pdPASS ? 0 : -EINVAL;
}
