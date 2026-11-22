/* SPDX-License-Identifier: MIT */
/* The FreeRTOS primitives of the generic kernel part of the Zephyr
 * compatibility layer (common/compat/kernel_generic/zephyr_kernel_os.h,
 * which states what each must do; read only through it).  What depends on the FreeRTOS port and the
 * board (z_os_in_isr(), z_os_yield_from_isr(), z_os_task_lock(), z_os_task_unlock(),
 * z_os_busy_wait_us() and the clock) is the port header "zephyr_os_port.h"
 * (freertos/drivers/compat/rp2040 on the board, freertos/tests/compat on the host's FreeRTOS POSIX
 * port); the rest is the same FreeRTOS calls on both:
 *
 *   wake flag            the task notification Z_NOTIFY_INDEX (latched: a
 *                        give before the take makes the take return at
 *                        once); index 0 is left to the Pico SDK and the
 *                        application
 *   z_os_isr_lock()      taskENTER_CRITICAL_FROM_ISR() (saves PRIMASK)
 *   z_os_thread_start()  xTaskCreateStatic() with the control block from
 *                        the FreeRTOS heap, the scheduler suspended around
 *                        it once it runs
 *   z_os_sleep()         vTaskDelay(): FreeRTOS's block time ends at the
 *                        current tick + n
 *   z_os_thread_local_*()  the task's thread local storage pointer
 *                        Z_TLS_INDEX
 *   z_os_suspend()       vTaskSuspend(NULL)
 *   k_mutex              a FreeRTOS recursive mutex with its priority
 *                        inheritance, created by its first use when its
 *                        handle is NULL
 *   Z_OS_ASSERT()        configASSERT() (FreeRTOSConfig.h: records the
 *                        fault and restarts the board)
 *
 * What the application provides: FreeRTOSConfig.h (the tick, two task
 * notifications per task, one thread local storage pointer, mutexes and
 * recursive mutexes, static and dynamic
 * allocation, configUSE_TICK_HOOK with a hook that calls
 * zephyr_compat_tick() of "zephyr_compat_os.h"), priorities_os.h
 * (common/compat/priorities.h) and os_clock.h (rp2040_timer_us(), read by the
 * board's port header).  Part of the FreeRTOS implementation of the layer
 * (freertos/drivers/compat; see freertos/drivers/README.md).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_KERNEL_OS_IMPL_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_KERNEL_OS_IMPL_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "zephyr_os_port.h"

/* Types and constants ------------------------------------------------------*/

typedef TaskHandle_t z_os_thread_t;
typedef UBaseType_t z_os_prio_t;
typedef TickType_t z_os_ticks_t;
typedef BaseType_t z_os_woken_t;

/* The board and the host tests tick at 1 ms (configTICK_RATE_HZ 1000; the
 * Zephyr build's CONFIG_SYS_CLOCK_TICKS_PER_SEC is 10000).  portMAX_DELAY
 * blocks for ever (INCLUDE_vTaskSuspend).
 */
#define Z_OS_TICKS_FOREVER portMAX_DELAY
#define Z_OS_TICK_HZ       configTICK_RATE_HZ

/* configMAX_TASK_NAME_LEN, 32 as Zephyr's CONFIG_THREAD_MAX_NAME_LEN. */
#define Z_OS_THREAD_NAME_SIZE configMAX_TASK_NAME_LEN

#if configTASK_NOTIFICATION_ARRAY_ENTRIES < 2
#error "the compatibility layer waits on task notification index 1"
#endif

/* The task notification that is the wake flag (k_sem_take(), k_poll()). */
#define Z_NOTIFY_INDEX 1

/* k_mutex: a FreeRTOS recursive mutex (recursive for its owner, with
 * FreeRTOS's priority inheritance) in the structure's own memory.  FreeRTOS
 * has no static initializer for one, so a mutex is created by its first
 * k_mutex_lock() or k_mutex_unlock() when handle is NULL (z_os_mutex_get()):
 * K_MUTEX_DEFINE() and a k_mutex left all zero (never initialized) both
 * work as an unlocked mutex, as Zephyr's do (lock_count 0).
 */
struct k_mutex {
	StaticSemaphore_t buffer;
	SemaphoreHandle_t handle;
};

#define Z_MUTEX_INITIALIZER(obj)                                                                   \
	{                                                                                          \
		.handle = NULL                                                                     \
	}

#define Z_OS_ASSERT(test) configASSERT(test)

/* The primitives that stand for one or two FreeRTOS calls are always
 * inlined, so that the compiler sees the FreeRTOS calls in their callers as
 * if those called FreeRTOS directly: with -Os its early inliner leaves a
 * wrapper whose body is larger than the call to the later inliner, and its
 * decisions in between (the partial inlining of zephyr_kernel.c's wake())
 * would differ.
 */
#define Z_OS_INLINE static inline __attribute__((always_inline))

/* Context and lock (z_os_in_isr(), z_os_task_lock(), z_os_task_unlock():
 * the port header) -----------------------------------------------------*/

Z_OS_INLINE uint32_t z_os_isr_lock(void)
{
	return (uint32_t)taskENTER_CRITICAL_FROM_ISR();
}

Z_OS_INLINE void z_os_isr_unlock(uint32_t state)
{
	taskEXIT_CRITICAL_FROM_ISR((UBaseType_t)state);
}

Z_OS_INLINE void z_os_yield(void)
{
	taskYIELD();
}

Z_OS_INLINE bool z_os_pre_kernel(void)
{
	return xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED;
}

/* Waits and wakes (z_os_yield_from_isr(): the port header) ------------------*/

Z_OS_INLINE void z_os_wait_prepare(void)
{
	(void)xTaskNotifyStateClearIndexed(NULL, Z_NOTIFY_INDEX);
	(void)ulTaskNotifyValueClearIndexed(NULL, Z_NOTIFY_INDEX, UINT32_MAX);
}

Z_OS_INLINE void z_os_wait(z_os_ticks_t ticks)
{
	(void)ulTaskNotifyTakeIndexed(Z_NOTIFY_INDEX, pdTRUE, ticks);
}

Z_OS_INLINE void z_os_wake(z_os_thread_t tid)
{
	(void)xTaskNotifyGiveIndexed(tid, Z_NOTIFY_INDEX);
}

Z_OS_INLINE void z_os_wake_from_isr(z_os_thread_t tid, z_os_woken_t *woken)
{
	vTaskNotifyGiveIndexedFromISR(tid, Z_NOTIFY_INDEX, woken);
}

/* Threads ------------------------------------------------------------------*/

Z_OS_INLINE z_os_thread_t z_os_thread_self(void)
{
	return xTaskGetCurrentTaskHandle();
}

Z_OS_INLINE z_os_prio_t z_os_thread_prio_self(void)
{
	return uxTaskPriorityGet(NULL);
}

/* Bytes; the port's smallest stack is 1024 (priorities.h). */
#define Z_OS_TASK_STACK_MIN 256

/* The priority must be 1..configMAX_PRIORITIES - 1.  The task's control
 * block comes from the FreeRTOS heap (FreeRTOS keeps no list of them; it
 * stays allocated if the task ends).  Once the scheduler runs, it is
 * suspended around the creation, so that *tid is set before the task can
 * run.
 */
static inline int z_os_thread_start(void (*fn)(void *arg), const char *name, void *stack,
				    size_t stack_size, int prio, void *arg, z_os_thread_t *tid)
{
	bool running = xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
	StaticTask_t *tcb;
	TaskHandle_t task;

	if (prio <= (int)tskIDLE_PRIORITY || prio >= configMAX_PRIORITIES || stack == NULL ||
	    stack_size < Z_OS_TASK_STACK_MIN) {
		return -EINVAL;
	}
	tcb = pvPortMalloc(sizeof(*tcb));
	if (tcb == NULL) {
		return -ENOMEM;
	}

	if (running) {
		vTaskSuspendAll();
	}
	task = xTaskCreateStatic(fn, name, stack_size / sizeof(StackType_t), arg, (UBaseType_t)prio,
				 (StackType_t *)stack, tcb);
	*tid = task;
	if (running) {
		(void)xTaskResumeAll();
	}

	if (task == NULL) {
		vPortFree(tcb);
		return -EINVAL;
	}
	return 0;
}

Z_OS_INLINE void z_os_thread_exit(void)
{
	vTaskDelete(NULL);
}

Z_OS_INLINE char *z_os_thread_name(z_os_thread_t tid)
{
	return pcTaskGetName(tid);
}

/* The thread local storage pointer that holds a task's custom data
 * (k_thread_custom_data_get(); the shell engine keeps a task's console in
 * it).  Before the scheduler starts FreeRTOS has no current task to ask.
 */
#define Z_TLS_INDEX 0

#if configNUM_THREAD_LOCAL_STORAGE_POINTERS <= Z_TLS_INDEX
#error "the compatibility layer needs configNUM_THREAD_LOCAL_STORAGE_POINTERS >= 1"
#endif

Z_OS_INLINE void *z_os_thread_local_get(void)
{
	if (z_os_pre_kernel()) {
		return NULL;
	}
	return pvTaskGetThreadLocalStoragePointer(NULL, Z_TLS_INDEX);
}

Z_OS_INLINE void z_os_thread_local_set(void *value)
{
	vTaskSetThreadLocalStoragePointer(NULL, Z_TLS_INDEX, value);
}

/* Time (z_os_busy_wait_us(): the port header) ------------------------------*/

Z_OS_INLINE void z_os_sleep(z_os_ticks_t ticks)
{
	vTaskDelay(ticks);
}

Z_OS_INLINE void z_os_suspend(void)
{
	vTaskSuspend(NULL);
}

/* rp2040_timer_us(): the board's TIMER (os_clock.h), or the host's clock. */
Z_OS_INLINE uint64_t z_os_clock_us(void)
{
	return rp2040_timer_us();
}

/* Mutexes --------------------------------------------------------------------*/

/* Priority inheritance (approximation, see freertos/drivers/README.md):
 * FreeRTOS raises the holder to the priority of the highest waiter, one
 * level only (Zephyr follows a chain of holders), and drops it back only when
 * the holder has released every mutex it holds (Zephyr adjusts it at each
 * unlock); a waiter that times out has it lowered again
 * (vTaskPriorityDisinheritAfterTimeout()).
 */

/* The mutex's FreeRTOS handle, created inside a critical section when it is
 * still NULL, so that two first users cannot both create it (creating gives
 * the new mutex once without waiting, which is allowed there).
 */
static inline SemaphoreHandle_t z_os_mutex_get(struct k_mutex *mutex)
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

Z_OS_INLINE int z_os_mutex_init(struct k_mutex *mutex)
{
	mutex->handle = xSemaphoreCreateRecursiveMutexStatic(&mutex->buffer);
	return mutex->handle != NULL ? 0 : -EINVAL;
}

Z_OS_INLINE bool z_os_mutex_lock(struct k_mutex *mutex, z_os_ticks_t ticks)
{
	return xSemaphoreTakeRecursive(z_os_mutex_get(mutex), ticks) == pdPASS;
}

Z_OS_INLINE int z_os_mutex_unlock(struct k_mutex *mutex)
{
	SemaphoreHandle_t handle = z_os_mutex_get(mutex);
	TaskHandle_t holder = xSemaphoreGetMutexHolder(handle);

	if (holder == NULL) {
		return -EINVAL;
	}
	/* As integers: the same comparison, which the compiler then orders as it
	 * did when this was the body of k_mutex_unlock() itself.
	 */
	if ((uintptr_t)holder != (uintptr_t)xTaskGetCurrentTaskHandle()) {
		return -EPERM;
	}
	return xSemaphoreGiveRecursive(handle) == pdPASS ? 0 : -EINVAL;
}

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_KERNEL_OS_IMPL_H */
