/* SPDX-License-Identifier: MIT */
/* FreeRTOS kernel configuration of the host test of the Zephyr compatibility
 * layer (freertos/tests/compat, the FreeRTOS POSIX port; the shell's test,
 * freertos/tests/shell, uses it as well): the board's
 * (freertos/app/usb-console/FreeRTOSConfig.h) where the layer depends on it
 * (time slices by the tick hook, four task notifications, mutexes, static
 * and dynamic allocation, the priorities, the length of task names, the 1 ms
 * tick; the POSIX port's tick is a thread that sleeps a tick between
 * signals), with the C library's heap.  The NINA binding's test
 * (freertos/tests/nina) builds its kernel once more with another tick
 * (-DconfigTICK_RATE_HZ).
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "priorities_os.h"

#define configUSE_PREEMPTION                  1
#ifndef configTICK_RATE_HZ
#define configTICK_RATE_HZ                    1000
#endif
#define configTICK_TYPE_WIDTH_IN_BITS         TICK_TYPE_WIDTH_64_BITS
#define configUSE_TICKLESS_IDLE               0
#define configUSE_TIME_SLICING                0
#define configUSE_TICK_HOOK                   1
#define configMAX_PRIORITIES                  32
#define configMAX_TASK_NAME_LEN               32
#define configIDLE_SHOULD_YIELD               1
#define configUSE_IDLE_HOOK                   0
#define configNUMBER_OF_CORES                 1

#define configUSE_MUTEXES                     1
#define configUSE_RECURSIVE_MUTEXES           1
#define configUSE_COUNTING_SEMAPHORES         1
#define configUSE_TASK_NOTIFICATIONS          1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES 4
#define configUSE_QUEUE_SETS                  0
#define configQUEUE_REGISTRY_SIZE             0
#define configUSE_EVENT_GROUPS                0
#define configUSE_STREAM_BUFFERS              0
/* Pointer 0: the task's custom data of the layer, its console
 * (common/compat/shell_engine/shell_console.h, the host test of
 * freertos/tests/shell). */
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 1
#define configUSE_TIMERS                      0

#define configMINIMAL_STACK_SIZE              256
#define configSUPPORT_STATIC_ALLOCATION       1
#define configKERNEL_PROVIDED_STATIC_MEMORY   1
#define configSUPPORT_DYNAMIC_ALLOCATION      1
#define configTOTAL_HEAP_SIZE                 (64 * 1024) /* unused by heap_3 */
#define configCHECK_FOR_STACK_OVERFLOW        0
#define configUSE_MALLOC_FAILED_HOOK          0
#define configUSE_TRACE_FACILITY              0
#define configGENERATE_RUN_TIME_STATS         0

#include <assert.h>
#define configASSERT(x) assert(x)

#define INCLUDE_vTaskPrioritySet          1
#define INCLUDE_uxTaskPriorityGet         1
#define INCLUDE_vTaskDelete               1
#define INCLUDE_vTaskSuspend              1
#define INCLUDE_vTaskDelay                1
#define INCLUDE_xTaskDelayUntil           1
#define INCLUDE_xTaskGetSchedulerState    1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_xTaskGetIdleTaskHandle    1
#define INCLUDE_eTaskGetState             1
#define INCLUDE_xTaskAbortDelay           1
#define INCLUDE_xSemaphoreGetMutexHolder  1

#endif /* FREERTOS_CONFIG_H */
