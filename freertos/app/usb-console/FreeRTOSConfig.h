/* SPDX-License-Identifier: MIT */
/* FreeRTOS kernel configuration of the USB console (RP2040, one core). */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Scheduler ---------------------------------------------------------------*/

#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_TICKLESS_IDLE                 0
#define configTICK_RATE_HZ                      1000
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configMAX_PRIORITIES                    32
#define configMAX_TASK_NAME_LEN                 16
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0

/* The RP2040 port: one core (the second one stays in the boot ROM), and
 * the Pico SDK's locks and sleeps (mutex_enter_blocking(), sleep_ms(), the
 * lock inside stdio) block the calling task instead of spinning. */

#define configNUMBER_OF_CORES                   1
#define configSUPPORT_PICO_SYNC_INTEROP         1
#define configSUPPORT_PICO_TIME_INTEROP         1

/* Kernel objects ------------------------------------------------------------*/

#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_QUEUE_SETS                    0
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_EVENT_GROUPS                  1
#define configUSE_STREAM_BUFFERS                1

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            256

/* Memory ----------------------------------------------------------------------*/

/* Stack sizes are in words (4 bytes). */

#define configMINIMAL_STACK_SIZE                256
#define configSUPPORT_STATIC_ALLOCATION         1
#define configKERNEL_PROVIDED_STATIC_MEMORY     1   /* idle and timer tasks */
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   (32 * 1024) /* heap_4 */
#define configAPPLICATION_ALLOCATED_HEAP        0
#define configSTACK_DEPTH_TYPE                  uint32_t

/* Checks ----------------------------------------------------------------------*/

#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1

#ifndef __ASSEMBLER__
void app_assert_failed(const char *file, int line);
#define configASSERT(x)                                                                            \
	do {                                                                                       \
		if (!(x)) {                                                                        \
			app_assert_failed(__FILE__, __LINE__);                                     \
		}                                                                                  \
	} while (0)
#endif

/* The "ps" command ------------------------------------------------------------*/

/* Run time is counted in microseconds of the RP2040 TIMER. */

#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configGENERATE_RUN_TIME_STATS           1
#define configRUN_TIME_COUNTER_TYPE             uint64_t

#ifndef __ASSEMBLER__
#include <stdint.h>
uint64_t app_run_time_us(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()
#define portGET_RUN_TIME_COUNTER_VALUE() app_run_time_us()
#endif

/* Optional functions ----------------------------------------------------------*/

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xSemaphoreGetMutexHolder        1

#endif /* FREERTOS_CONFIG_H */
