/* SPDX-License-Identifier: MIT */
/* FreeRTOS kernel configuration of the USB console (RP2040, one core). */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "priorities_os.h"

/* Scheduler ---------------------------------------------------------------*/

/* The tick is 1 ms (the Zephyr build's CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000
 * and the NuttX build's CONFIG_USEC_PER_TICK=100 are 100 us).  It is 32 bits
 * wide (the RP2040 port has no 64-bit tick) and wraps after about 49.7 days;
 * the time since the start comes from the TIMER instead (rp2040_timer_us()).
 * Both other builds are tickless; this one has a periodic tick (SysTick),
 * whose handler counts as CPU load.
 */
#define configUSE_PREEMPTION                    1
#define configTICK_RATE_HZ                      1000
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configUSE_TICKLESS_IDLE                 0

/* Time slices of 20 ms among tasks of the same priority (Zephyr:
 * CONFIG_TIMESLICE_SIZE=20, CONFIG_TIMESLICE_PRIORITY=0) are made by the tick
 * hook (src/tick.c); FreeRTOS's own slicing would switch at every tick.
 */
#define configUSE_TIME_SLICING                  0
#define configUSE_TICK_HOOK                     1

/* Priorities 0..31, a higher number runs first; the port's are
 * 20 - (Zephyr priority) (src/priorities_os.h, common/compat/priorities.h).
 * Task names as long as Zephyr's (CONFIG_THREAD_MAX_NAME_LEN=32, with the
 * terminating null).
 */
#define configMAX_PRIORITIES                    32
#define configMAX_TASK_NAME_LEN                 32

/* The idle task measures the CPU load (src/idle.c); no task of the port has
 * the idle priority.
 */
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_IDLE_HOOK                     1

/* The RP2040 port: one core (the second one stays in the boot ROM), and
 * the Pico SDK's locks and sleeps (mutex_enter_blocking(), sleep_ms(), the
 * lock inside stdio) block the calling task instead of spinning. */

#define configNUMBER_OF_CORES                   1
#define configSUPPORT_PICO_SYNC_INTEROP         1
#define configSUPPORT_PICO_TIME_INTEROP         1

/* The key-value store's flash operations do not stop core 1 while XIP is
 * off (common/pico_sdk/drivers/kv/rp2040_flash_kv_os.c).
 */
#if configNUMBER_OF_CORES != 1
#error "rp2040_flash_kv_os.c does not stop the other core while XIP is off"
#endif

/* Kernel objects ------------------------------------------------------------*/

#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_TASK_NOTIFICATIONS            1
/* Index 0 for the Pico SDK (pico_async_context_freertos) and the port's
 * plain notifications, index 1 for the compatibility layer's waits (F02),
 * index 2 for the NINA's waits (the socket events and the Wi-Fi thread's
 * wake-up, common/drivers/nina/port/freertos/nina_os.c, and the SPI DMA,
 * nina_bus_dma_rp2040.c), index 3 for the I2C
 * driver's semaphores (common/drivers/i2c_dw/port/freertos/i2c_dw_os_impl.h).
 */
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   4
#define configUSE_QUEUE_SETS                    0
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_EVENT_GROUPS                  1
#define configUSE_STREAM_BUFFERS                1
/* Pointer 0: the task's custom data of the layer, its console
 * (common/compat/shell_engine/shell_console.h). */
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 1

/* The timer task also wakes the tasks that wait on the Pico SDK's locks and
 * sleeps when an interrupt handler releases them; its priority and stack
 * are in src/priorities_os.h.
 */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               APP_PRIO_TIMER_SVC
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            (APP_STACK_TIMER_SVC / 4)

/* Memory ----------------------------------------------------------------------*/

/* Stack sizes are in words (4 bytes). */

#define configMINIMAL_STACK_SIZE                (APP_STACK_IDLE / 4) /* the idle task */
#define configSUPPORT_STATIC_ALLOCATION         1
#define configKERNEL_PROVIDED_STATIC_MEMORY     1   /* idle and timer tasks */
/* No kernel heap: every task, queue and semaphore is static (the
 * compatibility layer's threads keep their control blocks after their
 * stacks, freertos/drivers/compat/zephyr_kernel_os_impl.h), and a call that
 * would allocate does not link.  malloc() is newlib's (the shell, "zperf",
 * the NINA passthrough mode's buffer).
 */
#define configSUPPORT_DYNAMIC_ALLOCATION        0
#define configSTACK_DEPTH_TYPE                  uint32_t

/* Checks ----------------------------------------------------------------------*/

/* Both end in the common exit of fatal errors, which records the error and
 * restarts the board (src/fault.c of common/pico_sdk/app/usb-console; the
 * stack overflow hook is src/fault_os.c).
 */
#define configCHECK_FOR_STACK_OVERFLOW          2

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

/* Run time is counted in microseconds of the RP2040 TIMER
 * (app_run_time_us(), src/commands_os.c).
 */

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
