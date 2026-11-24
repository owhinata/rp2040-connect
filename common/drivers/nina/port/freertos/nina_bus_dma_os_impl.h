/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the NINA link's DMA waits (port/pico_sdk/nina_bus_dma_os.h,
 * which describes each item): the calling task's notification
 * NINA_BUS_DMA_NOTIFY_INDEX, the NINA's index (NINA_OS_NOTIFY_INDEX of
 * nina_os.c, 2: index 0 is the Pico SDK's and the application's, 1 the
 * compatibility layer's, 3 the I2C's; FreeRTOSConfig.h).  A task waits for
 * at most one of them at a time.
 *
 *   nina_bus_dma_os_wake_clear()      xTaskNotifyStateClearIndexed() and
 *                                     ulTaskNotifyValueClearIndexed()
 *   nina_bus_dma_os_wake_wait()       ulTaskNotifyTakeIndexed()
 *   nina_bus_dma_os_wake_from_isr()   vTaskNotifyGiveIndexedFromISR()
 *   nina_bus_dma_os_isr_end()         portYIELD_FROM_ISR() (PendSV)
 */

#ifndef NINA_BUS_DMA_OS_IMPL_H_
#define NINA_BUS_DMA_OS_IMPL_H_

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

/* The NINA's task notification (nina_os.c). */
#define NINA_BUS_DMA_NOTIFY_INDEX 2

#if configTASK_NOTIFICATION_ARRAY_ENTRIES <= NINA_BUS_DMA_NOTIFY_INDEX
#error "the NINA's SPI DMA waits use task notification index 2"
#endif

typedef TaskHandle_t nina_bus_dma_os_thread_t;
typedef BaseType_t nina_bus_dma_os_woken_t;
typedef TickType_t nina_bus_dma_os_ticks_t;

#define NINA_BUS_DMA_OS_NOT_WOKEN pdFALSE

/* Always inlined, so that the compiler sees the FreeRTOS calls in their
 * callers as if those called FreeRTOS directly (as Z_OS_INLINE of the
 * compatibility layer's zephyr_kernel_os_impl.h: with -Os the early inliner
 * would otherwise leave wrappers and change its later decisions).
 */
#define NINA_BUS_DMA_OS_INLINE static inline __attribute__((always_inline))

NINA_BUS_DMA_OS_INLINE bool nina_bus_dma_os_running(void)
{
	return xTaskGetSchedulerState() == taskSCHEDULER_RUNNING;
}

NINA_BUS_DMA_OS_INLINE nina_bus_dma_os_thread_t nina_bus_dma_os_self(void)
{
	return xTaskGetCurrentTaskHandle();
}

NINA_BUS_DMA_OS_INLINE void nina_bus_dma_os_wake_clear(void)
{
	(void)xTaskNotifyStateClearIndexed(NULL, NINA_BUS_DMA_NOTIFY_INDEX);
	(void)ulTaskNotifyValueClearIndexed(NULL, NINA_BUS_DMA_NOTIFY_INDEX, UINT32_MAX);
}

NINA_BUS_DMA_OS_INLINE nina_bus_dma_os_ticks_t nina_bus_dma_os_ticks_from_ms(uint64_t ms)
{
	return (TickType_t)((ms * configTICK_RATE_HZ + 999u) / 1000u + 1u);
}

NINA_BUS_DMA_OS_INLINE nina_bus_dma_os_ticks_t nina_bus_dma_os_tick_count(void)
{
	return xTaskGetTickCount();
}

NINA_BUS_DMA_OS_INLINE void nina_bus_dma_os_wake_wait(nina_bus_dma_os_ticks_t ticks)
{
	(void)ulTaskNotifyTakeIndexed(NINA_BUS_DMA_NOTIFY_INDEX, pdTRUE, ticks);
}

NINA_BUS_DMA_OS_INLINE void nina_bus_dma_os_wake_from_isr(nina_bus_dma_os_thread_t thread,
							  nina_bus_dma_os_woken_t *woken)
{
	vTaskNotifyGiveIndexedFromISR(thread, NINA_BUS_DMA_NOTIFY_INDEX, woken);
}

NINA_BUS_DMA_OS_INLINE void nina_bus_dma_os_isr_end(nina_bus_dma_os_woken_t woken)
{
	portYIELD_FROM_ISR(woken);
}

#endif /* NINA_BUS_DMA_OS_IMPL_H_ */
