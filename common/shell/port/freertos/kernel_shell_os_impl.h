/* SPDX-License-Identifier: MIT */
/* The FreeRTOS and Pico SDK part of the generic shell commands
 * (common/shell/kernel_shell.c, which reads this file through
 * common/shell/port/kernel_shell_os.h, where each item is described).
 */

#ifndef KERNEL_SHELL_OS_IMPL_H
#define KERNEL_SHELL_OS_IMPL_H

#include <stdint.h>

#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "FreeRTOS.h"
#include "task.h"

#include "fault.h"

/* FreeRTOS's tick count since the scheduler started: 1 ms, 32 bits (it wraps
 * after about 49.7 days; the time since the start is read from the TIMER
 * instead, rp2040_timer_us()).
 */
static inline uint32_t kernel_shell_os_tick_count(void)
{
	return (uint32_t)(xPortIsInsideInterrupt() ? xTaskGetTickCountFromISR()
						   : xTaskGetTickCount());
}

/* The watchdog restarts the board delay_ms later; the task waits for it. */
static inline void kernel_shell_os_reboot(uint32_t delay_ms)
{
	watchdog_reboot(0, 0, delay_ms);
	vTaskDelay(portMAX_DELAY);
}

static inline void kernel_shell_os_bootsel(uint32_t delay_ms)
{
	vTaskDelay(pdMS_TO_TICKS(delay_ms));
	reset_usb_boot(0, 0);
}

/* The record fault_boot() took at the start (fault.c). */
static inline const struct fault_record *kernel_shell_os_fault_last(void)
{
	return fault_last();
}

#endif /* KERNEL_SHELL_OS_IMPL_H */
