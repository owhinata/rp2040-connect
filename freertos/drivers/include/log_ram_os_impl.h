/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the RAM log (common/log/log_ram_os.h, read only by
 * common/log/log_ram.c):
 *
 * - the ring in RAM that the start-up code leaves alone (the Pico SDK's
 *   __uninitialized_ram): it survives the watchdog's restart after a fatal
 *   error, "reboot" and the RESET button, not a power cycle;
 * - the lock: save_and_disable_interrupts() and restore_interrupts()
 *   (PRIMASK, no FreeRTOS call inside), usable from interrupt handlers;
 * - the clock: nano_mono_us() (the TIMER, os_clock.h of the application);
 * - no OS log of its own (the FreeRTOS kernel and the Pico SDK have none).
 */

#ifndef FREERTOS_DRIVERS_INCLUDE_LOG_RAM_OS_IMPL_H
#define FREERTOS_DRIVERS_INCLUDE_LOG_RAM_OS_IMPL_H

#include <stdint.h>

#include "pico.h"
#include "hardware/sync.h"

#include "os_clock.h"

#define LOG_RAM_OS_RETAINED(name) __uninitialized_ram(name)

typedef uint32_t log_ram_os_irqstate_t;

static inline log_ram_os_irqstate_t log_ram_os_irq_save(void)
{
	return save_and_disable_interrupts();
}

static inline void log_ram_os_irq_restore(log_ram_os_irqstate_t state)
{
	restore_interrupts(state);
}

static inline uint64_t log_ram_os_uptime_us(void)
{
	return nano_mono_us();
}

static inline void log_ram_os_attach(void)
{
}

#endif /* FREERTOS_DRIVERS_INCLUDE_LOG_RAM_OS_IMPL_H */
