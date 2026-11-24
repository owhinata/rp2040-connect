/* SPDX-License-Identifier: MIT */
/* The part of the RAM log for an OS on the Pico SDK
 * (common/compat/log/log_ram_os.h, read only by
 * common/compat/log/log_ram.c):
 *
 * - the ring in RAM that the start-up code leaves alone (the Pico SDK's
 *   __uninitialized_ram): it survives the watchdog's restart after a fatal
 *   error, "reboot" and the RESET button, not a power cycle;
 * - the lock: save_and_disable_interrupts() and restore_interrupts()
 *   (PRIMASK, no kernel call inside), usable from interrupt handlers;
 * - the clock: rp2040_timer_us() (the TIMER, common/chip/rp2040);
 * - no OS log of its own (the kernels on the Pico SDK and the SDK have
 *   none).
 */

#ifndef COMMON_PICO_SDK_DRIVERS_INCLUDE_LOG_RAM_OS_IMPL_H
#define COMMON_PICO_SDK_DRIVERS_INCLUDE_LOG_RAM_OS_IMPL_H

#include <stdint.h>

#include "pico.h"
#include "hardware/sync.h"

#include "rp2040_timer_us.h"

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
	return rp2040_timer_us();
}

static inline void log_ram_os_attach(void)
{
}

#endif /* COMMON_PICO_SDK_DRIVERS_INCLUDE_LOG_RAM_OS_IMPL_H */
