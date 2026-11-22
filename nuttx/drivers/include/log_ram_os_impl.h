/* SPDX-License-Identifier: Apache-2.0 */
/* The NuttX part of the RAM log (common/log/log_ram_os.h, read only by
 * common/log/log_ram.c):
 *
 * - the ring in .noinit (arduino-nano-rp2040-connect-flash.ld places it
 *   between .data and .bss, so the startup code neither clears nor copies
 *   it): it survives "reboot", the RESET button and the restart after a
 *   fatal error, not a power cycle;
 * - the lock: up_irq_save() and up_irq_restore() (PRIMASK, always inline,
 *   no call into the OS), usable from interrupt handlers;
 * - the clock: rp2040_timer_us() (the TIMER,
 *   common/chip/rp2040/include/rp2040_timer_us.h);
 * - the OS's own log: the RAM log is the syslog channel
 *   (nuttx/drivers/log/log_ram_syslog.c).
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_LOG_RAM_OS_IMPL_H
#define __NUTTX_DRIVERS_INCLUDE_LOG_RAM_OS_IMPL_H

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/compiler.h>
#include <nuttx/irq.h>

#include "rp2040_timer_us.h"

#define LOG_RAM_OS_RETAINED(name) name locate_data(".noinit")

typedef irqstate_t log_ram_os_irqstate_t;

/* Register the RAM log as the syslog channel (log_ram_syslog.c). */

void log_ram_syslog_register(void);

static inline log_ram_os_irqstate_t log_ram_os_irq_save(void)
{
  return up_irq_save();
}

static inline void log_ram_os_irq_restore(log_ram_os_irqstate_t state)
{
  up_irq_restore(state);
}

static inline uint64_t log_ram_os_uptime_us(void)
{
  return rp2040_timer_us();
}

static inline void log_ram_os_attach(void)
{
  log_ram_syslog_register();
}

#endif /* __NUTTX_DRIVERS_INCLUDE_LOG_RAM_OS_IMPL_H */
