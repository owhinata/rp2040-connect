/* SPDX-License-Identifier: MIT */
/* What the RAM log (log_ram.c here) needs from the OS.  log_ram.c includes
 * this header before anything else; it includes the OS's part,
 * "log_ram_os_impl.h", then declares the functions that part must define,
 * so a different one does not compile.  The OS's part defines them as
 * static inline functions, so that they are compiled into log_ram.c as if
 * it called the OS directly: app_log_vprintf() runs for every LOG_*() line,
 * in threads, in interrupt handlers, before the scheduler starts and on the
 * way to a fatal error's restart, and must not get a deeper call chain or
 * anything that can block there.
 *
 * Each OS's build puts its "log_ram_os_impl.h" on the include path of
 * log_ram.c (the NuttX build's is common/compat/port/nuttx/log_ram_os_impl.h,
 * the FreeRTOS build's common/compat/port/pico_sdk/log_ram_os_impl.h).  It
 * defines:
 *
 *   LOG_RAM_OS_RETAINED(name)
 *       The declarator of the ring, struct retained_log name, with what
 *       puts it in RAM that the start-up code neither clears nor copies, so
 *       that it survives a restart that keeps the RAM powered.
 *   log_ram_os_irqstate_t
 *       The type of the state that log_ram_os_irq_save() returns.
 *
 * and the functions below.
 */

#ifndef COMMON_COMPAT_PORT_LOG_RAM_OS_H
#define COMMON_COMPAT_PORT_LOG_RAM_OS_H

#include "log_ram_os_impl.h"

#include <stdint.h>

#ifndef LOG_RAM_OS_RETAINED
#  error "log_ram_os_impl.h must define LOG_RAM_OS_RETAINED(name)"
#endif

/* Disable the interrupts of the core and return their previous state; the
 * lock of the ring, the rate limit and log_ram_piece() (one core: no other
 * lock).  From any context, nested; no OS call that can block.
 */

static log_ram_os_irqstate_t log_ram_os_irq_save(void);

/* Restore the state returned by log_ram_os_irq_save(). */

static void log_ram_os_irq_restore(log_ram_os_irqstate_t state);

/* Microseconds since the start of the board (the time stamp of each line
 * and, divided by 1000, the uptime of the rate limit).  From any context,
 * with interrupts disabled too; no lock.
 */

static uint64_t log_ram_os_uptime_us(void);

/* Called once by log_ram_initialize(), after the ring has been checked and
 * with interrupts enabled: make the RAM log the OS's own log as well, if
 * the OS has one (its bytes go in with log_ram_store()), or do nothing.
 */

static void log_ram_os_attach(void);

#endif /* COMMON_COMPAT_PORT_LOG_RAM_OS_H */
