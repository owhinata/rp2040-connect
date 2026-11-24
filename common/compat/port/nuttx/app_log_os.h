/* SPDX-License-Identifier: Apache-2.0 */
/* The NuttX part of the logging macros (common/compat/include/app_log.h, which
 * includes this header first): the configuration (CONFIG_LOG_*_LEVEL),
 * <syslog.h>, and the size of the RAM log's ring
 * (common/compat/internal/log_ring.h, log_ram.h).
 *
 * <syslog.h> defines LOG_ERR as a syslog priority.  This header includes
 * <syslog.h> first and app_log.h replaces LOG_ERR with the function-like
 * macro, so a file that includes app_log.h cannot use syslog(LOG_ERR, ...);
 * it can use APP_LOG_SYSLOG_ERR instead.
 *
 * The functions of app_log.h are in common/compat/log/log_ram.c (its NuttX part:
 * log_ram_os_impl.h here).
 * Shared by the kernel side (nuttx/drivers, the board code) and the
 * applications: common/compat/port/nuttx.
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_APP_LOG_OS_H
#define __NUTTX_DRIVERS_INCLUDE_APP_LOG_OS_H

#include <nuttx/config.h>

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/compiler.h>

/* The syslog priority that <syslog.h> calls LOG_ERR. */

#define APP_LOG_SYSLOG_ERR 3
#undef LOG_ERR

/* 8192 bytes (the Zephyr build's ring has 4096): a fatal error report with
 * the task list of the whole port has to fit from its first line.
 */

#define LOG_RAM_SIZE 8192

#endif /* __NUTTX_DRIVERS_INCLUDE_APP_LOG_OS_H */
