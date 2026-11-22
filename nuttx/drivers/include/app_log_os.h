/* SPDX-License-Identifier: Apache-2.0 */
/* The NuttX part of the logging macros (common/log/app_log.h, which
 * includes this header first): the configuration (CONFIG_LOG_*_LEVEL) and
 * <syslog.h>.
 *
 * <syslog.h> defines LOG_ERR as a syslog priority.  This header includes
 * <syslog.h> first and app_log.h replaces LOG_ERR with the function-like
 * macro, so a file that includes app_log.h cannot use syslog(LOG_ERR, ...);
 * it can use APP_LOG_SYSLOG_ERR instead.
 *
 * The functions of app_log.h are in nuttx/drivers/log/log_ram.c.
 * Shared by the kernel side (nuttx/drivers, the board code) and the
 * applications: $(TOPDIR)/external/include.
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

#endif /* __NUTTX_DRIVERS_INCLUDE_APP_LOG_OS_H */
