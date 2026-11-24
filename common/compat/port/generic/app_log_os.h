/* SPDX-License-Identifier: MIT */
/* The OS part of common/compat/include/app_log.h (the LOG_* macros) for an OS
 * without a log of its own: nothing to
 * read first and no options (Zephyr's default level 3 and maximum 4), and the
 * size of the RAM log's ring (common/compat/internal/log_ring.h).  The functions are
 * in common/compat/log/log_ram.c (the RAM log; its part for the OSes on the
 * Pico SDK is common/compat/port/pico_sdk/log_ram_os_impl.h).  Part of
 * the layer for an OS without its own (common/compat/port/generic; NuttX's is
 * common/compat/port/nuttx/app_log_os.h).
 */

#ifndef COMMON_COMPAT_PORT_GENERIC_APP_LOG_OS_H
#define COMMON_COMPAT_PORT_GENERIC_APP_LOG_OS_H

#include <stdarg.h>

/* 4096 bytes, as the Zephyr build's ring: a fatal error leaves a record of a
 * few lines, which fault_boot() writes into the log at the next start
 * (the OS's fault exit, e.g. common/imu/port/pico_sdk/fault.c).
 */
#define LOG_RAM_SIZE 4096

#endif /* COMMON_COMPAT_PORT_GENERIC_APP_LOG_OS_H */
