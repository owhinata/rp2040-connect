/* SPDX-License-Identifier: Apache-2.0 */
/* The RAM log: LOG_RAM_SIZE bytes (the OS's, app_log_os.h) kept across a
 * reset (the ring of log_ring.h, in
 * RAM that the startup code leaves alone), filled by the LOG_* macros
 * (app_log.h) and by the OS's own console log, shown by the "dmesg"
 * command.  log_ram.c implements these functions on log_ring.h and on the
 * OS's part declared in log_ram_os.h (on NuttX with syslog() as the OS's
 * log, common/compat/port/nuttx/log_ram_syslog.c; the command is nuttx/apps/dmesg;
 * on FreeRTOS common/shell/dmesg_shell.c).
 */

#ifndef COMMON_COMPAT_INCLUDE_LOG_RAM_H
#define COMMON_COMPAT_INCLUDE_LOG_RAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_log_os.h"

#ifndef LOG_RAM_SIZE
#  error "app_log_os.h must define LOG_RAM_SIZE"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/* Check the retained ring (empty it if its header is not intact) and make
 * the RAM log the OS's log (log_ram_os_attach(); on NuttX the syslog
 * channel).  Called once, early (on NuttX from the board's
 * rp2040_boardinitialize(), on the Pico SDK first in main()).
 */

void log_ram_initialize(void);

/* Append bytes as they are, with no prefix (the OS's own log: on NuttX the
 * syslog channel).  From any context; interrupts are disabled while
 * copying.
 */

void log_ram_store(const char *data, size_t length);

/* Copy the next piece of the log for "dmesg".  *position counts bytes from
 * boot; set *first to true before the first call.  Returns the number of
 * bytes copied into piece (at most max), 0 at the end.  The caller adds the
 * result to *position.  Interrupts are disabled while copying.
 */

size_t log_ram_piece(uint64_t *position, bool *first, char *piece,
                     size_t max);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_COMPAT_INCLUDE_LOG_RAM_H */
