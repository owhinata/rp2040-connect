/* SPDX-License-Identifier: Apache-2.0 */
/* The RAM log (drivers/log/log_ram.c): 8192 bytes kept in .noinit across a
 * reset, filled by the LOG_* macros (app_log.h) and by everything written
 * through syslog(), shown by the "dmesg" command (nuttx/apps/dmesg).
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_LOG_RAM_H
#define __NUTTX_DRIVERS_INCLUDE_LOG_RAM_H

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/compiler.h>

#define LOG_RAM_BUFFER_SIZE 8192

#ifdef __cplusplus
extern "C"
{
#endif

/* Check the retained ring (empty it if its header is not intact) and make
 * the RAM log the syslog channel.  Called once, early, from the board's
 * rp2040_boardinitialize().
 */

void log_ram_initialize(void);

/* Copy the next piece of the log for "dmesg".  *position counts bytes from
 * boot; set *first to true before the first call.  Returns the number of
 * bytes copied into piece (at most max), 0 at the end.  The caller adds the
 * result to *position.  Interrupts are disabled while copying.
 */

size_t log_ram_piece(FAR uint64_t *position, FAR bool *first,
                     FAR char *piece, size_t max);

#ifdef __cplusplus
}
#endif

#endif /* __NUTTX_DRIVERS_INCLUDE_LOG_RAM_H */
