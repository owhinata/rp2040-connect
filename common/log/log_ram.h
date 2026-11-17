/* SPDX-License-Identifier: Apache-2.0 */
/* The RAM log: 8192 bytes kept across a reset (the ring of log_ring.h, in
 * RAM that the startup code leaves alone), filled by the LOG_* macros
 * (app_log.h) and by the OS's own console log, shown by the "dmesg"
 * command.  The OS implements these functions on log_ring.h (on NuttX
 * nuttx/drivers/log/log_ram.c, with the ring in .noinit and syslog() as the
 * OS's log; the command is nuttx/apps/dmesg).
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_LOG_RAM_H
#define __NUTTX_DRIVERS_INCLUDE_LOG_RAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LOG_RAM_BUFFER_SIZE 8192

#ifdef __cplusplus
extern "C"
{
#endif

/* Check the retained ring (empty it if its header is not intact) and make
 * the RAM log the OS's log (on NuttX the syslog channel).  Called once,
 * early (on NuttX from the board's rp2040_boardinitialize()).
 */

void log_ram_initialize(void);

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

#endif /* __NUTTX_DRIVERS_INCLUDE_LOG_RAM_H */
