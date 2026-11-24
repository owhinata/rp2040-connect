/* SPDX-License-Identifier: Apache-2.0 */
/* The parts of the RAM log (log_ram.c) that need no lock and no clock: the
 * retained ring buffer, the line prefix and the rate-limit decision.  The
 * caller holds the lock.  Kept apart so the host tests (common/tests/log_ring.c) can run
 * them.  The ring and its rules are those of the Zephyr build's
 * zephyr/app/usb-console/src/log_ram.c.
 */

#ifndef COMMON_COMPAT_INTERNAL_LOG_RING_H
#define COMMON_COMPAT_INTERNAL_LOG_RING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* LOG_RAM_SIZE, the ring's size in bytes, is the OS's (its app_log_os.h:
 * 8192 in the NuttX build, whose fatal error report with the task list has
 * to fit from its first line; 4096 in the FreeRTOS build, as the Zephyr
 * build's ring).
 */

#include "app_log_os.h"

#ifndef LOG_RAM_SIZE
#  error "app_log_os.h must define LOG_RAM_SIZE"
#endif

/* The Zephyr build's magic number 0x4c4f4752 with the size mixed in
 * (0x4c4f6752 for 8192, 0x4c4f5752 for 4096), so that a ring of another
 * size left in RAM (another OS's firmware, the Zephyr build's plain
 * 0x4c4f4752) fails the header check at boot and is emptied instead of
 * being read with the wrong wrap-around.
 */

#define LOG_RAM_MAGIC (0x4c4f4752U ^ LOG_RAM_SIZE)

/* Kept across a reset in .noinit (Zephyr: __noinit). */

struct retained_log
{
  uint32_t magic;
  uint32_t magic_inverse;
  size_t start;
  size_t length;
  char buffer[LOG_RAM_SIZE];
};

/* Empty the ring unless its header is intact (magic, its inverse, start and
 * length in range); Zephyr's ram_init().
 */

void log_ring_validate(struct retained_log *log);

/* Append bytes, pushing the oldest ones out when full; *dropped counts the
 * bytes pushed out since boot (Zephyr's store_output()).
 */

void log_ring_store(struct retained_log *log, uint64_t *dropped,
                    const char *data, size_t length);

/* One piece for "dmesg" (the body of the loop in Zephyr's cmd_dmesg()):
 * *position numbers bytes from boot like dropped.  On the first call
 * (*first) or when the bytes at *position have been pushed out, start from
 * the oldest byte.  Copies up to max bytes from *position into piece and
 * returns how many; the caller adds that to *position.
 */

size_t log_ring_piece(const struct retained_log *log, uint64_t dropped,
                      uint64_t *position, bool *first, char *piece,
                      size_t max);

/* The line prefix of Zephyr's log_output.c with LOG_OUTPUT_FLAG_TIMESTAMP |
 * FORMAT_TIMESTAMP | LEVEL and 64-bit timestamps:
 * "[HH:MM:SS.mmm,uuu] <lvl> module: ", or "module.func: " when func is not
 * NULL.  us is microseconds; hours are printed with at least two digits.
 * Returns the length (snprintf rules).
 */

int log_ring_prefix(char *buf, size_t size, uint64_t us, int level,
                    const char *module, const char *func);

/* The rate-limit decision of Zephyr's _LOG_RATELIMIT_CORE for a call site
 * whose state is *last_ms and *skipped, at uptime now_ms (32-bit
 * milliseconds).  The caller makes it atomic.
 */

bool log_ring_ratelimit(uint32_t *last_ms, uint32_t *skipped,
                        uint32_t now_ms, uint32_t rate_ms,
                        uint32_t *skipped_out);

#endif /* COMMON_COMPAT_INTERNAL_LOG_RING_H */
