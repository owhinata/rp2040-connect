/* SPDX-License-Identifier: Apache-2.0 */
/* The RAM log (log_ram.h) and the functions of the LOG_* macros
 * (app_log.h) for the builds other than Zephyr's: a port of the Zephyr
 * build's zephyr/app/usb-console/src/log_ram.c, on the OS's part declared
 * in log_ram_os.h.
 *
 * - The ring (log_ring.[ch]) is in RAM that the start-up code leaves alone
 *   (LOG_RAM_OS_RETAINED(); Zephyr: __noinit) and survives a restart that
 *   keeps the RAM powered; log_ram_initialize() empties it only when its
 *   header is not intact (after power-on).  `dropped` (bytes pushed out
 *   since the start) is cleared at every start, as in Zephyr.
 * - The lock is "interrupts disabled" (log_ram_os_irq_save(); Zephyr: a
 *   spinlock, the same thing on one core), so the LOG_* macros may be used
 *   in interrupt handlers and before the scheduler starts.
 * - LOG_* lines are formatted by app_log_vprintf() at once, in the caller's
 *   context and on its stack: "[HH:MM:SS.mmm,uuu] <lvl> module: message\n"
 *   with the time of the call (log_ram_os_uptime_us()).  Zephyr's deferred
 *   mode takes the time at the call too but formats later in its logging
 *   thread.
 * - The OS's own log, if it has one, goes in as it is with log_ram_store()
 *   (log_ram_os_attach(); on NuttX syslog(), common/compat/port/nuttx/
 *   log_ram_syslog.c; Zephyr: printk, CONFIG_LOG_PRINTK).
 */

#include "log_ram_os.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_log.h"
#include "log_ram.h"
#include "log_ring.h"

/* The start-up code leaves this region untouched. */

static struct retained_log LOG_RAM_OS_RETAINED(retained);

/* Bytes pushed out of the buffer since the start.  Under the lock. */

static uint64_t dropped;

/* Inline in app_log_vprintf(): one call less on the path of every line. */

static inline void log_ram_append(const char *data, size_t length)
{
  log_ram_os_irqstate_t flags = log_ram_os_irq_save();

  log_ring_store(&retained, &dropped, data, length);
  log_ram_os_irq_restore(flags);
}

/* Public functions ----------------------------------------------------- */

void log_ram_initialize(void)
{
  log_ram_os_irqstate_t flags = log_ram_os_irq_save();

  log_ring_validate(&retained);
  log_ram_os_irq_restore(flags);

  log_ram_os_attach();
}

void log_ram_store(const char *data, size_t length)
{
  log_ram_append(data, length);
}

size_t log_ram_piece(uint64_t *position, bool *first, char *piece,
                     size_t max)
{
  log_ram_os_irqstate_t flags = log_ram_os_irq_save();
  size_t n = log_ring_piece(&retained, dropped, position, first,
                            piece, max);

  log_ram_os_irq_restore(flags);
  return n;
}

void app_log_vprintf(int level, const char *module, const char *func,
                     const char *fmt, va_list ap)
{
  char line[APP_LOG_LINE_MAX];
  uint64_t now = log_ram_os_uptime_us();
  size_t room = sizeof(line) - 1;  /* keep one byte for '\n' */
  size_t length;
  int n;

  n = log_ring_prefix(line, room + 1, now, level, module, func);
  length = n < 0 ? 0 : ((size_t)n < room ? (size_t)n : room);
  if (length < room)
    {
      n = vsnprintf(line + length, room - length + 1, fmt, ap);
      if (n > 0)
        {
          length += (size_t)n < room - length ? (size_t)n : room - length;
        }
    }

  line[length++] = '\n';
  log_ram_append(line, length);
}

void app_log_printf(int level, const char *module, const char *func,
                    const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  app_log_vprintf(level, module, func, fmt, ap);
  va_end(ap);
}

bool app_log_ratelimit_pass(uint32_t *last_ms, uint32_t *skipped,
                            uint32_t rate_ms, uint32_t *skipped_out)
{
  /* Zephyr: k_uptime_get_32(), milliseconds since the start. */

  uint32_t now_ms = (uint32_t)(log_ram_os_uptime_us() / 1000U);
  log_ram_os_irqstate_t flags = log_ram_os_irq_save();
  bool pass = log_ring_ratelimit(last_ms, skipped, now_ms, rate_ms,
                                 skipped_out);

  log_ram_os_irq_restore(flags);
  return pass;
}
