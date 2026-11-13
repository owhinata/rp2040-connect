/* SPDX-License-Identifier: Apache-2.0 */
/* The RAM log: a port of the Zephyr build's
 * zephyr/app/usb-console/src/log_ram.c.
 *
 * - The ring (log_ring.[ch]) lives in .noinit (nano-flash.ld places it
 *   between .data and .bss, so the startup code neither clears nor copies
 *   it) and survives a reset; log_ram_initialize() empties it only when its
 *   header is not intact.  `dropped` (bytes pushed out since boot) is in
 *   .bss and starts at 0 on every boot, as in Zephyr.
 * - The lock is "interrupts disabled" (Zephyr: a spinlock, the same thing
 *   on one core), so it may be used from interrupt handlers.
 * - LOG_* lines (app_log.h) are formatted by app_log_vprintf() in the
 *   caller's context: "[HH:MM:SS.mmm,uuu] <lvl> module: message\n" with the
 *   time of the call from nano_mono_us().  Zephyr's deferred mode takes the
 *   time at the call too but formats later in its logging thread.
 * - The RAM log is also the syslog channel (Zephyr: printk goes into the
 *   log as raw strings, CONFIG_LOG_PRINTK).  syslog() output is stored
 *   as is, with no prefix; with CONFIG_SYSLOG_CRLF each '\n' becomes "\r\n",
 *   as Zephyr's log_output does for printk strings (cr_out_func()).
 */

#include <nuttx/config.h>

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/syslog/syslog.h>

#include <arch/board/nano_timer.h>

#include "app_log.h"
#include "log_ram.h"
#include "log_ring.h"

#if LOG_RAM_BUFFER_SIZE != LOG_RAM_SIZE
#  error "log_ram.h and log_ring.h disagree on the size"
#endif

#ifndef CONFIG_SYSLOG_REGISTER
#  error "CONFIG_NANO_LOG_RAM needs CONFIG_SYSLOG_REGISTER"
#endif

/* The linker leaves this region untouched during normal startup. */

static struct retained_log g_retained locate_data(".noinit");

/* Bytes pushed out of the buffer since boot.  Protected by the lock. */

static uint64_t g_dropped;

static void log_ram_store(FAR const char *data, size_t length)
{
  irqstate_t flags = up_irq_save();

  log_ring_store(&g_retained, &g_dropped, data, length);
  up_irq_restore(flags);
}

/* syslog channel ------------------------------------------------------- */

static int log_ram_putc(FAR syslog_channel_t *channel, int ch)
{
  char c = (char)ch;

  UNUSED(channel);
  log_ram_store(&c, 1);
  return ch;
}

static ssize_t log_ram_write(FAR syslog_channel_t *channel,
                             FAR const char *buffer, size_t buflen)
{
  UNUSED(channel);
  log_ram_store(buffer, buflen);
  return (ssize_t)buflen;
}

static const struct syslog_channel_ops_s g_log_ram_channel_ops =
{
  .sc_putc        = log_ram_putc,
  .sc_force       = log_ram_putc,
  .sc_flush       = NULL,
  .sc_write       = log_ram_write,
  .sc_write_force = log_ram_write,
  .sc_close       = NULL,
};

/* sc_state 0: CRLF conversion on (see the file comment). */

static syslog_channel_t g_log_ram_channel =
{
  .sc_ops = &g_log_ram_channel_ops,
};

/* Public functions ----------------------------------------------------- */

void log_ram_initialize(void)
{
  irqstate_t flags = up_irq_save();

  log_ring_validate(&g_retained);
  up_irq_restore(flags);

  /* With CONFIG_SYSLOG_MAX_CHANNELS 1 this replaces channel 0. */

  syslog_channel_register(&g_log_ram_channel);
}

size_t log_ram_piece(FAR uint64_t *position, FAR bool *first,
                     FAR char *piece, size_t max)
{
  irqstate_t flags = up_irq_save();
  size_t n = log_ring_piece(&g_retained, g_dropped, position, first,
                            piece, max);

  up_irq_restore(flags);
  return n;
}

void app_log_vprintf(int level, FAR const char *module, FAR const char *func,
                     FAR const char *fmt, va_list ap)
{
  char line[APP_LOG_LINE_MAX];
  uint64_t now = nano_mono_us();
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
  log_ram_store(line, length);
}

void app_log_printf(int level, FAR const char *module, FAR const char *func,
                    FAR const char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  app_log_vprintf(level, module, func, fmt, ap);
  va_end(ap);
}

bool app_log_ratelimit_pass(FAR uint32_t *last_ms, FAR uint32_t *skipped,
                            uint32_t rate_ms, FAR uint32_t *skipped_out)
{
  /* Zephyr: k_uptime_get_32(), milliseconds since boot. */

  uint32_t now_ms = (uint32_t)(nano_mono_us() / 1000U);
  irqstate_t flags = up_irq_save();
  bool pass = log_ring_ratelimit(last_ms, skipped, now_ms, rate_ms,
                                 skipped_out);

  up_irq_restore(flags);
  return pass;
}
