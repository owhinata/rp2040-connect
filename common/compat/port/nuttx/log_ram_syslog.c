/* SPDX-License-Identifier: Apache-2.0 */
/* The RAM log (common/compat/log/log_ram.c) as the syslog channel (Zephyr: printk
 * goes into the log as raw strings, CONFIG_LOG_PRINTK).  syslog() output
 * is stored as is, with no prefix; with CONFIG_SYSLOG_CRLF each '\n'
 * becomes "\r\n", as Zephyr's log_output does for printk strings
 * (cr_out_func()).  Registered by log_ram_initialize() (log_ram_os_attach()
 * of common/compat/port/nuttx/log_ram_os_impl.h).
 */

#include <nuttx/config.h>

#include <stddef.h>
#include <sys/types.h>

#include <nuttx/compiler.h>
#include <nuttx/syslog/syslog.h>

#include "log_ram.h"
#include "log_ram_os_impl.h"

#ifndef CONFIG_SYSLOG_REGISTER
#  error "CONFIG_LOG_RAM needs CONFIG_SYSLOG_REGISTER"
#endif

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

void log_ram_syslog_register(void)
{
  /* With CONFIG_SYSLOG_MAX_CHANNELS 1 this replaces channel 0. */

  syslog_channel_register(&g_log_ram_channel);
}
