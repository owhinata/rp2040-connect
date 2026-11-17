/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the Zephyr shell's command handling
 * (common/shell/zshell.c, which includes this file first through
 * common/shell/zshell_os.h): the shell backends.  NuttX's is
 * nuttx/apps/subcmd/zshell_os_impl.h.
 *
 * The backend of a command is the console its task is bound to
 * (nano_console.h): the USB console's task, later the Telnet task, runs
 * the commands of its session itself, as Zephyr's shell thread does.
 * Output to a backend goes to that console's terminal from any task:
 * from the command itself it is what the command's stdout and stderr are
 * as well (its task is bound to the console, and the stdio of the Pico SDK
 * and newlib keep nothing buffered, nano_stdio.c and
 * freertos/drivers/libc/stdio_wrap.c), so it comes in order with what the
 * command prints with printf(); from another task (the sensor shell's
 * processing thread) it goes straight to the terminal, and is dropped while
 * the session has none (the USB port closed, the Telnet client gone).
 * Zephyr prints SHELL_WARNING and SHELL_ERROR in colors; the port does not
 * (the NuttX build neither).
 */

#ifndef FREERTOS_DRIVERS_SHELL_ZSHELL_OS_IMPL_H
#define FREERTOS_DRIVERS_SHELL_ZSHELL_OS_IMPL_H

#include <stdarg.h>
#include <stddef.h>

#include <zephyr/shell/shell.h>

#include "nano_console.h"

static struct shell *zshell_os_backend_get(void)
{
	struct nano_console *con = nano_console_bound();

	return con != NULL ? &con->sh : NULL;
}

void z_shell_backend_vfprintf(const struct shell *sh, enum shell_vt100_color color, const char *fmt,
			      va_list args)
{
	(void)color;
	nano_console_vprintf(sh->console, fmt, args);
}

/* Nothing to mark: the command's own output and that of other tasks go the
 * same way.  The getopt state is reset before every command
 * (nano_shell.c).
 */
static inline void zshell_os_cmd_enter(struct shell *sh)
{
	(void)sh;
}

static inline void zshell_os_cmd_exit(struct shell *sh)
{
	(void)sh;
}

/* The running command, for shell_help() (common/shell/zshell_os.h): kept in
 * the session's backend, which only the session's task runs commands on.
 */
#define ZSHELL_OS_ACTIVE_CMD 1

static inline void zshell_os_active_cmd_set(struct shell *sh, const struct shell_static_entry *cmd)
{
	sh->active_cmd = cmd;
}

static inline const struct shell_static_entry *zshell_os_active_cmd_get(const struct shell *sh)
{
	return sh->active_cmd;
}

#endif /* FREERTOS_DRIVERS_SHELL_ZSHELL_OS_IMPL_H */
