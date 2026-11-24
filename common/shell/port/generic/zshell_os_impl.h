/* SPDX-License-Identifier: MIT */
/* The shell engine's part of the Zephyr shell's command handling
 * (common/shell/zshell.c, which includes this file first through
 * common/shell/port/zshell_os.h): the shell backends.  NuttX's is
 * common/shell/port/nuttx/zshell_os_impl.h.
 *
 * The backend of a command is the console its thread is bound to
 * (shell_console.h): the serial session's thread, later the Telnet
 * session's, runs the commands of its session itself, as Zephyr's shell
 * thread does.  Output to a backend goes to that console's terminal from
 * any thread: from the command itself it is what the command's stdout and
 * stderr are as well (its thread is bound to the console, and the OS's
 * stdio keeps nothing buffered: the console transport, shell_serial_os.h),
 * so it comes in order with what the command prints with printf(); from
 * another thread (the sensor shell's processing thread) it goes straight to
 * the terminal, and is dropped while the session has none (the serial port
 * closed, the Telnet client gone).
 * Zephyr prints SHELL_WARNING and SHELL_ERROR in colors; the port does not
 * (the NuttX build neither).
 */

#ifndef COMMON_SHELL_PORT_GENERIC_ZSHELL_OS_IMPL_H
#define COMMON_SHELL_PORT_GENERIC_ZSHELL_OS_IMPL_H

#include <stdarg.h>
#include <stddef.h>

#include <zephyr/shell/shell.h>

#include "shell_console.h"

static struct shell *zshell_os_backend_get(void)
{
	struct shell_console *con = shell_console_bound();

	return con != NULL ? &con->sh : NULL;
}

void z_shell_backend_vfprintf(const struct shell *sh, enum shell_vt100_color color, const char *fmt,
			      va_list args)
{
	(void)color;
	shell_console_vprintf(sh->console, fmt, args);
}

/* Nothing to mark: the command's own output and that of other tasks go the
 * same way.  The getopt state is reset before every command
 * (common/compat/shell_engine/shell_engine.c).
 */
static inline void zshell_os_cmd_enter(struct shell *sh)
{
	(void)sh;
}

static inline void zshell_os_cmd_exit(struct shell *sh)
{
	(void)sh;
}

/* The running command, for shell_help() (common/shell/port/zshell_os.h): kept in
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

#endif /* COMMON_SHELL_PORT_GENERIC_ZSHELL_OS_IMPL_H */
