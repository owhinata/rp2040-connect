/* SPDX-License-Identifier: MIT */
/* The shell engine's part of <zephyr/shell/shell.h> of the Zephyr
 * compatibility layer (common/compat/include/zephyr/shell/shell.h, which includes
 * this file): struct shell, a shell backend.  In the engine a backend is a
 * console, the terminal of one shell session (shell_console.h, which holds
 * the struct): it lives as long as the system, as Zephyr's shell instances
 * do, and the output to it from any thread goes to that terminal
 * (zshell_os_impl.h).  NuttX's is common/compat/port/nuttx/zephyr_os_shell.h.
 */

#ifndef COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_SHELL_H
#define COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_SHELL_H

/* Zephyr's <zephyr/shell/shell.h> includes <zephyr/sys/sys_getopt.h> with
 * CONFIG_SHELL_GETOPT (the Zephyr build has it): the commands copied from
 * Zephyr unchanged (common/compat/subsys/net/lib/wifi_credentials/
 * wifi_credentials_shell.c) use sys_getopt_long() through it.  The build's
 * getopt state is per session (getopt_state.c).
 */
#include <zephyr/sys/sys_getopt.h>

struct shell_console;
struct shell_static_entry;

struct shell {
	struct shell_console *console;
	/* The command that runs on this session (its handler), for
	 * shell_help() (Zephyr: the shell context's active_cmd).
	 */
	const struct shell_static_entry *active_cmd;
};

#endif /* COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_SHELL_H */
