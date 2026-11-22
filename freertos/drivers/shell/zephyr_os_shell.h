/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of <zephyr/shell/shell.h> of the Zephyr compatibility
 * layer (common/compat/zephyr/shell/shell.h, which includes this file):
 * struct shell, a shell backend.  On FreeRTOS a backend is a console, the
 * terminal of one shell session (shell_console.h, which holds the struct):
 * it lives as long as the system, as Zephyr's shell instances do, and the
 * output to it from any task goes to that terminal (zshell_os_impl.h).
 * NuttX's is nuttx/drivers/compat/zephyr_os_shell.h.
 */

#ifndef FREERTOS_DRIVERS_SHELL_ZEPHYR_OS_SHELL_H
#define FREERTOS_DRIVERS_SHELL_ZEPHYR_OS_SHELL_H

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

#endif /* FREERTOS_DRIVERS_SHELL_ZEPHYR_OS_SHELL_H */
