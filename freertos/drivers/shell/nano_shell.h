/* SPDX-License-Identifier: MIT */
/* The commands of the FreeRTOS build's shell and their execution.
 *
 * Commands are written as in Zephyr: a handler
 *
 *   int handler(const struct shell *sh, size_t argc, char **argv)
 *
 * registered with SHELL_CMD_REGISTER() or SHELL_CMD_ARG_REGISTER() of
 * <zephyr/shell/shell.h>, with its subcommand sets, in the shared sources
 * (common/) and in the FreeRTOS build's.  The build defines
 * Z_SHELL_CMD_ROOT_GLOBAL for every source, so a registration makes the
 * global root entry z_shell_root_<syntax>; the application lists the roots
 * in its table, nano_shell_cmds[] (sorted by name, as Zephyr sorts its
 * root commands), with NANO_SHELL_CMD(syntax).  A command written for
 * NuttX's builtin applications on common/shell/subcmd.h (an
 * int main(int argc, char **argv) that calls subcmd_main()) is listed with
 * NANO_SHELL_MAIN(name, help, function).
 *
 * nano_shell_execute() runs one line in the calling task (a session's
 * task, bound to its console): the words are split as Zephyr's shell splits
 * them (shell_argv.h), the first one names the command, and the root's
 * words are matched, checked and run by common/shell/zshell.c with Zephyr's
 * help, wildcards and argument count check.  Its output goes to the
 * session (nano_console.h).
 */

#ifndef FREERTOS_DRIVERS_SHELL_NANO_SHELL_H
#define FREERTOS_DRIVERS_SHELL_NANO_SHELL_H

#include <stddef.h>

#include <zephyr/shell/shell.h>

struct nano_shell_cmd {
	const struct shell_static_entry *root; /* or NULL for main */
	const char *name;                      /* main's */
	const char *help;                      /* main's */
	int (*main)(int argc, char **argv);
};

#define NANO_SHELL_CMD(syntax)                                                                     \
	{                                                                                          \
		.root = &z_shell_root_##syntax,                                                    \
	}

#define NANO_SHELL_MAIN(_name, _help, _main)                                                       \
	{                                                                                          \
		.name = (_name), .help = (_help), .main = (_main),                                 \
	}

/* Declares the root of a command registered in another source. */
#define NANO_SHELL_CMD_DECLARE(syntax) extern const struct shell_static_entry z_shell_root_##syntax

/* The application's table. */
extern const struct nano_shell_cmd nano_shell_cmds[];
extern const size_t nano_shell_cmd_count;

/* Words of a command line, with the command (Zephyr: CONFIG_SHELL_ARGC_MAX
 * 30); one more is split off so that zshell.c sees a longer line and
 * reports "Too many arguments in the command."
 */
#define NANO_SHELL_ARGC_MAX 30

/* Runs LINE (changed in place) in the calling task.  Returns the
 * command's exit status (0 when its handler returned 0, else 1), 0 for an
 * empty line, 1 for a quote that is not closed and a name that is no
 * command (Zephyr's messages, on stderr).
 */
int nano_shell_execute(char *line);

/* The help of the table, as "help" prints it (Zephyr's "Available
 * commands:" list).
 */
void nano_shell_help(void);

#endif /* FREERTOS_DRIVERS_SHELL_NANO_SHELL_H */
