/* SPDX-License-Identifier: MIT */
/* What the Zephyr shell's command handling (zshell.c here) needs from the
 * OS: the shell backends.  zshell.c includes this header before anything
 * else; it includes the OS's part, "zshell_os_impl.h", then declares the
 * functions that part must define, so a different one does not compile.
 * The OS's part defines them as static (inline) functions, so that they are
 * compiled into zshell.c as if it called the OS directly.
 *
 * zshell.c itself is OS-independent: it splits nothing (it gets the words
 * of a command line), matches them against the command's subcommand sets,
 * expands wildcards (fnmatch()), checks the argument count, prints the help
 * and the errors, and calls the handler with the backend.  It prints the
 * help and its errors with printf() and fprintf(stderr) and flushes stdout,
 * so the OS has to make stdout and stderr of the task that runs a command
 * the terminal of that command's session (on NuttX each command is a task
 * with its own stdio).  It allocates the state of one command line (about
 * 1.2 KB) with calloc() and frees it before it returns.
 *
 * Each OS's build puts these files on the include path of zshell.c (the
 * NuttX build's are common/shell/port/nuttx/zshell_os_impl.h and
 * common/compat/port/nuttx/zephyr_os_shell.h):
 *
 *   zephyr_os_shell.h   struct shell (read by <zephyr/shell/shell.h>):
 *                       one backend, the terminal of a shell session.  It
 *                       outlives the command: a thread may keep the
 *                       pointer and write to the session later
 *   zshell_os_impl.h    the OS's headers, the functions below and
 *                       z_shell_backend_vfprintf() (<zephyr/shell/shell.h>):
 *
 *   void z_shell_backend_vfprintf(const struct shell *sh,
 *                                 enum shell_vt100_color color,
 *                                 const char *fmt, va_list args)
 *       shell_fprintf() and the macros on it, for sh != NULL.  From the
 *       command that runs on sh (between zshell_os_cmd_enter() and
 *       zshell_os_cmd_exit(), in its task): fflush(stdout), then the text
 *       on its stdout, or on its stderr for SHELL_WARNING and SHELL_ERROR,
 *       so that it comes in order with what the command prints itself.
 *       From any other thread, or after the command has returned: the text
 *       straight to the session's terminal, dropped if the session has no
 *       terminal now (a Telnet client that has gone); a line goes out in
 *       one piece where it can.  Any context but an interrupt handler.
 *
 * The functions declared below.
 */

#ifndef COMMON_SHELL_PORT_ZSHELL_OS_H
#define COMMON_SHELL_PORT_ZSHELL_OS_H

#include "zshell_os_impl.h"

#include <zephyr/shell/shell.h>

/* The backend of the session whose command the calling task runs (the
 * session of z_shell_cmd_main()'s caller), or NULL when there is none or
 * none can be made: the handler then gets NULL, and its shell_print()
 * calls go to its stdout and stderr.  Called once per command line, in the
 * task that runs it.
 */

static struct shell *zshell_os_backend_get(void);

/* Around the handler's call, in the task that runs it: from
 * zshell_os_cmd_enter(sh) to zshell_os_cmd_exit(sh), the output to sh from
 * this task is the command's own (z_shell_backend_vfprintf()).
 */

static inline void zshell_os_cmd_enter(struct shell *sh);
static inline void zshell_os_cmd_exit(struct shell *sh);

/* Optional: an OS whose zshell_os_impl.h defines ZSHELL_OS_ACTIVE_CMD
 * keeps the command that runs on sh (Zephyr: sh->ctx->active_cmd), set
 * right before its handler and cleared (NULL) after it, and gets
 * shell_help() (<zephyr/shell/shell.h>), which prints that command's help.
 * Without it, zshell.c has neither (the NuttX build's commands that print
 * their help call subcmd_help()).
 */

#ifdef ZSHELL_OS_ACTIVE_CMD
static inline void zshell_os_active_cmd_set(struct shell *sh,
                                            const struct shell_static_entry
                                            *cmd);
static inline const struct shell_static_entry *
zshell_os_active_cmd_get(const struct shell *sh);
#endif

#endif /* COMMON_SHELL_PORT_ZSHELL_OS_H */
