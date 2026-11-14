/* SPDX-License-Identifier: Apache-2.0 */
/* shell_print(), shell_warn() and shell_error() of Zephyr's
 * <zephyr/shell/shell.h>, for command handlers ported with their Zephyr
 * signature
 *
 *   int handler(const struct shell *sh, size_t argc, char **argv)
 *
 * and called from a builtin application of the port (registered with
 * nuttx/apps/subcmd, which reproduces the Zephyr shell's help and argument
 * checks).  The output goes where the port's other commands write it
 * (nuttx/apps/kv): shell_print() to stdout, shell_warn() and shell_error()
 * to stderr (Zephyr prints them in yellow and red), all the console of the
 * session that runs the command; each adds "\n", as Zephyr's.  The sh
 * argument is evaluated and not used (pass NULL).  The registration macros
 * (SHELL_CMD_REGISTER() and the subcommand sets) are not provided: the
 * application lists its subcommands with subcmd.h.  Part of the small
 * Zephyr compatibility layer of the port (nuttx/drivers/compat, see
 * nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SHELL_SHELL_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SHELL_SHELL_H

#include <stdio.h>

struct shell;

#define shell_print(sh, fmt, ...) \
  ((void)(sh), printf(fmt "\n", ##__VA_ARGS__))
#define shell_error(sh, fmt, ...) \
  ((void)(sh), fprintf(stderr, fmt "\n", ##__VA_ARGS__))
#define shell_warn(sh, fmt, ...) \
  ((void)(sh), fprintf(stderr, fmt "\n", ##__VA_ARGS__))

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SHELL_SHELL_H */
