/* SPDX-License-Identifier: Apache-2.0 */
/* Subcommand dispatch for the port's commands ("kv", "nina", and later
 * "imu", "ntp", "wifi", "i2c_dma"), with the output of the Zephyr shell
 * for a command registered with SHELL_CMD_REGISTER() and a static
 * subcommand set (zephyr/subsys/shell/shell.c execute(), exec_cmd(),
 * cmd_precheck(); shell_help.c; shell_wildcard.c), as configured in the
 * Zephyr build (CONFIG_SHELL_HELP=y, CONFIG_SHELL_HELP_ON_WRONG_ARGUMENT_COUNT=y,
 * CONFIG_SHELL_WILDCARD=y, CONFIG_SHELL_HELP_OPT_PARSE off):
 *
 * - The words after the command are matched against the subcommand sets
 *   level by level.  The deepest matched entry that has a handler runs,
 *   with argv[0] its own name and the rest of the words as arguments.
 * - No handler on the way (for example "kv" alone, or "kv foo"): the help
 *   of the deepest matched entry: "<name> - <help>", then "Subcommands:"
 *   and one line per subcommand ("  <name padded to the longest>  : <help>").
 * - Argument count: an entry made with SUBCMD() is not checked (Zephyr's
 *   SHELL_CMD(): mandatory 0).  One made with SUBCMD_ARG(..., mand, opt)
 *   needs mand <= argc <= mand + opt (argc counts the entry's own name, as
 *   SHELL_CMD_ARG()); SUBCMD_OPT_ANY as opt is SHELL_OPT_ARG_CHECK_SKIP.
 *   Otherwise "<name>: wrong parameter count" (on stderr) and the entry's
 *   help.
 * - Wildcards: a '*' or '?' in the word right after the deepest matched
 *   entry makes the Zephyr shell expand it against that entry's
 *   subcommands.  When nothing matches (always, for an entry without
 *   subcommands), the shell stops reading the words there, so the handler
 *   gets fewer arguments ("kv get a*" is a wrong parameter count, "nina
 *   version *" runs "nina version").  This is reproduced.  When the word
 *   does match subcommands, the Zephyr shell substitutes the matching names
 *   and runs the handler found before the word with them; that case is not
 *   reproduced: it is handled as a word that names no subcommand.
 * - "-h" and "--help" are ordinary words (SHELL_HELP_OPT_PARSE is off).
 *
 * Help texts are laid out as the Zephyr shell does (formatted_text_print()
 * of shell_help.c): a new line at each '\n' and before a word that would
 * pass the 80th column (CONFIG_SHELL_DEFAULT_TERMINAL_WIDTH; the "resize"
 * command is not provided), continued at the column where the text began;
 * the Zephyr shell moves the cursor there with a VT100 sequence, this
 * prints spaces.  Errors go to stderr,
 * which is the same console; stdout is flushed before returning.  The
 * return value is the exit status: EXIT_SUCCESS when the handler returned
 * 0, EXIT_FAILURE when it returned anything else or when only the help or
 * an error was printed (Zephyr: SHELL_CMD_HELP_PRINTED, -EINVAL, -ENOEXEC,
 * seen only with the "retval" command).
 */

#ifndef __NUTTX_APPS_SUBCMD_SUBCMD_H
#define __NUTTX_APPS_SUBCMD_SUBCMD_H

#include <stdint.h>

struct subcmd
{
  const char *syntax;                  /* NULL ends a set */
  const struct subcmd *subcmds;        /* NULL-terminated set, or NULL */
  const char *help;
  int (*handler)(int argc, char **argv);
  uint8_t mandatory;                   /* 0: argument count not checked */
  uint8_t optional;
};

#define SUBCMD_OPT_ANY 0xff

/* SHELL_CMD(syntax, subcmd, help, handler) */

#define SUBCMD(syntax, subcmds, help, handler) \
  { #syntax, subcmds, help, handler, 0, 0 }

/* SHELL_CMD_ARG(syntax, subcmd, help, handler, mandatory, optional) */

#define SUBCMD_ARG(syntax, subcmds, help, handler, mand, opt) \
  { #syntax, subcmds, help, handler, mand, opt }

#define SUBCMD_SET_END { NULL, NULL, NULL, NULL, 0, 0 }

/* Run the command ROOT (SHELL_CMD_REGISTER(syntax, subcmds, help,
 * handler)) with the words of main(): ARGV[0] is the command name.
 */

int subcmd_main(const struct subcmd *root, int argc, char **argv);

/* The help of ENTRY as subcmd_main() prints it ("<name> - <help>" and its
 * subcommands): Zephyr's shell_help(), for a handler that prints the help
 * of its own entry.
 */

void subcmd_help(const struct subcmd *entry);

#endif /* __NUTTX_APPS_SUBCMD_SUBCMD_H */
