/* SPDX-License-Identifier: MIT */
/* The split of a command line into words, as Zephyr's shell does it
 * (shell_argv.c, from subsys/shell/shell_utils.c): words are separated by
 * white space; '\'' and '"' quote, '\\' escapes the next character, and
 * inside quotes "\0ooo" (octal) and "\xhh" (hexadecimal) are bytes.  The
 * line is changed in place.
 */

#ifndef COMMON_COMPAT_INTERNAL_SHELL_ARGV_H
#define COMMON_COMPAT_INTERNAL_SHELL_ARGV_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Splits CMD into at most MAX_ARGC words in ARGV (*ARGC of them; when
 * there are max_argc, the rest of the line stays in the last one's
 * buffer unsplit).  Returns the quote that is not closed at the end of the
 * line, or 0 (Zephyr: "not terminated: %c").
 */
char z_shell_make_argv(size_t *argc, const char **argv, char *cmd, uint8_t max_argc);

#endif /* COMMON_COMPAT_INTERNAL_SHELL_ARGV_H */
