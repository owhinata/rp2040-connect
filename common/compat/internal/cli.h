/* SPDX-License-Identifier: MIT */
/* The line editor of a shell session.  cli.c uses neither the kernel nor
 * the SDK: the session's task feeds it the received characters and gives
 * it a function that writes to the terminal and one that runs a line.
 *
 * Editing: printable characters are appended and echoed; Backspace (0x08)
 * and Delete (0x7f) remove the last one; Ctrl-C drops the line ("^C" and a
 * new prompt); CR, LF or CR LF end it.  Up and Down (VT100 "ESC [ A",
 * "ESC [ B", or "ESC O A", "ESC O B") step through the lines entered
 * before (the last ones that fit CLI_HISTORY_SIZE bytes; a line equal to
 * the previous one, an empty one or a longer one is not kept); Down after
 * the newest gives an empty line.  Other escape sequences (Left, Right,
 * Home, ...) and the other control characters (Tab, ...) are ignored.
 * Characters beyond CLI_LINE_MAX are dropped.
 */

#ifndef COMMON_COMPAT_INTERNAL_CLI_H
#define COMMON_COMPAT_INTERNAL_CLI_H

#include <stddef.h>

/* Characters in a line, without the terminating null (Zephyr:
 * CONFIG_SHELL_CMD_BUFF_SIZE=512 with it).
 */
#define CLI_LINE_MAX     511
/* Lines entered before, with their nulls (Zephyr: CONFIG_SHELL_HISTORY_BUFFER=512). */
#define CLI_HISTORY_SIZE 512

struct cli {
	const char *prompt;
	/* Writes to the terminal. */
	void (*write)(void *ctx, const char *text, size_t len);
	/* Runs a line that has ended (changes it in place). */
	void (*execute)(void *ctx, char *line);
	void *ctx;
	char line[CLI_LINE_MAX + 1];
	size_t len;
	char last;         /* the previous character, to take CR LF as one end of line */
	unsigned char esc; /* where in an escape sequence */
	char history[CLI_HISTORY_SIZE]; /* lines with their nulls, the oldest first */
	size_t history_len;             /* bytes used */
	size_t history_pos;             /* 0: the line being typed, n: the n-th newest */
};

void cli_init(struct cli *cli, const char *prompt,
	      void (*write)(void *ctx, const char *text, size_t len),
	      void (*execute)(void *ctx, char *line), void *ctx);

/* Writes the prompt and what has been typed so far (after the terminal
 * connects, for example).
 */
void cli_prompt(struct cli *cli);

/* Takes one received character (see above); at the end of a non-empty or
 * empty line, runs it with execute, then writes the prompt.
 */
void cli_input(struct cli *cli, char c);

#endif /* COMMON_COMPAT_INTERNAL_CLI_H */
