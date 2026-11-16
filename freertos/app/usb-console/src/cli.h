/* SPDX-License-Identifier: MIT */
/* The command line of the USB console: a line editor and a table of commands.
 * cli.c does not use the kernel or the SDK; the caller feeds it the received
 * characters and gives it a function that writes.
 */

#ifndef CLI_H
#define CLI_H

#include <stddef.h>

#define CLI_LINE_MAX 128 /* characters in a line, without the terminating null */
#define CLI_ARGS_MAX 16  /* words in a line, with the command's name */

/* A command. argv[0] is its name; the handler returns 0 or a negative errno,
 * which the command line reports.
 */
struct cli_command {
	const char *name;
	const char *args; /* shown by "help" after the name; may be "" */
	const char *help;
	int (*handler)(int argc, char **argv);
};

struct cli {
	const struct cli_command *commands;
	size_t command_count;
	const char *prompt;
	void (*write)(const char *text, size_t len);
	char line[CLI_LINE_MAX + 1];
	size_t len;
	char last; /* the previous character, to take CR LF as one end of line */
};

void cli_init(struct cli *cli, const struct cli_command *commands, size_t command_count,
	      const char *prompt, void (*write)(const char *text, size_t len));

/* Writes the prompt and what has been typed so far (after the terminal
 * connects, for example).
 */
void cli_prompt(struct cli *cli);

/* Takes one received character: echoes it, edits the line (backspace, Ctrl-C
 * to drop the line) and at the end of the line runs the command.
 */
void cli_input(struct cli *cli, char c);

/* Writes one line per command: its name, arguments and description. */
void cli_help(const struct cli *cli);

/* Splits line in place into words separated by blanks. Returns the number of
 * words, or -1 if there are more than max.
 */
int cli_split(char *line, char **argv, int max);

#endif /* CLI_H */
