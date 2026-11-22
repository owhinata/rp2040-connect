/* SPDX-License-Identifier: MIT */
/* The line editor of a shell session (cli.h). */
#include <stdbool.h>
#include <string.h>
#include "cli.h"

#define CHAR_CTRL_C    0x03
#define CHAR_BACKSPACE 0x08
#define CHAR_ESC       0x1b
#define CHAR_DELETE    0x7f

/* The states of cli->esc. */
enum {
	ESC_NONE,
	ESC_START, /* after ESC */
	ESC_CSI,   /* after ESC [ or ESC O, before the final byte */
};

/* VT100: erase from the cursor to the end of the line. */
#define VT100_ERASE_EOL "\x1b[K"

static void write_string(const struct cli *cli, const char *text)
{
	cli->write(cli->ctx, text, strlen(text));
}

void cli_init(struct cli *cli, const char *prompt,
	      void (*write)(void *ctx, const char *text, size_t len),
	      void (*execute)(void *ctx, char *line), void *ctx)
{
	memset(cli, 0, sizeof(*cli));
	cli->prompt = prompt;
	cli->write = write;
	cli->execute = execute;
	cli->ctx = ctx;
}

void cli_prompt(struct cli *cli)
{
	write_string(cli, cli->prompt);
	cli->write(cli->ctx, cli->line, cli->len);
}

/* The start of the n-th newest line of the history (n >= 1), or NULL. */
static const char *history_get(const struct cli *cli, size_t n)
{
	size_t end = cli->history_len;

	while (end > 0) {
		size_t start = end - 1; /* at its null */

		while (start > 0 && cli->history[start - 1] != '\0') {
			start--;
		}
		if (--n == 0) {
			return &cli->history[start];
		}
		end = start;
	}
	return NULL;
}

static void history_add(struct cli *cli)
{
	size_t size = cli->len + 1;
	const char *newest = history_get(cli, 1);

	if (cli->len == 0 || size > sizeof(cli->history) ||
	    (newest != NULL && strcmp(newest, cli->line) == 0)) {
		return;
	}
	/* Push the oldest lines out until the new one fits. */
	while (cli->history_len + size > sizeof(cli->history)) {
		size_t first = strlen(cli->history) + 1;

		memmove(cli->history, cli->history + first, cli->history_len - first);
		cli->history_len -= first;
	}
	memcpy(cli->history + cli->history_len, cli->line, size);
	cli->history_len += size;
}

/* Replaces the line on the terminal and in the buffer by TEXT. */
static void line_replace(struct cli *cli, const char *text)
{
	size_t len = strlen(text);

	memcpy(cli->line, text, len);
	cli->len = len;
	write_string(cli, "\r");
	cli_prompt(cli);
	write_string(cli, VT100_ERASE_EOL);
}

static void history_step(struct cli *cli, bool older)
{
	const char *text;

	if (older) {
		text = history_get(cli, cli->history_pos + 1);
		if (text == NULL) {
			return;
		}
		cli->history_pos++;
	} else {
		if (cli->history_pos == 0) {
			return;
		}
		cli->history_pos--;
		text = cli->history_pos > 0 ? history_get(cli, cli->history_pos) : "";
	}
	line_replace(cli, text);
}

/* Returns true if C belonged to an escape sequence. */
static bool escape_input(struct cli *cli, char c)
{
	switch (cli->esc) {
	case ESC_START:
		/* ESC [ (CSI) or ESC O (SS3, the arrows in application mode);
		 * anything else ends the sequence (Alt + key).
		 */
		cli->esc = (c == '[' || c == 'O') ? ESC_CSI : ESC_NONE;
		return true;
	case ESC_CSI:
		/* Parameters and intermediates until the final byte. */
		if (c >= 0x40 && c <= 0x7e) {
			cli->esc = ESC_NONE;
			if (c == 'A') {
				history_step(cli, true);
			} else if (c == 'B') {
				history_step(cli, false);
			}
		} else if (c < 0x20 || c > 0x3f) {
			cli->esc = ESC_NONE; /* not a sequence after all */
		}
		return true;
	default:
		if (c == CHAR_ESC) {
			cli->esc = ESC_START;
			return true;
		}
		return false;
	}
}

static void line_end(struct cli *cli)
{
	write_string(cli, "\r\n");
	cli->line[cli->len] = '\0';
	history_add(cli);
	cli->history_pos = 0;
	cli->execute(cli->ctx, cli->line);
	cli->len = 0;
	cli_prompt(cli);
}

void cli_input(struct cli *cli, char c)
{
	char last = cli->last;

	cli->last = c;
	if (escape_input(cli, c)) {
		return;
	}
	switch (c) {
	case '\n':
		if (last == '\r') {
			break; /* the LF of a CR LF */
		}
		__attribute__((fallthrough));
	case '\r':
		line_end(cli);
		break;
	case CHAR_BACKSPACE:
	case CHAR_DELETE:
		if (cli->len > 0) {
			cli->len--;
			write_string(cli, "\b \b");
		}
		break;
	case CHAR_CTRL_C:
		write_string(cli, "^C\r\n");
		cli->len = 0;
		cli->history_pos = 0;
		cli_prompt(cli);
		break;
	default:
		if (c >= ' ' && c < CHAR_DELETE && cli->len < CLI_LINE_MAX) {
			cli->line[cli->len++] = c;
			cli->write(cli->ctx, &c, 1);
		}
		break;
	}
}
