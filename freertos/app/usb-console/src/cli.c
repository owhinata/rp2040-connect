/* SPDX-License-Identifier: MIT */
/* The command line of the USB console (cli.h). */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "cli.h"

#define CHAR_CTRL_C    0x03
#define CHAR_BACKSPACE 0x08
#define CHAR_DELETE    0x7f

/* Width of "name arguments" in the output of "help". */
#define USAGE_WIDTH 16

static void write_string(const struct cli *cli, const char *text)
{
	cli->write(text, strlen(text));
}

static bool is_blank(char c)
{
	return c == ' ' || c == '\t';
}

void cli_init(struct cli *cli, const struct cli_command *commands, size_t command_count,
	      const char *prompt, void (*write)(const char *text, size_t len))
{
	memset(cli, 0, sizeof(*cli));
	cli->commands = commands;
	cli->command_count = command_count;
	cli->prompt = prompt;
	cli->write = write;
}

void cli_prompt(struct cli *cli)
{
	write_string(cli, cli->prompt);
	cli->write(cli->line, cli->len);
}

int cli_split(char *line, char **argv, int max)
{
	int argc = 0;
	char *p = line;

	for (;;) {
		while (is_blank(*p)) {
			p++;
		}
		if (*p == '\0') {
			return argc;
		}
		if (argc == max) {
			return -1;
		}
		argv[argc++] = p;
		while (*p != '\0' && !is_blank(*p)) {
			p++;
		}
		if (*p != '\0') {
			*p++ = '\0';
		}
	}
}

void cli_help(const struct cli *cli)
{
	char usage[32];
	char text[96];

	for (size_t i = 0; i < cli->command_count; i++) {
		const struct cli_command *command = &cli->commands[i];

		snprintf(usage, sizeof(usage), "%s %s", command->name, command->args);
		snprintf(text, sizeof(text), "  %-*s %s\r\n", USAGE_WIDTH, usage, command->help);
		write_string(cli, text);
	}
}

static void run_line(struct cli *cli)
{
	char *argv[CLI_ARGS_MAX];
	char text[64];
	int argc = cli_split(cli->line, argv, CLI_ARGS_MAX);
	int ret;

	if (argc == 0) {
		return;
	}
	if (argc < 0) {
		write_string(cli, "Too many arguments\r\n");
		return;
	}
	for (size_t i = 0; i < cli->command_count; i++) {
		if (strcmp(argv[0], cli->commands[i].name) != 0) {
			continue;
		}
		ret = cli->commands[i].handler(argc, argv);
		if (ret != 0) {
			snprintf(text, sizeof(text), "%s: error %d\r\n", argv[0], ret);
			write_string(cli, text);
		}
		return;
	}
	write_string(cli, argv[0]);
	write_string(cli, ": command not found (try \"help\")\r\n");
}

void cli_input(struct cli *cli, char c)
{
	char last = cli->last;

	cli->last = c;
	switch (c) {
	case '\n':
		if (last == '\r') {
			break; /* the LF of a CR LF */
		}
		__attribute__((fallthrough));
	case '\r':
		write_string(cli, "\r\n");
		cli->line[cli->len] = '\0';
		run_line(cli);
		cli->len = 0;
		cli_prompt(cli);
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
		cli_prompt(cli);
		break;
	default:
		if (c >= ' ' && c < CHAR_DELETE && cli->len < CLI_LINE_MAX) {
			cli->line[cli->len++] = c;
			cli->write(&c, 1);
		}
		break;
	}
}
