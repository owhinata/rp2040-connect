/* SPDX-License-Identifier: MIT */
/* The commands of the shell engine and their execution (shell_engine.h). */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_getopt.h>

#include "shell_engine.h"
#include "shell_argv.h"
#include "subcmd.h"

/* Zephyr's SHELL_MSG_CMD_NOT_FOUND. */
#define MSG_CMD_NOT_FOUND ": command not found"

static const char *cmd_name(const struct shell_engine_cmd *cmd)
{
	return cmd->root != NULL ? cmd->root->syntax : cmd->name;
}

static const char *cmd_help_text(const struct shell_engine_cmd *cmd)
{
	const char *help = cmd->root != NULL ? cmd->root->help : cmd->help;

	if (shell_help_is_structured(help)) {
		return ((const struct shell_cmd_help *)help)->description;
	}
	return help;
}

static const struct shell_engine_cmd *find(const char *name)
{
	for (size_t i = 0; i < shell_engine_cmd_count; i++) {
		if (strcmp(cmd_name(&shell_engine_cmds[i]), name) == 0) {
			return &shell_engine_cmds[i];
		}
	}
	return NULL;
}

/* z_shell_help_subcmd_print() for the root commands, with
 * help_item_print()'s layout (common/shell/zshell.c).
 */
void shell_engine_help(void)
{
	size_t longest = 0;

	for (size_t i = 0; i < shell_engine_cmd_count; i++) {
		size_t n = strlen(cmd_name(&shell_engine_cmds[i]));

		longest = n > longest ? n : longest;
	}
	printf("Available commands:\n");
	for (size_t i = 0; i < shell_engine_cmd_count; i++) {
		const char *help = cmd_help_text(&shell_engine_cmds[i]);

		printf("  %-*s", (int)longest, cmd_name(&shell_engine_cmds[i]));
		if (help == NULL) {
			putchar('\n');
			continue;
		}
		printf("  : ");
		subcmd_text_print(help, 2 * 2 + longest + 2);
	}
	fflush(stdout);
}

/* Zephyr's "help" without arguments (subsys/shell/shell_cmds.c). */
static int cmd_help(const struct shell *sh, size_t argc, char **argv)
{
	(void)sh;
	(void)argc;
	(void)argv;

	shell_engine_help();
	return 0;
}

SHELL_CMD_ARG_REGISTER(help, NULL, "Prints the help message.", cmd_help, 1, 0);

int shell_engine_execute(char *line)
{
	const char *argv[SHELL_ENGINE_ARGC_MAX + 2];
	const struct shell_engine_cmd *cmd;
	size_t argc;
	char quote;

	quote = z_shell_make_argv(&argc, argv, line, SHELL_ENGINE_ARGC_MAX + 1);
	if (argc == 0) {
		return 0;
	}
	if (quote != 0) {
		fprintf(stderr, "not terminated: %c\n", quote);
		return 1;
	}
	argv[argc] = NULL;

	cmd = find(argv[0]);
	if (cmd == NULL) {
		fprintf(stderr, "%s%s\n", argv[0], MSG_CMD_NOT_FOUND);
		return 1;
	}

	/* Zephyr's shell: before each command (CONFIG_SHELL_GETOPT). */
	sys_getopt_init();

	if (cmd->root != NULL) {
		return z_shell_cmd_main(cmd->root, (int)argc, (char **)argv);
	}
	return cmd->main((int)argc, (char **)argv);
}
