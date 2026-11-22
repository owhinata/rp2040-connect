/* SPDX-License-Identifier: MIT */
/* The commands of the FreeRTOS build's shell and their execution
 * (nano_shell.h).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_getopt.h>

#include "nano_shell.h"
#include "shell_argv.h"
#include "subcmd.h"

/* Zephyr's SHELL_MSG_CMD_NOT_FOUND. */
#define MSG_CMD_NOT_FOUND ": command not found"

static const char *cmd_name(const struct nano_shell_cmd *cmd)
{
	return cmd->root != NULL ? cmd->root->syntax : cmd->name;
}

static const char *cmd_help(const struct nano_shell_cmd *cmd)
{
	const char *help = cmd->root != NULL ? cmd->root->help : cmd->help;

	if (shell_help_is_structured(help)) {
		return ((const struct shell_cmd_help *)help)->description;
	}
	return help;
}

static const struct nano_shell_cmd *find(const char *name)
{
	for (size_t i = 0; i < nano_shell_cmd_count; i++) {
		if (strcmp(cmd_name(&nano_shell_cmds[i]), name) == 0) {
			return &nano_shell_cmds[i];
		}
	}
	return NULL;
}

/* z_shell_help_subcmd_print() for the root commands, with
 * help_item_print()'s layout (common/shell/zshell.c).
 */
void nano_shell_help(void)
{
	size_t longest = 0;

	for (size_t i = 0; i < nano_shell_cmd_count; i++) {
		size_t n = strlen(cmd_name(&nano_shell_cmds[i]));

		longest = n > longest ? n : longest;
	}
	printf("Available commands:\n");
	for (size_t i = 0; i < nano_shell_cmd_count; i++) {
		const char *help = cmd_help(&nano_shell_cmds[i]);

		printf("  %-*s", (int)longest, cmd_name(&nano_shell_cmds[i]));
		if (help == NULL) {
			putchar('\n');
			continue;
		}
		printf("  : ");
		subcmd_text_print(help, 2 * 2 + longest + 2);
	}
	fflush(stdout);
}

int nano_shell_execute(char *line)
{
	const char *argv[NANO_SHELL_ARGC_MAX + 2];
	const struct nano_shell_cmd *cmd;
	size_t argc;
	char quote;

	quote = z_shell_make_argv(&argc, argv, line, NANO_SHELL_ARGC_MAX + 1);
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
