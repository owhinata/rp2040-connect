/* SPDX-License-Identifier: Apache-2.0 */
/* The "kv" command: cmd_set(), cmd_sethex(), cmd_get() and cmd_delete() of
 * zephyr/app/usb-console/src/kv_store.c, with the same syntax and output.
 *
 * The Zephyr shell itself printed some of the output; it is reproduced
 * here (zephyr/subsys/shell/shell.c, shell_help.c):
 * - "kv" alone, or with a word that is not one of its subcommands: the
 *   command's help (exec_cmd() -> shell_internal_help_print()),
 * - a subcommand with the wrong number of arguments (SHELL_CMD_ARG
 *   mandatory 3 or 2, optional 0): "<sub>: wrong parameter count" and the
 *   subcommand's help (cmd_precheck(), SHELL_HELP_ON_WRONG_ARGUMENT_COUNT),
 * - a '*' or '?' in the first argument after the subcommand: Zephyr's
 *   wildcard expansion (SHELL_WILDCARD) finds no matching subcommand and
 *   stops reading the arguments there, which ends in the same "wrong
 *   parameter count" (a valid key never contains these characters).
 * "-h" and "--help" are ordinary arguments, as in the Zephyr build
 * (SHELL_HELP_OPT_PARSE is off).  The Zephyr shell printed errors in the
 * same stream as the rest; here they go to stderr, which is the same
 * console.  The exit status is 0 on success and 1 otherwise (Zephyr: the
 * handler's return value, seen only with the "retval" command).
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "kv_hex.h"
#include "kv_name.h"
#include "kv_store.h"

struct kv_subcmd {
	const char *syntax;
	const char *help;
	int mandatory; /* argc including the subcommand, as SHELL_CMD_ARG */
	int (*handler)(int argc, char **argv);
};

static int cmd_set(int argc, char **argv)
{
	int ret;

	(void)argc;
	if (!kv_shell_key_allowed(argv[1])) {
		fprintf(stderr, "Reserved key\n");
		return -EACCES;
	}
	ret = kv_store_set(argv[1], argv[2], strlen(argv[2]));
	if (ret) {
		fprintf(stderr, "KV write failed: %d\n", ret);
	} else {
		printf("Saved\n");
	}
	return ret;
}

static int cmd_sethex(int argc, char **argv)
{
	uint8_t value[KV_STORE_VALUE_MAX];
	size_t chars = strlen(argv[2]);
	size_t len;
	int ret;

	(void)argc;
	if (!kv_shell_key_allowed(argv[1])) {
		fprintf(stderr, "Reserved key\n");
		return -EACCES;
	}
	if (!chars || chars % 2 || chars > sizeof(value) * 2) {
		fprintf(stderr, "Expected 1-128 bytes as hexadecimal pairs\n");
		return -EINVAL;
	}
	len = kv_hex2bin(argv[2], chars, value, sizeof(value));
	if (len != chars / 2) {
		fprintf(stderr, "Invalid hexadecimal value\n");
		return -EINVAL;
	}
	ret = kv_store_set(argv[1], value, len);
	if (ret) {
		fprintf(stderr, "KV write failed: %d\n", ret);
	} else {
		printf("Saved\n");
	}
	return ret;
}

static int cmd_get(int argc, char **argv)
{
	uint8_t value[KV_STORE_VALUE_MAX];
	char hex[2 * KV_STORE_VALUE_MAX + 1];
	ssize_t len;

	(void)argc;
	if (!kv_shell_key_allowed(argv[1])) {
		fprintf(stderr, "Reserved key\n");
		return -EACCES;
	}
	len = kv_store_get(argv[1], value, sizeof(value));
	if (len < 0) {
		fprintf(stderr, "KV read failed: %d\n", (int)len);
		return (int)len;
	}
	kv_bin2hex(value, len, hex, sizeof(hex));
	printf("%s\n", hex);
	return 0;
}

static int cmd_delete(int argc, char **argv)
{
	int ret;

	(void)argc;
	if (!kv_shell_key_allowed(argv[1])) {
		fprintf(stderr, "Reserved key\n");
		return -EACCES;
	}
	ret = kv_store_delete(argv[1]);
	if (ret) {
		fprintf(stderr, "KV delete failed: %d\n", ret);
	} else {
		printf("Deleted\n");
	}
	return ret;
}

static const struct kv_subcmd kv_commands[] = {
	{ "set", "Store a text value: <key> <value>", 3, cmd_set },
	{ "sethex", "Store hex bytes: <key> <hex>", 3, cmd_sethex },
	{ "get", "Print a value as hex: <key>", 2, cmd_get },
	{ "delete", "Delete a value: <key>", 2, cmd_delete },
};

#define KV_NCOMMANDS (sizeof(kv_commands) / sizeof(kv_commands[0]))

/* shell_internal_help_print() for "kv": z_shell_help_cmd_print() and
 * z_shell_help_subcmd_print() with help_item_print()'s layout (two
 * spaces, the name padded to the longest name, two spaces, ": ").
 */
static void kv_help(void)
{
	int longest = 0;

	for (size_t i = 0; i < KV_NCOMMANDS; i++) {
		int n = (int)strlen(kv_commands[i].syntax);

		longest = n > longest ? n : longest;
	}
	printf("kv - Application key-value storage\n");
	printf("Subcommands:\n");
	for (size_t i = 0; i < KV_NCOMMANDS; i++) {
		printf("  %-*s  : %s\n", longest, kv_commands[i].syntax,
		       kv_commands[i].help);
	}
}

static bool has_wildcard(const char *s)
{
	return strchr(s, '*') != NULL || strchr(s, '?') != NULL;
}

int main(int argc, char *argv[])
{
	const struct kv_subcmd *sub = NULL;
	int ret;

	if (argc >= 2) {
		for (size_t i = 0; i < KV_NCOMMANDS; i++) {
			if (strcmp(argv[1], kv_commands[i].syntax) == 0) {
				sub = &kv_commands[i];
				break;
			}
		}
	}
	if (!sub) {
		kv_help();
		fflush(stdout);
		return EXIT_FAILURE;
	}

	/* argv of the subcommand: argv[0] is its name. */
	argc--;
	argv++;
	if (argc != sub->mandatory || has_wildcard(argv[1])) {
		fprintf(stderr, "%s: wrong parameter count\n", sub->syntax);
		printf("%s - %s\n", sub->syntax, sub->help);
		fflush(stdout);
		return EXIT_FAILURE;
	}

	ret = sub->handler(argc, argv);
	fflush(stdout);
	return ret ? EXIT_FAILURE : EXIT_SUCCESS;
}
