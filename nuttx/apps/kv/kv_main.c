/* SPDX-License-Identifier: Apache-2.0 */
/* The "kv" command: cmd_set(), cmd_sethex(), cmd_get() and cmd_delete() of
 * zephyr/app/usb-console/src/kv_store.c, with the same syntax and output.
 *
 * The Zephyr shell itself printed some of the output (the help for "kv"
 * alone or an unknown subcommand, "<sub>: wrong parameter count" for the
 * SHELL_CMD_ARG() counts, the wildcard rule); ../subcmd reproduces it.  The
 * Zephyr shell printed errors in the same stream as the rest; here they go
 * to stderr, which is the same console.  The exit status is 0 on success
 * and 1 otherwise (Zephyr: the handler's return value, seen only with the
 * "retval" command).
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
#include "subcmd.h"

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

static const struct subcmd kv_commands[] = {
	SUBCMD_ARG(set, NULL, "Store a text value: <key> <value>", cmd_set, 3, 0),
	SUBCMD_ARG(sethex, NULL, "Store hex bytes: <key> <hex>", cmd_sethex, 3, 0),
	SUBCMD_ARG(get, NULL, "Print a value as hex: <key>", cmd_get, 2, 0),
	SUBCMD_ARG(delete, NULL, "Delete a value: <key>", cmd_delete, 2, 0),
	SUBCMD_SET_END
};

/* SHELL_CMD_REGISTER(kv, &kv_commands, "Application key-value storage", NULL) */
static const struct subcmd kv_root =
	SUBCMD(kv, kv_commands, "Application key-value storage", NULL);

int main(int argc, char *argv[])
{
	return subcmd_main(&kv_root, argc, argv);
}
