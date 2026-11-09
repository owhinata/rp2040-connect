/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include <zephyr/settings/settings.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>
#include "kv_store.h"

#define KV_PREFIX        "app_kv/"
#define KV_SYSTEM_PREFIX "sys."

static bool shell_key_allowed(const char *key)
{
	return strncmp(key, KV_SYSTEM_PREFIX, sizeof(KV_SYSTEM_PREFIX) - 1) !=
	       0;
}

static int make_name(const char *key, char *name, size_t capacity)
{
	size_t len;

	if (!key) {
		return -EINVAL;
	}
	len = strnlen(key, KV_STORE_KEY_MAX + 1);
	if (!len || len > KV_STORE_KEY_MAX) {
		return -EINVAL;
	}
	for (size_t i = 0; i < len; i++) {
		char c = key[i];

		if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
		      (c >= '0' && c <= '9') || c == '_' || c == '-' ||
		      c == '.')) {
			return -EINVAL;
		}
	}
	if (sizeof(KV_PREFIX) + len > capacity) {
		return -ENAMETOOLONG;
	}
	memcpy(name, KV_PREFIX, sizeof(KV_PREFIX) - 1);
	memcpy(name + sizeof(KV_PREFIX) - 1, key, len + 1);
	return 0;
}

int kv_store_init(void)
{
	return settings_subsys_init();
}

int kv_store_set(const char *key, const void *value, size_t len)
{
	char name[sizeof(KV_PREFIX) + KV_STORE_KEY_MAX];
	int ret = make_name(key, name, sizeof(name));

	if (ret) {
		return ret;
	}
	if (!value || !len || len > KV_STORE_VALUE_MAX) {
		return -EINVAL;
	}
	ret = kv_store_init();
	if (ret) {
		return ret;
	}
	return settings_save_one(name, value, len);
}

ssize_t kv_store_get(const char *key, void *value, size_t capacity)
{
	char name[sizeof(KV_PREFIX) + KV_STORE_KEY_MAX];
	ssize_t len;
	int ret = make_name(key, name, sizeof(name));

	if (ret) {
		return ret;
	}
	if (!value || !capacity) {
		return -EINVAL;
	}
	ret = kv_store_init();
	if (ret) {
		return ret;
	}
	len = settings_load_one(name, value, capacity);
	if (len == 0) {
		return -ENOENT;
	}
	if (len > 0 && (size_t)len > capacity) {
		return -ENOBUFS;
	}
	return len;
}

int kv_store_delete(const char *key)
{
	char name[sizeof(KV_PREFIX) + KV_STORE_KEY_MAX];
	int ret = make_name(key, name, sizeof(name));

	if (ret) {
		return ret;
	}
	ret = kv_store_init();
	return ret ? ret : settings_delete(name);
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	if (!shell_key_allowed(argv[1])) {
		shell_error(sh, "Reserved key");
		return -EACCES;
	}
	ret = kv_store_set(argv[1], argv[2], strlen(argv[2]));
	if (ret) {
		shell_error(sh, "KV write failed: %d", ret);
	} else {
		shell_print(sh, "Saved");
	}
	return ret;
}

static int cmd_sethex(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t value[KV_STORE_VALUE_MAX];
	size_t chars = strlen(argv[2]);
	size_t len;
	int ret;

	ARG_UNUSED(argc);
	if (!shell_key_allowed(argv[1])) {
		shell_error(sh, "Reserved key");
		return -EACCES;
	}
	if (!chars || chars % 2 || chars > sizeof(value) * 2) {
		shell_error(sh, "Expected 1-128 bytes as hexadecimal pairs");
		return -EINVAL;
	}
	len = hex2bin(argv[2], chars, value, sizeof(value));
	if (len != chars / 2) {
		shell_error(sh, "Invalid hexadecimal value");
		return -EINVAL;
	}
	ret = kv_store_set(argv[1], value, len);
	if (ret) {
		shell_error(sh, "KV write failed: %d", ret);
	} else {
		shell_print(sh, "Saved");
	}
	return ret;
}

static int cmd_get(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t value[KV_STORE_VALUE_MAX];
	char hex[2 * KV_STORE_VALUE_MAX + 1];
	ssize_t len;

	ARG_UNUSED(argc);
	if (!shell_key_allowed(argv[1])) {
		shell_error(sh, "Reserved key");
		return -EACCES;
	}
	len = kv_store_get(argv[1], value, sizeof(value));
	if (len < 0) {
		shell_error(sh, "KV read failed: %d", (int)len);
		return (int)len;
	}
	bin2hex(value, len, hex, sizeof(hex));
	shell_print(sh, "%s", hex);
	return 0;
}

static int cmd_delete(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	if (!shell_key_allowed(argv[1])) {
		shell_error(sh, "Reserved key");
		return -EACCES;
	}
	ret = kv_store_delete(argv[1]);
	if (ret) {
		shell_error(sh, "KV delete failed: %d", ret);
	} else {
		shell_print(sh, "Deleted");
	}
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	kv_commands,
	SHELL_CMD_ARG(set, NULL, "Store a text value: <key> <value>", cmd_set,
		      3, 0),
	SHELL_CMD_ARG(sethex, NULL, "Store hex bytes: <key> <hex>", cmd_sethex,
		      3, 0),
	SHELL_CMD_ARG(get, NULL, "Print a value as hex: <key>", cmd_get, 2, 0),
	SHELL_CMD_ARG(delete, NULL, "Delete a value: <key>", cmd_delete, 2, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(kv, &kv_commands, "Application key-value storage", NULL);
