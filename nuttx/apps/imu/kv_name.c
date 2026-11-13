/* SPDX-License-Identifier: Apache-2.0 */
/* make_name() and shell_key_allowed() of zephyr/app/usb-console/src/
 * kv_store.c, unchanged except for their names (see kv_name.h).
 */

#include <errno.h>
#include <string.h>

#include "kv_name.h"

bool kv_shell_key_allowed(const char *key)
{
	return strncmp(key, KV_SYSTEM_PREFIX, sizeof(KV_SYSTEM_PREFIX) - 1) !=
	       0;
}

int kv_make_name(const char *key, char *name, size_t capacity)
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
