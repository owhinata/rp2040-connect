/* SPDX-License-Identifier: Apache-2.0 */
/* apps/imu/kv_name.c: the key rules of the Zephyr build's kv_store.c
 * (make_name(), shell_key_allowed()).
 */
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "kv_name.h"

int main(void)
{
	char name[KV_NAME_SIZE];
	char key[40];

	assert(KV_NAME_SIZE == 40);
	assert(kv_make_name("imu.port", name, sizeof(name)) == 0);
	assert(strcmp(name, "app_kv/imu.port") == 0);
	/* All allowed characters. */
	assert(kv_make_name("azAZ09_-.", name, sizeof(name)) == 0);
	assert(strcmp(name, "app_kv/azAZ09_-.") == 0);
	/* 1 and 32 characters, not 0 or 33. */
	assert(kv_make_name("a", name, sizeof(name)) == 0);
	memset(key, 'k', 32);
	key[32] = '\0';
	assert(kv_make_name(key, name, sizeof(name)) == 0);
	assert(strlen(name) == 39);
	key[32] = 'k';
	key[33] = '\0';
	assert(kv_make_name(key, name, sizeof(name)) == -EINVAL);
	assert(kv_make_name("", name, sizeof(name)) == -EINVAL);
	assert(kv_make_name(NULL, name, sizeof(name)) == -EINVAL);
	/* Other characters. */
	const char *bad[] = { "a b", "a/b", "a:b", "a*", "a?", "\xe3\x81\x82",
			      "a\tb", "a+b", "a=b", "$a" };

	for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
		assert(kv_make_name(bad[i], name, sizeof(name)) == -EINVAL);
	}
	/* The key is checked before the buffer. */
	assert(kv_make_name("a", name, 9) == 0);
	assert(kv_make_name("ab", name, 9) == -ENAMETOOLONG);
	assert(kv_make_name("a b", name, 1) == -EINVAL);
	/* "sys." is reserved for the shell; "sys" and "sysx" are not. */
	assert(!kv_shell_key_allowed("sys."));
	assert(!kv_shell_key_allowed("sys.x"));
	assert(kv_shell_key_allowed("sys"));
	assert(kv_shell_key_allowed("sysx"));
	assert(kv_shell_key_allowed("SYS.x"));
	assert(kv_shell_key_allowed("imu.port"));
	puts("kv_name ok");
	return 0;
}
