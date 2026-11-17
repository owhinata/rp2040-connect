/* SPDX-License-Identifier: MIT */
/* The OS part of the compatibility layer's <zephyr/shell/shell.h> for the
 * host tests of the key-value store (kv_settings.c), which do not run
 * commands: a shell backend with nothing in it.
 */
#ifndef TEST_KV_ZEPHYR_OS_SHELL_H
#define TEST_KV_ZEPHYR_OS_SHELL_H
struct shell {
	int unused;
};
#endif
