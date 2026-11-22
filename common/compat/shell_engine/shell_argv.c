/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* The FreeRTOS build's split of a command line into words (shell_argv.h):
 * make_argv() and z_shell_make_argv() of Zephyr's
 * subsys/shell/shell_utils.c (Zephyr v4.5.0), with z_shell_strlen() as
 * strlen() and the C99 forms of the casts; otherwise unchanged.
 */

#include <ctype.h>
#include <stdint.h>
#include <string.h>

#include "shell_argv.h"

static char make_argv(char **ppcmd, uint8_t c)
{
	char *cmd = *ppcmd;
	char quote = 0;

	while (1) {
		c = *cmd;

		if (c == '\0') {
			break;
		}

		if (!quote) {
			switch (c) {
			case '\\':
				memmove(cmd, cmd + 1, strlen(cmd));
				cmd += 1;
				continue;

			case '\'':
			case '\"':
				memmove(cmd, cmd + 1, strlen(cmd));
				quote = c;
				continue;
			default:
				break;
			}
		}

		if (quote == c) {
			memmove(cmd, cmd + 1, strlen(cmd));
			quote = 0;
			continue;
		}

		if (quote && c == '\\') {
			char t = *(cmd + 1);

			if (t == quote) {
				memmove(cmd, cmd + 1, strlen(cmd));
				cmd += 1;
				continue;
			}

			if (t == '0') {
				uint8_t i;
				uint8_t v = 0U;

				for (i = 2U; i < (2 + 3); i++) {
					t = *(cmd + i);

					if (t >= '0' && t <= '7') {
						v = (uint8_t)((v << 3) | (t - '0'));
					} else {
						break;
					}
				}

				if (i > 2) {
					memmove(cmd, cmd + (i - 1), strlen(cmd) - (i - 2));
					*cmd++ = (char)v;
					continue;
				}
			}

			if (t == 'x') {
				uint8_t i;
				uint8_t v = 0U;

				for (i = 2U; i < (2 + 2); i++) {
					t = *(cmd + i);

					if (t >= '0' && t <= '9') {
						v = (uint8_t)((v << 4) | (t - '0'));
					} else if ((t >= 'a') && (t <= 'f')) {
						v = (uint8_t)((v << 4) | (t - 'a' + 10));
					} else if ((t >= 'A') && (t <= 'F')) {
						v = (uint8_t)((v << 4) | (t - 'A' + 10));
					} else {
						break;
					}
				}

				if (i > 2) {
					memmove(cmd, cmd + (i - 1), strlen(cmd) - (i - 2));
					*cmd++ = (char)v;
					continue;
				}
			}
		}

		if (!quote && isspace((int)c) != 0) {
			break;
		}

		cmd += 1;
	}
	*ppcmd = cmd;

	return quote;
}

char z_shell_make_argv(size_t *argc, const char **argv, char *cmd, uint8_t max_argc)
{
	char quote = 0;
	char c;

	*argc = 0;
	do {
		c = *cmd;
		if (c == '\0') {
			break;
		}

		if (isspace((int)c) != 0) {
			*cmd++ = '\0';
			continue;
		}

		argv[(*argc)++] = cmd;
		if (*argc == max_argc) {
			break;
		}
		quote = make_argv(&cmd, (uint8_t)c);
	} while (true);

	return quote;
}
