/* SPDX-License-Identifier: Apache-2.0 */
/* The "nina" command: cmd_nina_version() and the command's registration
 * from zephyr/app/usb-console/src/main.c, with the same syntax and output.
 *
 * "version" is SHELL_CMD() (no argument count check: extra words are
 * ignored).  "nina" alone or with an unknown word prints the help, as the
 * Zephyr shell does (../subcmd).  The error line goes to stderr, which is
 * the same console (Zephyr: shell_error()).  The exit status is 0 on
 * success and 1 otherwise.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "nina_proto.h"
#include "subcmd.h"

static int cmd_nina_version(int argc, char **argv)
{
	uint8_t version[16];
	size_t len;
	int ret;

	(void)argc;
	(void)argv;
	ret = nina_request(0x37, NULL, 0, false, false, version, sizeof(version) - 1, &len);
	if (ret) {
		fprintf(stderr, "NINA version request failed: %d\n", ret);
		return ret;
	}
	version[len] = '\0';
	printf("NINA firmware: %s\n", version);
	return 0;
}

static const struct subcmd nina_commands[] = {
	SUBCMD(version, NULL, "Show NINA firmware version", cmd_nina_version),
	SUBCMD_SET_END
};

/* SHELL_CMD_REGISTER(nina, &nina_commands, "NINA Wi-Fi module", NULL) */
static const struct subcmd nina_root =
	SUBCMD(nina, nina_commands, "NINA Wi-Fi module", NULL);

int main(int argc, char *argv[])
{
	return subcmd_main(&nina_root, argc, argv);
}
