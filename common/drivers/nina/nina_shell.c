/* SPDX-License-Identifier: MIT */
/* The "nina" shell command: "nina version" asks the module for its firmware
 * version (nina_request(), nina_spi.c).  The text of the Zephyr build's
 * src/main.c (cmd_nina_version() and the registration), unchanged; the
 * FreeRTOS build uses it (freertos/drivers/nina/nina.cmake).
 *
 * The Zephyr build keeps its own lines in src/main.c for now: building this
 * file there instead changes the image (the linker orders three flash
 * veneers differently), so the move waits for Z01, which changes the image
 * anyway.  The NuttX build has the same command as a builtin application on
 * subcmd (nuttx/apps/nina/nina_main.c); moving it onto this file changes
 * its image too (its help and errors would come from zshell.c).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/shell/shell.h>
#include "nina_proto.h"

static int cmd_nina_version(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t version[16];
	size_t len;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	ret = nina_request(0x37, NULL, 0, false, false, version, sizeof(version) - 1, &len);
	if (ret) {
		shell_error(sh, "NINA version request failed: %d", ret);
		return ret;
	}
	version[len] = '\0';
	shell_print(sh, "NINA firmware: %s", version);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(nina_commands,
	SHELL_CMD(version, NULL, "Show NINA firmware version", cmd_nina_version),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(nina, &nina_commands, "NINA Wi-Fi module", NULL);
