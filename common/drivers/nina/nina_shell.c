/* SPDX-License-Identifier: MIT */
/* The "nina" shell command: "nina version" asks the module for its firmware
 * version (nina_request(), nina_spi.c) and prints the capability bits of
 * the firmware's patches 0003 to 0006 (the module's and those in use,
 * nina_proto.h).  It was the Zephyr build's
 * src/main.c (cmd_nina_version() and the registration); the Zephyr and
 * FreeRTOS builds use it (zephyr/app/usb-console/CMakeLists.txt,
 * freertos/drivers/nina/nina.cmake).  The NuttX build has the same command
 * as a builtin application on subcmd (nuttx/apps/nina/nina_main.c); moving
 * it onto this file changes its image (its help and errors would come from
 * zshell.c).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/shell/shell.h>
#include "nina_proto.h"

/* The Zephyr build's "nina passthrough" (zephyr/app/usb-console/drivers/
 * nina/nina_passthrough.h); the FreeRTOS build adds its own to this set
 * (freertos/drivers/nina/nina_passthrough.c).
 */
#ifdef CONFIG_NANO_NINA_PASSTHROUGH
#include "nina_passthrough.h"
#else
#define NINA_PASSTHROUGH_SHELL_CMD
#endif

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
	shell_print(sh, "NINA capabilities: 0x%08x (in use 0x%08x)",
		    (unsigned int)nina_module_capabilities(), (unsigned int)nina_capabilities());
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(nina_commands, NINA_PASSTHROUGH_SHELL_CMD
	SHELL_CMD(version, NULL, "Show NINA firmware version", cmd_nina_version),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(nina, &nina_commands, "NINA Wi-Fi module", NULL);
