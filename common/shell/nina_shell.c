/* SPDX-License-Identifier: MIT */
/* The "nina" shell command: "nina version" asks the module for its firmware
 * version (nina_request(), nina_proto.c) and prints the capability bits of
 * the firmware's patches 0003 to 0006 (the module's and those in use,
 * nina_proto.h); with CONFIG_NINA_PASSTHROUGH, "nina passthrough confirm"
 * restarts the board into the NINA passthrough mode
 * (common/drivers/nina/nina_passthrough.h).  It was the Zephyr build's
 * src/main.c (cmd_nina_version() and the registration); the Zephyr and
 * FreeRTOS builds use it (zephyr/app/usb-console/CMakeLists.txt,
 * freertos/drivers/nina/nina.cmake).  The NuttX build has the same command
 * as a builtin application on subcmd (nuttx/apps/nina/nina_main.c); moving
 * it onto this file changes its image (its help and errors would come from
 * zshell.c).
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include "internal/nina_proto.h"

#ifdef CONFIG_NINA_PASSTHROUGH
#include "nina_passthrough.h"

/* Time for the USB code to send the command's last message before the
 * restart.
 */
#define PASSTHROUGH_RESTART_DELAY_MS 100
#endif

static int cmd_nina_version(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t version[16];
	size_t len;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	ret = nina_request(NINA_CMD_GET_FW_VERSION, NULL, 0, false, false, version,
			   sizeof(version) - 1, &len);
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

#ifdef CONFIG_NINA_PASSTHROUGH
/* "nina passthrough confirm": the request, then the restart. */
static int cmd_nina_passthrough(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	if (strcmp(argv[1], "confirm") != 0) {
		shell_error(sh, "Type \"nina passthrough confirm\" to restart into the NINA "
				"passthrough");
		return -EINVAL;
	}
	shell_print(sh, "Restarting into the NINA passthrough: the USB port relays to the "
			"NINA's UART (for esptool).");
	shell_print(sh, "To leave it: close the port at %u bps, or press RESET.",
		    NINA_PASSTHROUGH_EXIT_BAUD);
	(void)k_msleep(PASSTHROUGH_RESTART_DELAY_MS);
	nina_passthrough_request();
}

#define NINA_PASSTHROUGH_SHELL_CMD                                                                 \
	SHELL_CMD_ARG(passthrough, NULL,                                                           \
		      "Restart into the NINA passthrough for esptool: passthrough confirm\n"       \
		      "(USB <-> the NINA's UART; leave it by closing the port at 300 bps, "        \
		      "or with RESET)",                                                            \
		      cmd_nina_passthrough, 2, 0),
#else
#define NINA_PASSTHROUGH_SHELL_CMD
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(nina_commands, NINA_PASSTHROUGH_SHELL_CMD
	SHELL_CMD(version, NULL, "Show NINA firmware version", cmd_nina_version),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(nina, &nina_commands, "NINA Wi-Fi module", NULL);
