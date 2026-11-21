/* SPDX-License-Identifier: Apache-2.0 */
/* The "nina" command: cmd_nina_version() and the command's registration
 * from zephyr/app/usb-console/src/main.c, with the same syntax and output
 * (common/shell/nina_shell.c today, with the line of the module's
 * capability bits and those in use, nina_wifi.h).  With the board's NINA
 * passthrough mode (CONFIG_NINA_PASSTHROUGH), also "passthrough confirm"
 * of the Zephyr and FreeRTOS builds (common/shell/nina_shell.c
 * cmd_nina_passthrough()), with its messages: it restarts the board into
 * the mode (nina_passthrough_request() of common/drivers/nina).
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

#include "nina_wifi.h"
#include "subcmd.h"

#ifdef CONFIG_NINA_PASSTHROUGH
#  include <errno.h>
#  include <string.h>
#  include <unistd.h>

#  include "nina_passthrough.h"

/* Time for the USB device (or Telnet) to send the messages before the
 * restart (common/shell/nina_shell.c's PASSTHROUGH_RESTART_DELAY_MS).
 */

#  define PASSTHROUGH_RESTART_DELAY_MS 100
#endif

static int cmd_nina_version(int argc, char **argv)
{
	char version[NINA_WIFI_FW_VERSION_SIZE];
	uint32_t module, in_use;
	int ret;

	(void)argc;
	(void)argv;
	ret = nina_wifi_fw_version(version);
	if (ret) {
		fprintf(stderr, "NINA version request failed: %d\n", ret);
		return ret;
	}
	nina_wifi_capabilities(&module, &in_use);
	printf("NINA firmware: %s\n", version);
	printf("NINA capabilities: 0x%08x (in use 0x%08x)\n", (unsigned int)module,
	       (unsigned int)in_use);
	return 0;
}

#ifdef CONFIG_NINA_PASSTHROUGH
static int cmd_nina_passthrough(int argc, char **argv)
{
	(void)argc;
	if (strcmp(argv[1], "confirm") != 0) {
		fprintf(stderr, "Type \"nina passthrough confirm\" to restart into the NINA "
				"passthrough\n");
		return -EINVAL;
	}
	printf("Restarting into the NINA passthrough: the USB port relays to the NINA's UART "
	       "(for esptool).\n");
	printf("To leave it: close the port at %u bps, or press RESET.\n", NINA_PASSTHROUGH_EXIT_BAUD);
	fflush(stdout);
	usleep(PASSTHROUGH_RESTART_DELAY_MS * 1000);
	nina_passthrough_request();
	return 0;
}
#endif

/* Sorted by name, as Zephyr's shell lists them. */
static const struct subcmd nina_commands[] = {
#ifdef CONFIG_NINA_PASSTHROUGH
	SUBCMD_ARG(passthrough, NULL,
		   "Restart into the NINA passthrough for esptool: passthrough confirm\n"
		   "(USB <-> the NINA's UART; leave it by closing the port at 300 bps, "
		   "or with RESET)",
		   cmd_nina_passthrough, 2, 0),
#endif
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
