/* SPDX-License-Identifier: MIT */
/* The "dmesg" shell command on the RAM log (common/compat/include/log_ram.h), for the
 * builds that run the Zephyr shell's commands on the compatibility layer's
 * shell (the FreeRTOS build, common/compat/port/pico_sdk/compat.cmake; NuttX
 * has its own application, nuttx/apps/dmesg).  cmd_dmesg() of the Zephyr
 * build's zephyr/app/usb-console/src/log_ram.c, with the same text: the log
 * in pieces of up to 96 bytes, so that no second copy of it is needed; the
 * position counts bytes from the start (log_ram_piece()), which keeps it
 * valid when new messages push old ones out while printing; printing stops
 * after twice the ring's size in case logging keeps up with it.  Arguments
 * are ignored.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/shell/shell.h>

#include "log_ram.h"

static int cmd_dmesg(const struct shell *sh, size_t argc, char **argv)
{
	char piece[97];
	uint64_t position = 0;
	bool first = true;
	bool any = false;

	(void)argc;
	(void)argv;
	for (size_t printed = 0; printed < 2u * LOG_RAM_SIZE;) {
		size_t n = log_ram_piece(&position, &first, piece, sizeof(piece) - 1);

		if (n == 0) {
			break;
		}
		piece[n] = '\0';
		shell_fprintf(sh, SHELL_NORMAL, "%s", piece);
		position += n;
		printed += n;
		any = true;
	}
	if (!any) {
		shell_print(sh, "No logs.");
	}
	return 0;
}

SHELL_CMD_REGISTER(dmesg, NULL, "Show logs stored in RAM", cmd_dmesg);
