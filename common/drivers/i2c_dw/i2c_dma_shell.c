/* SPDX-License-Identifier: Apache-2.0 */
/* The "i2c_dma" shell command: i2c_dw_dma_mode(), i2c_dw_dma_stats() and the
 * command's registration from zephyr/app/usb-console/drivers/i2c_dw/
 * i2c_dw.c:770-801, with the same syntax, help and output, on the state of
 * the shared I2C0 core (nano_i2c_dma.h) in place of the Zephyr driver's
 * atomics (atomic_get() and atomic_set() of one word each: plain reads and
 * writes here).  The FreeRTOS build uses it (freertos/drivers/i2c_dw/
 * i2c_dw.cmake), on the shared shell (common/shell/zshell.c).
 *
 * "mode" takes 0 or 1 argument, "stats" none; more is the shell's "wrong
 * parameter count".  "i2c_dma" alone prints the help.  "mode on|off" only
 * sets the flag, as in the Zephyr build: nothing waits for a transfer in
 * progress, so switch while no stream or transfer runs.
 *
 * The Zephyr build keeps these lines in its driver (moving them would change
 * its image until its driver moves onto the core, Z02).  The NuttX build has
 * the same command as a builtin application on subcmd
 * (nuttx/apps/i2c_dma/i2c_dma_main.c); moving it onto this file changes its
 * image too (its help and errors would come from zshell.c).
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/shell/shell.h>

#include "nano_i2c_dma.h"

static int i2c_dw_dma_stats(const struct shell *sh, size_t argc, char **argv)
{
	struct nano_i2c_dma_stats stats;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	nano_i2c_dma_get_stats(&stats);
	shell_print(sh, "dma_transfers=%u dma_bytes=%u dma_errors=%u dma_fallbacks=%u",
		    (unsigned int)stats.transfers, (unsigned int)stats.bytes,
		    (unsigned int)stats.errors, (unsigned int)stats.fallbacks);
	return 0;
}

static int i2c_dw_dma_mode(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 2) {
		if (!strcmp(argv[1], "on")) {
			nano_i2c_dma_set_enabled(true);
		} else if (!strcmp(argv[1], "off")) {
			nano_i2c_dma_set_enabled(false);
		} else {
			shell_error(sh, "Expected on or off");
			return -EINVAL;
		}
	}
	shell_print(sh, "i2c_dma=%s", nano_i2c_dma_enabled() ? "on" : "off");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(i2c_dma_commands,
	SHELL_CMD_ARG(mode, NULL, "Set or show I2C DMA mode: [on|off]", i2c_dw_dma_mode, 1, 1),
	SHELL_CMD_ARG(stats, NULL, "Show RP2040 I2C DMA counters", i2c_dw_dma_stats, 1, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(i2c_dma, &i2c_dma_commands, "RP2040 I2C DMA controls", NULL);
