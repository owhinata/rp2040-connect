/* SPDX-License-Identifier: Apache-2.0 */
/* The "i2c_dma" command: i2c_dw_dma_mode(), i2c_dw_dma_stats() and the
 * command's registration from zephyr/app/usb-console/drivers/i2c_dw/
 * i2c_dw.c:770-801, with the same syntax and output.
 *
 * "mode" is SHELL_CMD_ARG(mode, ..., 1, 1) and "stats" SHELL_CMD_ARG(stats,
 * ..., 1, 0): one more word than that is "wrong parameter count" (../subcmd).
 * "i2c_dma" alone or with an unknown word prints the help.  "mode on|off"
 * only sets the flag, as in the Zephyr build: nothing waits for a transfer
 * in progress, so switch while no stream or transfer runs.  The error line
 * goes to stderr, which is the same console (Zephyr: shell_error()).  The
 * exit status is 0 on success and 1 otherwise.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "nano_i2c_dma.h"
#include "subcmd.h"

static int i2c_dw_dma_stats(int argc, char **argv)
{
	struct nano_i2c_dma_stats stats;

	(void)argc;
	(void)argv;
	nano_i2c_dma_get_stats(&stats);
	printf("dma_transfers=%u dma_bytes=%u dma_errors=%u dma_fallbacks=%u\n",
	       (unsigned int)stats.transfers, (unsigned int)stats.bytes,
	       (unsigned int)stats.errors, (unsigned int)stats.fallbacks);
	return 0;
}

static int i2c_dw_dma_mode(int argc, char **argv)
{
	if (argc == 2) {
		if (!strcmp(argv[1], "on")) {
			nano_i2c_dma_set_enabled(true);
		} else if (!strcmp(argv[1], "off")) {
			nano_i2c_dma_set_enabled(false);
		} else {
			fprintf(stderr, "Expected on or off\n");
			return -EINVAL;
		}
	}
	printf("i2c_dma=%s\n", nano_i2c_dma_enabled() ? "on" : "off");
	return 0;
}

static const struct subcmd i2c_dma_commands[] = {
	SUBCMD_ARG(mode, NULL, "Set or show I2C DMA mode: [on|off]", i2c_dw_dma_mode, 1, 1),
	SUBCMD_ARG(stats, NULL, "Show RP2040 I2C DMA counters", i2c_dw_dma_stats, 1, 0),
	SUBCMD_SET_END
};

/* SHELL_CMD_REGISTER(i2c_dma, &i2c_dma_commands, "RP2040 I2C DMA controls", NULL) */
static const struct subcmd i2c_dma_root =
	SUBCMD(i2c_dma, i2c_dma_commands, "RP2040 I2C DMA controls", NULL);

int main(int argc, char *argv[])
{
	return subcmd_main(&i2c_dma_root, argc, argv);
}
