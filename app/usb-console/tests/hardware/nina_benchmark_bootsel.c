/* SPDX-License-Identifier: Apache-2.0 */
#include <pico/bootrom.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

static int cmd_benchmark_bootsel(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "Entering USB bootloader");
	k_msleep(100);
	reset_usb_boot(0, 0);
	return 0;
}

SHELL_CMD_REGISTER(benchmark_bootsel, NULL, "Enter BOOTSEL for benchmark builds",
		   cmd_benchmark_bootsel);
