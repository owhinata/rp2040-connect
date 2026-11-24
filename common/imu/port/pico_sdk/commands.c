/* SPDX-License-Identifier: MIT */
/* The commands of the builds on the Pico SDK that only the OS and the
 * Pico SDK can answer ("version", "free", and "ps", whose handler and the
 * kernel's name and version are the OS's, commands_os.h), written as
 * Zephyr shell commands (common/compat/include/shell_engine.h), and
 * the table of all the commands of the shell (shell_engine_cmds[]).
 */
#include <malloc.h>
#include <stdint.h>
#include "pico/version.h"
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include "shell_engine.h"
#include "commands_os.h"

static int cmd_version(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	shell_print(sh, "%s %s, Pico SDK %s, GCC %s", commands_os_kernel_name(),
		    commands_os_kernel_version(), PICO_SDK_VERSION_STRING, __VERSION__);
	return 0;
}

SHELL_CMD_REGISTER(version, NULL, "Show the versions of the kernel, SDK and compiler", cmd_version);

/* newlib's malloc() (the shell's state of a command line, "zperf"'s
 * buffers, the NINA passthrough mode's): what it has taken from the system
 * (_sbrk(), the RAM from the end of .bss up), of that in use and free, and
 * what it can still take (up to __StackLimit, the bound the Pico SDK's
 * _sbrk() checks; the kernel has no heap of its own).
 */
extern char __StackLimit;
void *_sbrk(int incr);

static int cmd_free(const struct shell *sh, size_t argc, char **argv)
{
	struct mallinfo m = mallinfo();
	char *brk = _sbrk(0);

	(void)argc;
	(void)argv;

	shell_print(sh, "malloc: %u bytes from the system, in use %u, free %u, more to take %u",
		    (unsigned int)m.arena, (unsigned int)m.uordblks, (unsigned int)m.fordblks,
		    (unsigned int)(&__StackLimit - brk));
	return 0;
}

SHELL_CMD_REGISTER(free, NULL, "Show the use of malloc()", cmd_free);

/* The OS's "ps" (commands_os.h). */
SHELL_CMD_REGISTER(ps, NULL, "List the tasks and show the CPU load", commands_os_ps);

/* The commands of other sources: "help" of the shell engine
 * (common/compat/shell_engine/shell_engine.c), the generic commands of
 * common/shell/kernel_shell.c ("bootsel", "date", "devmem", "fault", "id",
 * "reboot", "sleep", "uptime"), common/shell/dmesg_shell.c,
 * common/shell/i2c_dma_shell.c, common/shell/ifconfig_shell.c,
 * common/imu/imu_stream.c, common/imu/kv_store.c (with "kv wipe" of
 * common/compat/kv_store_wipe.c), common/shell/nina_shell.c ("nina"),
 * common/compat/zephyr_wifi_shell.c ("wifi", with "wifi cred" of the copy
 * of Zephyr's wifi_credentials_shell.c), common/imu/time_sync.c ("ntp"),
 * common/compat/zephyr_zperf_shell.c,
 * common/shell/sensor_shell.c ("sensor") and
 * common/shell/i2c_shell.c ("i2c").
 */
SHELL_ENGINE_CMD_DECLARE(bootsel);
SHELL_ENGINE_CMD_DECLARE(date);
SHELL_ENGINE_CMD_DECLARE(devmem);
SHELL_ENGINE_CMD_DECLARE(dmesg);
SHELL_ENGINE_CMD_DECLARE(fault);
SHELL_ENGINE_CMD_DECLARE(help);
SHELL_ENGINE_CMD_DECLARE(i2c);
SHELL_ENGINE_CMD_DECLARE(i2c_dma);
SHELL_ENGINE_CMD_DECLARE(id);
SHELL_ENGINE_CMD_DECLARE(ifconfig);
SHELL_ENGINE_CMD_DECLARE(imu);
SHELL_ENGINE_CMD_DECLARE(kv);
SHELL_ENGINE_CMD_DECLARE(nina);
SHELL_ENGINE_CMD_DECLARE(ntp);
SHELL_ENGINE_CMD_DECLARE(reboot);
SHELL_ENGINE_CMD_DECLARE(sensor);
SHELL_ENGINE_CMD_DECLARE(sleep);
SHELL_ENGINE_CMD_DECLARE(uptime);
SHELL_ENGINE_CMD_DECLARE(wifi);
SHELL_ENGINE_CMD_DECLARE(zperf);

/* All the commands, sorted by name (Zephyr's "help" lists them so). */
/* One command per line. */
/* clang-format off */
const struct shell_engine_cmd shell_engine_cmds[] = {
	SHELL_ENGINE_CMD(bootsel),
	SHELL_ENGINE_CMD(date),
	SHELL_ENGINE_CMD(devmem),
	SHELL_ENGINE_CMD(dmesg),
	SHELL_ENGINE_CMD(fault),
	SHELL_ENGINE_CMD(free),
	SHELL_ENGINE_CMD(help),
	SHELL_ENGINE_CMD(i2c),
	SHELL_ENGINE_CMD(i2c_dma),
	SHELL_ENGINE_CMD(id),
	SHELL_ENGINE_CMD(ifconfig),
	SHELL_ENGINE_CMD(imu),
	SHELL_ENGINE_CMD(kv),
	SHELL_ENGINE_CMD(nina),
	SHELL_ENGINE_CMD(ntp),
	SHELL_ENGINE_CMD(ps),
	SHELL_ENGINE_CMD(reboot),
	SHELL_ENGINE_CMD(sensor),
	SHELL_ENGINE_CMD(sleep),
	SHELL_ENGINE_CMD(uptime),
	SHELL_ENGINE_CMD(version),
	SHELL_ENGINE_CMD(wifi),
	SHELL_ENGINE_CMD(zperf),
};
/* clang-format on */

const size_t shell_engine_cmd_count = sizeof(shell_engine_cmds) / sizeof(shell_engine_cmds[0]);
