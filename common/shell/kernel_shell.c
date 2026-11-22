/* SPDX-License-Identifier: MIT */
/* The generic shell commands of the builds that run the Zephyr shell's
 * commands on the compatibility layer's shell (the FreeRTOS build,
 * freertos/app/usb-console/CMakeLists.txt; the NuttX build has nsh's and
 * its own applications): "id", "uptime", "date", "sleep", "devmem",
 * "fault", "reboot" and "bootsel", written on Zephyr's shell API and the
 * layer, with the little they need from the OS behind kernel_shell_os.h.
 * The application lists them in its command table.
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/clock.h>

#include "fault_record.h"
#include "kernel_shell_os.h"

/* Time for the terminal to get a command's last message before a restart. */
#define RESTART_DELAY_MS 100

/* The longest device ID shown (Zephyr's hwinfo shell takes 16 bytes). */
#define ID_SIZE_MAX 16

/* The device ID of the layer's hwinfo, in upper-case hexadecimal (on the
 * RP2040 the flash's 64-bit unique ID, as the Pico SDK's
 * pico_get_unique_board_id_string() writes it).
 */
static int cmd_id(const struct shell *sh, size_t argc, char **argv)
{
	static const char digits[] = "0123456789ABCDEF";
	uint8_t id[ID_SIZE_MAX];
	char text[2 * ID_SIZE_MAX + 1];
	ssize_t n;

	(void)argc;
	(void)argv;

	n = hwinfo_get_device_id(id, sizeof(id));
	if (n <= 0) {
		shell_error(sh, "Could not read the device ID");
		return n < 0 ? (int)n : -EIO;
	}
	for (ssize_t i = 0; i < n; i++) {
		text[2 * i] = digits[id[i] >> 4];
		text[2 * i + 1] = digits[id[i] & 0xf];
	}
	text[2 * n] = '\0';
	shell_print(sh, "%s", text);
	return 0;
}

SHELL_CMD_REGISTER(id, NULL, "Show the board's unique ID (of the flash)", cmd_id);

static int cmd_uptime(const struct shell *sh, size_t argc, char **argv)
{
	uint64_t us = k_cycle_get_64();

	(void)argc;
	(void)argv;

	shell_print(sh, "%llu.%06llu s (TIMER), %lu ticks of 1/%d s",
		    (unsigned long long)(us / 1000000u), (unsigned long long)(us % 1000000u),
		    (unsigned long)kernel_shell_os_tick_count(), (int)(1000000 / Z_USEC_PER_TICK));
	return 0;
}

SHELL_CMD_REGISTER(uptime, NULL, "Show the time since the start (TIMER) and the ticks", cmd_uptime);

/* The wall clock (UTC, set by the time synchronization), in the format of the
 * NuttX build's "date".  It counts from 1970 until the time is known.
 */
static int cmd_date(const struct shell *sh, size_t argc, char **argv)
{
	struct timespec ts;
	struct tm tm;
	char text[32];

	(void)argc;
	(void)argv;

	if (sys_clock_gettime(SYS_CLOCK_REALTIME, &ts) != 0 || gmtime_r(&ts.tv_sec, &tm) == NULL ||
	    strftime(text, sizeof(text), "%a, %b %d %H:%M:%S %Y", &tm) == 0) {
		shell_error(sh, "Could not read the time");
		return -EIO;
	}
	shell_print(sh, "%s", text);
	return 0;
}

SHELL_CMD_ARG_REGISTER(date, NULL, "Show the date and time (UTC)", cmd_date, 1, 0);

/* Zephyr's "kernel sleep <ms>" with its messages
 * (subsys/shell/modules/kernel_service): k_msleep(ms), here k_sleep() of
 * K_MSEC() so that every value of an unsigned long counts.
 */
static int cmd_sleep(const struct shell *sh, size_t argc, char **argv)
{
	char *end;
	unsigned long ms;

	(void)argc;
	errno = 0;
	ms = strtoul(argv[1], &end, 10);
	if (errno != 0 || end == argv[1] || *end != '\0') {
		shell_error(sh, "Unable to parse input (err %d)", -EINVAL);
		return -EINVAL;
	}
	(void)k_sleep(K_MSEC((int64_t)ms));
	return 0;
}

SHELL_CMD_ARG_REGISTER(sleep, NULL, "Sleep for <ms> milliseconds (Zephyr's kernel sleep)",
		       cmd_sleep, 2, 0);

/* Zephyr's "devmem" (subsys/shell/modules/devmem_service.c): the same
 * arguments (address and value in hexadecimal, width in bits) and the same
 * output.  Reads or writes any address: one that does not exist ends in a
 * fatal error ("fault" after the restart, where the OS keeps a record).
 */
static int cmd_devmem(const struct shell *sh, size_t argc, char **argv)
{
	uintptr_t addr;
	unsigned long width = 32;
	uint32_t value;

	addr = (uintptr_t)strtoul(argv[1], NULL, 16);
	if (argc >= 3) {
		width = strtoul(argv[2], NULL, 10);
	}
	shell_print(sh, "Using data width %lu", width);
	if (width != 8 && width != 16 && width != 32) {
		shell_error(sh, "Incorrect data width");
		return -EINVAL;
	}
	if (argc <= 3) {
		if (width == 8) {
			value = *(volatile uint8_t *)addr;
		} else if (width == 16) {
			value = *(volatile uint16_t *)addr;
		} else {
			value = *(volatile uint32_t *)addr;
		}
		shell_print(sh, "Read value 0x%lx", (unsigned long)value);
		return 0;
	}
	value = (uint32_t)strtoul(argv[3], NULL, 16);
	shell_print(sh, "Writing value 0x%lx", (unsigned long)value);
	if (width == 8) {
		*(volatile uint8_t *)addr = (uint8_t)value;
	} else if (width == 16) {
		*(volatile uint16_t *)addr = (uint16_t)value;
	} else {
		*(volatile uint32_t *)addr = value;
	}
	return 0;
}

SHELL_CMD_ARG_REGISTER(devmem, NULL,
		       "Read or write memory: devmem <address> [<width> [<value>]]\n"
		       "(address and value in hexadecimal, width 8, 16 or 32 bits)",
		       cmd_devmem, 2, 2);

/* The fatal error that ended the run before this start. */
static int cmd_fault(const struct shell *sh, size_t argc, char **argv)
{
	const struct fault_record *r = kernel_shell_os_fault_last();

	(void)argc;
	(void)argv;

	if (r == NULL) {
		shell_print(sh, "No fatal error before this start");
		return 0;
	}
	fault_record_print(r);
	return 0;
}

SHELL_CMD_REGISTER(fault, NULL, "Show the fatal error that ended the last run", cmd_fault);

/* Zephyr's "kernel reboot": restarts at once, whatever the arguments. */
static int cmd_reboot(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	shell_print(sh, "Rebooting");
	kernel_shell_os_reboot(RESTART_DELAY_MS);
	return 0;
}

SHELL_CMD_REGISTER(reboot, NULL, "Restart the board", cmd_reboot);

/* Restarts into the boot ROM's USB loader (the RPI-RP2 drive), as holding the
 * board's REC pad low at reset does.
 */
static int cmd_bootsel(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	shell_print(sh, "Entering BOOTSEL (the RPI-RP2 drive)");
	kernel_shell_os_bootsel(RESTART_DELAY_MS);
	return 0;
}

SHELL_CMD_REGISTER(bootsel, NULL, "Restart into the USB bootloader (BOOTSEL)", cmd_bootsel);
