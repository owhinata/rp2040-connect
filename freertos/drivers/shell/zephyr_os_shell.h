/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of <zephyr/shell/shell.h> of the Zephyr compatibility
 * layer (common/compat/zephyr/shell/shell.h, which includes this file):
 * struct shell, a shell backend.  On FreeRTOS a backend is a console, the
 * terminal of one shell session (nano_console.h, which holds the struct):
 * it lives as long as the system, as Zephyr's shell instances do, and the
 * output to it from any task goes to that terminal (zshell_os_impl.h).
 * NuttX's is nuttx/drivers/compat/zephyr_os_shell.h.
 */

#ifndef FREERTOS_DRIVERS_SHELL_ZEPHYR_OS_SHELL_H
#define FREERTOS_DRIVERS_SHELL_ZEPHYR_OS_SHELL_H

struct nano_console;

struct shell {
	struct nano_console *console;
};

#endif /* FREERTOS_DRIVERS_SHELL_ZEPHYR_OS_SHELL_H */
