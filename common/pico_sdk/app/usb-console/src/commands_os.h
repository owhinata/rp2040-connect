/* SPDX-License-Identifier: MIT */
/* What the commands of the USB console on the Pico SDK (commands.c) need
 * from the OS: "ps" and the kernel's part of "version".  The OS's part,
 * src/commands_os.c of the OS's application directory, defines these;
 * commands.c registers the commands.
 */

#ifndef COMMANDS_OS_H
#define COMMANDS_OS_H

#include <stddef.h>

struct shell;

/* The kernel's name as "version" prints it before the version ("FreeRTOS
 * kernel"), and its version ("V11.3.3"): "version" prints "<name>
 * <version>, Pico SDK <x>, GCC <y>".  From any shell session's thread (two
 * at once).
 */
const char *commands_os_kernel_name(void);
const char *commands_os_kernel_version(void);

/* The handler of "ps": one row per thread (name, state, priority, the least
 * free stack so far in bytes, the share of the CPU time since the previous
 * "ps", common/shell/ps_interval.h), the row "(interrupts)"
 * (irq_stack_size(), irq_stack_used() of app.h), and the CPU load over the
 * same interval (idle_ps_window() of app.h), in one format for every OS.
 * 0, or a negative errno.  From any shell session's thread (two at once).
 */
int commands_os_ps(const struct shell *sh, size_t argc, char **argv);

#endif /* COMMANDS_OS_H */
