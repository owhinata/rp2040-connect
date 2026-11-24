/* SPDX-License-Identifier: MIT */
/* What the commands of the USB console on the Pico SDK (commands.c) need
 * from the OS.  commands.c includes this header before its own commands; it
 * includes the OS's part, "commands_os_impl.h" (in the OS's application
 * directory), which defines:
 *
 *   COMMANDS_OS_KERNEL_NAME     a string literal, the kernel's name as
 *                               "version" prints it before the version
 *                               ("FreeRTOS kernel")
 *   COMMANDS_OS_KERNEL_VERSION  an expression of type const char *, the
 *                               kernel's version ("V11.3.3"); "version"
 *                               prints "<name> <version>, Pico SDK <x>,
 *                               GCC <y>"
 *   cmd_ps()                    static int cmd_ps(const struct shell *sh,
 *                               size_t argc, char **argv), the handler of
 *                               "ps" (commands.c registers it): one row per
 *                               thread (name, state,
 *                               priority, the least free stack so far in
 *                               bytes, the share of the CPU time since the
 *                               previous "ps", common/shell/ps_interval.h),
 *                               the row "(interrupts)" (irq_stack_size(),
 *                               irq_stack_used() of app.h), and the CPU load
 *                               over the same interval (idle_ps_window() of
 *                               app.h), in the FreeRTOS build's format;
 *                               0, or a negative errno.  From any shell
 *                               session's thread (two at once)
 */

#ifndef COMMANDS_OS_H
#define COMMANDS_OS_H

#include "commands_os_impl.h"

#endif /* COMMANDS_OS_H */
