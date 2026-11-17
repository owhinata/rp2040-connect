/* SPDX-License-Identifier: MIT */
/* What the sources of the USB console share. */

#ifndef APP_H
#define APP_H

#include <stddef.h>
#include <stdint.h>
#include "cli.h"
#include "fault_record.h"

/* The command line of the console task (main.c). */
extern struct cli console_cli;

/* The commands (commands.c). */
extern const struct cli_command console_commands[];
extern const size_t console_command_count;

/* Writes the lines of a fatal error's record to the console (commands.c). */
void fault_print(const struct fault_record *r);

/* The CPU load since the previous call (since the start for the first), in
 * tenths of a percent, as cpu_load_get(): for "ps", which does not reset
 * cpu_load_get()'s window.  *total_us is the length of the window (idle.c).
 */
int idle_ps_window(uint64_t *total_us);

/* The interrupt stack (the main stack, irq_stack.c): paint it first in
 * main(), then read its lowest address, size and high-water mark in bytes.
 */
void irq_stack_paint(void);
uintptr_t irq_stack_base(void);
size_t irq_stack_size(void);
size_t irq_stack_used(void);

#endif /* APP_H */
