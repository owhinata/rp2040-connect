/* SPDX-License-Identifier: MIT */
/* What main.c and commands.c of the USB console share. */

#ifndef APP_H
#define APP_H

#include <stddef.h>
#include "cli.h"

/* The command line of the console task (main.c). */
extern struct cli console_cli;

/* The commands (commands.c). */
extern const struct cli_command console_commands[];
extern const size_t console_command_count;

#endif /* APP_H */
