/* SPDX-License-Identifier: MIT */
/* The shell engine's part of Zephyr's getopt (<zephyr/sys/sys_getopt.h>;
 * the copies of Zephyr's lib/utils/getopt/getopt.c and getopt_long.c in
 * common/compat/lib/utils/getopt): what Zephyr's getopt_common.c does with
 * CONFIG_SHELL_GETOPT=y, on the consoles of shell_console.h.  A thread
 * bound to a console (a shell session's thread, which runs its commands)
 * uses the session's state, so the serial and the Telnet sessions can parse
 * options at the same time; any other thread uses one common state.  The
 * shell resets the session's state before each command (sys_getopt_init()
 * in shell_engine.c, as Zephyr's shell does).  Zephyr's global copies
 * (sys_getopt_optind, ...) are kept as there, valid only while one thread
 * uses getopt.
 */

#include <zephyr/sys/sys_getopt.h>

#include "getopt_common.h"
#include "shell_console.h"

int sys_getopt_opterr = 1;
int sys_getopt_optind = 1;
int sys_getopt_optopt;
int sys_getopt_optreset;
char *sys_getopt_optarg;

static struct sys_getopt_state common_state = {
	.opterr = 1,
	.optind = 1,
	.optopt = 0,
	.optreset = 0,
	.optarg = NULL,
	.place = "",
#if CONFIG_GETOPT_LONG
	.nonopt_start = -1,
	.nonopt_end = -1,
#endif
};

void z_getopt_global_state_update(struct sys_getopt_state *state)
{
	sys_getopt_opterr = state->opterr;
	sys_getopt_optind = state->optind;
	sys_getopt_optopt = state->optopt;
	sys_getopt_optreset = state->optreset;
	sys_getopt_optarg = state->optarg;
}

struct sys_getopt_state *sys_getopt_state_get(void)
{
	struct shell_console *con = shell_console_bound();

	return con != NULL ? &con->getopt : &common_state;
}
