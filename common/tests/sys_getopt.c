/* SPDX-License-Identifier: MIT */
/* Host test of the copies of Zephyr's getopt (compat/lib/utils/getopt:
 * getopt.c, getopt_long.c, built as they are, with CONFIG_GETOPT_LONG=1):
 * the parse of short and long options, the permutation of the words that
 * are not options, the errors, and that two states parse two command lines
 * interleaved, one option at a time, as two shell sessions do (the OS
 * returns the state of the calling session from sys_getopt_state_get();
 * here the test chooses it).
 */

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/sys/sys_getopt.h>

#include "app_log.h"
#include "getopt_common.h"

static struct sys_getopt_state states[2];
static struct sys_getopt_state *current = &states[0];
static int warnings;

int sys_getopt_opterr = 1;
int sys_getopt_optind = 1;
int sys_getopt_optopt;
int sys_getopt_optreset;
char *sys_getopt_optarg;

struct sys_getopt_state *sys_getopt_state_get(void)
{
	return current;
}

void z_getopt_global_state_update(struct sys_getopt_state *state)
{
	sys_getopt_optind = state->optind;
	sys_getopt_optopt = state->optopt;
	sys_getopt_optarg = state->optarg;
}

void app_log_vprintf(int level, const char *module, const char *func, const char *fmt, va_list ap)
{
	(void)module;
	(void)func;
	(void)fmt;
	(void)ap;
	if (level == LOG_LEVEL_WRN) {
		warnings++;
	}
}

void app_log_printf(int level, const char *module, const char *func, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	app_log_vprintf(level, module, func, fmt, ap);
	va_end(ap);
}

bool app_log_ratelimit_pass(uint32_t *last_ms, uint32_t *skipped, uint32_t rate_ms,
			    uint32_t *skipped_out)
{
	(void)last_ms;
	(void)skipped;
	(void)rate_ms;
	*skipped_out = 0;
	return true;
}

/* "wifi cred add"'s options (Zephyr's wifi_credentials_shell.c). */
static const struct sys_getopt_option long_options[] = {
	{"ssid", sys_getopt_required_argument, 0, 's'},
	{"passphrase", sys_getopt_required_argument, 0, 'p'},
	{"key-mgmt", sys_getopt_required_argument, 0, 'k'},
	{"help", sys_getopt_no_argument, 0, 'h'},
	{0, 0, 0, 0},
};

static int next(int argc, char **argv)
{
	int index = 0;

	return sys_getopt_long(argc, argv, "s:p:k:h", long_options, &index);
}

static void use(int session)
{
	current = &states[session];
}

int main(void)
{
	char a0[] = "add", a1[] = "-s", a2[] = "home net", a3[] = "extra", a4[] = "--key-mgmt",
	     a5[] = "1", a6[] = "--passphrase=secret";
	char *a[] = {a0, a1, a2, a3, a4, a5, a6, NULL};
	char b0[] = "add", b1[] = "-k0", b2[] = "-x", b3[] = "--ssid", b4[] = "office";
	char *b[] = {b0, b1, b2, b3, b4, NULL};
	char c0[] = "add", c1[] = "-s";
	char *c[] = {c0, c1, NULL};

	/* Two sessions, one option of each in turn. */
	use(0);
	sys_getopt_init();
	use(1);
	sys_getopt_init();

	use(0);
	assert(next(7, a) == 's');
	assert(strcmp(sys_getopt_state_get()->optarg, "home net") == 0);
	use(1);
	assert(next(5, b) == 'k');
	assert(strcmp(sys_getopt_state_get()->optarg, "0") == 0);
	use(0);
	assert(next(7, a) == 'k'); /* "extra" is skipped, moved to the end later */
	assert(strcmp(sys_getopt_state_get()->optarg, "1") == 0);
	use(1);
	assert(next(5, b) == '?'); /* -x is unknown: a warning in the log */
	assert(sys_getopt_state_get()->optopt == 'x');
	assert(warnings == 1);
	use(0);
	assert(next(7, a) == 'p');
	assert(strcmp(sys_getopt_state_get()->optarg, "secret") == 0);
	use(1);
	assert(next(5, b) == 's');
	assert(strcmp(sys_getopt_state_get()->optarg, "office") == 0);
	use(0);
	assert(next(7, a) == -1);
	/* The word that is not an option is permuted to the end. */
	assert(sys_getopt_state_get()->optind == 6);
	assert(strcmp(a[6], "extra") == 0);
	use(1);
	assert(next(5, b) == -1);
	assert(sys_getopt_state_get()->optind == 5);

	/* A missing argument; then a new command starts afresh. */
	use(0);
	sys_getopt_init();
	assert(next(2, c) == '?');
	assert(sys_getopt_state_get()->optopt == 's');
	sys_getopt_init();
	assert(sys_getopt_state_get()->optind == 1);
	assert(next(1, c) == -1);

	printf("sys_getopt: ok\n");
	return 0;
}
