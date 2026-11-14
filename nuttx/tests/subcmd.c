/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of nuttx/apps/subcmd/subcmd.c: the Zephyr shell's output for
 * the "kv" and "nina" command tables (SHELL_CMD_ARG() and SHELL_CMD()), as
 * recorded from the Zephyr build (host/recordings/zephyr-reference-outputs.txt
 * for "kv") or derived from zephyr/subsys/shell/shell.c for the rest, and a
 * nested set.
 */

#define _POSIX_C_SOURCE 200809L

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "subcmd.h"

static char out[4096], err[4096];
static int calls, last_argc;
static char last_argv[8][32];

static int handler(int argc, char **argv)
{
	calls++;
	last_argc = argc;
	for (int i = 0; i < argc && i < 8; i++) {
		snprintf(last_argv[i], sizeof(last_argv[i]), "%s", argv[i]);
	}
	return strcmp(argv[0], "fail") == 0 ? -5 : 0;
}

static const struct subcmd kv_commands[] = {
	SUBCMD_ARG(set, NULL, "Store a text value: <key> <value>", handler, 3, 0),
	SUBCMD_ARG(sethex, NULL, "Store hex bytes: <key> <hex>", handler, 3, 0),
	SUBCMD_ARG(get, NULL, "Print a value as hex: <key>", handler, 2, 0),
	SUBCMD_ARG(delete, NULL, "Delete a value: <key>", handler, 2, 0),
	SUBCMD_SET_END
};
static const struct subcmd kv_root =
	SUBCMD(kv, kv_commands, "Application key-value storage", NULL);

static const struct subcmd nina_commands[] = {
	SUBCMD(version, NULL, "Show NINA firmware version", handler),
	SUBCMD(fail, NULL, "Fails", handler),
	SUBCMD_SET_END
};
static const struct subcmd nina_root =
	SUBCMD(nina, nina_commands, "NINA Wi-Fi module", NULL);

/* A nested set, a handler with optional arguments and one without a check. */
static const struct subcmd cred_commands[] = {
	SUBCMD_ARG(add, NULL, "Add: -s <ssid> [-p <pass>]", handler, 3, 2),
	SUBCMD_ARG(list, NULL, "List", handler, 1, 0),
	SUBCMD_ARG(any, NULL, "Any count", handler, 2, SUBCMD_OPT_ANY),
	SUBCMD_SET_END
};
static const struct subcmd wifi_commands[] = {
	SUBCMD(cred, cred_commands, "Credentials", NULL),
	SUBCMD(status, NULL, "Status", handler),
	SUBCMD_SET_END
};
static const struct subcmd wifi_root = SUBCMD(wifi, wifi_commands, "Wi-Fi", NULL);

/* A root command with a handler of its own and subcommands. */
static const struct subcmd cal_commands[] = {
	SUBCMD(reload, NULL, "Reload", handler),
	SUBCMD_SET_END
};
static const struct subcmd imu_commands[] = {
	SUBCMD_ARG(cal, cal_commands, "Show: [reload|zero]", handler, 1, 1),
	SUBCMD_SET_END
};
static const struct subcmd imu_root = SUBCMD(imu, imu_commands, "IMU", NULL);

/* Help texts with newlines and longer than the 80 columns of the Zephyr
 * shell's terminal (formatted_text_print() of shell_help.c).
 */
#define WORDS8 "0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 " \
	       "0123456789 0123456789"
static const struct subcmd text_commands[] = {
	SUBCMD(nl, NULL, "first\nsecond\n", handler),
	SUBCMD(long, NULL, WORDS8, handler),
	SUBCMD_SET_END
};
static const struct subcmd text_root = SUBCMD(txt, text_commands, "Text\nmore", NULL);

static void slurp(FILE *f, char *buf, size_t size)
{
	size_t n;

	fflush(f);
	rewind(f);
	n = fread(buf, 1, size - 1, f);
	buf[n] = '\0';
}

/* Run ROOT with the words, capturing stdout and stderr. */
static int run(const struct subcmd *root, ...)
{
	char *argv[16];
	int argc = 0, ret, saved_out, saved_err;
	FILE *fo = tmpfile(), *fe = tmpfile();
	va_list ap;

	assert(fo && fe);
	va_start(ap, root);
	while ((argv[argc] = va_arg(ap, char *)) != NULL) {
		argc++;
	}
	va_end(ap);
	calls = 0;
	last_argc = -1;
	fflush(stdout);
	fflush(stderr);
	saved_out = dup(1);
	saved_err = dup(2);
	dup2(fileno(fo), 1);
	dup2(fileno(fe), 2);
	ret = subcmd_main(root, argc, argv);
	fflush(stdout);
	fflush(stderr);
	dup2(saved_out, 1);
	dup2(saved_err, 2);
	close(saved_out);
	close(saved_err);
	slurp(fo, out, sizeof(out));
	slurp(fe, err, sizeof(err));
	fclose(fo);
	fclose(fe);
	return ret;
}

static const char kv_help[] =
	"kv - Application key-value storage\n"
	"Subcommands:\n"
	"  set     : Store a text value: <key> <value>\n"
	"  sethex  : Store hex bytes: <key> <hex>\n"
	"  get     : Print a value as hex: <key>\n"
	"  delete  : Delete a value: <key>\n";

static const char nina_help[] =
	"nina - NINA Wi-Fi module\n"
	"Subcommands:\n"
	"  version  : Show NINA firmware version\n"
	"  fail     : Fails\n";

int main(void)
{
	/* kv: help for the command alone or an unknown word (the reference
	 * recording of the Zephyr build for "kv").
	 */
	assert(run(&kv_root, "kv", NULL) == EXIT_FAILURE);
	assert(strcmp(out, kv_help) == 0 && err[0] == '\0' && calls == 0);
	assert(run(&kv_root, "kv", "foo", "a", NULL) == EXIT_FAILURE);
	assert(strcmp(out, kv_help) == 0 && calls == 0);
	assert(run(&kv_root, "kv", "-h", NULL) == EXIT_FAILURE && strcmp(out, kv_help) == 0);
	assert(run(&kv_root, "kv", "s*", NULL) == EXIT_FAILURE && strcmp(out, kv_help) == 0);
	assert(run(&kv_root, "kv", "x*", NULL) == EXIT_FAILURE && strcmp(out, kv_help) == 0);
	/* The handler gets its own name and the arguments. */
	assert(run(&kv_root, "kv", "set", "k", "v", NULL) == EXIT_SUCCESS);
	assert(calls == 1 && last_argc == 3 && !strcmp(last_argv[0], "set") &&
	       !strcmp(last_argv[1], "k") && !strcmp(last_argv[2], "v"));
	assert(out[0] == '\0' && err[0] == '\0');
	/* Wrong counts. */
	assert(run(&kv_root, "kv", "get", NULL) == EXIT_FAILURE && calls == 0);
	assert(strcmp(err, "get: wrong parameter count\n") == 0);
	assert(strcmp(out, "get - Print a value as hex: <key>\n") == 0);
	assert(run(&kv_root, "kv", "set", "k", NULL) == EXIT_FAILURE && calls == 0);
	assert(strcmp(err, "set: wrong parameter count\n") == 0);
	assert(strcmp(out, "set - Store a text value: <key> <value>\n") == 0);
	assert(run(&kv_root, "kv", "delete", "a", "b", NULL) == EXIT_FAILURE && calls == 0);
	/* A wildcard right after the subcommand ends the words there. */
	assert(run(&kv_root, "kv", "get", "a*", NULL) == EXIT_FAILURE && calls == 0);
	assert(strcmp(err, "get: wrong parameter count\n") == 0);
	assert(run(&kv_root, "kv", "set", "k?", "v", NULL) == EXIT_FAILURE && calls == 0);
	/* Only right after it. */
	assert(run(&kv_root, "kv", "set", "k", "v*", NULL) == EXIT_SUCCESS && calls == 1);
	assert(!strcmp(last_argv[2], "v*"));
	/* "-h" is an ordinary argument. */
	assert(run(&kv_root, "kv", "get", "-h", NULL) == EXIT_SUCCESS && calls == 1);

	/* nina: SHELL_CMD() checks no count. */
	assert(run(&nina_root, "nina", NULL) == EXIT_FAILURE && strcmp(out, nina_help) == 0);
	assert(run(&nina_root, "nina", "foo", NULL) == EXIT_FAILURE &&
	       strcmp(out, nina_help) == 0 && calls == 0);
	assert(run(&nina_root, "nina", "version", NULL) == EXIT_SUCCESS && last_argc == 1);
	assert(run(&nina_root, "nina", "version", "x", "y", NULL) == EXIT_SUCCESS &&
	       last_argc == 3 && !strcmp(last_argv[2], "y"));
	assert(run(&nina_root, "nina", "version", "*", NULL) == EXIT_SUCCESS && last_argc == 1);
	assert(run(&nina_root, "nina", "v*", NULL) == EXIT_FAILURE && calls == 0 &&
	       strcmp(out, nina_help) == 0);
	assert(run(&nina_root, "nina", "fail", NULL) == EXIT_FAILURE && calls == 1);

	/* Nested sets: the deepest entry's help, optional counts. */
	assert(run(&wifi_root, "wifi", "cred", NULL) == EXIT_FAILURE);
	assert(strcmp(out, "cred - Credentials\nSubcommands:\n"
			   "  add   : Add: -s <ssid> [-p <pass>]\n"
			   "  list  : List\n"
			   "  any   : Any count\n") == 0);
	assert(run(&wifi_root, "wifi", "cred", "nope", NULL) == EXIT_FAILURE &&
	       !strncmp(out, "cred - ", 7));
	assert(run(&wifi_root, "wifi", "cred", "add", "-s", "x", NULL) == EXIT_SUCCESS &&
	       last_argc == 3 && !strcmp(last_argv[0], "add"));
	assert(run(&wifi_root, "wifi", "cred", "add", "-s", "x", "-p", "y", NULL) ==
	       EXIT_SUCCESS && last_argc == 5);
	assert(run(&wifi_root, "wifi", "cred", "add", "-s", "x", "-p", NULL) == EXIT_SUCCESS);
	assert(run(&wifi_root, "wifi", "cred", "add", "-s", "x", "-p", "y", "z", NULL) ==
	       EXIT_FAILURE && !strcmp(err, "add: wrong parameter count\n"));
	assert(run(&wifi_root, "wifi", "cred", "add", "-s", NULL) == EXIT_FAILURE);
	assert(run(&wifi_root, "wifi", "cred", "list", "x", NULL) == EXIT_FAILURE);
	assert(run(&wifi_root, "wifi", "cred", "any", "1", "2", "3", "4", "5", NULL) ==
	       EXIT_SUCCESS && last_argc == 6);
	assert(run(&wifi_root, "wifi", "cred", "any", NULL) == EXIT_FAILURE);
	assert(run(&wifi_root, "wifi", "status", "x", NULL) == EXIT_SUCCESS);

	/* An entry with a handler and subcommands: the words that are not
	 * subcommands are its arguments; a subcommand runs instead.
	 */
	assert(run(&imu_root, "imu", "cal", NULL) == EXIT_SUCCESS && last_argc == 1);
	assert(run(&imu_root, "imu", "cal", "zero", NULL) == EXIT_SUCCESS && last_argc == 2 &&
	       !strcmp(last_argv[0], "cal") && !strcmp(last_argv[1], "zero"));
	assert(run(&imu_root, "imu", "cal", "reload", NULL) == EXIT_SUCCESS && last_argc == 1 &&
	       !strcmp(last_argv[0], "reload"));
	assert(run(&imu_root, "imu", "cal", "a", "b", NULL) == EXIT_FAILURE &&
	       !strcmp(err, "cal: wrong parameter count\n") &&
	       !strcmp(out, "cal - Show: [reload|zero]\nSubcommands:\n  reload  : Reload\n"));
	/* "z*" matches no subcommand of cal: the words end before it. */
	assert(run(&imu_root, "imu", "cal", "z*", NULL) == EXIT_SUCCESS && last_argc == 1);

	/* Text layout: continuation lines start at the help's column, a long
	 * line breaks at the last space before column 80, a final newline
	 * leaves an empty indented line.
	 */
	assert(run(&text_root, "txt", NULL) == EXIT_FAILURE);
	assert(!strcmp(out, "txt - Text\n      more\nSubcommands:\n"
			    "  nl    : first\n          second\n          \n"
			    "  long  : 0123456789 0123456789 0123456789 0123456789 0123456789 "
			    "0123456789\n          0123456789 0123456789\n"));
	assert(run(&text_root, "txt", "long", "a", NULL) == EXIT_SUCCESS);

	puts("subcmd tests passed");
	return 0;
}
