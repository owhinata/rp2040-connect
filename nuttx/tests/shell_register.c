/* SPDX-License-Identifier: MIT */
/* Host test of the "imu", "ntp" and "kv" commands' registration: the
 * SHELL_STATIC_SUBCMD_SET_CREATE() ... SHELL_CMD_REGISTER() text of
 * common/imu/imu_stream.c, common/imu/time_sync.c and common/imu/kv_store.c
 * (cut out of the files by script/host_tests.sh, IMU_SHELL_INC,
 * NTP_SHELL_INC and KV_SHELL_INC; "kv" with the "wipe" entry of
 * common/compat/include/kv_store_os.h), compiled with
 * the compatibility layer's <zephyr/shell/shell.h> and
 * Z_SHELL_CMD_NAMED_MAIN and run on common/shell/zshell.c (with the NuttX
 * backends, apps/subcmd/zshell_os_impl.h), against the same sets written
 * as common/shell/subcmd.h tables (subcmd_main(), as the NuttX
 * build ran these commands before they were shared with the Zephyr
 * build: the handlers then got sh == NULL; "kv" as nuttx/apps/kv/kv_main.c
 * had it, with "wipe" added).
 *
 * For each command line, both must give the same exit status, the same
 * handler calls with the same argc and argv, and the same bytes on stdout,
 * on stderr and, with both on one file, in the same order.  The handlers
 * print complete lines on stdout and stderr through shell_print(),
 * shell_warn() and shell_error() with the sh they get (NULL before, the
 * terminal's backend now) and one line longer than the 256-byte buffer of
 * the NuttX backend (zshell_os_impl.h).  stdout is line buffered with
 * NuttX's 64-byte buffer (CONFIG_STDIO_LINEBUFFER, CONFIG_STDIO_BUFFER_SIZE).
 *
 * Also the cases where the two differ, with the Zephyr shell's output
 * (zephyr/subsys/shell/shell.c execute()): a wildcard that matches
 * subcommands followed by a subcommand with a handler, and more than
 * CONFIG_SHELL_ARGC_MAX (30) words.
 */

#define _GNU_SOURCE

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <zephyr/shell/shell.h>

#include "kv_store_os.h"
#include "subcmd.h"

/* Handler calls ----------------------------------------------------------*/

struct call {
	int calls;
	const struct shell *sh;
	size_t argc;
	char argv[8][32];
	char last[64];     /* argv[argc] (NULL: "-") */
};

static struct call got;

static int record(const char *name, const struct shell *sh, size_t argc, char **argv)
{
	char line[301];
	size_t i;

	got.calls++;
	got.sh = sh;
	got.argc = argc;
	for (i = 0; i < argc && i < 8; i++) {
		snprintf(got.argv[i], sizeof(got.argv[i]), "%s", argv[i]);
	}
	snprintf(got.last, sizeof(got.last), "%s", argv[argc] ? argv[argc] : "-");
	shell_print(sh, "%s: argc %zu", name, argc);
	for (i = 0; i < argc; i++) {
		shell_print(sh, "  argv[%zu] = %s", i, argv[i]);
	}
	shell_warn(sh, "%s: a warning", name);
	shell_print(sh, "%s: a line of normal output after it, longer than the 64-byte buffer", name);
	shell_error(sh, "%s: an error", name);
	memset(line, 'x', sizeof(line) - 1);
	line[sizeof(line) - 1] = '\0';
	shell_print(sh, "%s", line);
	return strcmp(argv[argc - 1], "fail") == 0 ? -EINVAL : 0;
}

/* The sets as subcmd tables (the NuttX build before) ----------------------*/

static int ref_start(int argc, char **argv)
{
	return record("start", NULL, argc, argv);
}

static int ref_stop(int argc, char **argv)
{
	return record("stop", NULL, argc, argv);
}

static int ref_cal(int argc, char **argv)
{
	return record("cal", NULL, argc, argv);
}

static int ref_status(int argc, char **argv)
{
	return record("status", NULL, argc, argv);
}

static int ref_ntp_status(int argc, char **argv)
{
	return record("ntp status", NULL, argc, argv);
}

static const struct subcmd ref_imu_commands[] = {
	SUBCMD_ARG(start, NULL, "Start streaming to the MQTT server (settings: kv imu.*)",
		   ref_start, 1, 0),
	SUBCMD_ARG(stop, NULL, "Stop streaming", ref_stop, 1, 0),
	SUBCMD_ARG(cal, NULL,
		   "Show the conversion used by sensor get/stream: [reload|zero]",
		   ref_cal, 1, 1),
	SUBCMD_ARG(status, NULL, "Show counters and errors of the current or last run",
		   ref_status, 1, 0),
	SUBCMD_SET_END
};
static const struct subcmd ref_imu_root =
	SUBCMD(imu, ref_imu_commands, "IMU RAW streaming", NULL);

static const struct subcmd ref_ntp_commands[] = {
	SUBCMD(status, NULL, "Show time synchronization status", ref_ntp_status),
	SUBCMD_SET_END
};
static const struct subcmd ref_ntp_root =
	SUBCMD(ntp, ref_ntp_commands, "Network time synchronization", NULL);

static int ref_kv_set(int argc, char **argv)
{
	return record("set", NULL, argc, argv);
}

static int ref_kv_sethex(int argc, char **argv)
{
	return record("sethex", NULL, argc, argv);
}

static int ref_kv_get(int argc, char **argv)
{
	return record("get", NULL, argc, argv);
}

static int ref_kv_delete(int argc, char **argv)
{
	return record("delete", NULL, argc, argv);
}

static int ref_kv_wipe(int argc, char **argv)
{
	return record("wipe", NULL, argc, argv);
}

static const struct subcmd ref_kv_commands[] = {
	SUBCMD_ARG(set, NULL, "Store a text value: <key> <value>", ref_kv_set, 3, 0),
	SUBCMD_ARG(sethex, NULL, "Store hex bytes: <key> <hex>", ref_kv_sethex, 3, 0),
	SUBCMD_ARG(get, NULL, "Print a value as hex: <key>", ref_kv_get, 2, 0),
	SUBCMD_ARG(delete, NULL, "Delete a value: <key>", ref_kv_delete, 2, 0),
	SUBCMD_ARG(wipe, NULL, "Erase the whole store (also Wi-Fi credentials): confirm",
		   ref_kv_wipe, 2, 0),
	SUBCMD_SET_END
};
static const struct subcmd ref_kv_root =
	SUBCMD(kv, ref_kv_commands, "Application key-value storage", NULL);

/* The sources' own text --------------------------------------------------*/

static int cmd_start(const struct shell *sh, size_t argc, char **argv)
{
	return record("start", sh, argc, argv);
}

static int cmd_stop(const struct shell *sh, size_t argc, char **argv)
{
	return record("stop", sh, argc, argv);
}

static int cmd_cal(const struct shell *sh, size_t argc, char **argv)
{
	return record("cal", sh, argc, argv);
}

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	return record("status", sh, argc, argv);
}

static int cmd_ntp_status(const struct shell *sh, size_t argc, char **argv)
{
	return record("ntp status", sh, argc, argv);
}

static int cmd_set(const struct shell *sh, size_t argc, char **argv)
{
	return record("set", sh, argc, argv);
}

static int cmd_sethex(const struct shell *sh, size_t argc, char **argv)
{
	return record("sethex", sh, argc, argv);
}

static int cmd_get(const struct shell *sh, size_t argc, char **argv)
{
	return record("get", sh, argc, argv);
}

static int cmd_delete(const struct shell *sh, size_t argc, char **argv)
{
	return record("delete", sh, argc, argv);
}

int kv_store_cmd_wipe(const struct shell *sh, size_t argc, char **argv)
{
	return record("wipe", sh, argc, argv);
}

#include IMU_SHELL_INC
#include NTP_SHELL_INC
#include KV_SHELL_INC

int z_shell_main_imu(int argc, char *argv[]);
int z_shell_main_ntp(int argc, char *argv[]);
int z_shell_main_kv(int argc, char *argv[]);

/* Running a command line -------------------------------------------------*/

struct result {
	int ret;
	struct call call;
	char out[4096], err[4096], both[8192];
};

static char dir[] = "/tmp/shXXXXXX";
static char out_path[32], err_path[32];

static void slurp(const char *path, char *buf, size_t size)
{
	FILE *f = fopen(path, "r");
	size_t n;

	assert(f);
	n = fread(buf, 1, size - 1, f);
	assert(n < size - 1);
	buf[n] = '\0';
	fclose(f);
}

/* Run the words on subcmd_main(REF) or on MAIN with stdout on out_path and
 * stderr on ERR (out_path again for both on one file).
 */
static int run_once(const struct subcmd *ref, int (*main_fn)(int, char **), int argc,
		    char **argv, const char *err)
{
	int saved_out, saved_err, fd, ret;

	fflush(stdout);
	fflush(stderr);
	saved_out = dup(1);
	saved_err = dup(2);
	fd = open(out_path, O_WRONLY | O_CREAT | O_TRUNC, 0600);
	assert(fd >= 0);
	dup2(fd, 1);
	close(fd);
	if (strcmp(err, out_path) == 0) {
		dup2(1, 2);
	} else {
		fd = open(err, O_WRONLY | O_CREAT | O_TRUNC, 0600);
		assert(fd >= 0);
		dup2(fd, 2);
		close(fd);
	}
	memset(&got, 0, sizeof(got));
	ret = ref ? subcmd_main(ref, argc, argv) : main_fn(argc, argv);
	fflush(stdout);
	fflush(stderr);
	dup2(saved_out, 1);
	dup2(saved_err, 2);
	close(saved_out);
	close(saved_err);
	return ret;
}

static void run(struct result *r, const struct subcmd *ref, int (*main_fn)(int, char **),
		int argc, char **words)
{
	char *argv[40];
	int i;

	/* Each run gets its own copy of the words, NULL-terminated as nsh's. */
	for (i = 0; i < argc; i++) {
		argv[i] = words[i];
	}
	argv[argc] = NULL;
	r->ret = run_once(ref, main_fn, argc, argv, err_path);
	r->call = got;
	slurp(out_path, r->out, sizeof(r->out));
	slurp(err_path, r->err, sizeof(r->err));
	run_once(ref, main_fn, argc, argv, out_path);
	slurp(out_path, r->both, sizeof(r->both));
}

static int split(char *line, char **words)
{
	int n = 0;
	char *w;

	for (w = strtok(line, " "); w; w = strtok(NULL, " ")) {
		words[n++] = w;
	}
	return n;
}

static struct result a, b;
static int shared_sh;

/* LINE runs the same on both (SHARED: the handler's sh is a backend). */
static void same(const char *line)
{
	char buf[256], *words[40];
	int argc;
	const struct subcmd *ref;
	int (*main_fn)(int, char **);
	size_t i;

	snprintf(buf, sizeof(buf), "%s", line);
	argc = split(buf, words);
	if (strcmp(words[0], "imu") == 0) {
		ref = &ref_imu_root;
		main_fn = z_shell_main_imu;
	} else if (strcmp(words[0], "ntp") == 0) {
		ref = &ref_ntp_root;
		main_fn = z_shell_main_ntp;
	} else {
		ref = &ref_kv_root;
		main_fn = z_shell_main_kv;
	}
	run(&a, ref, NULL, argc, words);
	run(&b, NULL, main_fn, argc, words);
	if (a.ret != b.ret || strcmp(a.out, b.out) || strcmp(a.err, b.err) ||
	    strcmp(a.both, b.both) || a.call.calls != b.call.calls ||
	    a.call.argc != b.call.argc || strcmp(a.call.last, b.call.last)) {
		fprintf(stderr, "differs: %s\n--- subcmd (%d)\n%s--\n%s--- zshell (%d)\n%s--\n%s",
			line, a.ret, a.out, a.err, b.ret, b.out, b.err);
		abort();
	}
	for (i = 0; i < a.call.argc && i < 8; i++) {
		assert(strcmp(a.call.argv[i], b.call.argv[i]) == 0);
	}
	assert(a.call.sh == NULL);
	if (b.call.calls) {
		assert(b.call.sh != NULL);
		shared_sh++;
	}
}

static const char imu_help[] =
	"imu - IMU RAW streaming\n"
	"Subcommands:\n"
	"  start   : Start streaming to the MQTT server (settings: kv imu.*)\n"
	"  stop    : Stop streaming\n"
	"  cal     : Show the conversion used by sensor get/stream: [reload|zero]\n"
	"  status  : Show counters and errors of the current or last run\n";

static const char ntp_help[] =
	"ntp - Network time synchronization\n"
	"Subcommands:\n"
	"  status  : Show time synchronization status\n";

int main(void)
{
	static const char *const lines[] = {
		/* Help: the command alone, -h and --help (ordinary words),
		 * unknown subcommands, wildcards.
		 */
		"imu", "imu --help", "imu -h", "imu foo", "imu foo bar", "imu st*", "imu *",
		"imu x*", "imu ?", "imu foo start",
		"ntp", "ntp --help", "ntp -h", "ntp foo", "ntp foo status", "ntp s*", "ntp x*",
		/* Handlers and their argument counts. */
		"imu start", "imu start --help", "imu start x", "imu start x y",
		"imu stop", "imu stop --help", "imu stop x",
		"imu cal", "imu cal reload", "imu cal zero", "imu cal fail", "imu cal --help",
		"imu cal a b", "imu cal a b c",
		"imu status", "imu status --help", "imu status x",
		"ntp status", "ntp status --help", "ntp status x y", "ntp status fail",
		/* A wildcard after a handler's entry ends the words there. */
		"imu start *", "imu cal *", "imu cal z*", "imu cal reload *", "imu status ?",
		"ntp status *", "ntp status x*",
		/* After a word with a wildcard that matches nothing. */
		"imu x* start", "ntp x* status",
		/* kv: help, counts, handlers, wildcards. */
		"kv", "kv --help", "kv -h", "kv foo", "kv foo set", "kv s*", "kv *", "kv x*",
		"kv set", "kv set k", "kv set k v", "kv set k v w", "kv set k fail", "kv set --help",
		"kv sethex", "kv sethex k 01", "kv sethex k 01 02",
		"kv get", "kv get k", "kv get k l", "kv get fail", "kv get k*",
		"kv delete", "kv delete k", "kv delete k l",
		"kv wipe", "kv wipe confirm", "kv wipe confirm x", "kv wipe fail", "kv wipe *",
		"kv x* get k",
	};
	size_t i;

	setvbuf(stdout, NULL, _IOLBF, 64);
	assert(mkdtemp(dir));
	snprintf(out_path, sizeof(out_path), "%s/o", dir);
	snprintf(err_path, sizeof(err_path), "%s/e", dir);

	for (i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
		same(lines[i]);
	}
	assert(shared_sh > 0);

	/* A few of them in full. */
	same("imu");
	assert(a.ret == EXIT_FAILURE && strcmp(b.out, imu_help) == 0 && b.err[0] == '\0');
	same("imu --help");
	assert(strcmp(b.out, imu_help) == 0 && b.call.calls == 0);
	same("ntp");
	assert(strcmp(b.out, ntp_help) == 0 && b.err[0] == '\0');
	same("imu start --help");
	assert(b.ret == EXIT_FAILURE && b.call.calls == 0);
	assert(strcmp(b.err, "start: wrong parameter count\n") == 0);
	assert(strcmp(b.out, "start - Start streaming to the MQTT server (settings: kv imu.*)\n") == 0);
	assert(strcmp(b.both, "start: wrong parameter count\n"
			      "start - Start streaming to the MQTT server (settings: kv imu.*)\n") == 0);
	same("imu cal a b");
	assert(strcmp(b.err, "cal: wrong parameter count\n") == 0);
	assert(strcmp(b.out, "cal - Show the conversion used by sensor get/stream: [reload|zero]\n") == 0);
	same("imu cal zero");
	assert(b.ret == EXIT_SUCCESS && b.call.calls == 1 && b.call.argc == 2 &&
	       strcmp(b.call.argv[0], "cal") == 0 && strcmp(b.call.argv[1], "zero") == 0 &&
	       strcmp(b.call.last, "-") == 0);
	{
		static const char start[] = "cal: argc 2\n  argv[0] = cal\n  argv[1] = zero\n"
					    "cal: a warning\ncal: a line of normal output after it";

		assert(strncmp(b.both, start, strlen(start)) == 0);
	}
	same("imu cal fail");
	assert(b.ret == EXIT_FAILURE && b.call.calls == 1);
	same("ntp status");
	assert(b.ret == EXIT_SUCCESS && b.call.argc == 1 && strcmp(b.call.argv[0], "status") == 0);

	/* Where they differ: zshell.c gives the Zephyr shell's output. */
	{
		char line[] = "imu s* status";
		char *words[4];
		int argc = split(line, words);

		run(&a, &ref_imu_root, NULL, argc, words);
		run(&b, NULL, z_shell_main_imu, argc, words);
		assert(a.ret == EXIT_FAILURE && strcmp(a.out, imu_help) == 0 && a.err[0] == '\0');
		assert(b.ret == EXIT_FAILURE && b.out[0] == '\0' && b.call.calls == 0);
		assert(strcmp(b.err, "Error: requested multiple function executions\n") == 0);
	}
	{
		char *words[34];
		int argc;

		words[0] = "imu";
		words[1] = "status";
		for (argc = 2; argc < 34; argc++) {
			words[argc] = "x";
		}
		run(&a, &ref_imu_root, NULL, argc, words);
		run(&b, NULL, z_shell_main_imu, argc, words);
		assert(a.ret == EXIT_FAILURE && a.call.calls == 0);
		assert(strcmp(a.err, "status: wrong parameter count\n") == 0);
		assert(b.ret == EXIT_FAILURE && b.call.calls == 0 && b.out[0] == '\0');
		assert(strcmp(b.err, "Too many arguments in the command.\n\n") == 0);
	}

	unlink(out_path);
	unlink(err_path);
	rmdir(dir);
	puts("shell_register tests passed");
	return 0;
}
