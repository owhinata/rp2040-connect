/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the NuttX build's "i2c_dma" command: the command of the
 * Zephyr and FreeRTOS builds, common/drivers/i2c_dw/i2c_dma_shell.c
 * (included below, compiled as nuttx/apps/i2c_dma/Makefile builds it, with
 * the prelude; Z_SHELL_CMD_NAMED_MAIN turns its main() into
 * z_shell_main_i2c_dma()), on common/shell/zshell.c with the NuttX
 * backends (apps/subcmd/zshell_os_impl.h), with the I2C0 core's side
 * (common/drivers/i2c_dw/nano_i2c_dma.h) replaced by fakes.
 *
 * Against the command that the NuttX build had until 2026-11-18
 * (nuttx/apps/i2c_dma/i2c_dma_main.c: the same set as a
 * common/shell/subcmd.h table on subcmd_main(), printing with printf()
 * and stderr; copied below as ref_*), each command line must give the same
 * exit status, the same calls of the fakes, and the same bytes on stdout,
 * on stderr and, with both on one file, in the same order (stdout line
 * buffered with NuttX's 64-byte buffer).  Then the exact output (the
 * Zephyr build's text), and the cases where the two differ, with the
 * Zephyr shell's output: a wildcard that matches subcommands followed by a
 * subcommand with a handler, and more than CONFIG_SHELL_ARGC_MAX (30)
 * words.
 */

#define _GNU_SOURCE

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "nano_i2c_dma.h"
#include "subcmd.h"

/* The shared command (its SHELL_CMD_REGISTER() makes z_shell_main_i2c_dma()). */
#include "i2c_dma_shell.c"

int z_shell_main_i2c_dma(int argc, char *argv[]);

/* The I2C0 core's side ---------------------------------------------------*/

static bool fake_enabled = true;
static int set_calls;
static int get_calls;
static struct nano_i2c_dma_stats fake_stats;

void nano_i2c_dma_get_stats(struct nano_i2c_dma_stats *stats)
{
	get_calls++;
	*stats = fake_stats;
}

bool nano_i2c_dma_enabled(void)
{
	return fake_enabled;
}

void nano_i2c_dma_set_enabled(bool enabled)
{
	set_calls++;
	fake_enabled = enabled;
}

/* The NuttX build's command before (i2c_dma_main.c) -----------------------*/

static int ref_stats(int argc, char **argv)
{
	struct nano_i2c_dma_stats stats;

	(void)argc;
	(void)argv;
	nano_i2c_dma_get_stats(&stats);
	printf("dma_transfers=%u dma_bytes=%u dma_errors=%u dma_fallbacks=%u\n",
	       (unsigned int)stats.transfers, (unsigned int)stats.bytes,
	       (unsigned int)stats.errors, (unsigned int)stats.fallbacks);
	return 0;
}

static int ref_mode(int argc, char **argv)
{
	if (argc == 2) {
		if (!strcmp(argv[1], "on")) {
			nano_i2c_dma_set_enabled(true);
		} else if (!strcmp(argv[1], "off")) {
			nano_i2c_dma_set_enabled(false);
		} else {
			fprintf(stderr, "Expected on or off\n");
			return -EINVAL;
		}
	}
	printf("i2c_dma=%s\n", nano_i2c_dma_enabled() ? "on" : "off");
	return 0;
}

static const struct subcmd ref_commands[] = {
	SUBCMD_ARG(mode, NULL, "Set or show I2C DMA mode: [on|off]", ref_mode, 1, 1),
	SUBCMD_ARG(stats, NULL, "Show RP2040 I2C DMA counters", ref_stats, 1, 0),
	SUBCMD_SET_END
};
static const struct subcmd ref_root =
	SUBCMD(i2c_dma, ref_commands, "RP2040 I2C DMA controls", NULL);

/* Running a command line -------------------------------------------------*/

struct result {
	int ret;
	int set_calls, get_calls;
	bool enabled;
	char out[4096], err[4096], both[8192];
};

static char dir[] = "/tmp/i2cXXXXXX";
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

/* Run ARGV on the earlier command (REF) or on the shared one, with DMA on
 * at the start, stdout on out_path and stderr on ERR (out_path again for
 * both on one file).
 */
static int run_once(bool ref, int argc, char **argv, const char *err)
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
	fake_enabled = true;
	set_calls = 0;
	get_calls = 0;
	ret = ref ? subcmd_main(&ref_root, argc, argv) : z_shell_main_i2c_dma(argc, argv);
	fflush(stdout);
	fflush(stderr);
	dup2(saved_out, 1);
	dup2(saved_err, 2);
	close(saved_out);
	close(saved_err);
	return ret;
}

static void run(struct result *r, bool ref, int argc, char **words)
{
	char *argv[40];
	int i;

	/* Each run gets its own copy of the words, NULL-terminated as nsh's. */
	for (i = 0; i < argc; i++) {
		argv[i] = words[i];
	}
	argv[argc] = NULL;
	r->ret = run_once(ref, argc, argv, err_path);
	r->set_calls = set_calls;
	r->get_calls = get_calls;
	r->enabled = fake_enabled;
	slurp(out_path, r->out, sizeof(r->out));
	slurp(err_path, r->err, sizeof(r->err));
	run_once(ref, argc, argv, out_path);
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

/* LINE runs the same on both: A is the earlier command's result, B the
 * shared one's.
 */
static void same(const char *line)
{
	char buf[256], *words[40];
	int argc;

	snprintf(buf, sizeof(buf), "%s", line);
	argc = split(buf, words);
	run(&a, true, argc, words);
	run(&b, false, argc, words);
	if (a.ret != b.ret || strcmp(a.out, b.out) || strcmp(a.err, b.err) ||
	    strcmp(a.both, b.both) || a.set_calls != b.set_calls ||
	    a.get_calls != b.get_calls || a.enabled != b.enabled) {
		fprintf(stderr, "differs: %s\n--- subcmd (%d)\n%s--\n%s--- zshell (%d)\n%s--\n%s",
			line, a.ret, a.out, a.err, b.ret, b.out, b.err);
		abort();
	}
}

static const char help[] =
	"i2c_dma - RP2040 I2C DMA controls\n"
	"Subcommands:\n"
	"  mode   : Set or show I2C DMA mode: [on|off]\n"
	"  stats  : Show RP2040 I2C DMA counters\n";

static const char mode_help[] = "mode - Set or show I2C DMA mode: [on|off]\n";
static const char stats_help[] = "stats - Show RP2040 I2C DMA counters\n";

int main(void)
{
	static const char *const lines[] = {
		/* Help: the command alone, -h and --help (ordinary words),
		 * unknown subcommands, wildcards.
		 */
		"i2c_dma", "i2c_dma --help", "i2c_dma -h", "i2c_dma foo", "i2c_dma foo mode",
		"i2c_dma foo bar", "i2c_dma m*", "i2c_dma *", "i2c_dma x*", "i2c_dma ?",
		"i2c_dma x* mode", "i2c_dma MODE",
		/* Handlers and their argument counts. */
		"i2c_dma mode", "i2c_dma mode on", "i2c_dma mode off", "i2c_dma mode ON",
		"i2c_dma mode x", "i2c_dma mode --help", "i2c_dma mode on off",
		"i2c_dma mode a b c", "i2c_dma stats", "i2c_dma stats x", "i2c_dma stats --help",
		"i2c_dma stats x y",
		/* A wildcard after a handler's entry ends the words there. */
		"i2c_dma mode *", "i2c_dma mode o*", "i2c_dma mode on *", "i2c_dma stats *",
		"i2c_dma stats ?",
	};
	size_t i;

	setvbuf(stdout, NULL, _IOLBF, 64);
	assert(mkdtemp(dir));
	snprintf(out_path, sizeof(out_path), "%s/o", dir);
	snprintf(err_path, sizeof(err_path), "%s/e", dir);

	for (i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
		same(lines[i]);
	}

	/* The output in full: the reference recording at boot. */
	same("i2c_dma stats");
	assert(b.ret == EXIT_SUCCESS && b.get_calls == 1 && b.err[0] == '\0');
	assert(!strcmp(b.out, "dma_transfers=0 dma_bytes=0 dma_errors=0 dma_fallbacks=0\n"));

	/* 32-bit counters, printed unsigned. */
	fake_stats = (struct nano_i2c_dma_stats){6454, 4294967295u, 1, 2};
	same("i2c_dma stats");
	assert(!strcmp(b.out,
		       "dma_transfers=6454 dma_bytes=4294967295 dma_errors=1 dma_fallbacks=2\n"));

	/* mode: show, switch, refuse (DMA on at the start of each run). */
	same("i2c_dma mode");
	assert(b.ret == EXIT_SUCCESS && !strcmp(b.out, "i2c_dma=on\n") && b.set_calls == 0);
	same("i2c_dma mode off");
	assert(b.ret == EXIT_SUCCESS && !strcmp(b.out, "i2c_dma=off\n"));
	assert(b.set_calls == 1 && !b.enabled);
	same("i2c_dma mode on");
	assert(b.ret == EXIT_SUCCESS && !strcmp(b.out, "i2c_dma=on\n"));
	assert(b.set_calls == 1 && b.enabled);
	same("i2c_dma mode ON");
	assert(b.ret == EXIT_FAILURE && b.out[0] == '\0' && b.set_calls == 0 && b.enabled);
	assert(!strcmp(b.err, "Expected on or off\n"));

	/* Argument counts of SHELL_CMD_ARG(mode, .., 1, 1) and (stats, .., 1, 0). */
	same("i2c_dma mode on off");
	assert(b.ret == EXIT_FAILURE && b.set_calls == 0);
	assert(!strcmp(b.err, "mode: wrong parameter count\n") && !strcmp(b.out, mode_help));
	assert(!strcmp(b.both, "mode: wrong parameter count\n"
			       "mode - Set or show I2C DMA mode: [on|off]\n"));
	same("i2c_dma stats x");
	assert(b.ret == EXIT_FAILURE && b.get_calls == 0);
	assert(!strcmp(b.err, "stats: wrong parameter count\n") && !strcmp(b.out, stats_help));

	/* The command alone or an unknown word: the help. */
	same("i2c_dma");
	assert(b.ret == EXIT_FAILURE && !strcmp(b.out, help) && b.err[0] == '\0');
	same("i2c_dma foo");
	assert(b.ret == EXIT_FAILURE && !strcmp(b.out, help) && b.err[0] == '\0');

	/* Where they differ: zshell.c gives the Zephyr shell's output. */
	{
		char line[] = "i2c_dma m* stats";
		char *words[4];
		int argc = split(line, words);

		run(&a, true, argc, words);
		run(&b, false, argc, words);
		assert(a.ret == EXIT_FAILURE && !strcmp(a.out, help) && a.err[0] == '\0');
		assert(b.ret == EXIT_FAILURE && b.out[0] == '\0' && b.get_calls == 0);
		assert(!strcmp(b.err, "Error: requested multiple function executions\n"));
	}
	{
		char *words[34];
		int argc;

		words[0] = "i2c_dma";
		words[1] = "mode";
		for (argc = 2; argc < 34; argc++) {
			words[argc] = "x";
		}
		run(&a, true, argc, words);
		run(&b, false, argc, words);
		assert(a.ret == EXIT_FAILURE && a.set_calls == 0);
		assert(!strcmp(a.err, "mode: wrong parameter count\n") && !strcmp(a.out, mode_help));
		assert(b.ret == EXIT_FAILURE && b.set_calls == 0 && b.out[0] == '\0');
		assert(!strcmp(b.err, "Too many arguments in the command.\n\n"));
	}

	unlink(out_path);
	unlink(err_path);
	rmdir(dir);
	puts("i2c_dma_app tests passed");
	return 0;
}
