/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of nuttx/apps/i2c_dma/i2c_dma_main.c: the output of the
 * "i2c_dma" command against the Zephyr build's (i2c_dw.c:770-801, and
 * host/recordings/zephyr-reference-outputs.txt for "i2c_dma stats"), with
 * the board's side (nano_i2c_dma.h) replaced by fakes.
 */

#define _POSIX_C_SOURCE 200809L

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "nano_i2c_dma.h"

/* The command, with its main() renamed. */
#define main i2c_dma_main
#include "i2c_dma_main.c"
#undef main

static bool fake_enabled = true;
static int set_calls;
static struct nano_i2c_dma_stats fake_stats;

void nano_i2c_dma_get_stats(struct nano_i2c_dma_stats *stats)
{
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

static char out[4096], err[4096];

static void slurp(FILE *f, char *buf, size_t size)
{
	size_t n;

	fflush(f);
	rewind(f);
	n = fread(buf, 1, size - 1, f);
	buf[n] = '\0';
}

/* Run "i2c_dma" with the words, capturing stdout and stderr. */
static int run(const char *first, ...)
{
	char *argv[16];
	int argc = 0, ret, saved_out, saved_err;
	FILE *fo = tmpfile(), *fe = tmpfile();
	va_list ap;
	const char *w;

	assert(fo && fe);
	argv[argc++] = "i2c_dma";
	va_start(ap, first);
	for (w = first; w != NULL; w = va_arg(ap, const char *)) {
		argv[argc++] = (char *)w;
	}
	va_end(ap);
	argv[argc] = NULL;
	set_calls = 0;
	fflush(stdout);
	fflush(stderr);
	saved_out = dup(1);
	saved_err = dup(2);
	dup2(fileno(fo), 1);
	dup2(fileno(fe), 2);
	ret = i2c_dma_main(argc, argv);
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

static const char help[] =
	"i2c_dma - RP2040 I2C DMA controls\n"
	"Subcommands:\n"
	"  mode   : Set or show I2C DMA mode: [on|off]\n"
	"  stats  : Show RP2040 I2C DMA counters\n";

static const char mode_help[] = "mode - Set or show I2C DMA mode: [on|off]\n";
static const char stats_help[] = "stats - Show RP2040 I2C DMA counters\n";

int main(void)
{
	/* The reference recording at boot. */
	assert(run("stats", NULL) == EXIT_SUCCESS);
	assert(!strcmp(out, "dma_transfers=0 dma_bytes=0 dma_errors=0 dma_fallbacks=0\n"));
	assert(err[0] == '\0');

	/* 32-bit counters, printed unsigned. */
	fake_stats = (struct nano_i2c_dma_stats){6454, 4294967295u, 1, 2};
	assert(run("stats", NULL) == EXIT_SUCCESS);
	assert(!strcmp(out,
		       "dma_transfers=6454 dma_bytes=4294967295 dma_errors=1 dma_fallbacks=2\n"));

	/* mode: show, switch, refuse. */
	assert(run("mode", NULL) == EXIT_SUCCESS);
	assert(!strcmp(out, "i2c_dma=on\n") && set_calls == 0);
	assert(run("mode", "off", NULL) == EXIT_SUCCESS);
	assert(!strcmp(out, "i2c_dma=off\n") && set_calls == 1 && !fake_enabled);
	assert(run("mode", NULL) == EXIT_SUCCESS);
	assert(!strcmp(out, "i2c_dma=off\n"));
	assert(run("mode", "on", NULL) == EXIT_SUCCESS);
	assert(!strcmp(out, "i2c_dma=on\n") && fake_enabled);
	assert(run("mode", "ON", NULL) == EXIT_FAILURE);
	assert(out[0] == '\0' && !strcmp(err, "Expected on or off\n") && set_calls == 0);
	assert(fake_enabled);

	/* Argument counts of SHELL_CMD_ARG(mode, .., 1, 1) and (stats, .., 1, 0). */
	assert(run("mode", "on", "off", NULL) == EXIT_FAILURE);
	assert(!strcmp(err, "mode: wrong parameter count\n") && set_calls == 0);
	assert(!strcmp(out, mode_help));
	assert(run("stats", "x", NULL) == EXIT_FAILURE);
	assert(!strcmp(err, "stats: wrong parameter count\n"));
	assert(!strcmp(out, stats_help));

	/* The command alone or an unknown word: the help. */
	assert(run(NULL) == EXIT_FAILURE);
	assert(!strcmp(out, help));
	assert(run("foo", NULL) == EXIT_FAILURE);
	assert(!strcmp(out, help));
	return 0;
}
