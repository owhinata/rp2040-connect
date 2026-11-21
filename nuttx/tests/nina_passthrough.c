/* SPDX-License-Identifier: MIT */
/* Host test of the NuttX build's "nina passthrough" (nuttx/apps/nina/
 * nina_main.c with CONFIG_NINA_PASSTHROUGH, its main() renamed) on the
 * subcommand dispatch (common/shell/subcmd.c), with fakes of the NINA
 * requests and of the driver's nina_passthrough_request()
 * (common/drivers/nina/include/nina_passthrough.h): the help with "passthrough"
 * before "version", the confirmation word, the argument counts, the
 * messages, and the request made only with "confirm".  The mode itself is
 * common/tests/nina_passthrough.c's.
 */

#define _DEFAULT_SOURCE

#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* The command, with its main() renamed. */
#define main nina_main
#include "nina_main.c"
#undef main

static int failures;

#define CHECK(cond)                                                       \
	do {                                                              \
		if (!(cond)) {                                            \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__,    \
			       #cond);                                    \
			failures++;                                       \
		}                                                         \
	} while (0)

/* The command ---------------------------------------------------------*/

static uint32_t fake_capabilities;
static int requests;
static jmp_buf request_jmp;

int nina_wifi_fw_version(char version[NINA_WIFI_FW_VERSION_SIZE])
{
	strcpy(version, "3.0.2");
	return 0;
}

void nina_wifi_capabilities(uint32_t *module, uint32_t *in_use)
{
	*module = fake_capabilities;
	*in_use = fake_capabilities;
}

void nina_passthrough_request(void)
{
	requests++;
	longjmp(request_jmp, 1);
}

#define REQUESTED 1000

static char out[4096], err[4096];

static void slurp(FILE *f, char *buf, size_t size)
{
	size_t n;

	fflush(f);
	rewind(f);
	n = fread(buf, 1, size - 1, f);
	buf[n] = '\0';
}

/* Run "nina" with the words, capturing stdout and stderr; REQUESTED when
 * it asked for the mode.
 */
static int run(const char *first, ...)
{
	char *argv[16];
	int argc = 0, saved_out, saved_err;
	volatile int ret;
	FILE *fo = tmpfile(), *fe = tmpfile();
	va_list ap;
	const char *w;

	assert(fo && fe);
	argv[argc++] = "nina";
	va_start(ap, first);
	for (w = first; w != NULL; w = va_arg(ap, const char *)) {
		argv[argc++] = (char *)w;
	}
	va_end(ap);
	argv[argc] = NULL;
	requests = 0;
	fflush(stdout);
	fflush(stderr);
	saved_out = dup(1);
	saved_err = dup(2);
	dup2(fileno(fo), 1);
	dup2(fileno(fe), 2);
	if (setjmp(request_jmp) == 0) {
		ret = nina_main(argc, argv);
	} else {
		ret = REQUESTED;
	}
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

static const char passthrough_help[] =
	"passthrough - Restart into the NINA passthrough for esptool: passthrough confirm\n"
	"              (USB <-> the NINA's UART; leave it by closing the port at 300 bps,\n"
	"              or with RESET)\n";

static void test_command(void)
{
	/* The help: "passthrough" sorted before "version". */
	CHECK(run(NULL) == EXIT_FAILURE);
	CHECK(!strncmp(out, "nina - NINA Wi-Fi module\nSubcommands:\n  passthrough  : ", 55));
	CHECK(strstr(out, "\n  version      : Show NINA firmware version\n") != NULL);
	CHECK(strstr(out, "passthrough") < strstr(out, "version"));
	CHECK(requests == 0);

	/* "version" is unchanged. */
	fake_capabilities = 0x3;
	CHECK(run("version", NULL) == EXIT_SUCCESS);
	CHECK(!strcmp(out, "NINA firmware: 3.0.2\n"
			   "NINA capabilities: 0x00000003 (in use 0x00000003)\n"));

	/* The confirmation word. */
	CHECK(run("passthrough", "confirm", NULL) == REQUESTED);
	CHECK(requests == 1);
	CHECK(!strcmp(out, "Restarting into the NINA passthrough: the USB port relays to the "
			   "NINA's UART (for esptool).\n"
			   "To leave it: close the port at 300 bps, or press RESET.\n"));
	CHECK(err[0] == '\0');

	CHECK(run("passthrough", "Confirm", NULL) == EXIT_FAILURE);
	CHECK(requests == 0 && out[0] == '\0');
	CHECK(!strcmp(err, "Type \"nina passthrough confirm\" to restart into the NINA "
			   "passthrough\n"));

	/* SUBCMD_ARG(passthrough, .., 2, 0). */
	CHECK(run("passthrough", NULL) == EXIT_FAILURE);
	CHECK(requests == 0);
	CHECK(!strcmp(err, "passthrough: wrong parameter count\n"));
	CHECK(!strcmp(out, passthrough_help));
	CHECK(run("passthrough", "confirm", "now", NULL) == EXIT_FAILURE);
	CHECK(requests == 0);
	CHECK(!strcmp(err, "passthrough: wrong parameter count\n"));
}

int main(void)
{
	test_command();
	if (failures != 0) {
		printf("%d failures\n", failures);
		return 1;
	}
	printf("nina_passthrough: all passed\n");
	return 0;
}
