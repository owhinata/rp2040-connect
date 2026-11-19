/* SPDX-License-Identifier: MIT */
/* Host test of the NuttX side of the NINA passthrough mode:
 *
 * - The board's glue (boards/arduino-nano-rp2040-connect/src/
 *   nina_passthrough_glue.h) between cdcacm's state and the shared
 *   decisions (common/drivers/nina/nina_passthrough_logic.h, tested on its
 *   own by freertos/tests/passthrough): the line coding's rate bytes, the
 *   control line bits (DTR bit 0, RTS bit 1) with the module's lines, the
 *   way out decided on the lines before the change, the count of closes,
 *   and the rate the UART follows; esptool's own sequences (the port
 *   opened, the custom reset sequence R0|D1|W0.2|R1|W0.6, the close), the
 *   "stty 300" way out and a port opened at a stale 300 bps.
 * - The words of UART1's data register from the receive interrupt: data,
 *   a break (no data), an overrun and a framing error (counted, kept), the
 *   ring full (dropped and counted).
 * - The "nina" command (nuttx/apps/nina/nina_main.c with
 *   CONFIG_NINA_PASSTHROUGH, its main() renamed) on the subcommand
 *   dispatch (common/shell/subcmd.c), with fakes of the NINA requests and
 *   of the board's nina_passthrough_request(): the help with
 *   "passthrough" before "version", the confirmation word, the argument
 *   counts, the messages, and the request made only with "confirm".
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

#include "nina_passthrough_glue.h"

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

/* The glue ------------------------------------------------------------*/

static struct nina_pt_pins pins(const struct nina_pt_host *h, uint32_t now)
{
	return nina_pt_lines_pins(&h->lines, now);
}

static void test_coding(void)
{
	const uint8_t b115200[4] = {0x00, 0xc2, 0x01, 0x00};
	const uint8_t b921600[4] = {0x00, 0x10, 0x0e, 0x00};
	const uint8_t b300[4] = {0x2c, 0x01, 0x00, 0x00};
	const uint8_t bmax[4] = {0xff, 0xff, 0xff, 0xff};
	struct nina_pt_host h;
	uint32_t baud = NINA_PT_BAUD_INIT;

	CHECK(nina_pt_coding_rate(b115200) == 115200);
	CHECK(nina_pt_coding_rate(b921600) == 921600);
	CHECK(nina_pt_coding_rate(b300) == 300);
	CHECK(nina_pt_coding_rate(bmax) == UINT32_MAX);

	nina_pt_host_init(&h);
	CHECK(h.rate == 115200 && h.closes == 0 && !h.exit);
	CHECK(!h.lines.dtr && !h.lines.rts);

	/* esptool's --baud 921600: the port's rate after the stub's change. */
	baud = nina_pt_host_coding(&h, 921600, baud);
	CHECK(baud == 921600 && h.rate == 921600);
	/* The exit rate and those out of range keep the UART's rate, but are
	 * the host's rate for the way out.
	 */
	baud = nina_pt_host_coding(&h, 300, baud);
	CHECK(baud == 921600 && h.rate == 300);
	baud = nina_pt_host_coding(&h, 0, baud);
	CHECK(baud == 921600 && h.rate == 0);
	baud = nina_pt_host_coding(&h, NINA_PT_BAUD_MAX + 1, baud);
	CHECK(baud == 921600);
	baud = nina_pt_host_coding(&h, 115200, baud);
	CHECK(baud == 115200 && h.rate == 115200);
}

static void test_lines(void)
{
	struct nina_pt_host h;
	struct nina_pt_pins p;

	nina_pt_host_init(&h);
	p = pins(&h, 0);
	CHECK(!p.reset_released && p.gpio0 == NINA_PT_GPIO0_HIGH && p.uart_tx);

	/* Linux opens the port: DTR and RTS at once; the ROM's download mode. */
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_DTR | NINA_PT_CTRL_RTS, 1000));
	p = pins(&h, 1000);
	CHECK(p.reset_released && p.gpio0 == NINA_PT_GPIO0_LOW && p.uart_tx);

	/* esptool's custom_reset_sequence R0|D1|W0.2|R1|W0.6: into reset with
	 * DTR (GPIO0 low), then released into the download mode, RTS kept.
	 */
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_DTR, 2000));
	p = pins(&h, 2000);
	CHECK(!p.reset_released && p.gpio0 == NINA_PT_GPIO0_LOW && p.uart_tx);
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_DTR, 2001));
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_DTR | NINA_PT_CTRL_RTS, 2200));
	p = pins(&h, 2800);
	CHECK(p.reset_released && p.gpio0 == NINA_PT_GPIO0_LOW && p.uart_tx);
	CHECK(!h.lines.own_firmware && h.closes == 0 && !h.exit);

	/* esptool exits (--after no-reset) and the port closes at 921600:
	 * held in reset, a close counted, no way out.
	 */
	nina_pt_host_coding(&h, 921600, 115200);
	CHECK(nina_pt_host_ctrlline(&h, 0, 3000));
	p = pins(&h, 3000);
	CHECK(!p.reset_released && p.gpio0 == NINA_PT_GPIO0_HIGH && p.uart_tx);
	CHECK(h.closes == 1 && !h.exit);
	/* Bits other than DTR and RTS are not lines. */
	CHECK(!nina_pt_host_ctrlline(&h, 0xfc, 3001));
	CHECK(!h.lines.dtr && !h.lines.rts && h.closes == 1);

	/* RTS alone (a terminal with DTR off): the module's own firmware;
	 * GPIO0 high for 750 ms, then released, GPIO8 not driven.
	 */
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_RTS, 4000));
	p = pins(&h, 4000 + NINA_PT_GPIO0_HOLD_MS - 1);
	CHECK(p.reset_released && p.gpio0 == NINA_PT_GPIO0_HIGH && !p.uart_tx);
	p = pins(&h, 4000 + NINA_PT_GPIO0_HOLD_MS);
	CHECK(p.gpio0 == NINA_PT_GPIO0_RELEASED && !p.uart_tx);
	/* DTR is ignored while the module's firmware runs. */
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_DTR | NINA_PT_CTRL_RTS, 5000));
	p = pins(&h, 5000);
	CHECK(p.gpio0 == NINA_PT_GPIO0_RELEASED && !p.uart_tx);
	CHECK(nina_pt_host_ctrlline(&h, 0, 6000));
	CHECK(h.closes == 2 && !h.exit);
}

static void test_exit(void)
{
	struct nina_pt_host h;
	uint32_t baud = 115200;

	/* "stty -F /dev/ttyACM0 300": opened, set to 300, closed. */
	nina_pt_host_init(&h);
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_DTR | NINA_PT_CTRL_RTS, 100));
	baud = nina_pt_host_coding(&h, 300, baud);
	CHECK(baud == 115200 && !h.exit);
	CHECK(nina_pt_host_ctrlline(&h, 0, 120));
	CHECK(h.exit && h.closes == 1);

	/* A port opened at the stale 300 bps and set to 115200 before it
	 * closes does not leave.
	 */
	nina_pt_host_init(&h);
	nina_pt_host_coding(&h, 300, baud);
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_DTR | NINA_PT_CTRL_RTS, 100));
	CHECK(!h.exit);
	nina_pt_host_coding(&h, 115200, baud);
	CHECK(nina_pt_host_ctrlline(&h, 0, 200));
	CHECK(!h.exit && h.closes == 1);

	/* At 300 bps without the port open (DTR never set): no way out. */
	nina_pt_host_init(&h);
	nina_pt_host_coding(&h, 300, baud);
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_RTS, 100));
	CHECK(!nina_pt_host_ctrlline(&h, 0, 200));
	CHECK(!h.exit && h.closes == 0);

	/* RTS dropped alone at 300 bps (DTR kept): no way out. */
	nina_pt_host_init(&h);
	nina_pt_host_coding(&h, 300, baud);
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_DTR | NINA_PT_CTRL_RTS, 100));
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_DTR, 200));
	CHECK(!h.exit);
	/* The way out stays once decided. */
	CHECK(nina_pt_host_ctrlline(&h, 0, 300));
	CHECK(h.exit);
	CHECK(!nina_pt_host_ctrlline(&h, NINA_PT_CTRL_DTR | NINA_PT_CTRL_RTS, 400));
	CHECK(h.exit);
}

static void test_uart_words(void)
{
	static uint8_t buf[8];
	struct nina_pt_ring r;
	struct nina_pt_uart_counts c;
	const uint8_t *data;
	uint32_t n;

	memset(&c, 0, sizeof(c));
	nina_pt_ring_init(&r, buf, sizeof(buf));

	nina_pt_uart_word(&r, &c, 0x41);
	nina_pt_uart_word(&r, &c, NINA_PT_UARTDR_BE); /* a break: one 0, no data */
	nina_pt_uart_word(&r, &c, NINA_PT_UARTDR_OE | 0x42);
	nina_pt_uart_word(&r, &c, NINA_PT_UARTDR_FE | 0x43);
	nina_pt_uart_word(&r, &c, NINA_PT_UARTDR_OE | NINA_PT_UARTDR_BE);
	nina_pt_uart_word(&r, &c, 0x1ff & ~NINA_PT_UARTDR_FE); /* only bits 0..7 */
	CHECK(c.overruns == 2 && c.breaks == 2 && c.framing == 1);
	n = nina_pt_ring_peek(&r, &data);
	CHECK(n == 4);
	CHECK(n == 4 && data[0] == 0x41 && data[1] == 0x42 && data[2] == 0x43 &&
	      data[3] == 0xff);
	nina_pt_ring_consume(&r, n);

	/* Full: the rest is dropped and counted. */
	for (int i = 0; i < 10; i++) {
		nina_pt_uart_word(&r, &c, (uint32_t)i);
	}
	CHECK(r.dropped == 2);
	CHECK(r.head - r.tail == sizeof(buf));
}

/* The command ---------------------------------------------------------*/

static uint32_t fake_capabilities;
static int requests;
static jmp_buf request_jmp;

int nina_request(uint8_t cmd, const struct nina_param *params, size_t nparams, bool param16,
		 bool resp16, uint8_t *resp, size_t resp_cap, size_t *resp_len)
{
	static const char version[] = "3.0.2";

	(void)params;
	(void)nparams;
	(void)param16;
	(void)resp16;
	assert(cmd == 0x37);
	assert(resp_cap >= sizeof(version) - 1);
	memcpy(resp, version, sizeof(version) - 1);
	*resp_len = sizeof(version) - 1;
	return 0;
}

uint32_t nina_capabilities(void)
{
	return fake_capabilities;
}

uint32_t nina_module_capabilities(void)
{
	return fake_capabilities;
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
	test_coding();
	test_lines();
	test_exit();
	test_uart_words();
	test_command();
	if (failures != 0) {
		printf("%d failures\n", failures);
		return 1;
	}
	printf("nina_passthrough: all passed\n");
	return 0;
}
