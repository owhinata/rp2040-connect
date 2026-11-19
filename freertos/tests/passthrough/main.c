/* SPDX-License-Identifier: MIT */
/* Host test of the NINA passthrough mode's decisions
 * (common/drivers/nina/nina_passthrough_logic.h, which
 * freertos/drivers/nina/nina_passthrough.c runs on the board), and of the
 * merge of its "nina" subcommand (common/shell/nina_shell.h).
 *
 * - The request in the watchdog's scratch registers: the words, a damaged
 *   copy, and the fatal error record of the same registers
 *   (common/log/fault_record.c): neither is taken for the other.
 * - The lines from DTR and RTS: the port opened (download mode), RTS
 *   pulsed ("--after hard_reset"), closed (held in reset), DTR then RTS
 *   (esptool's "--before default-reset --after no-reset" with the
 *   custom_reset_sequence R0|D1|W0.2|R1|W0.6 of ESPTOOL_CFGFILE, which
 *   keeps RTS set), a start into the module's own firmware (GPIO0
 *   released after 750 ms, GPIO8 not driven, DTR ignored), the clock's
 *   wrap.
 * - The rate: the exit rate and those out of range keep the UART's rate.
 * - The way out: only the port closed at 300 bps; a port opened at the
 *   stale 300 bps and set to 115200 before it closes does not leave.
 * - The buffer: wrap, full (counted drops), peak, the counters' wrap, and a
 *   producer thread against a consumer thread (every byte in order or
 *   counted as dropped).
 * - The "nina" subcommands: "passthrough" merged by name into sets before,
 *   between and after their entries, and into an empty one.
 */

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fault_record.h"
#include "nina_passthrough_logic.h"
#include "nina_shell.h"

static int failures;

#define CHECK(cond)                                                                                \
	do {                                                                                       \
		if (!(cond)) {                                                                     \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                     \
			failures++;                                                                \
		}                                                                                  \
	} while (0)

static void test_mark(void)
{
	uint32_t s[NINA_PT_MARK_WORDS];
	uint32_t f[FAULT_SCRATCH_WORDS];
	struct fault_record r;
	struct fault_record copy;

	nina_pt_mark(s);
	CHECK(nina_pt_mark_valid(s));
	CHECK(!fault_record_from_scratch(s, &copy));
	for (int i = 0; i < NINA_PT_MARK_WORDS; i++) {
		for (int bit = 0; bit < 32; bit++) {
			uint32_t t[NINA_PT_MARK_WORDS];

			memcpy(t, s, sizeof(t));
			t[i] ^= 1u << bit;
			CHECK(!nina_pt_mark_valid(t));
		}
	}
	memset(s, 0, sizeof(s));
	CHECK(!nina_pt_mark_valid(s));

	/* A fatal error's words (what a fault during the restart leaves). */
	memset(&r, 0, sizeof(r));
	r.reason = FAULT_REASON_HARDFAULT;
	r.pc = 0x10001234u;
	r.lr = 0x10005679u;
	fault_record_to_scratch(&r, f);
	CHECK(fault_record_from_scratch(f, &copy));
	CHECK(!nina_pt_mark_valid(f));
}

static void check_pins(const struct nina_pt_lines *l, uint32_t now, bool released,
		       enum nina_pt_gpio0 gpio0, bool uart_tx, int line)
{
	struct nina_pt_pins p = nina_pt_lines_pins(l, now);

	if (p.reset_released != released || p.gpio0 != gpio0 || p.uart_tx != uart_tx) {
		printf("FAIL %s:%d: pins %d %d %d, want %d %d %d\n", __FILE__, line,
		       p.reset_released, p.gpio0, p.uart_tx, released, gpio0, uart_tx);
		failures++;
	}
}

#define PINS(l, now, released, gpio0, tx) check_pins(l, now, released, gpio0, tx, __LINE__)

static void test_lines(void)
{
	struct nina_pt_lines l;

	/* The mode's start: the port closed, the module in reset. */
	nina_pt_lines_init(&l);
	PINS(&l, 0, false, NINA_PT_GPIO0_HIGH, true);

	/* esptool opens the port (Linux sets DTR and RTS in one request):
	 * GPIO0 low and the reset released, the ROM's download mode.
	 */
	nina_pt_lines_set(&l, true, true, 1000);
	CHECK(!l.own_firmware);
	PINS(&l, 1000, true, NINA_PT_GPIO0_LOW, true);
	PINS(&l, 1000 + 60000, true, NINA_PT_GPIO0_LOW, true);

	/* "--after hard_reset": RTS set (no change), 100 ms, RTS clear: held
	 * in reset with GPIO0 low.
	 */
	nina_pt_lines_set(&l, true, true, 70000);
	PINS(&l, 70000, true, NINA_PT_GPIO0_LOW, true);
	nina_pt_lines_set(&l, true, false, 70100);
	PINS(&l, 70100, false, NINA_PT_GPIO0_LOW, true);

	/* The port closed: both clear, in reset with GPIO0 high. */
	nina_pt_lines_set(&l, false, false, 71000);
	PINS(&l, 71000, false, NINA_PT_GPIO0_HIGH, true);

	/* Opened again with DTR and RTS set one after the other: DTR first
	 * (still in reset), then RTS (download mode).
	 */
	nina_pt_lines_set(&l, true, false, 80000);
	PINS(&l, 80000, false, NINA_PT_GPIO0_LOW, true);
	nina_pt_lines_set(&l, true, true, 80001);
	CHECK(!l.own_firmware);
	PINS(&l, 80001, true, NINA_PT_GPIO0_LOW, true);

	/* DTR cleared while the ROM (or esptool's stub) runs: GPIO0 follows,
	 * the UART stays connected (the sketch's behaviour).
	 */
	nina_pt_lines_set(&l, false, true, 90000);
	PINS(&l, 90000, true, NINA_PT_GPIO0_HIGH, true);
	PINS(&l, 99999, true, NINA_PT_GPIO0_HIGH, true);

	/* RTS first with DTR clear: the module's own firmware.  GPIO0 high
	 * for 750 ms, then released; GPIO8 not driven; DTR ignored.
	 */
	nina_pt_lines_set(&l, false, false, 100000);
	nina_pt_lines_set(&l, false, true, 100010);
	CHECK(l.own_firmware);
	PINS(&l, 100010, true, NINA_PT_GPIO0_HIGH, false);
	PINS(&l, 100010 + NINA_PT_GPIO0_HOLD_MS - 1, true, NINA_PT_GPIO0_HIGH, false);
	PINS(&l, 100010 + NINA_PT_GPIO0_HOLD_MS, true, NINA_PT_GPIO0_RELEASED, false);
	nina_pt_lines_set(&l, true, true, 101000);
	PINS(&l, 101000, true, NINA_PT_GPIO0_RELEASED, false);
	PINS(&l, 100010 + 3600000, true, NINA_PT_GPIO0_RELEASED, false);

	/* Back into reset: the UART again, GPIO0 the strap of DTR. */
	nina_pt_lines_set(&l, true, false, 102000);
	PINS(&l, 102000, false, NINA_PT_GPIO0_LOW, true);
	nina_pt_lines_set(&l, true, true, 102100);
	CHECK(!l.own_firmware);
	PINS(&l, 102100, true, NINA_PT_GPIO0_LOW, true);

	/* The hold across the wrap of the ms clock. */
	nina_pt_lines_init(&l);
	nina_pt_lines_set(&l, false, true, UINT32_MAX - 100);
	PINS(&l, UINT32_MAX - 100, true, NINA_PT_GPIO0_HIGH, false);
	PINS(&l, 500, true, NINA_PT_GPIO0_HIGH, false);
	PINS(&l, NINA_PT_GPIO0_HOLD_MS - 101, true, NINA_PT_GPIO0_RELEASED, false);
}

static void test_rate_and_exit(void)
{
	CHECK(nina_pt_uart_baud(115200, 115200) == 115200);
	CHECK(nina_pt_uart_baud(921600, 115200) == 921600);
	CHECK(nina_pt_uart_baud(460800, 921600) == 460800);
	CHECK(nina_pt_uart_baud(NINA_PT_EXIT_BAUD, 921600) == 921600);
	CHECK(nina_pt_uart_baud(0, 115200) == 115200);
	CHECK(nina_pt_uart_baud(600, 115200) == 115200);
	CHECK(nina_pt_uart_baud(NINA_PT_BAUD_MIN, 115200) == NINA_PT_BAUD_MIN);
	CHECK(nina_pt_uart_baud(NINA_PT_BAUD_MAX, 115200) == NINA_PT_BAUD_MAX);
	CHECK(nina_pt_uart_baud(NINA_PT_BAUD_MAX + 1, 115200) == 115200);

	/* "stty -F /dev/ttyACM0 300": opened (DTR up at the old rate), set to
	 * 300, closed.
	 */
	CHECK(!nina_pt_exit(false, true, 115200));
	CHECK(nina_pt_exit(true, false, NINA_PT_EXIT_BAUD));
	/* esptool: opened at a stale 300 (Linux keeps the tty's last rate),
	 * set to 115200, then 921600, closed.
	 */
	CHECK(!nina_pt_exit(false, true, NINA_PT_EXIT_BAUD));
	CHECK(!nina_pt_exit(true, true, NINA_PT_EXIT_BAUD));
	CHECK(!nina_pt_exit(true, false, 115200));
	CHECK(!nina_pt_exit(true, false, 921600));
	/* DTR clear already, and other rates. */
	CHECK(!nina_pt_exit(false, false, NINA_PT_EXIT_BAUD));
	CHECK(!nina_pt_exit(true, false, 1200));
}

/* Drains the ring into OUT (at most CAP bytes), as the loop does. */
static uint32_t drain(struct nina_pt_ring *r, uint8_t *out, uint32_t cap)
{
	const uint8_t *data;
	uint32_t total = 0;
	uint32_t n;

	while (total < cap && (n = nina_pt_ring_peek(r, &data)) > 0) {
		if (n > cap - total) {
			n = cap - total;
		}
		memcpy(out + total, data, n);
		nina_pt_ring_consume(r, n);
		total += n;
	}
	return total;
}

static void test_ring(void)
{
	static uint8_t buf[16];
	struct nina_pt_ring r;
	const uint8_t *data;
	uint8_t out[64];
	uint32_t n;

	nina_pt_ring_init(&r, buf, sizeof(buf));
	CHECK(nina_pt_ring_peek(&r, &data) == 0);

	/* Fill exactly, then one more: dropped. */
	for (int i = 0; i < 16; i++) {
		nina_pt_ring_put(&r, (uint8_t)i);
	}
	CHECK(r.dropped == 0);
	nina_pt_ring_put(&r, 99);
	CHECK(r.dropped == 1);
	n = drain(&r, out, sizeof(out));
	CHECK(n == 16);
	CHECK(r.peak == 16);
	for (int i = 0; i < 16; i++) {
		CHECK(out[i] == i);
	}

	/* Across the end of the buffer: the peek stops at the end. */
	for (int i = 0; i < 10; i++) {
		nina_pt_ring_put(&r, (uint8_t)(100 + i));
	}
	n = drain(&r, out, 6);
	CHECK(n == 6);
	for (int i = 0; i < 10; i++) {
		nina_pt_ring_put(&r, (uint8_t)(110 + i));
	}
	/* 14 waiting from index 22 (6 in the buffer): 10 up to its end. */
	CHECK(r.head - r.tail == 14);
	n = nina_pt_ring_peek(&r, &data);
	CHECK(n == 10);
	CHECK(data == buf + 6);
	n = drain(&r, out, sizeof(out));
	CHECK(n == 14);
	for (int i = 0; i < 14; i++) {
		CHECK(out[i] == 106 + i);
	}
	CHECK(r.dropped == 1);

	/* The indices' wrap at 2^32. */
	nina_pt_ring_init(&r, buf, sizeof(buf));
	r.head = r.tail = UINT32_MAX - 4;
	for (int i = 0; i < 12; i++) {
		nina_pt_ring_put(&r, (uint8_t)(200 + i));
	}
	CHECK(r.head == 7);
	n = drain(&r, out, sizeof(out));
	CHECK(n == 12);
	for (int i = 0; i < 12; i++) {
		CHECK(out[i] == 200 + i);
	}
	CHECK(r.dropped == 0);
}

/* A producer (the interrupt) and a consumer (the loop) on two threads: the
 * consumer sees the producer's sequence in order, with gaps only where the
 * producer counted drops.
 */
#define STRESS_BYTES 20000000u

static struct nina_pt_ring stress;
static uint8_t stress_buf[1024];
static volatile bool stress_done;

static void *producer(void *arg)
{
	uint32_t seq = 0;

	(void)arg;
	for (uint32_t i = 0; i < STRESS_BYTES; i++) {
		/* A lost byte leaves a gap: the next one has the next value. */
		nina_pt_ring_put(&stress, (uint8_t)seq++);
	}
	__atomic_store_n(&stress_done, true, __ATOMIC_SEQ_CST);
	return NULL;
}

static void test_ring_threads(void)
{
	pthread_t thread;
	uint64_t received = 0;
	uint8_t expect = 0;
	uint32_t gaps = 0;
	uint8_t out[300];

	nina_pt_ring_init(&stress, stress_buf, sizeof(stress_buf));
	pthread_create(&thread, NULL, producer, NULL);
	for (;;) {
		bool done = __atomic_load_n(&stress_done, __ATOMIC_SEQ_CST);
		uint32_t n = drain(&stress, out, sizeof(out));

		for (uint32_t i = 0; i < n; i++) {
			if (out[i] != expect) {
				gaps++;
			}
			expect = (uint8_t)(out[i] + 1);
		}
		received += n;
		if (done && n == 0 && stress.head == stress.tail) {
			break;
		}
	}
	pthread_join(thread, NULL);
	printf("ring threads: %llu received, %lu dropped, %lu gaps, peak %lu\n",
	       (unsigned long long)received, (unsigned long)stress.dropped, (unsigned long)gaps,
	       (unsigned long)stress.peak);
	CHECK(received + stress.dropped == STRESS_BYTES);
	/* Every gap is a run of drops: no more gaps than drops. */
	CHECK(gaps <= stress.dropped);
	CHECK(stress.peak <= sizeof(stress_buf));
}

static int handler(const struct shell *sh, size_t argc, char **argv)
{
	(void)sh;
	(void)argc;
	(void)argv;
	return 0;
}

static void check_order(const struct shell_static_entry *shared, const char *const *want, int line)
{
	static const struct shell_static_entry extra =
		SHELL_CMD_ARG(passthrough, NULL, "Extra", handler, 2, 0);
	struct shell_static_entry e;
	size_t i = 0;

	for (; want[i] != NULL; i++) {
		nina_pt_subcmd_get(shared, &extra, i, &e);
		if (e.syntax == NULL || strcmp(e.syntax, want[i]) != 0) {
			printf("FAIL %s:%d: entry %zu is %s, want %s\n", __FILE__, line, i,
			       e.syntax ? e.syntax : "(end)", want[i]);
			failures++;
			return;
		}
		if (strcmp(e.syntax, "passthrough") == 0) {
			CHECK(e.handler == handler && e.args.mandatory == 2 &&
			      e.args.optional == 0);
		}
	}
	nina_pt_subcmd_get(shared, &extra, i, &e);
	CHECK(e.syntax == NULL);
	nina_pt_subcmd_get(shared, &extra, i + 5, &e);
	CHECK(e.syntax == NULL);
}

static void test_subcmd(void)
{
	static const struct shell_static_entry only_version[] = {
		SHELL_CMD(version, NULL, "Version", handler), SHELL_SUBCMD_SET_END};
	static const struct shell_static_entry before_and_after[] = {
		SHELL_CMD(alpha, NULL, "A", handler), SHELL_CMD(version, NULL, "V", handler),
		SHELL_SUBCMD_SET_END};
	static const struct shell_static_entry all_before[] = {SHELL_CMD(alpha, NULL, "A", handler),
							       SHELL_CMD(beta, NULL, "B", handler),
							       SHELL_SUBCMD_SET_END};
	static const struct shell_static_entry none[] = {SHELL_SUBCMD_SET_END};

	check_order(only_version, (const char *const[]){"passthrough", "version", NULL}, __LINE__);
	check_order(before_and_after,
		    (const char *const[]){"alpha", "passthrough", "version", NULL}, __LINE__);
	check_order(all_before, (const char *const[]){"alpha", "beta", "passthrough", NULL},
		    __LINE__);
	check_order(none, (const char *const[]){"passthrough", NULL}, __LINE__);
}

int main(void)
{
	test_mark();
	test_lines();
	test_rate_and_exit();
	test_ring();
	test_ring_threads();
	test_subcmd();
	if (failures) {
		printf("%d failures\n", failures);
		return 1;
	}
	printf("passthrough: all passed\n");
	return 0;
}
