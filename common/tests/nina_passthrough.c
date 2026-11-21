/* SPDX-License-Identifier: MIT */
/* Host test of the NINA passthrough mode's core
 * (common/drivers/nina/nina_passthrough.c, which the test includes) on
 * fakes of its two boundaries: nina_passthrough_bus.h (the scratch
 * registers, the restart, the module's lines, the UART's FIFOs and
 * interrupt, the LED) and nina_passthrough_os.h (a clock that only the
 * test and the sleeps move, a host that takes a given number of bytes or
 * none, the log).  The OS bindings' own parts are the builds' tests
 * (zephyr/app/usb-console/tests/passthrough.c; NuttX: the command,
 * nuttx/tests/nina_passthrough.c).
 *
 * - The request: the words, every bit of them, and the fatal error record
 *   of the same registers (common/log/fault_record.c): neither is taken
 *   for the other; nina_passthrough_check() without and with the request
 *   (cleared at once), nina_passthrough_request() (the words, then the
 *   restart).
 * - The lines from DTR and RTS: the port opened (download mode), RTS
 *   pulsed ("--after hard_reset"), closed (held in reset), DTR then RTS
 *   (esptool's custom_reset_sequence R0|D1|W0.2|R1|W0.6), a start into the
 *   module's own firmware (GPIO0 released after 750 ms, GPIO8 not driven,
 *   DTR ignored), the clock's wrap.
 * - The rate: the exit rate and those out of range keep the UART's rate.
 * - The way out: only the port closed at 300 bps; a port opened at the
 *   stale 300 bps and set to 115200 before it closes does not leave; the
 *   decision stays.
 * - The buffer: wrap, full (counted drops), peak, the counters' wrap, and a
 *   producer thread against a consumer thread (every byte in order or
 *   counted as dropped).
 * - The start (hold, begin): the order of the lines, the UART, its
 *   interrupt, the LED and the first log line.
 * - Through the entry points: the host's lines (the levels in the order
 *   the module needs, only the changes, the closes), the rate, the UART's
 *   interrupt (breaks, overruns, framing errors, the buffer full), host to
 *   module (the FIFO full), module to host (a host that takes a few bytes,
 *   none, or has gone: discarded), the tick (GPIO0's release, the LED at
 *   5 Hz, the time to the next thing to do), the counts in the log, the
 *   way out (its sequence and sleeps, the restart).
 */

#include <assert.h>
#include <pthread.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "fault_record.h"
#include "../drivers/nina/nina_passthrough.c"

/* The fake bus -----------------------------------------------------------*/

static uint32_t scratch[8];
static jmp_buf restart_jmp;
static unsigned int restarts;

static char trace[64][32];
static unsigned int trace_len;

static void note(const char *fmt, ...)
{
	va_list ap;

	if (trace_len < 64) {
		va_start(ap, fmt);
		vsnprintf(trace[trace_len++], sizeof(trace[0]), fmt, ap);
		va_end(ap);
	}
}

static bool traced(const char *const *expected, unsigned int n)
{
	if (trace_len != n) {
		return false;
	}
	for (unsigned int i = 0; i < n; i++) {
		if (strcmp(trace[i], expected[i]) != 0) {
			return false;
		}
	}
	return true;
}

#define TRACED(...)                                                                                \
	traced((const char *const[]){__VA_ARGS__},                                                 \
	       sizeof((const char *const[]){__VA_ARGS__}) / sizeof(const char *))

static int resetn = -1;    /* -1: not driven */
static int gpio0 = -1;     /* enum nina_passthrough_gpio0, -1: untouched */
static int tx_driven = -1; /* GPIO8: 1 the UART's TX, 0 input */
static uint32_t uart_rate;
static unsigned int rate_sets;
static bool irq_on;
static bool led_on;
static unsigned int led_changes;

#define RXQ 65536
static uint32_t rxq[RXQ];
static unsigned int rxq_head, rxq_tail;
#define TXFIFO 32
static uint8_t txfifo[TXFIFO];
static unsigned int txfifo_len;

void nina_passthrough_bus_mark_get(uint32_t s[NINA_PASSTHROUGH_MARK_WORDS])
{
	memcpy(s, scratch, NINA_PASSTHROUGH_MARK_WORDS * sizeof(uint32_t));
}

void nina_passthrough_bus_mark_set(const uint32_t s[NINA_PASSTHROUGH_MARK_WORDS])
{
	memcpy(scratch, s, NINA_PASSTHROUGH_MARK_WORDS * sizeof(uint32_t));
	note("mark");
}

void nina_passthrough_bus_restart(void)
{
	restarts++;
	note("restart");
	longjmp(restart_jmp, 1);
}

void nina_passthrough_bus_lines_init(void)
{
	resetn = 0;
	note("lines");
}

void nina_passthrough_bus_resetn(bool released)
{
	resetn = released;
	note("resetn %d", released);
}

void nina_passthrough_bus_gpio0(enum nina_passthrough_gpio0 state)
{
	gpio0 = (int)state;
	note("gpio0 %d", (int)state);
}

void nina_passthrough_bus_uart_tx_connect(bool connected)
{
	tx_driven = connected;
	note("tx %d", connected);
}

void nina_passthrough_bus_uart_init(uint32_t rate)
{
	uart_rate = rate;
	tx_driven = 0;
	note("uart %u", (unsigned int)rate);
}

void nina_passthrough_bus_uart_set_baud(uint32_t rate)
{
	uart_rate = rate;
	rate_sets++;
}

void nina_passthrough_bus_uart_irq_on(void)
{
	irq_on = true;
	note("irq on");
}

void nina_passthrough_bus_uart_irq_off(void)
{
	irq_on = false;
	note("irq off");
}

bool nina_passthrough_bus_uart_rx(uint32_t *word)
{
	if (rxq_head == rxq_tail) {
		return false;
	}
	*word = rxq[rxq_tail++ % RXQ];
	return true;
}

bool nina_passthrough_bus_uart_tx(uint8_t c)
{
	if (txfifo_len == TXFIFO) {
		return false;
	}
	txfifo[txfifo_len++] = c;
	return true;
}

void nina_passthrough_bus_led_init(void)
{
	led_on = false;
	note("led");
}

void nina_passthrough_bus_led(bool on)
{
	led_on = on;
	led_changes++;
}

/* The fake OS ------------------------------------------------------------*/

static uint32_t now = 5000;
static unsigned int isr_calls;
static bool isr_received;
static uint8_t to_host_buf[65536];
static unsigned int to_host_len;
static int room = 1 << 30; /* bytes the host takes now; -1: no host */
static char last_log[512];
static struct nina_passthrough_counts last_counts;

uint32_t nina_passthrough_os_now_ms(void)
{
	return now;
}

void nina_passthrough_os_sleep_ms(uint32_t ms)
{
	note("sleep %u", (unsigned int)ms);
	now += ms;
}

int nina_passthrough_os_write(const uint8_t *data, uint32_t len)
{
	uint32_t n = len;

	if (room < 0) {
		return -107; /* ENOTCONN */
	}
	if (n > (uint32_t)room) {
		n = (uint32_t)room;
	}
	/* At most 64 bytes at once, as a USB packet. */
	if (n > 64) {
		n = 64;
	}
	memcpy(to_host_buf + to_host_len, data, n);
	to_host_len += n;
	room -= (int)n;
	return (int)n;
}

void nina_passthrough_os_uart_irq(bool received)
{
	isr_calls++;
	isr_received = received;
}

void nina_passthrough_os_log_start(uint32_t rate)
{
	snprintf(last_log, sizeof(last_log), NINA_PASSTHROUGH_LOG_START, NINA_PASSTHROUGH_LOG_START_ARGS(rate));
}

void nina_passthrough_os_log_counts(const char *why, const struct nina_passthrough_counts *c)
{
	snprintf(last_log, sizeof(last_log), NINA_PASSTHROUGH_LOG_COUNTS, NINA_PASSTHROUGH_LOG_COUNTS_ARGS(why, c));
	last_counts = *c;
}

/* Helpers ----------------------------------------------------------------*/

static void module_sends(const uint8_t *data, unsigned int n, uint32_t flags)
{
	for (unsigned int i = 0; i < n; i++) {
		rxq[rxq_head++ % RXQ] = data[i] | flags;
	}
	nina_passthrough_uart_isr();
}

static void check_pins(const struct lines *l, uint32_t at, bool released, enum nina_passthrough_gpio0 g0,
		       bool uart_tx, int line)
{
	struct pins p = lines_pins(l, at);

	if (p.reset_released != released || p.gpio0 != g0 || p.uart_tx != uart_tx) {
		printf("FAIL %s:%d: pins %d %d %d, want %d %d %d\n", __FILE__, line,
		       p.reset_released, p.gpio0, p.uart_tx, released, g0, uart_tx);
		assert(0);
	}
}

#define PINS(l, at, released, g0, tx) check_pins(l, at, released, g0, tx, __LINE__)

/* The decisions ------------------------------------------------------------*/

static void test_mark(void)
{
	uint32_t s[NINA_PASSTHROUGH_MARK_WORDS];
	uint32_t f[FAULT_SCRATCH_WORDS];
	struct fault_record r;
	struct fault_record copy;

	mark(s);
	assert(mark_valid(s));
	assert(!fault_record_from_scratch(s, &copy));
	for (int i = 0; i < NINA_PASSTHROUGH_MARK_WORDS; i++) {
		for (int bit = 0; bit < 32; bit++) {
			uint32_t t[NINA_PASSTHROUGH_MARK_WORDS];

			memcpy(t, s, sizeof(t));
			t[i] ^= 1u << bit;
			assert(!mark_valid(t));
		}
	}
	memset(s, 0, sizeof(s));
	assert(!mark_valid(s));

	/* A fatal error's words (what a fault during the restart leaves). */
	memset(&r, 0, sizeof(r));
	r.reason = FAULT_REASON_HARDFAULT;
	r.pc = 0x10001234u;
	r.lr = 0x10005679u;
	fault_record_to_scratch(&r, f);
	assert(fault_record_from_scratch(f, &copy));
	assert(!mark_valid(f));
}

static void test_lines(void)
{
	struct lines l;

	/* The mode's start: the port closed, the module in reset. */
	lines_init(&l);
	PINS(&l, 0, false, NINA_PASSTHROUGH_GPIO0_HIGH, true);

	/* esptool opens the port (Linux sets DTR and RTS in one request):
	 * GPIO0 low and the reset released, the ROM's download mode.
	 */
	lines_set(&l, true, true, 1000);
	assert(!l.own_firmware);
	PINS(&l, 1000, true, NINA_PASSTHROUGH_GPIO0_LOW, true);
	PINS(&l, 1000 + 60000, true, NINA_PASSTHROUGH_GPIO0_LOW, true);

	/* "--after hard_reset": RTS set (no change), 100 ms, RTS clear: held
	 * in reset with GPIO0 low.
	 */
	lines_set(&l, true, true, 70000);
	PINS(&l, 70000, true, NINA_PASSTHROUGH_GPIO0_LOW, true);
	lines_set(&l, true, false, 70100);
	PINS(&l, 70100, false, NINA_PASSTHROUGH_GPIO0_LOW, true);

	/* The port closed: both clear, in reset with GPIO0 high. */
	lines_set(&l, false, false, 71000);
	PINS(&l, 71000, false, NINA_PASSTHROUGH_GPIO0_HIGH, true);

	/* Opened again with DTR and RTS set one after the other: DTR first
	 * (still in reset), then RTS (download mode).
	 */
	lines_set(&l, true, false, 80000);
	PINS(&l, 80000, false, NINA_PASSTHROUGH_GPIO0_LOW, true);
	lines_set(&l, true, true, 80001);
	assert(!l.own_firmware);
	PINS(&l, 80001, true, NINA_PASSTHROUGH_GPIO0_LOW, true);

	/* DTR cleared while the ROM (or esptool's stub) runs: GPIO0 follows,
	 * the UART stays connected (the sketch's behaviour).
	 */
	lines_set(&l, false, true, 90000);
	PINS(&l, 90000, true, NINA_PASSTHROUGH_GPIO0_HIGH, true);
	PINS(&l, 99999, true, NINA_PASSTHROUGH_GPIO0_HIGH, true);

	/* RTS first with DTR clear: the module's own firmware.  GPIO0 high
	 * for 750 ms, then released; GPIO8 not driven; DTR ignored.
	 */
	lines_set(&l, false, false, 100000);
	lines_set(&l, false, true, 100010);
	assert(l.own_firmware);
	PINS(&l, 100010, true, NINA_PASSTHROUGH_GPIO0_HIGH, false);
	PINS(&l, 100010 + GPIO0_HOLD_MS - 1, true, NINA_PASSTHROUGH_GPIO0_HIGH, false);
	PINS(&l, 100010 + GPIO0_HOLD_MS, true, NINA_PASSTHROUGH_GPIO0_RELEASED, false);
	lines_set(&l, true, true, 101000);
	PINS(&l, 101000, true, NINA_PASSTHROUGH_GPIO0_RELEASED, false);
	PINS(&l, 100010 + 3600000, true, NINA_PASSTHROUGH_GPIO0_RELEASED, false);

	/* Back into reset: the UART again, GPIO0 the strap of DTR. */
	lines_set(&l, true, false, 102000);
	PINS(&l, 102000, false, NINA_PASSTHROUGH_GPIO0_LOW, true);
	lines_set(&l, true, true, 102100);
	assert(!l.own_firmware);
	PINS(&l, 102100, true, NINA_PASSTHROUGH_GPIO0_LOW, true);

	/* The hold across the wrap of the ms clock. */
	lines_init(&l);
	lines_set(&l, false, true, UINT32_MAX - 100);
	PINS(&l, UINT32_MAX - 100, true, NINA_PASSTHROUGH_GPIO0_HIGH, false);
	PINS(&l, 500, true, NINA_PASSTHROUGH_GPIO0_HIGH, false);
	PINS(&l, GPIO0_HOLD_MS - 101, true, NINA_PASSTHROUGH_GPIO0_RELEASED, false);
}

static void test_rate_and_exit(void)
{
	assert(uart_baud(115200, 115200) == 115200);
	assert(uart_baud(921600, 115200) == 921600);
	assert(uart_baud(460800, 921600) == 460800);
	assert(uart_baud(NINA_PASSTHROUGH_EXIT_BAUD, 921600) == 921600);
	assert(uart_baud(0, 115200) == 115200);
	assert(uart_baud(600, 115200) == 115200);
	assert(uart_baud(BAUD_MIN, 115200) == BAUD_MIN);
	assert(uart_baud(BAUD_MAX, 115200) == BAUD_MAX);
	assert(uart_baud(BAUD_MAX + 1, 115200) == 115200);

	/* "stty -F /dev/ttyACM0 300": opened (DTR up at the old rate), set to
	 * 300, closed.
	 */
	assert(!exit_decided(false, true, 115200));
	assert(exit_decided(true, false, NINA_PASSTHROUGH_EXIT_BAUD));
	/* esptool: opened at a stale 300 (Linux keeps the tty's last rate),
	 * set to 115200, then 921600, closed.
	 */
	assert(!exit_decided(false, true, NINA_PASSTHROUGH_EXIT_BAUD));
	assert(!exit_decided(true, true, NINA_PASSTHROUGH_EXIT_BAUD));
	assert(!exit_decided(true, false, 115200));
	assert(!exit_decided(true, false, 921600));
	/* DTR clear already, and other rates. */
	assert(!exit_decided(false, false, NINA_PASSTHROUGH_EXIT_BAUD));
	assert(!exit_decided(true, false, 1200));
}

/* Drains the ring into OUT (at most CAP bytes), as the consumer does. */
static uint32_t drain(struct ring *r, uint8_t *out, uint32_t cap)
{
	const uint8_t *data;
	uint32_t total = 0;
	uint32_t n;

	while (total < cap && (n = ring_peek(r, &data)) > 0) {
		if (n > cap - total) {
			n = cap - total;
		}
		memcpy(out + total, data, n);
		ring_consume(r, n);
		total += n;
	}
	return total;
}

static void test_ring(void)
{
	static uint8_t buf[16];
	struct ring r;
	const uint8_t *data;
	uint8_t out[64];
	uint32_t n;

	ring_init(&r, buf, sizeof(buf));
	assert(ring_peek(&r, &data) == 0);

	/* Fill exactly, then one more: dropped. */
	for (int i = 0; i < 16; i++) {
		ring_put(&r, (uint8_t)i);
	}
	assert(r.dropped == 0);
	ring_put(&r, 99);
	assert(r.dropped == 1);
	n = drain(&r, out, sizeof(out));
	assert(n == 16 && r.peak == 16);
	for (int i = 0; i < 16; i++) {
		assert(out[i] == i);
	}

	/* Across the end of the buffer: the peek stops at the end. */
	for (int i = 0; i < 10; i++) {
		ring_put(&r, (uint8_t)(100 + i));
	}
	n = drain(&r, out, 6);
	assert(n == 6);
	for (int i = 0; i < 10; i++) {
		ring_put(&r, (uint8_t)(110 + i));
	}
	/* 14 waiting from index 22 (6 in the buffer): 10 up to its end. */
	assert(r.head - r.tail == 14);
	n = ring_peek(&r, &data);
	assert(n == 10 && data == buf + 6);
	n = drain(&r, out, sizeof(out));
	assert(n == 14);
	for (int i = 0; i < 14; i++) {
		assert(out[i] == 106 + i);
	}
	assert(r.dropped == 1);

	/* The indices' wrap at 2^32. */
	ring_init(&r, buf, sizeof(buf));
	r.head = r.tail = UINT32_MAX - 4;
	for (int i = 0; i < 12; i++) {
		ring_put(&r, (uint8_t)(200 + i));
	}
	assert(r.head == 7);
	n = drain(&r, out, sizeof(out));
	assert(n == 12);
	for (int i = 0; i < 12; i++) {
		assert(out[i] == 200 + i);
	}
	assert(r.dropped == 0);
}

/* A producer (the interrupt) and a consumer on two threads: the consumer
 * sees the producer's sequence in order, with gaps only where the producer
 * counted drops.
 */
#define STRESS_BYTES 20000000u

static struct ring stress;
static uint8_t stress_buf[1024];
static volatile bool stress_done;

static void *producer(void *arg)
{
	uint32_t seq = 0;

	(void)arg;
	for (uint32_t i = 0; i < STRESS_BYTES; i++) {
		/* A lost byte leaves a gap: the next one has the next value. */
		ring_put(&stress, (uint8_t)seq++);
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

	ring_init(&stress, stress_buf, sizeof(stress_buf));
	assert(pthread_create(&thread, NULL, producer, NULL) == 0);
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
	assert(received + stress.dropped == STRESS_BYTES);
	/* Every gap is a run of drops: no more gaps than drops. */
	assert(gaps <= stress.dropped);
	assert(stress.peak <= sizeof(stress_buf));
}

/* The entry points ---------------------------------------------------------*/

static uint8_t ring_mem[NINA_PASSTHROUGH_RING_SIZE];

static void test_request(void)
{
	/* No request: nothing changes. */
	scratch[0] = 0x46410000u; /* a fatal error record's start, say */
	assert(!nina_passthrough_check() && !nina_passthrough_active());
	assert(scratch[0] == 0x46410000u && trace_len == 0);

	/* "nina passthrough confirm": the words, then the restart. */
	if (setjmp(restart_jmp) == 0) {
		nina_passthrough_request();
	}
	assert(restarts == 1 && TRACED("mark", "restart"));
	assert(mark_valid(scratch));

	/* The next start: the request, cleared at once. */
	trace_len = 0;
	assert(nina_passthrough_check() && nina_passthrough_active());
	for (int i = 0; i < NINA_PASSTHROUGH_MARK_WORDS; i++) {
		assert(scratch[i] == 0);
	}
	assert(TRACED("mark"));
}

static void test_start(void)
{
	trace_len = 0;
	nina_passthrough_hold();
	assert(TRACED("lines") && resetn == 0);

	/* The UART, then every line in the order RESETN last, the interrupt,
	 * the LED, the first line of the log.
	 */
	trace_len = 0;
	nina_passthrough_begin(ring_mem);
	assert(TRACED("uart 115200", "tx 1", "gpio0 1", "resetn 0", "irq on", "led"));
	assert(irq_on && tx_driven == 1 && gpio0 == NINA_PASSTHROUGH_GPIO0_HIGH && !led_on);
	assert(!strcmp(last_log, "NINA passthrough: USB <-> UART1 (GPIO8 TX, GPIO9 RX) at "
				 "115200 bps, DTR -> GPIO0 (GPIO2), RTS -> RESETN (GPIO3); close "
				 "the port at 300 bps or press RESET to leave"));
}

static void test_host_lines(void)
{
	/* Linux opens the port: DTR and RTS at once, GPIO0 settled before
	 * the reset is released.
	 */
	trace_len = 0;
	assert(!nina_passthrough_host_lines(true, true, 115200));
	assert(TRACED("gpio0 0", "resetn 1"));

	/* The same lines again: nothing. */
	trace_len = 0;
	assert(!nina_passthrough_host_lines(true, true, 115200));
	assert(trace_len == 0);

	/* esptool's custom_reset_sequence R0|D1|W0.2|R1|W0.6: into reset with
	 * DTR (GPIO0 low), then released into the download mode, RTS kept.
	 */
	assert(!nina_passthrough_host_lines(true, false, 115200));
	assert(resetn == 0 && gpio0 == NINA_PASSTHROUGH_GPIO0_LOW && tx_driven == 1);
	now += 200;
	assert(!nina_passthrough_host_lines(true, true, 115200));
	assert(resetn == 1 && gpio0 == NINA_PASSTHROUGH_GPIO0_LOW && tx_driven == 1);

	/* esptool exits (--after no-reset) and the port closes at 921600:
	 * held in reset, a close, no way out.
	 */
	trace_len = 0;
	assert(nina_passthrough_host_lines(false, false, 921600));
	assert(TRACED("gpio0 1", "resetn 0") && !nina_passthrough_exit_requested());

	/* RTS alone (a terminal with DTR off): the module's own firmware;
	 * GPIO8 let go and GPIO0 high before the release; then 750 ms.
	 */
	trace_len = 0;
	assert(!nina_passthrough_host_lines(false, true, 115200));
	assert(TRACED("tx 0", "resetn 1"));
	assert(nina_passthrough_tick() <= GPIO0_HOLD_MS);
	now += GPIO0_HOLD_MS - 1;
	(void)nina_passthrough_tick();
	assert(gpio0 == NINA_PASSTHROUGH_GPIO0_HIGH);
	now += 1;
	(void)nina_passthrough_tick();
	assert(gpio0 == NINA_PASSTHROUGH_GPIO0_RELEASED && tx_driven == 0);
	/* DTR is ignored while the module's firmware runs. */
	assert(!nina_passthrough_host_lines(true, true, 115200));
	assert(gpio0 == NINA_PASSTHROUGH_GPIO0_RELEASED && tx_driven == 0);
	assert(nina_passthrough_host_lines(false, false, 115200));
	assert(gpio0 == NINA_PASSTHROUGH_GPIO0_HIGH && tx_driven == 1 && resetn == 0);
	assert(!nina_passthrough_exit_requested());

	/* At 300 bps without the port open (DTR never set), and with RTS
	 * dropped alone: no way out.
	 */
	assert(!nina_passthrough_host_lines(false, true, NINA_PASSTHROUGH_EXIT_BAUD));
	assert(!nina_passthrough_host_lines(false, false, NINA_PASSTHROUGH_EXIT_BAUD));
	assert(!nina_passthrough_host_lines(true, true, NINA_PASSTHROUGH_EXIT_BAUD));
	assert(!nina_passthrough_host_lines(true, false, NINA_PASSTHROUGH_EXIT_BAUD));
	assert(!nina_passthrough_exit_requested());
	assert(nina_passthrough_host_lines(false, false, 115200));
	assert(!nina_passthrough_exit_requested());
}

static void test_rate(void)
{
	unsigned int sets = rate_sets;

	nina_passthrough_host_rate(921600);
	assert(uart_rate == 921600 && rate_sets == sets + 1);
	/* The same rate, the exit rate, those out of range: no change. */
	nina_passthrough_host_rate(921600);
	nina_passthrough_host_rate(NINA_PASSTHROUGH_EXIT_BAUD);
	nina_passthrough_host_rate(0);
	nina_passthrough_host_rate(BAUD_MAX + 1);
	assert(uart_rate == 921600 && rate_sets == sets + 1);
	nina_passthrough_host_rate(115200);
	assert(uart_rate == 115200 && rate_sets == sets + 2);
}

static void test_to_module(void)
{
	uint8_t data[100];
	uint32_t n;

	for (unsigned int i = 0; i < sizeof(data); i++) {
		data[i] = (uint8_t)(i * 7 + 1);
	}
	txfifo_len = 0;
	n = nina_passthrough_to_module(data, sizeof(data));
	assert(n == TXFIFO && !memcmp(txfifo, data, TXFIFO));
	assert(nina_passthrough_to_module(data + n, sizeof(data) - n) == 0);
	txfifo_len = 0;
	assert(nina_passthrough_to_module(data + n, 10) == 10 && !memcmp(txfifo, data + n, 10));
	assert(nina_passthrough_to_module(data, 0) == 0);
	nina_passthrough_log_counts("x");
	assert(last_counts.to_nina == TXFIFO + 10);
	txfifo_len = 0;
}

static void test_to_host(void)
{
	static uint8_t data[NINA_PASSTHROUGH_RING_SIZE + 3000];
	const uint64_t to_nina_before = to_nina;

	for (unsigned int i = 0; i < sizeof(data); i++) {
		data[i] = (uint8_t)(i * 13 + 5);
	}

	/* Nothing waits: nothing to do. */
	assert(!nina_passthrough_waiting() && !nina_passthrough_to_host());

	/* The interrupt fills the buffer; the host takes it in packets. */
	isr_calls = 0;
	module_sends(data, 100, 0);
	assert(isr_calls == 1 && isr_received && nina_passthrough_waiting());
	to_host_len = 0;
	assert(nina_passthrough_to_host());
	assert(to_host_len == 100 && !memcmp(to_host_buf, data, 100));
	assert(!nina_passthrough_waiting());

	/* A host that takes 10 bytes now: the rest waits. */
	module_sends(data + 100, 1000, 0);
	room = 10;
	assert(nina_passthrough_to_host());
	assert(to_host_len == 110 && nina_passthrough_waiting());
	room = 0;
	assert(!nina_passthrough_to_host() && to_host_len == 110);
	room = 1 << 30;
	assert(nina_passthrough_to_host() && to_host_len == 1100);
	assert(!memcmp(to_host_buf, data, 1100));

	/* Breaks are no data; overruns and framing errors are counted. */
	module_sends(data, 3, NINA_PASSTHROUGH_UART_BREAK);
	assert(!isr_received && !nina_passthrough_waiting());
	module_sends(data + 1100, 1, NINA_PASSTHROUGH_UART_OVERRUN);
	module_sends(data + 1101, 1, NINA_PASSTHROUGH_UART_FRAMING);
	module_sends(data + 1102, 1, NINA_PASSTHROUGH_UART_OVERRUN | NINA_PASSTHROUGH_UART_BREAK);
	assert(nina_passthrough_to_host() && to_host_len == 1102);
	assert(!memcmp(to_host_buf + 1100, data + 1100, 2));

	/* More than the buffer while the host does not read: counted as
	 * lost, the peak at the buffer's size.
	 */
	module_sends(data, sizeof(data), 0);
	room = 0;
	(void)nina_passthrough_to_host();
	room = 1 << 30;
	to_host_len = 0;
	assert(nina_passthrough_to_host() && to_host_len == NINA_PASSTHROUGH_RING_SIZE);
	assert(!memcmp(to_host_buf, data, NINA_PASSTHROUGH_RING_SIZE));

	/* No host: discarded; and the discard without the host. */
	module_sends(data, 500, 0);
	room = -1;
	assert(!nina_passthrough_to_host() && !nina_passthrough_waiting());
	module_sends(data, 70, 0);
	nina_passthrough_discard();
	assert(!nina_passthrough_waiting());
	room = 1 << 30;

	nina_passthrough_log_counts("Port closed");
	assert(last_counts.from_nina == 1102 + NINA_PASSTHROUGH_RING_SIZE);
	assert(last_counts.discarded == 570 && last_counts.lost == 3000);
	assert(last_counts.peak == NINA_PASSTHROUGH_RING_SIZE &&
	       last_counts.size == NINA_PASSTHROUGH_RING_SIZE);
	assert(last_counts.overruns == 2 && last_counts.breaks == 4 && last_counts.framing == 1);
	assert(last_counts.to_nina == to_nina_before && last_counts.baud == 115200);
	assert(!strncmp(last_log, "Port closed: 42 bytes to the NINA, ", 35));
	assert(strstr(last_log, ", 3000 lost (buffer full, peak 16384 of 16384), UART: 2 overruns, "
				"4 breaks, 1 framing errors, 115200 bps") != NULL);
}

static void test_tick(void)
{
	unsigned int changes;
	uint32_t ms;

	/* The LED every 100 ms; the time to its next change. */
	led_at = now;
	changes = led_changes;
	ms = nina_passthrough_tick();
	assert(led_changes == changes + 1 && led_on && ms == LED_HALF_MS);
	now += 40;
	assert(nina_passthrough_tick() == 60 && led_changes == changes + 1);
	now += 60;
	assert(nina_passthrough_tick() == LED_HALF_MS && !led_on);
	/* Late by 30 ms: on its old beat; late by more than a period: from
	 * now on.
	 */
	now += 130;
	assert(nina_passthrough_tick() == 70 && led_on);
	now += 500;
	assert(nina_passthrough_tick() == LED_HALF_MS && !led_on);

	/* GPIO0's release is the next thing to do when it comes first. */
	assert(!nina_passthrough_host_lines(false, true, 115200));
	now += GPIO0_HOLD_MS - 20;
	ms = nina_passthrough_tick();
	assert(ms <= 20 && gpio0 == NINA_PASSTHROUGH_GPIO0_HIGH);
	now += 20;
	ms = nina_passthrough_tick();
	assert(gpio0 == NINA_PASSTHROUGH_GPIO0_RELEASED && ms >= 1 && ms <= LED_HALF_MS);
	assert(nina_passthrough_host_lines(false, false, 115200) == false);
}

static void test_way_out(void)
{
	/* Opened at the stale 300 bps, set to 115200, closed: stays. */
	assert(!nina_passthrough_host_lines(true, true, NINA_PASSTHROUGH_EXIT_BAUD));
	assert(nina_passthrough_host_lines(false, false, 115200));
	assert(!nina_passthrough_exit_requested());

	/* Closed at 300 bps: the way out, which stays decided. */
	assert(!nina_passthrough_host_lines(true, true, 115200));
	assert(nina_passthrough_host_lines(false, false, NINA_PASSTHROUGH_EXIT_BAUD));
	assert(nina_passthrough_exit_requested());
	assert(!nina_passthrough_host_lines(true, true, 115200));
	assert(nina_passthrough_exit_requested());

	trace_len = 0;
	nina_passthrough_stop();
	assert(TRACED("irq off") && !irq_on);

	trace_len = 0;
	led_on = true;
	if (setjmp(restart_jmp) == 0) {
		nina_passthrough_leave();
	}
	assert(TRACED("tx 0", "gpio0 1", "resetn 0", "sleep 10", "resetn 1", "sleep 750", "gpio0 2",
		      "sleep 100", "restart"));
	assert(!led_on && !strncmp(last_log, "Leaving the NINA passthrough: ", 30));
	assert(!mark_valid(scratch));
}

int main(void)
{
	test_mark();
	test_lines();
	test_rate_and_exit();
	test_ring();
	test_ring_threads();
	test_request();
	test_start();
	test_host_lines();
	test_rate();
	test_to_module();
	test_to_host();
	test_tick();
	test_way_out();
	puts("nina_passthrough: all passed");
	return 0;
}
