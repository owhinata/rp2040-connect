/* SPDX-License-Identifier: MIT */
/* Host test of this build's NINA passthrough mode (drivers/nina/
 * nina_passthrough.c, which the test includes), on fakes of the Zephyr
 * calls (tests/passthrough_fakes) and of the RP2040 hardware
 * (nina_passthrough_hw.h, defined here: the scratch registers, the
 * module's lines, UART1's FIFOs and interrupt mask).  The decisions are
 * common/drivers/nina/nina_passthrough_logic.h's, which
 * freertos/tests/passthrough tests; this test checks the glue around them.
 *
 * - The start: no request, no mode (nothing touched); a request is cleared
 *   at once, holds the module in reset with GPIO0 high, takes the USB
 *   messages, then (POST_KERNEL 99) UART1 at 115200 bps, its interrupt at
 *   priority 0, the CDC ACM callback in place of the shell's.
 * - The lines on the stack's messages and on the tick: esptool's sequence
 *   (download mode), a start into the module's firmware (GPIO0 released
 *   after 750 ms, GPIO8 not driven), a change seen by the tick only.
 * - The rate on LINE_CODING: followed, but not to the exit rate.
 * - Host to module: a packet partly into the FIFO, the rest by the
 *   transmit interrupt while the CDC ACM receive interrupt is off, in
 *   order; many packets at once.
 * - Module to host: the receive interrupt into the buffer, breaks dropped
 *   and counted, overruns counted; the CDC ACM transmit interrupt turned
 *   on by the interrupt and off when the buffer is empty; a host that takes
 *   a few bytes at a time; dropped and counted while USB is not configured.
 * - The way out: a port opened at the stale 300 bps and set to 115200
 *   before it closes stays; closing at 300 bps turns everything off, resets
 *   the module into its firmware (GPIO0 high, RESETN low 10 ms, 750 ms,
 *   GPIO0 released) and restarts without a request.
 * - The command: without "confirm" nothing happens; with it the request
 *   and the restart.
 */
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define CONFIG_NINA_PASSTHROUGH 1

#include "nina_passthrough_logic.h"

/* The fake of nina_passthrough_hw.h --------------------------------------*/

#define PT_HW_UART_IRQN  21
#define PT_HW_DR_OVERRUN 0x800u
#define PT_HW_DR_BREAK   0x400u
#define PT_HW_DR_FRAMING 0x100u

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

static int resetn = -1;    /* -1: not driven */
static int gpio0 = -1;     /* enum nina_pt_gpio0, -1: untouched */
static int tx_driven = -1; /* GPIO8: 1 UART1 TX, 0 input */
static unsigned int uart_inits;
static uint32_t uart_rate;
static bool uart_irqs = false;
static bool txim;
static unsigned int led_toggles;
static bool led_on;

#define RXQ 4096
static uint32_t rxq[RXQ];
static unsigned int rxq_head, rxq_tail;
#define TXFIFO 32
static uint8_t txfifo[TXFIFO];
static unsigned int txfifo_len;
static uint8_t wire[8192]; /* what left UART1 for the module */
static unsigned int wire_len;

static inline void pt_hw_mark_get(uint32_t s[NINA_PT_MARK_WORDS])
{
	memcpy(s, scratch, NINA_PT_MARK_WORDS * sizeof(uint32_t));
}

static inline void pt_hw_mark_set(const uint32_t s[NINA_PT_MARK_WORDS])
{
	memcpy(scratch, s, NINA_PT_MARK_WORDS * sizeof(uint32_t));
}

static inline __attribute__((noreturn)) void pt_hw_restart(void)
{
	restarts++;
	note("restart");
	longjmp(restart_jmp, 1);
}

static inline void pt_hw_lines_init(void)
{
	resetn = 0;
	note("lines init");
}

static inline void pt_hw_resetn(bool released)
{
	resetn = released;
	note("resetn %d", released);
}

static inline void pt_hw_gpio0(enum nina_pt_gpio0 state)
{
	gpio0 = (int)state;
	note("gpio0 %d", (int)state);
}

static inline void pt_hw_uart_tx_connect(bool connected)
{
	tx_driven = connected;
	note("tx %d", connected);
}

static inline void pt_hw_uart_set_baud(uint32_t baud)
{
	uart_rate = baud;
}

static inline void pt_hw_uart_init(uint32_t baud)
{
	uart_inits++;
	uart_rate = baud;
	uart_irqs = true;
	tx_driven = 0;
}

static inline void pt_hw_uart_irqs_off(void)
{
	uart_irqs = false;
	txim = false;
}

static inline void pt_hw_uart_tx_irq(bool on)
{
	txim = on;
}

static inline bool pt_hw_uart_tx_irq_on(void)
{
	return txim;
}

static inline bool pt_hw_uart_rx(uint32_t *dr)
{
	if (rxq_head == rxq_tail) {
		return false;
	}
	*dr = rxq[rxq_tail++ % RXQ];
	return true;
}

static inline bool pt_hw_uart_tx(uint8_t c)
{
	if (txfifo_len == TXFIFO) {
		return false;
	}
	txfifo[txfifo_len++] = c;
	return true;
}

static inline void pt_hw_led_init(void)
{
	led_on = false;
}

static inline void pt_hw_led_toggle(void)
{
	led_on = !led_on;
	led_toggles++;
}

static inline void pt_hw_led_off(void)
{
	led_on = false;
}

/* The real header is not read. */
#define ZEPHYR_APP_DRIVERS_NINA_NINA_PASSTHROUGH_HW_H

#include "nina_passthrough.c"

/* The fakes of the Zephyr calls -------------------------------------------*/

const struct device fake_cdc_dev;
struct usbd_context fake_usbd_contexts[1];
char fake_log[512];
unsigned int fake_logs;
char fake_shell_out[512];

static usbd_msg_cb_t msg_cb;
static int register_ret;
static unsigned int irq_connects, irq_prio;
static void (*irq_isr)(const void *arg);
static bool irq_on;
static uint32_t now;
static unsigned int schedules;

/* The CDC ACM port: the host's lines and data, the driver's callback and
 * interrupt flags, the room in its transmit buffers.
 */
static uint32_t cdc_dtr, cdc_rts, cdc_rate = 115200;
static uart_irq_callback_user_data_t cdc_cb;
static bool cdc_rx_on, cdc_tx_on;
static unsigned int cdc_tx_enables;
static uint8_t from_host[8192];
static unsigned int from_host_len, from_host_pos;
static uint8_t to_host[65536];
static unsigned int to_host_len;
static unsigned int room = 1u << 30; /* bytes the host takes now */

void fake_irq_connect(unsigned int irq, unsigned int prio, void (*isr)(const void *arg))
{
	assert(irq == PT_HW_UART_IRQN);
	irq_connects++;
	irq_prio = prio;
	irq_isr = isr;
}

void irq_enable(unsigned int irq)
{
	assert(irq == PT_HW_UART_IRQN);
	irq_on = true;
}

void irq_disable(unsigned int irq)
{
	assert(irq == PT_HW_UART_IRQN);
	irq_on = false;
}

void k_sem_give(struct k_sem *sem)
{
	if (sem->count < sem->limit) {
		sem->count++;
	}
}

int k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
	assert(timeout.ms == -1);
	assert(sem->count > 0); /* the test never blocks */
	sem->count--;
	return 0;
}

int k_work_schedule(struct k_work_delayable *dwork, k_timeout_t delay)
{
	assert(dwork == &tick_work);
	assert(delay.ms == 0 || delay.ms == PT_TICK_MS);
	schedules++;
	return 0;
}

bool k_work_cancel_delayable_sync(struct k_work_delayable *dwork, struct k_work_sync *sync)
{
	(void)sync;
	assert(dwork == &tick_work);
	return false;
}

uint32_t k_uptime_get_32(void)
{
	return now;
}

int32_t k_msleep(int32_t ms)
{
	note("sleep %d", (int)ms);
	now += (uint32_t)ms;
	return 0;
}

atomic_val_t atomic_set(atomic_t *target, atomic_val_t value)
{
	atomic_val_t old = *target;

	*target = value;
	return old;
}

bool atomic_cas(atomic_t *target, atomic_val_t old_value, atomic_val_t new_value)
{
	if (*target != old_value) {
		return false;
	}
	*target = new_value;
	return true;
}

int uart_line_ctrl_get(const struct device *dev, uint32_t ctrl, uint32_t *val)
{
	assert(dev == &fake_cdc_dev);
	switch (ctrl) {
	case UART_LINE_CTRL_DTR:
		*val = cdc_dtr;
		return 0;
	case UART_LINE_CTRL_RTS:
		*val = cdc_rts;
		return 0;
	case UART_LINE_CTRL_BAUD_RATE:
		*val = cdc_rate;
		return 0;
	default:
		return -ENOTSUP;
	}
}

int uart_irq_callback_user_data_set(const struct device *dev, uart_irq_callback_user_data_t cb,
				    void *user_data)
{
	assert(dev == &fake_cdc_dev && user_data == NULL);
	cdc_cb = cb;
	return 0;
}

void uart_irq_rx_enable(const struct device *dev)
{
	assert(dev == &fake_cdc_dev);
	cdc_rx_on = true;
}

void uart_irq_rx_disable(const struct device *dev)
{
	assert(dev == &fake_cdc_dev);
	cdc_rx_on = false;
}

void uart_irq_tx_enable(const struct device *dev)
{
	assert(dev == &fake_cdc_dev);
	cdc_tx_on = true;
	cdc_tx_enables++;
}

void uart_irq_tx_disable(const struct device *dev)
{
	assert(dev == &fake_cdc_dev);
	cdc_tx_on = false;
}

void uart_irq_update(const struct device *dev)
{
	assert(dev == &fake_cdc_dev);
}

int uart_irq_rx_ready(const struct device *dev)
{
	assert(dev == &fake_cdc_dev);
	return cdc_rx_on && from_host_pos < from_host_len;
}

int uart_irq_tx_ready(const struct device *dev)
{
	assert(dev == &fake_cdc_dev);
	return cdc_tx_on && room > 0 ? (int)(room < 64 ? room : 64) : 0;
}

int uart_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	unsigned int n = from_host_len - from_host_pos;

	assert(dev == &fake_cdc_dev);
	if (n > (unsigned int)size) {
		n = (unsigned int)size;
	}
	memcpy(rx_data, from_host + from_host_pos, n);
	from_host_pos += n;
	return (int)n;
}

int uart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	unsigned int n = (unsigned int)size;

	assert(dev == &fake_cdc_dev);
	if (n > room) {
		n = room;
	}
	memcpy(to_host + to_host_len, tx_data, n);
	to_host_len += n;
	room -= n;
	return (int)n;
}

int usbd_msg_register_cb(struct usbd_context *const ctx, const usbd_msg_cb_t cb)
{
	assert(ctx == &fake_usbd_contexts[0]);
	msg_cb = cb;
	return register_ret;
}

/* Helpers -----------------------------------------------------------------*/

/* The class driver's work: the callback while it has something to do. */
static void pump(void)
{
	for (int i = 0; i < 10000; i++) {
		bool rx = cdc_rx_on && from_host_pos < from_host_len;
		bool tx = cdc_tx_on && room > 0;

		if (!rx && !tx) {
			return;
		}
		cdc_cb(&fake_cdc_dev, NULL);
	}
	assert(!"the callback does not settle");
}

/* The UART sends what is in its FIFO; its interrupt comes. */
static void uart_send(void)
{
	memcpy(wire + wire_len, txfifo, txfifo_len);
	wire_len += txfifo_len;
	txfifo_len = 0;
	if (irq_on && (txim || rxq_head != rxq_tail)) {
		irq_isr(NULL);
	}
}

static void module_sends(const uint8_t *data, unsigned int n, uint32_t flags)
{
	for (unsigned int i = 0; i < n; i++) {
		rxq[rxq_head++ % RXQ] = data[i] | flags;
	}
	if (irq_on) {
		irq_isr(NULL);
	}
}

static void lines_msg(uint32_t dtr, uint32_t rts)
{
	cdc_dtr = dtr;
	cdc_rts = rts;
	msg_cb(&fake_usbd_contexts[0],
	       &(struct usbd_msg){.type = USBD_MSG_CDC_ACM_CONTROL_LINE_STATE, .dev = cdc});
}

static void coding_msg(uint32_t rate)
{
	cdc_rate = rate;
	msg_cb(&fake_usbd_contexts[0],
	       &(struct usbd_msg){.type = USBD_MSG_CDC_ACM_LINE_CODING, .dev = cdc});
}

static void config_msg(int value)
{
	msg_cb(&fake_usbd_contexts[0],
	       &(struct usbd_msg){.type = USBD_MSG_CONFIGURATION, .status = value});
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

/* The tests ---------------------------------------------------------------*/

static void test_no_request(void)
{
	scratch[0] = 0x46410000u; /* a fatal error record's start, say */
	assert(pt_check() == 0 && pt_start() == 0);
	assert(!nina_passthrough_active());
	assert(scratch[0] == 0x46410000u);
	assert(trace_len == 0 && uart_inits == 0 && irq_connects == 0 && msg_cb == NULL &&
	       cdc_cb == NULL && schedules == 0);
	scratch[0] = 0;
}

static void test_start(void)
{
	static const char *const early[] = {"lines init", "gpio0 1"};
	static const char *const later[] = {"tx 1"};

	nina_pt_mark(scratch);
	assert(pt_check() == 0);
	assert(nina_passthrough_active());
	for (int i = 0; i < NINA_PT_MARK_WORDS; i++) {
		assert(scratch[i] == 0);
	}
	assert(traced(early, 2) && resetn == 0 && gpio0 == NINA_PT_GPIO0_HIGH);
	assert(msg_cb == usb_msg && !configured);

	trace_len = 0;
	assert(pt_start() == 0);
	/* applied_valid was cleared: every line again, in the order RESETN last. */
	assert(trace_len == 3 && !strcmp(trace[0], later[0]) && !strcmp(trace[1], "gpio0 1") &&
	       !strcmp(trace[2], "resetn 0"));
	assert(uart_inits == 1 && uart_rate == 115200 && uart_irqs && tx_driven == 1);
	assert(irq_connects == 1 && irq_prio == 0 && irq_on);
	assert(cdc_cb == cdc_irq && cdc_rx_on);
	assert(schedules == 1);
	assert(strstr(fake_log, "NINA passthrough: USB <-> UART1") != NULL);
	config_msg(1);
	assert(configured);
}

static void test_lines(void)
{
	/* esptool's custom_reset_sequence R0|D1|W0.2|R1|W0.6, after Linux's
	 * open (both set): in reset with GPIO0 low, then released into the
	 * ROM's download mode.
	 */
	lines_msg(1, 1);
	assert(resetn == 1 && gpio0 == NINA_PT_GPIO0_LOW && tx_driven == 1);
	lines_msg(1, 0);
	assert(resetn == 0 && gpio0 == NINA_PT_GPIO0_LOW);
	now += 200;
	lines_msg(1, 1);
	assert(resetn == 1 && gpio0 == NINA_PT_GPIO0_LOW && tx_driven == 1);

	/* The same state again changes nothing. */
	trace_len = 0;
	lines_msg(1, 1);
	tick(NULL);
	assert(trace_len == 0);

	/* A start into the module's firmware (DTR clear, RTS set), seen by the
	 * tick only (the message came late): GPIO0 high, GPIO8 not driven,
	 * GPIO0 released after 750 ms, whatever DTR does then.
	 */
	cdc_rts = 0;
	tick(NULL);
	assert(resetn == 0);
	cdc_dtr = 0;
	cdc_rts = 1;
	now += 10;
	tick(NULL);
	assert(resetn == 1 && gpio0 == NINA_PT_GPIO0_HIGH && tx_driven == 0);
	now += 749;
	tick(NULL);
	assert(gpio0 == NINA_PT_GPIO0_HIGH);
	now += 1;
	tick(NULL);
	assert(gpio0 == NINA_PT_GPIO0_RELEASED && tx_driven == 0);

	/* Closed (both clear): held in reset, GPIO0 high, GPIO8 driven idle. */
	lines_msg(0, 0);
	assert(resetn == 0 && gpio0 == NINA_PT_GPIO0_HIGH && tx_driven == 1);
}

static void test_rate(void)
{
	coding_msg(921600);
	assert(uart_rate == 921600 && baud == 921600);
	coding_msg(NINA_PT_EXIT_BAUD);
	assert(uart_rate == 921600);
	cdc_rate = 460800;
	tick(NULL);
	assert(uart_rate == 460800);
	coding_msg(115200);
	assert(uart_rate == 115200);
}

static void test_host_to_nina(void)
{
	unsigned int n = 1000;

	lines_msg(1, 1);
	for (unsigned int i = 0; i < n; i++) {
		from_host[i] = (uint8_t)(i * 7 + 1);
	}
	from_host_len = n;
	from_host_pos = 0;
	wire_len = 0;

	/* The first packet: 32 bytes into the FIFO, the rest to the interrupt,
	 * the CDC ACM receive interrupt off meanwhile.
	 */
	pump();
	assert(from_host_pos == 64 && txfifo_len == TXFIFO && txim && !cdc_rx_on);
	assert(out_len == 64 && out_pos == TXFIFO);
	for (int i = 0; i < 1000 && wire_len < n; i++) {
		uart_send();
		pump();
	}
	uart_send();
	assert(wire_len == n && !memcmp(wire, from_host, n));
	assert(!txim && cdc_rx_on && to_nina == n);

	/* A packet that fits: no interrupt's turn. */
	from_host_len = from_host_pos + 20;
	pump();
	assert(!txim && cdc_rx_on && txfifo_len == 20);
	uart_send();
	assert(wire_len == n + 20);
}

static void test_nina_to_host(void)
{
	uint8_t data[3000];
	unsigned int base;

	for (unsigned int i = 0; i < sizeof(data); i++) {
		data[i] = (uint8_t)(i * 13 + 5);
	}
	to_host_len = 0;
	pump();
	assert(!cdc_tx_on);

	/* The interrupt fills the buffer and turns the CDC ACM transmit
	 * interrupt on; the callback empties it and turns it off.
	 */
	assert(tx_idle == 1);
	module_sends(data, 100, 0);
	assert(cdc_tx_on);
	pump();
	assert(to_host_len == 100 && !memcmp(to_host, data, 100) && !cdc_tx_on && tx_idle == 1);

	/* A host that takes 10 bytes at a time: the rest waits in the buffer,
	 * the interrupt stays on.
	 */
	room = 10;
	module_sends(data + 100, 1000, 0);
	pump();
	assert(to_host_len == 110 && cdc_tx_on);
	while (to_host_len < 1100) {
		room = 10;
		pump();
	}
	assert(!memcmp(to_host, data, 1100) && !cdc_tx_on);
	room = 1u << 30;

	/* Breaks are no data; overruns and framing errors are counted. */
	base = to_host_len;
	module_sends(data, 3, PT_HW_DR_BREAK);
	module_sends(data + 1100, 1, PT_HW_DR_OVERRUN);
	module_sends(data + 1101, 1, PT_HW_DR_FRAMING);
	pump();
	assert(uart_breaks == 3 && uart_overruns == 1 && uart_framing == 1);
	assert(to_host_len == base + 2 && !memcmp(to_host + base, data + 1100, 2));

	/* The tick turns it on when something waits (a missed turn). */
	nina_pt_ring_put(&ring, 0x55);
	assert(!cdc_tx_on);
	tick(NULL);
	assert(cdc_tx_on);
	pump();
	assert(to_host[to_host_len - 1] == 0x55);

	/* More than the buffer while the host does not read: counted as lost. */
	room = 0;
	for (unsigned int i = 0; i < PT_RING_SIZE / sizeof(data) + 2; i++) {
		module_sends(data, sizeof(data), 0);
	}
	assert(ring.dropped > 0 && ring.peak <= PT_RING_SIZE);
	room = 1u << 30;
	pump();
	tick(NULL);
	pump();

	/* Not configured: dropped by the tick and counted apart. */
	config_msg(0);
	base = to_host_len;
	module_sends(data, 500, 0);
	tick(NULL);
	assert(discarded == 500);
	pump();
	assert(to_host_len == base);
	config_msg(1);
}

static void test_way_out(void)
{
	static const char *const expected[] = {
		"tx 0",      "gpio0 1", "resetn 0",  "sleep 10", "resetn 1",
		"sleep 750", "gpio0 2", "sleep 100", "restart",
	};

	/* Opened at the stale 300 bps, set to 115200, closed: stays. */
	lines_msg(0, 0);
	cdc_rate = NINA_PT_EXIT_BAUD;
	lines_msg(1, 1);
	coding_msg(115200);
	lines_msg(0, 0);
	assert(!leaving && leave_sem.count == 0);
	assert(strstr(fake_log, "Port closed:") != NULL);

	/* Closed at 300 bps: everything off, then main()'s way out. */
	lines_msg(1, 1);
	coding_msg(NINA_PT_EXIT_BAUD);
	lines_msg(0, 0);
	assert(leaving && leave_sem.count == 1 && !irq_on && !uart_irqs && !cdc_rx_on &&
	       !cdc_tx_on);
	assert(resetn == 0);

	/* Nothing moves once leaving. */
	trace_len = 0;
	lines_msg(1, 1);
	tick(NULL);
	assert(trace_len == 0);

	if (setjmp(restart_jmp) == 0) {
		nina_passthrough_run();
	}
	assert(restarts == 1 && traced(expected, sizeof(expected) / sizeof(expected[0])));
	assert(strstr(fake_log, "Leaving the NINA passthrough:") != NULL && !led_on);
	assert(!nina_pt_mark_valid(scratch));
}

static void test_command(void)
{
	char *wrong[] = {"passthrough", "now"};
	char *right[] = {"passthrough", "confirm"};
	struct shell sh = {0};

	restarts = 0;
	assert(nina_passthrough_cmd(&sh, 2, wrong) == -EINVAL);
	assert(restarts == 0 && !nina_pt_mark_valid(scratch));
	assert(strstr(fake_shell_out, "nina passthrough confirm") != NULL);

	trace_len = 0;
	if (setjmp(restart_jmp) == 0) {
		(void)nina_passthrough_cmd(&sh, 2, right);
	}
	assert(restarts == 1 && nina_pt_mark_valid(scratch));
	assert(trace_len == 2 && !strcmp(trace[0], "sleep 100") && !strcmp(trace[1], "restart"));
}

int main(void)
{
	test_no_request();
	test_start();
	test_lines();
	test_rate();
	test_host_to_nina();
	test_nina_to_host();
	test_way_out();
	test_command();
	puts("NINA passthrough on Zephyr: tests passed");
	return 0;
}
