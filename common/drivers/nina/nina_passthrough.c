/* SPDX-License-Identifier: MIT */
/* The NINA passthrough mode (nina_passthrough.h), without an OS: the
 * request, the module's lines from the host's DTR and RTS, the UART's rate
 * from the host's line coding, the data both ways, the way out, the counts
 * and the LED.  The module's side is nina_passthrough_bus.h, the host's
 * nina_passthrough_os.h; the OS binding calls the functions below at its
 * events.
 *
 * - The request: four words, "NINA" "PASS" and their complements.  The
 *   words are those of the fatal error record (common/log/fault_record.h),
 *   which starts with "FA" in the high half of the first; this one starts
 *   with "NI", so neither is taken for the other, and a fatal error during
 *   the restart replaces the request (the next start is a normal one).
 * - The lines (lines_set(), lines_pins()): what DTR and RTS
 *   (SET_CONTROL_LINE_STATE) make of the module's GPIO0 and RESETN, and
 *   whether the UART drives the module's receive line (GPIO8).  The
 *   mapping is the sketch's: RESETN follows RTS (RTS set releases the
 *   reset), GPIO0 is low while DTR is set.  So opening the port (Linux sets
 *   both at once) starts the ROM's download mode, and closing it holds the
 *   module in reset.  esptool keeps RTS set with "--before default-reset
 *   --after no-reset" and a custom_reset_sequence of "R0|D1|W0.2|R1|W0.6"
 *   (ESPTOOL_CFGFILE); with "--before no-reset" esptool 5.3.0 got no
 *   answer.  Unlike the sketch, when the module leaves reset into its own
 *   firmware (DTR clear), GPIO0 is released after 750 ms and GPIO8 stops
 *   driving, since the firmware drives both nets.  The levels change in
 *   the order the module needs: GPIO8 and GPIO0 settle before RESETN, so
 *   the strap is in place when the reset is released.
 * - The rate (uart_baud()) and the way out (exit_decided()) from the host's
 *   line coding (SET_LINE_CODING): the UART follows the rate (esptool's
 *   --baud: the stub's CHANGE_BAUDRATE, then the port's rate), 8N1 always;
 *   closing the port at NINA_PT_EXIT_BAUD ends the mode.  A port opened at
 *   that rate (Linux keeps a tty's last rate) and set to another one before
 *   it is closed does not end it.  1200 bps is the Pico SDK's own: it
 *   restarts into BOOTSEL at once (pico_usb_reset), so the range starts
 *   above it.
 * - Module to host: the UART's receive interrupt (the build's highest
 *   priority) moves every byte from the 32-byte FIFO into a 16 KiB buffer
 *   (one producer, the interrupt; one consumer, nina_passthrough_to_host()
 *   in the binding's context).  The interrupt comes at 4 bytes in the FIFO
 *   (or after 32 bit times without more), so 28 bytes of margin remain:
 *   300 us at 921600 bps.  The buffer holds 178 ms at 921600 bps for the
 *   times the host does not read; bytes are lost only when it stops
 *   reading for that long, and they are counted.  A break (the line held
 *   low, as at the module's reset) is no data and is only counted; an
 *   overrun and a framing error are counted and the byte is kept.  While
 *   USB is not configured the bytes are dropped and counted apart.
 * - Host to module: the binding reads the host's bytes only when the last
 *   ones are in the UART's transmit FIFO (nina_passthrough_to_module());
 *   until then the USB stack takes no more packets and the host waits
 *   (NAK), so nothing is lost at any rate.
 * - The LED blinks at 5 Hz.
 * - The way out: closing the port at NINA_PT_EXIT_BAUD, or the RESET
 *   button.  The module is reset into its own firmware as nina_spi.c's
 *   init_nina() does it (GPIO0 high, RESETN low for 10 ms, 750 ms, GPIO0
 *   released), then the board restarts into the normal firmware.
 * - The log gets a line when the mode starts, the binding's line each time
 *   the port closes (nina_passthrough_log_counts()) and one at the way out,
 *   with the counts.
 *
 * Contexts: the binding calls the lines, the rate and the tick from
 * contexts that do not preempt one another (or masks the one that does:
 * NuttX's tick runs in a critical section against cdcacm's callback in the
 * USB interrupt); the buffer has one producer and one consumer (its rules
 * are below); the counters of the UART's flags are written by the
 * interrupt only; host to module has one caller at a time.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nina_passthrough.h"
#include "nina_passthrough_bus.h"
#include "nina_passthrough_os.h"

/* A build may place the UART's interrupt in a section of its own (the
 * FreeRTOS build: RAM, as the Pico SDK's __not_in_flash_func()).
 */
#ifdef NINA_PASSTHROUGH_ISR_SECTION
#define ISR_FUNC __attribute__((section(NINA_PASSTHROUGH_ISR_SECTION)))
#else
#define ISR_FUNC
#endif

#define MARK_0 0x4e494e41u /* "NINA" */
#define MARK_1 0x50415353u /* "PASS" */

/* After the module leaves reset with GPIO0 high (its own firmware starts),
 * GPIO0 is driven high this long and then left to the module, as
 * init_nina() does: the module's firmware drives GPIO0 itself (its IRQ
 * line).
 */
#define GPIO0_HOLD_MS 750u

#define BAUD_MIN  1200u
/* clk_peri / 16 at 125 MHz: the PL011's fastest. */
#define BAUD_MAX  7812500u
#define BAUD_INIT 115200u

/* init_nina(): the reset pulse. */
#define RESET_MS         10u
/* Time for the USB stack to send the last bytes, or the log to store the
 * last lines, before a restart.
 */
#define RESTART_DELAY_MS 100u
#define LED_HALF_MS      100u

/* The request ------------------------------------------------------------*/

static void mark(uint32_t s[NINA_PT_MARK_WORDS])
{
	s[0] = MARK_0;
	s[1] = MARK_1;
	s[2] = ~MARK_0;
	s[3] = ~MARK_1;
}

static bool mark_valid(const uint32_t s[NINA_PT_MARK_WORDS])
{
	return s[0] == MARK_0 && s[1] == MARK_1 && s[2] == ~MARK_0 && s[3] == ~MARK_1;
}

/* The lines --------------------------------------------------------------*/

struct pins {
	bool reset_released; /* RESETN high; low holds the module in reset */
	enum nina_pt_gpio0 gpio0;
	bool uart_tx; /* GPIO8 is the UART's TX; else an input */
};

struct lines {
	bool dtr;
	bool rts;
	bool own_firmware;  /* left reset with GPIO0 high: the module's firmware runs */
	uint32_t run_since; /* ms: when it left reset */
};

/* The port closed (DTR and RTS clear): the module held in reset. */
static void lines_init(struct lines *l)
{
	memset(l, 0, sizeof(*l));
}

/* The host set DTR and RTS at NOW_MS. */
static void lines_set(struct lines *l, bool dtr, bool rts, uint32_t now_ms)
{
	if (rts && !l->rts) {
		l->own_firmware = !dtr;
		l->run_since = now_ms;
	}
	l->dtr = dtr;
	l->rts = rts;
}

/* The levels at NOW_MS.
 *
 * - In reset (RTS clear): GPIO0 as DTR says (the strap for the next
 *   release), GPIO8 driven by the UART (idle high).
 * - Out of reset in the ROM's download mode (released with DTR set): as
 *   the sketch, GPIO0 follows DTR (the ROM and esptool's stub do not use
 *   it) and the UART is connected both ways.
 * - Out of reset with the module's own firmware (released with DTR clear):
 *   GPIO0 high for GPIO0_HOLD_MS, then released, whatever DTR does (its
 *   firmware drives the line); GPIO8 is an input, since the firmware
 *   drives that net as its SPI's MISO.  The sketch keeps driving both,
 *   which would fight the module's outputs.
 */
static struct pins lines_pins(const struct lines *l, uint32_t now_ms)
{
	struct pins p;

	p.reset_released = l->rts;
	p.uart_tx = true;
	p.gpio0 = l->dtr ? NINA_PT_GPIO0_LOW : NINA_PT_GPIO0_HIGH;
	if (l->rts && l->own_firmware) {
		p.uart_tx = false;
		p.gpio0 = (uint32_t)(now_ms - l->run_since) < GPIO0_HOLD_MS
				  ? NINA_PT_GPIO0_HIGH
				  : NINA_PT_GPIO0_RELEASED;
	}
	return p;
}

/* The rate and the way out ---------------------------------------------*/

/* The UART's new rate for the host's REQUESTED one: CURRENT for the exit
 * rate and for those out of range.
 */
static uint32_t uart_baud(uint32_t requested, uint32_t current)
{
	if (requested == NINA_PT_EXIT_BAUD || requested < BAUD_MIN || requested > BAUD_MAX) {
		return current;
	}
	return requested;
}

/* Whether the change of DTR from OLD_DTR to NEW_DTR at the host's RATE
 * ends the mode: the port closed at the exit rate.
 */
static bool exit_decided(bool old_dtr, bool new_dtr, uint32_t rate)
{
	return old_dtr && !new_dtr && rate == NINA_PT_EXIT_BAUD;
}

/* The buffer -----------------------------------------------------------*/

#ifndef NINA_PT_BARRIER
#define NINA_PT_BARRIER() __asm__ volatile("" ::: "memory")
#endif

/* SIZE is a power of two.  head and tail count bytes since the start (they
 * wrap at 2^32); head is written by the producer only, tail and peak by
 * the consumer only, dropped by the producer only.  On the one core of
 * the RP2040 the barriers keep the compiler from moving the stores of
 * data past the index that publishes them.
 */
struct ring {
	uint8_t *buf;
	uint32_t size;
	volatile uint32_t head;
	volatile uint32_t tail;
	volatile uint32_t dropped; /* bytes that found the buffer full */
	uint32_t peak;             /* most bytes seen waiting (consumer) */
};

static void ring_init(struct ring *r, uint8_t *buf, uint32_t size)
{
	r->buf = buf;
	r->size = size;
	r->head = 0;
	r->tail = 0;
	r->dropped = 0;
	r->peak = 0;
}

/* Producer: stores C, or counts it as dropped when the buffer is full. */
static inline __attribute__((always_inline)) void ring_put(struct ring *r, uint8_t c)
{
	uint32_t head = r->head;

	if (head - r->tail >= r->size) {
		r->dropped++;
		return;
	}
	r->buf[head & (r->size - 1)] = c;
	NINA_PT_BARRIER();
	r->head = head + 1;
}

/* Consumer: the bytes waiting from *DATA on without a wrap (0 when none);
 * then ring_consume() with the number taken.
 */
static uint32_t ring_peek(struct ring *r, const uint8_t **data)
{
	uint32_t tail = r->tail;
	uint32_t count = r->head - tail;
	uint32_t at = tail & (r->size - 1);

	NINA_PT_BARRIER();
	if (count > r->peak) {
		r->peak = count;
	}
	*data = r->buf + at;
	return count < r->size - at ? count : r->size - at;
}

static void ring_consume(struct ring *r, uint32_t n)
{
	NINA_PT_BARRIER();
	r->tail = r->tail + n;
}

/* The state --------------------------------------------------------------*/

static bool requested;
static volatile bool exit_requested;
static struct lines lines;
static struct pins applied;
static bool applied_valid;
static uint32_t baud;

static struct ring ring;
static volatile uint32_t uart_overruns;
static volatile uint32_t uart_breaks;
static volatile uint32_t uart_framing;
static uint64_t to_nina;
static uint64_t from_nina;
static uint64_t discarded; /* from the module while USB was not configured */

static bool led;
static uint32_t led_at;

/* The levels of P, in the order the module needs: GPIO8 and GPIO0 settle
 * before RESETN changes, so the strap is in place when it is released.
 */
static void apply_pins(struct pins p)
{
	if (!applied_valid || p.uart_tx != applied.uart_tx) {
		nina_pt_bus_uart_tx_connect(p.uart_tx);
	}
	if (!applied_valid || p.gpio0 != applied.gpio0) {
		nina_pt_bus_gpio0(p.gpio0);
	}
	if (!applied_valid || p.reset_released != applied.reset_released) {
		nina_pt_bus_resetn(p.reset_released);
	}
	applied = p;
	applied_valid = true;
}

/* The request ------------------------------------------------------------*/

bool nina_passthrough_check(void)
{
	uint32_t s[NINA_PT_MARK_WORDS];

	nina_pt_bus_mark_get(s);
	if (!mark_valid(s)) {
		return false;
	}
	memset(s, 0, sizeof(s));
	nina_pt_bus_mark_set(s);
	requested = true;
	return true;
}

bool nina_passthrough_active(void)
{
	return requested;
}

void nina_passthrough_request(void)
{
	uint32_t s[NINA_PT_MARK_WORDS];

	mark(s);
	nina_pt_bus_mark_set(s);
	nina_pt_bus_restart();
}

/* The start --------------------------------------------------------------*/

void nina_passthrough_hold(void)
{
	lines_init(&lines);
	baud = BAUD_INIT;
	nina_pt_bus_lines_init();
}

void nina_passthrough_begin(uint8_t *buf)
{
	ring_init(&ring, buf, NINA_PASSTHROUGH_RING_SIZE);
	nina_pt_bus_uart_init(baud);
	applied_valid = false;
	apply_pins(lines_pins(&lines, nina_pt_os_now_ms()));
	nina_pt_bus_uart_irq_on();
	nina_pt_bus_led_init();
	led = false;
	led_at = nina_pt_os_now_ms();
	nina_pt_os_log_start(baud);
}

/* The data ---------------------------------------------------------------*/

ISR_FUNC void nina_passthrough_uart_isr(void)
{
	bool received = false;
	uint32_t word;

	while (nina_pt_bus_uart_rx(&word)) {
		if (word & NINA_PT_UART_OVERRUN) {
			uart_overruns++;
		}
		if (word & NINA_PT_UART_BREAK) {
			uart_breaks++;
			continue;
		}
		if (word & NINA_PT_UART_FRAMING) {
			uart_framing++;
		}
		ring_put(&ring, (uint8_t)word);
		received = true;
	}
	nina_pt_os_uart_irq(received);
}

uint32_t nina_passthrough_to_module(const uint8_t *data, uint32_t len)
{
	uint32_t n = 0;

	while (n < len && nina_pt_bus_uart_tx(data[n])) {
		n++;
	}
	to_nina += n;
	return n;
}

bool nina_passthrough_to_host(void)
{
	const uint8_t *data;
	uint32_t n;
	bool wrote = false;

	while ((n = ring_peek(&ring, &data)) > 0) {
		int done = nina_pt_os_write(data, n);

		if (done < 0) {
			ring_consume(&ring, n);
			discarded += n;
			continue;
		}
		if (done == 0) {
			break;
		}
		ring_consume(&ring, (uint32_t)done);
		from_nina += (uint32_t)done;
		wrote = true;
	}
	return wrote;
}

void nina_passthrough_discard(void)
{
	const uint8_t *data;
	uint32_t n;

	while ((n = ring_peek(&ring, &data)) > 0) {
		ring_consume(&ring, n);
		discarded += n;
	}
}

bool nina_passthrough_waiting(void)
{
	return ring.head != ring.tail;
}

/* The host's lines and rate ------------------------------------------------*/

bool nina_passthrough_host_lines(bool dtr, bool rts, uint32_t rate)
{
	bool closed;
	uint32_t now;

	if (dtr == lines.dtr && rts == lines.rts) {
		return false;
	}
	if (exit_decided(lines.dtr, dtr, rate)) {
		exit_requested = true;
	}
	closed = lines.dtr && !dtr;
	now = nina_pt_os_now_ms();
	lines_set(&lines, dtr, rts, now);
	apply_pins(lines_pins(&lines, now));
	return closed;
}

void nina_passthrough_host_rate(uint32_t rate)
{
	uint32_t next = uart_baud(rate, baud);

	if (next != baud) {
		baud = next;
		nina_pt_bus_uart_set_baud(baud);
	}
}

bool nina_passthrough_exit_requested(void)
{
	return exit_requested;
}

uint32_t nina_passthrough_tick(void)
{
	uint32_t now = nina_pt_os_now_ms();
	uint32_t ms;

	apply_pins(lines_pins(&lines, now));
	if ((int32_t)(now - led_at) >= 0) {
		led = !led;
		nina_pt_bus_led(led);
		led_at += LED_HALF_MS;
		if ((int32_t)(now - led_at) >= 0) {
			led_at = now + LED_HALF_MS;
		}
	}

	/* Until the LED's next change, or until GPIO0 is to be released. */
	ms = (int32_t)(led_at - now) > 0 ? led_at - now : 1;
	if (lines.rts && lines.own_firmware && applied.gpio0 != NINA_PT_GPIO0_RELEASED) {
		uint32_t held = now - lines.run_since;
		uint32_t left = held < GPIO0_HOLD_MS ? GPIO0_HOLD_MS - held : 1;

		if (left < ms) {
			ms = left;
		}
	}
	return ms;
}

/* The log and the way out -------------------------------------------------*/

void nina_passthrough_log_counts(const char *why)
{
	struct nina_pt_counts c = {
		.to_nina = to_nina,
		.from_nina = from_nina,
		.discarded = discarded,
		.lost = ring.dropped,
		.peak = ring.peak,
		.size = ring.size,
		.overruns = uart_overruns,
		.breaks = uart_breaks,
		.framing = uart_framing,
		.baud = baud,
	};

	nina_pt_os_log_counts(why, &c);
}

void nina_passthrough_stop(void)
{
	nina_pt_bus_uart_irq_off();
}

void nina_passthrough_leave(void)
{
	nina_passthrough_log_counts("Leaving the NINA passthrough");
	nina_pt_bus_uart_tx_connect(false);
	nina_pt_bus_gpio0(NINA_PT_GPIO0_HIGH);
	nina_pt_bus_resetn(false);
	nina_pt_os_sleep_ms(RESET_MS);
	nina_pt_bus_resetn(true);
	nina_pt_os_sleep_ms(GPIO0_HOLD_MS);
	nina_pt_bus_gpio0(NINA_PT_GPIO0_RELEASED);
	nina_pt_bus_led(false);
	nina_pt_os_sleep_ms(RESTART_DELAY_MS);
	nina_pt_bus_restart();
}
