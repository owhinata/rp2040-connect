/* SPDX-License-Identifier: MIT */
/* The decisions of the NINA passthrough mode (nina_passthrough.c), without
 * the hardware, so that the host test (freertos/tests/passthrough) runs
 * them as the board does.
 *
 * - The request: four words in the watchdog's scratch registers 0..3 that
 *   "nina passthrough confirm" leaves for the next start.  Registers 0..3
 *   are those of the fatal error record (common/log/fault_record.h), which
 *   starts with "FA" in the high half of register 0; the request starts
 *   with "NI", so neither is taken for the other, and a fatal error during
 *   the restart replaces the request (the next start is a normal one).  The
 *   Pico SDK's watchdog_reboot() uses registers 4..7 only.
 * - The lines (nina_pt_lines_*()): what the USB host's DTR and RTS
 *   (SET_CONTROL_LINE_STATE) make of the module's GPIO0 (RP2040 GPIO2) and
 *   RESETN (RP2040 GPIO3), and whether the RP2040 drives the module's UART
 *   receive line (RP2040 GPIO8, UART1 TX).
 * - The rate (nina_pt_uart_baud()) and the way out (nina_pt_exit()) from
 *   the host's line coding (SET_LINE_CODING).
 * - The buffer from the UART's receive interrupt to the USB side
 *   (nina_pt_ring_*()): one producer (the interrupt), one consumer (the
 *   main loop).
 * - The "nina" command's subcommands: the shared set
 *   (common/drivers/nina/nina_shell.c) with this build's "passthrough"
 *   merged in by name (nina_pt_subcmd_get()).
 */

#ifndef NINA_PASSTHROUGH_LOGIC_H
#define NINA_PASSTHROUGH_LOGIC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/shell/shell.h>

/* The request ----------------------------------------------------------*/

#define NINA_PT_MARK_WORDS 4
#define NINA_PT_MARK_0     0x4e494e41u /* "NINA" */
#define NINA_PT_MARK_1     0x50415353u /* "PASS" */

static inline void nina_pt_mark(uint32_t s[NINA_PT_MARK_WORDS])
{
	s[0] = NINA_PT_MARK_0;
	s[1] = NINA_PT_MARK_1;
	s[2] = ~NINA_PT_MARK_0;
	s[3] = ~NINA_PT_MARK_1;
}

static inline bool nina_pt_mark_valid(const uint32_t s[NINA_PT_MARK_WORDS])
{
	return s[0] == NINA_PT_MARK_0 && s[1] == NINA_PT_MARK_1 && s[2] == ~NINA_PT_MARK_0 &&
	       s[3] == ~NINA_PT_MARK_1;
}

/* The lines ------------------------------------------------------------*/

/* After the module leaves reset with GPIO0 high (its own firmware starts),
 * GPIO0 is driven high this long and then left to the module (an input
 * with the pull-down), as nina_spi.c's init_nina() does: the module's
 * firmware drives GPIO0 itself (its IRQ line), so the RP2040 must not keep
 * driving it.
 */
#define NINA_PT_GPIO0_HOLD_MS 750u

enum nina_pt_gpio0 {
	NINA_PT_GPIO0_LOW,      /* driven low: the ROM's download mode at reset */
	NINA_PT_GPIO0_HIGH,     /* driven high: a normal start at reset */
	NINA_PT_GPIO0_RELEASED, /* input with the pull-down (init_nina()'s end) */
};

struct nina_pt_pins {
	bool reset_released; /* RESETN (GPIO3) high; low holds the module in reset */
	enum nina_pt_gpio0 gpio0;
	bool uart_tx; /* GPIO8 is UART1's TX; else an input */
};

struct nina_pt_lines {
	bool dtr;
	bool rts;
	bool own_firmware;  /* left reset with GPIO0 high: the module's firmware runs */
	uint32_t run_since; /* ms: when it left reset */
};

/* The port closed (DTR and RTS clear): the module held in reset. */
static inline void nina_pt_lines_init(struct nina_pt_lines *l)
{
	memset(l, 0, sizeof(*l));
}

/* The host set DTR and RTS at NOW_MS.  The mapping is the Arduino sketch's
 * (WiFiNINA's SerialNINAPassthrough for the mbed core): RESETN follows RTS
 * (RTS set releases the reset), GPIO0 is low while DTR is set.  So opening
 * the port (Linux sets both at once) starts the ROM's download mode, and
 * closing it holds the module in reset; esptool keeps RTS set ("--before
 * default-reset --after no-reset", custom_reset_sequence R0|D1|W0.2|R1|W0.6
 * in ESPTOOL_CFGFILE).  The caller applies GPIO0 before RESETN, so a
 * request that changes both releases the reset with GPIO0 already set.
 */
static inline void nina_pt_lines_set(struct nina_pt_lines *l, bool dtr, bool rts, uint32_t now_ms)
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
 *   GPIO0 high for NINA_PT_GPIO0_HOLD_MS, then released, whatever DTR does
 *   (its firmware drives the line); GPIO8 is an input, since the firmware
 *   drives that net as its SPI's MISO.  The sketch keeps driving both,
 *   which would fight the module's outputs.
 */
static inline struct nina_pt_pins nina_pt_lines_pins(const struct nina_pt_lines *l, uint32_t now_ms)
{
	struct nina_pt_pins p;

	p.reset_released = l->rts;
	p.uart_tx = true;
	p.gpio0 = l->dtr ? NINA_PT_GPIO0_LOW : NINA_PT_GPIO0_HIGH;
	if (l->rts && l->own_firmware) {
		p.uart_tx = false;
		p.gpio0 = (uint32_t)(now_ms - l->run_since) < NINA_PT_GPIO0_HOLD_MS
				  ? NINA_PT_GPIO0_HIGH
				  : NINA_PT_GPIO0_RELEASED;
	}
	return p;
}

/* The rate and the way out ---------------------------------------------*/

/* Closing the port while its rate is this one ends the mode (the board
 * restarts normally), as the "1200 bps touch" of Arduino boards does: for
 * example "stty -F /dev/ttyACM0 300".  esptool never uses it.  A port
 * opened at this rate (Linux keeps a tty's last rate) and set to another
 * one before it is closed does not end the mode.  1200 bps is the Pico
 * SDK's own: it restarts into BOOTSEL at once (pico_usb_reset).
 */
#define NINA_PT_EXIT_BAUD 300u
#define NINA_PT_BAUD_MIN  1200u
/* clk_peri / 16 at 125 MHz: the PL011's fastest. */
#define NINA_PT_BAUD_MAX  7812500u
#define NINA_PT_BAUD_INIT 115200u

/* The UART's new rate for the host's REQUESTED one: CURRENT for the exit
 * rate and for those out of range.
 */
static inline uint32_t nina_pt_uart_baud(uint32_t requested, uint32_t current)
{
	if (requested == NINA_PT_EXIT_BAUD || requested < NINA_PT_BAUD_MIN ||
	    requested > NINA_PT_BAUD_MAX) {
		return current;
	}
	return requested;
}

/* Whether the change of DTR from OLD_DTR to NEW_DTR at the host's RATE
 * ends the mode: the port closed at the exit rate.
 */
static inline bool nina_pt_exit(bool old_dtr, bool new_dtr, uint32_t rate)
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
struct nina_pt_ring {
	uint8_t *buf;
	uint32_t size;
	volatile uint32_t head;
	volatile uint32_t tail;
	volatile uint32_t dropped; /* bytes that found the buffer full */
	uint32_t peak;             /* most bytes seen waiting (consumer) */
};

static inline void nina_pt_ring_init(struct nina_pt_ring *r, uint8_t *buf, uint32_t size)
{
	r->buf = buf;
	r->size = size;
	r->head = 0;
	r->tail = 0;
	r->dropped = 0;
	r->peak = 0;
}

/* Producer: stores C, or counts it as dropped when the buffer is full. */
static inline __attribute__((always_inline)) void nina_pt_ring_put(struct nina_pt_ring *r,
								   uint8_t c)
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
 * then nina_pt_ring_consume() with the number taken.
 */
static inline uint32_t nina_pt_ring_peek(struct nina_pt_ring *r, const uint8_t **data)
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

static inline void nina_pt_ring_consume(struct nina_pt_ring *r, uint32_t n)
{
	NINA_PT_BARRIER();
	r->tail = r->tail + n;
}

/* The "nina" subcommands -----------------------------------------------*/

/* The IDX-th of the static set SHARED (ending with an entry without a
 * syntax) with EXTRA put in by name, in ENTRY (no syntax past the end),
 * for a dynamic set (Zephyr sorts a command's subcommands by name).
 */
static inline void nina_pt_subcmd_get(const struct shell_static_entry *shared,
				      const struct shell_static_entry *extra, size_t idx,
				      struct shell_static_entry *entry)
{
	size_t before = 0; /* the shared entries before EXTRA */
	size_t n = 0;

	while (shared[n].syntax != NULL) {
		if (strcmp(shared[n].syntax, extra->syntax) < 0) {
			before = n + 1;
		}
		n++;
	}
	memset(entry, 0, sizeof(*entry));
	if (idx < before) {
		*entry = shared[idx];
	} else if (idx == before) {
		*entry = *extra;
	} else if (idx <= n) {
		*entry = shared[idx - 1];
	}
}

#endif /* NINA_PASSTHROUGH_LOGIC_H */
