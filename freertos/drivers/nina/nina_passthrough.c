/* SPDX-License-Identifier: MIT */
/* The NINA passthrough mode (nina_passthrough.h): esptool on the PC talks
 * through the board's USB port to the ESP32 ROM bootloader of the
 * NINA-W102, as WiFiNINA's SerialNINAPassthrough sketch (mbed core) does,
 * without replacing this firmware or touching the flash.
 *
 * The board (the mbed core's variants/NANO_RP2040_CONNECT: pins_arduino.h
 * SerialNina = Serial3 on D25/D26, NINA_GPIO0 D20, NINA_RESETN D24, and
 * variant.cpp's pin table; common/drivers/nina/nano_nina.h):
 *
 *   RP2040   UART1 in this mode                SPI link of the normal firmware
 *   GPIO8    TX  -> the module's UART0 RX      SPI1 RX (MISO)
 *   GPIO9    RX  <- the module's UART0 TX      CS, active low
 *   GPIO2    the module's GPIO0 (boot strap)   IRQ (driven only at the reset)
 *   GPIO3    the module's RESETN, active low   RESET
 *
 * GPIO8 and GPIO9 are both links at once, so the SPI link must not run:
 * this mode is a start of its own.  "nina passthrough confirm" leaves a
 * request in the watchdog's scratch registers 0..3 and restarts the board;
 * main() sees it before anything else starts and runs
 * nina_passthrough_run() instead of the normal firmware: no task, no
 * scheduler, no SPI1, no NINA request, no I2C, no key-value store.  The
 * request is cleared at once, so the RESET button, any watchdog restart and
 * a fatal error start the normal firmware again.  Nothing is written to
 * the flash; the state is in RAM and the scratch registers only.
 *
 * The mode, in one loop on the processor (no scheduler):
 * - USB: TinyUSB is started here (tusb_init()) instead of by the Pico
 *   SDK's stdio, and the loop calls tud_task() itself, so the CDC data are
 *   raw both ways: no stdio, no CR/LF translation, no echo, no line
 *   editing.  The device is the same one (the SDK's descriptors: the same
 *   /dev/serial/by-id name, the SDK's 1200 bps reset to BOOTSEL and the
 *   picotool reset interface still work).
 * - Host to module: read from TinyUSB's receive FIFO 64 bytes at a time
 *   only when the last ones are in the UART's transmit FIFO; until then
 *   TinyUSB does not take the next packet (the host waits), so nothing is
 *   lost at any rate.
 * - Module to host: UART1's receive interrupt (the highest priority: no
 *   other interrupt delays it, and nothing here masks interrupts for long)
 *   moves every byte from the 32-byte FIFO into a 16 KiB buffer (from the
 *   FreeRTOS heap, which nothing else uses in this mode); the loop moves
 *   it into TinyUSB's transmit FIFO as there is room.  The interrupt comes
 *   at 4 bytes in the FIFO (or after 32 bit times without more), so 28
 *   bytes of margin remain: 300 us at 921600 bps.  The buffer holds 178 ms
 *   at 921600 bps, for the times the host does not read.  Bytes are lost
 *   only when the host stops reading for that long; they are counted.
 * - Lines: tud_cdc_line_state_cb() maps DTR and RTS as the sketch does
 *   (nina_passthrough_logic.h: RTS set releases RESETN, DTR set pulls GPIO0
 *   low), so opening the port starts the ROM's download mode and closing it
 *   holds the module in reset.  esptool keeps RTS set with
 *   "--before default-reset --after no-reset" and a custom_reset_sequence
 *   of "R0|D1|W0.2|R1|W0.6" (ESPTOOL_CFGFILE); with "--before no-reset"
 *   esptool 5.3.0 got no answer.
 *   Unlike the sketch, when the module leaves reset into its own firmware
 *   (DTR clear), GPIO0 is released after 750 ms and GPIO8 stops driving,
 *   as the firmware drives both nets.
 * - Rate: the host's line coding (polled each turn of the loop) sets
 *   UART1's rate, 8N1 always, as the sketch does (esptool's --baud: the
 *   stub's CHANGE_BAUDRATE, then the port's rate).  The receive pin has a
 *   pull-up, so the line idles high while the module is in reset.
 * - LED (GPIO6): blinks at 5 Hz (the normal firmware: 1 Hz).
 * - Way out: closing the port at 300 bps (nina_pt_exit()), or the RESET
 *   button.  Then the module is reset into its own firmware as
 *   nina_spi.c's init_nina() does it (GPIO0 high, RESETN low for 10 ms,
 *   750 ms, GPIO0 released), and the watchdog restarts the board.
 * - The RAM log ("dmesg" after the way out) gets a line when the mode
 *   starts and one each time the port closes, with the counts.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "tusb.h"

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/shell/shell.h>

#include "app_log.h"
#include "nano_shell.h"
#include "nina_passthrough.h"
#include "nina_passthrough_logic.h"

LOG_MODULE_REGISTER(nina_passthrough, LOG_LEVEL_INF);

#define PT_UART             uart1
#define PT_UART_IRQ         UART1_IRQ
#define PT_PIN_TX           8 /* UART1 TX */
#define PT_PIN_RX           9 /* UART1 RX */
#define PT_PIN_GPIO0        2
#define PT_PIN_RESETN       3
#define PT_RING_SIZE        16384u
#define PT_LED_HALF_US      100000u
/* init_nina() (common/drivers/nina/nina_spi.c): the reset pulse and the
 * time GPIO0 stays high after it.
 */
#define PT_RESET_MS         10u
/* Time for the USB code to send the command's last message, or the last
 * bytes of the mode, before a restart (commands.c's RESTART_DELAY_MS).
 */
#define PT_RESTART_DELAY_MS 100u

static bool active;
static volatile bool exit_requested;
static struct nina_pt_lines lines;
static struct nina_pt_pins applied;
static bool applied_valid;
static uint32_t baud;

static struct nina_pt_ring ring;
static volatile uint32_t uart_overruns;
static volatile uint32_t uart_breaks;
static volatile uint32_t uart_framing;
static uint64_t to_nina;
static uint64_t from_nina;
static uint64_t discarded; /* from the module while USB was not configured */

static uint8_t out_buf[64]; /* from the host, on its way into the UART */
static uint32_t out_len;
static uint32_t out_pos;

static uint32_t now_ms(void)
{
	return (uint32_t)(time_us_64() / 1000u);
}

/* The request -----------------------------------------------------------*/

bool nina_passthrough_requested(void)
{
	uint32_t s[NINA_PT_MARK_WORDS];
	bool requested;

	for (int i = 0; i < NINA_PT_MARK_WORDS; i++) {
		s[i] = watchdog_hw->scratch[i];
	}
	requested = nina_pt_mark_valid(s);
	if (requested) {
		for (int i = 0; i < NINA_PT_MARK_WORDS; i++) {
			watchdog_hw->scratch[i] = 0;
		}
	}
	return requested;
}

/* The lines --------------------------------------------------------------*/

static void set_gpio0(enum nina_pt_gpio0 state)
{
	switch (state) {
	case NINA_PT_GPIO0_LOW:
		gpio_put(PT_PIN_GPIO0, 0);
		gpio_set_dir(PT_PIN_GPIO0, GPIO_OUT);
		break;
	case NINA_PT_GPIO0_HIGH:
		gpio_put(PT_PIN_GPIO0, 1);
		gpio_set_dir(PT_PIN_GPIO0, GPIO_OUT);
		break;
	case NINA_PT_GPIO0_RELEASED:
		gpio_put(PT_PIN_GPIO0, 0);
		gpio_set_dir(PT_PIN_GPIO0, GPIO_IN);
		break;
	}
}

/* GPIO8 to the UART, or a plain input (SIO with the direction in). */
static void set_uart_tx(bool connected)
{
	gpio_set_function(PT_PIN_TX, connected ? GPIO_FUNC_UART : GPIO_FUNC_SIO);
}

/* The levels of P, in the order the module needs: GPIO8 and GPIO0 settle
 * before RESETN changes, so the strap is in place when it is released.
 */
static void apply_pins(struct nina_pt_pins p)
{
	if (!applied_valid || p.uart_tx != applied.uart_tx) {
		set_uart_tx(p.uart_tx);
	}
	if (!applied_valid || p.gpio0 != applied.gpio0) {
		set_gpio0(p.gpio0);
	}
	if (!applied_valid || p.reset_released != applied.reset_released) {
		gpio_put(PT_PIN_RESETN, p.reset_released);
	}
	applied = p;
	applied_valid = true;
}

static void log_counts(const char *why)
{
	LOG_INF("%s: %llu bytes to the NINA, %llu from it, %llu discarded without USB, "
		"%lu lost (buffer full, peak %lu of %lu), UART: %lu overruns, %lu breaks, "
		"%lu framing errors, %lu bps",
		why, to_nina, from_nina, discarded, (unsigned long)ring.dropped,
		(unsigned long)ring.peak, (unsigned long)ring.size, (unsigned long)uart_overruns,
		(unsigned long)uart_breaks, (unsigned long)uart_framing, (unsigned long)baud);
}

/* TinyUSB's callback (a weak symbol in cdc_device.h), from tud_task(): in
 * this mode from the loop below; in the normal firmware from the Pico
 * SDK's USB work, where it does nothing.
 */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	cdc_line_coding_t coding;
	uint32_t now;

	if (!active || itf != 0) {
		return;
	}
	tud_cdc_get_line_coding(&coding);
	if (nina_pt_exit(lines.dtr, dtr, coding.bit_rate)) {
		exit_requested = true;
	}
	if (lines.dtr && !dtr) {
		log_counts("Port closed");
	}
	now = now_ms();
	nina_pt_lines_set(&lines, dtr, rts, now);
	apply_pins(nina_pt_lines_pins(&lines, now));
}

/* The data ---------------------------------------------------------------*/

/* UART1's receive interrupt: empties the FIFO into the buffer.  A break
 * (the line held low, as while the module is in reset without the
 * pull-up winning, or at its reset) is no data and is only counted.
 */
static void __not_in_flash_func(uart_irq)(void)
{
	uart_hw_t *hw = uart_get_hw(PT_UART);

	while (!(hw->fr & UART_UARTFR_RXFE_BITS)) {
		uint32_t dr = hw->dr;

		if (dr & UART_UARTDR_OE_BITS) {
			uart_overruns++;
		}
		if (dr & UART_UARTDR_BE_BITS) {
			uart_breaks++;
			continue;
		}
		if (dr & UART_UARTDR_FE_BITS) {
			uart_framing++;
		}
		nina_pt_ring_put(&ring, (uint8_t)dr);
	}
}

static void host_to_nina(void)
{
	uart_hw_t *hw = uart_get_hw(PT_UART);

	if (out_pos == out_len) {
		out_pos = 0;
		out_len = tud_cdc_available() ? tud_cdc_read(out_buf, sizeof(out_buf)) : 0;
	}
	while (out_pos < out_len && !(hw->fr & UART_UARTFR_TXFF_BITS)) {
		hw->dr = out_buf[out_pos++];
		to_nina++;
	}
}

static void nina_to_host(void)
{
	const uint8_t *data;
	uint32_t n;
	bool wrote = false;

	while ((n = nina_pt_ring_peek(&ring, &data)) > 0) {
		uint32_t room;

		if (!tud_mounted()) {
			nina_pt_ring_consume(&ring, n);
			discarded += n;
			continue;
		}
		room = tud_cdc_write_available();
		if (room == 0) {
			break;
		}
		n = tud_cdc_write(data, n < room ? n : room);
		if (n == 0) {
			break;
		}
		nina_pt_ring_consume(&ring, n);
		from_nina += n;
		wrote = true;
	}
	if (wrote) {
		tud_cdc_write_flush();
	}
}

static void follow_rate(void)
{
	cdc_line_coding_t coding;
	uint32_t next;

	tud_cdc_get_line_coding(&coding);
	next = nina_pt_uart_baud(coding.bit_rate, baud);
	if (next != baud) {
		baud = next;
		uart_set_baudrate(PT_UART, baud);
	}
}

/* One turn of the loop. */
static void service(void)
{
	tud_task();
	apply_pins(nina_pt_lines_pins(&lines, now_ms()));
	follow_rate();
	host_to_nina();
	nina_to_host();
}

/* Waits MS milliseconds, serving USB meanwhile. */
static void wait_ms(uint32_t ms)
{
	uint64_t until = time_us_64() + (uint64_t)ms * 1000u;

	while (time_us_64() < until) {
		tud_task();
	}
}

/* The way out: the module into its own firmware, as init_nina() does it,
 * then the watchdog's restart into the normal firmware.
 */
static void __attribute__((noreturn)) leave(void)
{
	irq_set_enabled(PT_UART_IRQ, false);
	log_counts("Leaving the NINA passthrough");
	set_uart_tx(false);
	set_gpio0(NINA_PT_GPIO0_HIGH);
	gpio_put(PT_PIN_RESETN, 0);
	wait_ms(PT_RESET_MS);
	gpio_put(PT_PIN_RESETN, 1);
	wait_ms(NINA_PT_GPIO0_HOLD_MS);
	set_gpio0(NINA_PT_GPIO0_RELEASED);
	gpio_put(PICO_DEFAULT_LED_PIN, 0);
	watchdog_reboot(0, 0, PT_RESTART_DELAY_MS);
	for (;;) {
		tud_task();
	}
}

void nina_passthrough_run(void)
{
	uint8_t *buf = pvPortMalloc(PT_RING_SIZE);
	uint32_t led_at;

	if (buf == NULL) {
		panic("nina_passthrough: no buffer");
	}
	nina_pt_ring_init(&ring, buf, PT_RING_SIZE);
	nina_pt_lines_init(&lines);
	baud = NINA_PT_BAUD_INIT;

	/* The module's lines as the port closed (DTR, RTS clear) wants them:
	 * held in reset, GPIO0 high.  RESETN without pulls (Zephyr's
	 * reset-gpios), GPIO0 with the pull-down (irq-gpios), as nano_nina.c.
	 */
	gpio_init(PT_PIN_RESETN);
	gpio_disable_pulls(PT_PIN_RESETN);
	gpio_put(PT_PIN_RESETN, 0);
	gpio_set_dir(PT_PIN_RESETN, GPIO_OUT);
	gpio_init(PT_PIN_GPIO0);
	gpio_pull_down(PT_PIN_GPIO0);

	/* UART1, 8N1, FIFOs on, no flow control; GPIO8 as SIO input until
	 * apply_pins() connects it; GPIO9 with the pull-up.
	 */
	uart_init(PT_UART, baud);
	gpio_init(PT_PIN_TX);
	gpio_disable_pulls(PT_PIN_TX);
	gpio_set_function(PT_PIN_RX, GPIO_FUNC_UART);
	gpio_pull_up(PT_PIN_RX);
	apply_pins(nina_pt_lines_pins(&lines, now_ms()));

	irq_set_exclusive_handler(PT_UART_IRQ, uart_irq);
	irq_set_priority(PT_UART_IRQ, PICO_HIGHEST_IRQ_PRIORITY);
	uart_set_irqs_enabled(PT_UART, true, false);
	irq_set_enabled(PT_UART_IRQ, true);

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	LOG_INF("NINA passthrough: USB <-> UART1 (GPIO8 TX, GPIO9 RX) at %lu bps, "
		"DTR -> GPIO0 (GPIO2), RTS -> RESETN (GPIO3); close the port at %u bps or "
		"press RESET to leave",
		(unsigned long)baud, NINA_PT_EXIT_BAUD);

	active = true;
	tusb_init();

	led_at = time_us_32();
	for (;;) {
		service();
		if (exit_requested) {
			leave();
		}
		if ((int32_t)(time_us_32() - led_at) >= 0) {
			gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
			led_at += PT_LED_HALF_US;
		}
	}
}

/* The command ------------------------------------------------------------*/

static int cmd_nina_passthrough(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t s[NINA_PT_MARK_WORDS];

	(void)argc;
	if (strcmp(argv[1], "confirm") != 0) {
		shell_error(sh, "Type \"nina passthrough confirm\" to restart into the NINA "
				"passthrough");
		return -EINVAL;
	}
	shell_print(sh, "Restarting into the NINA passthrough: the USB port relays to the "
			"NINA's UART (for esptool).");
	shell_print(sh, "To leave it: close the port at %u bps, or press RESET.",
		    NINA_PT_EXIT_BAUD);
	vTaskDelay(pdMS_TO_TICKS(PT_RESTART_DELAY_MS));
	nina_pt_mark(s);
	for (int i = 0; i < NINA_PT_MARK_WORDS; i++) {
		watchdog_hw->scratch[i] = s[i];
	}
	watchdog_reboot(0, 0, PT_RESTART_DELAY_MS);
	vTaskDelay(portMAX_DELAY);
	return 0;
}

/* "nina": the shared command (common/drivers/nina/nina_shell.c, which
 * the Zephyr build uses as well) with this build's "passthrough".  The
 * shared set is static and defined there; this root's set is dynamic and
 * merges the two by name.  commands.c lists this root instead of the
 * shared one.  The help is the shared root's text.
 */
NANO_SHELL_CMD_DECLARE(nina);

static const struct shell_static_entry passthrough_entry =
	SHELL_CMD_ARG(passthrough, NULL,
		      "Restart into the NINA passthrough for esptool: passthrough confirm\n"
		      "(USB <-> the NINA's UART; leave it by closing the port at 300 bps, "
		      "or with RESET)",
		      cmd_nina_passthrough, 2, 0);

static void nina_subcmd_get(size_t idx, struct shell_static_entry *entry)
{
	static const struct shell_static_entry none[] = {SHELL_SUBCMD_SET_END};
	const struct shell_cmd_entry *shared = z_shell_root_nina.subcmd;

	nina_pt_subcmd_get(shared != NULL && !shared->is_dynamic ? shared->entry : none,
			   &passthrough_entry, idx, entry);
}

SHELL_DYNAMIC_CMD_CREATE(nina_subcmds, nina_subcmd_get);

NANO_SHELL_CMD_DECLARE(nina_board);
const struct shell_static_entry z_shell_root_nina_board =
	SHELL_CMD(nina, &nina_subcmds, "NINA Wi-Fi module", NULL);
