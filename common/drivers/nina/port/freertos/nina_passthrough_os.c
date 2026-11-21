/* SPDX-License-Identifier: MIT */
/* The NINA passthrough mode on FreeRTOS (nina_passthrough_os.h of the
 * core, next to this binding): TinyUSB's CDC port as the host's side of
 * the core (common/drivers/nina/nina_passthrough.c, which explains the
 * mode), written on TinyUSB and the Pico SDK.  The module's side is the
 * build's (common/drivers/nina/port/freertos/nina_passthrough_bus_rp2040.c).
 *
 * main() asks nina_passthrough_check() first thing (before fault_boot(),
 * which clears the same scratch registers); with the request it starts
 * nothing of the normal firmware (no task, no scheduler, no SPI1, no NINA
 * request, no I2C, no key-value store) and runs nina_passthrough_run(),
 * one loop on the processor:
 *
 * - USB: TinyUSB is started here (tusb_init()) instead of by the Pico
 *   SDK's stdio, and the loop calls tud_task() itself, so the CDC data are
 *   raw both ways: no stdio, no CR/LF translation, no echo, no line
 *   editing.  The device is the same one (the SDK's descriptors: the same
 *   /dev/serial/by-id name, the SDK's 1200 bps reset to BOOTSEL and the
 *   picotool reset interface still work).
 * - Host to module: read from TinyUSB's receive FIFO 64 bytes at a time
 *   only when the last ones are in the UART's transmit FIFO; until then
 *   TinyUSB does not take the next packet (the host waits).
 * - Module to host: the core's buffer (16 KiB from the FreeRTOS heap,
 *   which nothing else uses in this mode) into TinyUSB's transmit FIFO as
 *   there is room, flushed once a turn; while USB is not mounted the bytes
 *   are discarded.
 * - Lines: tud_cdc_line_state_cb() (from tud_task()) to
 *   nina_passthrough_host_lines(), with the line coding's rate at the time.
 * - Rate: the host's line coding, polled each turn of the loop.
 * - The waits of the way out serve USB (tud_task()) meanwhile.
 *
 * Nothing runs in UART1's interrupt but the core's receive path
 * (nina_passthrough_os_uart_irq() is empty), so the loop needs no lock.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "tusb.h"

#include "FreeRTOS.h"

#include "app_log.h"
#include "nina_passthrough.h"
#include "nina_passthrough_os.h"

LOG_MODULE_REGISTER(nina_passthrough, LOG_LEVEL_INF);

#define PT_PACKET 64u /* read from the host at once */

static uint8_t out_buf[PT_PACKET]; /* from the host, on its way into the UART */
static uint32_t out_len;
static uint32_t out_pos;

/* nina_passthrough_os.h --------------------------------------------------*/

uint32_t nina_passthrough_os_now_ms(void)
{
	return (uint32_t)(time_us_64() / 1000u);
}

void nina_passthrough_os_sleep_ms(uint32_t ms)
{
	uint64_t until = time_us_64() + (uint64_t)ms * 1000u;

	while (time_us_64() < until) {
		tud_task();
	}
}

int nina_passthrough_os_write(const uint8_t *data, uint32_t len)
{
	uint32_t room;

	if (!tud_mounted()) {
		return -ENOTCONN;
	}
	room = tud_cdc_write_available();
	if (room == 0) {
		return 0;
	}
	return (int)tud_cdc_write(data, len < room ? len : room);
}

void __not_in_flash_func(nina_passthrough_os_uart_irq)(bool received)
{
	(void)received;
}

void nina_passthrough_os_log_start(uint32_t baud)
{
	LOG_INF(NINA_PASSTHROUGH_OS_LOG_START, NINA_PASSTHROUGH_OS_LOG_START_ARGS(baud));
}

void nina_passthrough_os_log_counts(const char *why, const struct nina_passthrough_os_counts *c)
{
	LOG_INF(NINA_PASSTHROUGH_OS_LOG_COUNTS, NINA_PASSTHROUGH_OS_LOG_COUNTS_ARGS(why, c));
}

/* The loop ---------------------------------------------------------------*/

/* TinyUSB's callback (a weak symbol in cdc_device.h), from tud_task(): in
 * this mode from the loop below; in the normal firmware from the Pico
 * SDK's USB work, where it does nothing.
 */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	cdc_line_coding_t coding;

	if (!nina_passthrough_active() || itf != 0) {
		return;
	}
	tud_cdc_get_line_coding(&coding);
	if (nina_passthrough_host_lines(dtr, rts, coding.bit_rate)) {
		nina_passthrough_log_counts("Port closed");
	}
}

static void host_to_nina(void)
{
	if (out_pos == out_len) {
		out_pos = 0;
		out_len = tud_cdc_available() ? tud_cdc_read(out_buf, sizeof(out_buf)) : 0;
	}
	out_pos += nina_passthrough_to_module(out_buf + out_pos, out_len - out_pos);
}

static void follow_rate(void)
{
	cdc_line_coding_t coding;

	tud_cdc_get_line_coding(&coding);
	nina_passthrough_host_rate(coding.bit_rate);
}

void nina_passthrough_run(void)
{
	uint8_t *buf = pvPortMalloc(NINA_PASSTHROUGH_RING_SIZE);

	if (buf == NULL) {
		panic("nina_passthrough: no buffer");
	}
	nina_passthrough_hold();
	nina_passthrough_begin(buf);
	tusb_init();

	for (;;) {
		tud_task();
		(void)nina_passthrough_tick();
		follow_rate();
		host_to_nina();
		if (nina_passthrough_to_host()) {
			tud_cdc_write_flush();
		}
		if (nina_passthrough_exit_requested()) {
			nina_passthrough_stop();
			nina_passthrough_leave();
		}
	}
}
