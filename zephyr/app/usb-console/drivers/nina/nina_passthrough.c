/* SPDX-License-Identifier: MIT */
/* The NINA passthrough mode of the Zephyr build (nina_passthrough.h): esptool
 * on the PC talks through the board's USB port to the ESP32 ROM bootloader
 * of the NINA-W102, as WiFiNINA's SerialNINAPassthrough sketch (mbed core)
 * does, without replacing this firmware or touching the flash.  The
 * behaviour is the FreeRTOS build's (freertos/drivers/nina/
 * nina_passthrough.c), with the same decisions
 * (common/drivers/nina/nina_passthrough_logic.h); the hardware is in
 * nina_passthrough_hw.h.
 *
 * GPIO8 and GPIO9 are both UART1 (to the module's UART0) and the SPI link
 * (MISO and CS), so the SPI link must not run: this mode is a start of its
 * own.  "nina passthrough confirm" leaves a request in the watchdog's
 * scratch registers 0..3 and restarts the board with the watchdog.  Then:
 *
 * - POST_KERNEL 0 (pt_check()): the request is read and cleared at once,
 *   so the RESET button, any restart and a fatal error start the normal
 *   firmware again.  Nothing is written to the flash; the state is in RAM
 *   and the scratch registers only.  With the request, the module is held
 *   in reset (RESETN low, GPIO0 high, as the closed port wants it), and
 *   this file takes the USB device stack's messages (usbd_msg_register_cb()
 *   on the context of CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT, before
 *   that enables it at APPLICATION).
 * - The NINA core (common/drivers/nina) sees no bus (nina_bus_ready()
 *   is false), so the Wi-Fi interface's start (net_init) and every later
 *   request (the Telnet server's socket, "nina version", ...) fail with
 *   -ENODEV without touching SPI1 or the module's lines.  spi1 itself is
 *   initialized at boot as always (GPIO8 SPI1 RX, GPIO9 CS high) while the
 *   module is held in reset.
 * - POST_KERNEL 99 (pt_start(), after spi1 at 50 and the shell's backend
 *   at 90): UART1 and the pins (nina_passthrough_hw.h), the UART's
 *   interrupt, and the CDC ACM port's interrupt callback, which replaces
 *   the shell's: from then on the shell gets no input and its output
 *   (the prompt) never reaches the port; its thread waits on its full
 *   transmit buffer.  The log has no backend on the port (the RAM log
 *   only, printk() included: CONFIG_LOG_PRINTK), so the port carries the
 *   module's bytes only.
 * - main() runs nina_passthrough_run() instead of the application (no
 *   key-value store, no calibration, no time synchronization, no Wi-Fi
 *   connection).
 *
 * The mode:
 * - Host to module: the CDC ACM callback (the class driver's own work queue,
 *   CONFIG_USBD_CDC_ACM_WORKQUEUE, where uart_fifo_read() and
 *   uart_fifo_fill() must be called) reads one
 *   packet (64 bytes) and puts it into UART1's transmit FIFO; what does not
 *   fit is left to UART1's transmit interrupt, and the CDC ACM receive
 *   interrupt stays off until that interrupt has written the last byte.
 *   Meanwhile the class driver's receive buffers stay full and the host's
 *   next packets wait (NAK), so nothing is lost at any rate.
 * - Module to host: UART1's interrupt (priority 0, above every other
 *   interrupt; the kernel masks interrupts only briefly and nothing in
 *   this mode writes the flash) moves every byte from the 32-byte FIFO into
 *   a 16 KiB buffer.  The interrupt comes at 4 bytes in the FIFO (or after
 *   32 bit times without more), so 28 bytes of margin remain: 300 us at
 *   921600 bps.  The CDC ACM callback moves the buffer into the class
 *   driver's transmit buffers as there is room; when the buffer is empty it
 *   turns the CDC ACM transmit interrupt off, and UART1's interrupt turns it
 *   on again with the next byte (the tick below also does, every
 *   millisecond).  The buffer holds 178 ms at 921600 bps for the times the
 *   host does not read; bytes are lost only when it stops reading for that
 *   long, and they are counted.  While USB is not configured the bytes are
 *   dropped and counted apart (as the FreeRTOS build does).
 * - Lines: the class driver's DTR and RTS (uart_line_ctrl_get()), read on
 *   each of the stack's CONTROL_LINE_STATE messages (the system work queue,
 *   which runs right after the stack's thread has handled the request) and
 *   on each tick, map as the sketch does (nina_pt_lines_*(): RTS set
 *   releases RESETN, DTR set pulls GPIO0 low), so opening the port starts
 *   the ROM's download mode and closing it holds the module in reset.
 *   When the module leaves reset into its own firmware (DTR clear), GPIO0
 *   is released after 750 ms and GPIO8 stops driving.
 * - Rate: the host's line coding (UART_LINE_CTRL_BAUD_RATE, on each
 *   LINE_CODING message and each tick) sets UART1's rate, 8N1 always
 *   (nina_pt_uart_baud()).
 * - LED (GPIO6): blinks at 5 Hz.
 * - Way out: closing the port at 300 bps (nina_pt_exit()), or the RESET
 *   button.  Then the module is reset into its own firmware as
 *   nina_spi.c's init_nina() does it (GPIO0 high, RESETN low for 10 ms,
 *   750 ms, GPIO0 released), and the watchdog restarts the board.
 * - The RAM log ("dmesg" after the way out) gets a line when the mode
 *   starts, one each time the port closes and one at the way out, with the
 *   counts.
 *
 * The stack's messages and the tick run on the system work queue, the CDC
 * ACM callback on the class driver's work queue (or the system one without
 * CONFIG_USBD_CDC_ACM_WORKQUEUE).  Both are cooperative threads (the build
 * asserts it), which no other thread preempts, and none of these handlers
 * sleeps, so the lines and the rate (the system work queue only), the
 * buffer's consumer side (the CDC ACM callback, and the tick's dropping
 * while USB is not configured) and the counters need no lock; the
 * interrupt paths use the buffer's and the handover's own rules.  The way
 * out runs in main() after the work queue has turned everything off.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/usb/usbd.h>

#include "nina_passthrough.h"
#include "nina_passthrough_hw.h"
#include "nina_passthrough_logic.h"

LOG_MODULE_REGISTER(nina_passthrough, LOG_LEVEL_INF);

/* The two work queues (above) do not preempt each other. */
BUILD_ASSERT(CONFIG_SYSTEM_WORKQUEUE_PRIORITY < 0, "the system work queue is cooperative");

#define PT_RING_SIZE        16384u
#define PT_PACKET           64u /* the full-speed bulk endpoint's size */
#define PT_TICK_MS          1
#define PT_LED_HALF_MS      100u
/* init_nina() (common/drivers/nina/nina_spi.c): the reset pulse. */
#define PT_RESET_MS         10u
/* Time for the USB code to send the command's last message, or the log
 * thread to store the last lines, before a restart.
 */
#define PT_RESTART_DELAY_MS 100u

static const struct device *const cdc = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

static bool active;
static bool leaving;
static K_SEM_DEFINE(leave_sem, 0, 1);

static struct nina_pt_lines lines;
static struct nina_pt_pins applied;
static bool applied_valid;
static uint32_t baud;
static bool configured;

static uint8_t ring_buf[PT_RING_SIZE];
static struct nina_pt_ring ring;
static volatile uint32_t uart_overruns;
static volatile uint32_t uart_breaks;
static volatile uint32_t uart_framing;
/* Set while the CDC ACM transmit interrupt is off with the buffer empty
 * (at the start too); whoever takes it back turns the interrupt on.
 */
static atomic_t tx_idle = ATOMIC_INIT(1);
static uint64_t to_nina;
static uint64_t from_nina;
static uint64_t discarded; /* from the module while USB was not configured */

/* One packet from the host on its way into the UART; UART1's transmit
 * interrupt owns it while that interrupt is on.
 */
static uint8_t out_buf[PT_PACKET];
static volatile uint32_t out_len;
static volatile uint32_t out_pos;

static uint32_t led_at;

static void tick(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tick_work, tick);

static uint32_t now_ms(void)
{
	return k_uptime_get_32();
}

bool nina_passthrough_active(void)
{
	return active;
}

/* The lines --------------------------------------------------------------*/

/* The levels of P, in the order the module needs: GPIO8 and GPIO0 settle
 * before RESETN changes, so the strap is in place when it is released.
 */
static void apply_pins(struct nina_pt_pins p)
{
	if (!applied_valid || p.uart_tx != applied.uart_tx) {
		pt_hw_uart_tx_connect(p.uart_tx);
	}
	if (!applied_valid || p.gpio0 != applied.gpio0) {
		pt_hw_gpio0(p.gpio0);
	}
	if (!applied_valid || p.reset_released != applied.reset_released) {
		pt_hw_resetn(p.reset_released);
	}
	applied = p;
	applied_valid = true;
}

static void log_counts(const char *why)
{
	LOG_INF("%s: %llu bytes to the NINA, %llu from it, %llu discarded without USB, "
		"%lu lost (buffer full, peak %lu of %lu), UART: %lu overruns, %lu breaks, "
		"%lu framing errors, %lu bps",
		why, (unsigned long long)to_nina, (unsigned long long)from_nina,
		(unsigned long long)discarded, (unsigned long)ring.dropped,
		(unsigned long)ring.peak, (unsigned long)ring.size, (unsigned long)uart_overruns,
		(unsigned long)uart_breaks, (unsigned long)uart_framing, (unsigned long)baud);
}

/* Everything off, then main() takes the way out (leave()). */
static void begin_leave(void)
{
	leaving = true;
	irq_disable(PT_HW_UART_IRQN);
	pt_hw_uart_irqs_off();
	uart_irq_rx_disable(cdc);
	uart_irq_tx_disable(cdc);
	k_sem_give(&leave_sem);
}

/* DTR and RTS as the class driver has them now; on a change, what
 * FreeRTOS's tud_cdc_line_state_cb() does: the way out at the exit rate,
 * the counts when the port closes, the module's lines.
 */
static void sync_lines(void)
{
	uint32_t dtr = 0;
	uint32_t rts = 0;
	uint32_t rate = 0;
	bool exit;
	uint32_t now;

	if (uart_line_ctrl_get(cdc, UART_LINE_CTRL_DTR, &dtr) != 0 ||
	    uart_line_ctrl_get(cdc, UART_LINE_CTRL_RTS, &rts) != 0 ||
	    uart_line_ctrl_get(cdc, UART_LINE_CTRL_BAUD_RATE, &rate) != 0) {
		return;
	}
	if ((dtr != 0) == lines.dtr && (rts != 0) == lines.rts) {
		return;
	}
	exit = nina_pt_exit(lines.dtr, dtr != 0, rate);
	if (lines.dtr && dtr == 0) {
		log_counts("Port closed");
	}
	now = now_ms();
	nina_pt_lines_set(&lines, dtr != 0, rts != 0, now);
	apply_pins(nina_pt_lines_pins(&lines, now));
	if (exit) {
		begin_leave();
	}
}

static void follow_rate(void)
{
	uint32_t rate;
	uint32_t next;

	if (uart_line_ctrl_get(cdc, UART_LINE_CTRL_BAUD_RATE, &rate) != 0) {
		return;
	}
	next = nina_pt_uart_baud(rate, baud);
	if (next != baud) {
		baud = next;
		pt_hw_uart_set_baud(baud);
	}
}

/* The USB device stack's messages, on the system work queue. */
static void usb_msg(struct usbd_context *const ctx, const struct usbd_msg *const msg)
{
	ARG_UNUSED(ctx);

	if (leaving) {
		return;
	}
	switch (msg->type) {
	case USBD_MSG_CONFIGURATION:
		configured = msg->status != 0;
		break;
	case USBD_MSG_RESET:
	case USBD_MSG_VBUS_REMOVED:
		configured = false;
		break;
	case USBD_MSG_CDC_ACM_CONTROL_LINE_STATE:
		if (msg->dev == cdc) {
			sync_lines();
		}
		break;
	case USBD_MSG_CDC_ACM_LINE_CODING:
		if (msg->dev == cdc) {
			follow_rate();
		}
		break;
	default:
		break;
	}
}

/* The data ---------------------------------------------------------------*/

/* UART1's interrupt: empties the receive FIFO into the buffer, and writes
 * the rest of the host's packet while the transmit interrupt is on.  A
 * break (the line held low, as at the module's reset) is no data and is
 * only counted.
 */
static void uart_isr(const void *arg)
{
	bool received = false;
	uint32_t dr;

	ARG_UNUSED(arg);
	while (pt_hw_uart_rx(&dr)) {
		if (dr & PT_HW_DR_OVERRUN) {
			uart_overruns++;
		}
		if (dr & PT_HW_DR_BREAK) {
			uart_breaks++;
			continue;
		}
		if (dr & PT_HW_DR_FRAMING) {
			uart_framing++;
		}
		nina_pt_ring_put(&ring, (uint8_t)dr);
		received = true;
	}
	if (received && atomic_cas(&tx_idle, 1, 0)) {
		uart_irq_tx_enable(cdc);
	}

	if (pt_hw_uart_tx_irq_on()) {
		while (out_pos < out_len && pt_hw_uart_tx(out_buf[out_pos])) {
			out_pos++;
		}
		if (out_pos == out_len) {
			pt_hw_uart_tx_irq(false);
			uart_irq_rx_enable(cdc);
		}
	}
}

/* One packet from the host into the UART; the rest to the interrupt. */
static void host_to_nina(const struct device *dev)
{
	uint32_t pos = 0;
	int n;

	n = uart_fifo_read(dev, out_buf, sizeof(out_buf));
	if (n <= 0) {
		return;
	}
	to_nina += (uint32_t)n;
	while (pos < (uint32_t)n && pt_hw_uart_tx(out_buf[pos])) {
		pos++;
	}
	if (pos < (uint32_t)n) {
		out_pos = pos;
		out_len = (uint32_t)n;
		uart_irq_rx_disable(dev);
		pt_hw_uart_tx_irq(true);
	}
}

static void nina_to_host(const struct device *dev)
{
	const uint8_t *data;
	uint32_t n;

	while ((n = nina_pt_ring_peek(&ring, &data)) > 0) {
		int done = uart_fifo_fill(dev, data, (int)n);

		if (done <= 0) {
			return;
		}
		nina_pt_ring_consume(&ring, (uint32_t)done);
		from_nina += (uint32_t)done;
	}
	/* Nothing waits: off until UART1's interrupt or the tick has more.  A
	 * byte that came in between is seen by the second look.
	 */
	uart_irq_tx_disable(dev);
	atomic_set(&tx_idle, 1);
	if (nina_pt_ring_peek(&ring, &data) > 0 && atomic_cas(&tx_idle, 1, 0)) {
		uart_irq_tx_enable(dev);
	}
}

/* The CDC ACM port's callback (the system work queue), in place of the
 * shell's.
 */
static void cdc_irq(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (leaving) {
		uart_irq_rx_disable(dev);
		uart_irq_tx_disable(dev);
		return;
	}
	uart_irq_update(dev);
	if (uart_irq_rx_ready(dev)) {
		host_to_nina(dev);
	}
	if (uart_irq_tx_ready(dev)) {
		nina_to_host(dev);
	}
}

/* Every millisecond on the system work queue: the lines (GPIO0's release
 * 750 ms after a start into the module's firmware, and a change of DTR or
 * RTS whose message came late), the rate, the buffer, the LED.
 */
static void tick(struct k_work *work)
{
	const uint8_t *data;
	uint32_t now = now_ms();
	uint32_t n;

	ARG_UNUSED(work);
	if (leaving) {
		return;
	}
	sync_lines();
	if (leaving) {
		return;
	}
	apply_pins(nina_pt_lines_pins(&lines, now));
	follow_rate();
	if (!configured) {
		while ((n = nina_pt_ring_peek(&ring, &data)) > 0) {
			nina_pt_ring_consume(&ring, n);
			discarded += n;
		}
	} else if (nina_pt_ring_peek(&ring, &data) > 0) {
		atomic_set(&tx_idle, 0);
		uart_irq_tx_enable(cdc);
	}
	if ((int32_t)(now - led_at) >= 0) {
		pt_hw_led_toggle();
		led_at = now + PT_LED_HALF_MS;
	}
	k_work_schedule(&tick_work, K_MSEC(PT_TICK_MS));
}

/* The start --------------------------------------------------------------*/

static int pt_check(void)
{
	uint32_t s[NINA_PT_MARK_WORDS];
	int ret = -ENODEV;

	pt_hw_mark_get(s);
	if (!nina_pt_mark_valid(s)) {
		return 0;
	}
	memset(s, 0, sizeof(s));
	pt_hw_mark_set(s);
	active = true;

	/* The module's lines as the closed port (DTR, RTS clear) wants them:
	 * held in reset, GPIO0 high.
	 */
	nina_pt_lines_init(&lines);
	baud = NINA_PT_BAUD_INIT;
	pt_hw_lines_init();
	pt_hw_gpio0(nina_pt_lines_pins(&lines, now_ms()).gpio0);

	STRUCT_SECTION_FOREACH(usbd_context, ctx) {
		ret = usbd_msg_register_cb(ctx, usb_msg);
	}
	if (ret != 0) {
		/* The tick still follows the lines and the rate; USB is taken
		 * as configured.
		 */
		configured = true;
		LOG_ERR("NINA passthrough: no USB messages (%d)", ret);
	}
	return 0;
}

SYS_INIT(pt_check, POST_KERNEL, 0);

static int pt_start(void)
{
	if (!active) {
		return 0;
	}
	nina_pt_ring_init(&ring, ring_buf, sizeof(ring_buf));
	pt_hw_uart_init(baud);
	applied_valid = false;
	apply_pins(nina_pt_lines_pins(&lines, now_ms()));

	IRQ_CONNECT(PT_HW_UART_IRQN, 0, uart_isr, NULL, 0);
	irq_enable(PT_HW_UART_IRQN);

	uart_irq_callback_user_data_set(cdc, cdc_irq, NULL);
	uart_irq_rx_enable(cdc);

	pt_hw_led_init();
	led_at = now_ms();

	LOG_INF("NINA passthrough: USB <-> UART1 (GPIO8 TX, GPIO9 RX) at %lu bps, "
		"DTR -> GPIO0 (GPIO2), RTS -> RESETN (GPIO3); close the port at %u bps or "
		"press RESET to leave",
		(unsigned long)baud, NINA_PT_EXIT_BAUD);
	k_work_schedule(&tick_work, K_NO_WAIT);
	return 0;
}

SYS_INIT(pt_start, POST_KERNEL, 99);

/* The way out: the module into its own firmware, as init_nina() does it,
 * then the watchdog's restart into the normal firmware.
 */
void nina_passthrough_run(void)
{
	struct k_work_sync sync;

	k_sem_take(&leave_sem, K_FOREVER);
	(void)k_work_cancel_delayable_sync(&tick_work, &sync);
	log_counts("Leaving the NINA passthrough");
	pt_hw_uart_tx_connect(false);
	pt_hw_gpio0(NINA_PT_GPIO0_HIGH);
	pt_hw_resetn(false);
	k_msleep(PT_RESET_MS);
	pt_hw_resetn(true);
	k_msleep(NINA_PT_GPIO0_HOLD_MS);
	pt_hw_gpio0(NINA_PT_GPIO0_RELEASED);
	pt_hw_led_off();
	k_msleep(PT_RESTART_DELAY_MS);
	pt_hw_restart();
}

/* The command ------------------------------------------------------------*/

int nina_passthrough_cmd(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t s[NINA_PT_MARK_WORDS];

	ARG_UNUSED(argc);
	if (strcmp(argv[1], "confirm") != 0) {
		shell_error(sh, "Type \"nina passthrough confirm\" to restart into the NINA "
				"passthrough");
		return -EINVAL;
	}
	shell_print(sh, "Restarting into the NINA passthrough: the USB port relays to the "
			"NINA's UART (for esptool).");
	shell_print(sh, "To leave it: close the port at %u bps, or press RESET.",
		    NINA_PT_EXIT_BAUD);
	k_msleep(PT_RESTART_DELAY_MS);
	nina_pt_mark(s);
	pt_hw_mark_set(s);
	pt_hw_restart();
}
