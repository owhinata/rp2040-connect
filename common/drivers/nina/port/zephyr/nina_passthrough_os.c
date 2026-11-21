/* SPDX-License-Identifier: MIT */
/* The NINA passthrough mode on Zephyr (nina_passthrough_os.h of the core,
 * next to this binding; CONFIG_NINA_PASSTHROUGH): the USB device stack's
 * CDC ACM port (cdc_acm_uart0) as the host's side of the core
 * (common/drivers/nina/nina_passthrough.c, which explains the mode).  The
 * module's side is the application's
 * (common/drivers/nina/port/zephyr/nina_passthrough_bus_rp2040.c).
 *
 * - POST_KERNEL 0 (pt_check()): nina_passthrough_check(); with the request,
 *   the module is held in reset (nina_passthrough_hold()), and this file
 *   takes the USB device stack's messages (usbd_msg_register_cb() on the
 *   context of CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT, before that
 *   enables it at APPLICATION).
 * - The NINA core sees no bus (nina_bus_ready() of zephyr/nina_bus.c is
 *   false while nina_passthrough_active()), so the Wi-Fi interface's start
 *   (net_init) and every later request (the Telnet server's socket, "nina
 *   version", ...) fail with -ENODEV without touching SPI1 or the module's
 *   lines.  spi1 itself is initialized at boot as always (GPIO8 SPI1 RX,
 *   GPIO9 CS high) while the module is held in reset.
 * - POST_KERNEL 99 (pt_start(), after spi1 at 50 and the shell's backend
 *   at 90): nina_passthrough_begin() (UART1, its interrupt, the lines, the
 *   LED), and the CDC ACM port's interrupt callback, which replaces the
 *   shell's: from then on the shell gets no input and its output (the
 *   prompt) never reaches the port; its thread waits on its full transmit
 *   buffer.  The log has no backend on the port (the RAM log only,
 *   printk() included: CONFIG_LOG_PRINTK), so the port carries the
 *   module's bytes only.
 * - main() runs nina_passthrough_run() instead of the application (no
 *   key-value store, no calibration, no time synchronization, no Wi-Fi
 *   connection).
 *
 * The mode:
 * - Host to module: the CDC ACM callback (the class driver's own work
 *   queue, CONFIG_USBD_CDC_ACM_WORKQUEUE, where uart_fifo_read() and
 *   uart_fifo_fill() must be called) reads one packet (64 bytes) and puts
 *   it into UART1's transmit FIFO; what does not fit is left to UART1's
 *   transmit interrupt (nina_passthrough_os_uart_irq()), and the CDC ACM receive
 *   interrupt stays off until that interrupt has written the last byte.
 *   Meanwhile the class driver's receive buffers stay full and the host's
 *   next packets wait (NAK).
 * - Module to host: the CDC ACM callback moves the core's buffer into the
 *   class driver's transmit buffers as there is room; when the buffer is
 *   empty it turns the CDC ACM transmit interrupt off, and UART1's
 *   interrupt turns it on again with the next byte (the tick below also
 *   does, every millisecond).  While USB is not configured the tick drops
 *   the bytes (nina_passthrough_discard()).
 * - Lines: the class driver's DTR and RTS (uart_line_ctrl_get()), read on
 *   each of the stack's CONTROL_LINE_STATE messages (the system work queue,
 *   which runs right after the stack's thread has handled the request) and
 *   on each tick, to nina_passthrough_host_lines().
 * - Rate: the host's line coding (UART_LINE_CTRL_BAUD_RATE, on each
 *   LINE_CODING message and each tick) to nina_passthrough_host_rate().
 * - Way out: when the port closes at the exit rate, everything is turned
 *   off on the work queue, and main() takes nina_passthrough_leave().
 *
 * The stack's messages and the tick run on the system work queue, the CDC
 * ACM callback on the class driver's work queue (or the system one without
 * CONFIG_USBD_CDC_ACM_WORKQUEUE).  Both are cooperative threads (the build
 * asserts it), which no other thread preempts, and none of these handlers
 * sleeps, so the lines and the rate (the system work queue only), the
 * buffer's consumer side (the CDC ACM callback, and the tick's dropping
 * while USB is not configured) and the counters need no lock; the
 * interrupt paths use the buffer's and the handover's own rules.  The way
 * out runs in main() after the work queue has turned everything off.  The
 * host test is zephyr/app/usb-console/tests/passthrough.c.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/iterable_sections.h>
#include <zephyr/usb/usbd.h>

#include "nina_passthrough.h"
#include "nina_passthrough_bus.h"
#include "nina_passthrough_os.h"

LOG_MODULE_REGISTER(nina_passthrough, LOG_LEVEL_INF);

/* The two work queues (above) do not preempt each other. */
BUILD_ASSERT(CONFIG_SYSTEM_WORKQUEUE_PRIORITY < 0, "the system work queue is cooperative");

#define PT_PACKET  64u /* the full-speed bulk endpoint's size */
#define PT_TICK_MS 1

static const struct device *const cdc = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

static bool leaving;
static K_SEM_DEFINE(leave_sem, 0, 1);
static bool configured;

static uint8_t ring_buf[NINA_PASSTHROUGH_RING_SIZE];
/* Set while the CDC ACM transmit interrupt is off with the buffer empty
 * (at the start too); whoever takes it back turns the interrupt on.
 */
static atomic_t tx_idle = ATOMIC_INIT(1);

/* One packet from the host on its way into the UART; UART1's transmit
 * interrupt owns it while that interrupt is on.
 */
static uint8_t out_buf[PT_PACKET];
static volatile uint32_t out_len;
static volatile uint32_t out_pos;

static void tick(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(tick_work, tick);

/* nina_passthrough_os.h --------------------------------------------------*/

uint32_t nina_passthrough_os_now_ms(void)
{
	return k_uptime_get_32();
}

void nina_passthrough_os_sleep_ms(uint32_t ms)
{
	(void)k_msleep((int32_t)ms);
}

int nina_passthrough_os_write(const uint8_t *data, uint32_t len)
{
	int done = uart_fifo_fill(cdc, data, (int)len);

	return done < 0 ? 0 : done;
}

/* UART1's interrupt: the CDC ACM transmit interrupt on with the first byte
 * for the host, and the rest of the host's packet while UART1's transmit
 * interrupt is on.
 */
void nina_passthrough_os_uart_irq(bool received)
{
	if (received && atomic_cas(&tx_idle, 1, 0)) {
		uart_irq_tx_enable(cdc);
	}

	if (nina_passthrough_bus_uart_tx_irq_on()) {
		out_pos += nina_passthrough_to_module(out_buf + out_pos, out_len - out_pos);
		if (out_pos == out_len) {
			nina_passthrough_bus_uart_tx_irq(false);
			uart_irq_rx_enable(cdc);
		}
	}
}

void nina_passthrough_os_log_start(uint32_t baud)
{
	LOG_INF(NINA_PASSTHROUGH_OS_LOG_START, NINA_PASSTHROUGH_OS_LOG_START_ARGS(baud));
}

void nina_passthrough_os_log_counts(const char *why, const struct nina_passthrough_os_counts *c)
{
	LOG_INF(NINA_PASSTHROUGH_OS_LOG_COUNTS, NINA_PASSTHROUGH_OS_LOG_COUNTS_ARGS(why, c));
}

/* The lines and the rate ---------------------------------------------------*/

/* Everything off, then main() takes the way out. */
static void begin_leave(void)
{
	leaving = true;
	nina_passthrough_stop();
	uart_irq_rx_disable(cdc);
	uart_irq_tx_disable(cdc);
	k_sem_give(&leave_sem);
}

/* DTR and RTS as the class driver has them now. */
static void sync_lines(void)
{
	uint32_t dtr = 0;
	uint32_t rts = 0;
	uint32_t rate = 0;

	if (uart_line_ctrl_get(cdc, UART_LINE_CTRL_DTR, &dtr) != 0 ||
	    uart_line_ctrl_get(cdc, UART_LINE_CTRL_RTS, &rts) != 0 ||
	    uart_line_ctrl_get(cdc, UART_LINE_CTRL_BAUD_RATE, &rate) != 0) {
		return;
	}
	if (nina_passthrough_host_lines(dtr != 0, rts != 0, rate)) {
		nina_passthrough_log_counts("Port closed");
	}
	if (nina_passthrough_exit_requested()) {
		begin_leave();
	}
}

static void follow_rate(void)
{
	uint32_t rate;

	if (uart_line_ctrl_get(cdc, UART_LINE_CTRL_BAUD_RATE, &rate) != 0) {
		return;
	}
	nina_passthrough_host_rate(rate);
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

/* One packet from the host into the UART; the rest to UART1's transmit
 * interrupt.
 */
static void host_to_nina(const struct device *dev)
{
	uint32_t pos;
	int n;

	n = uart_fifo_read(dev, out_buf, sizeof(out_buf));
	if (n <= 0) {
		return;
	}
	pos = nina_passthrough_to_module(out_buf, (uint32_t)n);
	if (pos < (uint32_t)n) {
		out_pos = pos;
		out_len = (uint32_t)n;
		uart_irq_rx_disable(dev);
		nina_passthrough_bus_uart_tx_irq(true);
	}
}

static void nina_to_host(const struct device *dev)
{
	(void)nina_passthrough_to_host();
	if (nina_passthrough_waiting()) {
		/* The host took less: the interrupt stays on. */
		return;
	}
	/* Nothing waits: off until UART1's interrupt or the tick has more.  A
	 * byte that came in between is seen by the second look.
	 */
	uart_irq_tx_disable(dev);
	atomic_set(&tx_idle, 1);
	if (nina_passthrough_waiting() && atomic_cas(&tx_idle, 1, 0)) {
		uart_irq_tx_enable(dev);
	}
}

/* The CDC ACM port's callback (the class driver's work queue), in place
 * of the shell's.
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

/* Every millisecond on the system work queue: the lines (a change of DTR
 * or RTS whose message came late), the core's tick (GPIO0's release, the
 * LED), the rate, the buffer.
 */
static void tick(struct k_work *work)
{
	ARG_UNUSED(work);
	if (leaving) {
		return;
	}
	sync_lines();
	if (leaving) {
		return;
	}
	(void)nina_passthrough_tick();
	follow_rate();
	if (!configured) {
		nina_passthrough_discard();
	} else if (nina_passthrough_waiting()) {
		atomic_set(&tx_idle, 0);
		uart_irq_tx_enable(cdc);
	}
	k_work_schedule(&tick_work, K_MSEC(PT_TICK_MS));
}

/* The start --------------------------------------------------------------*/

static int pt_check(void)
{
	int ret = -ENODEV;

	if (!nina_passthrough_check()) {
		return 0;
	}
	nina_passthrough_hold();

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
	if (!nina_passthrough_active()) {
		return 0;
	}
	nina_passthrough_begin(ring_buf);

	uart_irq_callback_user_data_set(cdc, cdc_irq, NULL);
	uart_irq_rx_enable(cdc);
	k_work_schedule(&tick_work, K_NO_WAIT);
	return 0;
}

SYS_INIT(pt_start, POST_KERNEL, 99);

void nina_passthrough_run(void)
{
	struct k_work_sync sync;

	(void)k_sem_take(&leave_sem, K_FOREVER);
	(void)k_work_cancel_delayable_sync(&tick_work, &sync);
	nina_passthrough_leave();
}
