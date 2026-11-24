/* SPDX-License-Identifier: MIT */
/* The long transfers of nina_bus_rp2040.c by DMA, on the Pico SDK's
 * hardware_dma and hardware_irq and the OS's wait (nina_bus_dma_os.h): the
 * calling task sleeps while two DMA channels move the bytes through SPI1's
 * data register, instead of spinning on the FIFOs.  Shorter transfers stay
 * on the SDK's polled calls (nina_bus_rp2040.c).
 *
 * - nina_bus_dma_initialize(), from nina_bus_initialize() (the normal
 *   mode's first request; the passthrough mode never calls it): two
 *   channels from the SDK's record (dma_claim_unused_channel(false), kept
 *   for good; none kept if the second is missing, and an error logged),
 *   this file's handler added to DMA_IRQ_1 with irq_add_shared_handler()
 *   (the default order priority) and the line enabled, then the RX
 *   channel's interrupt there.  DMA_IRQ_1 is the line of the I2C0 reads'
 *   channels (common/drivers/i2c_dw/port/pico_sdk/i2c_dw_dma_os_rp2040.c),
 *   shared the SDK's way: each handler looks at and acknowledges only its
 *   own channels' flags (INTS1 written with its own bit alone; INTS is
 *   write 1 to clear, never written through an alias), and DMA_IRQ_0 stays
 *   free for other users.
 * - The threshold: a transfer of at least NINA_BUS_DMA_MIN (64) bytes goes
 *   by DMA once the scheduler runs.  The set-up of the two channels, the
 *   switch away, the interrupt and the switch back cost the CPU about
 *   10-20 us, as much as 10-20 bytes of polling at 1.024 us a byte
 *   (7.8125 MHz); at 64 the polling it saves is at least three times that,
 *   and the core's single bytes, commands and short parameters (up to 63
 *   bytes) stay polled.  The long ones are the core's reads in chunks of
 *   128 bytes and the parameters of the socket writes.
 * - A transfer: SPI1's end state checked (the shifter idle, the RX FIFO
 *   empty, the overrun flag cleared, as the SDK's polled calls leave it),
 *   the task's wake-up cleared (nina_bus_dma_os_wake_clear()), the RX
 *   channel started (8-bit reads of SSPDR without increment, DREQ_SPI1_RX,
 *   into RX or, when RX is NULL, into one sink byte without increment),
 *   SSPDMACR's TXDMAE and RXDMAE set, the TX channel started (8-bit reads
 *   of TX with increment, so the same bytes as the polled calls, into SSPDR
 *   without increment, DREQ_SPI1_TX).  The transfer has ended when the RX
 *   channel has taken LEN bytes: every byte clocked out and in.  The
 *   handler acknowledges the RX channel's flag, notes the end and notifies
 *   the task (nina_bus_dma_os_wake_from_isr(), nina_bus_dma_os_isr_end()).  Then
 *   SSPDMACR is cleared again (off between transfers, as the polled calls
 *   expect), the TX channel's flag acknowledged, and the end state checked
 *   once more.  No bus priority is set.
 * - The wait: in the calling task (nina_proto.c holds nina_lock: one
 *   transfer at a time), on its wake-up, for at most 2 ms + LEN / 256
 *   ms rounded up to ticks, plus one tick (the transfer itself takes
 *   LEN x 1.024 us).  A notification left from a socket event wait or a
 *   wake-up of the Wi-Fi thread (the same wake-up, nina_os.h) is cleared
 *   before the start, and the wait looks at the end flag, not at the
 *   wake-up.  If the time runs out, or a
 *   channel reports a bus error: the RX channel's interrupt off, both
 *   channels aborted (CHAN_ABORT) and waited for at most
 *   NINA_BUS_DMA_STOP_US, both flags acknowledged (an abort may raise one:
 *   RP2040-E13, as the SDK's dma_channel_abort() describes), the error
 *   flags cleared, the interrupt on again; SPI1's end state as above.
 *   A channel that does not stop leaves the DMA unused from then on (every
 *   transfer polled).
 * - The errors: a bus error of a channel, the overrun flag after the
 *   transfer (a received byte lost, so the RX channel never ends either),
 *   or a transfer that did not end in time return -EIO, which the core
 *   passes on as the request's error, and log one line each in the RAM log
 *   ("dmesg": <err> nina_bus), rate-limited per kind (LOG_ERR_RATELIMIT():
 *   one line per 5 s, then the number held back), so that a fault of every
 *   long transfer does not fill the log.  Giving up the DMA logs once.
 *
 * The wake-up is the calling thread's own, the one of the NINA's other
 * waits (nina_os.h; on FreeRTOS a task notification,
 * port/freertos/nina_bus_dma_os_impl.h).
 *
 * The handler is short (a flag, a notification) and runs at the SDK's
 * default priority (0x80), as I2C0's and the other DMA handler's: on the
 * single core they do not interrupt each other.  While the DMA runs the
 * calling task is blocked: the IMU's FIFO task and the I2C0 reads run as
 * before, now without a spinning task beneath them.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/timer.h"

#include "app_log.h"
#include "nina_board_rp2040.h"
#include "nina_bus_dma_os.h"
#include "nina_bus_dma_rp2040.h"

LOG_MODULE_REGISTER(nina_bus, LOG_LEVEL_INF);

/* The DMA interrupt line of the channels (0 or 1). */
#define NINA_BUS_DMA_IRQ_INDEX 1

/* The threshold in bytes (see above). */
#define NINA_BUS_DMA_MIN 64

/* The longest wait for an abort and for the shifter to finish, in us. */
#define NINA_BUS_DMA_STOP_US 100

#define NINA_BUS_DMA_ERROR_BITS                                                                    \
	(DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS)

static bool nina_bus_dma_ready;
static uint32_t nina_bus_dma_tx;
static uint32_t nina_bus_dma_rx;
static uint8_t nina_bus_dma_sink;
static nina_bus_dma_os_thread_t volatile nina_bus_dma_waiter;
static volatile bool nina_bus_dma_done;

static void nina_bus_dma_isr(void)
{
	nina_bus_dma_os_woken_t woken = NINA_BUS_DMA_OS_NOT_WOKEN;
	nina_bus_dma_os_thread_t task;

	if (!dma_irqn_get_channel_status(NINA_BUS_DMA_IRQ_INDEX, nina_bus_dma_rx)) {
		return;
	}
	dma_irqn_acknowledge_channel(NINA_BUS_DMA_IRQ_INDEX, nina_bus_dma_rx);
	nina_bus_dma_done = true;
	task = nina_bus_dma_waiter;
	if (task != NULL) {
		nina_bus_dma_os_wake_from_isr(task, &woken);
	}
	nina_bus_dma_os_isr_end(woken);
}

void nina_bus_dma_initialize(void)
{
	int tx, rx;

	if (nina_bus_dma_ready) {
		return;
	}
	tx = dma_claim_unused_channel(false);
	rx = tx < 0 ? -1 : dma_claim_unused_channel(false);
	if (rx < 0) {
		if (tx >= 0) {
			dma_channel_unclaim((uint)tx);
		}
		LOG_ERR("No DMA channels for SPI1: every transfer polled");
		return;
	}
	nina_bus_dma_tx = (uint32_t)tx;
	nina_bus_dma_rx = (uint32_t)rx;
	irq_add_shared_handler(DMA_IRQ_NUM(NINA_BUS_DMA_IRQ_INDEX), nina_bus_dma_isr,
			       PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
	irq_set_enabled(DMA_IRQ_NUM(NINA_BUS_DMA_IRQ_INDEX), true);
	dma_irqn_acknowledge_channel(NINA_BUS_DMA_IRQ_INDEX, nina_bus_dma_rx);
	dma_irqn_set_channel_enabled(NINA_BUS_DMA_IRQ_INDEX, nina_bus_dma_rx, true);
	nina_bus_dma_ready = true;
}

bool nina_bus_dma_wanted(size_t len)
{
	return nina_bus_dma_ready && len >= NINA_BUS_DMA_MIN && nina_bus_dma_os_running();
}

/* The end state of the SDK's polled transfers: the shifter idle (for at
 * most NINA_BUS_DMA_STOP_US), the RX FIFO empty, no overrun flag.  True
 * when the overrun flag was set.
 */
static bool nina_bus_dma_spi_settle(spi_hw_t *hw)
{
	uint64_t start = time_us_64();
	bool overrun;

	while ((hw->sr & SPI_SSPSR_BSY_BITS) && time_us_64() - start < NINA_BUS_DMA_STOP_US) {
	}
	while (hw->sr & SPI_SSPSR_RNE_BITS) {
		(void)hw->dr;
	}
	overrun = (hw->ris & SPI_SSPRIS_RORRIS_BITS) != 0;
	hw->icr = SPI_SSPICR_RORIC_BITS;
	return overrun;
}

static bool nina_bus_dma_busy(uint32_t mask)
{
	return dma_channel_is_busy(nina_bus_dma_tx) || dma_channel_is_busy(nina_bus_dma_rx) ||
	       (dma_hw->abort & mask) != 0;
}

/* Both channels aborted (see above).  False when they did not stop. */
static bool nina_bus_dma_stop(void)
{
	uint32_t mask = (1u << nina_bus_dma_tx) | (1u << nina_bus_dma_rx);
	uint64_t start;
	bool busy;

	dma_irqn_set_channel_enabled(NINA_BUS_DMA_IRQ_INDEX, nina_bus_dma_rx, false);
	dma_hw->abort = mask;
	start = time_us_64();
	while ((busy = nina_bus_dma_busy(mask)) && time_us_64() - start < NINA_BUS_DMA_STOP_US) {
	}
	dma_irqn_acknowledge_channel(NINA_BUS_DMA_IRQ_INDEX, nina_bus_dma_tx);
	dma_irqn_acknowledge_channel(NINA_BUS_DMA_IRQ_INDEX, nina_bus_dma_rx);
	hw_set_bits(&dma_hw->ch[nina_bus_dma_tx].al1_ctrl, NINA_BUS_DMA_ERROR_BITS);
	hw_set_bits(&dma_hw->ch[nina_bus_dma_rx].al1_ctrl, NINA_BUS_DMA_ERROR_BITS);
	if (busy) {
		return false;
	}
	dma_irqn_set_channel_enabled(NINA_BUS_DMA_IRQ_INDEX, nina_bus_dma_rx, true);
	return true;
}

/* The wait's ticks: 2 ms + LEN / 256 ms, rounded up to ticks, plus one. */
static nina_bus_dma_os_ticks_t nina_bus_dma_ticks(size_t len)
{
	return nina_bus_dma_os_ticks_from_ms(2u + len / 256u);
}

int nina_bus_dma_transceive(const uint8_t *tx, uint8_t *rx, size_t len)
{
	spi_hw_t *hw = spi_get_hw(NINA_SPI);
	nina_bus_dma_os_ticks_t limit = nina_bus_dma_ticks(len);
	nina_bus_dma_os_ticks_t start;
	dma_channel_config c;
	bool error, overrun;

	(void)nina_bus_dma_spi_settle(hw);
	nina_bus_dma_os_wake_clear();
	nina_bus_dma_done = false;
	nina_bus_dma_waiter = nina_bus_dma_os_self();

	c = dma_channel_get_default_config(nina_bus_dma_rx);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, rx != NULL);
	channel_config_set_dreq(&c, spi_get_dreq(NINA_SPI, false));
	dma_channel_configure(nina_bus_dma_rx, &c, rx != NULL ? rx : &nina_bus_dma_sink, &hw->dr,
			      dma_encode_transfer_count((uint)len), true);

	hw->dmacr = SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS;

	c = dma_channel_get_default_config(nina_bus_dma_tx);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_dreq(&c, spi_get_dreq(NINA_SPI, true));
	dma_channel_configure(nina_bus_dma_tx, &c, &hw->dr, tx,
			      dma_encode_transfer_count((uint)len), true);

	start = nina_bus_dma_os_tick_count();
	while (!nina_bus_dma_done) {
		nina_bus_dma_os_ticks_t spent = nina_bus_dma_os_tick_count() - start;

		if (spent >= limit) {
			break;
		}
		nina_bus_dma_os_wake_wait(limit - spent);
	}

	hw->dmacr = 0;
	error = ((dma_hw->ch[nina_bus_dma_tx].al1_ctrl | dma_hw->ch[nina_bus_dma_rx].al1_ctrl) &
		 DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS) != 0;
	if (!nina_bus_dma_done || error) {
		if (!nina_bus_dma_stop()) {
			nina_bus_dma_ready = false;
			LOG_ERR("SPI1 DMA channels %u and %u do not stop: every transfer polled",
				(unsigned int)nina_bus_dma_tx, (unsigned int)nina_bus_dma_rx);
		}
	} else {
		dma_irqn_acknowledge_channel(NINA_BUS_DMA_IRQ_INDEX, nina_bus_dma_tx);
	}
	nina_bus_dma_waiter = NULL;
	overrun = nina_bus_dma_spi_settle(hw);

	if (error) {
		LOG_ERR_RATELIMIT("SPI1 DMA bus error (%u bytes)", (unsigned int)len);
		return -EIO;
	}
	if (overrun) {
		LOG_ERR_RATELIMIT("SPI1 RX overrun by DMA (%u bytes)", (unsigned int)len);
		return -EIO;
	}
	if (!nina_bus_dma_done) {
		LOG_ERR_RATELIMIT("SPI1 DMA did not end in %u ticks (%u bytes)",
				  (unsigned int)limit, (unsigned int)len);
		return -EIO;
	}
	return 0;
}
