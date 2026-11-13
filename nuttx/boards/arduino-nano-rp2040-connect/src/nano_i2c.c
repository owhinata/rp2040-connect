/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_i2c.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Andrei-Edward Popa
 *
 ****************************************************************************/

/* I2C0 of the RP2040 (a DesignWare DW_apb_i2c block): a port of the
 * controller path of the Zephyr build's driver,
 * zephyr/app/usb-console/drivers/i2c_dw/i2c_dw.c (Zephyr's i2c_dw.c with
 * the RP2040 changes and the DMA reads), with the same functions, branches,
 * order, register values, timeout and error values.  NuttX's own
 * arch/arm/src/rp2040/rp2040_i2c.c (CONFIG_RP2040_I2C) is not used.
 *
 *   Zephyr                                here
 *   struct device, dev->data, dev->config struct i2c_dw_dev_config, the
 *                                         constants of nano_i2c_dw.h
 *   i2c_transfer() (msgs, num, addr)      I2C_TRANSFER() (struct i2c_msg_s:
 *                                         nano_i2c_dw_msg_flags() turns the
 *                                         NuttX flags into I2C_MSG_*)
 *   i2c_configure() (speed class)         the message frequency, applied by
 *                                         i2c_dw_transfer()
 *   i2c_recover_bus()                     I2C_RESET()
 *   k_sem (bus_sem, device_sync_sem)      sem_t (no priority inheritance)
 *   k_sem_take(.., timeout)               nxsem_tickwait_uninterruptible()
 *   k_sem_reset()                         nxsem_reset(.., 0)
 *   IRQ_CONNECT(), irq_enable()           irq_attach(), up_enable_irq()
 *   reset_line_toggle_dt()                RESETS (set, clear, RESET_DONE)
 *   pinctrl_apply_state()                 i2c_dw_pinctrl() (Zephyr's
 *                                         pinctrl_rpi_pico.c steps)
 *   LOG_*() of module i2c_dw at level 3   the same (app_log.h)
 *
 * Left out (not built in the Zephyr build or not used by it): target
 * (slave) mode, the Intel LPSS DMA, CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS
 * (need_setup), CONFIG_I2C_DW_EXTENDED_SUPPORT (SDA/SCL timeouts), power
 * management, PCIe, the RTS5912 quirk, the recover/check-bus callbacks
 * (only wrappers of other SoCs register them; without one the recovery
 * does nothing and returns 0), the never-set i2c_stat_not_ready counter,
 * and other instances than i2c0.
 *
 * The DMA reads of i2c0 (CONFIG_I2C_DW_RP2040_DMA of the Zephyr build,
 * i2c_dw.c:578-801, 816-837, 1045-1049, 1191-1196, 1290-1302, 1469-1473,
 * 1891-1893) are CONFIG_NANO_I2C0_DMA: the same conditions, command words,
 * thresholds, interrupt mask, start order, completion and timeout, clean-up
 * and stop sequence, quarantine, counters and log lines, on nano_dma.c (a
 * port of Zephyr's dma_rpi_pico.c).  atomic_t becomes a volatile word with
 * the read-modify-write under up_irq_save() where an interrupt also writes
 * it (rp_dma_events); the counters are only written by the transfer, which
 * holds bus_sem.
 *
 * Differences of the NuttX interface (see nuttx/drivers/README.md):
 * the speed comes with each transfer (msgs[0].frequency; 100 kHz, 400 kHz
 * and 1 MHz, as Zephyr's i2c_map_dt_bitrate() takes them) instead of from
 * i2c_configure(); one address per transfer; no 10-bit addresses.
 */

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>
#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rp2040_memorymap.h"
#include "hardware/rp2040_pads_bank0.h"
#include "hardware/rp2040_resets.h"
#include "rp2040_gpio.h"

/* CONFIG_I2C_LOG_LEVEL of the Zephyr build (3, info). */

#define LOG_LEVEL 3
#include "app_log.h"

#include "nano_i2c.h"
#include "nano_i2c_dw.h"

#ifdef CONFIG_NANO_I2C0_DMA
#  include "i2c_dw_rp2040_commands.h"
#  include "nano_dma.h"
#  include "nano_i2c_dma.h"
#endif

LOG_MODULE_REGISTER(i2c_dw);

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

#if NANO_I2C_M_READ != I2C_M_READ || NANO_I2C_M_TEN != I2C_M_TEN || \
    NANO_I2C_M_NOSTOP != I2C_M_NOSTOP || NANO_I2C_M_NOSTART != I2C_M_NOSTART
#  error "nano_i2c_dw.h: the I2C_M_* values do not match NuttX's"
#endif

/* CONFIG_I2C_DW_CLOCK_SPEED of the Zephyr build: clk_peri in MHz. */

#if BOARD_PERI_FREQ % 1000000 != 0
#  error "clk_peri is not a whole number of MHz"
#endif
#define I2C_DW_CLOCK_SPEED (BOARD_PERI_FREQ / 1000000)

/* The devicetree name of the Zephyr build's controller (dev->name). */

#define I2C_DW_NAME "i2c@40044000"

struct i2c_dw_dev_config {
	struct i2c_master_s dev; /* NuttX's I2C master: first */
	mm_reg_t base;
	sem_t device_sync_sem;
	sem_t bus_sem;
	uint32_t speed; /* the speed class of Zephyr's app_config */

	volatile uint8_t *xfr_buf;
	volatile uint32_t xfr_len;
	volatile uint32_t rx_pending;

	uint16_t hcnt;
	uint16_t lcnt;

	volatile uint8_t state; /* last direction of transfer */
	uint32_t request_bytes;
	uint8_t xfr_flags;
	bool support_hs_mode;
	bool ready;

#ifdef CONFIG_NANO_I2C0_DMA
	/* i2c_dw.h:198-207 */
	sem_t rp_dma_sem;
	volatile uint32_t rp_dma_events;
	volatile uint32_t rp_dma_active;
	int rp_dma_tx_channel;
	int rp_dma_rx_channel;
	bool rp_dma_quarantined;
	uint16_t rp_dma_commands[I2C_DW_RP2040_DMA_MAX_READ];
	uint8_t rp_dma_buffer[I2C_DW_RP2040_DMA_MAX_READ];
#endif
};

static int i2c_dw_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs,
			   int count);
#ifdef CONFIG_I2C_RESET
static int i2c_dw_recovery_bus(struct i2c_master_s *dev);
#endif

static const struct i2c_ops_s funcs = {
	.transfer = i2c_dw_transfer,
#ifdef CONFIG_I2C_RESET
	.reset = i2c_dw_recovery_bus,
#endif
};

/* In .bss, as Zephyr's i2c_0_runtime (the DMA buffers make it about
 * 11 KiB); nano_i2c0_initialize() sets the operations and the base.
 */

static struct i2c_dw_dev_config i2c_0_runtime;

/* No recover-bus callback is registered (i2c_dw.c:108-122). */

static int i2c_recovery_bus(struct i2c_dw_dev_config *dw)
{
	(void)dw;
	return 0;
}

#ifdef CONFIG_I2C_RESET
static int i2c_dw_recovery_bus(struct i2c_master_s *dev)
{
	int ret = 0;
	struct i2c_dw_dev_config *const dw = (struct i2c_dw_dev_config *)dev;

	/* lock bus */
	ret = nxsem_wait_uninterruptible(&dw->bus_sem);
	if (ret != 0) {
		return ret;
	}
	/* do bus recovery */
	ret = i2c_recovery_bus(dw);
	/* unlock bus */
	nxsem_post(&dw->bus_sem);

	return ret;
}
#endif

static int i2c_dw_error_chk(struct i2c_dw_dev_config *dw)
{
	mm_reg_t reg_base = dw->base;
	union ic_interrupt_register intr_stat;
	union ic_txabrt_register ic_txabrt_src;
	/* Cache ic_intr_stat and txabrt_src for processing,
	 * so there is no need to read the register multiple times.
	 */
	intr_stat.raw = read_intr_stat(reg_base);
	ic_txabrt_src.raw = read_txabrt_src(reg_base);
	/* NACK and SDA_STUCK are below TX_Abort */
	if (intr_stat.bits.tx_abrt) {
		/* check 7bit NACK Tx Abort */
		if (ic_txabrt_src.bits.ADDR7BNACK) {
			dw->state |= I2C_DW_NACK;
			LOG_ERR_RATELIMIT("NACK on %s", I2C_DW_NAME);
		}
		/* check SDA stuck low Tx abort, need to do bus recover */
		if (ic_txabrt_src.bits.SDASTUCKLOW) {
			dw->state |= I2C_DW_SDA_STUCK;
			LOG_ERR("SDA Stuck Low on %s", I2C_DW_NAME);
		}
		/* check if user abort the transmit */
		if (ic_txabrt_src.bits.USRABRT) {
			dw->state |= I2C_DW_USER_ABRT;
			LOG_ERR("User Abort on %s", I2C_DW_NAME);
		}
		/* check if user abort the transmit */
		if (ic_txabrt_src.bits.ARBLOST) {
			dw->state |= I2C_DW_USER_ABRT;
			LOG_ERR("ARB lost on %s", I2C_DW_NAME);
		}
		/* TX abrt because STOP */
		if (intr_stat.bits.stop_det) {
			dw->state |= I2C_DW_USER_ABRT;
			LOG_ERR_RATELIMIT("ABR and STOP on %s", I2C_DW_NAME);
		}
	}

	/* check SCL stuck low */
	if (intr_stat.bits.scl_stuck_low) {
		dw->state |= I2C_DW_SCL_STUCK;
		LOG_ERR("SCL Stuck Low on %s", I2C_DW_NAME);
	}
	if (dw->state & I2C_DW_ERR_MASK) {
		LOG_ERR_RATELIMIT("IO Fail on %s", I2C_DW_NAME);
		return -EIO;
	}
	return 0;
}

static inline void i2c_dw_data_ask(struct i2c_dw_dev_config *dw)
{
	uint32_t data;
	int tx_empty;
	int rx_empty;
	int cnt;
	int rx_buffer_depth, tx_buffer_depth;
	mm_reg_t reg_base = dw->base;

	/* No more bytes to request, so command queue is no longer needed */
	if (dw->request_bytes == 0U) {
		clear_bit_intr_mask_tx_empty(reg_base);
		return;
	}

	/* RP2040 does not implement IC_COMP_PARAM_1 (i2c_dw.c:63-66, 387-394). */
	rx_buffer_depth = RP2040_I2C_FIFO_DEPTH;
	tx_buffer_depth = RP2040_I2C_FIFO_DEPTH;

	/* How many bytes we can actually ask */
	rx_empty = (rx_buffer_depth - read_rxflr(reg_base)) - dw->rx_pending;

	if (rx_empty < 0) {
		/* Continue to the RX wait path so TX_EMPTY can be masked. */
		rx_empty = 0;
	}

	/* How many empty slots in TX FIFO (as command queue) */
	tx_empty = tx_buffer_depth - read_txflr(reg_base);

	/* Figure out how many bytes we can request */
	cnt = MIN(rx_buffer_depth, dw->request_bytes);
	cnt = MIN(MIN(tx_empty, rx_empty), cnt);

	/* TX_EMPTY is level-triggered. When the RX FIFO has no safe space,
	 * mask it until pending reads make enough progress for RX_FULL.
	 */
	if (cnt == 0 && dw->rx_pending > 0U) {
		uint32_t threshold = MIN(RP2040_I2C_RX_BATCH,
					 MIN(dw->xfr_len, dw->rx_pending));

		if (threshold > 0U) {
			write_rx_tl(threshold - 1U, reg_base);
			clear_bit_intr_mask_tx_empty(reg_base);
			return;
		}
	}

	while (cnt > 0) {
		/* Tell controller to get another byte, RESTART and STOP as
		 * needed.
		 */
		data = nano_i2c_dw_read_cmd(&dw->xfr_flags, dw->request_bytes);

		write_cmd_data(data, reg_base);

		if (i2c_dw_error_chk(dw)) {
			return;
		}

		dw->rx_pending++;
		dw->request_bytes--;
		cnt--;
	}
}

static void i2c_dw_data_read(struct i2c_dw_dev_config *dw)
{
	mm_reg_t reg_base = dw->base;

	while (test_bit_status_rfne(reg_base) && (dw->xfr_len > 0)) {
		dw->xfr_buf[0] = (uint8_t)read_cmd_data(reg_base);

		dw->xfr_buf++;
		dw->xfr_len--;
		dw->rx_pending--;

		if (dw->xfr_len == 0U) {
			break;
		}

		if (i2c_dw_error_chk(dw)) {
			return;
		}
	}
	/* Lower the threshold for the last short portion of a read. */
	if (dw->xfr_len > 0U) {
		write_rx_tl(MIN(RP2040_I2C_RX_BATCH, dw->xfr_len) - 1U, reg_base);
		if (dw->request_bytes > 0U) {
			set_bit_intr_mask_tx_empty(reg_base);
		}
	}
	/* Nothing to receive anymore */
	if (dw->xfr_len == 0U) {
		dw->state &= ~I2C_DW_CMD_RECV;
		return;
	}
}

static int i2c_dw_data_send(struct i2c_dw_dev_config *dw)
{
	uint32_t data = 0U;
	mm_reg_t reg_base = dw->base;

	/* Nothing to send anymore, mask the interrupt */
	if (dw->xfr_len == 0U) {
		clear_bit_intr_mask_tx_empty(reg_base);

		dw->state &= ~I2C_DW_CMD_SEND;

		return 0;
	}

	while (test_bit_status_tfnt(reg_base) && (dw->xfr_len > 0)) {
		/* We have something to transmit to a specific host, with
		 * RESTART and STOP as needed.
		 */
		data = nano_i2c_dw_write_cmd(&dw->xfr_flags, dw->xfr_buf[0],
					     dw->xfr_len);

		write_cmd_data(data, reg_base);
		dw->xfr_len--;
		dw->xfr_buf++;

		if (test_bit_intr_stat_tx_abrt(reg_base)) {
			return -EIO;
		}

		if (i2c_dw_error_chk(dw)) {
			return -EIO;
		}
	}

	return 0;
}

static inline void i2c_dw_transfer_complete(struct i2c_dw_dev_config *dw)
{
	mm_reg_t reg_base = dw->base;

	write_intr_mask(DW_DISABLE_ALL_I2C_INT, reg_base);
	(void)read_clr_intr(reg_base);
	nxsem_post(&dw->device_sync_sem);
}

#ifdef CONFIG_NANO_I2C0_DMA
#define RP_DMA_TX_DONE BIT(0)
static volatile uint32_t rp_dma_transfers, rp_dma_bytes, rp_dma_errors, rp_dma_fallbacks;
static volatile bool rp_dma_enabled = true;
#define RP_DMA_RX_DONE BIT(1)
#define RP_DMA_STOP    BIT(2)
#define RP_DMA_ERROR   BIT(3)
#define RP_DMA_DONE    (RP_DMA_TX_DONE | RP_DMA_RX_DONE | RP_DMA_STOP)

/* atomic_or(): returns the old value. */

static uint32_t rp_dma_atomic_or(volatile uint32_t *target, uint32_t value)
{
	irqstate_t flags = up_irq_save();
	uint32_t old = *target;

	*target = old | value;
	up_irq_restore(flags);
	return old;
}

/* k_uptime_get(): milliseconds since boot. */

static int64_t rp_dma_uptime_ms(void)
{
	return (int64_t)TICK2MSEC(clock_systime_ticks());
}

static void i2c_dw_rp_dma_notify(struct i2c_dw_dev_config *dw, uint32_t events)
{
	uint32_t all = rp_dma_atomic_or(&dw->rp_dma_events, events) | events;
	if ((all & RP_DMA_ERROR) || (all & RP_DMA_DONE) == RP_DMA_DONE) {
		nxsem_post(&dw->rp_dma_sem);
	}
}

static void i2c_dw_rp_dma_callback(void *user_data, uint32_t channel, int status)
{
	struct i2c_dw_dev_config *dw = user_data;
	if (!dw->rp_dma_active) {
		return;
	}
	i2c_dw_rp_dma_notify(dw, status < 0 ? RP_DMA_ERROR
					    : (channel == (uint32_t)dw->rp_dma_tx_channel
						       ? RP_DMA_TX_DONE
						       : RP_DMA_RX_DONE));
}

/* A stop only requests an abort in Zephyr's RP2040 DMA driver.  Wait for BUSY,
 * then acknowledge late completions (RP2040-E13) before releasing channels.
 */
static bool i2c_dw_rp_dma_stop(int tx, int rx)
{
	int64_t deadline = rp_dma_uptime_ms() + 100;
	(void)nano_dma_stop(tx);
	(void)nano_dma_stop(rx);
	while (nano_dma_channel_is_busy(tx) || nano_dma_channel_is_busy(rx) ||
	       nano_dma_abort_pending()) {
		if (rp_dma_uptime_ms() >= deadline) {
			return false;
		}
		nxsched_msleep(1);
	}
	nano_dma_irqn_acknowledge_channel(0, tx);
	nano_dma_irqn_acknowledge_channel(1, tx);
	nano_dma_irqn_acknowledge_channel(0, rx);
	nano_dma_irqn_acknowledge_channel(1, rx);
	return true;
}

/* i2c_dw_rp_dma_read(dev, msg): msg->buf, msg->len and msg->flags (the
 * Zephyr flags of the message, see nano_i2c_dw_msg_flags()).
 */
static int i2c_dw_rp_dma_read(struct i2c_dw_dev_config *dw, uint8_t *buf, uint32_t len,
			      uint8_t msg_flags)
{
	mm_reg_t base = dw->base;
	struct nano_dma_block_config tx_block = {0}, rx_block = {0};
	struct nano_dma_config tx_cfg = {0}, rx_cfg = {0};
	int tx, rx, ret;
	bool started = false;

	/* dev != i2c0 cannot happen: i2c0 is the only instance.  10-bit
	 * addresses are refused before (no I2C_MSG_ADDR_10_BITS here, and
	 * app_config never has I2C_ADDR_10_BITS).
	 */
	if (!rp_dma_enabled || !nano_dma_is_ready() || len < 32 ||
	    len > I2C_DW_RP2040_DMA_MAX_READ || !(msg_flags & I2C_MSG_STOP) ||
	    (msg_flags & I2C_MSG_ADDR_10_BITS)) {
		return -EAGAIN;
	}
	tx = nano_dma_request_channel();
	if (tx < 0) {
		rp_dma_fallbacks++;
		return -EAGAIN;
	}
	rx = nano_dma_request_channel();
	if (rx < 0) {
		rp_dma_fallbacks++;
		nano_dma_release_channel(tx);
		return -EAGAIN;
	}
	dw->rp_dma_tx_channel = tx;
	dw->rp_dma_rx_channel = rx;
	i2c_dw_rp2040_read_commands(dw->rp_dma_commands, ARRAY_SIZE(dw->rp_dma_commands), len,
				    dw->xfr_flags & I2C_MSG_RESTART);
	tx_block.source_address = (uintptr_t)dw->rp_dma_commands;
	tx_block.dest_address = base + DW_IC_REG_DATA_CMD;
	tx_block.block_size = len * sizeof(uint16_t);
	tx_block.source_addr_adj = NANO_DMA_ADDR_ADJ_INCREMENT;
	tx_block.dest_addr_adj = NANO_DMA_ADDR_ADJ_NO_CHANGE;
	tx_cfg.dma_slot = NANO_DMA_SLOT_I2C0_TX;
	tx_cfg.channel_direction = NANO_DMA_MEMORY_TO_PERIPHERAL;
	tx_cfg.source_data_size = 2;
	tx_cfg.dest_data_size = 2;
	tx_cfg.block_count = 1;
	tx_cfg.head_block = &tx_block;
	tx_cfg.dma_callback = i2c_dw_rp_dma_callback;
	tx_cfg.user_data = dw;
	rx_block.source_address = base + DW_IC_REG_DATA_CMD;
	rx_block.dest_address = (uintptr_t)dw->rp_dma_buffer;
	rx_block.block_size = len;
	rx_block.source_addr_adj = NANO_DMA_ADDR_ADJ_NO_CHANGE;
	rx_block.dest_addr_adj = NANO_DMA_ADDR_ADJ_INCREMENT;
	rx_cfg.dma_slot = NANO_DMA_SLOT_I2C0_RX;
	rx_cfg.channel_direction = NANO_DMA_PERIPHERAL_TO_MEMORY;
	rx_cfg.source_data_size = 1;
	rx_cfg.dest_data_size = 1;
	rx_cfg.block_count = 1;
	rx_cfg.head_block = &rx_block;
	rx_cfg.dma_callback = i2c_dw_rp_dma_callback;
	rx_cfg.user_data = dw;
	ret = nano_dma_config(tx, &tx_cfg);
	if (!ret) {
		ret = nano_dma_config(rx, &rx_cfg);
	}
	if (ret) {
		rp_dma_fallbacks++;
		ret = -EAGAIN;
		goto release;
	}
	dw->rp_dma_events = 0;
	nxsem_reset(&dw->rp_dma_sem, 0);
	dw->rp_dma_active = 1;
	nano_dw_write32(8, base + 0x8c); /* TX FIFO threshold */
	nano_dw_write32(0, base + 0x90); /* RX FIFO threshold */
	write_intr_mask(DW_INTR_STAT_STOP_DET | DW_INTR_STAT_TX_ABRT | DW_INTR_STAT_TX_OVER |
				DW_INTR_STAT_RX_OVER | DW_INTR_STAT_RX_UNDER |
				DW_INTR_STAT_SCL_STUCK_LOW,
			base);
	started = true;
	ret = nano_dma_start(rx);
	if (!ret) {
		ret = nano_dma_start(tx);
	}
	if (!ret) {
		/* sys_timepoint_calc(rom->transfer_timeout) */
		clock_t deadline = clock_systime_ticks() +
				   MSEC2TICK(NANO_I2C_DW_TRANSFER_TIMEOUT_MS);
		write_dma_cr(3, base);
		while ((dw->rp_dma_events & RP_DMA_DONE) != RP_DMA_DONE) {
			int64_t left;

			if (dw->rp_dma_events & RP_DMA_ERROR) {
				ret = -EIO;
				break;
			}
			/* sys_timepoint_timeout(deadline): K_NO_WAIT once it
			 * has passed (a last try without waiting).
			 */
			left = (int64_t)(deadline - clock_systime_ticks());
			if (left < 0) {
				left = 0;
			}
			if (nxsem_tickwait_uninterruptible(&dw->rp_dma_sem, (uint32_t)left)) {
				ret = -ETIMEDOUT;
				break;
			}
		}
		if (dw->rp_dma_events & RP_DMA_ERROR) {
			ret = -EIO;
		}
	}
	write_dma_cr(0, base);
	write_intr_mask(0, base);
	dw->rp_dma_active = 0;
	if (ret) {
		set_bit_enable_abort(base);
	}
	if (ret && !i2c_dw_rp_dma_stop(tx, rx)) {
		dw->rp_dma_quarantined = true;
		rp_dma_errors++;
		LOG_ERR("DMA channels did not stop; I2C controller quarantined");
		return -EIO;
	}
	(void)read_clr_intr(base);
	if (ret) {
		int64_t abort_deadline = rp_dma_uptime_ms() + 100;
		while (test_bit_enable_abort(base)) {
			if (rp_dma_uptime_ms() >= abort_deadline) {
				dw->rp_dma_quarantined = true;
				LOG_ERR("I2C abort did not finish; controller quarantined");
				break;
			}
			nxsched_msleep(1);
		}
		(void)read_clr_intr(base);
	}
	if (!ret) {
		rp_dma_transfers++;
		rp_dma_bytes += len;
		memcpy(buf, dw->rp_dma_buffer, len);
		dw->xfr_len = 0;
	}
release:
	if (ret && ret != -EAGAIN) {
		rp_dma_errors++;
	}
	if (!started || !dw->rp_dma_quarantined) {
		nano_dma_release_channel(tx);
		nano_dma_release_channel(rx);
	}
	return ret;
}

/* The i2c_dma shell command's side (i2c_dw.c:770-801): nuttx/apps/i2c_dma
 * prints and parses; see nano_i2c_dma.h.
 */

void nano_i2c_dma_get_stats(struct nano_i2c_dma_stats *stats)
{
	stats->transfers = rp_dma_transfers;
	stats->bytes = rp_dma_bytes;
	stats->errors = rp_dma_errors;
	stats->fallbacks = rp_dma_fallbacks;
}

bool nano_i2c_dma_enabled(void)
{
	return rp_dma_enabled;
}

void nano_i2c_dma_set_enabled(bool enabled)
{
	rp_dma_enabled = enabled;
}
#endif

static int i2c_dw_isr(int irq, void *context, void *arg)
{
	struct i2c_dw_dev_config *const dw = arg;
	union ic_interrupt_register intr_stat;
	int ret = 0;
	mm_reg_t reg_base = dw->base;

	(void)irq;
	(void)context;

	/* Cache I2C Interrupt Status Register(ic_intr_stat) for processing,
	 * so there is no need to read the register multiple times.
	 */
	intr_stat.raw = read_intr_stat(reg_base);
#ifdef CONFIG_NANO_I2C0_DMA
	if (dw->rp_dma_active) {
		uint32_t events = 0;
		if (intr_stat.raw &
		    (DW_INTR_STAT_TX_ABRT | DW_INTR_STAT_TX_OVER | DW_INTR_STAT_RX_OVER |
		     DW_INTR_STAT_RX_UNDER | DW_INTR_STAT_SCL_STUCK_LOW)) {
			dw->state |= I2C_DW_CMD_ERROR;
			(void)i2c_dw_error_chk(dw);
			events |= RP_DMA_ERROR;
		}
		if (intr_stat.raw & DW_INTR_STAT_STOP_DET) {
			events |= RP_DMA_STOP;
		}
		write_intr_mask(0, reg_base);
		(void)read_clr_intr(reg_base);
		i2c_dw_rp_dma_notify(dw, events);
		return 0;
	}
#endif

	/*
	 * Causes of an interrupt:
	 *   - STOP condition is detected
	 *   - Transfer is aborted
	 *   - Transmit FIFO is empty
	 *   - Transmit FIFO has overflowed
	 *   - Receive FIFO is full
	 *   - Receive FIFO has overflowed
	 *   - Received FIFO has underrun
	 *   - Transmit data is required (tx_req)
	 *   - Receive data is available (rx_avail)
	 */

	/* Check if we are configured as a controller device */
	if (test_bit_con_master_mode(reg_base)) {
		/* Bail early if there is any error. */
		if ((DW_INTR_STAT_TX_ABRT | DW_INTR_STAT_TX_OVER | DW_INTR_STAT_RX_OVER |
		     DW_INTR_STAT_RX_UNDER | DW_INTR_STAT_SCL_STUCK_LOW) &
		    intr_stat.raw) {
			dw->state = I2C_DW_CMD_ERROR;
			i2c_dw_error_chk(dw);
			(void)read_clr_tx_abrt(reg_base);
			goto done;
		}

		/* Check if the RX FIFO reached threshold */
		if (intr_stat.bits.rx_full) {
			i2c_dw_data_read(dw);
			/* TX_EMPTY may be masked while waiting for the last RX batch.
			 * A no-STOP message must still complete when its data arrives.
			 */
			if (dw->xfr_len == 0U && !(dw->xfr_flags & I2C_MSG_STOP)) {
				goto done;
			}
		}

		if (intr_stat.bits.tx_empty) {
			if ((dw->xfr_flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
				ret = i2c_dw_data_send(dw);
			} else {
				i2c_dw_data_ask(dw);
			}

			/* If STOP is not expected, finish processing this
			 * message if there is nothing left to do anymore.
			 */
			if (((dw->xfr_len == 0U) && !(dw->xfr_flags & I2C_MSG_STOP)) ||
			    (ret != 0)) {
				goto done;
			}
		}

		/* STOP detected: finish processing this message */
		if (intr_stat.bits.stop_det) {
			/* Drain any final bytes even if the RX threshold was not met. */
			if ((dw->xfr_flags & I2C_MSG_RW_MASK) == I2C_MSG_READ &&
			    dw->xfr_len > 0U) {
				i2c_dw_data_read(dw);
			}
			(void)read_clr_stop_det(reg_base);
			goto done;
		}
	}

	return 0;

done:
	i2c_dw_transfer_complete(dw);
	return 0;
}

static int i2c_dw_setup(struct i2c_dw_dev_config *dw, uint16_t target_address)
{
	union ic_con_register ic_con;
	union ic_tar_register ic_tar;
	mm_reg_t reg_base = dw->base;

	ic_con.raw = 0U;

	/* Disable the device controller to be able set TAR */
	clear_bit_enable_en(reg_base);

	/* enable bus clear feature */
	ic_con.bits.bus_clear = 1U;
	/* enable tx empty control to fix timing issue */
	ic_con.bits.tx_empty_ctl = 1U;

	/* Disable interrupts */
	write_intr_mask(0, reg_base);

	/* Clear interrupts */
	(void)read_clr_intr(reg_base);

	/* Controller mode (the only one here): set both the master_mode and
	 * slave_disable bits.
	 */
	ic_con.bits.master_mode = 1U;
	ic_con.bits.slave_disable = 1U;

	ic_con.bits.restart_en = 1U;
#ifdef CONFIG_NANO_I2C0_DMA
	/* I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_BITS (bit 9) with the DMA reads
	 * enabled (i2c0 is the only instance here).
	 */
	if (rp_dma_enabled) {
		ic_con.raw |= BIT(9);
	}
#endif

	/* 7-bit addresses */
	ic_con.bits.addr_master_10bit = 0U;

	/* Setup the clock frequency and speed mode */
	switch (dw->speed) {
	case NANO_I2C_SPEED_STANDARD:
		write_ss_scl_lcnt(dw->lcnt, reg_base);
		write_ss_scl_hcnt(dw->hcnt, reg_base);
		ic_con.bits.speed = I2C_DW_SPEED_STANDARD;

		break;
	case NANO_I2C_SPEED_FAST:
	case NANO_I2C_SPEED_FAST_PLUS:
		write_fs_scl_lcnt(dw->lcnt, reg_base);
		write_fs_scl_hcnt(dw->hcnt, reg_base);
		ic_con.bits.speed = I2C_DW_SPEED_FAST;

		break;
	case NANO_I2C_SPEED_HIGH:
		if (!dw->support_hs_mode) {
			return -EINVAL;
		}

		write_hs_scl_lcnt(dw->lcnt, reg_base);
		write_hs_scl_hcnt(dw->hcnt, reg_base);
		ic_con.bits.speed = I2C_DW_SPEED_HIGH;

		break;
	default:
		return -EINVAL;
	}

	/* Set the IC_CON register */
	write_con(ic_con.raw, reg_base);

	/* Set RX fifo threshold level.
	 * Setting it to zero automatically triggers interrupt
	 * RX_FULL whenever there is data received.
	 */
	write_rx_tl(0, reg_base);

	/* Set TX fifo threshold level.
	 * TX_EMPTY interrupt is triggered only when the
	 * TX FIFO is truly empty. So that we can let
	 * the controller do the transfers for longer period
	 * before we need to fill the FIFO again. This may
	 * cause some pauses during transfers, but this keeps
	 * the device from interrupting often.
	 */
	write_tx_tl(0, reg_base);

	ic_tar.raw = read_tar(reg_base);

	/* Set address of target */
	ic_tar.bits.ic_tar = target_address;

	/* 7-bit addressing (I2C_DYNAMIC_TAR_UPDATE) */
	ic_tar.bits.ic_10bitaddr_master = 0U;
	write_tar(ic_tar.raw, reg_base);

	return 0;
}

static bool i2c_dw_is_busy(struct i2c_dw_dev_config *dw)
{
	mm_reg_t reg_base = dw->base;

	if (test_bit_status_activity(reg_base) || (dw->state & I2C_DW_BUSY)) {
		return true;
	}

	return false;
}

/* i2c_dw_configure() (i2c_dw.c:1385-1455) for a speed class.  Zephyr
 * stores the (minimum) counts even when the class is refused and keeps the
 * old class, so the next transfer runs the old class with them; here a
 * refused class changes nothing.
 */

static int i2c_dw_configure(struct i2c_dw_dev_config *dw, uint32_t speed)
{
	uint16_t lcnt_val;
	uint16_t hcnt_val;
	int rc;

	rc = nano_i2c_dw_scl_counts(speed, I2C_DW_CLOCK_SPEED, dw->support_hs_mode,
				    &hcnt_val, &lcnt_val);
	if (rc == 0) {
		dw->lcnt = lcnt_val;
		dw->hcnt = hcnt_val;
		dw->speed = speed;
	}

	/*
	 * Clear any interrupts currently waiting in the controller
	 */
	(void)read_clr_intr(dw->base);

	return rc;
}

static int i2c_dw_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count)
{
	struct i2c_dw_dev_config *const dw = (struct i2c_dw_dev_config *)dev;
	struct i2c_msg_s *cur_msg = msgs;
	int msg_left = count;
	uint16_t target_address;
	uint32_t speed;
	uint8_t pflags;
	uint8_t mflags;
	int ret;
	int i;
	mm_reg_t reg_base = dw->base;

	/* The checks of a NuttX request that the Zephyr API makes by its types:
	 * up to 255 messages, one 7-bit address and one speed per transfer.
	 */
	if (msgs == NULL || count < 0 || count > UINT8_MAX || !dw->ready) {
		return -EINVAL;
	}
	if (count == 0) {
		return 0;
	}
	target_address = msgs[0].addr;
	speed = nano_i2c_dw_speed(msgs[0].frequency);
	if (speed == 0 || target_address > 0x7f) {
		return -EINVAL;
	}
	for (i = 0; i < count; i++) {
		if (msgs[i].addr != target_address ||
		    msgs[i].frequency != msgs[0].frequency ||
		    (msgs[i].flags & I2C_M_TEN) || msgs[i].length < 0 ||
		    (msgs[i].length > 0 && msgs[i].buffer == NULL)) {
			return -EINVAL;
		}
	}

	/* semaphore to support I2C_CALLBACK */
	ret = nxsem_wait_uninterruptible(&dw->bus_sem);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_NANO_I2C0_DMA
	/* Also what i2c_dw_runtime_configure() returns (i2c_dw.c:1469-1473). */
	if (dw->rp_dma_quarantined) {
		ret = -EIO;
		goto error;
	}
#endif

	/* i2c_configure() with the speed of this transfer */
	if (speed != dw->speed) {
		ret = i2c_dw_configure(dw, speed);
		if (ret) {
			nxsem_post(&dw->bus_sem);
			return ret;
		}
	}

	/* check if the state is sda/scl stuck at low state */
	if (dw->state & I2C_DW_STUCK_ERR_MASK) {
		/* try to recovery */
		if (i2c_recovery_bus(dw)) {
			ret = -ETIME;
			goto error;
		}
		/* reset state */
		dw->state = I2C_DW_STATE_READY;
	}

	/* First step, check if there is current activity */
	if (i2c_dw_is_busy(dw)) {
		ret = -EBUSY;
		goto error;
	}

	dw->state |= I2C_DW_BUSY;

	ret = i2c_dw_setup(dw, target_address);
	if (ret) {
		goto error;
	}

	/* Enable controller */
	set_bit_enable_en(reg_base);

	/* Process all the messages */
	while (msg_left > 0) {
		i = count - msg_left;
		mflags = nano_i2c_dw_msg_flags(i > 0 ? msgs[i - 1].flags : 0, cur_msg->flags,
					       msg_left > 1 ? msgs[i + 1].flags : 0, i == 0,
					       msg_left == 1);

		pflags = dw->xfr_flags;

		dw->xfr_buf = cur_msg->buffer;
		dw->xfr_len = (uint32_t)cur_msg->length;
		/* Workaround for I2C scanner as DW HW does not support 0 byte transfers.*/
		if ((dw->xfr_len == 0) && (cur_msg->buffer != NULL)) {
			dw->xfr_len = 1;
		}
		dw->xfr_flags = mflags;
		dw->rx_pending = 0U;

		/* Need to RESTART if changing transfer direction */
		dw->xfr_flags = nano_i2c_dw_next_flags(pflags, dw->xfr_flags);

		dw->state &= ~(I2C_DW_CMD_SEND | I2C_DW_CMD_RECV);

		if ((dw->xfr_flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			dw->state |= I2C_DW_CMD_SEND;
			dw->request_bytes = 0U;
		} else {
			dw->state |= I2C_DW_CMD_RECV;
			dw->request_bytes = dw->xfr_len;
		}

		/* Short reads must not wait for a full RX batch. */
		if (test_bit_con_master_mode(reg_base) &&
		    (dw->xfr_flags & I2C_MSG_RW_MASK) == I2C_MSG_READ && dw->xfr_len > 0U) {
			write_rx_tl(MIN(RP2040_I2C_RX_BATCH, dw->xfr_len) - 1U, reg_base);
		}

#ifdef CONFIG_NANO_I2C0_DMA
		if (msg_left == 1 && (dw->xfr_flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			ret = i2c_dw_rp_dma_read(dw, cur_msg->buffer, (uint32_t)cur_msg->length,
						 mflags);
			if (ret != -EAGAIN) {
				if (ret) {
					break;
				}
				cur_msg++;
				msg_left--;
				continue;
			}
		}
#endif

		/* Enable necessary interrupts to trigger ISR */
		write_intr_mask((DW_ENABLE_TX_INT_I2C_CONTROLLER |
				 DW_ENABLE_RX_INT_I2C_CONTROLLER),
				reg_base);

		/* Wait for transfer to be done */
		ret = nxsem_tickwait_uninterruptible(&dw->device_sync_sem,
						     MSEC2TICK(NANO_I2C_DW_TRANSFER_TIMEOUT_MS));
		if (ret != 0) {
			/* Trigger abort and wait for it to complete. */
			set_bit_enable_abort(reg_base);
			(void)nxsem_tickwait_uninterruptible(&dw->device_sync_sem,
							     MSEC2TICK(1));
			write_intr_mask(DW_DISABLE_ALL_I2C_INT, reg_base);
			(void)read_clr_intr(reg_base);
			ret = -ETIMEDOUT;
			nxsem_reset(&dw->device_sync_sem, 0);
			break;
		}

		if (dw->state & I2C_DW_CMD_ERROR) {
			ret = -EIO;
			break;
		}

		/* Something wrong if there is something left to do */
		if (dw->xfr_len > 0) {
			ret = -EIO;
			break;
		}

		if (dw->state & I2C_DW_ERR_MASK) {
			if (dw->state & (I2C_DW_SDA_STUCK | I2C_DW_SCL_STUCK)) {
				ret = -ETIME;
			} else if (dw->state & (I2C_DW_NACK | I2C_DW_CMD_ERROR)) {
				ret = -EIO;
			}
			break;
		}

		cur_msg++;
		msg_left--;
	}

error:
	/* keep error mask for bus recovery */
	dw->state &= I2C_DW_STUCK_ERR_MASK;
	nxsem_post(&dw->bus_sem);

	return ret;
}

static int i2c_dw_init_config(struct i2c_dw_dev_config *dw)
{
	union ic_sdahold_register sda_hold;
	mm_reg_t reg_base = dw->base;

	clear_bit_enable_en(reg_base);

	/* Set up SDAHOLD timing register: no sda-hold property, so the value
	 * stays as it is (the reset value, 1).
	 */
	sda_hold.raw = read_sdahold(reg_base);
	write_sdahold(sda_hold.raw, reg_base);

	/*
	 * depending on the IP configuration, we may have to disable block mode in
	 * controller mode
	 */
	clear_bit_enable_block(reg_base);

	/* Set spike length */
	write_fs_spklen(NANO_I2C_DW_FS_SPK_LEN, reg_base);
	write_hs_spklen(NANO_I2C_DW_HS_SPK_LEN, reg_base);

	/* clock-frequency = <I2C_BITRATE_STANDARD> in the board devicetree */
	if (i2c_dw_configure(dw, NANO_I2C_SPEED_STANDARD) != 0) {
		return -EIO;
	}

	dw->state = I2C_DW_STATE_READY;

	return 0;
}

/* pinctrl_configure_pin() of Zephyr's RP2040 pinctrl
 * (zephyr/drivers/pinctrl/pinctrl_rpi_pico.c) for the board's i2c0_default
 * group (arduino_nano_connect-pinctrl.dtsi: I2C0_SDA_P12, I2C0_SCL_P13,
 * input-enable, input-schmitt-enable; no pulls, 4 mA, slow slew, no
 * overrides: set_function() writes the whole CTRL register).
 */

static void i2c_dw_pinctrl(uint32_t gpio)
{
	rp2040_gpio_init(gpio);
	rp2040_gpio_set_function(gpio, RP2040_GPIO_FUNC_I2C);
	rp2040_gpio_set_pulls(gpio, false, false);
	rp2040_gpio_set_drive_strength(gpio, RP2040_PADS_BANK0_GPIO_DRIVE_4MA);
	rp2040_gpio_set_slew_fast(gpio, false);
	rp2040_gpio_set_input_hysteresis_enabled(gpio, true);
	modbits_reg32(RP2040_PADS_BANK0_GPIO_IE, RP2040_PADS_BANK0_GPIO_IE,
		      RP2040_PADS_BANK0_GPIO(gpio));
}

static int i2c_dw_prepare(struct i2c_dw_dev_config *dw)
{
	(void)dw;

	/* clk_peri runs from boot (rp2040_clockconfig()). */

	/* reset_line_toggle_dt(): reset_block_num(), then
	 * unreset_block_num_wait_blocking().
	 */
	setbits_reg32(RP2040_RESETS_RESET_I2C0, RP2040_RESETS_RESET);
	clrbits_reg32(RP2040_RESETS_RESET_I2C0, RP2040_RESETS_RESET);
	while ((getreg32(RP2040_RESETS_RESET_DONE) & RP2040_RESETS_RESET_I2C0) == 0) {
	}

	i2c_dw_pinctrl(BOARD_I2C0_SDA_GPIO);
	i2c_dw_pinctrl(BOARD_I2C0_SCL_GPIO);

	return 0;
}

static int i2c_dw_probe_hw(struct i2c_dw_dev_config *dw)
{
	union ic_con_register ic_con;
	mm_reg_t reg_base = dw->base;

	if (read_comp_type(reg_base) != I2C_DW_MAGIC_KEY) {
		return -EIO;
	}

	/*
	 * Grab the default value on initialization. This should be set to the
	 * IC_MAX_SPEED_MODE in the hardware. If it does support high speed we
	 * can provide support for it.
	 */
	ic_con.raw = read_con(reg_base);
	if (ic_con.bits.speed == I2C_DW_SPEED_HIGH) {
		dw->support_hs_mode = true;
	} else {
		dw->support_hs_mode = false;
	}

	return 0;
}

static int i2c_dw_turn_on(struct i2c_dw_dev_config *dw)
{
	int ret;

	/* Configure clock, optional reset, and pinctrl */
	ret = i2c_dw_prepare(dw);
	if (ret < 0) {
		return ret;
	}

	/* Check if the hardware is supported and set the support_hs_mode flag */
	ret = i2c_dw_probe_hw(dw);
	if (ret < 0) {
		return ret;
	}

	/* Initialize the controller registers */
	ret = i2c_dw_init_config(dw);
	if (ret < 0) {
		return ret;
	}

	/* Configure IRQ */
	irq_attach(RP2040_I2C0_IRQ, i2c_dw_isr, dw);
	up_enable_irq(RP2040_I2C0_IRQ);

	return 0;
}

struct i2c_master_s *nano_i2c0_initialize(void)
{
	struct i2c_dw_dev_config *const dw = &i2c_0_runtime;

	if (dw->ready) {
		return &dw->dev;
	}

	dw->dev.ops = &funcs;
	dw->base = RP2040_I2C0_BASE;
	nxsem_init(&dw->device_sync_sem, 0, 0);
	nxsem_init(&dw->bus_sem, 0, 1);
#ifdef CONFIG_NANO_I2C0_DMA
	nxsem_init(&dw->rp_dma_sem, 0, 0);
#endif

	if (i2c_dw_turn_on(dw) < 0) {
		return NULL;
	}

	dw->ready = true;
	return &dw->dev;
}
