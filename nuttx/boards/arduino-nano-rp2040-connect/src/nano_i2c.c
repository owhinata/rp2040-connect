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
 * and other instances than i2c0.  The DMA reads of i2c0
 * (CONFIG_I2C_DW_RP2040_DMA) are not here yet; IC_CON gets the RX FIFO
 * full hold bit as with them enabled (their default), see i2c_dw_setup().
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

LOG_MODULE_REGISTER(i2c_dw);

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
};

/* rp_dma_enabled of the Zephyr build (i2c_dw.c:581): on unless the
 * i2c_dma command turns it off.
 */

static bool rp_dma_enabled = true;

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

static struct i2c_dw_dev_config i2c_0_runtime = {
	.dev = {.ops = &funcs},
	.base = RP2040_I2C0_BASE,
};

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
	/* RX_FIFO_FULL_HLD_CTRL (IC_CON bit 9) with the DMA reads enabled. */
	if (rp_dma_enabled) {
		ic_con.raw |= BIT(9);
	}

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

	nxsem_init(&dw->device_sync_sem, 0, 0);
	nxsem_init(&dw->bus_sem, 0, 1);

	if (i2c_dw_turn_on(dw) < 0) {
		return NULL;
	}

	dw->ready = true;
	return &dw->dev;
}
