/* i2c_dw.c - I2C file for Design Ware */

/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Andrei-Edward Popa
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* I2C0 and I2C1 of the RP2040 (DesignWare DW_apb_i2c blocks): a port of the
 * controller path of the Zephyr build's driver, Zephyr's
 * drivers/i2c/i2c_dw.c with the local patch
 * 0003-i2c-dw-rp2040-fifo-and-dma.patch (zephyr/manifest/zephyr/patches/
 * zephyr/: the RP2040 changes and the DMA reads, i2c_dw_rp2040.h and
 * i2c_dw_rp2040_dma_priv.h; a file name with a line number, as
 * i2c_dw.c:55, is one of those patched files), with the same functions,
 * branches, order, register values, timeout and error values.  The NuttX
 * build uses it in place of NuttX's own arch/arm/src/rp2040/rp2040_i2c.c
 * (CONFIG_RP2040_I2C off).
 * common/tests/i2c_dw_sync.py checks that the two stay the same, but for
 * the differences listed here.
 *
 * The files follow Zephyr's: this file (i2c_dw.c) and, in internal/,
 * i2c_dw.h (Zephyr's definitions, and the pure functions that the host
 * tests check), i2c_dw_registers.h (Zephyr's register map), i2c_dw_rp2040.h
 * (the RP2040's FIFO depth and RX batch) and i2c_dw_rp2040_dma_priv.h (the
 * DMA reads, included once below, on the DMA boundary port/i2c_dw_dma_os.h;
 * the "i2c_dma" state they define is declared in
 * include/i2c_dw_rp2040_dma.h).  Zephyr's address type mm_reg_t is uintptr_t here
 * (the same type on Arm).
 *
 * This is the OS-independent core.  The OS's part is in port/i2c_dw_os.h
 * (semaphores, clock, sleeps, critical section, the form of the messages:
 * i2c_dw_os_impl.h of the OS's binding, port/<OS>) and in
 * i2c_dw_instance.h of the binding, read at the end of this file, which defines
 * i2c_dw_os_prepare() and i2c_dw_os_irq_enable() (below) and makes the
 * device of the OS's driver model for each instance: it initializes the
 * semaphores and dw->base of i2c_0_runtime or i2c_1_runtime, calls
 * i2c_dw_turn_on() once and then sets dw->ready, and calls
 * i2c_dw_transfer_msgs() for a transfer and i2c_dw_isr_handler() from the
 * controller's interrupt.
 *
 *   Zephyr                                here
 *   I2C_DEVICE_INIT_DW(n) for each        CONFIG_I2C_DW_I2C0 and
 *   enabled node (i2c0, i2c1)             CONFIG_I2C_DW_I2C1 of the OS's
 *                                         build (at least one): i2c_0_runtime
 *                                         and i2c_1_runtime (below)
 *   struct device, dev->data, dev->config struct i2c_dw_dev_config, the
 *                                         constants of i2c_dw.h (the same
 *                                         for both nodes)
 *   dev->name, dev == DEVICE_DT_GET(      i2c_dw_name(dw), i2c_dw_is_i2c0(dw)
 *   DT_NODELABEL(i2c0))                   (I2C_DW_INST(), below)
 *   i2c_transfer() (msgs, num, addr)      i2c_dw_transfer_msgs() (the OS's
 *                                         messages, I2C_DW_OS_MSG_*())
 *   i2c_configure() (speed class)         the speed of each transfer,
 *                                         applied by i2c_dw_transfer_msgs()
 *   k_sem, k_sem_*(), k_uptime_get(),     i2c_dw_os_*() (i2c_dw_os.h)
 *   k_msleep(), atomic_or()
 *   IRQ_CONNECT(), irq_enable()           i2c_dw_os_irq_enable()
 *   reset_line_toggle_dt(),               i2c_dw_os_prepare()
 *   pinctrl_apply_state()
 *   LOG_*() of module i2c_dw at level 3   the same (app_log.h)
 *
 * Left out (not built in the Zephyr build or not used by it): target
 * (slave) mode, the Intel LPSS DMA, CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS
 * (need_setup), CONFIG_I2C_DW_EXTENDED_SUPPORT (SDA/SCL timeouts), power
 * management, PCIe, the RTS5912 quirk, the recover/check-bus callbacks
 * (only wrappers of other SoCs register them; without one the recovery
 * does nothing and returns 0), and the never-set i2c_stat_not_ready
 * counter.  The calls of i2c_dw_rp2040_controller() (i2c_dw_rp2040.h) are
 * left out too: both instances are RP2040 controllers, so the check is
 * always true here.
 *
 * The DMA reads of i2c0 (CONFIG_I2C_DW_RP2040_DMA of the Zephyr build,
 * i2c_dw_rp2040_dma_priv.h, i2c_dw.c:55-57, 566-587, 795-799, 941-946,
 * 1040-1052, 1219-1223) are CONFIG_I2C_DW_RP2040_DMA here too, with the same
 * conditions, command words, thresholds, interrupt mask, start order,
 * completion and timeout, clean-up and stop sequence, quarantine, counters
 * and log lines, on the OS's own DMA driver through the boundary of
 * i2c_dw_dma_os.h in place of Zephyr's DMA API (one call for each of
 * Zephyr's, the channel's setup as two calls, for the TX and the RX side, in
 * place of the dma_config() structures; the stop waits for the two
 * channels' own aborts, not for any channel's).  atomic_t becomes a volatile word with the read-modify-write under
 * i2c_dw_os_irq_lock() where an interrupt also writes it
 * (rp2040_dma_events); the counters are only written by the transfer,
 * which holds bus_sem.  As in the patched driver, the DMA reads are
 * i2c0's only (i2c1 always takes the interrupt path, and its IC_CON never
 * gets RX_FIFO_FULL_HLD_CTRL), the read's semaphore, events, channels,
 * command words and buffer are static variables for i2c0
 * (i2c_dw_rp2040_dma_priv.h:57-63), and rp2040_dma_active and
 * rp2040_dma_quarantined are members of struct i2c_dw_dev_config
 * (i2c_dw.h:194-197).  CONFIG_I2C_DW_RP2040_DMA needs CONFIG_I2C_DW_I2C0.  The semaphore is an i2c_dw_os_sem_t, which the OS's
 * part (i2c_dw_instance.h) initializes to 0 with the controller's
 * semaphores, in place of K_SEM_DEFINE().
 *
 * A transfer is the Zephyr one with the speed class as an argument (the
 * OS's part keeps what i2c_configure() set, or takes it from the
 * messages) and 7-bit addresses only (the OS's part refuses others).
 */

#include "i2c_dw_os.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* CONFIG_I2C_LOG_LEVEL of the Zephyr build (3, info). */

#define LOG_LEVEL 3
#include "app_log.h"

#include "i2c_dw.h"
#include "i2c_dw_registers.h"

LOG_MODULE_REGISTER(i2c_dw);

#include "i2c_dw_rp2040.h"

#if !defined(CONFIG_I2C_DW_I2C0) && !defined(CONFIG_I2C_DW_I2C1)
#error "i2c_dw.c: no instance (CONFIG_I2C_DW_I2C0, CONFIG_I2C_DW_I2C1)"
#endif
#if defined(CONFIG_I2C_DW_RP2040_DMA) && !defined(CONFIG_I2C_DW_I2C0)
#error "i2c_dw.c: the DMA reads (CONFIG_I2C_DW_RP2040_DMA) are i2c0's"
#endif

/* The devicetree names of the Zephyr build's controllers (dev->name). */

#define I2C_DW_I2C0_NAME "i2c@40044000"
#define I2C_DW_I2C1_NAME "i2c@40048000"

struct i2c_dw_dev_config {
	I2C_DW_OS_DEV_HEAD
	uintptr_t base;
	i2c_dw_os_sem_t device_sync_sem;
	i2c_dw_os_sem_t bus_sem;
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

#ifdef CONFIG_I2C_DW_RP2040_DMA
	/* i2c_dw.h:194-197 */
	volatile uint32_t rp2040_dma_active;
	bool rp2040_dma_quarantined;
#endif
};

/* The instances, in .bss as Zephyr's i2c_##n##_runtime of
 * I2C_DEVICE_INIT_DW(n) (i2c_dw.c:1772); the OS's part (i2c_dw_instance.h)
 * sets them up.
 */

#ifdef CONFIG_I2C_DW_I2C0
static struct i2c_dw_dev_config i2c_0_runtime;
#endif
#ifdef CONFIG_I2C_DW_I2C1
static struct i2c_dw_dev_config i2c_1_runtime;
#endif

/* I2C_DW_INST(dw, v0, v1): v0 for i2c0's instance, v1 for i2c1's, as
 * Zephyr's per-instance values (DT_INST_*(n), dev->name).  With only one
 * instance built, that one's value without a test (so a build of one
 * instance compiles as if the value were written in).  For the OS's part
 * too (its pins, reset, interrupt and base address).
 */

#if defined(CONFIG_I2C_DW_I2C0) && defined(CONFIG_I2C_DW_I2C1)
#define I2C_DW_INST(dw, v0, v1) ((dw) == &i2c_0_runtime ? (v0) : (v1))
#elif defined(CONFIG_I2C_DW_I2C0)
#define I2C_DW_INST(dw, v0, v1) (v0)
#else
#define I2C_DW_INST(dw, v0, v1) (v1)
#endif

/* dev->name and dev == DEVICE_DT_GET(DT_NODELABEL(i2c0)) */

#define i2c_dw_name(dw)    I2C_DW_INST(dw, I2C_DW_I2C0_NAME, I2C_DW_I2C1_NAME)
#define i2c_dw_is_i2c0(dw) I2C_DW_INST(dw, true, false)

/* No recover-bus callback is registered (i2c_dw.c:84-98). */

static int i2c_recovery_bus(struct i2c_dw_dev_config *dw)
{
	(void)dw;
	return 0;
}

static int i2c_dw_error_chk(struct i2c_dw_dev_config *dw)
{
	uintptr_t reg_base = dw->base;
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
			LOG_ERR_RATELIMIT("NACK on %s", i2c_dw_name(dw));
		}
		/* check SDA stuck low Tx abort, need to do bus recover */
		if (ic_txabrt_src.bits.SDASTUCKLOW) {
			dw->state |= I2C_DW_SDA_STUCK;
			LOG_ERR("SDA Stuck Low on %s", i2c_dw_name(dw));
		}
		/* check if user abort the transmit */
		if (ic_txabrt_src.bits.USRABRT) {
			dw->state |= I2C_DW_USER_ABRT;
			LOG_ERR("User Abort on %s", i2c_dw_name(dw));
		}
		/* check if user abort the transmit */
		if (ic_txabrt_src.bits.ARBLOST) {
			dw->state |= I2C_DW_USER_ABRT;
			LOG_ERR("ARB lost on %s", i2c_dw_name(dw));
		}
		/* TX abrt because STOP */
		if (intr_stat.bits.stop_det) {
			dw->state |= I2C_DW_USER_ABRT;
			LOG_ERR_RATELIMIT("ABR and STOP on %s", i2c_dw_name(dw));
		}
	}

	/* check SCL stuck low */
	if (intr_stat.bits.scl_stuck_low) {
		dw->state |= I2C_DW_SCL_STUCK;
		LOG_ERR("SCL Stuck Low on %s", i2c_dw_name(dw));
	}
	if (dw->state & I2C_DW_ERR_MASK) {
		LOG_ERR_RATELIMIT("IO Fail on %s", i2c_dw_name(dw));
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
	uintptr_t reg_base = dw->base;

	/* No more bytes to request, so command queue is no longer needed */
	if (dw->request_bytes == 0U) {
		clear_bit_intr_mask_tx_empty(reg_base);
		return;
	}

	/* RP2040 does not implement IC_COMP_PARAM_1 (i2c_dw_rp2040.h:20-23, i2c_dw.c:363-370). */
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
		data = i2c_dw_read_cmd(&dw->xfr_flags, dw->request_bytes);

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
	uintptr_t reg_base = dw->base;

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
	uintptr_t reg_base = dw->base;

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
		data = i2c_dw_write_cmd(&dw->xfr_flags, dw->xfr_buf[0],
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
	uintptr_t reg_base = dw->base;

	write_intr_mask(DW_DISABLE_ALL_I2C_INT, reg_base);
	(void)read_clr_intr(reg_base);
	i2c_dw_os_sem_give(&dw->device_sync_sem);
}

#ifdef CONFIG_I2C_DW_RP2040_DMA
#include "i2c_dw_rp2040_dma_priv.h"
#endif

/* i2c_dw_isr(): the body of the controller's interrupt handler, which the
 * OS's handler (i2c_dw_instance.h) calls with the instance.  Interrupt.
 */

static inline void i2c_dw_isr_handler(struct i2c_dw_dev_config *const dw)
{
	union ic_interrupt_register intr_stat;
	int ret = 0;
	uintptr_t reg_base = dw->base;

	/* Cache I2C Interrupt Status Register(ic_intr_stat) for processing,
	 * so there is no need to read the register multiple times.
	 */
	intr_stat.raw = read_intr_stat(reg_base);
#ifdef CONFIG_I2C_DW_RP2040_DMA
	if (dw->rp2040_dma_active) {
		uint32_t events = 0;
		if (intr_stat.raw &
		    (DW_INTR_STAT_TX_ABRT | DW_INTR_STAT_TX_OVER | DW_INTR_STAT_RX_OVER |
		     DW_INTR_STAT_RX_UNDER | DW_INTR_STAT_SCL_STUCK_LOW)) {
			dw->state |= I2C_DW_CMD_ERROR;
			(void)i2c_dw_error_chk(dw);
			events |= RP2040_DMA_ERROR;
		}
		if (intr_stat.raw & DW_INTR_STAT_STOP_DET) {
			events |= RP2040_DMA_STOP;
		}
		write_intr_mask(0, reg_base);
		(void)read_clr_intr(reg_base);
		i2c_dw_rp2040_dma_notify(events);
		return;
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

	return;

done:
	i2c_dw_transfer_complete(dw);
}

static int i2c_dw_setup(struct i2c_dw_dev_config *dw, uint16_t target_address)
{
	union ic_con_register ic_con;
	union ic_tar_register ic_tar;
	uintptr_t reg_base = dw->base;

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
#ifdef CONFIG_I2C_DW_RP2040_DMA
	/* I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_BITS (bit 9) with the DMA reads
	 * enabled, on i2c0.
	 */
	if (i2c_dw_is_i2c0(dw) && rp2040_dma_enabled) {
		ic_con.raw |= BIT(9);
	}
#endif

	/* 7-bit addresses */
	ic_con.bits.addr_master_10bit = 0U;

	/* Setup the clock frequency and speed mode */
	switch (dw->speed) {
	case I2C_SPEED_STANDARD:
		write_ss_scl_lcnt(dw->lcnt, reg_base);
		write_ss_scl_hcnt(dw->hcnt, reg_base);
		ic_con.bits.speed = I2C_DW_SPEED_STANDARD;

		break;
	case I2C_SPEED_FAST:
	case I2C_SPEED_FAST_PLUS:
		write_fs_scl_lcnt(dw->lcnt, reg_base);
		write_fs_scl_hcnt(dw->hcnt, reg_base);
		ic_con.bits.speed = I2C_DW_SPEED_FAST;

		break;
	case I2C_SPEED_HIGH:
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
	uintptr_t reg_base = dw->base;

	if (test_bit_status_activity(reg_base) || (dw->state & I2C_DW_BUSY)) {
		return true;
	}

	return false;
}

/* i2c_dw_configure() (i2c_dw.c:1135-1205) for a speed class.  Zephyr
 * stores the (minimum) counts even when the class is refused and keeps the
 * old class, so the next transfer runs the old class with them; here a
 * refused class changes nothing.
 */

static int i2c_dw_configure(struct i2c_dw_dev_config *dw, uint32_t speed)
{
	uint16_t lcnt_val;
	uint16_t hcnt_val;
	int rc;

	rc = i2c_dw_scl_counts(speed, I2C_DW_CLOCK_SPEED, dw->support_hs_mode,
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

/* i2c_dw_transfer(dev, msgs, count, target_address) for a request that
 * the OS's part has checked: 1 to 255 messages (count), a 7-bit address,
 * a buffer for every message that is not empty, and speed, the Zephyr
 * speed class of the transfer (one that differs from the last runs
 * i2c_configure() first).  The OS's transfer entry (i2c_dw_instance.h)
 * calls it; it returns 0 or a negative errno.
 */

static inline int i2c_dw_transfer_msgs(struct i2c_dw_dev_config *const dw,
				       i2c_dw_os_msg_t *msgs, int count,
				       uint16_t target_address,
				       const volatile uint32_t *speed_of_transfer)
{
	i2c_dw_os_msg_t *cur_msg = msgs;
	uint32_t speed;
	int msg_left = count;
	uint8_t pflags;
	uint8_t mflags;
	int ret;
	int i;
	uintptr_t reg_base = dw->base;

	/* semaphore to support I2C_CALLBACK */
	ret = i2c_dw_os_bus_lock(&dw->bus_sem);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_I2C_DW_RP2040_DMA
	/* Also what i2c_dw_runtime_configure() returns (i2c_dw.c:1219-1223). */
	if (dw->rp2040_dma_quarantined) {
		ret = -EIO;
		goto error;
	}
#endif

	/* i2c_configure() with the speed of this transfer, read under the
	 * bus lock: a speed that another task sets while this one waits for
	 * the lock is the one this transfer runs at.
	 */
	speed = *speed_of_transfer;
	if (speed != dw->speed) {
		ret = i2c_dw_configure(dw, speed);
		if (ret) {
			i2c_dw_os_bus_unlock(&dw->bus_sem);
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
		mflags = I2C_DW_OS_MSG_FLAGS(msgs, cur_msg, i, msg_left);

		pflags = dw->xfr_flags;

		dw->xfr_buf = I2C_DW_OS_MSG_BUF(cur_msg);
		dw->xfr_len = I2C_DW_OS_MSG_LEN(cur_msg);
		/* Workaround for I2C scanner as DW HW does not support 0 byte transfers.*/
		if ((dw->xfr_len == 0) && (I2C_DW_OS_MSG_BUF(cur_msg) != NULL)) {
			dw->xfr_len = 1;
		}
		dw->xfr_flags = mflags;
		dw->rx_pending = 0U;

		/* Need to RESTART if changing transfer direction */
		dw->xfr_flags = i2c_dw_next_flags(pflags, dw->xfr_flags);

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

#ifdef CONFIG_I2C_DW_RP2040_DMA
		if (msg_left == 1 && (dw->xfr_flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			ret = i2c_dw_rp2040_dma_read(dw, I2C_DW_OS_MSG_BUF(cur_msg),
						     I2C_DW_OS_MSG_LEN(cur_msg), mflags);
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
		ret = i2c_dw_os_sem_take_ms(&dw->device_sync_sem,
					    I2C_DW_TRANSFER_TIMEOUT_MS);
		if (ret != 0) {
			/* Trigger abort and wait for it to complete. */
			set_bit_enable_abort(reg_base);
			(void)i2c_dw_os_sem_take_ms(&dw->device_sync_sem, 1);
			write_intr_mask(DW_DISABLE_ALL_I2C_INT, reg_base);
			(void)read_clr_intr(reg_base);
			ret = -ETIMEDOUT;
			i2c_dw_os_sem_reset(&dw->device_sync_sem);
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
	i2c_dw_os_bus_unlock(&dw->bus_sem);

	return ret;
}

static int i2c_dw_init_config(struct i2c_dw_dev_config *dw)
{
	union ic_sdahold_register sda_hold;
	uintptr_t reg_base = dw->base;

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
	write_fs_spklen(I2C_DW_FS_SPK_LEN, reg_base);
	write_hs_spklen(I2C_DW_HS_SPK_LEN, reg_base);

	/* clock-frequency = <I2C_BITRATE_STANDARD> in the board devicetree */
	if (i2c_dw_configure(dw, I2C_SPEED_STANDARD) != 0) {
		return -EIO;
	}

	dw->state = I2C_DW_STATE_READY;

	return 0;
}

/* The OS's part (i2c_dw_instance.h). */

/* The I2C block's clock, reset_line_toggle_dt() (reset_block_num(), then
 * unreset_block_num_wait_blocking()) and pinctrl_apply_state() of the
 * instance's pins as Zephyr's pinctrl_rpi_pico.c sets them (i2c0: the
 * board's i2c0_default group, GPIO12 SDA, GPIO13 SCL: function I2C, no
 * pulls, 4 mA, slow slew, input and Schmitt trigger enabled; i2c1: the OS's
 * pins, the same with pull-ups, i2c_dw_os.h).  0 or a negative errno.
 */

static int i2c_dw_os_prepare(struct i2c_dw_dev_config *dw);

/* IRQ_CONNECT() and irq_enable() of the instance's interrupt (I2C0_IRQ,
 * I2C1_IRQ): a handler that calls i2c_dw_isr_handler(dw), then enabled.
 * 0, or a negative errno when the handler could not be connected: the
 * interrupt stays disabled and the instance is not ready.
 */

static int i2c_dw_os_irq_enable(struct i2c_dw_dev_config *dw);

static int i2c_dw_probe_hw(struct i2c_dw_dev_config *dw)
{
	union ic_con_register ic_con;
	uintptr_t reg_base = dw->base;

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
	ret = i2c_dw_os_prepare(dw);
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
	return i2c_dw_os_irq_enable(dw);
}

/* The OS's part: the functions declared above, the OS's interrupt handler
 * and transfer entry, and the device.
 */

#include "i2c_dw_instance.h"
