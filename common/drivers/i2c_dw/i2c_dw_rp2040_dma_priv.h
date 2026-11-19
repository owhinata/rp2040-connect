/*
 * SPDX-License-Identifier: Apache-2.0
 */

/* CONFIG_I2C_DW_RP2040_DMA: the DMA reads of i2c0, the part of i2c_dw.c that
 * corresponds to the Zephyr build's drivers/i2c/i2c_dw_rp2040_dma_priv.h
 * (local patch 0003; i2c_dw.c lists the differences): the events, the
 * counters, the static state of the read in progress, the command words
 * (i2c_dw_rp2040_read_commands()), i2c_dw_rp2040_dma_notify(), _callback(),
 * _stop() and _read() on the OS's DMA through i2c_dw_dma_os.h (in place of
 * Zephyr's DMA API), and the functions of the "i2c_dma" command
 * (i2c_dw_rp2040_dma_get_stats(), _enabled(), _set_enabled()).
 *
 * Included once, by i2c_dw.c, with CONFIG_I2C_DW_RP2040_DMA: these are
 * i2c_dw.c's own static functions and variables, on its register accessors
 * and struct i2c_dw_dev_config, and i2c_dw_isr_handler(), i2c_dw_setup()
 * and i2c_dw_transfer_msgs() use them.  Zephyr's i2c_dw.c includes its file
 * after its declarations (struct i2c_dw_dev_config is in Zephyr's i2c_dw.h);
 * here the structure holds the OS's types and is defined in i2c_dw.c, so
 * the file is included after it, where the functions were before
 * (after i2c_dw_transfer_complete()).
 */

#ifndef COMMON_DRIVERS_I2C_DW_I2C_DW_RP2040_DMA_PRIV_H_
#define COMMON_DRIVERS_I2C_DW_I2C_DW_RP2040_DMA_PRIV_H_

#include "i2c_dw_rp2040_dma.h"

#include "i2c_dw_dma_os.h"

/* i2c_dw_rp2040_dma_priv.h:46: the longest read the DMA path takes (its
 * buffers are this long).
 */

#define I2C_DW_RP2040_DMA_MAX_READ (512U * 7U)

#define RP2040_DMA_TX_DONE BIT(0)
#define RP2040_DMA_RX_DONE BIT(1)
#define RP2040_DMA_STOP    BIT(2)
#define RP2040_DMA_ERROR   BIT(3)
#define RP2040_DMA_DONE    (RP2040_DMA_TX_DONE | RP2040_DMA_RX_DONE | RP2040_DMA_STOP)

static volatile uint32_t rp2040_dma_transfers, rp2040_dma_bytes, rp2040_dma_errors,
	rp2040_dma_fallbacks;
static volatile bool rp2040_dma_enabled = true;

/* The read in progress on i2c0 (i2c_dw_rp2040_dma_read()).  rp2040_dma_sem is
 * K_SEM_DEFINE(rp2040_dma_sem, 0, K_SEM_MAX_LIMIT): the OS's part
 * (i2c_dw_instance.h) initializes it to 0 before i2c_dw_turn_on().
 */
static i2c_dw_os_sem_t rp2040_dma_sem;
static volatile uint32_t rp2040_dma_events;
static int rp2040_dma_tx_channel;
static int rp2040_dma_rx_channel;
static uint16_t rp2040_dma_commands[I2C_DW_RP2040_DMA_MAX_READ];
static uint8_t rp2040_dma_buffer[I2C_DW_RP2040_DMA_MAX_READ];

/* The IC_DATA_CMD words of a read (one read command per byte, RESTART on the
 * first when asked, STOP on the last). The Zephyr build's patched driver has
 * the same function in its drivers/i2c/i2c_dw_rp2040_dma_priv.h;
 * common/tests/i2c_dw_sync.py checks that the two stay the same.
 */
#define RP2040_I2C_CMD_READ    (1U << 8)
#define RP2040_I2C_CMD_STOP    (1U << 9)
#define RP2040_I2C_CMD_RESTART (1U << 10)
static int i2c_dw_rp2040_read_commands(uint16_t *commands, size_t capacity, size_t length,
				       bool restart)
{
	if (!commands || !length || length > capacity) {
		return -EINVAL;
	}
	for (size_t i = 0; i < length; i++) {
		commands[i] = RP2040_I2C_CMD_READ;
	}
	if (restart) {
		commands[0] |= RP2040_I2C_CMD_RESTART;
	}
	commands[length - 1] |= RP2040_I2C_CMD_STOP;
	return 0;
}

/* atomic_or(): returns the old value. */

static uint32_t rp2040_dma_atomic_or(volatile uint32_t *target, uint32_t value)
{
	i2c_dw_os_irqstate_t flags = i2c_dw_os_irq_lock();
	uint32_t old = *target;

	*target = old | value;
	i2c_dw_os_irq_unlock(flags);
	return old;
}

/* k_uptime_get(): milliseconds since boot. */

static int64_t rp2040_dma_uptime_ms(void)
{
	return i2c_dw_os_uptime_ms();
}

static void i2c_dw_rp2040_dma_notify(uint32_t events)
{
	uint32_t all = rp2040_dma_atomic_or(&rp2040_dma_events, events) | events;
	if ((all & RP2040_DMA_ERROR) || (all & RP2040_DMA_DONE) == RP2040_DMA_DONE) {
		i2c_dw_os_sem_give(&rp2040_dma_sem);
	}
}

static void i2c_dw_rp2040_dma_callback(void *user_data, uint32_t channel, int status)
{
	struct i2c_dw_dev_config *dw = user_data;
	if (!dw->rp2040_dma_active) {
		return;
	}
	i2c_dw_rp2040_dma_notify(status < 0 ? RP2040_DMA_ERROR
					    : (channel == (uint32_t)rp2040_dma_tx_channel
						       ? RP2040_DMA_TX_DONE
						       : RP2040_DMA_RX_DONE));
}

/* A stop only requests an abort (i2c_dw_dma_os_stop()).  Wait for BUSY and
 * the channels' aborts, then acknowledge late completions (RP2040-E13)
 * before releasing channels.
 */
static bool i2c_dw_rp2040_dma_stop(int tx, int rx)
{
	int64_t deadline = rp2040_dma_uptime_ms() + 100;
	(void)i2c_dw_dma_os_stop(tx);
	(void)i2c_dw_dma_os_stop(rx);
	while (i2c_dw_dma_os_is_busy(tx) || i2c_dw_dma_os_is_busy(rx)) {
		if (rp2040_dma_uptime_ms() >= deadline) {
			return false;
		}
		i2c_dw_os_msleep(1);
	}
	i2c_dw_dma_os_acknowledge(tx);
	i2c_dw_dma_os_acknowledge(rx);
	return true;
}

/* i2c_dw_rp2040_dma_read(dev, msg): msg->buf, msg->len and msg->flags (the
 * Zephyr flags of the message, see i2c_dw_msg_flags()).
 */
static int i2c_dw_rp2040_dma_read(struct i2c_dw_dev_config *dw, uint8_t *buf, uint32_t len,
				  uint8_t msg_flags)
{
	uintptr_t base = dw->base;
	int tx, rx, ret;
	bool started = false;

	/* dev != i2c0 cannot happen: i2c0 is the only instance.  10-bit
	 * addresses are refused before (no I2C_MSG_ADDR_10_BITS here, and
	 * app_config never has I2C_ADDR_10_BITS).
	 */
	if (!rp2040_dma_enabled || !i2c_dw_dma_os_is_ready() || len < 32 ||
	    len > I2C_DW_RP2040_DMA_MAX_READ || !(msg_flags & I2C_MSG_STOP) ||
	    (msg_flags & I2C_MSG_ADDR_10_BITS)) {
		return -EAGAIN;
	}
	tx = i2c_dw_dma_os_request_channel();
	if (tx < 0) {
		rp2040_dma_fallbacks++;
		return -EAGAIN;
	}
	rx = i2c_dw_dma_os_request_channel();
	if (rx < 0) {
		rp2040_dma_fallbacks++;
		i2c_dw_dma_os_release_channel(tx);
		return -EAGAIN;
	}
	rp2040_dma_tx_channel = tx;
	rp2040_dma_rx_channel = rx;
	i2c_dw_rp2040_read_commands(rp2040_dma_commands, ARRAY_SIZE(rp2040_dma_commands), len,
				    dw->xfr_flags & I2C_MSG_RESTART);
	ret = i2c_dw_dma_os_config_tx(tx, rp2040_dma_commands, base + DW_IC_REG_DATA_CMD, len,
				      i2c_dw_rp2040_dma_callback, dw);
	if (!ret) {
		ret = i2c_dw_dma_os_config_rx(rx, base + DW_IC_REG_DATA_CMD, rp2040_dma_buffer, len,
					      i2c_dw_rp2040_dma_callback, dw);
	}
	if (ret) {
		rp2040_dma_fallbacks++;
		ret = -EAGAIN;
		goto release;
	}
	rp2040_dma_events = 0;
	i2c_dw_os_sem_reset(&rp2040_dma_sem);
	dw->rp2040_dma_active = 1;
	i2c_dw_write32(8, base + 0x8c); /* TX FIFO threshold */
	i2c_dw_write32(0, base + 0x90); /* RX FIFO threshold */
	write_intr_mask(DW_INTR_STAT_STOP_DET | DW_INTR_STAT_TX_ABRT | DW_INTR_STAT_TX_OVER |
				DW_INTR_STAT_RX_OVER | DW_INTR_STAT_RX_UNDER |
				DW_INTR_STAT_SCL_STUCK_LOW,
			base);
	started = true;
	ret = i2c_dw_dma_os_start(rx);
	if (!ret) {
		ret = i2c_dw_dma_os_start(tx);
	}
	if (!ret) {
		/* sys_timepoint_calc(rom->transfer_timeout) */
		i2c_dw_os_timepoint_t deadline =
			i2c_dw_os_timepoint_calc(I2C_DW_TRANSFER_TIMEOUT_MS);
		write_dma_cr(3, base);
		while ((rp2040_dma_events & RP2040_DMA_DONE) != RP2040_DMA_DONE) {
			if (rp2040_dma_events & RP2040_DMA_ERROR) {
				ret = -EIO;
				break;
			}
			/* sys_timepoint_timeout(deadline) */
			if (i2c_dw_os_sem_take_until(&rp2040_dma_sem, deadline)) {
				ret = -ETIMEDOUT;
				break;
			}
		}
		if (rp2040_dma_events & RP2040_DMA_ERROR) {
			ret = -EIO;
		}
	}
	write_dma_cr(0, base);
	write_intr_mask(0, base);
	dw->rp2040_dma_active = 0;
	if (ret) {
		set_bit_enable_abort(base);
	}
	if (ret && !i2c_dw_rp2040_dma_stop(tx, rx)) {
		dw->rp2040_dma_quarantined = true;
		rp2040_dma_errors++;
		LOG_ERR("DMA channels did not stop; I2C controller quarantined");
		return -EIO;
	}
	(void)read_clr_intr(base);
	if (ret) {
		int64_t abort_deadline = rp2040_dma_uptime_ms() + 100;
		while (test_bit_enable_abort(base)) {
			if (rp2040_dma_uptime_ms() >= abort_deadline) {
				dw->rp2040_dma_quarantined = true;
				LOG_ERR("I2C abort did not finish; controller quarantined");
				break;
			}
			i2c_dw_os_msleep(1);
		}
		(void)read_clr_intr(base);
	}
	if (!ret) {
		rp2040_dma_transfers++;
		rp2040_dma_bytes += len;
		memcpy(buf, rp2040_dma_buffer, len);
		dw->xfr_len = 0;
	}
release:
	if (ret && ret != -EAGAIN) {
		rp2040_dma_errors++;
	}
	if (!started || !dw->rp2040_dma_quarantined) {
		i2c_dw_dma_os_release_channel(tx);
		i2c_dw_dma_os_release_channel(rx);
	}
	return ret;
}

/* The i2c_dma shell command's side (i2c_dw_rp2040_dma_priv.h:272-291): the
 * command (common/shell/i2c_dma_shell.c, nuttx/apps/i2c_dma) prints and
 * parses; see i2c_dw_rp2040_dma.h.
 */

void i2c_dw_rp2040_dma_get_stats(struct i2c_dw_rp2040_dma_stats *stats)
{
	stats->transfers = rp2040_dma_transfers;
	stats->bytes = rp2040_dma_bytes;
	stats->errors = rp2040_dma_errors;
	stats->fallbacks = rp2040_dma_fallbacks;
}

bool i2c_dw_rp2040_dma_enabled(void)
{
	return rp2040_dma_enabled;
}

void i2c_dw_rp2040_dma_set_enabled(bool enabled)
{
	rp2040_dma_enabled = enabled;
}

#endif /* COMMON_DRIVERS_I2C_DW_I2C_DW_RP2040_DMA_PRIV_H_ */
