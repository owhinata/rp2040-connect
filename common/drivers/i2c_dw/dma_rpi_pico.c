/*
 * Copyright (c) 2023 TOKITA Hiroshi <tokita.hiroshi@fujitsu.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The RP2040 DMA for the I2C0 DMA reads (i2c_dw.c, CONFIG_I2C_DW_RP2040_DMA): a port of the
 * Zephyr build's DMA driver, zephyr/drivers/dma/dma_rpi_pico.c, in its order and with its
 * names and statements, but:
 *
 *   Zephyr                                here
 *   struct device, dev->config, dev->data the one instance, dma_rpi_pico0_config and
 *                                         dma_rpi_pico0_data (RPI_PICO_DMA_INIT(0) written
 *                                         out below the structures), no dev argument
 *   dma_config(), dma_start(), dma_stop() dma_rpi_pico_config(), _start(), _stop(), called
 *   of the DMA API (DEVICE_API(dma))      directly
 *   dma_request_channel(dma, NULL),       dma_rpi_pico_request_channel(), _release_channel()
 *   dma_release_channel() (dma.h)         (below): the same channels, with irq_lock() for
 *                                         the atomic bit operations
 *   device_is_ready(dma)                  dma_rpi_pico_is_ready()
 *   the pico-sdk (<hardware/dma.h>)       the same names and shapes, written out in
 *                                         dma_rpi_pico.h
 *   dma_callback_t                        dma_rpi_pico_callback_t, without the device
 *   dma_rpi_pico_isr()                    dma_rpi_pico_isr_handler(), which the OS's
 *                                         handler calls
 *   dma_rpi_pico_init(), irq_configure(), dma_rpi_pico_initialize(), and the OS's part:
 *   reset_line_toggle_dt()                dma_rpi_pico_instance.h, read at the end of this
 *                                         file, defines dma_rpi_pico_os_reset() (the reset
 *                                         of the DMA block) and dma_rpi_pico_os_irq_enable()
 *                                         (DMA_IRQ_0 and DMA_IRQ_1 on a handler that calls
 *                                         dma_rpi_pico_isr_handler(), then enabled)
 *   LOG_*() of module dma_rpi_pico at     the same (app_log.h)
 *   CONFIG_DMA_LOG_LEVEL (3)
 *
 * The same register values in the same order:
 *
 * - dma_rpi_pico_config() writes nothing: it keeps the CTRL value of the pico-sdk's
 *   dma_channel_get_default_config() (EN, CHAIN_TO = the channel itself, no ring, no byte swap,
 *   IRQ_QUIET off, sniff off) with INCR_READ, INCR_WRITE, DATA_SIZE, TREQ_SEL (the DREQ of the
 *   slot) and HIGH_PRIORITY (channel_priority != 0) set from the configuration, and the
 *   addresses and the block size.
 * - dma_rpi_pico_start(): nothing if the channel is BUSY; else acknowledge the channel on its
 *   interrupt line, enable it there (INTE set alias), then READ_ADDR, WRITE_ADDR, TRANS_COUNT
 *   (block size / data size) and CTRL_TRIG (dma_channel_configure(..., true): starts the
 *   channel).
 * - dma_rpi_pico_stop(): disable the channel on its line (INTE clear alias), write AL1_CTRL
 *   back with the error bits cleared, CHAN_ABORT = the channel's bit.  It does not wait for the
 *   abort (the caller does).
 * - The interrupt line of a channel: DMA_IRQ_0 for the channels listed in the devicetree's
 *   irq0-channels, DMA_IRQ_1 for the others.  The Zephyr build has no irq0-channels property,
 *   so the list is the default {0}: channel 0 on DMA_IRQ_0, channels 1-11 on DMA_IRQ_1.  Both
 *   lines run the same handler, which goes through all channels: for each one whose status is
 *   set on its own line, the status of the bus error bits, the acknowledge, disable on the line,
 *   the error bits written back, then the callback with 0 or -EIO.
 * - dma_rpi_pico_request_channel() (dma_request_channel(dma, NULL)): the lowest channel not
 *   taken (the RP2040 driver's filter accepts every channel without a parameter), or -EINVAL;
 *   it never waits.  dma_rpi_pico_release_channel(): the RP2040 driver has no chan_release, so
 *   only the channel's bit is cleared.
 * - dma_rpi_pico_initialize(): reset the DMA block and attach and enable both interrupts.
 *
 * No other user of the DMA may own its interrupts or channels: as in the Zephyr build, where
 * i2c0 is the DMA's only user, the allocation here does not know of other users, and the reset
 * at init stops any channel that runs.  (The NuttX build turns NuttX's own DMA layer off,
 * nuttx/drivers/README.md.)
 *
 * Left out (not used by i2c_dw.c): reload, get_status, chan_filter, block chains,
 * memory-to-memory, more than one DMA instance.  common/tests/i2c_dw_sync.py compares the
 * functions with Zephyr's.
 */

#include "i2c_dw_os.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* CONFIG_DMA_LOG_LEVEL of the Zephyr build (3). */
#define LOG_LEVEL 3
#include "app_log.h"

#include "dma_rpi_pico.h"

#define DMA_INT_ERROR_FLAGS                                                                        \
	(DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS | DMA_CH0_CTRL_TRIG_READ_ERROR_BITS |                    \
	 DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS)

LOG_MODULE_REGISTER(dma_rpi_pico);

struct dma_rpi_pico_config {
	uint32_t reg;
	uint32_t channels;
	const uint32_t *irq0_channels;
	size_t irq0_channels_size;
};

struct dma_rpi_pico_channel {
	dma_rpi_pico_callback_t callback;
	void *user_data;
	uint32_t direction;
	dma_channel_config config;
	void *source_address;
	void *dest_address;
	size_t block_size;
	uint32_t source_data_size;
};

struct dma_rpi_pico_data {
	struct dma_rpi_pico_channel *channels;
};

/*
 * RPI_PICO_DMA_INIT(0) for the devicetree's dma node (dma-channels = <12>, no irq0-channels).
 * The configuration and the data are const, so the compiler folds them into the functions.
 */
static const uint32_t dma_rpi_pico0_irq0_channels[] = {0};
static const struct dma_rpi_pico_config dma_rpi_pico0_config = {
	.reg = DMA_BASE,
	.channels = DMA_RPI_PICO_CHANNELS,
	.irq0_channels = dma_rpi_pico0_irq0_channels,
	.irq0_channels_size = ARRAY_SIZE(dma_rpi_pico0_irq0_channels),
};
static struct dma_rpi_pico_channel dma_rpi_pico0_channels[DMA_RPI_PICO_CHANNELS];
/* ATOMIC_DEFINE(dma_rpi_pico_atomic0, 12): dma_context.atomic, the taken channels. */
static uint32_t dma_rpi_pico_atomic0;
static const struct dma_rpi_pico_data dma_rpi_pico0_data = {
	.channels = dma_rpi_pico0_channels,
};
/* The device's initialized state (device_is_ready()). */
static bool dma_rpi_pico0_ready;

/*
 * Register access functions
 */

static inline void rpi_pico_dma_channel_clear_error_flags(uint32_t channel)
{
	const struct dma_rpi_pico_config *cfg = &dma_rpi_pico0_config;

	((dma_hw_t *)cfg->reg)->ch[channel].al1_ctrl &= ~DMA_INT_ERROR_FLAGS;
}

static inline uint32_t rpi_pico_dma_channel_get_error_flags(uint32_t channel)
{
	const struct dma_rpi_pico_config *cfg = &dma_rpi_pico0_config;

	return ((dma_hw_t *)cfg->reg)->ch[channel].al1_ctrl & DMA_INT_ERROR_FLAGS;
}

static inline void rpi_pico_dma_channel_abort(uint32_t channel)
{
	const struct dma_rpi_pico_config *cfg = &dma_rpi_pico0_config;

	((dma_hw_t *)cfg->reg)->abort = BIT(channel);
}

/*
 * Utility functions
 */

static inline uint32_t dma_rpi_pico_transfer_size(uint32_t width)
{
	switch (width) {
	case 4:
		return DMA_SIZE_32;
	case 2:
		return DMA_SIZE_16;
	default:
		return DMA_SIZE_8;
	}
}

static inline uint32_t dma_rpi_pico_channel_irq(uint32_t channel)
{
	const struct dma_rpi_pico_config *cfg = &dma_rpi_pico0_config;

	for (size_t i = 0; i < cfg->irq0_channels_size; i++) {
		if (cfg->irq0_channels[i] == channel) {
			return 0;
		}
	}

	return 1;
}

/*
 * API functions
 */

int dma_rpi_pico_config(uint32_t channel, struct dma_rpi_pico_channel_config *dma_cfg)
{
	const struct dma_rpi_pico_config *cfg = &dma_rpi_pico0_config;
	const struct dma_rpi_pico_data *data = &dma_rpi_pico0_data;

	if (channel >= cfg->channels) {
		LOG_ERR("channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, channel);
		return -EINVAL;
	}

	if (dma_cfg->block_count != 1) {
		LOG_ERR("chained block transfer not supported.");
		return -ENOTSUP;
	}

	if (dma_cfg->channel_priority > 3) {
		LOG_ERR("channel_priority must be < 4 (%" PRIu32 ")", dma_cfg->channel_priority);
		return -EINVAL;
	}

	if (dma_cfg->head_block->source_addr_adj == DMA_RPI_PICO_ADDR_ADJ_DECREMENT) {
		LOG_ERR("source_addr_adj not supported DMA_ADDR_ADJ_DECREMENT");
		return -ENOTSUP;
	}

	if (dma_cfg->head_block->dest_addr_adj == DMA_RPI_PICO_ADDR_ADJ_DECREMENT) {
		LOG_ERR("dest_addr_adj not supported DMA_ADDR_ADJ_DECREMENT");
		return -ENOTSUP;
	}

	if (dma_cfg->head_block->source_addr_adj != DMA_RPI_PICO_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->source_addr_adj != DMA_RPI_PICO_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid source_addr_adj %" PRIu16, dma_cfg->head_block->source_addr_adj);
		return -ENOTSUP;
	}
	if (dma_cfg->head_block->dest_addr_adj != DMA_RPI_PICO_ADDR_ADJ_INCREMENT &&
	    dma_cfg->head_block->dest_addr_adj != DMA_RPI_PICO_ADDR_ADJ_NO_CHANGE) {
		LOG_ERR("invalid dest_addr_adj %" PRIu16, dma_cfg->head_block->dest_addr_adj);
		return -ENOTSUP;
	}

	if (dma_cfg->source_data_size != 1 && dma_cfg->source_data_size != 2 &&
	    dma_cfg->source_data_size != 4) {
		LOG_ERR("source_data_size must be 1, 2, or 4 (%" PRIu32 ")",
			dma_cfg->source_data_size);
		return -EINVAL;
	}

	if (dma_cfg->source_data_size != dma_cfg->dest_data_size) {
		return -EINVAL;
	}

	if (dma_cfg->dest_data_size != 1 && dma_cfg->dest_data_size != 2 &&
	    dma_cfg->dest_data_size != 4) {
		LOG_ERR("dest_data_size must be 1, 2, or 4 (%" PRIu32 ")", dma_cfg->dest_data_size);
		return -EINVAL;
	}

	if (dma_cfg->channel_direction > DMA_RPI_PICO_PERIPHERAL_TO_MEMORY) {
		LOG_ERR("channel_direction must be MEMORY_TO_MEMORY, "
			"MEMORY_TO_PERIPHERAL or PERIPHERAL_TO_MEMORY (%" PRIu32 ")",
			dma_cfg->channel_direction);
		return -ENOTSUP;
	}

	data->channels[channel].config = dma_channel_get_default_config(channel);

	data->channels[channel].source_address = (void *)dma_cfg->head_block->source_address;
	data->channels[channel].dest_address = (void *)dma_cfg->head_block->dest_address;
	data->channels[channel].block_size = dma_cfg->head_block->block_size;
	channel_config_set_read_increment(&data->channels[channel].config,
					  dma_cfg->head_block->source_addr_adj ==
						  DMA_RPI_PICO_ADDR_ADJ_INCREMENT);
	channel_config_set_write_increment(&data->channels[channel].config,
					   dma_cfg->head_block->dest_addr_adj ==
						   DMA_RPI_PICO_ADDR_ADJ_INCREMENT);
	channel_config_set_transfer_data_size(
		&data->channels[channel].config,
		dma_rpi_pico_transfer_size(dma_cfg->source_data_size));
	channel_config_set_dreq(&data->channels[channel].config,
				DMA_RPI_PICO_SLOT_TO_DREQ(dma_cfg->dma_slot));
	channel_config_set_high_priority(&data->channels[channel].config,
					 !!(dma_cfg->channel_priority));

	data->channels[channel].callback = dma_cfg->dma_callback;
	data->channels[channel].user_data = dma_cfg->user_data;
	data->channels[channel].source_data_size = dma_cfg->source_data_size;
	data->channels[channel].direction = dma_cfg->channel_direction;

	return 0;
}

int dma_rpi_pico_start(uint32_t ch)
{
	const struct dma_rpi_pico_config *cfg = &dma_rpi_pico0_config;
	const struct dma_rpi_pico_data *data = &dma_rpi_pico0_data;

	if (ch >= cfg->channels) {
		LOG_ERR("start channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, ch);
		return -EINVAL;
	}

	if (dma_channel_is_busy(ch)) {
		return 0;
	}

	dma_irqn_acknowledge_channel(dma_rpi_pico_channel_irq(ch), ch);
	dma_irqn_set_channel_enabled(dma_rpi_pico_channel_irq(ch), ch, true);

	dma_channel_configure(ch, &data->channels[ch].config, data->channels[ch].dest_address,
			      data->channels[ch].source_address,
			      data->channels[ch].block_size / data->channels[ch].source_data_size,
			      true);

	return 0;
}

int dma_rpi_pico_stop(uint32_t ch)
{
	const struct dma_rpi_pico_config *cfg = &dma_rpi_pico0_config;

	if (ch >= cfg->channels) {
		LOG_ERR("stop channel must be < %" PRIu32 " (%" PRIu32 ")", cfg->channels, ch);
		return -EINVAL;
	}

	dma_irqn_set_channel_enabled(dma_rpi_pico_channel_irq(ch), ch, false);
	rpi_pico_dma_channel_clear_error_flags(ch);

	/*
	 * Considering the possibility of being called in an interrupt context,
	 * it does not wait until the abort bit becomes clear.
	 * Ensure the busy status is canceled with dma_channel_is_busy()
	 * before the next transfer starts.
	 */
	rpi_pico_dma_channel_abort(ch);

	return 0;
}

/*
 * dma_request_channel(dma, NULL) and dma_release_channel() of zephyr/drivers/dma.h: the device's
 * initialized state in place of its magic, a critical section in place of the atomic bit
 * operations, and no filter (the driver's accepts every channel without a parameter) or
 * chan_release (the driver has none).
 */

int dma_rpi_pico_request_channel(void)
{
	int channel = -EINVAL;
	i2c_dw_os_irqstate_t flags;

	if (!dma_rpi_pico0_ready) {
		return channel;
	}

	flags = i2c_dw_os_irq_lock();
	for (int i = 0; i < DMA_RPI_PICO_CHANNELS; i++) {
		if ((dma_rpi_pico_atomic0 & (1u << i)) == 0) {
			dma_rpi_pico_atomic0 |= 1u << i;
			channel = i;
			break;
		}
	}

	i2c_dw_os_irq_unlock(flags);
	return channel;
}

void dma_rpi_pico_release_channel(uint32_t channel)
{
	i2c_dw_os_irqstate_t flags;

	if (!dma_rpi_pico0_ready) {
		return;
	}

	if (channel < DMA_RPI_PICO_CHANNELS) {
		flags = i2c_dw_os_irq_lock();
		dma_rpi_pico_atomic0 &= ~(1u << channel);
		i2c_dw_os_irq_unlock(flags);
	}
}

bool dma_rpi_pico_is_ready(void)
{
	return dma_rpi_pico0_ready;
}

/* The OS's part (dma_rpi_pico_instance.h): reset_line_toggle_dt() and irq_configure(). */
static void dma_rpi_pico_os_reset(void);
static void dma_rpi_pico_os_irq_enable(void);

int dma_rpi_pico_initialize(void)
{
	if (dma_rpi_pico0_ready) {
		return 0;
	}

	dma_rpi_pico_os_reset();

	dma_rpi_pico_os_irq_enable();

	dma_rpi_pico0_ready = true;
	return 0;
}

/* The body of the OS's handler of DMA_IRQ_0 and DMA_IRQ_1 (dma_rpi_pico_instance.h). */
static inline void dma_rpi_pico_isr_handler(void)
{
	const struct dma_rpi_pico_config *cfg = &dma_rpi_pico0_config;
	const struct dma_rpi_pico_data *data = &dma_rpi_pico0_data;

	for (uint32_t i = 0; i < cfg->channels; i++) {
		int err = 0;

		if (!dma_irqn_get_channel_status(dma_rpi_pico_channel_irq(i), i)) {
			continue;
		}

		if (rpi_pico_dma_channel_get_error_flags(i)) {
			err = -EIO;
		}

		dma_irqn_acknowledge_channel(dma_rpi_pico_channel_irq(i), i);
		dma_irqn_set_channel_enabled(dma_rpi_pico_channel_irq(i), i, false);
		rpi_pico_dma_channel_clear_error_flags(i);

		if (data->channels[i].callback) {
			data->channels[i].callback(data->channels[i].user_data, i, err);
		}
	}
}

/* The OS's part: the functions declared above and the OS's interrupt handler. */
#include "dma_rpi_pico_instance.h"
