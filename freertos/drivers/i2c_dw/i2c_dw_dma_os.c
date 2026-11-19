/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the DMA reads of I2C0: the boundary of
 * common/drivers/i2c_dw/i2c_dw_dma_os.h on the Pico SDK's hardware_dma and
 * hardware_irq, which other code of this build may use as usual.
 *
 *   the boundary                  here
 *   i2c_dw_dma_os_initialize()    irq_add_shared_handler() on DMA_IRQ_1
 *                                 (the default order priority), then
 *                                 irq_set_enabled(); once
 *   _request_channel()            dma_claim_unused_channel(false): the
 *                                 lowest unclaimed channel of the SDK's
 *                                 record, -EBUSY if there is none
 *   _release_channel()            the channel off DMA_IRQ_1, its flag
 *                                 cleared, dma_channel_unclaim()
 *   _config_tx(), _config_rx()    dma_channel_get_default_config(), the
 *                                 increments, DMA_SIZE_16 or DMA_SIZE_8,
 *                                 DREQ_I2C0_TX or DREQ_I2C0_RX, then
 *                                 dma_channel_configure() without trigger
 *   _start()                      the channel's flag on DMA_IRQ_1 cleared,
 *                                 its interrupt there enabled, then
 *                                 dma_channel_start()
 *   _stop()                       its interrupt disabled, then its bit
 *                                 written to CHAN_ABORT (dma_hw->abort);
 *                                 no wait (the SDK's dma_channel_abort()
 *                                 would wait without a bound)
 *   _is_busy()                    dma_channel_is_busy(), or its bit still
 *                                 set in CHAN_ABORT
 *   _acknowledge()                dma_irqn_acknowledge_channel() on DMA_IRQ_1
 *
 * The handler looks only at the channels that a read holds (a mask kept
 * under save_and_disable_interrupts()) whose flag is set on DMA_IRQ_1;
 * other users' channels on the shared line are left to their handlers.
 * For each of ours: a bus error (AHB_ERROR: READ_ERROR or WRITE_ERROR) gives
 * -EIO, the flag is cleared, the channel's interrupt disabled, its error
 * flags cleared (written with 1 through the SET alias of AL1_CTRL, which
 * is not a trigger), then the callback.  A channel stops by itself on a bus
 * error, and sets its flag then as at the end.
 *
 * The line's priority is the SDK's default for the peripherals
 * (PICO_DEFAULT_IRQ_PRIORITY, 0x80), the same as I2C0's, and the build has
 * one core (configNUMBER_OF_CORES 1), so the I2C0 and DMA handlers do not
 * interrupt each other.  The other handlers on the line run in the order
 * of their order priorities; each must clear only its own channels' flags
 * (the SDK's rule for shared handlers).  irq_set_enabled() also clears the
 * line's pending state in the NVIC; the DMA's lines are level interrupts,
 * so a request that is still active is pending again at once.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include "i2c_dw_dma_os.h"

/* The DMA interrupt line of the reads' channels (0 or 1). */
#define I2C_DW_DMA_IRQ_INDEX 1

#define I2C_DW_DMA_ERROR_BITS                                                                      \
	(DMA_CH0_CTRL_TRIG_READ_ERROR_BITS | DMA_CH0_CTRL_TRIG_WRITE_ERROR_BITS)

struct i2c_dw_dma_os_channel {
	i2c_dw_dma_os_callback_t callback;
	void *user_data;
};

static struct i2c_dw_dma_os_channel i2c_dw_dma_os_channels[NUM_DMA_CHANNELS];
/* The channels that a read holds (claimed by i2c_dw_dma_os_request_channel()). */
static volatile uint32_t i2c_dw_dma_os_held;
static bool i2c_dw_dma_os_ready;

static void i2c_dw_dma_os_isr(void)
{
	uint32_t held = i2c_dw_dma_os_held;

	for (uint32_t ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
		const struct i2c_dw_dma_os_channel *c = &i2c_dw_dma_os_channels[ch];
		int status = 0;

		if (!(held & (1u << ch)) ||
		    !dma_irqn_get_channel_status(I2C_DW_DMA_IRQ_INDEX, ch)) {
			continue;
		}
		if (dma_hw->ch[ch].al1_ctrl & DMA_CH0_CTRL_TRIG_AHB_ERROR_BITS) {
			status = -EIO;
		}
		dma_irqn_acknowledge_channel(I2C_DW_DMA_IRQ_INDEX, ch);
		dma_irqn_set_channel_enabled(I2C_DW_DMA_IRQ_INDEX, ch, false);
		if (status) {
			hw_set_bits(&dma_hw->ch[ch].al1_ctrl, I2C_DW_DMA_ERROR_BITS);
		}
		if (c->callback) {
			c->callback(c->user_data, ch, status);
		}
	}
}

int i2c_dw_dma_os_initialize(void)
{
	if (i2c_dw_dma_os_ready) {
		return 0;
	}
	irq_add_shared_handler(DMA_IRQ_NUM(I2C_DW_DMA_IRQ_INDEX), i2c_dw_dma_os_isr,
			       PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
	irq_set_enabled(DMA_IRQ_NUM(I2C_DW_DMA_IRQ_INDEX), true);
	i2c_dw_dma_os_ready = true;
	return 0;
}

bool i2c_dw_dma_os_is_ready(void)
{
	return i2c_dw_dma_os_ready;
}

int i2c_dw_dma_os_request_channel(void)
{
	uint32_t flags;
	int ch;

	if (!i2c_dw_dma_os_ready) {
		return -ENODEV;
	}
	ch = dma_claim_unused_channel(false);
	if (ch < 0) {
		return -EBUSY;
	}
	flags = save_and_disable_interrupts();
	i2c_dw_dma_os_channels[ch].callback = NULL;
	i2c_dw_dma_os_channels[ch].user_data = NULL;
	i2c_dw_dma_os_held |= 1u << ch;
	restore_interrupts(flags);
	return ch;
}

void i2c_dw_dma_os_release_channel(uint32_t channel)
{
	uint32_t flags;

	if (channel >= NUM_DMA_CHANNELS || !(i2c_dw_dma_os_held & (1u << channel))) {
		return;
	}
	flags = save_and_disable_interrupts();
	i2c_dw_dma_os_held &= ~(1u << channel);
	i2c_dw_dma_os_channels[channel].callback = NULL;
	i2c_dw_dma_os_channels[channel].user_data = NULL;
	dma_irqn_set_channel_enabled(I2C_DW_DMA_IRQ_INDEX, channel, false);
	dma_irqn_acknowledge_channel(I2C_DW_DMA_IRQ_INDEX, channel);
	restore_interrupts(flags);
	dma_channel_unclaim(channel);
}

static int i2c_dw_dma_os_config(uint32_t channel, const dma_channel_config *config,
				volatile void *write_addr, const volatile void *read_addr,
				uint32_t count, i2c_dw_dma_os_callback_t callback, void *user_data)
{
	uint32_t flags;

	if (channel >= NUM_DMA_CHANNELS || !(i2c_dw_dma_os_held & (1u << channel))) {
		return -EINVAL;
	}
	if (dma_channel_is_busy(channel)) {
		return -EBUSY;
	}
	dma_channel_configure(channel, config, write_addr, read_addr,
			      dma_encode_transfer_count(count), false);
	flags = save_and_disable_interrupts();
	i2c_dw_dma_os_channels[channel].callback = callback;
	i2c_dw_dma_os_channels[channel].user_data = user_data;
	restore_interrupts(flags);
	return 0;
}

int i2c_dw_dma_os_config_tx(uint32_t channel, const uint16_t *words, uintptr_t dest, uint32_t count,
			    i2c_dw_dma_os_callback_t callback, void *user_data)
{
	dma_channel_config c;

	if (channel >= NUM_DMA_CHANNELS) {
		return -EINVAL;
	}
	c = dma_channel_get_default_config(channel);
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
	channel_config_set_dreq(&c, DREQ_I2C0_TX);
	return i2c_dw_dma_os_config(channel, &c, (volatile void *)dest, words, count, callback,
				    user_data);
}

int i2c_dw_dma_os_config_rx(uint32_t channel, uintptr_t source, uint8_t *bytes, uint32_t count,
			    i2c_dw_dma_os_callback_t callback, void *user_data)
{
	dma_channel_config c;

	if (channel >= NUM_DMA_CHANNELS) {
		return -EINVAL;
	}
	c = dma_channel_get_default_config(channel);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_dreq(&c, DREQ_I2C0_RX);
	return i2c_dw_dma_os_config(channel, &c, bytes, (const volatile void *)source, count,
				    callback, user_data);
}

int i2c_dw_dma_os_start(uint32_t channel)
{
	if (channel >= NUM_DMA_CHANNELS || !(i2c_dw_dma_os_held & (1u << channel))) {
		return -EINVAL;
	}
	if (dma_channel_is_busy(channel)) {
		return -EBUSY;
	}
	dma_irqn_acknowledge_channel(I2C_DW_DMA_IRQ_INDEX, channel);
	dma_irqn_set_channel_enabled(I2C_DW_DMA_IRQ_INDEX, channel, true);
	dma_channel_start(channel);
	return 0;
}

int i2c_dw_dma_os_stop(uint32_t channel)
{
	if (channel >= NUM_DMA_CHANNELS || !(i2c_dw_dma_os_held & (1u << channel))) {
		return -EINVAL;
	}
	dma_irqn_set_channel_enabled(I2C_DW_DMA_IRQ_INDEX, channel, false);
	dma_hw->abort = 1u << channel;
	return 0;
}

bool i2c_dw_dma_os_is_busy(uint32_t channel)
{
	if (channel >= NUM_DMA_CHANNELS) {
		return false;
	}
	return dma_channel_is_busy(channel) || (dma_hw->abort & (1u << channel));
}

void i2c_dw_dma_os_acknowledge(uint32_t channel)
{
	if (channel < NUM_DMA_CHANNELS) {
		dma_irqn_acknowledge_channel(I2C_DW_DMA_IRQ_INDEX, channel);
	}
}
