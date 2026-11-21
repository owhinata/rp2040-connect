/* SPDX-License-Identifier: MIT */
/* What the DMA reads of I2C0 (CONFIG_I2C_DW_RP2040_DMA,
 * i2c_dw_rp2040_dma_priv.h) need from the OS's DMA: two channels for one
 * read, set up for I2C0's IC_DATA_CMD, started, stopped and given back.
 * Each OS's binding implements these functions on the OS's own DMA driver
 * (port/freertos/i2c_dw_dma_os_rp2040.c on the Pico SDK's hardware_dma and
 * hardware_irq, port/nuttx/i2c_dw_dma_os_rp2040.c on NuttX's
 * rp2040_dmac.c), so that other code of the same program may use that DMA
 * the usual way.  The core touches no DMA register.
 *
 * The calls correspond to those of the Zephyr build's patched driver on
 * Zephyr's DMA API (drivers/i2c/i2c_dw_rp2040_dma_priv.h of patch 0003):
 *
 *   Zephyr                                   here
 *   device_is_ready(dma)                     i2c_dw_dma_os_is_ready()
 *   dma_request_channel(dma, NULL)           i2c_dw_dma_os_request_channel()
 *   dma_release_channel(dma, ch)             i2c_dw_dma_os_release_channel()
 *   dma_config(dma, tx, &tx_cfg)             i2c_dw_dma_os_config_tx()
 *   dma_config(dma, rx, &rx_cfg)             i2c_dw_dma_os_config_rx()
 *   dma_start(dma, ch)                       i2c_dw_dma_os_start()
 *   dma_stop(dma, ch)                        i2c_dw_dma_os_stop()
 *   dma_channel_is_busy(ch) (and             i2c_dw_dma_os_is_busy()
 *   dma_hw->abort, of all channels)
 *   dma_irqn_acknowledge_channel(0/1, ch)    i2c_dw_dma_os_acknowledge()
 *
 * Only the transfer that holds I2C0's bus lock calls them (one read at a
 * time), from a thread, except the callback, which the OS's part calls from
 * the DMA interrupt.  The I2C0 interrupt and the DMA interrupt that calls
 * the callback run at the same priority on the one core, so neither
 * interrupts the other (the core's events are updated under
 * i2c_dw_os_irq_lock()).  A channel is a number of the RP2040's DMA,
 * 0 to 11.  No function waits for another user of the DMA.
 */

#ifndef COMMON_DRIVERS_I2C_DW_I2C_DW_DMA_OS_H_
#define COMMON_DRIVERS_I2C_DW_I2C_DW_DMA_OS_H_

#include <stdbool.h>
#include <stdint.h>

/* The end of a channel's transfer, from the DMA interrupt: status 0 when
 * the channel has made all its transfers, -EIO when it stopped on a bus
 * error (a read or write error of the channel; the OS's part has cleared
 * the error and stopped the channel).  At most once per
 * i2c_dw_dma_os_start(), and not after i2c_dw_dma_os_stop() or
 * i2c_dw_dma_os_release_channel() of the channel.
 */

typedef void (*i2c_dw_dma_os_callback_t)(void *user_data, uint32_t channel, int status);

/* The OS's part's start-up (the OS's i2c_dw_instance.h calls it before
 * i2c_dw_turn_on(), as the Zephyr build initializes its DMA device before
 * i2c0; the core does not call it): what the OS's DMA needs before the first
 * read (the interrupt handler, channels held for the reads).  It resets
 * nothing: the OS (or the SDK) has initialized the DMA.  0, or a negative
 * errno; the reads then take the interrupt path (i2c_dw_dma_os_is_ready()
 * is false).  A second call returns the first result.  Thread.
 */

int i2c_dw_dma_os_initialize(void);

/* device_is_ready(dma): i2c_dw_dma_os_initialize() returned 0. */

bool i2c_dw_dma_os_is_ready(void);

/* dma_request_channel(dma, NULL): a free channel for this read, or a
 * negative errno (-EBUSY: none free now; -ENODEV: not ready).  Never
 * waits.  The core then counts a fallback and reads on the interrupts.
 */

int i2c_dw_dma_os_request_channel(void);

/* dma_release_channel(): the channel goes back.  Its transfer has ended
 * (both callbacks came) or was stopped and is no longer busy
 * (i2c_dw_dma_os_is_busy() false), or it was never started.  The channel's
 * callback is not called after this.
 */

void i2c_dw_dma_os_release_channel(uint32_t channel);

/* dma_config() of the TX side: count 16-bit transfers from words
 * (incrementing) to the register at dest (IC_DATA_CMD, not incrementing),
 * paced by I2C0's TX DREQ (32), at normal priority, the end reported to
 * callback(user_data, channel, status).  Not started.  0, or a negative
 * errno (-EBUSY: the channel is still busy), with nothing set up.
 */

int i2c_dw_dma_os_config_tx(uint32_t channel, const uint16_t *words, uintptr_t dest,
			    uint32_t count, i2c_dw_dma_os_callback_t callback, void *user_data);

/* dma_config() of the RX side: count 8-bit transfers from the register at
 * source (IC_DATA_CMD, not incrementing) to bytes (incrementing), paced by
 * I2C0's RX DREQ (33), otherwise as i2c_dw_dma_os_config_tx().
 */

int i2c_dw_dma_os_config_rx(uint32_t channel, uintptr_t source, uint8_t *bytes, uint32_t count,
			    i2c_dw_dma_os_callback_t callback, void *user_data);

/* dma_start(): start the channel as set up (its interrupt enabled, an old
 * completion flag of it cleared first).  0, or a negative errno (-EBUSY:
 * still busy), and then the channel did not start.
 */

int i2c_dw_dma_os_start(uint32_t channel);

/* dma_stop(): request the abort of the channel's transfer and disable its
 * interrupt; returns at once, without waiting for the abort (the caller
 * waits on i2c_dw_dma_os_is_busy(), with its own bound).  0, or a negative
 * errno.
 */

int i2c_dw_dma_os_stop(uint32_t channel);

/* The channel is still transferring or its abort has not finished
 * (dma_channel_is_busy(), and this channel's bit of CHAN_ABORT).
 */

bool i2c_dw_dma_os_is_busy(uint32_t channel);

/* Clear a completion flag of the channel that is still pending: after a
 * stop, an aborted transfer may still flag its completion (RP2040-E13).
 */

void i2c_dw_dma_os_acknowledge(uint32_t channel);

#endif /* COMMON_DRIVERS_I2C_DW_I2C_DW_DMA_OS_H_ */
