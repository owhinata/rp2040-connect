/* SPDX-License-Identifier: Apache-2.0 */
/* The I2C0 DMA reads of the board's I2C driver
 * (boards/arduino-nano-rp2040-connect/src/nano_i2c.c, CONFIG_NANO_I2C0_DMA):
 * the state that the Zephyr build's "i2c_dma" shell command reads and sets
 * (zephyr/app/usb-console/drivers/i2c_dw/i2c_dw.c: rp_dma_enabled and the
 * counters rp_dma_transfers, rp_dma_bytes, rp_dma_errors,
 * rp_dma_fallbacks).  The flat build lets the command (nuttx/apps/i2c_dma)
 * call these directly.
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_NANO_I2C_DMA_H
#define __NUTTX_DRIVERS_INCLUDE_NANO_I2C_DMA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Counters since boot, 32 bits, wrapping (Zephyr's atomic_t):
 * transfers  reads done by DMA
 * bytes      bytes of those reads
 * errors     DMA reads that failed (-EIO, -ETIMEDOUT), counted once each
 * fallbacks  reads that met the conditions but went to the interrupt path
 *            because a DMA channel could not be had or configured
 */

struct nano_i2c_dma_stats
{
  uint32_t transfers;
  uint32_t bytes;
  uint32_t errors;
  uint32_t fallbacks;
};

void nano_i2c_dma_get_stats(struct nano_i2c_dma_stats *stats);

/* rp_dma_enabled: on at boot.  Turning it off sends every read to the
 * interrupt path and drops the RX_FIFO_FULL_HLD_CTRL bit from IC_CON at
 * the next transfer.  Nothing waits for a transfer in progress (as in the
 * Zephyr build): switch only while no transfer runs.
 */

bool nano_i2c_dma_enabled(void);
void nano_i2c_dma_set_enabled(bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* __NUTTX_DRIVERS_INCLUDE_NANO_I2C_DMA_H */
