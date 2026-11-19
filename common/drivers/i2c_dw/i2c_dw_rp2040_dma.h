/* SPDX-License-Identifier: Apache-2.0 */
/* The DMA reads of I2C0: the state that the "i2c_dma" shell command reads
 * and sets (common/shell/i2c_dma_shell.c).  The core defines these
 * functions with CONFIG_I2C_DW_RP2040_DMA (i2c_dw_rp2040_dma_priv.h).  No OS
 * header.
 *
 * The Zephyr build's patched driver has the same declarations in Zephyr's
 * tree (include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h of the patch);
 * common/tests/i2c_dw_sync.py checks that the two stay the same.  The
 * command includes <zephyr/drivers/i2c/i2c_dw_rp2040_dma.h> in the three
 * builds; in the builds of this core that is a header of common/compat
 * which includes this one.
 */

#ifndef COMMON_DRIVERS_I2C_DW_I2C_DW_RP2040_DMA_H_
#define COMMON_DRIVERS_I2C_DW_I2C_DW_RP2040_DMA_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Counters since boot, 32 bits, wrapping
 */
struct i2c_dw_rp2040_dma_stats {
	/** Reads done by DMA */
	uint32_t transfers;
	/** Bytes of those reads */
	uint32_t bytes;
	/** DMA reads that failed (-EIO, -ETIMEDOUT), counted once each */
	uint32_t errors;
	/**
	 * Reads that met the conditions but went to the interrupt path
	 * because a DMA channel could not be had or configured
	 */
	uint32_t fallbacks;
};

/**
 * @brief Get the counters of the DMA reads.
 *
 * @param stats The counters.
 */
void i2c_dw_rp2040_dma_get_stats(struct i2c_dw_rp2040_dma_stats *stats);

/**
 * @brief Whether the DMA reads are on (they are at boot).
 */
bool i2c_dw_rp2040_dma_enabled(void);

/**
 * @brief Turn the DMA reads on or off.
 *
 * Off sends every read to the interrupt path and drops the
 * RX_FIFO_FULL_HLD_CTRL bit from IC_CON at the next transfer. Nothing waits
 * for a transfer in progress: switch only while no transfer runs.
 *
 * @param enabled true for on.
 */
void i2c_dw_rp2040_dma_set_enabled(bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_DRIVERS_I2C_DW_I2C_DW_RP2040_DMA_H_ */
