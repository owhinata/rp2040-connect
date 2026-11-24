/* SPDX-License-Identifier: MIT */
/* The DMA part of nina_bus_rp2040.c: the long transfers of SPI1 by two DMA
 * channels while the calling task sleeps (nina_bus_dma_rp2040.c).
 */

#ifndef NINA_BUS_DMA_RP2040_H_
#define NINA_BUS_DMA_RP2040_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* From nina_bus_initialize(), once SPI1 is set up: claims the two channels
 * and adds the handler.  When the SDK has no two free channels, nothing is
 * kept and every transfer stays polled.
 */
void nina_bus_dma_initialize(void);

/* True when a transfer of LEN bytes goes by DMA: the channels are there,
 * the scheduler runs, and LEN is at least the threshold.
 */
bool nina_bus_dma_wanted(size_t len);

/* The transfer of nina_bus_transceive() (TX and RX of LEN bytes, or no RX
 * when RX is NULL) by DMA: returns once every byte has been clocked out and
 * in; 0, or -EIO (logged) on a bus error, a lost byte or a transfer that
 * did not end in time (the channels aborted, SPI1 left as the polled
 * transfers leave it).
 */
int nina_bus_dma_transceive(const uint8_t *tx, uint8_t *rx, size_t len);

#endif /* NINA_BUS_DMA_RP2040_H_ */
