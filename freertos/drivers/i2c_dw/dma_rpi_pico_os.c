/* SPDX-License-Identifier: MIT */
/* The claim of the DMA channels in the Pico SDK's record for the FreeRTOS
 * part of the DMA core (dma_rpi_pico_instance.h, which says why).  A file
 * of its own: the core's dma_rpi_pico.h writes out the SDK's DMA types and
 * functions (dma_hw_t, dma_channel_configure(), ...) unless the SDK's
 * headers were read first, so the SDK's hardware/dma.h cannot follow it in
 * dma_rpi_pico.c.  Here it comes first, and dma_rpi_pico.h after it takes
 * the SDK's (its macros, the same as the SDK's, are checked by the
 * compiler).
 */

#include <stdint.h>

#include "hardware/dma.h"

#include "dma_rpi_pico.h"

void dma_rpi_pico_os_claim(void);

void dma_rpi_pico_os_claim(void)
{
	dma_claim_mask((1u << DMA_RPI_PICO_CHANNELS) - 1u);
}
