/* SPDX-License-Identifier: MIT */
/* The claim of the DMA channels in the Pico SDK's record for the FreeRTOS
 * part of the DMA core (nano_dma_instance.h, which says why).  A file of its
 * own: the SDK's hardware/dma.h declares functions of the names that the
 * core's nano_dma.c defines for itself (dma_channel_configure(), ...).
 */

#include <stdint.h>

#include "hardware/dma.h"

#include "nano_dma.h"

void nano_dma_os_claim(void);

void nano_dma_os_claim(void)
{
	dma_claim_mask((1u << NANO_DMA_CHANNELS) - 1u);
}
