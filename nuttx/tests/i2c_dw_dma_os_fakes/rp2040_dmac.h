/* SPDX-License-Identifier: MIT */
/* arch/arm/src/rp2040/rp2040_dmac.h (NuttX 13.1.0) for the host test of the
 * I2C core's i2c_dw_dma_os_rp2040.c: the types, values and functions it uses, with
 * NuttX's names; the test implements the functions as rp2040_dmac.c does
 * them, on its array of registers, and records the calls.
 */
#ifndef TEST_I2C_DW_DMA_RP2040_DMAC_H
#define TEST_I2C_DW_DMA_RP2040_DMAC_H
#include <stddef.h>
#include <stdint.h>
#include "hardware/rp2040_dma.h"
#define RP2040_DMA_NCHANNELS     12
#define RP2040_DMA_DREQ_I2C0_TX  32
#define RP2040_DMA_DREQ_I2C0_RX  33
#define RP2040_DMA_SIZE_BYTE     0
#define RP2040_DMA_SIZE_HALFWORD 1
#define RP2040_DMA_SIZE_WORD     2
typedef void *DMA_HANDLE;
typedef void (*dma_callback_t)(DMA_HANDLE handle, uint8_t status, void *arg);
typedef struct {
	uint8_t dreq;
	uint8_t size;
	uint8_t noincr;
} dma_config_t;
DMA_HANDLE rp2040_dmachannel(void);
DMA_HANDLE rp2040_dmachannel_trywait(void); /* the local patch nuttx/patches/nuttx/0011 */
void rp2040_dmafree(DMA_HANDLE handle);
void rp2040_rxdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr, size_t nbytes,
		       dma_config_t config);
void rp2040_txdmasetup(DMA_HANDLE handle, uintptr_t paddr, uintptr_t maddr, size_t nbytes,
		       dma_config_t config);
void rp2040_dmastart(DMA_HANDLE handle, dma_callback_t callback, void *arg);
void rp2040_dmastop(DMA_HANDLE handle);
uintptr_t rp2040_dma_register(DMA_HANDLE handle, uint16_t offset);
#endif
