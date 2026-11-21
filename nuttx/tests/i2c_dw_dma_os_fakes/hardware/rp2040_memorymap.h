/* SPDX-License-Identifier: MIT */
/* arch/arm/src/rp2040/hardware/rp2040_memorymap.h for the host test of the
 * I2C core's i2c_dw_dma_os_rp2040.c: the DMA's base is the test's array of registers,
 * and setbits_reg32() and clrbits_reg32() (writes to the atomic set and clear
 * aliases) are the test's functions.
 */
#ifndef TEST_I2C_DW_DMA_RP2040_MEMORYMAP_H
#define TEST_I2C_DW_DMA_RP2040_MEMORYMAP_H
#include <stdint.h>
extern uint32_t fake_dma_regs[0x460 / 4];
#define RP2040_DMA_BASE ((uintptr_t)fake_dma_regs)
void setbits_reg32(uint32_t value, uintptr_t addr);
void clrbits_reg32(uint32_t value, uintptr_t addr);
#endif
