/* SPDX-License-Identifier: MIT */
/* "hardware/rp2040_memorymap.h" for the host test of rp2040_dmac.c: the
 * DMA's base, and the writes through a register's atomic set and clear
 * aliases, on the test's model.
 */
#ifndef TEST_RP2040_DMAC_MEMORYMAP_H
#define TEST_RP2040_DMAC_MEMORYMAP_H
#include <stdint.h>
#define RP2040_DMA_BASE 0x50000000
void setbits_reg32(uint32_t value, uintptr_t addr);
void clrbits_reg32(uint32_t value, uintptr_t addr);
void modbits_reg32(uint32_t value, uint32_t mask, uintptr_t addr);
#endif
