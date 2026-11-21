/* SPDX-License-Identifier: MIT */
/* "arm_internal.h" for the host test of rp2040_dmac.c: the register
 * accesses, on the test's model of the DMA's registers.
 */
#ifndef TEST_RP2040_DMAC_ARM_INTERNAL_H
#define TEST_RP2040_DMAC_ARM_INTERNAL_H
#include <stdint.h>
uint32_t getreg32(uintptr_t addr);
void putreg32(uint32_t value, uintptr_t addr);
#endif
