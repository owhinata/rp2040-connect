/* SPDX-License-Identifier: MIT */
/* arch/arm/src/common/arm_internal.h for the host test of the I2C core's
 * i2c_dw_dma_os_rp2040.c: getreg32() and putreg32() on the DMA's registers, which
 * the test keeps (tests/i2c_dw_dma_os.c: INTS0 and CHAN_ABORT as the
 * hardware treats a write).
 */
#ifndef TEST_I2C_DW_DMA_ARM_INTERNAL_H
#define TEST_I2C_DW_DMA_ARM_INTERNAL_H
#include <stdint.h>
uint32_t getreg32(uintptr_t addr);
void putreg32(uint32_t value, uintptr_t addr);
#endif
