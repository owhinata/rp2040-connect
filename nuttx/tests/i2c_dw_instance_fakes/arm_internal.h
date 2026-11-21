/* SPDX-License-Identifier: MIT */
/* arch/arm/src/common/arm_internal.h for the host test of the I2C core's
 * NuttX binding (tests/i2c_dw_instance.c, which implements these on its fake
 * RESETS and PADS_BANK0 registers and records the writes).
 */
#ifndef TEST_I2C_DW_NUTTX_ARM_INTERNAL_H
#define TEST_I2C_DW_NUTTX_ARM_INTERNAL_H
#include <stdint.h>
uint32_t getreg32(uintptr_t addr);
void setbits_reg32(uint32_t bits, uintptr_t addr);
void clrbits_reg32(uint32_t bits, uintptr_t addr);
void modbits_reg32(uint32_t value, uint32_t mask, uintptr_t addr);
#endif
