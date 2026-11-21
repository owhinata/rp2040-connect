/* SPDX-License-Identifier: MIT */
/* arch/arm/src/rp2040/hardware/rp2040_memorymap.h for the host test of the
 * I2C core's NuttX binding (tests/i2c_dw_instance.c): the controllers' bases
 * are the test's arrays of registers.  The real addresses: I2C0 0x40044000,
 * I2C1 0x40048000 (the test checks the names that the core logs).
 */
#ifndef TEST_I2C_DW_NUTTX_RP2040_MEMORYMAP_H
#define TEST_I2C_DW_NUTTX_RP2040_MEMORYMAP_H
#include <stdint.h>
extern volatile uint32_t fake_i2c0_regs[64];
extern volatile uint32_t fake_i2c1_regs[64];
#define RP2040_I2C0_BASE ((uintptr_t)fake_i2c0_regs)
#define RP2040_I2C1_BASE ((uintptr_t)fake_i2c1_regs)
#endif
