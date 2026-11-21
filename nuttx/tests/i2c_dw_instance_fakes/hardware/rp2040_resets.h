/* SPDX-License-Identifier: MIT */
/* arch/arm/src/rp2040/hardware/rp2040_resets.h for the host test of the I2C
 * core's NuttX binding: the bits as NuttX's, the registers as the test's
 * addresses (tests/i2c_dw_instance.c).
 */
#ifndef TEST_I2C_DW_NUTTX_RP2040_RESETS_H
#define TEST_I2C_DW_NUTTX_RP2040_RESETS_H
#define RP2040_RESETS_RESET      ((uintptr_t)0x4000c000)
#define RP2040_RESETS_RESET_DONE ((uintptr_t)0x4000c008)
#define RP2040_RESETS_RESET_I2C0 (1 << 3)
#define RP2040_RESETS_RESET_I2C1 (1 << 4)
#endif
