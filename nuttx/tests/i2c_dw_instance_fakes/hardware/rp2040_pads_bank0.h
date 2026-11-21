/* SPDX-License-Identifier: MIT */
/* arch/arm/src/rp2040/hardware/rp2040_pads_bank0.h for the host test of the
 * I2C core's NuttX binding: NuttX's names and values.
 */
#ifndef TEST_I2C_DW_NUTTX_RP2040_PADS_BANK0_H
#define TEST_I2C_DW_NUTTX_RP2040_PADS_BANK0_H
#define RP2040_PADS_BANK0_GPIO(n)        ((uintptr_t)0x4001c004 + 4 * (n))
#define RP2040_PADS_BANK0_GPIO_IE        (1 << 6)
#define RP2040_PADS_BANK0_GPIO_DRIVE_4MA (0x1 << 4)
#endif
