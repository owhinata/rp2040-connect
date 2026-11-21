/* SPDX-License-Identifier: MIT */
/* <nuttx/compiler.h> for the host test of the I2C core's NuttX binding
 * (tests/i2c_dw_instance.c): inline_function as NuttX defines it with the
 * optimization on.
 */
#ifndef TEST_I2C_DW_NUTTX_COMPILER_H
#define TEST_I2C_DW_NUTTX_COMPILER_H
#define inline_function __attribute__((always_inline)) inline
#endif
