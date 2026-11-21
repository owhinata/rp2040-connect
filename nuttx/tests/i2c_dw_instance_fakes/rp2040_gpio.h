/* SPDX-License-Identifier: MIT */
/* arch/arm/src/rp2040/rp2040_gpio.h for the host test of the I2C core's NuttX
 * binding (tests/i2c_dw_instance.c implements the functions: they record the
 * calls).  NuttX's names and signatures.
 */
#ifndef TEST_I2C_DW_NUTTX_RP2040_GPIO_H
#define TEST_I2C_DW_NUTTX_RP2040_GPIO_H
#include <stdbool.h>
#include <stdint.h>
#define RP2040_GPIO_FUNC_I2C 3
void rp2040_gpio_init(uint32_t gpio);
void rp2040_gpio_set_function(uint32_t gpio, uint32_t func);
void rp2040_gpio_set_pulls(uint32_t gpio, int up, int down);
void rp2040_gpio_set_drive_strength(uint32_t gpio, uint32_t drive_strength);
void rp2040_gpio_set_slew_fast(uint32_t gpio, bool fast);
void rp2040_gpio_set_input_hysteresis_enabled(uint32_t gpio, bool enabled);
#endif
