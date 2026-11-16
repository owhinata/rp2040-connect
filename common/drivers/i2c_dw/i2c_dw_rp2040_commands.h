/* SPDX-License-Identifier: Apache-2.0 */
#ifndef I2C_DW_RP2040_COMMANDS_H_
#define I2C_DW_RP2040_COMMANDS_H_
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
int i2c_dw_rp2040_read_commands(uint16_t *commands, size_t capacity, size_t length, bool restart);
#endif
