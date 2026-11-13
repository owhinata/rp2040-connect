/* SPDX-License-Identifier: Apache-2.0 */
#include "i2c_dw_rp2040_commands.h"
#include <errno.h>
#define RP2040_I2C_CMD_READ    (1U << 8)
#define RP2040_I2C_CMD_STOP    (1U << 9)
#define RP2040_I2C_CMD_RESTART (1U << 10)
int i2c_dw_rp2040_read_commands(uint16_t *commands, size_t capacity, size_t length, bool restart)
{
	if (!commands || !length || length > capacity) {
		return -EINVAL;
	}
	for (size_t i = 0; i < length; i++) {
		commands[i] = RP2040_I2C_CMD_READ;
	}
	if (restart) {
		commands[0] |= RP2040_I2C_CMD_RESTART;
	}
	commands[length - 1] |= RP2040_I2C_CMD_STOP;
	return 0;
}
