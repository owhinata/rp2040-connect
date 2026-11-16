/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <errno.h>
#include "i2c_dw_rp2040_commands.h"

int main(void)
{
	uint16_t commands[3585] = {0};

	assert(i2c_dw_rp2040_read_commands(NULL, 4, 1, false) == -EINVAL);
	assert(i2c_dw_rp2040_read_commands(commands, 4, 0, false) == -EINVAL);
	assert(i2c_dw_rp2040_read_commands(commands, 4, 5, false) == -EINVAL);
	assert(i2c_dw_rp2040_read_commands(commands, 4, 1, true) == 0);
	assert(commands[0] == 0x700);
	assert(i2c_dw_rp2040_read_commands(commands, 4, 4, false) == 0);
	assert(commands[0] == 0x100 && commands[1] == 0x100 && commands[2] == 0x100 &&
	       commands[3] == 0x300);
	assert(i2c_dw_rp2040_read_commands(commands, 3584, 3584, true) == 0);
	assert(commands[0] == 0x500 && commands[3583] == 0x300 && commands[3584] == 0);
	for (size_t i = 1; i < 3583; i++) {
		assert(commands[i] == 0x100);
	}
	return 0;
}
