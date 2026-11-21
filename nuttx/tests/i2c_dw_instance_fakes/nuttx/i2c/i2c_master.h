/* SPDX-License-Identifier: MIT */
/* <nuttx/i2c/i2c_master.h> for the host test of the I2C core's NuttX binding
 * (tests/i2c_dw_instance.c): the definitions of NuttX 13.1.0's header that
 * the binding uses, with NuttX's values and members.
 */
#ifndef TEST_I2C_DW_NUTTX_I2C_MASTER_H
#define TEST_I2C_DW_NUTTX_I2C_MASTER_H
#include <stdint.h>
#include <sys/types.h>
#define I2C_M_READ    0x0001
#define I2C_M_TEN     0x0002
#define I2C_M_NOSTOP  0x0040
#define I2C_M_NOSTART 0x0080
#define I2C_SPEED_STANDARD  100000
#define I2C_SPEED_FAST      400000
#define I2C_SPEED_FAST_PLUS 1000000
#define I2C_SPEED_HIGH      3400000
#define I2C_TRANSFER(d, m, c) ((d)->ops->transfer(d, m, c))
#define I2C_RESET(d)          ((d)->ops->reset(d))
struct i2c_master_s;
struct i2c_msg_s;
struct i2c_ops_s {
	int (*transfer)(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
	int (*reset)(struct i2c_master_s *dev);
#endif
	int (*setup)(struct i2c_master_s *dev);
	int (*shutdown)(struct i2c_master_s *dev);
};
struct i2c_msg_s {
	uint32_t frequency;
	uint16_t addr;
	uint16_t flags;
	uint8_t *buffer;
	ssize_t length;
};
struct i2c_master_s {
	const struct i2c_ops_s *ops;
};
#endif
