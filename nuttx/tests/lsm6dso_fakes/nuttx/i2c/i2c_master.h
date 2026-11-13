/* SPDX-License-Identifier: Apache-2.0 */
/* The part of NuttX 13.1.0's <nuttx/i2c/i2c_master.h> that
 * compat/zephyr_i2c.c uses, with the same values (and the I2C_SPEED_* in
 * Hz that <zephyr/drivers/i2c.h> must replace).
 */
#ifndef TEST_FAKE_NUTTX_I2C_MASTER_H
#define TEST_FAKE_NUTTX_I2C_MASTER_H
#include <stdint.h>
#include <sys/types.h>
#define I2C_SPEED_STANDARD   100000
#define I2C_SPEED_FAST       400000
#define I2C_SPEED_FAST_PLUS  1000000
#define I2C_SPEED_HIGH       3400000
#define I2C_M_READ           0x0001
#define I2C_M_TEN            0x0002
#define I2C_M_NOSTOP         0x0040
#define I2C_M_NOSTART        0x0080
struct i2c_msg_s {
	uint32_t frequency;
	uint16_t addr;
	uint16_t flags;
	uint8_t *buffer;
	ssize_t length;
};
struct i2c_master_s;
struct i2c_ops_s {
	int (*transfer)(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count);
};
struct i2c_master_s {
	const struct i2c_ops_s *ops;
};
#define I2C_TRANSFER(d, m, c) ((d)->ops->transfer(d, m, c))
#endif
