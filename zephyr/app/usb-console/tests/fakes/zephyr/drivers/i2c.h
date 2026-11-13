/* SPDX-License-Identifier: Apache-2.0 */
#ifndef TEST_FAKE_I2C_H_
#define TEST_FAKE_I2C_H_
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
struct i2c_dt_spec { int unused; };
int i2c_burst_read_dt(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t *buf, size_t len);
#endif
