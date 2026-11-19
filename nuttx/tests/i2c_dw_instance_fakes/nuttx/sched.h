/* SPDX-License-Identifier: MIT */
/* <nuttx/sched.h> for the host test of the board's I2C part
 * (tests/i2c_dw_instance.c).
 */
#ifndef TEST_I2C_DW_NUTTX_SCHED_H
#define TEST_I2C_DW_NUTTX_SCHED_H
#include <unistd.h>
static inline void nxsched_msleep(unsigned int ms)
{
	usleep(ms * 1000u);
}
#endif
