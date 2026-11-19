/* SPDX-License-Identifier: MIT */
/* <nuttx/clock.h> for the host test of the board's I2C part
 * (tests/i2c_dw_instance.c): ticks of 1 ms on the host's monotonic clock.
 */
#ifndef TEST_I2C_DW_NUTTX_CLOCK_H
#define TEST_I2C_DW_NUTTX_CLOCK_H
#include <stdint.h>
#include <time.h>
#define MSEC2TICK(ms) ((clock_t)(ms))
#define TICK2MSEC(t)  ((int64_t)(t))
static inline clock_t clock_systime_ticks(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (clock_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}
#endif
