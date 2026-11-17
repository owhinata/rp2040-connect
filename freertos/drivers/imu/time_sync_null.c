/* SPDX-License-Identifier: Apache-2.0 */
/* STUB of the FreeRTOS build until it has the time synchronization (then
 * common/imu/time_sync.c, shared with the Zephyr and NuttX builds, and
 * time_offset.c are built instead: freertos/drivers/imu/imu.cmake).  A copy
 * of the NuttX build's stub (nuttx/apps/imu/time_sync_null.c, its build
 * without NANO_TIME_SYNC), on FreeRTOS's clock instead of NuttX's.
 *
 * The API of time_sync.h as the Zephyr build's time_sync.c behaves before
 * its first NTP reply: no server time estimate exists, so
 * time_sync_is_ready() is false and time_sync_unix_us() and
 * time_sync_estimate_us() return false (the IMU service then counts
 * unix_unavailable for each batch). time_sync_mono_us() is already the real
 * one: the RP2040's 1 MHz TIMER read through its raw registers, the same read
 * as the Zephyr build's (nano_mono_us(), os_clock.h of the application:
 * time_us_64(), TIMERAWH, TIMERAWL, TIMERAWH until the high words agree).
 */
#include "os_clock.h"

#include "time_sync.h"

void time_sync_init(void)
{
}

bool time_sync_is_ready(void)
{
	return false;
}

int64_t time_sync_mono_us(void)
{
	return (int64_t)nano_mono_us();
}

bool time_sync_unix_us(int64_t mono_us, int64_t *unix_us, uint32_t *delay_us)
{
	(void)mono_us;
	(void)unix_us;
	(void)delay_us;
	return false;
}

bool time_sync_estimate_us(int64_t mono_us, int64_t *unix_us, uint32_t *delay_us)
{
	(void)mono_us;
	(void)unix_us;
	(void)delay_us;
	return false;
}
