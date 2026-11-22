/* SPDX-License-Identifier: Apache-2.0 */
/* The time synchronization's null implementation, for a build without
 * CONFIG_TIME_SYNC (with it, time_sync.c is built instead).  The IMU service
 * (CONFIG_APP_IMU) needs time_sync.h in every build: its monotonic clock,
 * time_sync_mono_us(), stamps the batches, and app_main() calls
 * time_sync_init() (app_main.h).  OS-independent; the NuttX build selects it
 * (nuttx/apps/imu/Makefile).
 *
 * The API of time_sync.h as the Zephyr build's time_sync.c behaves before
 * its first NTP reply: no server time estimate exists, so
 * time_sync_is_ready() is false and time_sync_unix_us() and
 * time_sync_estimate_us() return false (the IMU service then counts
 * unix_unavailable for each batch). time_sync_mono_us() is already the real
 * one: the RP2040's 1 MHz TIMER read through its raw registers, the same read
 * as the Zephyr build's (rp2040_timer_us() of common/chip/rp2040).
 */
#include "rp2040_timer_us.h"
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
	return (int64_t)rp2040_timer_us();
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
