/* SPDX-License-Identifier: MIT */
/* The Zephyr build's main() after the NINA passthrough mode's branch, for
 * all three builds (app_main.h).  The settings are first initialized by
 * the Wi-Fi credentials' initialization before this (app_main.h, step 1);
 * kv_store_init() calls settings_subsys_init() again, which does nothing
 * more.
 */
#include <zephyr/logging/log.h>
#include "app_main.h"
#ifdef CONFIG_KV_STORE
#include "kv_store.h"
#endif
#ifdef CONFIG_APP_IMU
#include "imu_cal.h"
#include "time_sync.h"
#endif
#ifdef CONFIG_WIFI_PROFILE
#include "wifi_profile.h"
#endif

LOG_MODULE_REGISTER(usb_console, LOG_LEVEL_INF);

void app_main(void)
{
#if defined(CONFIG_KV_STORE) || defined(CONFIG_APP_IMU)
	int ret;
#endif

	LOG_INF("USB console started");
#ifdef CONFIG_KV_STORE
	ret = kv_store_init();
	if (ret) {
		LOG_ERR("Could not initialize settings: %d", ret);
	}
#endif
#ifdef CONFIG_APP_IMU
	ret = imu_cal_init();
	if (ret) {
		LOG_ERR("Could not set up the IMU calibration: %d", ret);
	}
	/* The time synchronization's Wi-Fi events (common/imu/time_sync.c):
	 * from a connection on, time_sync_tid asks the NTP server (the
	 * default gateway, port 123) every 2 s; a disconnection stops it.
	 * Before wifi_profile_init(), so that the automatic connection's
	 * event is seen.  Without CONFIG_TIME_SYNC, time_sync_null.c.
	 */
	time_sync_init();
#endif
#ifdef CONFIG_WIFI_PROFILE
	/* The automatic connection: the event callback, the migration of an
	 * old profile to the stored credentials, and a request to the
	 * reconnection thread, which connects to a stored network 4 s later
	 * and retries every 32 s while there is no address (nothing when no
	 * network is stored).
	 */
	(void)wifi_profile_init();
#endif
}
