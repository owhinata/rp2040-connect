/* SPDX-License-Identifier: MIT */
/* What the Zephyr build's kernel does for the application before main(),
 * for the builds whose kernel does not (app_main.h): the application's part
 * of SYS_INIT(POST_KERNEL) and the table of the K_THREAD_DEFINE() threads.
 * The Zephyr build does not compile this file.
 */
#include <zephyr/logging/log.h>
#include "app_main.h"
#include "zephyr_compat.h"
#ifdef CONFIG_WIFI_CREDENTIALS
#include <zephyr/net/wifi_credentials.h>
#endif
#ifdef CONFIG_NINA_NET
#include "nina_net.h"
#endif

LOG_MODULE_DECLARE(usb_console, LOG_LEVEL_INF);

void app_init_post_kernel(void)
{
#ifdef CONFIG_WIFI_CREDENTIALS
	(void)wifi_credentials_init();
#endif
}

static void thread_started(int ret, const char *name)
{
	if (ret) {
		LOG_ERR("Could not start %s: %d", name, ret);
	}
}

/* One entry of the table: the thread that a source defined with
 * K_THREAD_DEFINE(name, ...).  common/tests/app_threads.py checks that the
 * table names every such thread once, each under the option of the module
 * that defines it.
 */
#define APP_THREAD_START(name) thread_started(K_THREAD_DEFINE_START(name), #name)

/* The order of creation is the order of the table: it is the order in
 * which the threads first run among those of one priority, and the order
 * of "ps".  They have lower priorities than the calling thread, or its own
 * (the sensor shell's processing thread, Zephyr's 0), so they first run
 * when it waits.
 */
void app_threads_start(void)
{
	/* The NINA driver's (scan, connect, link monitor; the core's
	 * nina_wifi.c).  The Zephyr binding defines them
	 * (common/drivers/nina/port/zephyr/nina_net.c); the NuttX binding
	 * starts them itself (common/drivers/nina/port/nuttx/nina_netdev.c).
	 */
#ifdef CONFIG_WIFI_NINA_LOCAL
	APP_THREAD_START(nina_scan_tid);
	APP_THREAD_START(nina_connect_tid);
	APP_THREAD_START(nina_monitor_tid);
#endif
#ifdef CONFIG_NINA_NET
	thread_started(nina_net_start(), "nina_net_start");
#endif
	/* The IMU service's (common/imu/imu_stream.c): waits for "imu
	 * start".
	 */
#ifdef CONFIG_APP_IMU
	APP_THREAD_START(imu_stream_tid);
#endif
	/* Its MQTT transport's (common/imu/imu_raw_mqtt.c). */
#ifdef CONFIG_IMU_RAW_MQTT
	APP_THREAD_START(imu_raw_mqtt_tid);
#endif
	/* The time synchronization's (common/imu/time_sync.c): waits for the
	 * Wi-Fi events of time_sync_init().
	 */
#ifdef CONFIG_TIME_SYNC
	APP_THREAD_START(time_sync_tid);
#endif
	/* The Wi-Fi profile's reconnection (common/imu/wifi_profile.c). */
#ifdef CONFIG_WIFI_PROFILE
	APP_THREAD_START(wifi_reconnect_tid);
#endif
	/* The sensor shell's processing thread
	 * (common/compat/sensor_shell_stream.c): waits for the completions
	 * of "sensor get" and "sensor stream".
	 */
#ifdef CONFIG_APP_SENSOR
	APP_THREAD_START(sensor_shell_processing_tid);
#endif
	/* The iperf3 server of "zperf tcp download"
	 * (common/compat/zperf_iperf3.h), Zephyr's net_socket_service
	 * thread: waits until the server is started.
	 */
#ifdef CONFIG_NET_ZPERF
	APP_THREAD_START(zperf_server_tid);
#endif
}
