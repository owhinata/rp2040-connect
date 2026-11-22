/* SPDX-License-Identifier: MIT */
/* The start of the application, the same in every build: what the Zephyr
 * build's kernel and its main() do for the application's modules, in one
 * order (zephyr/zephyr/kernel/init.c, bg_thread_main()):
 *
 *   1. SYS_INIT(POST_KERNEL), after the devices: app_init_post_kernel();
 *   2. SYS_INIT(APPLICATION): the shell's parts, which are the OS's (the
 *      FreeRTOS build's "wifi" command and Telnet shell);
 *   3. z_init_static_threads(): app_threads_start();
 *   4. main(): app_main().
 *
 * The Zephyr build calls only app_main(), from its main(); its kernel does
 * 1 to 3 itself.  A build whose kernel does not calls all four, in this
 * order, from a thread, after its devices (the I2C controllers, the
 * LSM6DSOX, the network interface) and its console.  The NuttX build
 * starts its Telnet shell (2) after 3 (nuttx/apps/init/rp2040_init.c).
 *
 * The modules a build may leave out are named by the NuttX build's options
 * (CONFIG_KV_STORE, CONFIG_APP_IMU, CONFIG_IMU_RAW_MQTT, CONFIG_TIME_SYNC,
 * CONFIG_WIFI_CREDENTIALS, CONFIG_WIFI_PROFILE, CONFIG_APP_SENSOR) and by
 * Zephyr's (CONFIG_WIFI_NINA_LOCAL, the NINA's Zephyr binding;
 * CONFIG_NET_ZPERF): a build defines the symbol of each module it builds
 * (Zephyr: its Kconfig; FreeRTOS: the module's .cmake file).  A module
 * whose symbol is not defined is not started.
 */

#ifndef APP_MAIN_H_
#define APP_MAIN_H_

/* The application's part of SYS_INIT(POST_KERNEL): the Wi-Fi credentials
 * library's initialization (CONFIG_WIFI_CREDENTIALS), which initializes the
 * settings (mounts NVS on the key-value area; data that is not Zephyr's
 * NVS is left alone, and the call fails with -EILSEQ) and reads the stored
 * SSIDs.  common/imu/app_init.c.
 */
void app_init_post_kernel(void);

/* The threads of K_THREAD_DEFINE() that Zephyr starts before main()
 * (z_init_static_threads()), in one table, each started after the one
 * before it; a failure is logged and the next is started.  Each waits for
 * its work at once.  common/imu/app_init.c.
 */
void app_threads_start(void);

/* The Zephyr build's main() after the NINA passthrough mode's branch: logs
 * "USB console started", then the key-value store, the IMU calibration,
 * the time synchronization's Wi-Fi events and the Wi-Fi profile (the
 * automatic connection).  Returns when done.  common/imu/app_main.c.
 */
void app_main(void);

#endif /* APP_MAIN_H_ */
