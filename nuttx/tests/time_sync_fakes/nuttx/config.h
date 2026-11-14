/* SPDX-License-Identifier: Apache-2.0 */
/* Stand-in for NuttX's generated <nuttx/config.h> (the host test
 * time_sync): the time synchronization's options with the imu
 * configuration's values (the Zephyr build's CONFIG_NINA_NTP_*).
 */
#ifndef TEST_FAKE_NUTTX_CONFIG_H
#define TEST_FAKE_NUTTX_CONFIG_H
#define CONFIG_NANO_TIME_SYNC 1
#define CONFIG_NANO_NINA_NTP_SERVER ""
#define CONFIG_NANO_NINA_NTP_PORT 123
#define CONFIG_NANO_NINA_NTP_INTERVAL_MS 2000
#endif
