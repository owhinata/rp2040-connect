/* SPDX-License-Identifier: MIT */
/* The Zephyr part of the Wi-Fi profile (common/imu/wifi_profile.c, which
 * includes this header after its own objects): the reconnection thread's
 * ID, defined at the end of that file by K_THREAD_DEFINE(), which makes it
 * a constant.  The NuttX build's is nuttx/apps/wifi/wifi_profile_os.h.
 */

#ifndef WIFI_PROFILE_OS_H_
#define WIFI_PROFILE_OS_H_

#include <zephyr/kernel.h>

extern const k_tid_t wifi_reconnect_tid;

#endif /* WIFI_PROFILE_OS_H_ */
