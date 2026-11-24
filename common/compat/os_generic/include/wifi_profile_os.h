/* SPDX-License-Identifier: MIT */
/* The OS part of the Wi-Fi profile on the compatibility layer
 * (common/imu/wifi_profile.c, which
 * includes this header after its own objects): the reconnection thread's
 * ID, defined at the end of that file by K_THREAD_DEFINE().  Zephyr's is a
 * constant (zephyr/app/usb-console/src/wifi_profile_os.h); the
 * compatibility layer's K_THREAD_DEFINE() defines it without const, because
 * the thread's start (z_thread_static_start(), when the task "main" starts
 * the thread) stores the OS's thread handle in it, as on NuttX
 * (nuttx/apps/wifi/wifi_profile_os.h).
 */

#ifndef WIFI_PROFILE_OS_H_
#define WIFI_PROFILE_OS_H_

#include <zephyr/kernel.h>

extern k_tid_t wifi_reconnect_tid;

#endif /* WIFI_PROFILE_OS_H_ */
