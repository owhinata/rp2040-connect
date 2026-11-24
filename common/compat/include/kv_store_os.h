/* SPDX-License-Identifier: MIT */
/* The part of common/imu/kv_store.c for the builds on the Zephyr
 * compatibility layer (NuttX, FreeRTOS; the Zephyr build's is
 * zephyr/app/usb-console/src/kv_store_os.h): the subcommand
 *
 *   kv wipe confirm
 *
 * which erases the whole key-value area (kv_store_wipe.c).  It is how the
 * area is reused when it holds something other than Zephyr's NVS (the
 * NuttX build's earlier format, an OpenMV file system, ...), which these
 * builds do not erase on their own (<zephyr/storage/flash_map.h>).  The
 * word "confirm" is required, so that a mistyped "kv" line does not erase
 * the calibration and the Wi-Fi credentials.
 */

#ifndef COMMON_COMPAT_INCLUDE_KV_STORE_OS_H
#define COMMON_COMPAT_INCLUDE_KV_STORE_OS_H

#include <stddef.h>

#include <zephyr/shell/shell.h>

/* Erase the whole key-value area, then initialize the settings on it again
 * (an empty store), all under the settings' lock.  0 or a negative errno.
 * The values and the Wi-Fi credentials that were read from the store
 * before (the IMU calibration, the credentials' SSID list) stay in memory
 * until the next start.
 */

int kv_store_wipe(void);

int kv_store_cmd_wipe(const struct shell *sh, size_t argc, char **argv);

#define KV_STORE_OS_COMMANDS                                                 \
	SHELL_CMD_ARG(wipe, NULL,                                            \
		      "Erase the whole store (also Wi-Fi credentials): confirm", \
		      kv_store_cmd_wipe, 2, 0),

#endif
