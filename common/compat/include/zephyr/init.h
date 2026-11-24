/* SPDX-License-Identifier: MIT */
/* Zephyr's <zephyr/init.h>, which the LSM6DSO driver's Zephyr binding
 * (common/drivers/lsm6dso/port/zephyr/lsm6dso.c) includes.  It defines nothing here:
 * Zephyr's SYS_INIT() and init levels (POST_KERNEL) are not used outside
 * the Zephyr build's own devicetree instance macros; on NuttX the board
 * initializes the devices in its bring-up (device_init(), see
 * <zephyr/device.h>).  Part of the small Zephyr compatibility layer of the
 * port (common/compat).
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_INIT_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_INIT_H

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_INIT_H */
