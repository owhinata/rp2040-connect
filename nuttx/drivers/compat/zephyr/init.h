/* SPDX-License-Identifier: MIT */
/* Zephyr's <zephyr/init.h>, which the ported LSM6DSO driver
 * (common/drivers/lsm6dso/lsm6dso.c) includes.  It defines nothing here:
 * Zephyr's SYS_INIT() and init levels (POST_KERNEL) are not used outside
 * the Zephyr build's own devicetree instance macros; on NuttX the board
 * initializes the devices in its bring-up (device_init(), see
 * <zephyr/device.h>).  Part of the small Zephyr compatibility layer of the
 * port (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_INIT_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_INIT_H

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_INIT_H */
