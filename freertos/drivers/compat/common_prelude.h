/* SPDX-License-Identifier: MIT */
/* The prelude of the sources shared with the Zephyr and NuttX builds
 * (common/), which the FreeRTOS build includes before each of them (-include,
 * as the Zephyr build reads its configuration into every file with -imacros
 * autoconf.h, and the NuttX build its nuttx/drivers/compat/common_prelude.h):
 * the values of the Zephyr build's Kconfig options that the shared sources
 * use and that the FreeRTOS build has no option of its own for
 * (zephyr/build/usb-console/zephyr/.config).  No FreeRTOS header has to come
 * first (none of them takes a name of the compatibility layer, see
 * zephyr_os_atomic.h).  Part of the FreeRTOS implementation of the small
 * Zephyr compatibility layer of the port (freertos/drivers/compat).
 *
 * The OS-independent part of the layer, the RAM log's ring, the shell's
 * zshell.c and subcmd.c and the copies of Zephyr's fnmatch() and getopt need
 * none (CONFIG_GETOPT_LONG, which changes a structure, is defined for every
 * source of the build, compat.cmake); the LSM6DSO driver's are below (the
 * same values as the NuttX build's prelude); the options of the others (the
 * NTP options of the time synchronization, ...) are added with them.
 */

#ifndef FREERTOS_DRIVERS_COMPAT_COMMON_PRELUDE_H
#define FREERTOS_DRIVERS_COMPAT_COMMON_PRELUDE_H

/* The Zephyr build's CONFIG_SENSOR_LOG_LEVEL (3, info): the log level of
 * lsm6dso.c, lsm6dso_compat.c and sensor_shell.c.
 */
#define CONFIG_SENSOR_LOG_LEVEL 3

/* The FIFO stream of the LSM6DSO driver, CONFIG_SENSOR_ASYNC_API in the
 * Zephyr build (lsm6dso.c, lsm6dso.h): on with the driver
 * (CONFIG_NANO_LSM6DSO_STREAM, which freertos/drivers/lsm6dso/lsm6dso.cmake
 * defines for every source of the build, as the NuttX build's Kconfig
 * option of that name).  It changes the driver's structures, so a FreeRTOS
 * source that includes lsm6dso.h is compiled with this prelude too.
 */
#ifdef CONFIG_NANO_LSM6DSO_STREAM
#define CONFIG_SENSOR_ASYNC_API 1
#endif

#endif /* FREERTOS_DRIVERS_COMPAT_COMMON_PRELUDE_H */
