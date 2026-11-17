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
 * The shared sources that the FreeRTOS build compiles so far (the
 * OS-independent part of the layer, the RAM log's ring, the shell's
 * zshell.c and subcmd.c, the copies of Zephyr's fnmatch() and getopt) need
 * none (CONFIG_GETOPT_LONG, which changes a structure, is defined for every
 * source of the build, compat.cmake); the
 * options of the others (CONFIG_SENSOR_LOG_LEVEL of the LSM6DSO driver, the
 * NTP options of the time synchronization, ...) are added with them.
 */

#ifndef FREERTOS_DRIVERS_COMPAT_COMMON_PRELUDE_H
#define FREERTOS_DRIVERS_COMPAT_COMMON_PRELUDE_H

#endif /* FREERTOS_DRIVERS_COMPAT_COMMON_PRELUDE_H */
