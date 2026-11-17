/* SPDX-License-Identifier: MIT */
/* The prelude of the sources shared with the Zephyr build (common/), which
 * the NuttX build includes before each of them (-include, as the Zephyr
 * build reads its configuration into every file with -imacros autoconf.h):
 *
 * - NuttX's configuration, <nuttx/config.h>,
 * - the NuttX headers that must come before any header of the Zephyr
 *   compatibility layer: the layer's <zephyr/sys/atomic.h> takes over
 *   NuttX's atomic names, so NuttX's headers that use them (mutexes,
 *   semaphores and the headers that include those) have to be read first.
 *   These are the ones that the layer's <zephyr/kernel.h> reads first; a
 *   shared source then needs no NuttX header of its own.  The socket headers are not here
 *   (their names, e.g. accept(), would collide with the sources' own):
 *   <zephyr/net/socket.h> of the layer reads them,
 * - the values of the Zephyr build's Kconfig options that the shared
 *   sources use and that have no NuttX option of their own
 *   (zephyr/build/usb-console/zephyr/.config).
 *
 * The host tests include it too; without __NuttX__ only the Kconfig values
 * are defined.  Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_COMMON_PRELUDE_H
#define __NUTTX_DRIVERS_COMPAT_COMMON_PRELUDE_H

#ifdef __NuttX__
#  include <nuttx/config.h>
#  include <sys/types.h>
#  include <nuttx/irq.h>
#  include <nuttx/mutex.h>
#  include <nuttx/semaphore.h>
#  include <nuttx/wdog.h>
#  include <stdio.h>
#endif

/* The Zephyr build's CONFIG_SENSOR_LOG_LEVEL (3, info): the log level of
 * lsm6dso.c, lsm6dso_compat.c and sensor_shell.c.
 */

#define CONFIG_SENSOR_LOG_LEVEL 3

/* The Zephyr build's names of the port's NuttX options that the shared
 * sources use: the NTP server, port and interval of the time
 * synchronization (time_sync.c), CONFIG_NANO_NINA_NTP_* of
 * nuttx/apps/imu/Kconfig.
 */

#ifdef CONFIG_NANO_NINA_NTP_PORT
#  define CONFIG_NINA_NTP_SERVER      CONFIG_NANO_NINA_NTP_SERVER
#  define CONFIG_NINA_NTP_PORT        CONFIG_NANO_NINA_NTP_PORT
#  define CONFIG_NINA_NTP_INTERVAL_MS CONFIG_NANO_NINA_NTP_INTERVAL_MS
#endif

/* The FIFO stream of the LSM6DSO driver, CONFIG_SENSOR_ASYNC_API in the
 * Zephyr build (lsm6dso.c, lsm6dso.h): CONFIG_NANO_LSM6DSO_STREAM of
 * nuttx/drivers/lsm6dso/Kconfig.
 */

#ifdef CONFIG_NANO_LSM6DSO_STREAM
#  define CONFIG_SENSOR_ASYNC_API 1
#endif

#endif /* __NUTTX_DRIVERS_COMPAT_COMMON_PRELUDE_H */
