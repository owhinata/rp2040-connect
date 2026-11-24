/* SPDX-License-Identifier: MIT */
/* The prelude of the sources shared with the Zephyr and NuttX builds
 * (common/), which a build without Kconfig (FreeRTOS, ThreadX) includes
 * before each of them (-include, as the Zephyr build reads its
 * configuration into every file with -imacros autoconf.h, and the NuttX
 * build its common/compat/port/nuttx/common_prelude.h): the values of the
 * Zephyr build's Kconfig options that the shared sources use and that such
 * a build has no option of its own for
 * (zephyr/build/usb-console/zephyr/.config).  No OS header has to come
 * first (those of FreeRTOS and ThreadX take no name of the layer, see
 * zephyr_os_atomic.h).  Part of the compatibility layer for an OS without
 * the model of its own (common/compat/port/generic).
 *
 * The OS-independent part of the layer, the RAM log's ring, the shell's
 * zshell.c and subcmd.c and the copies of Zephyr's fnmatch() and getopt need
 * none (CONFIG_GETOPT_LONG, which changes a structure, is defined for every
 * source of the build, compat.cmake); the LSM6DSO driver's and the time
 * synchronization's are below (the same values as the NuttX build's
 * prelude and Kconfig); the options of the others are added with them.
 */

#ifndef COMMON_COMPAT_PORT_GENERIC_COMMON_PRELUDE_H
#define COMMON_COMPAT_PORT_GENERIC_COMMON_PRELUDE_H

/* The Zephyr build's CONFIG_SENSOR_LOG_LEVEL (3, info): the log level of
 * lsm6dso.c, lsm6dso_rtio.c and sensor_shell.c.
 */
#define CONFIG_SENSOR_LOG_LEVEL 3

/* The time synchronization (common/imu/time_sync.c): the NTP server (empty:
 * the default gateway), its UDP port and the interval between queries, the
 * Zephyr build's CONFIG_NINA_NTP_SERVER, CONFIG_NINA_NTP_PORT and
 * CONFIG_NINA_NTP_INTERVAL_MS (zephyr/app/usb-console/Kconfig; the NuttX
 * build has options of the same names).  The same values in the three
 * builds.
 */
#define CONFIG_NINA_NTP_SERVER      ""
#define CONFIG_NINA_NTP_PORT        123
#define CONFIG_NINA_NTP_INTERVAL_MS 2000

/* The NINA's sockets: the largest TCP send of one command and the wait
 * before a send that found no room is tried again (the core,
 * common/drivers/nina/internal/nina_sock.h, and its Zephyr binding
 * common/drivers/nina/port/zephyr/nina_socket.c), and the priority with which
 * the binding registers its sockets (NET_SOCKET_OFFLOAD_REGISTER() of the
 * layer's <zephyr/net/socket.h>): the Zephyr build's CONFIG_NINA_TCP_MAX_IO,
 * CONFIG_NINA_SEND_RETRY_MS (common/drivers/nina/port/zephyr/Kconfig) and
 * CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY.
 */
#define CONFIG_NINA_TCP_MAX_IO              3584
#define CONFIG_NINA_SEND_RETRY_MS           1
#define CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY 40

/* The NINA passthrough mode (common/drivers/nina/nina_passthrough.c, the
 * Zephyr build's CONFIG_NINA_PASSTHROUGH, zephyr/app/usb-console/Kconfig):
 * "nina passthrough" of the shared "nina" command
 * (common/shell/nina_shell.c).  The builds on the Pico SDK always have the
 * mode (common/drivers/nina/port/pico_sdk/nina.cmake).
 */
#define CONFIG_NINA_PASSTHROUGH 1

/* The FIFO stream of the LSM6DSO driver, CONFIG_SENSOR_ASYNC_API in the
 * Zephyr build (lsm6dso.c, lsm6dso.h): on with the driver
 * (CONFIG_LSM6DSO_STREAM, which common/drivers/lsm6dso/port/pico_sdk/lsm6dso.cmake
 * defines for every source of the build, as the NuttX build's Kconfig
 * option of that name).  It changes the driver's structures, so a build's
 * own source that includes lsm6dso.h is compiled with this prelude too.
 */
#ifdef CONFIG_LSM6DSO_STREAM
#define CONFIG_SENSOR_ASYNC_API 1
#endif

#endif /* COMMON_COMPAT_PORT_GENERIC_COMMON_PRELUDE_H */
