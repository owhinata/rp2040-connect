/* SPDX-License-Identifier: Apache-2.0 */
/* Thread priorities and stack sizes of the port, from the Zephyr build
 * (zephyr/app/usb-console and the Zephyr subsystems it enables): each
 * thread's Zephyr priority p as the OS's NANO_PRIO_FROM_ZEPHYR(p), and its
 * Zephyr stack size.
 *
 * Zephyr: 16 cooperative (-16..-1) and 15 preemptible (0..14) application
 * priorities, idle at 15; a lower number runs first.  The OS's mapping
 * keeps the order.  As in the Zephyr build, interrupts have a stack of
 * their own (Zephyr's CONFIG_ISR_STACK_SIZE): a thread's stack only takes
 * the frame the hardware saves when it is interrupted.
 *
 * The OS part: "priorities_os.h", which this header includes first, from
 * the OS's directory (nuttx/drivers/include on NuttX), defines
 * NANO_PRIO_FROM_ZEPHYR(p), the OS's priority of Zephyr's priority p
 * (-16..14, keeping the order), and the priorities and stacks of the OS's
 * own threads.  The shared part of the Zephyr compatibility layer
 * (common/compat) and the OS's threads use the values below.
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_PRIORITIES_H
#define __NUTTX_DRIVERS_INCLUDE_PRIORITIES_H

#include "priorities_os.h"

/* Application threads (zephyr/app/usb-console and common/). */

/* common/imu/wifi_profile.c:186
 * K_THREAD_DEFINE(wifi_reconnect_tid, 2048, .., 8)
 */

#define NANO_PRIO_WIFI_RECONNECT     NANO_PRIO_FROM_ZEPHYR(8)
#define NANO_STACK_WIFI_RECONNECT    2048

/* The NINA core's threads (common/drivers/nina/include/nina_wifi.h): K_THREAD_DEFINE()
 * of common/drivers/nina/port/zephyr/nina_net.c, nina_scan_tid (1536, 8),
 * nina_connect_tid (1024, 8), nina_monitor_tid (1536, 9); the NuttX
 * binding (common/drivers/nina/port/nuttx/nina_netdev.c) starts them with these.
 */

#define NANO_PRIO_NINA_SCAN          NANO_PRIO_FROM_ZEPHYR(8)
#define NANO_STACK_NINA_SCAN         1536
#define NANO_PRIO_NINA_CONNECT       NANO_PRIO_FROM_ZEPHYR(8)
#define NANO_STACK_NINA_CONNECT      1024
#define NANO_PRIO_NINA_MONITOR       NANO_PRIO_FROM_ZEPHYR(9)
#define NANO_STACK_NINA_MONITOR      1536

/* common/imu/time_sync.c:295 time_sync_tid (2048, 9) */

#define NANO_PRIO_TIME_SYNC          NANO_PRIO_FROM_ZEPHYR(9)
#define NANO_STACK_TIME_SYNC         2048

/* common/imu/imu_stream.c:579 imu_stream_tid (2048, 9) */

#define NANO_PRIO_IMU_STREAM         NANO_PRIO_FROM_ZEPHYR(9)
#define NANO_STACK_IMU_STREAM        2048

/* common/drivers/lsm6dso/port/zephyr/lsm6dso_os.c:28 the "imu_fifo_worker" work queue,
 * K_PRIO_PREEMPT(10), WORK_Q_STACK_SIZE 2048.
 */

#define NANO_PRIO_IMU_FIFO_WORKER    NANO_PRIO_FROM_ZEPHYR(10)
#define NANO_STACK_IMU_FIFO_WORKER   2048

/* common/imu/imu_raw_mqtt.c:349 imu_raw_mqtt_tid (2048, 11) */

#define NANO_PRIO_IMU_RAW_MQTT       NANO_PRIO_FROM_ZEPHYR(11)
#define NANO_STACK_IMU_RAW_MQTT      2048

/* Zephyr's own threads ------------------------------------------------- */

/* main(): CONFIG_MAIN_THREAD_PRIORITY=0, CONFIG_MAIN_STACK_SIZE=2048.  The
 * OS's task that does what main() does (on NuttX the init task,
 * nuttx/apps/init).
 */

#define NANO_PRIO_MAIN               NANO_PRIO_FROM_ZEPHYR(0)
#define NANO_STACK_MAIN              2048

/* shell_uart and shell_telnet: K_LOWEST_APPLICATION_THREAD_PRIO (14),
 * CONFIG_SHELL_STACK_SIZE=3072.  Shell commands run in the shell thread, so
 * the OS's shell sessions and the commands use this priority.
 */

#define NANO_PRIO_SHELL              NANO_PRIO_FROM_ZEPHYR(14)
#define NANO_STACK_SHELL             3072

/* sensor_shell_processing_tid (drivers/sensor/sensor_shell_stream.c:38):
 * priority 0, CONFIG_SENSOR_SHELL_THREAD_STACK_SIZE=3072.
 */

#define NANO_PRIO_SENSOR_SHELL       NANO_PRIO_FROM_ZEPHYR(0)
#define NANO_STACK_SENSOR_SHELL      3072

/* sysworkq: the system work queue, CONFIG_SYSTEM_WORKQUEUE_PRIORITY -1
 * (cooperative), CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE 1024.  The compatibility
 * layer's k_sys_work_q, which runs the LSM6DSO FIFO interrupt's work
 * (lsm6dso_os.c).  Above every application thread, as in Zephyr.
 */

#define NANO_PRIO_SYSWORKQ           NANO_PRIO_FROM_ZEPHYR(-1)
#define NANO_STACK_SYSWORKQ          1024

/* "RTIO WQ 0": subsys/rtio/rtio_workq.c, CONFIG_RTIO_WORKQ_THREADS_POOL_PRIO
 * 0, _STACK_SIZE 1024.  Runs the LSM6DSO's single RTIO reads
 * (lsm6dso_rtio.c).
 */

#define NANO_PRIO_RTIO_WQ            NANO_PRIO_FROM_ZEPHYR(0)
#define NANO_STACK_RTIO_WQ           1024

/* "net_mgmt": the network event thread (subsys/net/ip/net_mgmt.c,
 * CONFIG_NET_MGMT_EVENT_THREAD, priority -1 (cooperative),
 * CONFIG_NET_MGMT_EVENT_STACK_SIZE 2048); the compatibility layer's
 * zephyr_net_mgmt.c.  It runs the Wi-Fi event callbacks.
 */

#define NANO_PRIO_NET_MGMT           NANO_PRIO_FROM_ZEPHYR(-1)
#define NANO_STACK_NET_MGMT          2048

/* "net_socket_service" (subsys/net/lib/sockets/sockets_service.c,
 * CONFIG_NET_SOCKETS_SERVICE_THREAD_PRIO 15, which Zephyr lowers to the
 * lowest application priority 14, CONFIG_NET_SOCKETS_SERVICE_STACK_SIZE
 * 2048): it accepts the Telnet clients (shell_telnet.c telnet_accept()).
 * The OS's Telnet server (on NuttX the "telnetd" task,
 * nuttx/apps/init/nano_telnet.c).
 */

#define NANO_PRIO_TELNET_SERVICE     NANO_PRIO_FROM_ZEPHYR(14)
#define NANO_STACK_TELNET_SERVICE    2048

#endif /* __NUTTX_DRIVERS_INCLUDE_PRIORITIES_H */
