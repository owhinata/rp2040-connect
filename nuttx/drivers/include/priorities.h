/* SPDX-License-Identifier: Apache-2.0 */
/* Thread priorities and stack sizes of the port, from the Zephyr build
 * (zephyr/app/usb-console and the Zephyr subsystems it enables) with
 *
 *   NuttX priority = 118 - Zephyr priority
 *
 * Zephyr: 16 cooperative (-16..-1) and 15 preemptible (0..14) application
 * priorities, idle at 15; a lower number runs first.  NuttX: 1..255, a
 * higher number runs first.  118 - p keeps the order and puts every Zephyr
 * application thread (0..14 -> 118..104) above nsh's default 100.
 * Cooperative Zephyr threads (negative) map above 118; NuttX cannot keep
 * them from being preempted by a higher one (Zephyr does not preempt a
 * cooperative thread at all), see nuttx/README.md.
 *
 * Shared by the kernel side (nuttx/drivers, the board code) and the
 * applications: $(TOPDIR)/external/include.  The stack sizes are those of
 * the Zephyr build.  As there, interrupts have a stack of their own
 * (CONFIG_ARCH_INTERRUPTSTACK=2048 in the imu defconfig, Zephyr's
 * CONFIG_ISR_STACK_SIZE): a thread's stack only takes the frame the
 * hardware saves when it is interrupted.
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_PRIORITIES_H
#define __NUTTX_DRIVERS_INCLUDE_PRIORITIES_H

#define NANO_PRIO_FROM_ZEPHYR(p)     (118 - (p))

/* Application threads (zephyr/app/usb-console). */

/* src/wifi_profile.c:186 K_THREAD_DEFINE(wifi_reconnect_tid, 2048, .., 8) */

#define NANO_PRIO_WIFI_RECONNECT     NANO_PRIO_FROM_ZEPHYR(8)   /* 110 */
#define NANO_STACK_WIFI_RECONNECT    2048

/* drivers/nina/nina_net.c:140 nina_scan_tid (1536, 8), :274
 * nina_connect_tid (1024, 8), :402 nina_monitor_tid (1536, 9).
 */

#define NANO_PRIO_NINA_SCAN          NANO_PRIO_FROM_ZEPHYR(8)   /* 110 */
#define NANO_STACK_NINA_SCAN         1536
#define NANO_PRIO_NINA_CONNECT       NANO_PRIO_FROM_ZEPHYR(8)   /* 110 */
#define NANO_STACK_NINA_CONNECT      1024
#define NANO_PRIO_NINA_MONITOR       NANO_PRIO_FROM_ZEPHYR(9)   /* 109 */
#define NANO_STACK_NINA_MONITOR      1536

/* src/time_sync.c:295 time_sync_tid (2048, 9) */

#define NANO_PRIO_TIME_SYNC          NANO_PRIO_FROM_ZEPHYR(9)   /* 109 */
#define NANO_STACK_TIME_SYNC         2048

/* src/imu_stream.c:579 imu_stream_tid (2048, 9) */

#define NANO_PRIO_IMU_STREAM         NANO_PRIO_FROM_ZEPHYR(9)   /* 109 */
#define NANO_STACK_IMU_STREAM        2048

/* drivers/lsm6dso/lsm6dso_stream.c:279 the "imu_fifo_worker" work queue,
 * K_PRIO_PREEMPT(10), STREAM_WORK_STACK_SIZE 2048.
 */

#define NANO_PRIO_IMU_FIFO_WORKER    NANO_PRIO_FROM_ZEPHYR(10)  /* 108 */
#define NANO_STACK_IMU_FIFO_WORKER   2048

/* src/imu_raw_mqtt.c:349 imu_raw_mqtt_tid (2048, 11) */

#define NANO_PRIO_IMU_RAW_MQTT       NANO_PRIO_FROM_ZEPHYR(11)  /* 107 */
#define NANO_STACK_IMU_RAW_MQTT      2048

/* "nina_event", the kernel thread that turns NINA socket polling into
 * usrsock events (nuttx/drivers/nina/nina_poller.c).  No Zephyr thread:
 * Zephyr polls in the caller of connect() and poll() (imu_raw_mqtt_tid 11
 * -> 107, net_socket_service 14 -> 104 for Telnet).  106: below the
 * application threads, above the shell.  The stack holds a NINA request
 * and usrsock's event delivery (poll and wait callbacks).
 */

#define NANO_PRIO_NINA_EVENT         106
#define NANO_STACK_NINA_EVENT        1536

/* Zephyr's own threads ------------------------------------------------- */

/* main(): CONFIG_MAIN_THREAD_PRIORITY=0, CONFIG_MAIN_STACK_SIZE=2048.  The
 * NuttX init task (CONFIG_INIT_PRIORITY, CONFIG_INIT_STACKSIZE) does what
 * main() does, at this priority, then drops to NANO_PRIO_SHELL to look
 * after the USB console (nuttx/apps/init).
 */

#define NANO_PRIO_MAIN               NANO_PRIO_FROM_ZEPHYR(0)   /* 118 */
#define NANO_STACK_MAIN              2048

/* shell_uart and shell_telnet: K_LOWEST_APPLICATION_THREAD_PRIO (14),
 * CONFIG_SHELL_STACK_SIZE=3072.  Shell commands run in the shell thread, so
 * the nsh sessions and the builtin commands use this priority.
 */

#define NANO_PRIO_SHELL              NANO_PRIO_FROM_ZEPHYR(14)  /* 104 */
#define NANO_STACK_SHELL             3072

/* sensor_shell_processing_tid (drivers/sensor/sensor_shell_stream.c:38):
 * priority 0, CONFIG_SENSOR_SHELL_THREAD_STACK_SIZE=3072.
 */

#define NANO_PRIO_SENSOR_SHELL       NANO_PRIO_FROM_ZEPHYR(0)   /* 118 */
#define NANO_STACK_SENSOR_SHELL      3072

/* sysworkq: the system work queue, CONFIG_SYSTEM_WORKQUEUE_PRIORITY -1
 * (cooperative), CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE 1024.  The compatibility
 * layer's k_sys_work_q (nuttx/drivers/compat), which runs the LSM6DSO FIFO
 * interrupt's work (lsm6dso_fifo_irq.c).  Above every application thread,
 * as in Zephyr; NuttX can preempt it from a higher priority (none of the
 * port's threads).
 */

#define NANO_PRIO_SYSWORKQ           NANO_PRIO_FROM_ZEPHYR(-1)  /* 119 */
#define NANO_STACK_SYSWORKQ          1024

/* "RTIO WQ 0": subsys/rtio/rtio_workq.c, CONFIG_RTIO_WORKQ_THREADS_POOL_PRIO
 * 0, _STACK_SIZE 1024.  Runs the LSM6DSO's single RTIO reads
 * (lsm6dso_compat.c).
 */

#define NANO_PRIO_RTIO_WQ            NANO_PRIO_FROM_ZEPHYR(0)   /* 118 */
#define NANO_STACK_RTIO_WQ           1024

/* Not used by the port yet; for reference (see nuttx/README.md):
 *   net_mgmt       -1 (cooperative)  -> 119, 2048
 *   rx_q[0]        -1 (cooperative)  -> 119, 1504 (Zephyr's IP stack, idle)
 *   tcp_work      -14 (cooperative)  -> 132, 1024 (Zephyr's IP stack, idle)
 *   usbd, usbd@50110000 -8 (coop.)   -> 126, 1024 / 512 (NuttX: interrupt)
 *   logging        14                -> 104, 768 (NuttX: none, see app_log.h)
 *   net_socket_service 15 -> 14      -> 104, 2048 (NuttX: telnetd)
 *   zperf_work_q   15 -> 14          -> 104, 2048 (NuttX: iperf)
 *   idle           15                -> NuttX idle (0), 256 (NuttX 1024)
 */

#endif /* __NUTTX_DRIVERS_INCLUDE_PRIORITIES_H */
