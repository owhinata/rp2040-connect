/* SPDX-License-Identifier: Apache-2.0 */
/* The NuttX part of the port's thread priorities and stack sizes
 * (common/compat/include/priorities.h, which includes this header first):
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
 * Interrupts have a stack of their own (CONFIG_ARCH_INTERRUPTSTACK=2048 in
 * the imu defconfig, Zephyr's CONFIG_ISR_STACK_SIZE).
 *
 * The values of priorities.h on NuttX:
 *
 *   wifi_reconnect_tid, nina_wifi_tid                        8 -> 110
 *   time_sync_tid, imu_stream_tid                            9 -> 109
 *   imu_fifo_worker                                         10 -> 108
 *   imu_raw_mqtt_tid                                        11 -> 107
 *   main (the init task, nuttx/apps/init: it does what main() does at
 *   this priority, then drops to APP_PRIO_SHELL to look after the USB
 *   console), sensor shell processing, RTIO WQ 0             0 -> 118
 *   the shell (nsh sessions and the builtin commands), Telnet
 *   service ("telnetd", nuttx/apps/init/rp2040_telnet.c)      14 -> 104
 *   sysworkq, net_mgmt (NuttX can preempt them from a higher
 *   priority: none of the port's threads)                   -1 -> 119
 *
 * Shared by the kernel side (nuttx/drivers, the board code) and the
 * applications: $(TOPDIR)/external/include.
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_PRIORITIES_OS_H
#define __NUTTX_DRIVERS_INCLUDE_PRIORITIES_OS_H

#define APP_PRIO_FROM_ZEPHYR(p)     (118 - (p))

/* "nina_event", the kernel thread that turns NINA socket polling into
 * usrsock events (common/drivers/nina/port/nuttx/nina_poller.c).  No Zephyr thread:
 * Zephyr polls in the caller of connect() and poll() (imu_raw_mqtt_tid 11
 * -> 107, net_socket_service 14 -> 104 for Telnet).  106: below the
 * application threads, above the shell.  The stack holds a NINA request
 * and usrsock's event delivery (poll and wait callbacks).
 */

#define APP_PRIO_NINA_EVENT         106
#define APP_STACK_NINA_EVENT        1536

/* The NINA passthrough mode (common/drivers/nina/port/nuttx/nina_passthrough_os.c): the
 * init task while it runs the mode, "nina_pt_rx" and "nina_pt_tx".  No
 * Zephyr thread (the FreeRTOS build runs the mode in one loop without the
 * scheduler).  Above every thread of the normal firmware, none of which
 * runs in this mode.  The stacks hold a write or read of /dev/ttyACM0
 * through the serial driver and cdcacm.
 */

#define APP_PRIO_NINA_PT            120
#define APP_STACK_NINA_PT           2048

/* Not used by the port; for reference (see nuttx/README.md):
 *   rx_q[0]        -1 (cooperative)  -> 119, 1504 (Zephyr's IP stack, idle)
 *   tcp_work      -14 (cooperative)  -> 132, 1024 (Zephyr's IP stack, idle)
 *   usbd, usbd@50110000 -8 (coop.)   -> 126, 1024 / 512 (NuttX: interrupt)
 *   logging        14                -> 104, 768 (NuttX: none, see
 *                                       common/compat/include/app_log.h)
 *   zperf_work_q   15 -> 14          -> 104, 2048 (NuttX: iperf, whose
 *                                       threads have fixed priorities
 *                                       100 and 101)
 *   idle           15                -> NuttX idle (0), 256 (NuttX 1024)
 */

#endif /* __NUTTX_DRIVERS_INCLUDE_PRIORITIES_OS_H */
