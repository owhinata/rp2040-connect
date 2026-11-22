/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the port's task priorities and stack sizes
 * (common/compat/priorities.h, which includes this header first):
 *
 *   FreeRTOS priority = 20 - Zephyr priority
 *
 * Zephyr: 16 cooperative (-16..-1) and 15 preemptible (0..14) application
 * priorities, idle at 15; a lower number runs first.  FreeRTOS:
 * 0..configMAX_PRIORITIES - 1 (0..31), idle at 0; a higher number runs
 * first.  20 - p keeps the order: the preemptible Zephyr priorities 0..14
 * become 20..6, Zephyr's idle 15 would be 5, and the cooperative ones
 * -1..-11 become 21..31 (the port uses only -1).  Zephyr priorities below
 * -11 do not fit; no thread of the port has one.  FreeRTOS cannot keep a
 * cooperative thread from being preempted by a higher one (Zephyr does not
 * preempt a cooperative thread at all), the approximation the NuttX build
 * makes as well.  Stack sizes are in bytes, as in Zephyr; FreeRTOS takes
 * words (StackType_t, 4 bytes).
 *
 * Interrupts have a stack of their own (the main stack, MSP: PICO_STACK_SIZE
 * 0x800 = 2048 bytes, Zephyr's CONFIG_ISR_STACK_SIZE=2048; the "(interrupts)" row of "ps").
 *
 * The values of priorities.h on FreeRTOS:
 *
 *   wifi_reconnect_tid, nina_scan_tid, nina_connect_tid       8 -> 12
 *   nina_monitor_tid, time_sync_tid, imu_stream_tid           9 -> 11
 *   imu_fifo_worker                                          10 -> 10
 *   imu_raw_mqtt_tid                                         11 ->  9
 *   main, sensor shell processing, RTIO WQ 0                  0 -> 20
 *   the shell ("console", the USB console task, and the
 *   commands it runs), Telnet service                        14 ->  6
 *   sysworkq, net_mgmt                                       -1 -> 21
 *
 * Shared by FreeRTOSConfig.h (the timer task) and the sources.
 */

#ifndef PRIORITIES_OS_H
#define PRIORITIES_OS_H

#define APP_PRIO_FROM_ZEPHYR(p) (20 - (p))

/* The kernel's timer task ("Tmr Svc", configTIMER_TASK_PRIORITY).  No Zephyr
 * thread: besides the kernel's software timers (the port uses none), it runs
 * the functions that interrupt handlers defer with
 * xTimerPendFunctionCallFromISR(), and the Pico SDK's locks and sleeps
 * (configSUPPORT_PICO_SYNC_INTEROP, configSUPPORT_PICO_TIME_INTEROP) wake
 * their waiting task that way when an interrupt handler releases the lock
 * or the sleep's alarm fires (xEventGroupSetBitsFromISR()): the USB code's
 * stdio mutex, released in the USB worker interrupt, is the one the port
 * meets.  In Zephyr the interrupt handler makes the waiting thread ready
 * itself, so the wake-up must not wait behind any task: the highest
 * priority, 31 (configMAX_PRIORITIES - 1, above the cooperative -1 -> 21).
 * Its stack (configTIMER_TASK_STACK_DEPTH, words) holds the event group
 * calls.
 */
#define APP_PRIO_TIMER_SVC  31
#define APP_STACK_TIMER_SVC 1024

/* The kernel's idle task ("IDLE"): priority 0 (tskIDLE_PRIORITY), below
 * every task of the port; it measures the CPU load (idle.c).  Zephyr's idle
 * thread has 256 bytes (CONFIG_IDLE_STACK_SIZE); FreeRTOS takes
 * configMINIMAL_STACK_SIZE words, which the Pico SDK also uses as its
 * default.
 */
#define APP_PRIO_IDLE  0
#define APP_STACK_IDLE 1024

/* "heartbeat": blinks the LED (GPIO6, main.c).  No Zephyr thread (the Zephyr
 * build leaves the LED alone): 1, below every thread of the port and above
 * idle.
 */
#define APP_PRIO_HEARTBEAT  1
#define APP_STACK_HEARTBEAT 1024

/* Not used by the port; for reference (Zephyr build, as in
 * nuttx/drivers/include/priorities_os.h):
 *   rx_q[0]        -1 (cooperative)  -> 21, 1504 (Zephyr's IP stack, idle)
 *   tcp_work      -14 (cooperative)  -> does not fit (Zephyr's IP stack)
 *   usbd, usbd@50110000 -8 (coop.)   -> 28 (FreeRTOS: the Pico SDK runs
 *                                       TinyUSB in interrupts)
 *   logging        14                ->  6, 768
 *   zperf_work_q   15 -> 14          ->  6, 2048
 *   idle           15                -> FreeRTOS idle (0)
 */

#endif /* PRIORITIES_OS_H */
