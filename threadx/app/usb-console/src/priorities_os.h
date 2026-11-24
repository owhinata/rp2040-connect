/* SPDX-License-Identifier: MIT */
/* The ThreadX part of the port's thread priorities and stack sizes
 * (common/compat/include/priorities.h, which includes this header first), and the
 * time slices:
 *
 *   layer priority = 15 - Zephyr priority      (larger = more urgent)
 *   ThreadX priority = 31 - layer priority = Zephyr priority + 16
 *
 * Zephyr: 16 cooperative (-16..-1) and 15 preemptible (0..14) application
 * priorities, idle at 15; a lower number runs first.  ThreadX:
 * 0..TX_MAX_PRIORITIES - 1 (0..31), 0 first, the same direction, so
 * Zephyr's 32 levels are ThreadX's 32 one to one (Zephyr p is ThreadX
 * p + 16, what "ps" shows).  The priorities of priorities.h
 * (APP_PRIO_FROM_ZEPHYR()) are in the order of the generic kernel part of
 * the layer, a larger value more urgent (z_os_prio_t,
 * common/compat/port/zephyr_kernel_os.h): 15 - p, so Zephyr
 * -16..14 is 31..1 and Zephyr's idle 15 would be 0, which no thread of the
 * layer takes (z_os_thread_start() refuses it).  z_os_thread_start() gives
 * ThreadX 31 - that value (Z_TX_PRIORITY(), zephyr_kernel_os_impl.h).
 *
 * Time slices: 20 ms among threads of the same priority, as in the Zephyr
 * build (CONFIG_TIMESLICE_SIZE=20, CONFIG_TIMESLICE_PRIORITY=0) and the
 * FreeRTOS build's tick hook: ThreadX's own time slice of a thread
 * (tx_thread_create()'s time_slice), which z_os_thread_start() gives the
 * preemptible Zephyr priorities (0..14, layer 15..1); the cooperative ones
 * (-1 and above, layer 16 and up) have none.  ThreadX keeps a preempted
 * thread's rest of its slice and starts a new one when the slice ends
 * (Zephyr starts a new slice at every switch, z_time_slice_reset()).
 */

#ifndef PRIORITIES_OS_H
#define PRIORITIES_OS_H

#define APP_PRIO_FROM_ZEPHYR(p) (15 - (p))

/* The ticks of a time slice (1 ms ticks), and the most urgent layer
 * priority that has slices (CONFIG_TIMESLICE_PRIORITY=0: Zephyr 0 and
 * below it).
 */
#define APP_TIMESLICE_TICKS    20
#define APP_TIMESLICE_PRIO_MAX APP_PRIO_FROM_ZEPHYR(0)

/* No thread of the kernel: ThreadX has no idle thread (its scheduler waits
 * when no thread is ready, in PendSV on the main stack) and, with
 * TX_TIMER_PROCESS_IN_ISR, no timer thread.  The FreeRTOS build's timer task
 * wakes the threads that wait on the Pico SDK's locks and sleeps; here an
 * interrupt handler wakes them itself (threadx/drivers/pico_sdk_interop).
 *
 * Interrupts have a stack of their own (the main stack, MSP: PICO_STACK_SIZE
 * 0x800 = 2048 bytes, Zephyr's CONFIG_ISR_STACK_SIZE=2048; the
 * "(interrupts)" row of "ps"), which also holds the frames of main() below
 * the scheduler's start and the scheduler's wait with the idle hooks.
 *
 * The stacks of the layer's own threads start at the FreeRTOS build's
 * values (freertos/app/usb-console/src/priorities_os.h, measured there:
 * sysworkq 200 bytes used of 384, net_mgmt 1024 of 2048, telnet_service
 * 1164 of 2048, sensor_shell_tid 1048 of 2048), until they are measured
 * here ("ps").  What a thread's stack holds of the kernel is about the
 * same on both: the context of a switched-out thread (ThreadX's Cortex-M0
 * port: r4-r11 and LR, 36 bytes, above the 32 the hardware pushes;
 * FreeRTOS's RP2040 port: r4-r11, 32), and the frames of the kernel's
 * services in the waits (ThreadX: the API's error check, txe_*(), then the
 * service and _tx_thread_system_suspend() with the stack check, which may
 * call _tx_thread_stack_analyze()).  ThreadX leaves the top word of each
 * stack for the stack check's pattern (tx_user.h).  The control block and
 * the rest of a thread's storage are not on its stack but after it
 * (Z_OS_THREAD_RESERVED: 272 bytes, TX_THREAD 200 with the execution
 * profile kit's times, the wake flag's TX_SEMAPHORE 28, the entry 8, the
 * name 32; FreeRTOS 136).  The system work queue's stack is larger than
 * the FreeRTOS build's 384: with 384 its lowest free mark on the board was
 * 136 bytes.
 */
#define APP_STACK_SYSWORKQ       512
#define APP_STACK_NET_MGMT       2048
#define APP_STACK_TELNET_SERVICE 2048
#define APP_STACK_SENSOR_SHELL   2048

#endif /* PRIORITIES_OS_H */
