/* SPDX-License-Identifier: MIT */
/* The ThreadX part of the port's thread priorities and stack sizes
 * (common/compat/priorities.h, which includes this header first), and the
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
 * common/compat/kernel_generic/zephyr_kernel_os.h): 15 - p, so Zephyr
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
 * when no thread is ready) and, with TX_TIMER_PROCESS_IN_ISR, no timer
 * thread.  The stacks of the layer's own threads are Zephyr's
 * (priorities.h) until they are measured on the board.
 */

#endif /* PRIORITIES_OS_H */
