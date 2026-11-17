/* SPDX-License-Identifier: MIT */
/* Zephyr's choice of the next thread among threads of the same priority, on
 * FreeRTOS: FreeRTOSConfig.h includes this header at its end, and its two
 * trace macros run inside the kernel's vTaskSwitchContext()
 * (FreeRTOS-Kernel/tasks.c, with interrupts masked), so they name the
 * kernel's own variables (pxCurrentTCB, pxReadyTasksLists, the TCB's
 * uxPriority and xStateListItem: kernel V11.3.3, one core).  The kernel's
 * source is not changed.
 *
 * Zephyr on one CPU (kernel/sched.c next_up()): the running thread stays at
 * its place in the run queue; a higher thread preempts it, and when that one
 * blocks the preempted thread goes on.  Only a yield (k_yield(),
 * k_sleep(K_NO_WAIT)) or the end of its time slice puts it behind the other
 * threads of its priority.  The time slice starts again whenever a thread is
 * switched in (kswap.h do_swap(), z_time_slice_reset()).
 *
 * FreeRTOS by itself picks the next task of the highest ready priority
 * round-robin at every switch (taskSELECT_HIGHEST_PRIORITY_TASK(),
 * listGET_OWNER_OF_NEXT_ENTRY() advances the list's pxIndex): a task that a
 * higher one preempts for a moment (the timer task woken by a USB interrupt,
 * a work queue) loses the CPU to the next task of its priority.
 *
 * traceTASK_SWITCHED_OUT(): unless the outgoing task gave up its turn
 * (z_sched_yield_pending, set by the layer's yields and by the tick hook at
 * the end of a slice, and cleared here), and if it is still ready (it was
 * preempted, it did not block), its priority's list index is set back one
 * entry, so that the next pick at that priority is this task again.
 * traceTASK_SWITCHED_IN(): when another task than the outgoing one comes in,
 * its time slice starts again (tick_slice_restart() of the application's tick
 * hook, freertos/app/usb-console/src/tick.c).
 *
 * Left as FreeRTOS does it: a task made ready goes behind the list index
 * (FreeRTOS's vListInsertEnd()), which, with the index set back, may put it
 * before tasks that waited longer (Zephyr: at the end of the queue; the
 * difference needs three or more ready tasks of one priority); a task whose
 * priority FreeRTOS lowers (the end of priority inheritance) keeps its turn
 * at the new priority (Zephyr queues it at the end).  A FreeRTOS taskYIELD()
 * that does not set z_sched_yield_pending counts as a preemption.
 *
 * Part of the FreeRTOS implementation of the Zephyr compatibility layer
 * (freertos/drivers/compat).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_SCHED_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_SCHED_H

#ifndef __ASSEMBLER__

#include <stdint.h>

/* Set before a yield that gives up the turn (zephyr_kernel.c, tick.c,
 * os_clock.h); written with interrupts masked or by the running task just
 * before its yield.
 */
extern volatile uint8_t z_sched_yield_pending;

/* The task of the last switch-out (zephyr_kernel.c). */
extern void *volatile z_sched_switched_out;

/* The application's: the running task's time slice starts again. */
void tick_slice_restart(void);

/* NANO_SCHED_FREERTOS (the build option -DNANO_SCHED_ZEPHYR=OFF) leaves the
 * macros out: FreeRTOS's own choice, and the tick hook's slices are not
 * restarted at a switch.
 */
#ifndef NANO_SCHED_FREERTOS

#define traceTASK_SWITCHED_OUT()                                                                   \
	do {                                                                                       \
		z_sched_switched_out = pxCurrentTCB;                                               \
		if (z_sched_yield_pending != 0) {                                                  \
			z_sched_yield_pending = 0;                                                 \
		} else if (listIS_CONTAINED_WITHIN(&pxReadyTasksLists[pxCurrentTCB->uxPriority],   \
						   &pxCurrentTCB->xStateListItem)) {               \
			pxReadyTasksLists[pxCurrentTCB->uxPriority].pxIndex =                      \
				pxCurrentTCB->xStateListItem.pxPrevious;                           \
		}                                                                                  \
	} while (0)

#define traceTASK_SWITCHED_IN()                                                                    \
	do {                                                                                       \
		if ((void *)pxCurrentTCB != z_sched_switched_out) {                                \
			tick_slice_restart();                                                      \
		}                                                                                  \
	} while (0)

#endif /* NANO_SCHED_FREERTOS */

#endif /* __ASSEMBLER__ */

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_SCHED_H */
