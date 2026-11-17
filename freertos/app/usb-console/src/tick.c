/* SPDX-License-Identifier: MIT */
/* The kernel tick's hook: time slices of 20 ms among tasks of the same
 * priority, as in the Zephyr build (CONFIG_TIMESLICE_SIZE=20,
 * CONFIG_TIMESLICE_PRIORITY=0).
 *
 * FreeRTOS's own time slicing (configUSE_TIME_SLICING 1) switches between
 * the ready tasks of the running priority at every tick (100 us, or 1 ms
 * with the build option NANO_TICK_HZ=1000).  It is
 * off (configUSE_TIME_SLICING 0), and this hook, which the tick interrupt
 * calls (configUSE_TICK_HOOK), asks for a switch once the running task has
 * run for TIMESLICE_TICKS ticks since it was switched in.  The slice starts
 * again whenever another task is switched in (tick_slice_restart(), from the
 * kernel's switch, see freertos/drivers/compat/zephyr_os_sched.h), as
 * Zephyr's z_time_slice_reset() at every switch: a task that a higher one
 * preempts gets a whole slice again when it goes on.  The switch at the end
 * of the slice gives the turn away (z_sched_yield_pending): the next ready
 * task of the same priority runs, or the same task again when no other one is
 * ready, at the cost of one context switch every 20 ms.  As with Zephyr's
 * CONFIG_TIMESLICE_PRIORITY=0, only the preemptible Zephyr priorities are sliced (0..14 -> 20..6,
 * and the port's tasks below them); the cooperative ones (-1 -> 21 and up) and the timer task are
 * not, nor is the idle task, which has no peer.
 *
 * Difference from Zephyr: the slice is counted in whole ticks
 * (TIMESLICE_TICKS: 200 of 100 us, or 20 of 1 ms), so a task switched in
 * between two ticks gets (TIMESLICE_TICKS - 1, TIMESLICE_TICKS] ticks
 * (Zephyr's timeout: (200, 201] of its 100 us ticks); one switched in at the
 * end of the previous slice gets TIMESLICE_TICKS, as in Zephyr.  With 1 ms
 * ticks a slice is so between 19 and 20 ms long.
 *
 * The hook also runs the Zephyr compatibility layer's timeouts (the timers
 * of delayable work items, z_add_timeout(): zephyr_compat_tick() of
 * freertos/drivers/compat), first, in the same interrupt.
 */

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "priorities.h"
#include "zephyr_compat_os.h"

#define TIMESLICE_MS    20
#define TIMESLICE_TICKS (TIMESLICE_MS * configTICK_RATE_HZ / 1000)

/* CONFIG_TIMESLICE_PRIORITY=0: Zephyr slices priorities 0 and below it. */
#define TIMESLICE_PRIO_MAX NANO_PRIO_FROM_ZEPHYR(0)

#if TIMESLICE_TICKS * 1000 != TIMESLICE_MS * configTICK_RATE_HZ
#error "the time slice is not a whole number of ticks"
#endif

#if configUSE_TIME_SLICING != 0 || configUSE_TICK_HOOK != 1
#error "tick.c makes the time slices"
#endif

/* Used in the tick interrupt and in the kernel's switch (PendSV), which have
 * the same, lowest priority and so do not interrupt each other.
 */
static uint32_t slice_ticks;

void tick_slice_restart(void)
{
	slice_ticks = 0;
}

void vApplicationTickHook(void)
{
	UBaseType_t prio;

	zephyr_compat_tick();

	if (++slice_ticks < TIMESLICE_TICKS) {
		return;
	}
	slice_ticks = 0;
	prio = uxTaskPriorityGetFromISR(xTaskGetCurrentTaskHandle());
	if (prio > NANO_PRIO_IDLE && prio <= TIMESLICE_PRIO_MAX) {
		/* The tick interrupt; PendSV runs after it.  The task gives up its
		 * turn (zephyr_os_sched.h).
		 */
		z_sched_yield_pending = 1;
		portYIELD_FROM_ISR(pdTRUE);
	}
}
