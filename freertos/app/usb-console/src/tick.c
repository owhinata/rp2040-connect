/* SPDX-License-Identifier: MIT */
/* The kernel tick's hook: time slices of 20 ms among tasks of the same
 * priority, as in the Zephyr build (CONFIG_TIMESLICE_SIZE=20,
 * CONFIG_TIMESLICE_PRIORITY=0).
 *
 * FreeRTOS's own time slicing (configUSE_TIME_SLICING 1) switches between
 * the ready tasks of the running priority at every tick (1 ms).  It is off
 * (configUSE_TIME_SLICING 0), and this hook, which the tick interrupt calls
 * (configUSE_TICK_HOOK), asks for a switch every TIMESLICE_TICKS ticks: the
 * task running at that tick, if its priority is sliced, gives the turn to
 * the next ready task of its priority (FreeRTOS's choice, in turn), or runs
 * on when no other one is ready, at the cost of one context switch every
 * 20 ms.  As with Zephyr's CONFIG_TIMESLICE_PRIORITY=0, only the
 * preemptible Zephyr priorities are sliced (0..14 -> 20..6, and the port's
 * tasks below them); the cooperative ones (-1 -> 21 and up) and the timer
 * task are not, nor is the idle task, which has no peer.
 *
 * Difference from Zephyr: the slices are a fixed grid of TIMESLICE_TICKS
 * ticks, not counted from the switch to the task (Zephyr restarts its slice
 * at every switch, z_time_slice_reset()).  A task switched in between two
 * ends of slices runs until the next end, so for at most 20 ms; one switched
 * in at an end gets the whole 20 ms.
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

/* Used in the tick interrupt only. */
static uint32_t slice_ticks;

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
		/* The tick interrupt; PendSV runs after it. */
		portYIELD_FROM_ISR(pdTRUE);
	}
}
