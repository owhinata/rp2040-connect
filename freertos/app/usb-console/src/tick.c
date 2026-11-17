/* SPDX-License-Identifier: MIT */
/* The kernel tick's hook: time slices of 20 ms among tasks of the same
 * priority, as in the Zephyr build (CONFIG_TIMESLICE_SIZE=20,
 * CONFIG_TIMESLICE_PRIORITY=0).
 *
 * FreeRTOS's own time slicing (configUSE_TIME_SLICING 1) switches between
 * the ready tasks of the running priority at every tick, 100 us here.  It is
 * off (configUSE_TIME_SLICING 0), and this hook, which the tick interrupt
 * calls (configUSE_TICK_HOOK), asks for a switch once the running task has
 * run for TIMESLICE_TICKS ticks in a row.  The switch (PendSV) takes the next
 * ready task of the same priority (FreeRTOS's selection goes round the
 * ready list of the highest priority, also without time slicing), or the
 * same task again when no other one is ready, at the cost of one context
 * switch every 20 ms.  As with Zephyr's CONFIG_TIMESLICE_PRIORITY=0, only
 * the preemptible Zephyr priorities are sliced (0..14 -> 20..6, and the
 * port's tasks below them); the cooperative ones (-1 -> 21 and up) and the
 * timer task are not, nor is the idle task, which has no peer.
 *
 * Differences from Zephyr: the slice is counted in whole ticks from the
 * first tick that finds the task running (Zephyr starts it when the thread
 * is switched in), and a task that was switched out and back in between two
 * ticks keeps its count.
 *
 * Later (F02) the compatibility layer's timeouts (z_add_timeout()) are
 * expected to run from this hook too.
 */

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "priorities.h"

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

/* Only used in the tick interrupt. */
static TaskHandle_t slice_task;
static uint32_t slice_ticks;

void vApplicationTickHook(void)
{
	TaskHandle_t task = xTaskGetCurrentTaskHandle();
	UBaseType_t prio;

	if (task != slice_task) {
		slice_task = task;
		slice_ticks = 0;
	}
	if (++slice_ticks < TIMESLICE_TICKS) {
		return;
	}
	slice_ticks = 0;
	prio = uxTaskPriorityGetFromISR(task);
	if (prio > NANO_PRIO_IDLE && prio <= TIMESLICE_PRIO_MAX) {
		/* The tick interrupt; PendSV runs after it. */
		portYIELD_FROM_ISR(pdTRUE);
	}
}
