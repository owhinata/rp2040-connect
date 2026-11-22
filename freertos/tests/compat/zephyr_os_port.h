/* SPDX-License-Identifier: MIT */
/* The port header of the Zephyr compatibility layer (see
 * freertos/drivers/compat/rp2040/zephyr_os_port.h, the board's) for the host
 * tests on the FreeRTOS POSIX port: with the FreeRTOS calls of
 * freertos/drivers/compat/zephyr_kernel_os_impl.h, the host implementation
 * of the generic kernel part's primitives (zephyr_kernel_os.h).  There the
 * tasks are threads of which one runs at a time, the interrupts are signals,
 * and the only "interrupt handler" is the tick hook (main.c counts it in
 * z_host_isr_depth).
 *
 * - The tick hook needs no switch request: xTaskIncrementTick() switches
 *   after the hook when a FromISR call made a higher task ready.  The POSIX
 *   port's portYIELD_FROM_ISR() would switch inside the signal handler.
 * - The end of a time slice (freertos/app/usb-console/src/tick.c) does call
 *   portYIELD_FROM_ISR(), so another thread runs while the interrupted one
 *   is still inside the hook: z_host_isr_depth is per thread, and the task
 *   switched to is not in an interrupt handler.
 * - The port switches threads at once when a task wakes a higher one inside a
 *   critical section (it does not wait for the section to end), so the lock
 *   of a task also suspends the scheduler: the switch then waits for the
 *   unlock, as on the board and in Zephyr.
 * - The clock is CLOCK_MONOTONIC.
 * - The tick hook's catch-up (freertos/app/usb-console/src/tick.c) reads the
 *   same clock, and cannot add a tick: the port's tick is a thread's
 *   usleep() and a signal, with nothing to pend, so z_os_tick_pend() does
 *   nothing and the count stays behind the clock as the port makes it.  The
 *   catch-up itself is checked by freertos/tests/tick, on a model of SysTick.
 */

#ifndef FREERTOS_TESTS_COMPAT_ZEPHYR_OS_PORT_H
#define FREERTOS_TESTS_COMPAT_ZEPHYR_OS_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

extern _Thread_local int z_host_isr_depth;

static inline uint64_t nano_mono_us(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000u;
}

static inline bool z_os_in_isr(void)
{
	return z_host_isr_depth > 0;
}

static inline void z_os_yield_from_isr(BaseType_t woken)
{
	(void)woken;
}

static inline void z_os_task_lock(void)
{
	vTaskSuspendAll();
	taskENTER_CRITICAL();
}

static inline void z_os_task_unlock(void)
{
	taskEXIT_CRITICAL();
	(void)xTaskResumeAll();
}

static inline void z_os_busy_wait_us(uint32_t us)
{
	uint64_t end = nano_mono_us() + us;

	while (nano_mono_us() < end) {
	}
}

static inline uint32_t z_os_tick_now_us32(void)
{
	return (uint32_t)nano_mono_us();
}

static inline void z_os_tick_pend(void)
{
}

#endif /* FREERTOS_TESTS_COMPAT_ZEPHYR_OS_PORT_H */
