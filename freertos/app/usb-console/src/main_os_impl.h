/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the USB console's start (main_os.h of
 * common/pico_sdk/app/usb-console/src, which describes each item; included
 * once, by main.c): the start-up task "main" made with xTaskCreateStatic(),
 * its end with vTaskDelete(), the scheduler's start with
 * vTaskStartScheduler(), and the run time of the tasks.
 */

#ifndef MAIN_OS_IMPL_H
#define MAIN_OS_IMPL_H

#include <stdint.h>
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "priorities.h"

#define MAIN_OS_PROMPT "freertos:~$ "

#define MAIN_OS_INLINE static inline __attribute__((always_inline))

/* Priorities and stacks (bytes): src/priorities_os.h and
 * common/compat/priorities.h.  The console (the shell engine's serial
 * session, shell_serial.h) is the Zephyr build's shell thread
 * (APP_PRIO_SHELL, 14 -> 6, 3072 bytes): the commands run in it.
 */
#define MAIN_PRIORITY   APP_PRIO_MAIN
#define MAIN_STACK_SIZE (APP_STACK_MAIN / sizeof(StackType_t))

/* The start-up task's stack, used only until the task ends, is not in
 * .bss but in SCRATCH_X, which only core 1's stack would use
 * (ld/section_extra_post_scratch.incl).  Not loaded: the kernel fills it
 * when it creates the task.
 */
static StackType_t main_stack[MAIN_STACK_SIZE]
	__attribute__((section(".scratch_x_noload.main_stack"), aligned(8)));
static StaticTask_t main_tcb;

/* The run time of the tasks ("ps"), named in FreeRTOSConfig.h.  The other
 * hooks are in fault_os_impl.h (fatal errors), idle_os.h and tick.c.
 */
uint64_t app_run_time_us(void)
{
	return time_us_64();
}

MAIN_OS_INLINE void main_os_kernel_init(void)
{
}

MAIN_OS_INLINE void main_os_thread_start(void (*fn)(void *arg))
{
	xTaskCreateStatic(fn, "main", MAIN_STACK_SIZE, NULL, MAIN_PRIORITY, main_stack, &main_tcb);
}

MAIN_OS_INLINE void main_os_thread_exit(void)
{
	vTaskDelete(NULL);
}

/* The tick hook's catch-up (tick.c, tick_catchup.h) takes SysTick's period
 * (clock_get_hz(clk_sys) / configTICK_RATE_HZ cycles, the RP2040 port's
 * vPortSetupTimerInterrupt()) for exactly one tick of the TIMER: a shorter
 * one would let the count run ahead of the time.
 */
MAIN_OS_INLINE void __attribute__((noreturn)) main_os_kernel_start(void)
{
	if (clock_get_hz(clk_sys) % configTICK_RATE_HZ != 0) {
		panic("clk_sys is not a whole number of ticks");
	}

	vTaskStartScheduler();

	for (;;) {
	}
}

#endif /* MAIN_OS_IMPL_H */
