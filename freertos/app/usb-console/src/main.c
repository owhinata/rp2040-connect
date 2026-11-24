/* SPDX-License-Identifier: MIT */
/* The start of the FreeRTOS build: main() runs the shared steps of the USB
 * console on the Pico SDK (common/imu/port/pico_sdk/
 * usb_console.h) with FreeRTOS's own: the task "main" and the scheduler.
 * FreeRTOS needs no set-up before its first object.
 */
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "priorities.h"
#include "usb_console.h"

#define PROMPT "freertos:~$ "

#define MAIN_STACK_SIZE (APP_STACK_MAIN / sizeof(StackType_t))

/* The task "main"'s stack, used only until the task ends, is not in .bss
 * but in SCRATCH_X, which only core 1's stack would use
 * (common/imu/port/pico_sdk/ld/section_extra_post_scratch.incl).
 * Not loaded: the kernel fills it when it creates the task.
 */
static StackType_t main_stack[MAIN_STACK_SIZE]
	__attribute__((section(".scratch_x_noload.main_stack"), aligned(8)));
static StaticTask_t main_tcb;

static void main_thread(void *arg)
{
	(void)arg;

	usb_console_main_thread(PROMPT);
	vTaskDelete(NULL);
}

int main(void)
{
	usb_console_early_init();
	usb_console_init();

	xTaskCreateStatic(main_thread, "main", MAIN_STACK_SIZE, NULL, APP_PRIO_MAIN, main_stack,
			  &main_tcb);
	usb_console_shell_start(PROMPT);

	/* The tick hook's catch-up (tick.c, tick_catchup.h) takes SysTick's
	 * period (clock_get_hz(clk_sys) / configTICK_RATE_HZ cycles, the RP2040
	 * port's vPortSetupTimerInterrupt()) for exactly one tick of the
	 * TIMER: a shorter one would let the count run ahead of the time.
	 */
	if (clock_get_hz(clk_sys) % configTICK_RATE_HZ != 0) {
		panic("clk_sys is not a whole number of ticks");
	}

	vTaskStartScheduler();

	for (;;) {
	}
}
