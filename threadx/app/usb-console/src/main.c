/* SPDX-License-Identifier: MIT */
/* The start of the ThreadX build: main() runs the shared steps of the USB
 * console on the Pico SDK (common/imu/port/pico_sdk/
 * usb_console.h) with ThreadX's own: the kernel set up without its
 * scheduler (_tx_initialize_kernel_setup()), the thread "main" started by
 * the compatibility layer (z_os_thread_start()) and the scheduler
 * (tx_kernel_enter()).
 *
 * Interrupts.  _tx_initialize_kernel_setup() runs the board's
 * _tx_initialize_low_level() (tx_initialize_low_level.c), which masks
 * interrupts (PRIMASK) and starts SysTick, then ThreadX's own
 * initialization.  Interrupts stay masked until the scheduler takes its
 * first thread (_tx_thread_schedule() unmasks them), so no tick and no
 * handler runs in main(): a handler that makes a thread ready, or the
 * tick's time slices and timeouts, would pend PendSV, and PendSV taken in
 * main() would enter the scheduler from the middle of main().
 * usb_console_init() and usb_console_shell_start() need no interrupt and no
 * time to pass (usb_console.h); their log lines go to the RAM log.  The USB
 * enumeration, the first ticks and the pending handlers run once the
 * scheduler starts.  The tick's catch-up (tick.c) starts at the first tick,
 * so the time spent here is not counted as lost ticks.
 * usb_console_early_init(), with the NINA passthrough mode's check and its
 * run, comes before, with interrupts enabled.
 *
 * The other hooks of the kernel are in tx_user.h, tick.c,
 * tx_initialize_low_level.c, idle_os.h (the idle wait), run_time.c (the
 * run time of "ps") and fault_os.c (the stack check's handler,
 * registered here).
 */
#include <stdint.h>
#include <string.h>
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "tx_api.h"
#include "tx_initialize.h"
#include "tx_thread.h"
#include "priorities.h"
#include "usb_console.h"
#include "zephyr_kernel_os.h"

#define PROMPT "threadx:~$ "

/* The stack check's handler (fault_os.c, tx_user.h). */
void fault_os_stack_error(TX_THREAD *thread);

/* The thread "main"'s stack, used only until the thread ends, with the
 * layer's storage of the thread after it (Z_OS_THREAD_RESERVED:
 * zephyr_kernel_os_impl.h of common/compat/port/threadx), is not in .bss but
 * in SCRATCH_X, which only core 1's stack would use
 * (common/imu/port/pico_sdk/ld/section_extra_post_scratch.incl; this
 * build does not link pico_multicore, so the whole 4 KiB is free).  Not
 * loaded: ThreadX fills the stack when it creates the thread, and main()
 * clears the storage after it, which a restart leaves as the last run had
 * it (z_os_thread_start() would take it for a thread that ended).
 */
static uint8_t main_stack[APP_STACK_MAIN + Z_OS_THREAD_RESERVED]
	__attribute__((section(".scratch_x_noload.main_stack"), aligned(8)));
static z_os_thread_t main_tid;

/* ThreadX keeps the ended thread in its list of created threads
 * (TX_TERMINATED); "ps" leaves such threads out.
 */
static void main_thread(void *arg)
{
	(void)arg;

	usb_console_main_thread(PROMPT);
	z_os_thread_exit();
}

/* ThreadX's last step of tx_kernel_enter() before the scheduler: everything
 * was made by main() before it.
 */
void tx_application_define(void *first_unused_memory)
{
	(void)first_unused_memory;
}

int main(void)
{
	usb_console_early_init();

	/* ThreadX's initialization without the scheduler, so that objects and
	 * threads can be made before tx_kernel_enter(), which then skips it.
	 * It resets the stack check's handler: registered after it.
	 */
	_tx_initialize_kernel_setup();
	(void)tx_thread_stack_error_notify(fault_os_stack_error);

	usb_console_init();

	/* Started by the layer, as every thread that waits through it must be
	 * (its wake flag is in the storage after the stack).
	 */
	memset(&main_stack[APP_STACK_MAIN], 0, Z_OS_THREAD_RESERVED);
	if (z_os_thread_start(main_thread, "main", main_stack, APP_STACK_MAIN, APP_PRIO_MAIN, NULL,
			      &main_tid) != 0) {
		panic("z_os_thread_start");
	}
	usb_console_shell_start(PROMPT);

	/* The tick's catch-up (tick.c, tick_catchup.h) takes SysTick's period
	 * (clock_get_hz(clk_sys) / TX_TIMER_TICKS_PER_SECOND cycles,
	 * tx_initialize_low_level.c) for exactly one tick of the TIMER: a
	 * shorter one would let the count run ahead of the time.
	 */
	if (clock_get_hz(clk_sys) % TX_TIMER_TICKS_PER_SECOND != 0) {
		panic("clk_sys is not a whole number of ticks");
	}

	tx_kernel_enter();

	for (;;) {
	}
}
