/* SPDX-License-Identifier: MIT */
/* The ThreadX part of the USB console's start (main_os.h of
 * common/pico_sdk/app/usb-console/src, which describes each item; included
 * once, by main.c): ThreadX set up without its scheduler
 * (_tx_initialize_kernel_setup()), the start-up thread "main" started by the
 * compatibility layer (z_os_thread_start()), its end with
 * z_os_thread_exit(), and the scheduler's start with tx_kernel_enter().
 *
 * Interrupts.  _tx_initialize_kernel_setup() runs the board's
 * _tx_initialize_low_level() (tx_initialize_low_level.c), which masks
 * interrupts (PRIMASK) and starts SysTick, then ThreadX's own
 * initialization; from main_os_kernel_init() on, interrupts stay masked
 * until the scheduler takes its first thread (_tx_thread_schedule()
 * unmasks them).  With them masked, no tick and no handler can run before
 * the scheduler: a handler that makes a thread ready, or the tick's time
 * slices and timeouts, would pend PendSV, and PendSV taken in main() would
 * enter the scheduler from the middle of main().  What main() does from
 * here on needs no interrupt and no time to pass: the USB stdio's start
 * (TinyUSB's controller set up, its handlers and alarm added; no wait for
 * the host: PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 0), the consoles and the
 * layer's objects and threads, I2C0's start (resets, pins, registers, its
 * handlers; no transfer: the LSM6DSOX is set up by the thread "main"), the
 * key-value area's bounds (no flash access), and the LED's GPIO.  Log
 * lines on the way go to the RAM log (the USB port is not open yet).  The
 * USB enumeration, the first ticks and the pending handlers all run once
 * the scheduler starts.  The tick's catch-up (tick.c) starts at the first
 * tick, so the time main() spent here is not counted as lost ticks.  The
 * NINA passthrough mode's check and its run come before
 * main_os_kernel_init(), with interrupts enabled.
 *
 * The other hooks of the kernel are in tx_user.h, tick.c,
 * tx_initialize_low_level.c, idle_os.h (the idle wait), run_time.c (the
 * run time of "ps") and fault_os_impl.h (the stack check's handler,
 * registered here).
 */

#ifndef MAIN_OS_IMPL_H
#define MAIN_OS_IMPL_H

#include <stdint.h>
#include <string.h>

#include "hardware/clocks.h"

#include "tx_api.h"
#include "tx_initialize.h"
#include "tx_thread.h"

#include "priorities.h"
#include "zephyr_kernel_os.h"

#define MAIN_OS_PROMPT "threadx:~$ "

#define MAIN_OS_INLINE static inline __attribute__((always_inline))

/* The stack check's handler (fault_os_impl.h, tx_user.h). */
void fault_os_stack_error(TX_THREAD *thread);

/* The start-up thread's stack, used only until the thread ends, with the
 * layer's storage of the thread after it (Z_OS_THREAD_RESERVED:
 * zephyr_kernel_os_impl.h of threadx/drivers/compat), is not in .bss but
 * in SCRATCH_X, which only core 1's stack would use
 * (ld/section_extra_post_scratch.incl; this build does not link
 * pico_multicore, so the whole 4 KiB is free).  Not loaded: ThreadX fills
 * the stack when it creates the thread, and main_os_thread_start() clears
 * the storage after it, which a restart leaves as the last run had it
 * (z_os_thread_start() would take it for a thread that ended).
 */
static uint8_t main_stack[APP_STACK_MAIN + Z_OS_THREAD_RESERVED]
	__attribute__((section(".scratch_x_noload.main_stack"), aligned(8)));
static z_os_thread_t main_tid;

/* ThreadX's last step of tx_kernel_enter() before the scheduler: everything
 * was made by main() before it.
 */
void tx_application_define(void *first_unused_memory)
{
	(void)first_unused_memory;
}

/* ThreadX's initialization without the scheduler, so that objects and
 * threads can be made before tx_kernel_enter(), which then skips it.  It
 * resets the stack check's handler: registered after it.
 */
MAIN_OS_INLINE void main_os_kernel_init(void)
{
	_tx_initialize_kernel_setup();
	(void)tx_thread_stack_error_notify(fault_os_stack_error);
}

/* Started by the layer, as every thread that waits through it must be (its
 * wake flag is in the storage after the stack).
 */
MAIN_OS_INLINE void main_os_thread_start(void (*fn)(void *arg))
{
	memset(&main_stack[APP_STACK_MAIN], 0, Z_OS_THREAD_RESERVED);
	if (z_os_thread_start(fn, "main", main_stack, APP_STACK_MAIN, APP_PRIO_MAIN, NULL,
			      &main_tid) != 0) {
		panic("main_os_thread_start");
	}
}

/* ThreadX keeps the ended thread in its list of created threads
 * (TX_TERMINATED); "ps" leaves such threads out.
 */
MAIN_OS_INLINE void main_os_thread_exit(void)
{
	z_os_thread_exit();
}

/* The tick's catch-up (tick.c, tick_catchup.h) takes SysTick's period
 * (clock_get_hz(clk_sys) / TX_TIMER_TICKS_PER_SECOND cycles,
 * tx_initialize_low_level.c) for exactly one tick of the TIMER: a shorter
 * one would let the count run ahead of the time.
 */
MAIN_OS_INLINE void __attribute__((noreturn)) main_os_kernel_start(void)
{
	if (clock_get_hz(clk_sys) % TX_TIMER_TICKS_PER_SECOND != 0) {
		panic("clk_sys is not a whole number of ticks");
	}

	tx_kernel_enter();

	for (;;) {
	}
}

#endif /* MAIN_OS_IMPL_H */
