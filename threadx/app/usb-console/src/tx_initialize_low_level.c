/* SPDX-License-Identifier: MIT */
/* The board's low-level initialization of ThreadX on the Pico SDK (ThreadX's
 * Cortex-M0 port, ports/cortex_m0/gnu; its example_build/
 * tx_initialize_low_level.S is the model), which _tx_initialize_kernel_setup()
 * and tx_kernel_enter() call once.
 *
 * The Pico SDK's crt0 owns the vector table and the main stack; ThreadX's
 * handlers take three of its entries, named in the link
 * (threadx_kernel.cmake: --defsym):
 *
 *   isr_pendsv   __tx_PendSVHandler   the scheduler (the context switch, and
 *                                     the wait for a ready thread)
 *   isr_svcall   __tx_SVCallHandler   the same code, as in ThreadX's example
 *                                     vectors (the Cortex-M0 port issues no
 *                                     SVC)
 *   isr_systick  isr_systick()        the tick (src/tick.c)
 *
 * Interrupts run on the main stack (MSP), which crt0 set to __StackTop and
 * main() then ran on; the scheduler is entered from main() and leaves its
 * few frames there.  Threads run on their own stacks (PSP).
 *
 * Priorities (the RP2040 has the top 2 bits: 0x00, 0x40, 0x80, 0xc0):
 *
 *   PendSV, SVCall  0xc0, the lowest: the scheduler runs after every other
 *                   handler
 *   SysTick         0x80, PICO_DEFAULT_IRQ_PRIORITY, the priority the SDK
 *                   gives every device interrupt: above PendSV, because the
 *                   scheduler waits for a ready thread inside PendSV with
 *                   interrupts enabled, and only interrupts above it can
 *                   end that wait (a tick at PendSV's priority would never
 *                   wake a sleeping thread while no thread is ready);
 *                   neither preempts a device interrupt nor is preempted
 *                   by one
 *
 * SysTick counts the processor clock (clk_sys, 125 MHz) and interrupts every
 * clock_get_hz(clk_sys) / TX_TIMER_TICKS_PER_SECOND cycles, 1 ms; the tick's
 * catch-up needs that to be exactly one tick of the 1 MHz TIMER, which
 * main() checks before the kernel starts.  It starts counting here, with
 * interrupts masked until the scheduler runs (_tx_thread_schedule()
 * unmasks them), so the first tick interrupt is taken then.
 */

#include <stdint.h>

#include "hardware/clocks.h"
#include "hardware/exception.h"
#include "hardware/irq.h"
#include "hardware/regs/m0plus.h"
#include "hardware/structs/systick.h"

#include "tx_api.h"
#include "tx_initialize.h"
#include "tx_thread.h"

/* The top of the main stack (the Pico SDK's linker script). */
extern char __StackTop[];

#define TX_LL_PRIO_SCHEDULER 0xc0
#define TX_LL_PRIO_TICK      PICO_DEFAULT_IRQ_PRIORITY

VOID _tx_initialize_low_level(VOID)
{
	__asm volatile("cpsid i" : : : "memory");

	/* No memory for tx_application_define() to hand out. */
	_tx_initialize_unused_memory = TX_NULL;
	/* Not read by the Cortex-M0 port (the hardware switches to the main
	 * stack for a handler); kept as ThreadX's other ports set it.
	 */
	_tx_thread_system_stack_ptr = __StackTop;

	(void)exception_set_priority(PENDSV_EXCEPTION, TX_LL_PRIO_SCHEDULER);
	(void)exception_set_priority(SVCALL_EXCEPTION, TX_LL_PRIO_SCHEDULER);
	(void)exception_set_priority(SYSTICK_EXCEPTION, TX_LL_PRIO_TICK);

	systick_hw->csr = 0;
	systick_hw->rvr = clock_get_hz(clk_sys) / TX_TIMER_TICKS_PER_SECOND - 1u;
	systick_hw->cvr = 0;
	systick_hw->csr = M0PLUS_SYST_CSR_CLKSOURCE_BITS | M0PLUS_SYST_CSR_TICKINT_BITS |
			  M0PLUS_SYST_CSR_ENABLE_BITS;
}
