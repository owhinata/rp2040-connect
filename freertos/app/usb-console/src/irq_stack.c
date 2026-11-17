/* SPDX-License-Identifier: MIT */
/* The use of the interrupt stack (the main stack, MSP), for the "(interrupts)" row of "ps".
 *
 * Interrupt handlers run on the main stack, tasks on their own stacks
 * (PSP).  The main stack is the Pico SDK's: PICO_STACK_SIZE (0x800 = 2048
 * bytes, Zephyr's CONFIG_ISR_STACK_SIZE=2048) from __StackBottom to
 * __StackTop, the top of SCRATCH_Y (0x20041800..0x20042000).  main() runs on
 * it before the scheduler starts, and the RP2040 port does not move MSP back
 * to the top when the first task starts (vPortStartFirstTask(), one core),
 * so the frames of main() down to vTaskStartScheduler() stay in use below
 * the top and interrupts use the rest.  Zephyr also runs its start-up code
 * on the interrupt stack.  Nothing guards the bottom: past it the stack
 * grows into the rest of SCRATCH_Y and SCRATCH_X (core 1's stack, unused).
 *
 * The use is a high-water mark, as Zephyr's CONFIG_INIT_STACKS and the NuttX
 * build's CONFIG_STACK_COLORATION: irq_stack_paint(), called first in
 * main(), fills the stack from the bottom up to just below the current stack
 * pointer with a pattern, and irq_stack_used() counts the bytes from the top
 * down to the lowest word that no longer holds it.  What the start-up code
 * used below main()'s frame before the paint is not counted.
 */

#include <stddef.h>
#include <stdint.h>
#include "app.h"

/* Zephyr fills unused stacks with 0xaa bytes (CONFIG_INIT_STACKS). */
#define PAINT 0xaaaaaaaau

/* Words left unpainted below the stack pointer of irq_stack_paint(). */
#define PAINT_MARGIN_WORDS 8

/* The Pico SDK's linker script (memmap_default.ld). */
extern uint32_t __StackBottom[];
extern uint32_t __StackTop[];

void __attribute__((noinline)) irq_stack_paint(void)
{
	uint32_t *sp;
	uint32_t *p = __StackBottom;

	__asm volatile("mov %0, sp" : "=r"(sp));
	sp -= PAINT_MARGIN_WORDS;
	while (p < sp) {
		*(volatile uint32_t *)p = PAINT;
		p++;
	}
}

size_t irq_stack_size(void)
{
	return (size_t)((uintptr_t)__StackTop - (uintptr_t)__StackBottom);
}

uintptr_t irq_stack_base(void)
{
	return (uintptr_t)__StackBottom;
}

size_t irq_stack_used(void)
{
	const volatile uint32_t *p = __StackBottom;

	while (p < __StackTop && *p == PAINT) {
		p++;
	}
	return (size_t)((uintptr_t)__StackTop - (uintptr_t)p);
}
