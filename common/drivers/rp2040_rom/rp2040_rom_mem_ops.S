/* SPDX-License-Identifier: MIT */
/* memcpy(), memset() and the ARM EABI's __aeabi_memcpy*() and
 * __aeabi_memset*() through the RP2040 bootrom's routines (rp2040_rom_mem.h).
 *
 * The same split as the Pico SDK's mem_ops_aeabi.S:
 *   memcpy, __aeabi_memcpy      -> 'M','C' (memcpy)
 *   __aeabi_memcpy4, 8          -> 'C','4' (memcpy44: dest and src aligned)
 *   memset                      -> 'M','S' (memset)
 *   __aeabi_memset              -> 'M','S' with n and c swapped
 *   __aeabi_memset4, 8          -> 'S','4' (memset4: dest aligned), swapped
 * The EABI's memset has the arguments (dest, n, c), the bootrom's
 * (dest, c, n).  __aeabi_memclr*() are left to the C library, which calls
 * memset().
 *
 * Each entry loads its routine from rp2040_rom_mem_fn[] and jumps to it with
 * the caller's registers and return address, as the Pico SDK's do; the only
 * addition is the test for NULL, an entry not looked up yet, which goes to
 * rp2040_rom_mem_slow below once per entry.
 *
 * With RP2040_ROM_MEM_WRAP the entries are named __wrap_memcpy, ... for the
 * linker's --wrap=memcpy, ... (the Zephyr build, whose picolibc defines the
 * plain names).
 */
#include "rp2040_rom_mem.h"

#ifdef RP2040_ROM_MEM_WRAP
#define ROM_MEM_SYM(name) __wrap_##name
#else
#define ROM_MEM_SYM(name) name
#endif

	.syntax unified
	.cpu cortex-m0plus
	.thumb

.macro rom_mem_func name
	.global \name
	.type \name, %function
	.thumb_func
\name:
.endm

/* Load the entry at \offset into r3 and jump to it; r0-r2 are the
 * arguments of the bootrom's routine.
 */
.macro rom_mem_jump offset
	ldr r3, =rp2040_rom_mem_fn
	ldr r3, [r3, #\offset]
	cmp r3, #0
	beq 1f
	bx r3
1:
	movs r3, #\offset
	b rp2040_rom_mem_slow
.endm

/* One section: the entries reach rp2040_rom_mem_slow with a short branch. */
	.section .text.rp2040_rom_mem_ops, "ax", %progbits
	.align 2

rom_mem_func ROM_MEM_SYM(memcpy)
rom_mem_func ROM_MEM_SYM(__aeabi_memcpy)
	rom_mem_jump RP2040_ROM_MEM_MEMCPY

rom_mem_func ROM_MEM_SYM(__aeabi_memcpy4)
rom_mem_func ROM_MEM_SYM(__aeabi_memcpy8)
	rom_mem_jump RP2040_ROM_MEM_MEMCPY44

rom_mem_func ROM_MEM_SYM(memset)
	rom_mem_jump RP2040_ROM_MEM_MEMSET

rom_mem_func ROM_MEM_SYM(__aeabi_memset)
	/* (dest, n, c) -> (dest, c, n) */
	eors r2, r1
	eors r1, r2
	eors r2, r1
	rom_mem_jump RP2040_ROM_MEM_MEMSET

rom_mem_func ROM_MEM_SYM(__aeabi_memset4)
rom_mem_func ROM_MEM_SYM(__aeabi_memset8)
	eors r2, r1
	eors r1, r2
	eors r2, r1
	rom_mem_jump RP2040_ROM_MEM_MEMSET4

/* r0-r2: the routine's arguments, r3: the entry's offset, lr: the caller's
 * return address.  Look the routine up (a C function: r0-r3, r12 and lr are
 * not kept), then jump to it with the arguments and lr restored.  Four words
 * pushed keep the stack 8-byte aligned.
 */
	.type rp2040_rom_mem_slow, %function
	.thumb_func
rp2040_rom_mem_slow:
	push {r0, r1, r2, lr}
	movs r0, r3
	bl rp2040_rom_mem_resolve
	mov r12, r0
	pop {r0, r1, r2, r3}
	mov lr, r3
	bx r12
	.size rp2040_rom_mem_slow, . - rp2040_rom_mem_slow

	.ltorg
