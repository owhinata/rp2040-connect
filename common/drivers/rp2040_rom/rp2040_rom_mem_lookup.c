/* SPDX-License-Identifier: MIT */
/* The lookup of the bootrom's memcpy() and memset() routines for
 * rp2040_rom_mem_ops.S (rp2040_rom_mem.h).
 *
 * The entries are looked up on first use instead of at a fixed point of the
 * boot, so no call can come before them, whatever the OS runs first, as long
 * as .bss is zero by then: an entry left over in RAM from before a reset is
 * never trusted.  The OS must not call these routines before .bss is
 * cleared (the Zephyr build's arch_early_memset() and arch_early_memcpy(),
 * which clear .bss and copy .data, use picolibc's own routines; the NuttX
 * build's __start() clears .bss with its own loop, which the board's
 * scripts/Make.defs keeps GCC from turning into a call to memset()).  .data
 * need not be copied yet: nothing here is in .data.
 *
 * Interrupts and threads can call a routine while another context is looking
 * the same entry up: both look up the same address and store it with one
 * word write, so either one's result is right.
 */
#include <stdint.h>

#include "rp2040_rom_mem.h"

#ifndef RP2040_ROM_LOOKUP
/* The bootrom's halfword at 0x14 is the address of its function table, the
 * one at 0x18 the address of rom_table_lookup(table, code) (RP2040 datasheet,
 * "Bootrom Contents"; the Pico SDK's rom_func_lookup()).  The asm hides the
 * constant address from the compiler, which warns about reads of the first
 * page.
 */
typedef void *(*rp2040_rom_table_lookup_fn)(const uint16_t *table, uint32_t code);

static void *rp2040_rom_lookup(uint32_t code)
{
	uintptr_t base = 0x14;

	__asm__("" : "+r"(base));

	const volatile uint16_t *hword = (const volatile uint16_t *)base;
	rp2040_rom_table_lookup_fn lookup = (rp2040_rom_table_lookup_fn)(uintptr_t)hword[2];

	return lookup((const uint16_t *)(uintptr_t)hword[0], code);
}

#define RP2040_ROM_LOOKUP(code) rp2040_rom_lookup(code)
#endif

void *rp2040_rom_mem_fn[RP2040_ROM_MEM_COUNT];

static const uint16_t rp2040_rom_mem_code[RP2040_ROM_MEM_COUNT] = {
	[RP2040_ROM_MEM_MEMSET / 4] = RP2040_ROM_CODE('M', 'S'),
	[RP2040_ROM_MEM_MEMCPY / 4] = RP2040_ROM_CODE('M', 'C'),
	[RP2040_ROM_MEM_MEMSET4 / 4] = RP2040_ROM_CODE('S', '4'),
	[RP2040_ROM_MEM_MEMCPY44 / 4] = RP2040_ROM_CODE('C', '4'),
};

void *rp2040_rom_mem_resolve(unsigned int offset)
{
	unsigned int i = offset / 4U;
	void *fn = RP2040_ROM_LOOKUP(rp2040_rom_mem_code[i]);

	rp2040_rom_mem_fn[i] = fn;
	return fn;
}
