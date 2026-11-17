/* SPDX-License-Identifier: MIT */
/* common/drivers/rp2040_rom/rp2040_rom_mem_lookup.c: the bootrom codes of
 * the entries, the lookup on first use and what it stores, on a fake
 * bootrom function table (the test includes the source with its lookup
 * replaced).  The entries' jumps (rp2040_rom_mem_ops.S) are Thumb code and
 * are not run here.
 */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static void *fake_lookup(uint32_t code);
#define RP2040_ROM_LOOKUP(code) fake_lookup(code)
#include "rp2040_rom_mem_lookup.c"

/* The RP2040 bootrom's codes (Pico SDK, boot/bootrom_constants.h). */
#define CODE_MEMSET   ('M' | ('S' << 8))
#define CODE_MEMCPY   ('M' | ('C' << 8))
#define CODE_MEMSET4  ('S' | ('4' << 8))
#define CODE_MEMCPY44 ('C' | ('4' << 8))

static char rom_memset, rom_memcpy, rom_memset4, rom_memcpy44;
static unsigned int lookups;
static uint32_t last_code;

static void *fake_lookup(uint32_t code)
{
	lookups++;
	last_code = code;
	switch (code) {
	case CODE_MEMSET:
		return &rom_memset;
	case CODE_MEMCPY:
		return &rom_memcpy;
	case CODE_MEMSET4:
		return &rom_memset4;
	case CODE_MEMCPY44:
		return &rom_memcpy44;
	default:
		return NULL;
	}
}

static void check(unsigned int offset, uint32_t code, void *fn)
{
	unsigned int before = lookups;

	assert(rp2040_rom_mem_fn[offset / 4] == NULL);
	assert(rp2040_rom_mem_resolve(offset) == fn);
	assert(lookups == before + 1);
	assert(last_code == code);
	assert(rp2040_rom_mem_fn[offset / 4] == fn);
}

int main(void)
{
	/* The offsets are those the entries of rp2040_rom_mem_ops.S use and
	 * the order of the Pico SDK's aeabi_mem_funcs.
	 */
	assert(RP2040_ROM_MEM_MEMSET == 0 && RP2040_ROM_MEM_MEMCPY == 4);
	assert(RP2040_ROM_MEM_MEMSET4 == 8 && RP2040_ROM_MEM_MEMCPY44 == 12);
	assert(sizeof(rp2040_rom_mem_fn) == RP2040_ROM_MEM_COUNT * sizeof(void *));

	/* .bss: nothing looked up before the first call. */
	for (unsigned int i = 0; i < RP2040_ROM_MEM_COUNT; i++) {
		assert(rp2040_rom_mem_fn[i] == NULL);
	}

	/* Each entry looks up its own code and stores only its own slot. */
	check(RP2040_ROM_MEM_MEMCPY, CODE_MEMCPY, &rom_memcpy);
	assert(rp2040_rom_mem_fn[RP2040_ROM_MEM_MEMSET / 4] == NULL);
	assert(rp2040_rom_mem_fn[RP2040_ROM_MEM_MEMSET4 / 4] == NULL);
	assert(rp2040_rom_mem_fn[RP2040_ROM_MEM_MEMCPY44 / 4] == NULL);
	check(RP2040_ROM_MEM_MEMSET, CODE_MEMSET, &rom_memset);
	check(RP2040_ROM_MEM_MEMCPY44, CODE_MEMCPY44, &rom_memcpy44);
	check(RP2040_ROM_MEM_MEMSET4, CODE_MEMSET4, &rom_memset4);

	/* A second lookup of the same entry (another context that saw NULL
	 * meanwhile) gives and stores the same routine.
	 */
	assert(rp2040_rom_mem_resolve(RP2040_ROM_MEM_MEMCPY) == &rom_memcpy);
	assert(rp2040_rom_mem_fn[RP2040_ROM_MEM_MEMCPY / 4] == &rom_memcpy);
	assert(lookups == 5);

	printf("rp2040_rom_mem: ok\n");
	return 0;
}
