/* SPDX-License-Identifier: MIT */
/* memcpy() and memset() through the RP2040 bootrom's routines, as the
 * Pico SDK's pico_mem_ops does (src/rp2_common/pico_mem_ops/mem_ops_aeabi.S).
 *
 * rp2040_rom_mem_ops.S defines memcpy(), memset(), __aeabi_memcpy{,4,8}() and
 * __aeabi_memset{,4,8}() (with RP2040_ROM_MEM_WRAP, the same names with
 * __wrap_, for the linker's --wrap).  Each loads its bootrom routine from
 * rp2040_rom_mem_fn[] and jumps to it; while the entry is still NULL it calls
 * rp2040_rom_mem_resolve() (rp2040_rom_mem_lookup.c) first, which looks the
 * routine up in the bootrom's function table and stores it.
 *
 * rp2040_rom_mem_fn[] is in .bss: the routines may only be called once .bss
 * is zero (see rp2040_rom_mem_lookup.c).
 *
 * This header is included by the assembler too.
 */
#ifndef RP2040_ROM_MEM_H_
#define RP2040_ROM_MEM_H_

/* Byte offsets of the entries of rp2040_rom_mem_fn[]. */
#define RP2040_ROM_MEM_MEMSET   0  /* 'M','S': memset(ptr, c, n) */
#define RP2040_ROM_MEM_MEMCPY   4  /* 'M','C': memcpy(dest, src, n) */
#define RP2040_ROM_MEM_MEMSET4  8  /* 'S','4': memset4(), ptr word aligned */
#define RP2040_ROM_MEM_MEMCPY44 12 /* 'C','4': memcpy44(), dest and src word aligned */
#define RP2040_ROM_MEM_COUNT    4

#ifndef __ASSEMBLER__
#include <stdint.h>

/* The bootrom's function table codes, in the order of the entries. */
#define RP2040_ROM_CODE(c1, c2) ((uint32_t)(c1) | ((uint32_t)(c2) << 8))

/* The routines found so far (NULL: not looked up yet). */
extern void *rp2040_rom_mem_fn[RP2040_ROM_MEM_COUNT];

/* Look up the routine of an entry (offset: RP2040_ROM_MEM_*), store it in
 * rp2040_rom_mem_fn[] and return it.
 */
void *rp2040_rom_mem_resolve(unsigned int offset);
#endif

#endif /* RP2040_ROM_MEM_H_ */
