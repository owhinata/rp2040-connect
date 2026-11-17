/* SPDX-License-Identifier: MIT */
/* The memcpy() and memset() of the early boot with CONFIG_NANO_ROM_MEM_OPS.
 *
 * memcpy(), memset() and __aeabi_memcpy*()/__aeabi_memset*() are wrapped
 * (--wrap, CMakeLists.txt) to the bootrom's routines of
 * common/drivers/rp2040_rom, which look the routines up on first use and
 * keep them in .bss.  The kernel's weak arch_early_memset() and
 * arch_early_memcpy() (arch/common/init.c) call memset() and memcpy(): they
 * clear .bss and copy .data (z_bss_zero(), z_data_copy()) and fill the
 * interrupt stack before that (CONFIG_INIT_STACKS, cortex_m/reset.S), so
 * they must not go through the wrappers.  These call picolibc's own
 * routines, which need no state.
 */
#include <stddef.h>

#include <zephyr/arch/common/init.h>

void *__real_memcpy(void *dst, const void *src, size_t n);
void *__real_memset(void *dst, int c, size_t n);

void arch_early_memset(void *dst, int c, size_t n)
{
	(void)__real_memset(dst, c, n);
}

void arch_early_memcpy(void *dst, const void *src, size_t n)
{
	(void)__real_memcpy(dst, src, n);
}
