/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's fixed-size block allocator (<zephyr/sys/mem_blocks.h>) as RTIO
 * uses it for the buffers of a context's memory pool: a buffer of
 * num_blocks blocks of 2^blk_sz_shift bytes and a bitmap of the allocated
 * ones.  sys_mem_blocks_alloc_contiguous() takes the lowest run of free
 * blocks that is long enough (first fit, as Zephyr's sys_bitarray_alloc()
 * does), sys_mem_blocks_free_contiguous() gives a run back.  The
 * implementation is zephyr_mem_blocks.c.  Part of the small Zephyr
 * compatibility layer of the port (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_MEM_BLOCKS_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_MEM_BLOCKS_H

#include <stddef.h>
#include <stdint.h>

#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>

struct sys_mem_blocks_info
{
  uint32_t num_blocks;
  uint8_t blk_sz_shift;
};

typedef struct sys_mem_blocks
{
  struct sys_mem_blocks_info info;
  uint8_t *buffer;
  uint32_t *bitmap;           /* bit set: block allocated */
  struct k_spinlock lock;
} sys_mem_blocks_t;

/* zephyr/sys/mem_blocks.h _SYS_MEM_BLOCKS_DEFINE_WITH_EXT_BUF(): blk_sz
 * must be a power of two.
 */

#define _SYS_MEM_BLOCKS_DEFINE_WITH_EXT_BUF(name, blk_sz, num_blks, buf, mbmod) \
  BUILD_ASSERT(((blk_sz) & ((blk_sz) - 1)) == 0, \
               "block size must be a power of two"); \
  static uint32_t _sys_mem_blocks_bitmap_##name[DIV_ROUND_UP(num_blks, 32)]; \
  mbmod sys_mem_blocks_t name = \
    { \
      .info = { .num_blocks = (num_blks), \
                .blk_sz_shift = __builtin_ctz(blk_sz) }, \
      .buffer = (buf), \
      .bitmap = _sys_mem_blocks_bitmap_##name, \
    }

/* 0, or -ENOMEM if there is no run of count free blocks. */

int sys_mem_blocks_alloc_contiguous(sys_mem_blocks_t *mem_block,
                                    size_t count, void **out_block);

/* 0, -EINVAL (count too large), or -EFAULT (block outside the buffer or not
 * all allocated: nothing is freed).
 */

int sys_mem_blocks_free_contiguous(sys_mem_blocks_t *mem_block, void *block,
                                   size_t count);

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_MEM_BLOCKS_H */
