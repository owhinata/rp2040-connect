/* SPDX-License-Identifier: Apache-2.0 */
/* The block allocator of <zephyr/sys/mem_blocks.h>: Zephyr's
 * lib/mem_blocks/mem_blocks.c sys_mem_blocks_alloc_contiguous() and
 * sys_mem_blocks_free_contiguous() with the first-fit search and the
 * checks of lib/utils/bitarray.c (sys_bitarray_alloc(),
 * sys_bitarray_free()), on a plain bitmap.  OS-independent.  Part of the
 * small Zephyr compatibility layer of the port (nuttx/drivers/compat).
 */

#ifdef __NuttX__
#  include <nuttx/config.h>
#endif

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/mem_blocks.h>

static bool bit_get(const uint32_t *bitmap, size_t bit)
{
  return (bitmap[bit / 32] >> (bit % 32)) & 1U;
}

static void bits_set(uint32_t *bitmap, size_t offset, size_t count,
                     bool value)
{
  for (size_t bit = offset; bit < offset + count; bit++)
    {
      if (value)
        {
          bitmap[bit / 32] |= 1U << (bit % 32);
        }
      else
        {
          bitmap[bit / 32] &= ~(1U << (bit % 32));
        }
    }
}

int sys_mem_blocks_alloc_contiguous(sys_mem_blocks_t *mem_block,
                                    size_t count, void **out_block)
{
  k_spinlock_key_t key;
  size_t offset;
  size_t bit;
  int ret = -ENOMEM;

  if (count == 0)
    {
      /* Nothing to allocate */

      *out_block = NULL;
      return 0;
    }

  if (count > mem_block->info.num_blocks)
    {
      /* Definitely not enough blocks to be allocated */

      return -ENOMEM;
    }

  key = k_spin_lock(&mem_block->lock);

  /* The first run of count free blocks, from the lowest offset. */

  offset = 0;
  while (offset + count <= mem_block->info.num_blocks)
    {
      for (bit = offset; bit < offset + count; bit++)
        {
          if (bit_get(mem_block->bitmap, bit))
            {
              break;
            }
        }

      if (bit == offset + count)
        {
          bits_set(mem_block->bitmap, offset, count, true);
          *out_block = mem_block->buffer +
                       (offset << mem_block->info.blk_sz_shift);
          ret = 0;
          break;
        }

      /* Fast-forward to the block just after the allocated one. */

      offset = bit + 1;
    }

  k_spin_unlock(&mem_block->lock, key);
  return ret;
}

int sys_mem_blocks_free_contiguous(sys_mem_blocks_t *mem_block, void *block,
                                   size_t count)
{
  k_spinlock_key_t key;
  uint8_t *blk = block;
  size_t offset;
  int ret = 0;

  if (count == 0)
    {
      /* Nothing to be freed. */

      return 0;
    }

  if (count > mem_block->info.num_blocks)
    {
      return -EINVAL;
    }

  /* Make sure incoming block is within the mem_block buffer */

  if (blk < mem_block->buffer)
    {
      return -EFAULT;
    }

  offset = (size_t)(blk - mem_block->buffer) >>
           mem_block->info.blk_sz_shift;
  if (offset >= mem_block->info.num_blocks)
    {
      return -EFAULT;
    }

  key = k_spin_lock(&mem_block->lock);

  /* sys_bitarray_free(): the whole run must be allocated, and lie inside
   * the bitmap.
   */

  if (offset + count > mem_block->info.num_blocks)
    {
      ret = -EINVAL;
    }
  else
    {
      for (size_t bit = offset; bit < offset + count; bit++)
        {
          if (!bit_get(mem_block->bitmap, bit))
            {
              ret = -EFAULT;
              break;
            }
        }
    }

  if (ret == 0)
    {
      bits_set(mem_block->bitmap, offset, count, false);
    }

  k_spin_unlock(&mem_block->lock, key);
  return ret;
}
