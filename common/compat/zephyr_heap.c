/* SPDX-License-Identifier: MIT */
/* k_heap of <zephyr/kernel.h> (zephyr_heap.h): a first-fit
 * heap in the static buffer of K_HEAP_DEFINE().  The buffer is a row of
 * blocks, each an 8-byte header (its size, header included, and whether it is
 * in use) and its data, rounded up to 8 bytes; an allocation takes the first
 * free block that is large enough and splits off the rest when that can hold
 * a block of its own, and a free joins the free blocks that follow each other
 * again.  The buffer becomes one free block at the first allocation.
 * Zephyr's k_heap is a sys_heap (lib/heap/heap.c) with a wait queue; what
 * the build relies on is the same: allocations come from that buffer only,
 * one that does not fit gives NULL at once (K_NO_WAIT), and freed memory is
 * used again.  OS-independent (k_spin_lock()): the same with either kernel
 * part of the layer.  The FreeRTOS build compiles it (its NINA sockets'
 * name resolution); the NuttX build has no user and does not.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/__assert.h>

struct z_heap_block {
	uint32_t size; /* bytes, header included, a multiple of 8 */
	uint32_t used;
};

#define Z_HEAP_HEADER sizeof(struct z_heap_block)

BUILD_ASSERT(sizeof(struct z_heap_block) == 8, "the data stays 8-byte aligned");

static struct k_spinlock heap_lock;

static struct z_heap_block *block_at(struct k_heap *heap, size_t offset)
{
	return (struct z_heap_block *)(void *)(heap->buf + offset);
}

void *z_heap_alloc(struct k_heap *heap, size_t bytes, int64_t timeout_ticks)
{
	k_spinlock_key_t key;
	size_t need, offset;
	void *mem = NULL;

	ARG_UNUSED(timeout_ticks);

	if (bytes == 0 || bytes > heap->size) {
		return NULL;
	}
	need = ((bytes + 7) & ~(size_t)7) + Z_HEAP_HEADER;

	key = k_spin_lock(&heap_lock);
	if (!heap->ready) {
		if (heap->size >= 2 * Z_HEAP_HEADER) {
			*block_at(heap, 0) = (struct z_heap_block){.size = heap->size, .used = 0};
			heap->ready = true;
		}
	}
	for (offset = 0; heap->ready && offset < heap->size;) {
		struct z_heap_block *b = block_at(heap, offset);

		if (!b->used && b->size >= need) {
			if (b->size - need >= 2 * Z_HEAP_HEADER) {
				*block_at(heap, offset + need) = (struct z_heap_block){
					.size = b->size - (uint32_t)need, .used = 0};
				b->size = (uint32_t)need;
			}
			b->used = 1;
			mem = (uint8_t *)b + Z_HEAP_HEADER;
			break;
		}
		offset += b->size;
	}
	k_spin_unlock(&heap_lock, key);
	return mem;
}

void k_heap_free(struct k_heap *heap, void *mem)
{
	k_spinlock_key_t key;
	size_t offset;

	if (mem == NULL) {
		return;
	}
	__ASSERT_NO_MSG((uint8_t *)mem >= heap->buf + Z_HEAP_HEADER &&
			(uint8_t *)mem < heap->buf + heap->size);

	key = k_spin_lock(&heap_lock);
	((struct z_heap_block *)(void *)((uint8_t *)mem - Z_HEAP_HEADER))->used = 0;
	/* Join each free block with the free ones after it. */
	for (offset = 0; offset < heap->size;) {
		struct z_heap_block *b = block_at(heap, offset);

		while (!b->used && offset + b->size < heap->size &&
		       !block_at(heap, offset + b->size)->used) {
			b->size += block_at(heap, offset + b->size)->size;
		}
		offset += b->size;
	}
	k_spin_unlock(&heap_lock, key);
}
