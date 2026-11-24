/* SPDX-License-Identifier: MIT */
/* k_heap of <zephyr/kernel.h> (Zephyr's K_HEAP_DEFINE(), k_heap_alloc(),
 * k_heap_free()), part of the layer's kernel API whichever kernel part an
 * OS uses (common/compat/include/zephyr/kernel.h includes this header at its end):
 * a heap of a fixed size in its own static buffer (zephyr_heap.c): first
 * fit, an 8-byte header before each block, blocks rounded up to 8 bytes and
 * joined again when freed, so an allocation fails (NULL) when no free block
 * of the buffer is large enough, as Zephyr's (whose sys_heap keeps a little
 * of the buffer for itself and 4 bytes per block).  Only K_NO_WAIT: nothing
 * in the builds waits for memory, and another timeout does not wait either.
 * Any thread (not interrupt handlers); OS-independent, on k_spin_lock()
 * only.  The NINA's Zephyr binding of its sockets uses it (its name
 * resolution, common/drivers/nina/port/zephyr/nina_socket.c); a build that
 * compiles no user does not compile zephyr_heap.c.
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_HEAP_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_HEAP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/kernel.h>

struct k_heap {
	uint8_t *buf;
	size_t size; /* bytes, a multiple of 8 */
	bool ready;  /* the buffer holds its first free block */
};

#define K_HEAP_DEFINE(name, bytes)                                                                 \
	static uint8_t __attribute__((aligned(8))) z_heap_mem_##name[(bytes) & ~(size_t)7];        \
	struct k_heap name = {.buf = z_heap_mem_##name, .size = (bytes) & ~(size_t)7}

void *z_heap_alloc(struct k_heap *heap, size_t bytes, int64_t timeout_ticks);
void k_heap_free(struct k_heap *heap, void *mem);

static inline void *k_heap_alloc(struct k_heap *heap, size_t bytes, k_timeout_t timeout)
{
	return z_heap_alloc(heap, bytes, timeout.ticks);
}

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_HEAP_H */
