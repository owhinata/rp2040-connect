/* SPDX-License-Identifier: MIT */
/* k_heap of <zephyr/kernel.h> for an OS whose part of the layer does not
 * have one: the OS's zephyr_os.h includes this header (the FreeRTOS build's,
 * common/compat/kernel_generic/zephyr_os.h).  k_heap: a heap of a fixed size in
 * its own static buffer (Zephyr's K_HEAP_DEFINE(), k_heap_alloc(),
 * k_heap_free(); zephyr_heap.c): first fit, an 8-byte header before each
 * block, blocks rounded up to 8 bytes and joined again when freed, so an
 * allocation fails (NULL) when no free block of the buffer is large enough,
 * as Zephyr's (whose sys_heap keeps a little of the buffer for itself and 4
 * bytes per block).  Only K_NO_WAIT: nothing in the build waits for memory,
 * and another timeout does not wait either.  Any task (not interrupt
 * handlers); OS-independent (k_spin_lock()).  Only the Zephyr binding of
 * the NINA's sockets uses it (its name resolution,
 * common/drivers/nina/port/zephyr/nina_socket.c), so it is part of the
 * network part of the layer for an OS without a network stack of its own
 * (common/compat/net_offload).  k_heap_alloc() is a macro on z_heap_alloc()
 * because zephyr_os.h comes before k_timeout_t.
 */

#ifndef COMMON_COMPAT_NET_OFFLOAD_ZEPHYR_HEAP_H
#define COMMON_COMPAT_NET_OFFLOAD_ZEPHYR_HEAP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

#define k_heap_alloc(heap, bytes, timeout) z_heap_alloc((heap), (bytes), (timeout).ticks)

#endif /* COMMON_COMPAT_NET_OFFLOAD_ZEPHYR_HEAP_H */
