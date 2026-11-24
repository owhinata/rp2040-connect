/* SPDX-License-Identifier: MIT */
/* The ThreadX part of the NINA link's DMA waits (port/pico_sdk/nina_bus_dma_os.h,
 * which describes each item): a ThreadX semaphore of the transfers' own
 * (nina_bus_dma_wake, binary: put with the ceiling 1), not a wake-up of the
 * thread as on FreeRTOS (ThreadX has none a driver may use: the per-thread
 * semaphore of the compatibility layer is the layer's wake flag).  One
 * transfer waits at a time (the caller holds nina_os_lock()), so one
 * semaphore serves every thread; the Wi-Fi thread's wake-up and the socket
 * event waits have their own (nina_os.c), and a transfer drops none of
 * theirs.  Made by the first transfer's nina_bus_dma_os_wake_clear() (with
 * nina_os_lock() held), never deleted.  Only the file of the DMA
 * transfers (nina_bus_dma_rp2040.c) includes this header.
 *
 *   nina_bus_dma_os_running()         a thread runs (tx_thread_identify():
 *                                     none before the kernel runs)
 *   nina_bus_dma_os_wake_clear()      tx_semaphore_get() without a wait
 *   nina_bus_dma_os_wake_wait()       tx_semaphore_get() for the ticks
 *   nina_bus_dma_os_wake_from_isr()   tx_semaphore_ceiling_put(.., 1); the
 *                                     thread is the one that waits on it
 *   nina_bus_dma_os_isr_end()         nothing: ThreadX's Cortex-M0 port pends
 *                                     PendSV in the put when the woken
 *                                     thread is more urgent, and the switch
 *                                     comes at the end of the handler
 */

#ifndef NINA_BUS_DMA_OS_IMPL_H_
#define NINA_BUS_DMA_OS_IMPL_H_

#include <stdbool.h>
#include <stdint.h>

#include "tx_api.h"

#if TX_TIMER_TICKS_PER_SECOND != 1000
#error "the NINA's SPI DMA waits are written for ThreadX's 1 ms tick (tx_user.h)"
#endif

typedef TX_THREAD *nina_bus_dma_os_thread_t;
typedef UINT nina_bus_dma_os_woken_t;
typedef ULONG nina_bus_dma_os_ticks_t;

#define NINA_BUS_DMA_OS_NOT_WOKEN 0u

/* Always inlined, so that the compiler sees the ThreadX calls in their
 * callers as if those called ThreadX directly (as Z_OS_INLINE of the
 * compatibility layer's zephyr_kernel_os_impl.h: with -Os the early inliner
 * would otherwise leave wrappers and change its later decisions).
 */
#define NINA_BUS_DMA_OS_INLINE static inline __attribute__((always_inline))

static TX_SEMAPHORE nina_bus_dma_wake;
static bool nina_bus_dma_wake_made;

NINA_BUS_DMA_OS_INLINE bool nina_bus_dma_os_running(void)
{
	return tx_thread_identify() != TX_NULL;
}

NINA_BUS_DMA_OS_INLINE nina_bus_dma_os_thread_t nina_bus_dma_os_self(void)
{
	return tx_thread_identify();
}

NINA_BUS_DMA_OS_INLINE void nina_bus_dma_os_wake_clear(void)
{
	if (!nina_bus_dma_wake_made) {
		nina_bus_dma_wake_made = tx_semaphore_create(&nina_bus_dma_wake,
							     (CHAR *)"nina_bus_dma", 0) == TX_SUCCESS;
		return;
	}
	(void)tx_semaphore_get(&nina_bus_dma_wake, TX_NO_WAIT);
}

NINA_BUS_DMA_OS_INLINE nina_bus_dma_os_ticks_t nina_bus_dma_os_ticks_from_ms(uint64_t ms)
{
	return (ULONG)((ms * TX_TIMER_TICKS_PER_SECOND + 999u) / 1000u + 1u);
}

NINA_BUS_DMA_OS_INLINE nina_bus_dma_os_ticks_t nina_bus_dma_os_tick_count(void)
{
	return tx_time_get();
}

NINA_BUS_DMA_OS_INLINE void nina_bus_dma_os_wake_wait(nina_bus_dma_os_ticks_t ticks)
{
	(void)tx_semaphore_get(&nina_bus_dma_wake, ticks);
}

/* THREAD is the one that waits on the semaphore; *WOKEN stays as it is
 * (nina_bus_dma_os_isr_end() has nothing to do).
 */
NINA_BUS_DMA_OS_INLINE void nina_bus_dma_os_wake_from_isr(nina_bus_dma_os_thread_t thread,
							  nina_bus_dma_os_woken_t *woken)
{
	(void)thread;
	(void)woken;
	(void)tx_semaphore_ceiling_put(&nina_bus_dma_wake, 1);
}

NINA_BUS_DMA_OS_INLINE void nina_bus_dma_os_isr_end(nina_bus_dma_os_woken_t woken)
{
	(void)woken;
}

#endif /* NINA_BUS_DMA_OS_IMPL_H_ */
