/* SPDX-License-Identifier: MIT */
/* What the long SPI1 transfers of the NINA link by DMA
 * (nina_bus_dma_rp2040.c next to this file) need from the OS: the calling
 * thread sleeps until the DMA interrupt handler wakes it or a time runs out.
 * The file includes this header; it includes the OS's part,
 * "nina_bus_dma_os_impl.h" (port/<OS>/, on the include path of the
 * build), then declares the inline functions that part must define, so a
 * missing or different one does not compile.  Static inline, so that the
 * compiler makes one function of each with its caller (the socket writes
 * and the module's reads in chunks of 128 bytes go this way).
 *
 * One transfer at a time (nina_os_lock(), nina_proto.c).  The wait is on a
 * wake-up the OS's part chooses: the calling thread's own, which may be
 * shared with the NINA's waits of nina_os.h (FreeRTOS: task notification
 * index 2), or an object of the binding's (ThreadX: one semaphore of the
 * transfers).  The transfer drops a wake-up left from before it starts (the
 * core sets its requests before it posts and looks at them before a wait).
 *
 * nina_bus_dma_os_impl.h: types and macros
 *
 *   nina_bus_dma_os_thread_t   a thread to wake, a pointer type; NULL is
 *                              none
 *   nina_bus_dma_os_woken_t    what nina_bus_dma_os_wake_from_isr() notes
 *                              for nina_bus_dma_os_isr_end()
 *   NINA_BUS_DMA_OS_NOT_WOKEN  its value at the start of the handler
 *   nina_bus_dma_os_ticks_t    a count of the OS's kernel ticks, unsigned;
 *                              differences of two tick counts wrap
 *
 * Threads, with nina_os_lock() held:
 *
 *   nina_bus_dma_os_running()      true when the scheduler runs (the DMA's
 *                                  waits need a thread to block; before,
 *                                  every transfer stays polled)
 *   nina_bus_dma_os_self()         the calling thread
 *   nina_bus_dma_os_wake_clear()   drops a wake-up left from before (with
 *                                  a shared wake-up, also one given by a
 *                                  post of nina_os.h): a later
 *                                  nina_bus_dma_os_wake_wait() blocks until
 *                                  a new one
 *   nina_bus_dma_os_ticks_from_ms(ms)
 *                                  MS milliseconds rounded up to ticks, plus
 *                                  one tick (as the NINA's other waits)
 *   nina_bus_dma_os_tick_count()   the kernel's tick count
 *   nina_bus_dma_os_wake_wait(t)   blocks the calling thread until the
 *                                  wake-up (taking it) or for at most T
 *                                  ticks (T > 0), or returns earlier for no
 *                                  reason; the caller looks at its flag and
 *                                  the time again in each case
 *
 * The DMA interrupt handler (any time, also with no thread waiting):
 *
 *   nina_bus_dma_os_wake_from_isr(thread, &woken)
 *                                  gives THREAD (the waiting thread) its
 *                                  wake-up (kept until it waits) and notes
 *                                  in WOKEN whether a switch to it is due
 *   nina_bus_dma_os_isr_end(woken) at the end of the handler: switches to
 *                                  the woken thread when WOKEN says so (or
 *                                  nothing, if the OS switches at the end
 *                                  of every interrupt by itself)
 */

#ifndef NINA_BUS_DMA_OS_H_
#define NINA_BUS_DMA_OS_H_

#include <stdbool.h>
#include <stdint.h>

#include "nina_bus_dma_os_impl.h"

static inline bool nina_bus_dma_os_running(void);
static inline nina_bus_dma_os_thread_t nina_bus_dma_os_self(void);
static inline void nina_bus_dma_os_wake_clear(void);
static inline nina_bus_dma_os_ticks_t nina_bus_dma_os_ticks_from_ms(uint64_t ms);
static inline nina_bus_dma_os_ticks_t nina_bus_dma_os_tick_count(void);
static inline void nina_bus_dma_os_wake_wait(nina_bus_dma_os_ticks_t ticks);
static inline void nina_bus_dma_os_wake_from_isr(nina_bus_dma_os_thread_t thread,
						 nina_bus_dma_os_woken_t *woken);
static inline void nina_bus_dma_os_isr_end(nina_bus_dma_os_woken_t woken);

#endif /* NINA_BUS_DMA_OS_H_ */
