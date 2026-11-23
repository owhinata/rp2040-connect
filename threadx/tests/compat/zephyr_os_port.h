/* SPDX-License-Identifier: MIT */
/* The port header of the Zephyr compatibility layer (see
 * threadx/drivers/compat/rp2040/zephyr_os_port.h, the board's) for the host
 * tests on ThreadX's Linux port (ports/linux/gnu), with the ThreadX calls of
 * threadx/drivers/compat/zephyr_kernel_os_impl.h the host implementation of
 * the generic kernel part's primitives (zephyr_kernel_os.h).  There every
 * ThreadX thread is a Linux thread of which the scheduler lets one run at a
 * time, an interrupt is a Linux thread that suspends the running one
 * (_tx_thread_context_save()), the only one being the tick's, and masked
 * interrupts are the port's recursive Linux mutex held.
 *
 *   z_os_in_isr()          ThreadX's interrupt nesting, which the port's
 *                          context save and restore count in
 *                          _tx_thread_system_state (its values from
 *                          TX_INITIALIZE_IN_PROGRESS up mean that the
 *                          kernel is not running yet)
 *   z_tx_irq_save(),       the port's TX_DISABLE and TX_RESTORE
 *   z_tx_irq_restore(s)    (_tx_thread_interrupt_disable(), _restore(): the
 *                          state is TX_INT_ENABLE or TX_INT_DISABLE)
 *   z_os_busy_wait_us(us)  spins on the clock
 *   z_os_clock_us()        CLOCK_MONOTONIC through the shared
 *                          rp2040_timer_us() (tests/compat/rp2040_timer_us.h)
 *   Z_TX_ASSERT(test)      assert()
 */

#ifndef THREADX_TESTS_COMPAT_ZEPHYR_OS_PORT_H
#define THREADX_TESTS_COMPAT_ZEPHYR_OS_PORT_H

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "tx_api.h"
#include "tx_initialize.h"
#include "tx_thread.h"
#include "rp2040_timer_us.h"

static inline bool z_os_in_isr(void)
{
	ULONG state = _tx_thread_system_state;

	return state != 0 && state < TX_INITIALIZE_IN_PROGRESS;
}

static inline UINT z_tx_irq_save(void)
{
	return _tx_thread_interrupt_disable();
}

static inline void z_tx_irq_restore(UINT state)
{
	_tx_thread_interrupt_restore(state);
}

static inline void z_os_busy_wait_us(uint32_t us)
{
	uint64_t end = rp2040_timer_us() + us;

	while (rp2040_timer_us() < end) {
	}
}

static inline uint64_t z_os_clock_us(void)
{
	return rp2040_timer_us();
}

#define Z_TX_ASSERT(test) assert(test)

#endif /* THREADX_TESTS_COMPAT_ZEPHYR_OS_PORT_H */
