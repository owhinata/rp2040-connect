/* SPDX-License-Identifier: MIT */
/* What the ThreadX part of the Zephyr compatibility layer
 * (threadx/drivers/compat) has that depends on the ThreadX port and the
 * board: here ThreadX's Cortex-M0 port (ports/cortex_m0/gnu) on the
 * RP2040 with the Pico SDK.  The host tests have their own
 * (threadx/tests/compat/zephyr_os_port.h, ThreadX's Linux port).
 * zephyr_kernel_os_impl.h includes this header.
 *
 *   z_os_in_isr()          true in an interrupt handler (IPSR != 0; the
 *                          Cortex-M ports count no interrupt nesting in
 *                          _tx_thread_system_state)
 *   z_tx_irq_save(),       PRIMASK: the port's own inline TX_DISABLE and
 *   z_tx_irq_restore(s)    TX_RESTORE (__disable_interrupts(),
 *                          __restore_interrupts() of tx_port.h; the state is
 *                          PRIMASK, 0 or 1)
 *   z_os_busy_wait_us(us)  spins on the TIMER (busy_wait_us_32())
 *   z_os_clock_us()        the RP2040 TIMER (rp2040_timer_us(),
 *                          common/chip/rp2040)
 *   Z_TX_ASSERT(test)      app_assert_failed() of the application
 *
 * Not compiled yet: the board's build comes with its start-up (the plan's
 * T2).
 */

#ifndef THREADX_DRIVERS_COMPAT_RP2040_ZEPHYR_OS_PORT_H
#define THREADX_DRIVERS_COMPAT_RP2040_ZEPHYR_OS_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/timer.h"
#include "tx_api.h"
#include "rp2040_timer_us.h"

static inline bool z_os_in_isr(void)
{
	return __get_ipsr_value() != 0;
}

static inline UINT z_tx_irq_save(void)
{
	return __disable_interrupts();
}

static inline void z_tx_irq_restore(UINT state)
{
	__restore_interrupts(state);
}

static inline void z_os_busy_wait_us(uint32_t us)
{
	busy_wait_us_32(us);
}

static inline uint64_t z_os_clock_us(void)
{
	return rp2040_timer_us();
}

void app_assert_failed(const char *file, int line);

#define Z_TX_ASSERT(test)                                                                          \
	do {                                                                                       \
		if (!(test)) {                                                                     \
			app_assert_failed(__FILE__, __LINE__);                                     \
		}                                                                                  \
	} while (0)

#endif /* THREADX_DRIVERS_COMPAT_RP2040_ZEPHYR_OS_PORT_H */
