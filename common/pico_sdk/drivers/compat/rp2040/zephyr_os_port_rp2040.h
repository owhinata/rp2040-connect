/* SPDX-License-Identifier: MIT */
/* The primitives of the generic kernel part of the Zephyr compatibility
 * layer (common/compat/kernel_generic/zephyr_kernel_os.h) and of a periodic
 * kernel tick that are the same for every OS on the Pico SDK on the RP2040:
 * the OS's port header ("zephyr_os_port.h", read by its
 * zephyr_kernel_os_impl.h) includes this one.  The OS's tick hook uses the
 * last two to keep its tick count up with the TIMER (tick_catchup.h of
 * common/compat/kernel_generic).
 *
 *   z_os_busy_wait_us(us)      spins on the TIMER (busy_wait_us_32()).  Any
 *                              context
 *   z_os_tick_now_us32()       the TIMER's low 32 bits (time_us_32(): one
 *                              read of TIMERAWL), for the tick hook's
 *                              catch-up.  Any context
 *   z_os_tick_pend()           pends SysTick (ICSR.PENDSTSET): the OS's
 *                              tick handler runs once more, right after the
 *                              current interrupt, and counts one more tick
 *                              (a pend that meets a pending SysTick merges
 *                              with it).  From the tick interrupt
 */

#ifndef COMMON_PICO_SDK_DRIVERS_COMPAT_RP2040_ZEPHYR_OS_PORT_RP2040_H
#define COMMON_PICO_SDK_DRIVERS_COMPAT_RP2040_ZEPHYR_OS_PORT_RP2040_H

#include <stdint.h>
#include "hardware/regs/m0plus.h"
#include "hardware/structs/scb.h"
#include "hardware/timer.h"

static inline void z_os_busy_wait_us(uint32_t us)
{
	busy_wait_us_32(us);
}

static inline uint32_t z_os_tick_now_us32(void)
{
	return time_us_32();
}

static inline void z_os_tick_pend(void)
{
	scb_hw->icsr = M0PLUS_ICSR_PENDSTSET_BITS;
}

#endif /* COMMON_PICO_SDK_DRIVERS_COMPAT_RP2040_ZEPHYR_OS_PORT_RP2040_H */
