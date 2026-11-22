/* SPDX-License-Identifier: MIT */
/* What the FreeRTOS part of the Zephyr compatibility layer
 * (freertos/drivers/compat) has that depends on the FreeRTOS port and the
 * board: here the RP2040 port of FreeRTOS on the Pico SDK.  The host tests
 * have their own (freertos/tests/compat/zephyr_os_port.h, the FreeRTOS POSIX
 * port).  The first five are primitives of the generic kernel part
 * (zephyr_kernel_os.h states what each must do; zephyr_kernel_os_impl.h
 * includes this header); the last two are the tick hook's
 * (freertos/app/usb-console/src/tick.c), and the shell's consoles use
 * z_os_in_isr() too.
 *
 *   z_os_in_isr()              true in an interrupt handler (IPSR != 0)
 *   z_os_yield_from_isr(w)     at the end of an interrupt handler that made
 *                              a task ready: portYIELD_FROM_ISR() (PendSV)
 *   z_os_task_lock(),          k_spin_lock() in a task: the kernel's
 *   z_os_task_unlock()         critical section, taskENTER_CRITICAL()
 *                              (PRIMASK and a nesting count, so that the
 *                              FreeRTOS calls made inside, which enter and
 *                              leave critical sections of their own, do not
 *                              enable interrupts; a switch they ask for is
 *                              a pending PendSV, taken at the unlock)
 *   z_os_busy_wait_us(us)      spins on the TIMER (busy_wait_us_32())
 *   z_os_tick_now_us32()       the TIMER's low 32 bits (time_us_32(): one
 *                              read of TIMERAWL), for the tick hook's
 *                              catch-up (the application's tick.c,
 *                              tick_catchup.h)
 *   z_os_tick_pend()           pends SysTick (ICSR.PENDSTSET): the port's
 *                              tick handler runs once more, right after the
 *                              current interrupt, and counts one more tick
 *                              (a pend that meets a pending SysTick merges
 *                              with it)
 *
 * It also makes rp2040_timer_us() (microseconds since the start, the RP2040
 * TIMER, os_clock.h of the application) available: z_os_clock_us() is it.
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_PORT_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/regs/m0plus.h"
#include "hardware/structs/scb.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "os_clock.h"

static inline bool z_os_in_isr(void)
{
	return portCHECK_IF_IN_ISR() != 0;
}

static inline void z_os_yield_from_isr(BaseType_t woken)
{
	portYIELD_FROM_ISR(woken);
}

static inline void z_os_task_lock(void)
{
	taskENTER_CRITICAL();
}

static inline void z_os_task_unlock(void)
{
	taskEXIT_CRITICAL();
}

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

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_PORT_H */
