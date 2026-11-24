/* SPDX-License-Identifier: MIT */
/* What the FreeRTOS part of the Zephyr compatibility layer
 * (common/compat/port/freertos) has that depends on the FreeRTOS port and the
 * board: here the RP2040 port of FreeRTOS on the Pico SDK.  The host tests
 * have their own (freertos/tests/compat/zephyr_os_port.h, the FreeRTOS POSIX
 * port).  These four are primitives of the generic kernel part
 * (zephyr_kernel_os.h states what each must do; zephyr_kernel_os_impl.h
 * includes this header), and the shell's consoles use z_os_in_isr() too;
 * the ones that are the same for every OS on the Pico SDK (z_os_busy_wait_us()
 * and the tick hook's z_os_tick_now_us32() and z_os_tick_pend()) are
 * common/compat/port/pico_sdk/zephyr_os_port_rp2040.h, included
 * here.
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
 *
 * It also makes rp2040_timer_us() (microseconds since the start, the RP2040
 * TIMER, common/chip/rp2040) available: z_os_clock_us() is it.
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_PORT_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "rp2040_timer_us.h"
#include "zephyr_os_port_rp2040.h"

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

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_PORT_H */
