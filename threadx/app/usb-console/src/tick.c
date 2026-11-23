/* SPDX-License-Identifier: MIT */
/* The kernel tick: SysTick's interrupt, 1 ms (isr_systick, the Pico SDK's
 * name of the vector; SysTick is set up by tx_initialize_low_level.c).
 *
 * Each interrupt counts one ThreadX tick (_tx_timer_interrupt() of the
 * Cortex-M0 port: the system clock, the running thread's time slice, and,
 * with TX_TIMER_PROCESS_IN_ISR, the timers, sleeps and timeouts that are
 * due), then runs the Zephyr compatibility layer's timeouts (the timers of
 * delayable work items, z_add_timeout(): zephyr_compat_tick()).  A thread
 * either makes ready is switched to by PendSV after this handler.  The
 * time slices of 20 ms are ThreadX's own (z_os_thread_start() gives them,
 * priorities_os.h).
 *
 * It also keeps the count up with the TIMER (tick_catchup.h, shared with the
 * FreeRTOS build): when tick interrupts were lost while interrupts were
 * masked (a flash erase), it pends SysTick again, once per lost tick, so
 * this handler runs again right after it returns and counts each of them
 * as a tick of its own, through the layer's timeouts too.  SysTick is above
 * PendSV, so the added ticks run one after another before the scheduler
 * switches.  The TIMER is read first, for the least latency since the
 * tick's interrupt.
 */

#include <stdint.h>

#include "tx_api.h"

#include "zephyr_compat_os.h"
#include "zephyr_os_port.h"

/* tick_catchup.h reads the tick rate under the FreeRTOS build's name. */
#define configTICK_RATE_HZ TX_TIMER_TICKS_PER_SECOND
/* Its path, wherever the FreeRTOS build keeps it (threadx_kernel.cmake). */
#include TICK_CATCHUP_HEADER

/* The Cortex-M0 port's tick (src/tx_timer_interrupt.S), which ThreadX
 * declares in no header.
 */
VOID _tx_timer_interrupt(VOID);

void isr_systick(void);

/* Used in the tick interrupt only. */
static struct tick_catchup catchup;

void isr_systick(void)
{
	uint32_t now = z_os_tick_now_us32();

	_tx_timer_interrupt();
	if (tick_catchup_step(&catchup, now)) {
		z_os_tick_pend();
	}
	zephyr_compat_tick();
}
