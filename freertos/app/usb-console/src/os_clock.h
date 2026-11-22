/* SPDX-License-Identifier: MIT */
/* Time on FreeRTOS: the clock since the start from the RP2040 TIMER, and the
 * kernel tick.
 *
 * The tick is 1 ms (configTICK_RATE_HZ 1000, FreeRTOSConfig.h; the Zephyr
 * and NuttX builds' is 100 us) and 32 bits wide (the RP2040 port takes no
 * 64-bit tick): it wraps after 2^32 / configTICK_RATE_HZ s, about 49.7 days,
 * and stands still while interrupts stay disabled for longer than a tick (a
 * flash erase), catching up when they are enabled again (tick.c,
 * tick_catchup.h).  So the time since the start is not counted in ticks but
 * read from the TIMER (64 bits, 1 us, from the 12 MHz crystal) with
 * rp2040_timer_us() of common/chip/rp2040, the same read as the Zephyr and
 * NuttX builds'.  The compatibility layer's k_uptime_*() and k_cycle_*()
 * (F02) are built on it.
 *
 * Do not use the Pico SDK's sleep_ms()/sleep_us(): they wait for a TIMER
 * alarm and spin the last PICO_TIME_SLEEP_OVERHEAD_ADJUST_US (150 us) in
 * the CPU.
 */

#ifndef OS_CLOCK_H
#define OS_CLOCK_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* rp2040_timer_us(): microseconds since the TIMER left reset at the start
 * (every start, including a restart by the watchdog), from the raw
 * registers (TIMERAWH, TIMERAWL and TIMERAWH again until the two high words
 * agree, as the Pico SDK's time_us_64() reads them): no side effect and no
 * lock, so it may be called from tasks, interrupt handlers and with
 * interrupts disabled.  Static inline, without a call.
 */
#include "rp2040_timer_us.h"

/* The kernel's tick count since the scheduler started (wraps, see above). */
static inline uint32_t os_tick_count(void)
{
	return (uint32_t)(xPortIsInsideInterrupt() ? xTaskGetTickCountFromISR()
						   : xTaskGetTickCount());
}

/* us in ticks, rounded up (Zephyr's K_USEC() and K_MSEC():
 * k_us_to_ticks_ceil64()), saturating at portMAX_DELAY - 1.
 */
static inline TickType_t os_ticks_from_us_ceil(uint64_t us)
{
	uint64_t ticks;

	if (us > UINT64_MAX / configTICK_RATE_HZ) {
		return portMAX_DELAY - 1;
	}
	ticks = (us * configTICK_RATE_HZ + 999999u) / 1000000u;
	return ticks < portMAX_DELAY - 1 ? (TickType_t)ticks : portMAX_DELAY - 1;
}

/* Sleeps as long as Zephyr's k_usleep(us) with a tick of this length (Zephyr's
 * is 100 us, this one 1 ms: a sleep shorter than 1 ms lasts 1 to 2 ms):
 * Zephyr wakes a relative timeout at the tick boundary after the current one plus the
 * rounded-up count (zephyr/kernel/timeout.c), (n, n + 1] ticks for n
 * rounded-up ticks; vTaskDelay(n) sleeps (n - 1, n] ticks
 * (FreeRTOS-Kernel/tasks.c), so this delays n + 1.  0 yields, as
 * k_sleep(K_NO_WAIT) does.  Not from an interrupt handler.
 */
static inline void os_sleep_us(uint64_t us)
{
	TickType_t ticks = os_ticks_from_us_ceil(us);

	if (ticks == 0) {
		/* Behind the other ready tasks of the priority, as Zephyr's yield. */
		taskYIELD();
		return;
	}
	vTaskDelay(ticks < portMAX_DELAY - 1 ? ticks + 1 : ticks);
}

#endif /* OS_CLOCK_H */
