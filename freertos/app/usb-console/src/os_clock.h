/* SPDX-License-Identifier: MIT */
/* Time on FreeRTOS: the clock since the start from the RP2040 TIMER, and the
 * kernel tick.
 *
 * The tick is 100 us by default (configTICK_RATE_HZ 10000, as the Zephyr
 * and NuttX builds' CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000 /
 * CONFIG_USEC_PER_TICK=100), or 1 ms with the build option
 * NANO_TICK_HZ=1000 (FreeRTOSConfig.h), and 32 bits wide (the RP2040 port
 * takes no 64-bit tick): it wraps after 2^32 / configTICK_RATE_HZ s, about
 * 4.97 days at 100 us, and loses counts while interrupts stay disabled for
 * longer than a tick (a flash erase).  So the time since the
 * start is not counted in ticks but read from the TIMER (64 bits, 1 us,
 * from the 12 MHz crystal), the time base of the Zephyr build's
 * time_sync_mono_us() and the NuttX build's nano_mono_us().  The
 * compatibility layer's k_uptime_*() and k_cycle_*() (F02) are built on
 * nano_mono_us().
 *
 * portTICK_PERIOD_MS is 1000 / configTICK_RATE_HZ = 0 at 10000 Hz: do not
 * use it; pdMS_TO_TICKS() is exact at both rates (10 or 1 tick per ms).  Do
 * not use the Pico
 * SDK's sleep_ms()/sleep_us() either: they wait for a TIMER alarm and spin
 * the last PICO_TIME_SLEEP_OVERHEAD_ADJUST_US (150 us) in the CPU.
 */

#ifndef OS_CLOCK_H
#define OS_CLOCK_H

#include <stdint.h>
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"

/* Microseconds since the TIMER left reset at the start (every start,
 * including a restart by the watchdog).  time_us_64() reads TIMERAWH,
 * TIMERAWL and TIMERAWH again until the two high words agree, the read of
 * time_sync_mono_us() and nano_mono_us(): no side effect and no lock, so it
 * may be called from tasks, interrupt handlers and with interrupts
 * disabled.
 */
static inline uint64_t nano_mono_us(void)
{
	return time_us_64();
}

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
 * is 100 us; with NANO_TICK_HZ=1000 this one is 1 ms, and a sleep shorter
 * than 1 ms lasts 1 to 2 ms): Zephyr wakes
 * a relative timeout at the tick boundary after the current one plus the
 * rounded-up count (zephyr/kernel/timeout.c), (n, n + 1] ticks for n
 * rounded-up ticks; vTaskDelay(n) sleeps (n - 1, n] ticks
 * (FreeRTOS-Kernel/tasks.c), so this delays n + 1.  0 yields, as
 * k_sleep(K_NO_WAIT) does.  Not from an interrupt handler.
 */
static inline void os_sleep_us(uint64_t us)
{
	TickType_t ticks = os_ticks_from_us_ceil(us);

	if (ticks == 0) {
		/* Behind the other ready tasks of the priority, as Zephyr's yield
		 * (z_sched_yield_pending, freertos/drivers/compat/zephyr_os_sched.h,
		 * read by FreeRTOSConfig.h).
		 */
		taskENTER_CRITICAL();
		z_sched_yield_pending = 1;
		taskYIELD();
		taskEXIT_CRITICAL();
		return;
	}
	vTaskDelay(ticks < portMAX_DELAY - 1 ? ticks + 1 : ticks);
}

#endif /* OS_CLOCK_H */
