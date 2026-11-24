/* SPDX-License-Identifier: MIT */
/* The idle wait's hooks and the CPU load measured from idle windows,
 * cpu_load_get() (common/compat/cpu_load.h), on ThreadX.
 *
 * ThreadX has no idle thread: with no thread ready, its Cortex-M0 scheduler
 * waits inside PendSV (tx_thread_schedule.S, __tx_ts_wait), in a loop of
 *
 *   CPSID i; tx_low_power_enter(); DSB; WFI; ISB; tx_low_power_exit(); CPSIE i
 *
 * (tx_user.h: TX_LOW_POWER, TX_ENABLE_WFI).  The hooks below open the window
 * before the WFI and close it after, so the steps are those of the other
 * builds' idle pass (common/pico_sdk/app/usb-console/src/idle.c, Zephyr's
 * CONFIG_CPU_LOAD with the idle hook, NuttX's up_idle()): interrupts masked
 * (PRIMASK), open, WFI, close, interrupts enabled.  WFI wakes on a pending
 * interrupt while PRIMASK masks it (ARMv6-M), so one that comes before the
 * WFI is not missed (WFI returns at once), and its handler runs after the
 * window closes, at the CPSIE: interrupt handlers count as load.  The tick
 * (SysTick, every 1 ms) wakes the wait every tick and counts as load, as on
 * FreeRTOS.  When a handler makes a thread ready, the loop leaves the wait
 * and switches to it; the next wait opens a new window.
 *
 * The hooks run in handler mode (PendSV, the lowest priority) on the main
 * stack, interrupts masked; they read the TIMER (rp2040_timer_us(), 1 us)
 * and may toggle a GPIO, nothing that waits.  The arithmetic is
 * common/compat/cpu_load.h's, the steps of Zephyr's cpu_load_get_cpu().
 *
 * tx_low_power_enter() also blinks the LED (GPIO6, idle_blink.h of
 * common/pico_sdk/app/usb-console/src), with the time read for the window:
 * one comparison per wait, counted as idle as the wait is, and a toggle
 * every 500 ms.  A firmware that drives the LED itself builds this file
 * with IDLE_BLINK=0.
 */

#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "tx_api.h"

#include "app.h"
#include "cpu_load.h"
#include "idle_blink.h"
#include "rp2040_timer_us.h"

#if !defined(TX_LOW_POWER) || !defined(TX_ENABLE_WFI)
#error "the idle wait needs TX_LOW_POWER and TX_ENABLE_WFI (tx_user.h)"
#endif

#ifndef IDLE_BLINK
#define IDLE_BLINK 1
#endif

/* Called by the scheduler's wait (tx_thread_schedule.S), which ThreadX
 * declares in no header.
 */
void tx_low_power_enter(void);
void tx_low_power_exit(void);

/* Changed only with interrupts masked.  cpuload is cpu_load_get()'s (the
 * IMU service resets it at "imu start"); ps_window is the "ps" command's,
 * reset by each "ps".
 */
static struct cpu_load_window cpuload;
static struct cpu_load_window ps_window;

#if IDLE_BLINK
/* The time of the LED's next change (idle_blink.h). */
static uint64_t blink_next_us;
#endif

/* See app.h. */
void idle_led_init(void)
{
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

void tx_low_power_enter(void)
{
	uint64_t now = rp2040_timer_us();

	cpu_load_enter(&cpuload, now);
	cpu_load_enter(&ps_window, now);
#if IDLE_BLINK
	if (idle_blink_due(&blink_next_us, now)) {
		gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
	}
#endif
}

void tx_low_power_exit(void)
{
	uint64_t now = rp2040_timer_us();

	cpu_load_exit(&cpuload, now);
	cpu_load_exit(&ps_window, now);
}

static int sample(struct cpu_load_window *s, bool reset, uint64_t *total_us)
{
	uint64_t now = rp2040_timer_us();
	uint64_t total;
	uint64_t idle;
	uint32_t flags = save_and_disable_interrupts();

	/* On one CPU the window is never open here (the caller is a thread,
	 * not the scheduler's wait); the second reading is kept for the same
	 * steps as Zephyr's.
	 */
	cpu_load_sample(s, now, s->open ? rp2040_timer_us() : now, reset, &total, &idle);
	restore_interrupts(flags);

	if (total_us != NULL) {
		*total_us = total;
	}
	/* The division (64-bit) runs with interrupts enabled, as in Zephyr. */
	return cpu_load_ratio(total, idle);
}

/* See cpu_load.h.  Zephyr's cpu_load_get(): reads the time before taking
 * the lock, then the window state under it.
 */
int cpu_load_get(bool reset)
{
	return sample(&cpuload, reset, NULL);
}

/* See app.h. */
int idle_ps_window(uint64_t *total_us)
{
	return sample(&ps_window, true, total_us);
}
