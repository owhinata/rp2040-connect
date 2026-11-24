/* SPDX-License-Identifier: MIT */
/* The idle wait and the CPU load measured from idle windows,
 * cpu_load_get() (common/compat/port/cpu_load.h), for the builds on the Pico SDK.
 *
 * The same measurement as the Zephyr build (CONFIG_CPU_LOAD with
 * CONFIG_CPU_LOAD_BACKEND_IDLE_HOOK) and the NuttX build's up_idle()
 * (nuttx/boards/arduino-nano-rp2040-connect/src/rp2040_idle_cpuload.c): disable
 * interrupts (PRIMASK), open the window, wait with WFI, close the window,
 * enable interrupts.  WFI wakes on a pending interrupt even while PRIMASK
 * masks it (ARMv6-M), so an interrupt that arrives between disabling
 * interrupts and WFI is not missed (WFI returns at once), and the handler of
 * the interrupt that woke the CPU runs after the window closes, so it counts
 * as load.  The arithmetic is common/compat/port/cpu_load.h's, the steps of
 * Zephyr's cpu_load_get_cpu().
 *
 * The OS's part, idle_os.h (in the OS's application directory), names the
 * functions defined here that the OS calls, whenever no other thread is
 * ready, in one of two shapes:
 *
 *   IDLE_OS_ENTER, IDLE_OS_EXIT
 *       The two halves, void IDLE_OS_ENTER(void) and void IDLE_OS_EXIT(void),
 *       for an OS whose own idle wait masks interrupts, then calls the
 *       first, waits with WFI, calls the second and enables interrupts
 *       again (ThreadX: its scheduler's wait, tx_low_power_enter() and
 *       tx_low_power_exit()).  The first opens the window, the second
 *       closes it; both are called with interrupts masked, from any mode
 *       and stack, and do nothing that waits.
 *   IDLE_OS_HOOK
 *       The whole pass, void IDLE_OS_HOOK(void): masks interrupts, opens the
 *       window, waits with WFI, closes the window and enables interrupts
 *       again, for an OS that calls it again and again from the lowest
 *       priority, in thread mode with interrupts enabled, and waits with
 *       no WFI of its own (FreeRTOS: the idle task's hook
 *       vApplicationIdleHook(), its RP2040 port without tickless idle).
 *
 * A periodic tick interrupt (SysTick, every 1 ms) wakes the CPU every tick
 * and its handler counts as load: the Zephyr and NuttX builds are tickless
 * and have no such interrupt.  When the interrupt makes a thread ready, the
 * switch it requests (PendSV) is taken right after interrupts are enabled
 * again, and the wait is left.
 *
 * Time base: the TIMER, 1 us (rp2040_timer_us()).
 *
 * Opening the window also blinks the LED (GPIO6, idle_blink.h), with the
 * time read for it: one comparison per wait, counted as idle as the wait
 * is, and a toggle every 500 ms.  A firmware that drives the LED itself
 * builds this file with IDLE_BLINK=0.
 */

#include <stdbool.h>
#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "idle_os.h"
#include "app.h"
#include "cpu_load.h"
#include "idle_blink.h"
#include "rp2040_timer_us.h"

#if !defined(IDLE_OS_HOOK) && !(defined(IDLE_OS_ENTER) && defined(IDLE_OS_EXIT))
#error "idle_os.h names IDLE_OS_HOOK or IDLE_OS_ENTER and IDLE_OS_EXIT"
#endif

#ifndef IDLE_BLINK
#define IDLE_BLINK 1
#endif

/* Changed only with interrupts disabled.  cpuload is cpu_load_get()'s (the
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

/* The halves of the wait, interrupts masked.  Always inlined: the whole
 * pass compiles as one function.
 */
static inline __attribute__((always_inline)) void idle_window_open(void)
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

static inline __attribute__((always_inline)) void idle_window_close(void)
{
	uint64_t now = rp2040_timer_us();

	cpu_load_exit(&cpuload, now);
	cpu_load_exit(&ps_window, now);
}

#if defined(IDLE_OS_ENTER) && defined(IDLE_OS_EXIT)
void IDLE_OS_ENTER(void)
{
	idle_window_open();
}

void IDLE_OS_EXIT(void)
{
	idle_window_close();
}
#endif

#ifdef IDLE_OS_HOOK
void IDLE_OS_HOOK(void)
{
	uint32_t flags = save_and_disable_interrupts();

	idle_window_open();

	/* Zephyr: __DSB() then __WFI() (SLEEP_IF_ALLOWED). */
	__asm volatile("dsb\n\twfi" ::: "memory");

	idle_window_close();

	/* The pending interrupt is taken here, outside the window. */
	restore_interrupts(flags);
}
#endif

static int sample(struct cpu_load_window *s, bool reset, uint64_t *total_us)
{
	uint64_t now = rp2040_timer_us();
	uint64_t total;
	uint64_t idle;
	uint32_t flags = save_and_disable_interrupts();

	/* On one CPU the window is never open here (the caller is not idle);
	 * the second reading is kept for the same steps as Zephyr's.
	 */
	cpu_load_sample(s, now, s->open ? rp2040_timer_us() : now, reset, &total, &idle);
	restore_interrupts(flags);

	if (total_us != NULL) {
		*total_us = total;
	}
	/* The division (64-bit, libgcc on the M0+) runs with interrupts
	 * enabled, as in Zephyr.
	 */
	return cpu_load_ratio(total, idle);
}

/* See cpu_load.h.  Zephyr's cpu_load_get(): reads the time before taking
 * the lock, then the window state under it.
 */
int cpu_load_get(bool reset)
{
	return sample(&cpuload, reset, NULL);
}

int idle_ps_window(uint64_t *total_us)
{
	return sample(&ps_window, true, total_us);
}
