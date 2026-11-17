/* SPDX-License-Identifier: MIT */
/* The idle task's hook and the CPU load measured from idle windows,
 * cpu_load_get() (common/compat/cpu_load.h).
 *
 * The same measurement as the Zephyr build (CONFIG_CPU_LOAD with
 * CONFIG_CPU_LOAD_BACKEND_IDLE_HOOK) and the NuttX build's up_idle()
 * (nuttx/boards/arduino-nano-rp2040-connect/src/nano_idle.c): disable
 * interrupts (PRIMASK), open the window, wait with WFI, close the window,
 * enable interrupts.  WFI wakes on a pending interrupt even while PRIMASK
 * masks it (ARMv6-M), so an interrupt that arrives between disabling
 * interrupts and WFI is not missed (WFI returns at once), and the handler of
 * the interrupt that woke the CPU runs after the window closes, so it counts
 * as load.  The arithmetic is common/compat/nano_cpuload.h, the steps of
 * Zephyr's cpu_load_get_cpu().
 *
 * FreeRTOS's idle task calls vApplicationIdleHook() in its loop
 * (configUSE_IDLE_HOOK); without tickless idle (configUSE_TICKLESS_IDLE 0)
 * the RP2040 port never waits with WFI itself.  The tick interrupt
 * (SysTick, every 100 us, or every 1 ms with the build option
 * NANO_TICK_HZ=1000) wakes the CPU every tick and its handler counts as
 * load: the
 * Zephyr and NuttX builds are tickless and have no such interrupt.  When
 * the interrupt makes a task ready, the PendSV it requests is taken right
 * after interrupts are enabled again, and the idle task is left.
 *
 * Time base: the TIMER, 1 us (nano_mono_us()).
 */

#include <stdbool.h>
#include <stdint.h>
#include "hardware/sync.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "cpu_load.h"
#include "nano_cpuload.h"
#include "os_clock.h"

/* Changed only with interrupts disabled.  cpuload is cpu_load_get()'s (the
 * IMU service resets it at "imu start"); ps_window is the "ps" command's,
 * reset by each "ps".
 */
static struct nano_cpuload_s cpuload;
static struct nano_cpuload_s ps_window;

void vApplicationIdleHook(void)
{
	uint32_t flags = save_and_disable_interrupts();
	uint64_t now = nano_mono_us();

	nano_cpuload_enter(&cpuload, now);
	nano_cpuload_enter(&ps_window, now);

	/* Zephyr: __DSB() then __WFI() (SLEEP_IF_ALLOWED). */
	__asm volatile("dsb\n\twfi" ::: "memory");

	now = nano_mono_us();
	nano_cpuload_exit(&cpuload, now);
	nano_cpuload_exit(&ps_window, now);

	/* The pending interrupt is taken here, outside the window. */
	restore_interrupts(flags);
}

static int sample(struct nano_cpuload_s *s, bool reset, uint64_t *total_us)
{
	uint64_t now = nano_mono_us();
	uint64_t total;
	uint64_t idle;
	uint32_t flags = save_and_disable_interrupts();

	/* On one CPU the window is never open here (the caller is not idle);
	 * the second reading is kept for the same steps as Zephyr's.
	 */
	nano_cpuload_sample(s, now, s->open ? nano_mono_us() : now, reset, &total, &idle);
	restore_interrupts(flags);

	if (total_us != NULL) {
		*total_us = total;
	}
	/* The division (64-bit, libgcc on the M0+) runs with interrupts
	 * enabled, as in Zephyr.
	 */
	return nano_cpuload_ratio(total, idle);
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
