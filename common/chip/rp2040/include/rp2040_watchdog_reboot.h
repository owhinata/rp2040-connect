/* SPDX-License-Identifier: MIT */
/* The RP2040's restart of the whole chip but its oscillators through the
 * watchdog, at once, as the Pico SDK's watchdog_reboot(0, 0, 0)
 * (hardware_watchdog/watchdog.c with _watchdog_enable()), on the raw
 * registers: for a build where no OS or SDK driver owns the watchdog (the
 * NuttX build, which does not build NuttX's watchdog driver, and the Zephyr
 * build without CONFIG_WATCHDOG).  The FreeRTOS build uses the Pico SDK's
 * hardware_watchdog itself.
 *
 * The watchdog's scratch registers 0..3 survive the restart (the NINA
 * passthrough's request, the fatal error record); scratch 4 is cleared, so
 * that the boot ROM starts the flash's image and not a vector left there.
 */

#ifndef COMMON_CHIP_RP2040_INCLUDE_RP2040_WATCHDOG_REBOOT_H
#define COMMON_CHIP_RP2040_INCLUDE_RP2040_WATCHDOG_REBOOT_H

#include <stdint.h>

/* Masks interrupts (PRIMASK), so that nothing runs between the decision and
 * the restart, then, in the order of the Pico SDK: the watchdog stopped
 * (CTRL.ENABLE cleared), scratch 4 cleared, every block of the power-on
 * state machine but ROSC and XOSC selected for the watchdog's reset (PSM
 * WDSEL), the reset triggered (CTRL.TRIGGER).  The set and clear aliases of
 * the registers (+0x2000, +0x3000) change only the given bits.  Does not
 * return.
 */
static inline __attribute__((noreturn)) void rp2040_watchdog_reboot(void)
{
	volatile uint32_t *const ctrl_set = (volatile uint32_t *)(0x40058000 + 0x2000);
	volatile uint32_t *const ctrl_clr = (volatile uint32_t *)(0x40058000 + 0x3000);
	volatile uint32_t *const scratch4 = (volatile uint32_t *)(0x40058000 + 0x1c);
	volatile uint32_t *const wdsel_set = (volatile uint32_t *)(0x40010000 + 0x2000 + 0x08);

	__asm__ volatile("cpsid i" ::: "memory");
	*ctrl_clr = 1u << 30;                   /* WATCHDOG CTRL.ENABLE */
	*scratch4 = 0;                          /* WATCHDOG SCRATCH4 */
	*wdsel_set = 0x1ffffu & ~(0x1u | 0x2u); /* PSM WDSEL: all but ROSC, XOSC */
	*ctrl_set = 1u << 31;                   /* WATCHDOG CTRL.TRIGGER */

	for (;;) {
	}
}

#endif /* COMMON_CHIP_RP2040_INCLUDE_RP2040_WATCHDOG_REBOOT_H */
