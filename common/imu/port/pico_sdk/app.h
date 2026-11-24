/* SPDX-License-Identifier: MIT */
/* What the sources of the USB console on the Pico SDK share
 * (common/imu/port/pico_sdk and the OS's application). */

#ifndef APP_H
#define APP_H

#include <stddef.h>
#include <stdint.h>

/* The CPU load since the previous call (since the start for the first), in
 * tenths of a percent, as cpu_load_get(): for "ps", which does not reset
 * cpu_load_get()'s window.  *total_us is the length of the window (idle.c).
 */
int idle_ps_window(uint64_t *total_us);

/* The LED (GPIO6), off, for the idle pass's blink (idle.c): from
 * usb_console_shell_start(), before the scheduler starts.
 */
void idle_led_init(void);

/* The interrupt stack (the main stack, irq_stack.c): paint it first in
 * usb_console_early_init(), then read its lowest address, size and
 * high-water mark in bytes.
 */
void irq_stack_paint(void);
uintptr_t irq_stack_base(void);
size_t irq_stack_size(void);
size_t irq_stack_used(void);

#endif /* APP_H */
