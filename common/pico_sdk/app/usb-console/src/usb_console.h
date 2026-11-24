/* SPDX-License-Identifier: MIT */
/* The start of the USB console on the Pico SDK, shared by the OSes on it
 * (usb_console.c).  The OS's main() (freertos/ and threadx/app/usb-console/
 * src/main.c) calls these in this order, with its own steps between them:
 *
 *   usb_console_early_init();
 *   the kernel's set-up (the OS's: whatever it needs before objects and
 *   threads can be made, e.g. ThreadX's _tx_initialize_kernel_setup())
 *   usb_console_init();
 *   the thread "main" (the OS's: static, APP_PRIO_MAIN, APP_STACK_MAIN
 *   bytes, priorities.h), whose body is usb_console_main_thread()
 *   usb_console_shell_start(prompt);
 *   the scheduler's start (the OS's; no return)
 *
 * From the kernel's set-up to the scheduler's start, what these functions
 * do needs no interrupt and no time to pass: the OS may keep interrupts
 * masked there (ThreadX does).  The prompt is the OS's name, then ":~$ "
 * ("freertos:~$ ").
 */

#ifndef USB_CONSOLE_H
#define USB_CONSOLE_H

/* First in main(), with interrupts enabled and before any object of the
 * kernel: the interrupt stack's paint, the RAM log, the NINA passthrough
 * mode's request and the last run's record of a fatal error.  When the
 * request is there, runs the mode instead of the firmware and does not
 * return (common/drivers/nina/include/nina_passthrough.h).
 */
void usb_console_early_init(void);

/* After the kernel's set-up, before the thread "main" is made: the USB
 * stdio and the console "usb", the compatibility layer (its work queues'
 * threads are made, not run), I2C0 (and I2C1 when built) and the
 * key-value area's bounds.  Panics when the layer does not start; I2C's
 * failures are logged and the start goes on.
 */
void usb_console_init(void);

/* The body of the thread "main", once the scheduler runs: the LSM6DSOX,
 * the NINA's network interface, the Wi-Fi credentials, the "wifi"
 * command's part, Telnet with this prompt, the application's threads and
 * app_main() (common/imu/app_main.h).  Returns when done; the OS then ends
 * the thread.
 */
void usb_console_main_thread(const char *prompt);

/* After the thread "main" is made, last before the scheduler: the thread
 * "console" (the shell on the USB port, with this prompt) and the LED the
 * idle pass blinks.  Panics when the console does not start.
 */
void usb_console_shell_start(const char *prompt);

#endif /* USB_CONSOLE_H */
