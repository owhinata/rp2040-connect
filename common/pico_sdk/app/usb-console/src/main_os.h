/* SPDX-License-Identifier: MIT */
/* What the start of the USB console on the Pico SDK (main.c) needs from the
 * OS.  main.c includes this header where it defines its own objects; it
 * includes the OS's part, "main_os_impl.h" (in the OS's application
 * directory), which defines:
 *
 *   MAIN_OS_PROMPT              the shell's prompt, a string literal
 *                               ("freertos:~$ ": the OS's name, then ":~$ ")
 *   main_os_kernel_init()       called once from main(), after the NINA
 *                               passthrough mode's check and before the
 *                               first object of the kernel or of the
 *                               compatibility layer is made (the USB stdio,
 *                               the consoles, zephyr_compat_initialize()):
 *                               whatever the OS must do before its objects
 *                               and threads can be made (FreeRTOS: nothing).
 *                               The OS may keep interrupts masked from here
 *                               to main_os_kernel_start(): main() in between
 *                               needs no interrupt and no time to pass
 *   main_os_thread_start(fn)    called once from main(), before the
 *                               scheduler starts: makes the start-up thread
 *                               "main", which runs fn(NULL) once the
 *                               scheduler runs, at APP_PRIO_MAIN with
 *                               APP_STACK_MAIN bytes of stack (priorities.h),
 *                               its stack and control block static (the
 *                               stack may be in SCRATCH_X's noload storage,
 *                               ld/section_extra_post_scratch.incl, which
 *                               keeps the last run's bytes: clear what the
 *                               OS keeps there)
 *   main_os_thread_exit()       the end of fn: ends the start-up thread (its
 *                               objects are not used again; no return)
 *   main_os_kernel_start()      the end of main(): starts the scheduler, with
 *                               the checks the OS's tick needs (no return)
 *
 * Static inline (always inline), so that main() compiles as if it called the
 * OS directly.  The OS's part may also define the OS's hooks that belong to
 * the start (FreeRTOS: app_run_time_us(), the run time counter of "ps").
 */

#ifndef MAIN_OS_H
#define MAIN_OS_H

#include "main_os_impl.h"

static inline void main_os_kernel_init(void);
static inline void main_os_thread_start(void (*fn)(void *arg));
static inline void main_os_thread_exit(void);
static inline void main_os_kernel_start(void);

#endif /* MAIN_OS_H */
