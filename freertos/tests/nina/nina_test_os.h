/* SPDX-License-Identifier: MIT */
/* What the NINA test (main.c, which is the same for every kernel it runs
 * on) needs from the kernel, and what main.c gives the kernel's part.  Each
 * kernel's part is a nina_test_os.c: this directory's for FreeRTOS's POSIX
 * port, threadx/tests/nina/nina_test_os.c for ThreadX's Linux port.  It
 * also has main(), which starts the runner (nina_test_run()) at the
 * shell's priority, and the tests of its own binding's behaviour that the
 * binding's header leaves to the kernel (nina_test_os_tests()).
 *
 * Priorities are levels above the runner's; ticks are the kernel's (their
 * count wraps at 32 bits).
 */

#ifndef FREERTOS_TESTS_NINA_NINA_TEST_OS_H
#define FREERTOS_TESTS_NINA_NINA_TEST_OS_H

#include <stdbool.h>
#include <stdint.h>

/* The kernel's tick count and rate, and a sleep of TICKS. */
uint32_t nina_test_os_ticks(void);
unsigned int nina_test_os_tick_hz(void);
void nina_test_os_delay(uint32_t ticks);

/* A thread that runs FN(ARG) at the runner's priority + ABOVE (from a
 * thread), the end of the calling one (from its function), the deletion of
 * one, and whether one is blocked (asleep or waiting).
 */
typedef void *nina_test_os_thread_t;

nina_test_os_thread_t nina_test_os_thread_start(void (*fn)(void *arg), void *arg, int above);
void nina_test_os_thread_exit(void);
void nina_test_os_thread_delete(nina_test_os_thread_t thread);
bool nina_test_os_thread_blocked(nina_test_os_thread_t thread);

/* The runner, and the calling thread's priority now (with what it
 * inherits), above the runner's own.
 */
nina_test_os_thread_t nina_test_os_runner(void);
int nina_test_os_priority(void);

/* FN(ARG) as an interrupt handler (at once; from a handler, a plain
 * call).
 */
void nina_test_os_isr(void (*fn)(void *arg), void *arg);

/* The DMA engine: nina_test_dma_move() a tick from now (the TX channel's
 * start), as the kernel's interrupts or a thread above the runner; and
 * whether the transfer's caller (the runner) waits for it then (blocked,
 * or in the kernel's own test of the moment before it blocks).
 */
void nina_test_os_dma_kick(void);
bool nina_test_os_dma_caller_waits(void);

/* The next blocking wait of the runner for the kind named gets its event
 * between the binding's last look and the block (the DMA engine's move of
 * the started transfer, a falling edge of READY with the module coming
 * alive, a rising edge of the IRQ line), as an interrupt at that moment
 * would: wrapped calls of the kernel (host_tests.sh).
 */
void nina_test_os_dma_window(void);
void nina_test_os_ready_window(void);
void nina_test_os_event_window(void);

/* A wake-up of the runner that is not the wait's own (FreeRTOS: the
 * NINA's task notification, which its waits share), given before a wait
 * that must not end at it.
 */
void nina_test_os_stray_wake(void);

/* The kernel's own tests (after main.c's of every kernel). */
void nina_test_os_tests(void);

/* main.c: the setup of the test's hardware (before the kernel's start),
 * the tests (the runner's function), the DMA engine's move of the started
 * transfer (as its interrupt), the fake TIMER in ms, a rising edge of the
 * IRQ line and the module's power (READY's level follows), each with its
 * interrupt.
 */
void nina_test_setup(void);
void nina_test_run(void);
void nina_test_dma_move(void);
int64_t nina_test_now_ms(void);
void nina_test_irq_rise(void);
void nina_test_module_alive(bool alive);

#endif /* FREERTOS_TESTS_NINA_NINA_TEST_OS_H */
