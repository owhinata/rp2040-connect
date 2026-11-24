/* SPDX-License-Identifier: MIT */
/* What the I2C test (main.c, which is the same for every kernel it runs on)
 * needs from the kernel, and what main.c gives the kernel's part.  Each
 * kernel's part is an i2c_dw_test_os.c: this directory's for FreeRTOS's
 * POSIX port, threadx/tests/i2c_dw/i2c_dw_test_os.c for ThreadX's Linux
 * port.  It also has main(), which starts the runner (main.c's tests) at
 * APP_PRIO_SHELL and the test's hardware (i2c_dw_test_hw_step() once per
 * tick), and the checks of its own boundary implementation
 * (i2c_dw_test_os_boundary()).
 *
 * Priorities are given as levels above APP_PRIO_SHELL (the runner's);
 * ticks are the kernel's (32 bits, the board's 1 ms).
 */

#ifndef FREERTOS_TESTS_I2C_DW_I2C_DW_TEST_OS_H
#define FREERTOS_TESTS_I2C_DW_I2C_DW_TEST_OS_H

#include <stdbool.h>
#include <stdint.h>

#include "i2c_dw_os_impl.h"

/* The kernel's tick count, and a sleep of TICKS. */
uint32_t i2c_dw_test_os_ticks(void);
void i2c_dw_test_os_delay(uint32_t ticks);

/* A thread that runs FN(ARG) at APP_PRIO_SHELL + ABOVE, from a thread, and
 * the end of the calling one (from its function).
 */
void i2c_dw_test_os_thread_start(void (*fn)(void *arg), void *arg, int above);
void i2c_dw_test_os_thread_exit(void);

/* FN(ARG) as an interrupt handler (at once; from a handler, a nested call),
 * and whether the caller is one.
 */
void i2c_dw_test_os_isr(void (*fn)(void *arg), void *arg);
bool i2c_dw_test_os_in_isr(void);

/* The interrupt mask behind the fake save_and_disable_interrupts() and
 * restore_interrupts() of main.c: the previous state, and its restore.
 */
uint32_t i2c_dw_test_os_irq_disable(void);
void i2c_dw_test_os_irq_restore(uint32_t state);

/* No other thread runs between the two (the C library's allocator). */
void i2c_dw_test_os_sched_lock(void);
void i2c_dw_test_os_sched_unlock(void);

/* A semaphore's count and its number of waiting threads. */
uint32_t i2c_dw_test_os_sem_count(i2c_dw_os_sem_t *sem);
unsigned int i2c_dw_test_os_sem_waiters(i2c_dw_os_sem_t *sem);

/* The next wait on SEM that blocks gets a give from an interrupt handler
 * between its look at the count and its block; pending until then.
 */
void i2c_dw_test_os_race(i2c_dw_os_sem_t *sem);
bool i2c_dw_test_os_race_pending(void);

/* The checks of the kernel's own implementation of the boundary (after
 * main.c's of every implementation, test_boundary()).
 */
void i2c_dw_test_os_boundary(void);

/* main.c: the tests (the runner's function) and one tick of the test's
 * hardware (the controllers' interrupts, the DMA and the devices).  A give
 * of SEM from an interrupt handler.
 */
void i2c_dw_test_run(void);
void i2c_dw_test_hw_step(void);
void i2c_dw_test_give_from_isr(i2c_dw_os_sem_t *sem);

#endif /* FREERTOS_TESTS_I2C_DW_I2C_DW_TEST_OS_H */
