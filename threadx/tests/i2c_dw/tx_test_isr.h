/* SPDX-License-Identifier: MIT */
/* Interrupt handlers of the driver tests on ThreadX's Linux port
 * (threadx/tests/i2c_dw and threadx/tests/nina).  On that port an
 * interrupt is a Linux thread that is not a ThreadX thread and runs between
 * _tx_thread_context_save(), which stops the running ThreadX thread (a
 * signal, wherever it is, unless it has interrupts masked: the port's
 * mutex), and _tx_thread_context_restore(), which resumes it or switches to
 * a more urgent thread that the handler made ready, as the end of a
 * handler on the board.  Here one such thread runs the tests' handlers: on
 * request, after a time, or every period; they do not nest with each
 * other (one priority, as the board's peripheral interrupts), only with
 * the port's tick.
 */

#ifndef THREADX_TESTS_I2C_DW_TX_TEST_ISR_H
#define THREADX_TESTS_I2C_DW_TX_TEST_ISR_H

#include <stdbool.h>
#include <stdint.h>

/* Starts the handlers' thread: once, from tx_application_define() (after
 * the port's start, whose CPU it shares).
 */
void tx_test_isr_start(void);

/* FN(ARG) as an interrupt handler, before the call returns: from a ThreadX
 * thread with interrupts enabled (the thread is stopped meanwhile, and a
 * thread that the handler readied and that is more urgent runs first), or
 * from a handler (a plain call).
 */
void tx_test_isr(void (*fn)(void *arg), void *arg);

/* FN(ARG) as an interrupt handler US microseconds from now, once (a later
 * call replaces one not run yet); from a thread or a handler.
 */
void tx_test_isr_after(uint32_t us, void (*fn)(void *arg), void *arg);

/* FN(ARG) as an interrupt handler every US microseconds; before
 * tx_test_isr_start().
 */
void tx_test_isr_every(uint32_t us, void (*fn)(void *arg), void *arg);

/* True in a handler of this file. */
bool tx_test_in_isr(void);

#endif /* THREADX_TESTS_I2C_DW_TX_TEST_ISR_H */
