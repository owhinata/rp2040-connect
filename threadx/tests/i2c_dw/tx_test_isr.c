/* SPDX-License-Identifier: MIT */
/* The interrupt handlers of the driver tests on ThreadX's Linux port
 * (tx_test_isr.h).  One Linux thread runs them.  It sleeps until a request
 * or the next timed handler, then makes an interrupt of the port: between
 * _tx_thread_context_save() and _tx_thread_context_restore() no ThreadX
 * thread runs, so the requests (a list that the threads extend with
 * interrupts masked) and the timed handlers' state are its own there.  A
 * request is a ThreadX thread's: it waits on a POSIX semaphore of its own
 * that the handlers' thread posts after the restore; the port stops a
 * thread that waits there as anywhere else, so a more urgent thread that a
 * handler readied runs first, and the requester goes on only when ThreadX
 * runs it again.
 */

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "tx_api.h"
#include "tx_thread.h"

#include "tx_test_isr.h"

struct tx_test_isr_request {
	void (*fn)(void *arg);
	void *arg;
	sem_t done;
	struct tx_test_isr_request *next;
};

/* The requests not run yet (written with interrupts masked). */
static struct tx_test_isr_request *requests;
static struct tx_test_isr_request **requests_tail = &requests;

/* The timed handlers (written with interrupts masked or in a handler):
 * the next run of the one after a time, and of the periodic one.
 */
static void (*after_fn)(void *arg);
static void *after_arg;
static uint64_t after_at;
static void (*every_fn)(void *arg);
static void *every_arg;
static uint64_t every_us, every_at;

static sem_t kick;
static pthread_t isr_thread;
static __thread bool in_isr;

static uint64_t now_us(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)t.tv_sec * 1000000u + (uint64_t)t.tv_nsec / 1000u;
}

/* Sleeps until a kick or the time AT (0: none). */
static void isr_sleep(uint64_t at)
{
	struct timespec t;

	if (at == 0) {
		while (sem_wait(&kick) != 0 && errno == EINTR) {
		}
		return;
	}
	t.tv_sec = (time_t)(at / 1000000u);
	t.tv_nsec = (long)(at % 1000000u) * 1000;
	while (sem_clockwait(&kick, CLOCK_MONOTONIC, &t) != 0 && errno == EINTR) {
	}
}

static void *isr_main(void *p)
{
	uint64_t next = every_fn != NULL ? every_at : 0;

	(void)p;
	for (;;) {
		struct tx_test_isr_request *done;
		uint64_t now;

		isr_sleep(next);

		_tx_thread_context_save();
		in_isr = true;
		done = requests;
		requests = NULL;
		requests_tail = &requests;
		for (struct tx_test_isr_request *r = done; r != NULL; r = r->next) {
			r->fn(r->arg);
		}
		now = now_us();
		if (after_fn != NULL && now >= after_at) {
			void (*fn)(void *arg) = after_fn;

			after_fn = NULL;
			fn(after_arg);
		}
		if (every_fn != NULL && now >= every_at) {
			every_at = now + every_us;
			every_fn(every_arg);
		}
		next = every_fn != NULL ? every_at : 0;
		if (after_fn != NULL && (next == 0 || after_at < next)) {
			next = after_at;
		}
		in_isr = false;
		_tx_thread_context_restore();

		while (done != NULL) {
			struct tx_test_isr_request *r = done;

			/* The requester may return and drop R once posted. */
			done = r->next;
			sem_post(&r->done);
		}
	}
	return NULL;
}

void tx_test_isr_start(void)
{
	sem_init(&kick, 0, 0);
	every_at = now_us() + every_us;
	assert(pthread_create(&isr_thread, NULL, isr_main, NULL) == 0);
}

bool tx_test_in_isr(void)
{
	return in_isr;
}

void tx_test_isr(void (*fn)(void *arg), void *arg)
{
	struct tx_test_isr_request r = {.fn = fn, .arg = arg, .next = NULL};
	UINT posture;

	if (in_isr) {
		fn(arg);
		return;
	}
	assert(tx_thread_identify() != TX_NULL);
	sem_init(&r.done, 0, 0);
	posture = tx_interrupt_control(TX_INT_DISABLE);
	assert(posture == TX_INT_ENABLE);
	*requests_tail = &r;
	requests_tail = &r.next;
	(void)tx_interrupt_control(posture);
	sem_post(&kick);
	while (sem_wait(&r.done) != 0) {
	}
	sem_destroy(&r.done);
}

void tx_test_isr_after(uint32_t us, void (*fn)(void *arg), void *arg)
{
	UINT posture = 0;

	if (!in_isr) {
		posture = tx_interrupt_control(TX_INT_DISABLE);
	}
	after_fn = fn;
	after_arg = arg;
	after_at = now_us() + us;
	if (!in_isr) {
		(void)tx_interrupt_control(posture);
		sem_post(&kick);
	}
}

void tx_test_isr_every(uint32_t us, void (*fn)(void *arg), void *arg)
{
	every_fn = fn;
	every_arg = arg;
	every_us = us;
}
