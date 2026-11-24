/* SPDX-License-Identifier: MIT */
/* The ThreadX part of the I2C test (freertos/tests/i2c_dw/i2c_dw_test_os.h,
 * whose main.c is the test) on ThreadX's Linux port, for the binding
 * common/drivers/i2c_dw/port/threadx/i2c_dw_os_impl.h.  The runner and the
 * other threads are threads of the compatibility layer (each with its wake
 * flag, z_os_thread_start()); the interrupt handlers, also the test's
 * hardware (i2c_dw_test_hw_step() every 1 ms), are tx_test_isr.c's.  The
 * tick count starts 3 s before it wraps, so that the tests' deadlines cross
 * the wrap.
 *
 * The checks of the binding's own parts (i2c_dw_test_os_boundary()):
 * - the wait does not end at the thread's wake flag of the layer (another
 *   wake-up of the same thread), nor at a put of another semaphore;
 * - a give from an interrupt handler once the waiter is suspended on the
 *   semaphore (ThreadX's count of suspended threads) wakes it within a
 *   tick; a give right before the wait's call into ThreadX is not lost
 *   (i2c_dw_test_os_race(): _txe_semaphore_get() wrapped, -Wl,--wrap);
 * - a deadline across the wrap of the tick count, waited for its ticks plus
 *   one;
 * - a second i2c_dw_os_sem_init() of a made semaphore: its count and limit
 *   anew, still one ThreadX object (the count of created semaphores);
 * - the calls before the kernel runs (main(), between
 *   _tx_initialize_kernel_setup() and tx_kernel_enter(), as the board's
 *   start): two inits, gives, takes without a wait, a take with a time of
 *   an empty semaphore (-EAGAIN at once: ThreadX refuses the wait) and of a
 *   given one (0), the bus lock of a free bus and of a held one (-EAGAIN),
 *   the unlock, the sleep (no wait), the deadline;
 * - in an interrupt handler: a take with a time of an empty semaphore
 *   (-EAGAIN at once) and of a given one (0), the bus lock of a free bus.
 */

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "tx_api.h"
#include "tx_initialize.h"
#include "tx_semaphore.h"
#include "tx_thread.h"

#include "zephyr_kernel_os.h"

#include "i2c_dw_os_impl.h"
#include "i2c_dw_test_os.h"
#include "priorities.h"
#include "tx_test_isr.h"

#define STACK_SIZE 16384
#define THREADS    8

/* The tick count at the start: 3 s before it wraps. */
#define START_TICKS ((ULONG)0 - 3000u)

/* The threads' stacks, each with the layer's storage after it. */
static uint8_t stacks[THREADS][STACK_SIZE + Z_OS_THREAD_RESERVED] __attribute__((aligned(8)));

static z_os_thread_t runner_tid;

uint32_t i2c_dw_test_os_ticks(void)
{
	return tx_time_get();
}

void i2c_dw_test_os_delay(uint32_t ticks)
{
	(void)tx_thread_sleep(ticks);
}

static int thread_start(void (*fn)(void *arg), void *arg, int above, z_os_thread_t *tid)
{
	for (int i = 0; i < THREADS; i++) {
		const struct z_tx_thread *t = (const void *)(stacks[i] + STACK_SIZE);

		if (t->thread.tx_thread_id != TX_THREAD_ID ||
		    t->thread.tx_thread_state == TX_COMPLETED ||
		    t->thread.tx_thread_state == TX_TERMINATED) {
			return z_os_thread_start(fn, "test", stacks[i], STACK_SIZE,
						 APP_PRIO_SHELL + above, arg, tid);
		}
	}
	return -ENOMEM;
}

void i2c_dw_test_os_thread_start(void (*fn)(void *arg), void *arg, int above)
{
	z_os_thread_t tid;

	assert(thread_start(fn, arg, above, &tid) == 0);
}

void i2c_dw_test_os_thread_exit(void)
{
	z_os_thread_exit();
}

void i2c_dw_test_os_isr(void (*fn)(void *arg), void *arg)
{
	tx_test_isr(fn, arg);
}

bool i2c_dw_test_os_in_isr(void)
{
	return tx_test_in_isr();
}

uint32_t i2c_dw_test_os_irq_disable(void)
{
	return tx_interrupt_control(TX_INT_DISABLE);
}

void i2c_dw_test_os_irq_restore(uint32_t state)
{
	(void)tx_interrupt_control(state);
}

/* With interrupts masked: the port then switches nothing, not even at its
 * tick.
 */
static UINT sched_posture;

void i2c_dw_test_os_sched_lock(void)
{
	sched_posture = tx_interrupt_control(TX_INT_DISABLE);
}

void i2c_dw_test_os_sched_unlock(void)
{
	(void)tx_interrupt_control(sched_posture);
}

uint32_t i2c_dw_test_os_sem_count(i2c_dw_os_sem_t *sem)
{
	return sem->sem.tx_semaphore_count;
}

unsigned int i2c_dw_test_os_sem_waiters(i2c_dw_os_sem_t *sem)
{
	return sem->sem.tx_semaphore_suspended_count;
}

/* The binding's waits call tx_semaphore_get(), _txe_semaphore_get() with
 * ThreadX's error checks: the wrapper gives the armed semaphore from an
 * interrupt handler right before a call that may block.
 */
static i2c_dw_os_sem_t *volatile race_sem;

UINT __real__txe_semaphore_get(TX_SEMAPHORE *semaphore_ptr, ULONG wait_option);
UINT __wrap__txe_semaphore_get(TX_SEMAPHORE *semaphore_ptr, ULONG wait_option);

UINT __wrap__txe_semaphore_get(TX_SEMAPHORE *semaphore_ptr, ULONG wait_option)
{
	i2c_dw_os_sem_t *sem = race_sem;

	if (sem != NULL && semaphore_ptr == &sem->sem && wait_option != TX_NO_WAIT) {
		race_sem = NULL;
		i2c_dw_test_give_from_isr(sem);
	}
	return __real__txe_semaphore_get(semaphore_ptr, wait_option);
}

void i2c_dw_test_os_race(i2c_dw_os_sem_t *sem)
{
	race_sem = sem;
}

bool i2c_dw_test_os_race_pending(void)
{
	return race_sem != NULL;
}

/* Gives SEM from an interrupt handler once a thread is suspended on it. */
static void suspended_giver_main(void *arg)
{
	i2c_dw_os_sem_t *sem = arg;

	while (sem->sem.tx_semaphore_suspended_count == 0) {
		(void)tx_thread_sleep(1);
	}
	i2c_dw_test_give_from_isr(sem);
	z_os_thread_exit();
}

/* The results of the calls before the kernel runs (main()). */
static struct {
	ULONG created_before, created_after;
	uint32_t count_after_init, limit_after_init;
	int take_free, take_empty, take_wait, take_until, take_wait_free;
	int lock_free, lock_held;
	uint32_t count_after_unlock;
	ULONG sleep_ticks;
	uint32_t timepoint_dt;
} pre;

static i2c_dw_os_sem_t pre_sem;

static void before_kernel(void)
{
	ULONG t;

	pre.created_before = _tx_semaphore_created_count;
	i2c_dw_os_sem_init(&pre_sem, 0, I2C_DW_OS_SEM_MAX_LIMIT);
	i2c_dw_os_sem_give(&pre_sem);
	i2c_dw_os_sem_give(&pre_sem);
	/* The second start of an instance: the count and limit anew. */
	i2c_dw_os_sem_init(&pre_sem, 1, 1);
	pre.created_after = _tx_semaphore_created_count;
	pre.count_after_init = pre_sem.sem.tx_semaphore_count;
	pre.limit_after_init = pre_sem.limit;
	i2c_dw_os_sem_give(&pre_sem);
	pre.take_free = i2c_dw_os_sem_take_ms(&pre_sem, 0);
	pre.take_empty = i2c_dw_os_sem_take_ms(&pre_sem, 0);
	pre.take_wait = i2c_dw_os_sem_take_ms(&pre_sem, 10);
	pre.take_until = i2c_dw_os_sem_take_until(&pre_sem, i2c_dw_os_timepoint_calc(10));
	i2c_dw_os_sem_give(&pre_sem);
	pre.take_wait_free = i2c_dw_os_sem_take_ms(&pre_sem, 10);
	i2c_dw_os_sem_give(&pre_sem);
	pre.lock_free = i2c_dw_os_bus_lock(&pre_sem);
	pre.lock_held = i2c_dw_os_bus_lock(&pre_sem);
	i2c_dw_os_bus_unlock(&pre_sem);
	pre.count_after_unlock = pre_sem.sem.tx_semaphore_count;
	t = tx_time_get();
	i2c_dw_os_msleep(5);
	pre.sleep_ticks = tx_time_get() - t;
	pre.timepoint_dt = i2c_dw_os_timepoint_calc(10) - tx_time_get();
}

/* In an interrupt handler. */
static int isr_take_wait, isr_take_free, isr_lock_free;

static void isr_takes(void *arg)
{
	i2c_dw_os_sem_t *sem = arg;

	isr_take_wait = i2c_dw_os_sem_take_ms(sem, 10);
	i2c_dw_os_sem_give(sem);
	isr_take_free = i2c_dw_os_sem_take_ms(sem, 10);
	i2c_dw_os_sem_give(sem);
	isr_lock_free = i2c_dw_os_bus_lock(sem);
}

void i2c_dw_test_os_boundary(void)
{
	static i2c_dw_os_sem_t sem, other;
	ULONG start, ticks, created;
	ULONG end;

	/* Before the kernel ran (main()). */
	assert(pre.created_after == pre.created_before + 1);
	assert(pre.count_after_init == 1 && pre.limit_after_init == 1);
	assert(pre.take_free == 0 && pre.take_empty == -EBUSY);
	assert(pre.take_wait == -EAGAIN && pre.take_until == -EAGAIN && pre.take_wait_free == 0);
	assert(pre.lock_free == 0 && pre.lock_held == -EAGAIN);
	assert(pre.count_after_unlock == 1);
	assert(pre.sleep_ticks == 0);
	assert(pre.timepoint_dt == 10);

	i2c_dw_os_sem_init(&sem, 0, I2C_DW_OS_SEM_MAX_LIMIT);
	i2c_dw_os_sem_init(&other, 0, I2C_DW_OS_SEM_MAX_LIMIT);

	/* The layer's wake flag of this thread, and a give of another
	 * semaphore: the wait goes on for its time.
	 */
	z_os_wake(runner_tid);
	i2c_dw_os_sem_give(&other);
	start = tx_time_get();
	assert(i2c_dw_os_sem_take_ms(&sem, 10) == -EAGAIN);
	ticks = tx_time_get() - start;
	assert(ticks >= 11 && ticks <= 13);
	assert(sem.sem.tx_semaphore_suspended_count == 0);
	/* Both are still there for their own waits. */
	assert(Z_TX_WAKE(runner_tid)->tx_semaphore_count == 1);
	z_os_wait_prepare();
	assert(i2c_dw_os_sem_take_ms(&other, 0) == 0);

	/* A give once the waiter is suspended on the semaphore. */
	assert(thread_start(suspended_giver_main, &sem, 1, &(z_os_thread_t){0}) == 0);
	start = tx_time_get();
	assert(i2c_dw_os_sem_take_ms(&sem, 1000) == 0);
	ticks = tx_time_get() - start;
	assert(ticks <= 3);
	assert(sem.sem.tx_semaphore_count == 0 && sem.sem.tx_semaphore_suspended_count == 0);

	/* A deadline across the wrap of the tick count. */
	tx_time_set((ULONG)0 - 4u);
	start = tx_time_get();
	end = i2c_dw_os_timepoint_calc(10);
	assert(end < start && end - start == 10);
	assert(i2c_dw_os_sem_take_until(&sem, end) == -EAGAIN);
	ticks = tx_time_get() - start;
	assert(ticks >= 11 && ticks <= 13 && tx_time_get() < start);

	/* A second init of a made semaphore. */
	i2c_dw_test_give_from_isr(&sem);
	i2c_dw_test_give_from_isr(&sem);
	i2c_dw_test_give_from_isr(&sem);
	created = _tx_semaphore_created_count;
	i2c_dw_os_sem_init(&sem, 1, 2);
	assert(_tx_semaphore_created_count == created);
	assert(sem.sem.tx_semaphore_count == 1 && sem.limit == 2);
	for (int i = 0; i < 3; i++) {
		i2c_dw_os_sem_give(&sem);
	}
	assert(sem.sem.tx_semaphore_count == 2);
	i2c_dw_os_sem_reset(&sem);

	/* In an interrupt handler: no wait. */
	tx_test_isr(isr_takes, &sem);
	assert(isr_take_wait == -EAGAIN && isr_take_free == 0 && isr_lock_free == 0);
	assert(sem.sem.tx_semaphore_count == 0);
	printf("i2c_dw: ThreadX: wake flag, suspended give, wrap, second init, before the kernel, "
	       "interrupt handler\n");
}

static void runner(void *arg)
{
	(void)arg;
	i2c_dw_test_run();
	fflush(stdout);
	_exit(0);
}

static void hw_step(void *arg)
{
	(void)arg;
	i2c_dw_test_hw_step();
}

void tx_application_define(void *first_unused_memory)
{
	(void)first_unused_memory;
	tx_time_set(START_TICKS);
	tx_test_isr_every(1000, hw_step, NULL);
	tx_test_isr_start();
	if (z_os_thread_start(runner, "runner", stacks[THREADS - 1], STACK_SIZE, APP_PRIO_SHELL,
			      NULL, &runner_tid) != 0) {
		_exit(2);
	}
}

int main(void)
{
	/* printf() must not allocate in a thread (the port suspends threads
	 * with a signal wherever they are, the C library's locks too).
	 */
	static char out[1 << 16];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	/* As the board's start: the kernel set up, the drivers' first calls,
	 * then the scheduler.
	 */
	_tx_initialize_kernel_setup();
	before_kernel();
	tx_kernel_enter();
	return 3;
}
