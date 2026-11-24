/* SPDX-License-Identifier: MIT */
/* The ThreadX part of the NINA test (freertos/tests/nina/nina_test_os.h,
 * whose main.c is the test) on ThreadX's Linux port, for the binding
 * common/drivers/nina/port/threadx (nina_os.c, nina_bus_dma_os_impl.h).
 * The runner and the helpers are ThreadX threads (no compatibility layer),
 * the runner at ThreadX priority 20; the interrupt handlers, also the DMA
 * engine's move a tick after the TX channel's start, are
 * threadx/tests/i2c_dw/tx_test_isr.c's.  The tick count starts 5 s before
 * it wraps.  The windows before a wait's block (nina_test_os_dma_window()
 * and the others) wrap the binding's tx_semaphore_get(),
 * _txe_semaphore_get() with ThreadX's error checks (-Wl,--wrap,
 * host_tests.sh), on the semaphores' names.  A wake-up that is not a wait's
 * own is a post of the Wi-Fi thread's wake-up (a semaphore of its own here,
 * kept until the Wi-Fi tests below).
 *
 * The binding's own behaviour, tested here:
 * - the calls before the kernel runs (main(), between
 *   _tx_initialize_kernel_setup() and tx_kernel_enter(), as the board's
 *   start): the objects made, the locks taken (also twice) and released
 *   without a wait, a Wi-Fi post, the sleeps (no wait), the clock;
 * - the Wi-Fi thread's wake-up (nina_os_wifi_post(), _wait()): a post
 *   made before the first wait is kept (here the posts of the tests
 *   before, at most one), so the first wait returns at once; the second,
 *   without a post, waits its whole time (rounded up to ticks, plus one);
 *   a post wakes a waiting thread at once; of three posts while it waits
 *   but cannot run, the first is that wait's and one more is kept: two
 *   waits end (FreeRTOS's notification ends one);
 * - priority inheritance across two mutexes: the holder drops to the
 *   highest waiter of the mutex it still holds when it releases the other
 *   (ThreadX's), and to its own when it releases both;
 * - a lock's wait that ThreadX aborts (tx_thread_wait_abort()):
 *   nina_os_lock() returns -EAGAIN without the lock, and the holder keeps
 *   the priority it inherited until its unlock;
 * - the socket event waits of three threads at three priorities, ended by
 *   one edge, most urgent first; an edge after a waiter's timeout but
 *   before it leaves the list (its semaphore put, then deleted); each wait's
 *   semaphore deleted at its end (the count of created semaphores).
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
#include "tx_mutex.h"
#include "tx_semaphore.h"
#include "tx_thread.h"

#include "nina_os.h"
#include "nina_test_os.h"
#include "tx_test_isr.h"

#define STACK_SIZE 16384
#define THREADS    8

/* The runner's ThreadX priority (the levels above it are smaller numbers). */
#define RUNNER_PRIORITY 20

/* The tick count at the start: 5 s before it wraps. */
#define START_TICKS ((ULONG)0 - 5000u)

struct thread_slot {
	TX_THREAD thread;
	void (*fn)(void *arg);
	void *arg;
	uint8_t stack[STACK_SIZE] __attribute__((aligned(8)));
};

/* The helpers' slots, the runner's last. */
static struct thread_slot slots[THREADS];

#define RUNNER (&slots[THREADS - 1].thread)

static void thread_entry(ULONG index)
{
	slots[index].fn(slots[index].arg);
}

static bool slot_free(const struct thread_slot *slot)
{
	return slot->thread.tx_thread_id != TX_THREAD_ID ||
	       slot->thread.tx_thread_state == TX_COMPLETED ||
	       slot->thread.tx_thread_state == TX_TERMINATED;
}

static TX_THREAD *thread_create(ULONG index, void (*fn)(void *arg), void *arg, UINT priority)
{
	struct thread_slot *slot = &slots[index];

	if (slot->thread.tx_thread_id == TX_THREAD_ID) {
		assert(tx_thread_delete(&slot->thread) == TX_SUCCESS);
	}
	slot->fn = fn;
	slot->arg = arg;
	assert(tx_thread_create(&slot->thread, (CHAR *)"helper", thread_entry, index, slot->stack,
				sizeof(slot->stack), priority, priority, TX_NO_TIME_SLICE,
				TX_AUTO_START) == TX_SUCCESS);
	return &slot->thread;
}

uint32_t nina_test_os_ticks(void)
{
	return tx_time_get();
}

unsigned int nina_test_os_tick_hz(void)
{
	return TX_TIMER_TICKS_PER_SECOND;
}

void nina_test_os_delay(uint32_t ticks)
{
	(void)tx_thread_sleep(ticks);
}

nina_test_os_thread_t nina_test_os_thread_start(void (*fn)(void *arg), void *arg, int above)
{
	for (ULONG i = 0; i < THREADS - 1; i++) {
		if (slot_free(&slots[i])) {
			return thread_create(i, fn, arg, (UINT)(RUNNER_PRIORITY - above));
		}
	}
	assert(0);
	return NULL;
}

void nina_test_os_thread_exit(void)
{
	for (;;) {
		(void)tx_thread_terminate(tx_thread_identify());
	}
}

void nina_test_os_thread_delete(nina_test_os_thread_t thread)
{
	assert(tx_thread_terminate(thread) == TX_SUCCESS);
	assert(tx_thread_delete(thread) == TX_SUCCESS);
}

bool nina_test_os_thread_blocked(nina_test_os_thread_t thread)
{
	UINT state = ((TX_THREAD *)thread)->tx_thread_state;

	return state != TX_READY && state != TX_COMPLETED && state != TX_TERMINATED;
}

nina_test_os_thread_t nina_test_os_runner(void)
{
	return RUNNER;
}

int nina_test_os_priority(void)
{
	return RUNNER_PRIORITY - (int)tx_thread_identify()->tx_thread_priority;
}

void nina_test_os_isr(void (*fn)(void *arg), void *arg)
{
	tx_test_isr(fn, arg);
}

/* The windows (see above): the kind armed, the semaphore it waits on. */
enum window {
	WINDOW_NONE,
	WINDOW_DMA,
	WINDOW_READY,
	WINDOW_EVENT,
};

static volatile enum window window;
static volatile bool in_window;

/* An edge of the IRQ line when the runner's next socket event wait times
 * out, before it leaves the list.
 */
static volatile bool edge_after_timeout;

void nina_test_os_dma_window(void)
{
	window = WINDOW_DMA;
}

void nina_test_os_ready_window(void)
{
	window = WINDOW_READY;
}

void nina_test_os_event_window(void)
{
	window = WINDOW_EVENT;
}

static void dma_move(void *arg)
{
	(void)arg;
	nina_test_dma_move();
}

static void irq_rise(void *arg)
{
	(void)arg;
	nina_test_irq_rise();
}

static bool named(const TX_SEMAPHORE *sem, const char *name)
{
	return strcmp((const char *)sem->tx_semaphore_name, name) == 0;
}

UINT __real__txe_semaphore_get(TX_SEMAPHORE *semaphore_ptr, ULONG wait_option);
UINT __wrap__txe_semaphore_get(TX_SEMAPHORE *semaphore_ptr, ULONG wait_option);

UINT __wrap__txe_semaphore_get(TX_SEMAPHORE *semaphore_ptr, ULONG wait_option)
{
	bool runner = tx_thread_identify() == RUNNER && !tx_test_in_isr();
	UINT ret;

	if (runner && wait_option != TX_NO_WAIT) {
		if (window == WINDOW_DMA && named(semaphore_ptr, "nina_bus_dma")) {
			window = WINDOW_NONE;
			in_window = true;
			tx_test_isr(dma_move, NULL);
			in_window = false;
		} else if (window == WINDOW_READY && named(semaphore_ptr, "nina_ready")) {
			window = WINDOW_NONE;
			nina_test_module_alive(true);
		} else if (window == WINDOW_EVENT && named(semaphore_ptr, "nina_event")) {
			window = WINDOW_NONE;
			tx_test_isr(irq_rise, NULL);
		}
	}
	ret = __real__txe_semaphore_get(semaphore_ptr, wait_option);
	if (runner && edge_after_timeout && ret == TX_NO_INSTANCE &&
	    named(semaphore_ptr, "nina_event")) {
		edge_after_timeout = false;
		tx_test_isr(irq_rise, NULL);
	}
	return ret;
}

/* In the DMA window the wrapper moves the bytes instead. */
void nina_test_os_dma_kick(void)
{
	if (window != WINDOW_DMA) {
		tx_test_isr_after(1000, dma_move, NULL);
	}
}

bool nina_test_os_dma_caller_waits(void)
{
	return in_window || nina_test_os_thread_blocked(RUNNER);
}

void nina_test_os_stray_wake(void)
{
	nina_os_wifi_post();
}

/* The time rounded up to ticks, plus one tick. */
static ULONG sleep_ticks(uint64_t us)
{
	return (ULONG)((us * TX_TIMER_TICKS_PER_SECOND + 999999u) / 1000000u + 1);
}

static void expect_ticks(ULONG got, ULONG expected)
{
	if (got < expected || got > expected + 5) {
		printf("ticks: expected %lu, got %lu\n", (unsigned long)expected,
		       (unsigned long)got);
		fflush(stdout);
		assert(0);
	}
}

/* The calls before the kernel runs (main()). */
static struct {
	int lock, lock_again, ready;
	ULONG sleep_ticks;
	bool objects;
	int64_t ms;
} pre;

static void before_kernel(void)
{
	ULONG t;

	pre.lock = nina_os_lock();
	pre.lock_again = nina_os_lock();
	nina_os_unlock();
	nina_os_unlock();
	nina_os_dns_lock();
	nina_os_dns_unlock();
	nina_os_wifi_lock();
	nina_os_wifi_unlock();
	nina_os_wifi_post();
	t = tx_time_get();
	nina_os_sleep_us(100);
	nina_os_sleep_ms(5);
	pre.sleep_ticks = tx_time_get() - t;
	pre.ms = nina_os_uptime_ms();
	pre.objects = _tx_mutex_created_count == 3 && _tx_semaphore_created_count == 2;
}

static volatile unsigned int wakes;
static volatile ULONG wake_ticks;

/* The Wi-Fi thread: waits 1000 ms again and again, and notes how many
 * waits ended and the ticks of the last.
 */
static void wifi_wake_helper(void *arg)
{
	(void)arg;
	for (;;) {
		ULONG t = tx_time_get();

		nina_os_wifi_wait(1000);
		wake_ticks = tx_time_get() - t;
		wakes++;
	}
}

static void test_wifi_wake(void)
{
	TX_THREAD *task;
	UINT old;
	unsigned int n;

	/* A post before the first wait is kept (the tests' stray wake-ups and
	 * the one made before the kernel ran, at most one): the first wait
	 * returns at once; the second, without a post, waits.
	 */
	nina_os_wifi_post();
	wakes = 0;
	task = nina_test_os_thread_start(wifi_wake_helper, NULL, 1);
	assert(wakes == 1 && wake_ticks == 0);
	(void)tx_thread_sleep(5);
	assert(wakes == 1 && nina_test_os_thread_blocked(task));
	/* A post wakes it at once. */
	nina_os_wifi_post();
	assert(wakes == 2 && wake_ticks <= 6);
	/* Three posts while it waits but cannot run: the first is its wait's
	 * (ThreadX hands it to the waiting thread at once, as Zephyr's k_sem),
	 * one of the other two is kept (at most one): two waits end at once,
	 * the next takes its whole time.
	 */
	assert(tx_thread_priority_change(task, RUNNER_PRIORITY + 1, &old) == TX_SUCCESS);
	nina_os_wifi_post();
	nina_os_wifi_post();
	nina_os_wifi_post();
	assert(wakes == 2);
	(void)tx_thread_sleep(2);
	assert(wakes == 4);
	assert(tx_thread_priority_change(task, RUNNER_PRIORITY - 1, &old) == TX_SUCCESS);
	n = wakes;
	(void)tx_thread_sleep(sleep_ticks(1000000) + 10);
	assert(wakes == n + 1);
	expect_ticks(wake_ticks, sleep_ticks(1000000));
	nina_test_os_thread_delete(task);
	printf("nina: ThreadX: Wi-Fi wake-up kept before the first wait\n");
}

static volatile int lock_result;
static volatile int helper_state;

static void lock_helper(void *arg)
{
	(void)arg;
	helper_state = 1;
	lock_result = nina_os_lock();
	helper_state = 2;
	if (lock_result == 0) {
		nina_os_unlock();
	}
	helper_state = 3;
	nina_test_os_thread_exit();
}

static void dns_helper(void *arg)
{
	(void)arg;
	nina_os_dns_lock();
	nina_os_dns_unlock();
	nina_test_os_thread_exit();
}

static void test_inheritance(void)
{
	TX_THREAD *high;

	/* Held: nina_lock and the DNS lock; a waiter 1 above on the second,
	 * then one 3 above on the first.
	 */
	assert(nina_os_lock() == 0);
	nina_os_dns_lock();
	helper_state = 0;
	(void)nina_test_os_thread_start(dns_helper, NULL, 1);
	assert(nina_test_os_priority() == 1);
	(void)nina_test_os_thread_start(lock_helper, NULL, 3);
	assert(helper_state == 1 && nina_test_os_priority() == 3);
	/* nina_lock released: the high waiter runs; the holder keeps the DNS
	 * waiter's priority.
	 */
	nina_os_unlock();
	assert(helper_state == 3 && lock_result == 0);
	assert(nina_test_os_priority() == 1);
	nina_os_dns_unlock();
	assert(nina_test_os_priority() == 0);

	/* A wait that ThreadX aborts: -EAGAIN, without the lock.  The holder
	 * keeps the priority it inherited until it releases the mutex
	 * (ThreadX lowers it at the put).
	 */
	assert(nina_os_lock() == 0);
	helper_state = 0;
	high = nina_test_os_thread_start(lock_helper, NULL, 2);
	assert(helper_state == 1 && nina_test_os_priority() == 2);
	assert(tx_thread_wait_abort(high) == TX_SUCCESS);
	(void)tx_thread_sleep(1);
	assert(helper_state == 3 && lock_result == -EAGAIN);
	assert(nina_test_os_priority() == 2);
	nina_os_unlock();
	assert(nina_test_os_priority() == 0);
	/* Free again: the runner takes it at once. */
	assert(nina_os_lock() == 0);
	nina_os_unlock();
	printf("nina: ThreadX: inheritance across two mutexes, an aborted wait\n");
}

static volatile int woke_order[3];
static volatile int woke_count;
static volatile uint32_t event_mark;

static void event_helper(void *arg)
{
	nina_os_event_wait(event_mark, 1000);
	woke_order[woke_count++] = (int)(intptr_t)arg;
	nina_test_os_thread_exit();
}

static void test_event_waiters(void)
{
	ULONG created = _tx_semaphore_created_count;
	uint32_t seq = nina_os_event_seq();
	ULONG t;

	/* Three waiters at three priorities (started from the lowest): one
	 * edge ends all, the most urgent runs first.
	 */
	event_mark = seq;
	woke_count = 0;
	(void)nina_test_os_thread_start(event_helper, (void *)1, 1);
	(void)nina_test_os_thread_start(event_helper, (void *)2, 2);
	(void)nina_test_os_thread_start(event_helper, (void *)3, 3);
	assert(woke_count == 0 && _tx_semaphore_created_count == created + 3);
	nina_test_irq_rise();
	assert(woke_count == 3 && woke_order[0] == 3 && woke_order[1] == 2 && woke_order[2] == 1);
	assert(nina_os_event_seq() == seq + 1);
	assert(_tx_semaphore_created_count == created);

	/* An edge after the wait's time is up, before it leaves the list: the
	 * edge puts its semaphore; the count has moved, the wait ends.
	 */
	seq = nina_os_event_seq();
	edge_after_timeout = true;
	t = tx_time_get();
	nina_os_event_wait(seq, 10);
	expect_ticks(tx_time_get() - t, sleep_ticks(10000));
	assert(!edge_after_timeout && nina_os_event_seq() == seq + 1);
	assert(_tx_semaphore_created_count == created);
	printf("nina: ThreadX: three event waiters, an edge after a timeout\n");
}

void nina_test_os_tests(void)
{
	assert(pre.lock == 0 && pre.lock_again == 0 && pre.objects);
	assert(pre.sleep_ticks == 0 && pre.ms >= 0);
	test_inheritance();
	test_event_waiters();
	test_wifi_wake();
}

static void runner(void *arg)
{
	(void)arg;
	nina_test_run();
	fflush(stdout);
	_exit(0);
}

void tx_application_define(void *first_unused_memory)
{
	(void)first_unused_memory;
	tx_time_set(START_TICKS);
	tx_test_isr_start();
	(void)thread_create(THREADS - 1, runner, NULL, RUNNER_PRIORITY);
}

int main(void)
{
	/* printf() must not allocate in a thread (the port suspends threads
	 * with a signal wherever they are, the C library's locks too).
	 */
	static char out[1 << 16];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	nina_test_setup();
	/* As the board's start: the kernel set up, the drivers' first calls,
	 * then the scheduler.
	 */
	_tx_initialize_kernel_setup();
	before_kernel();
	tx_kernel_enter();
	return 3;
}
