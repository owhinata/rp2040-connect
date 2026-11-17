/* SPDX-License-Identifier: MIT */
/* The self-test of the FreeRTOS part of the Zephyr compatibility layer: see
 * zephyr_selftest.h.  The checks use the layer's Zephyr API; the helper tasks,
 * the tick count and the priorities they read are FreeRTOS's.
 *
 * Times: "ticks" are FreeRTOS tick counts (xTaskGetTickCount()) from just
 * after a tick (st_tick_sync()), so a relative wait of n ticks, which Zephyr
 * ends at the current tick + n + 1, reads n + 1.  "us" are microseconds of the
 * TIMER (k_cycle_get_64()), checked on the board only: such a wait lasts more
 * than n and at most n + 1 ticks, plus the time to switch (SLACK_US).
 */

#include <errno.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include <zephyr/kernel.h>

#include "zephyr_selftest.h"

#define TICK_US     Z_USEC_PER_TICK
/* Ticks of 1 ms, rounded up (K_MSEC(1)). */
#define MS_TICKS    ((1000 + TICK_US - 1) / TICK_US)
/* Allowed lateness of a wake-up on the board (interrupts, the switch). */
#define SLACK_US    80
/* Zephyr's CONFIG_TIMESLICE_SIZE=20 (freertos/app/usb-console/src/tick.c). */
#define SLICE_TICKS (20000 / TICK_US)

#define HELPER_STACK 1024
#define JOIN_TIMEOUT K_MSEC(2000)

/* Zephyr priorities of the helpers; the caller runs at 14 (NANO_PRIO_SHELL),
 * below all of them.
 */
#define PRIO_LOW  12
#define PRIO_MID  11
#define PRIO_HIGH 10
#define PRIO_SPIN 13

/* Output ----------------------------------------------------------------------*/

/* A test's lines are kept here and written to the console after it, then the
 * console is let go quiet (zephyr_selftest_port_quiet()) before the next test:
 * on the board, writing to the USB console makes USB interrupts and wakes the
 * timer task (the Pico SDK's stdio mutex, released in an interrupt) for a
 * while after the write returns, which would land in the measurements.
 */
static char out[2048];
static size_t out_len;
static bool out_cut;

static void __attribute__((format(printf, 1, 2))) st_printf(const char *fmt, ...)
{
	va_list ap;
	int n;

	if (out_len >= sizeof(out) - 1) {
		out_cut = true;
		return;
	}
	va_start(ap, fmt);
	n = vsnprintf(out + out_len, sizeof(out) - out_len, fmt, ap);
	va_end(ap);
	if (n < 0) {
		return;
	}
	if ((size_t)n >= sizeof(out) - out_len) {
		out_len = sizeof(out) - 1;
		out_cut = true;
	} else {
		out_len += (size_t)n;
	}
}

static void st_flush(void)
{
	fputs(out, stdout);
	if (out_cut) {
		fputs("  (output cut)\n", stdout);
	}
	out_len = 0;
	out[0] = '\0';
	out_cut = false;
	zephyr_selftest_port_quiet();
}

/* Checks --------------------------------------------------------------------*/

static int test_failures;

static void fail_line(int line, const char *what)
{
	st_printf("  FAIL line %d: %s\n", line, what);
	test_failures++;
}

static void check(bool ok, int line, const char *what)
{
	if (!ok) {
		fail_line(line, what);
	}
}

static void check_eq(long long got, long long want, int line, const char *what)
{
	if (got != want) {
		st_printf("  FAIL line %d: %s: %lld, expected %lld\n", line, what, got, want);
		test_failures++;
	}
}

static void check_range(long long got, long long lo, long long hi, int line, const char *what)
{
	if (got < lo || got > hi) {
		st_printf("  FAIL line %d: %s: %lld, expected %lld..%lld\n", line, what, got, lo,
			  hi);
		test_failures++;
	}
}

#define CHECK(cond)              check((cond), __LINE__, #cond)
#define CHECK_EQ(got, want)      check_eq((long long)(got), (long long)(want), __LINE__, #got)
#define CHECK_RANGE(got, lo, hi) check_range((long long)(got), (lo), (hi), __LINE__, #got)

/* Time ----------------------------------------------------------------------*/

static uint32_t st_tick(void)
{
	return (uint32_t)xTaskGetTickCount();
}

/* Returns just after a tick. */
static void st_tick_sync(void)
{
	vTaskDelay(1);
}

static uint64_t st_us(void)
{
	return k_cycle_get_64();
}

/* A wait of n ticks after st_tick_sync(): n + 1 ticks, and in real time more
 * than n ticks and at most n + 1 ticks and SLACK_US.
 */
static void check_wait(const char *what, uint32_t ticks, uint64_t us, uint32_t n, int line)
{
	st_printf("  %s: %lu ticks, %llu us\n", what, (unsigned long)ticks, (unsigned long long)us);
	check_eq(ticks, (long long)n + 1, line, what);
	if (zephyr_selftest_port_realtime()) {
		check_range((long long)us, (long long)n * TICK_US + 1,
			    ((long long)n + 1) * TICK_US + SLACK_US, line, what);
	}
}

/* Helper tasks --------------------------------------------------------------*/

struct helper {
	void (*fn)(void *arg);
	void *arg;
	struct k_sem done;
	TaskHandle_t task;
};

static void helper_main(void *param)
{
	struct helper *h = param;

	h->fn(h->arg);
	k_sem_give(&h->done);
	vTaskDelete(NULL);
}

/* Starts fn(arg) at the Zephyr priority zprio; above the caller it runs at
 * once, until it blocks.
 */
static bool spawn(struct helper *h, const char *name, int zprio, void (*fn)(void *), void *arg)
{
	h->fn = fn;
	h->arg = arg;
	(void)k_sem_init(&h->done, 0, 1);
	return xTaskCreate(helper_main, name, HELPER_STACK / sizeof(StackType_t), h,
			   K_PRIO_PREEMPT(zprio), &h->task) == pdPASS;
}

static bool join(struct helper *h)
{
	return k_sem_take(&h->done, JOIN_TIMEOUT) == 0;
}

/* Mutexes -------------------------------------------------------------------*/

static struct k_mutex zero_mutex;
K_MUTEX_DEFINE(st_defined_mutex);
static K_MUTEX_DEFINE(st_mutex);

static void test_mutex_zero(void)
{
	/* Never initialized: unlocked (Zephyr's lock_count 0). */
	memset(&zero_mutex, 0, sizeof(zero_mutex));
	CHECK_EQ(k_mutex_lock(&zero_mutex, K_NO_WAIT), 0);
	CHECK_EQ(k_mutex_unlock(&zero_mutex), 0);
	CHECK_EQ(k_mutex_unlock(&zero_mutex), -EINVAL);

	memset(&zero_mutex, 0, sizeof(zero_mutex));
	CHECK_EQ(k_mutex_unlock(&zero_mutex), -EINVAL);
	CHECK_EQ(k_mutex_lock(&zero_mutex, K_FOREVER), 0);
	CHECK_EQ(k_mutex_unlock(&zero_mutex), 0);

	CHECK_EQ(k_mutex_lock(&st_defined_mutex, K_FOREVER), 0);
	CHECK_EQ(k_mutex_unlock(&st_defined_mutex), 0);
	CHECK_EQ(k_mutex_init(&zero_mutex), 0);
	CHECK_EQ(k_mutex_lock(&zero_mutex, K_NO_WAIT), 0);
	CHECK_EQ(k_mutex_unlock(&zero_mutex), 0);
}

struct other_lock {
	int nowait;
	int unlock;
};

static void other_lock_main(void *arg)
{
	struct other_lock *r = arg;

	r->nowait = k_mutex_lock(&st_mutex, K_NO_WAIT);
	r->unlock = k_mutex_unlock(&st_mutex);
}

static struct other_lock other_lock(void)
{
	struct other_lock r = {1, 1};
	struct helper h;

	if (spawn(&h, "st_other", PRIO_LOW, other_lock_main, &r)) {
		(void)join(&h);
	}
	return r;
}

static void test_mutex_recursive(void)
{
	struct other_lock r;

	CHECK_EQ(k_mutex_lock(&st_mutex, K_NO_WAIT), 0);
	CHECK_EQ(k_mutex_lock(&st_mutex, K_FOREVER), 0);
	r = other_lock();
	CHECK_EQ(r.nowait, -EBUSY);
	CHECK_EQ(r.unlock, -EPERM);

	CHECK_EQ(k_mutex_unlock(&st_mutex), 0);
	r = other_lock();
	CHECK_EQ(r.nowait, -EBUSY);

	CHECK_EQ(k_mutex_unlock(&st_mutex), 0);
	r = other_lock();
	CHECK_EQ(r.nowait, 0);
	CHECK_EQ(r.unlock, 0);
	CHECK_EQ(k_mutex_unlock(&st_mutex), -EINVAL);
}

struct timed_lock {
	k_timeout_t timeout;
	bool sync; /* start just after a tick (to measure), or lock at once */
	int ret;
	uint32_t ticks;
	uint64_t us;
};

static void timed_lock_main(void *arg)
{
	struct timed_lock *r = arg;
	uint32_t t0;
	uint64_t u0;

	if (r->sync) {
		st_tick_sync();
	}
	t0 = st_tick();
	u0 = st_us();
	r->ret = k_mutex_lock(&st_mutex, r->timeout);
	r->us = st_us() - u0;
	r->ticks = st_tick() - t0;
	if (r->ret == 0) {
		(void)k_mutex_unlock(&st_mutex);
	}
}

static void test_mutex_timeout(void)
{
	struct timed_lock r = {.timeout = K_MSEC(1), .sync = true, .ret = 1};
	struct helper h;

	CHECK_EQ(k_mutex_lock(&st_mutex, K_FOREVER), 0);
	CHECK(spawn(&h, "st_timed", PRIO_LOW, timed_lock_main, &r));
	CHECK(join(&h));
	CHECK_EQ(r.ret, -EAGAIN);
	check_wait("k_mutex_lock(K_MSEC(1)) held", r.ticks, r.us, MS_TICKS, __LINE__);
	CHECK_EQ(k_mutex_unlock(&st_mutex), 0);
}

/* The holder (PRIO_LOW) keeps the mutex until released. */
struct holder {
	struct k_sem locked;
	struct k_sem release;
	UBaseType_t prio_after;
	int seq_after;
};

static volatile int sequence;

static void holder_main(void *arg)
{
	struct holder *r = arg;

	(void)k_mutex_lock(&st_mutex, K_FOREVER);
	k_sem_give(&r->locked);
	(void)k_sem_take(&r->release, K_FOREVER);
	(void)k_mutex_unlock(&st_mutex);
	r->seq_after = ++sequence;
	r->prio_after = uxTaskPriorityGet(NULL);
}

static void test_mutex_inherit(void)
{
	struct holder hold;
	struct timed_lock wait = {.timeout = K_FOREVER, .ret = 1};
	struct timed_lock timed = {.timeout = K_MSEC(5), .ret = 1};
	struct helper hl;
	struct helper hh;
	int seq_waiter;

	/* A higher waiter raises the holder until it unlocks; the waiter then
	 * runs before the holder goes on (Zephyr switches at the unlock).
	 */
	(void)k_sem_init(&hold.locked, 0, 1);
	(void)k_sem_init(&hold.release, 0, 1);
	sequence = 0;
	CHECK(spawn(&hl, "st_holder", PRIO_LOW, holder_main, &hold));
	CHECK_EQ(k_sem_take(&hold.locked, K_MSEC(100)), 0);
	CHECK_EQ(uxTaskPriorityGet(hl.task), K_PRIO_PREEMPT(PRIO_LOW));
	CHECK(spawn(&hh, "st_waiter", PRIO_HIGH, timed_lock_main, &wait));
	CHECK_EQ(uxTaskPriorityGet(hl.task), K_PRIO_PREEMPT(PRIO_HIGH));
	k_sem_give(&hold.release);
	seq_waiter = ++sequence;
	CHECK(join(&hh));
	CHECK(join(&hl));
	CHECK_EQ(wait.ret, 0);
	CHECK_EQ(hold.prio_after, K_PRIO_PREEMPT(PRIO_LOW));
	/* The holder unlocked, the waiter ran and ended, the holder went on (1),
	 * then this task (2).
	 */
	CHECK_EQ(hold.seq_after, 1);
	CHECK_EQ(seq_waiter, 2);

	/* A waiter that times out lowers the holder again. */
	(void)k_sem_init(&hold.locked, 0, 1);
	(void)k_sem_init(&hold.release, 0, 1);
	CHECK(spawn(&hl, "st_holder", PRIO_LOW, holder_main, &hold));
	CHECK_EQ(k_sem_take(&hold.locked, K_MSEC(100)), 0);
	CHECK(spawn(&hh, "st_waiter", PRIO_HIGH, timed_lock_main, &timed));
	(void)k_msleep(1);
	CHECK_EQ(uxTaskPriorityGet(hl.task), K_PRIO_PREEMPT(PRIO_HIGH));
	CHECK(join(&hh));
	CHECK_EQ(timed.ret, -EAGAIN);
	CHECK_EQ(uxTaskPriorityGet(hl.task), K_PRIO_PREEMPT(PRIO_LOW));
	k_sem_give(&hold.release);
	CHECK(join(&hl));
}

/* Semaphores ----------------------------------------------------------------*/

struct taker {
	struct k_sem *sem;
	k_timeout_t timeout;
	int id;
	int ret;
	int *order;
	int *count;
};

static void taker_main(void *arg)
{
	struct taker *t = arg;

	t->ret = k_sem_take(t->sem, t->timeout);
	if (t->order != NULL) {
		t->order[(*t->count)++] = t->id;
	}
}

static void test_sem_basic(void)
{
	K_SEM_DEFINE(sem, 2, 3);
	struct taker t = {.sem = &sem, .timeout = K_FOREVER, .ret = 1};
	struct helper h;

	CHECK_EQ(k_sem_count_get(&sem), 2);
	k_sem_give(&sem);
	k_sem_give(&sem);
	CHECK_EQ(k_sem_count_get(&sem), 3);
	CHECK_EQ(k_sem_take(&sem, K_NO_WAIT), 0);
	CHECK_EQ(k_sem_take(&sem, K_NO_WAIT), 0);
	CHECK_EQ(k_sem_take(&sem, K_FOREVER), 0);
	CHECK_EQ(k_sem_take(&sem, K_NO_WAIT), -EBUSY);

	CHECK_EQ(k_sem_init(&sem, 0, 0), -EINVAL);
	CHECK_EQ(k_sem_init(&sem, 4, 3), -EINVAL);
	CHECK_EQ(k_sem_init(&sem, 0, K_SEM_MAX_LIMIT), 0);

	/* A give hands the count to the waiter. */
	CHECK(spawn(&h, "st_taker", PRIO_LOW, taker_main, &t));
	k_sem_give(&sem);
	CHECK(join(&h));
	CHECK_EQ(t.ret, 0);
	CHECK_EQ(k_sem_count_get(&sem), 0);
}

static void timed_take(const char *what, struct k_sem *sem, k_timeout_t timeout, uint32_t n,
		       int line)
{
	uint32_t t0;
	uint64_t u0;
	int ret;

	st_tick_sync();
	t0 = st_tick();
	u0 = st_us();
	ret = k_sem_take(sem, timeout);
	u0 = st_us() - u0;
	check_eq(ret, -EAGAIN, line, what);
	check_wait(what, st_tick() - t0, u0, n, line);
}

static void test_sem_timeout(void)
{
	K_SEM_DEFINE(sem, 0, 1);

	timed_take("k_sem_take(K_TICKS(1))", &sem, K_TICKS(1), 1, __LINE__);
	timed_take("k_sem_take(K_TICKS(3))", &sem, K_TICKS(3), 3, __LINE__);
	timed_take("k_sem_take(K_MSEC(1))", &sem, K_MSEC(1), MS_TICKS, __LINE__);
}

static void test_sem_order(void)
{
	K_SEM_DEFINE(sem, 0, 10);
	int order[2] = {0, 0};
	int count = 0;
	struct taker lo = {&sem, K_MSEC(500), 1, 1, order, &count};
	struct taker hi = {&sem, K_MSEC(500), 2, 1, order, &count};
	struct helper hl;
	struct helper hh;

	/* The higher priority first, although it came second. */
	CHECK(spawn(&hl, "st_lo", PRIO_LOW, taker_main, &lo));
	CHECK(spawn(&hh, "st_hi", PRIO_MID, taker_main, &hi));
	k_sem_give(&sem);
	CHECK_EQ(count, 1);
	CHECK_EQ(order[0], 2);
	k_sem_give(&sem);
	CHECK(join(&hh));
	CHECK(join(&hl));
	CHECK_EQ(count, 2);
	CHECK_EQ(order[1], 1);
	CHECK_EQ(lo.ret, 0);
	CHECK_EQ(hi.ret, 0);

	/* The same priority: first come first served. */
	count = 0;
	lo.ret = hi.ret = 1;
	CHECK(spawn(&hl, "st_first", PRIO_LOW, taker_main, &lo));
	CHECK(spawn(&hh, "st_second", PRIO_LOW, taker_main, &hi));
	k_sem_give(&sem);
	CHECK_EQ(count, 1);
	CHECK_EQ(order[0], 1);
	k_sem_give(&sem);
	CHECK(join(&hl));
	CHECK(join(&hh));
	CHECK_EQ(order[1], 2);
}

static void test_sem_reset(void)
{
	K_SEM_DEFINE(sem, 0, 5);
	struct taker t = {.sem = &sem, .timeout = K_FOREVER, .ret = 1};
	struct helper h;

	/* Zephyr: the waiters return -EAGAIN. */
	CHECK(spawn(&h, "st_taker", PRIO_LOW, taker_main, &t));
	k_sem_reset(&sem);
	CHECK(join(&h));
	CHECK_EQ(t.ret, -EAGAIN);

	k_sem_give(&sem);
	k_sem_give(&sem);
	CHECK_EQ(k_sem_count_get(&sem), 2);
	k_sem_reset(&sem);
	CHECK_EQ(k_sem_count_get(&sem), 0);
}

/* Sleeps and the clock ------------------------------------------------------*/

enum sleep_kind {
	SLEEP_TIMEOUT,
	SLEEP_MS,
	SLEEP_US,
};

static void timed_sleep(const char *what, enum sleep_kind kind, int64_t value, uint32_t n, int line)
{
	uint32_t t0;
	uint64_t u0;
	uint32_t ticks;

	st_tick_sync();
	t0 = st_tick();
	u0 = st_us();
	switch (kind) {
	case SLEEP_TIMEOUT:
		CHECK_EQ(k_sleep(K_TICKS(value)), 0);
		break;
	case SLEEP_MS:
		CHECK_EQ(k_msleep((int32_t)value), 0);
		break;
	case SLEEP_US:
		CHECK_EQ(k_usleep((int32_t)value), 0);
		break;
	}
	u0 = st_us() - u0;
	ticks = st_tick() - t0;
	if (n == 0) {
		/* A yield. */
		st_printf("  %s: %lu ticks, %llu us\n", what, (unsigned long)ticks,
			  (unsigned long long)u0);
		check_eq(ticks, 0, line, what);
		if (zephyr_selftest_port_realtime()) {
			check_range((long long)u0, 0, TICK_US - 1, line, what);
		}
		return;
	}
	check_wait(what, ticks, u0, n, line);
}

static void test_sleep(void)
{
	timed_sleep("k_sleep(K_NO_WAIT)", SLEEP_TIMEOUT, 0, 0, __LINE__);
	timed_sleep("k_sleep(K_TICKS(1))", SLEEP_TIMEOUT, 1, 1, __LINE__);
	timed_sleep("k_sleep(K_TICKS(5))", SLEEP_TIMEOUT, 5, 5, __LINE__);
	timed_sleep("k_msleep(0)", SLEEP_MS, 0, 0, __LINE__);
	timed_sleep("k_msleep(-3)", SLEEP_MS, -3, 0, __LINE__);
	timed_sleep("k_msleep(1)", SLEEP_MS, 1, MS_TICKS, __LINE__);
	timed_sleep("k_msleep(3)", SLEEP_MS, 3, 3 * MS_TICKS, __LINE__);
	timed_sleep("k_usleep(0)", SLEEP_US, 0, 0, __LINE__);
	timed_sleep("k_usleep(1)", SLEEP_US, 1, 1, __LINE__);
	timed_sleep("k_usleep(tick + 1)", SLEEP_US, TICK_US + 1, 2, __LINE__);
}

static void test_clock(void)
{
	uint64_t u0;
	uint64_t c;
	int64_t a;
	int64_t b;

	u0 = st_us();
	k_busy_wait(300);
	u0 = st_us() - u0;
	st_printf("  k_busy_wait(300): %llu us\n", (unsigned long long)u0);
	CHECK(u0 >= 300);
	if (zephyr_selftest_port_realtime()) {
		CHECK_RANGE(u0, 300, 300 + SLACK_US);
	}

	a = k_uptime_get();
	c = k_cycle_get_64();
	b = k_uptime_get();
	CHECK((uint64_t)a <= c / 1000 && c / 1000 <= (uint64_t)b);
	CHECK_RANGE((uint32_t)(k_cycle_get_32() - (uint32_t)c), 0, 1000);
	CHECK_EQ(sys_clock_hw_cycles_per_sec(), 1000000);
	CHECK_EQ(k_cyc_to_us_floor64(1234), 1234);
	CHECK_EQ(k_cyc_to_ns_floor64(1234), 1234000);
	CHECK_EQ(K_MSEC(1).ticks, MS_TICKS);
	CHECK_EQ(K_MSEC(-5).ticks, 0);
	CHECK_EQ(K_SECONDS(1).ticks, 1000000 / TICK_US);
}

/* Interrupts ----------------------------------------------------------------*/

/* A timeout of the layer runs in the tick interrupt. */
struct isr_call {
	struct _timeout timeout;
	struct k_sem *sem;
	struct k_work *work;
	int id;
	int *order;
	int *count;
	atomic_t *atomic;
	volatile bool in_isr;
	volatile uint64_t us;
};

static void isr_call_run(struct isr_call *c)
{
	c->in_isr = z_os_in_isr();
	c->us = st_us();
	if (c->order != NULL) {
		c->order[(*c->count)++] = c->id;
	}
	if (c->atomic != NULL) {
		(void)atomic_inc(c->atomic);
	}
	if (c->work != NULL) {
		(void)k_work_submit(c->work);
	}
	if (c->sem != NULL) {
		k_sem_give(c->sem);
	}
}

static void isr_call_timeout(struct _timeout *to)
{
	isr_call_run(CONTAINER_OF(to, struct isr_call, timeout));
}

static void isr_call_alarm(void *arg)
{
	isr_call_run(arg);
}

static void isr_call_after(struct isr_call *c, int64_t ticks)
{
	z_add_timeout(&c->timeout, isr_call_timeout, K_TICKS(ticks));
}

static void test_isr_tick(void)
{
	K_SEM_DEFINE(sem, 0, 1);
	struct isr_call c = {.sem = &sem};
	struct isr_call c3[3];
	int order[3] = {0, 0, 0};
	int count = 0;
	uint32_t t0;
	uint64_t u0;
	int ret;

	/* k_sem_give() from the tick interrupt wakes the waiter. */
	st_tick_sync();
	t0 = st_tick();
	u0 = st_us();
	isr_call_after(&c, 2);
	ret = k_sem_take(&sem, K_MSEC(100));
	u0 = st_us() - u0;
	CHECK_EQ(ret, 0);
	CHECK(c.in_isr);
	check_wait("timeout of K_TICKS(2), give in the tick interrupt", st_tick() - t0, u0, 2,
		   __LINE__);

	/* A stopped timeout does not run. */
	c.in_isr = false;
	isr_call_after(&c, 2);
	z_abort_timeout(&c.timeout);
	CHECK_EQ(k_sem_take(&sem, K_TICKS(5)), -EAGAIN);
	CHECK(!c.in_isr);

	/* The earliest deadline first; the same deadline in the order added. */
	memset(c3, 0, sizeof(c3));
	for (int i = 0; i < 3; i++) {
		c3[i].id = i + 1;
		c3[i].order = order;
		c3[i].count = &count;
	}
	c3[2].sem = &sem;
	isr_call_after(&c3[0], 3);
	isr_call_after(&c3[1], 1);
	isr_call_after(&c3[2], 3);
	CHECK_EQ(k_sem_take(&sem, K_MSEC(100)), 0);
	CHECK_EQ(count, 3);
	CHECK_EQ(order[0], 2);
	CHECK_EQ(order[1], 1);
	CHECK_EQ(order[2], 3);
}

static void test_isr_alarm(void)
{
	K_SEM_DEFINE(sem, 0, 1);
	struct isr_call c = {.sem = &sem};
	uint64_t u0;
	uint64_t woke;
	int ret;

	u0 = st_us();
	ret = zephyr_selftest_port_alarm(1000, isr_call_alarm, &c);
	if (ret == -ENOTSUP) {
		st_printf("  skipped: no hardware interrupt here\n");
		return;
	}
	CHECK_EQ(ret, 0);
	CHECK_EQ(k_sem_take(&sem, K_MSEC(100)), 0);
	woke = st_us();
	CHECK(c.in_isr);
	st_printf("  alarm after %llu us, task woken %llu us after the give\n",
		  (unsigned long long)(c.us - u0), (unsigned long long)(woke - c.us));
	CHECK(c.us - u0 >= 1000);
	CHECK_RANGE(woke - c.us, 0, SLACK_US);
}

/* Polling -------------------------------------------------------------------*/

struct giver {
	struct k_sem *sem;
};

static void giver_main(void *arg)
{
	struct giver *g = arg;

	(void)k_msleep(1);
	k_sem_give(g->sem);
}

static void test_poll(void)
{
	K_SEM_DEFINE(a, 0, 1);
	K_SEM_DEFINE(b, 0, 1);
	struct k_poll_event events[3];
	struct isr_call c = {.sem = &b};
	struct giver g = {.sem = &a};
	struct helper h;
	uint32_t t0;
	uint64_t u0;

	k_poll_event_init(&events[0], K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, &a);
	k_poll_event_init(&events[1], K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, &b);
	k_poll_event_init(&events[2], K_POLL_TYPE_IGNORE, K_POLL_MODE_NOTIFY_ONLY, NULL);

	/* Woken by a give in the tick interrupt; only that event is ready, and
	 * the count is left (NOTIFY_ONLY).
	 */
	isr_call_after(&c, 2);
	CHECK_EQ(k_poll(events, 3, K_FOREVER), 0);
	CHECK(c.in_isr);
	CHECK_EQ(events[0].state, K_POLL_STATE_NOT_READY);
	CHECK_EQ(events[1].state, K_POLL_STATE_SEM_AVAILABLE);
	CHECK_EQ(events[2].state, K_POLL_STATE_NOT_READY);
	CHECK_EQ(k_sem_count_get(&b), 1);

	/* Already available: at once. */
	st_tick_sync();
	t0 = st_tick();
	CHECK_EQ(k_poll(events, 3, K_MSEC(10)), 0);
	CHECK_EQ(st_tick() - t0, 0);
	CHECK_EQ(events[1].state, K_POLL_STATE_SEM_AVAILABLE);
	CHECK_EQ(k_sem_take(&b, K_NO_WAIT), 0);

	/* Nothing: K_NO_WAIT and a timeout return -EAGAIN. */
	CHECK_EQ(k_poll(events, 3, K_NO_WAIT), -EAGAIN);
	st_tick_sync();
	t0 = st_tick();
	u0 = st_us();
	CHECK_EQ(k_poll(events, 3, K_TICKS(3)), -EAGAIN);
	u0 = st_us() - u0;
	check_wait("k_poll(K_TICKS(3))", st_tick() - t0, u0, 3, __LINE__);
	CHECK_EQ(events[0].state, K_POLL_STATE_NOT_READY);
	CHECK_EQ(events[1].state, K_POLL_STATE_NOT_READY);

	/* Woken by a give of another task. */
	CHECK(spawn(&h, "st_giver", PRIO_LOW, giver_main, &g));
	CHECK_EQ(k_poll(events, 2, K_MSEC(100)), 0);
	CHECK_EQ(events[0].state, K_POLL_STATE_SEM_AVAILABLE);
	CHECK_EQ(events[1].state, K_POLL_STATE_NOT_READY);
	CHECK(join(&h));
	CHECK_EQ(k_sem_take(&a, K_NO_WAIT), 0);
}

struct poller {
	struct k_sem *sem;
	int id;
	int ret;
	int *order;
	int *count;
};

static void poller_main(void *arg)
{
	struct poller *p = arg;
	struct k_poll_event event;

	k_poll_event_init(&event, K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, p->sem);
	p->ret = k_poll(&event, 1, K_MSEC(500));
	(void)k_sem_take(p->sem, K_NO_WAIT);
	p->order[(*p->count)++] = p->id;
}

static void test_poll_order(void)
{
	K_SEM_DEFINE(sem, 0, 1);
	int order[2] = {0, 0};
	int count = 0;
	struct poller lo = {&sem, 1, 1, order, &count};
	struct poller hi = {&sem, 2, 1, order, &count};
	struct helper hl;
	struct helper hh;

	/* A give signals one event: the higher poller's. */
	CHECK(spawn(&hl, "st_poll_lo", PRIO_LOW, poller_main, &lo));
	CHECK(spawn(&hh, "st_poll_hi", PRIO_MID, poller_main, &hi));
	k_sem_give(&sem);
	CHECK_EQ(count, 1);
	CHECK_EQ(order[0], 2);
	k_sem_give(&sem);
	CHECK(join(&hh));
	CHECK(join(&hl));
	CHECK_EQ(count, 2);
	CHECK_EQ(order[1], 1);
	CHECK_EQ(lo.ret, 0);
	CHECK_EQ(hi.ret, 0);
}

/* Work items ----------------------------------------------------------------*/

struct st_work {
	struct k_work_delayable dwork;
	struct k_work work;
	struct k_sem done;
	volatile int runs;
	volatile uint32_t tick;
	volatile k_tid_t thread;
};

static void st_work_count(struct st_work *w)
{
	w->runs++;
	w->tick = st_tick();
	w->thread = k_current_get();
	k_sem_give(&w->done);
}

static void st_dwork_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);

	st_work_count(CONTAINER_OF(dwork, struct st_work, dwork));
}

static void st_work_handler(struct k_work *work)
{
	st_work_count(CONTAINER_OF(work, struct st_work, work));
}

/* A queue at the caller's priority (Zephyr 14): an item submitted to it
 * waits until the caller blocks.
 */
static struct k_work_q st_work_q;
static K_THREAD_STACK_DEFINE(st_work_q_stack, 1024);
static bool st_work_q_started;

static void st_work_q_start(void)
{
	static const struct k_work_queue_config cfg = {.name = "st_workq"};

	if (!st_work_q_started) {
		k_work_queue_start(&st_work_q, st_work_q_stack,
				   K_THREAD_STACK_SIZEOF(st_work_q_stack), K_PRIO_PREEMPT(14),
				   &cfg);
		st_work_q_started = true;
	}
}

/* The tick at which a work item's handler ran.  On the board the work queue
 * (priority 21) runs right after the tick interrupt; on the POSIX port the
 * switch to its thread is a Linux thread wake-up, which on a busy host can
 * take longer than the port's 1 ms tick, so one tick more is allowed there.
 */
static void check_handler_tick(uint32_t ticks, long long want, int line)
{
	check_range(ticks, want, zephyr_selftest_port_realtime() ? want : want + 1, line,
		    "ticks until the handler ran");
}

static void test_work(void)
{
	static struct st_work w;
	struct k_work_sync sync;
	uint32_t t0;

	memset(&w, 0, sizeof(w));
	(void)k_sem_init(&w.done, 0, 10);
	k_work_init_delayable(&w.dwork, st_dwork_handler);
	k_work_init(&w.work, st_work_handler);

	/* A delay of 5 ticks: the system work queue runs it 6 ticks later. */
	st_tick_sync();
	t0 = st_tick();
	CHECK_EQ(k_work_schedule_for_queue(&k_sys_work_q, &w.dwork, K_TICKS(5)), 1);
	CHECK_EQ(k_work_schedule_for_queue(&k_sys_work_q, &w.dwork, K_TICKS(1)), 0);
	CHECK_EQ(k_work_busy_get(&w.dwork.work), K_WORK_DELAYED);
	CHECK_EQ(k_sem_take(&w.done, K_MSEC(100)), 0);
	st_printf("  delay K_TICKS(5): ran %lu ticks later\n", (unsigned long)(w.tick - t0));
	check_handler_tick(w.tick - t0, 6, __LINE__);
	CHECK_EQ(w.runs, 1);
	CHECK(w.thread == k_work_queue_thread_get(&k_sys_work_q));

	/* A reschedule replaces the delay. */
	st_tick_sync();
	t0 = st_tick();
	CHECK_EQ(k_work_schedule_for_queue(&k_sys_work_q, &w.dwork, K_TICKS(50)), 1);
	CHECK_EQ(k_work_reschedule_for_queue(&k_sys_work_q, &w.dwork, K_TICKS(2)), 1);
	CHECK_EQ(k_sem_take(&w.done, K_MSEC(100)), 0);
	check_handler_tick(w.tick - t0, 3, __LINE__);
	CHECK_EQ(k_sem_take(&w.done, K_TICKS(60)), -EAGAIN);
	CHECK_EQ(w.runs, 2);

	/* K_NO_WAIT submits at once; the queue (above this task) runs it. */
	CHECK_EQ(k_work_schedule_for_queue(&k_sys_work_q, &w.dwork, K_NO_WAIT), 1);
	CHECK_EQ(w.runs, 3);
	CHECK_EQ(k_sem_take(&w.done, K_NO_WAIT), 0);

	/* A queued item is removed by k_work_cancel_sync() and does not run. */
	st_work_q_start();
	CHECK_EQ(k_work_submit_to_queue(&st_work_q, &w.work), 1);
	CHECK_EQ(k_work_submit_to_queue(&st_work_q, &w.work), 0);
	CHECK_EQ(k_work_busy_get(&w.work), K_WORK_QUEUED);
	CHECK(k_work_cancel_sync(&w.work, &sync));
	CHECK_EQ(k_work_busy_get(&w.work), 0);
	CHECK_EQ(k_sem_take(&w.done, K_MSEC(2)), -EAGAIN);
	CHECK_EQ(w.runs, 3);

	/* Submitted and left: runs on its queue's thread. */
	CHECK_EQ(k_work_submit_to_queue(&st_work_q, &w.work), 1);
	CHECK_EQ(k_sem_take(&w.done, K_MSEC(100)), 0);
	CHECK(w.thread == k_work_queue_thread_get(&st_work_q));
	CHECK(!k_work_cancel_sync(&w.work, &sync));
}

static void test_work_isr(void)
{
	static struct st_work w;
	struct isr_call c;

	/* k_work_submit() from the tick interrupt (as the LSM6DSO's FIFO
	 * interrupt does from the GPIO one).
	 */
	memset(&w, 0, sizeof(w));
	memset(&c, 0, sizeof(c));
	(void)k_sem_init(&w.done, 0, 10);
	k_work_init(&w.work, st_work_handler);
	c.work = &w.work;
	isr_call_after(&c, 1);
	CHECK_EQ(k_sem_take(&w.done, K_MSEC(100)), 0);
	CHECK(c.in_isr);
	CHECK_EQ(w.runs, 1);
	CHECK(w.thread == k_work_queue_thread_get(&k_sys_work_q));

	/* The same from a hardware interrupt. */
	c.in_isr = false;
	if (zephyr_selftest_port_alarm(500, isr_call_alarm, &c) == 0) {
		CHECK_EQ(k_sem_take(&w.done, K_MSEC(100)), 0);
		CHECK(c.in_isr);
		CHECK_EQ(w.runs, 2);
	}
}

/* The spin lock and atomics -------------------------------------------------*/

static void test_spinlock(void)
{
	K_SEM_DEFINE(sem, 0, 1);
	struct taker t = {.sem = &sem, .timeout = K_MSEC(500), .ret = 1};
	struct k_spinlock lock = {0};
	k_spinlock_key_t outer;
	k_spinlock_key_t inner;
	struct helper h;
	int ret_inside;
	int ret_between;

	/* A task made ready under the lock runs at the unlock, not before. */
	CHECK(spawn(&h, "st_taker", PRIO_LOW, taker_main, &t));
	outer = k_spin_lock(&lock);
	inner = k_spin_lock(&lock);
	k_sem_give(&sem);
	ret_inside = t.ret;
	k_spin_unlock(&lock, inner);
	ret_between = t.ret;
	k_spin_unlock(&lock, outer);
	CHECK_EQ(ret_inside, 1);
	CHECK_EQ(ret_between, 1);
	CHECK_EQ(t.ret, 0);
	CHECK(join(&h));
}

static void test_atomic(void)
{
	K_SEM_DEFINE(sem, 0, 1);
	atomic_t value = ATOMIC_INIT(5);
	struct isr_call c = {.sem = &sem, .atomic = &value};

	CHECK_EQ(atomic_get(&value), 5);
	CHECK_EQ(atomic_inc(&value), 5);
	CHECK_EQ(atomic_get(&value), 6);
	CHECK(atomic_cas(&value, 6, 10));
	CHECK(!atomic_cas(&value, 6, 11));
	CHECK_EQ(atomic_add(&value, 3), 10);
	CHECK_EQ(atomic_or(&value, 0x100), 13);
	CHECK_EQ(atomic_set(&value, 7), 0x10d);
	isr_call_after(&c, 1);
	CHECK_EQ(k_sem_take(&sem, K_MSEC(100)), 0);
	CHECK_EQ(atomic_clear(&value), 8);
	CHECK_EQ(atomic_get(&value), 0);
}

/* Threads -------------------------------------------------------------------*/

static volatile bool st_hi_ran;
static volatile bool st_eq_ran;
static k_tid_t volatile st_hi_self;
static k_tid_t volatile st_eq_self;
static K_SEM_DEFINE(st_static_park, 0, 1);

static void st_static_entry(void *p1, void *p2, void *p3)
{
	volatile bool *ran = p1;
	k_tid_t volatile *self = p2;

	(void)p3;
	*self = k_current_get();
	*ran = true;
	/* Never given: the threads stay. */
	(void)k_sem_take(&st_static_park, K_FOREVER);
}

/* Zephyr 13 runs before the caller (14), Zephyr 14 after it. */
K_THREAD_DEFINE(st_static_hi_tid, 1024, st_static_entry, (void *)&st_hi_ran, (void *)&st_hi_self,
		NULL, 13, 0, 0);
K_THREAD_DEFINE(st_static_eq_tid, 1024, st_static_entry, (void *)&st_eq_ran, (void *)&st_eq_self,
		NULL, 14, 0, 0);

static void test_static_threads(void)
{
	static bool started;

	if (started) {
		st_printf("  skipped: the threads start once per start of the board\n");
		return;
	}
	started = true;

	/* Above the caller: it has run (and blocked) when the start returns,
	 * and saw its own variable already set.
	 */
	CHECK_EQ(K_THREAD_DEFINE_START(st_static_hi_tid), 0);
	CHECK(st_hi_ran);
	CHECK(st_static_hi_tid != NULL);
	CHECK(st_hi_self == st_static_hi_tid);
	/* The caller's priority: it waits until the caller blocks. */
	CHECK_EQ(K_THREAD_DEFINE_START(st_static_eq_tid), 0);
	CHECK(!st_eq_ran);
	(void)k_sleep(K_TICKS(1));
	CHECK(st_eq_ran);
	CHECK(st_eq_self == st_static_eq_tid);

	CHECK_EQ(strcmp(pcTaskGetName(st_static_hi_tid), "st_static_hi_tid"), 0);
	CHECK_EQ(uxTaskPriorityGet(st_static_hi_tid), K_PRIO_PREEMPT(13));
	CHECK_EQ(uxTaskPriorityGet(st_static_eq_tid), K_PRIO_PREEMPT(14));
}

static void test_thread_name(void)
{
	k_tid_t thread;

	st_work_q_start();
	thread = k_work_queue_thread_get(&st_work_q);
	CHECK(thread != NULL);
	CHECK_EQ(strcmp(pcTaskGetName(thread), "st_workq"), 0);
	CHECK_EQ(k_thread_name_set(thread, "st_workq_renamed"), 0);
	CHECK_EQ(strcmp(pcTaskGetName(thread), "st_workq_renamed"), 0);
	/* Cut to CONFIG_THREAD_MAX_NAME_LEN - 1 (31) characters. */
	CHECK_EQ(k_thread_name_set(thread, "st_0123456789012345678901234567890123456789"), 0);
	CHECK_EQ(strlen(pcTaskGetName(thread)), configMAX_TASK_NAME_LEN - 1);
	CHECK_EQ(k_thread_name_set(thread, NULL), -EINVAL);
	CHECK_EQ(k_thread_name_set(thread, "st_workq"), 0);
	CHECK(k_current_get() == xTaskGetCurrentTaskHandle());
}

/* Time slices ---------------------------------------------------------------*/

#define SLICE_COUNT    7
#define SLICE_SWITCHES 16

struct slices {
	volatile int owner;
	volatile int switches;
	uint32_t end;
	uint32_t tick[SLICE_SWITCHES];
	uint64_t us[SLICE_SWITCHES];
};

struct spinner {
	struct slices *s;
	int id;
};

static void spinner_main(void *arg)
{
	struct spinner *p = arg;
	struct slices *s = p->s;

	while ((int32_t)(st_tick() - s->end) < 0) {
		if (s->owner != p->id) {
			/* A new turn (just after a switch, so not cut by one).  At
			 * the end the other task may still be inside the loop: a
			 * turn that starts at the end is not counted.
			 */
			uint32_t now = st_tick();
			int i = s->switches;

			if ((int32_t)(now - s->end) >= 0) {
				break;
			}

			if (i < SLICE_SWITCHES) {
				s->tick[i] = now;
				s->us[i] = st_us();
			}
			s->switches = i + 1;
			s->owner = p->id;
		}
	}
}

static void test_timeslice(void)
{
	static struct slices s;
	struct spinner a = {&s, 1};
	struct spinner b = {&s, 2};
	struct helper ha;
	struct helper hb;
	bool ok;
	int n;

	/* These test Zephyr's rules of zephyr_os_sched.h. */
	if (IS_ENABLED(NANO_SCHED_FREERTOS)) {
		st_printf("  skipped: built with FreeRTOS's own choice of the next task\n");
		return;
	}

	/* Two tasks of the same priority (above the caller) that never block
	 * take turns every 20 ms.  Both are started before either runs.
	 */
	memset(&s, 0, sizeof(s));
	vTaskSuspendAll();
	s.end = st_tick() + SLICE_COUNT * SLICE_TICKS;
	ok = spawn(&ha, "st_spin_a", PRIO_SPIN, spinner_main, &a);
	ok = spawn(&hb, "st_spin_b", PRIO_SPIN, spinner_main, &b) && ok;
	(void)xTaskResumeAll();
	CHECK(ok);
	CHECK(join(&ha));
	CHECK(join(&hb));

	n = s.switches < SLICE_SWITCHES ? s.switches : SLICE_SWITCHES;
	st_printf("  %d turns in %d slices; between them:", s.switches, SLICE_COUNT);
	for (int i = 1; i < n; i++) {
		st_printf(" %lu", (unsigned long)(s.tick[i] - s.tick[i - 1]));
	}
	st_printf(" ticks\n");
	CHECK_RANGE(s.switches, SLICE_COUNT - 1, SLICE_COUNT + 1);
	/* The first turn starts at the start; the others last a slice. */
	for (int i = 2; i < n; i++) {
		CHECK_RANGE(s.tick[i] - s.tick[i - 1], SLICE_TICKS - 1, SLICE_TICKS + 1);
		if (zephyr_selftest_port_realtime()) {
			CHECK_RANGE(s.us[i] - s.us[i - 1], SLICE_TICKS * TICK_US - 2 * TICK_US,
				    SLICE_TICKS * TICK_US + 2 * TICK_US);
		}
	}
}

/* Wakes every KICK_TICKS + 1 ticks until the end: a higher task that keeps
 * preempting the spinners for a moment.
 */
#define KICK_TICKS (SLICE_TICKS / 5)

struct kicker {
	struct slices *s;
	volatile int kicks;
};

static void kicker_main(void *arg)
{
	struct kicker *k = arg;

	while ((int32_t)(st_tick() - k->s->end) < 0) {
		(void)k_sleep(K_TICKS(KICK_TICKS));
		k->kicks++;
	}
}

static void test_timeslice_preempt(void)
{
	static struct slices s;
	struct spinner a = {&s, 1};
	struct spinner b = {&s, 2};
	struct kicker k = {&s, 0};
	struct helper ha;
	struct helper hb;
	struct helper hk;
	bool ok;

	/* These test Zephyr's rules of zephyr_os_sched.h. */
	if (IS_ENABLED(NANO_SCHED_FREERTOS)) {
		st_printf("  skipped: built with FreeRTOS's own choice of the next task\n");
		return;
	}

	/* Zephyr (one CPU): a thread preempted by a higher one stays at the head
	 * of its priority's queue and goes on when the higher one blocks, and
	 * its time slice starts again when it is switched back in
	 * (kernel/sched.c next_up(), kswap.h do_swap() z_time_slice_reset()).
	 * So with a higher thread waking more often than a slice, the first
	 * spinner keeps the CPU and the second gets no turn until the end.
	 * FreeRTOS by itself hands the CPU to the next task of the priority at
	 * every switch (listGET_OWNER_OF_NEXT_ENTRY()); the layer keeps the
	 * preempted task's place (zephyr_os_sched.h).
	 */
	memset(&s, 0, sizeof(s));
	vTaskSuspendAll();
	s.end = st_tick() + SLICE_COUNT * SLICE_TICKS;
	ok = spawn(&ha, "st_spin_a", PRIO_SPIN, spinner_main, &a);
	ok = spawn(&hb, "st_spin_b", PRIO_SPIN, spinner_main, &b) && ok;
	ok = spawn(&hk, "st_kicker", PRIO_MID, kicker_main, &k) && ok;
	(void)xTaskResumeAll();
	CHECK(ok);
	CHECK(join(&ha));
	CHECK(join(&hb));
	CHECK(join(&hk));

	st_printf("  %d turns, %d preemptions in %d slices\n", s.switches, k.kicks, SLICE_COUNT);
	CHECK_RANGE(k.kicks, SLICE_COUNT * SLICE_TICKS / (KICK_TICKS + 1) - 1,
		    SLICE_COUNT * SLICE_TICKS / (KICK_TICKS + 1) + 1);
	CHECK_EQ(s.switches, 1);
}

/* The run -------------------------------------------------------------------*/

struct test {
	const char *name;
	void (*fn)(void);
};

static const struct test tests[] = {
	{"mutex_zero", test_mutex_zero},
	{"mutex_recursive", test_mutex_recursive},
	{"mutex_timeout", test_mutex_timeout},
	{"mutex_inherit", test_mutex_inherit},
	{"sem_basic", test_sem_basic},
	{"sem_timeout", test_sem_timeout},
	{"sem_order", test_sem_order},
	{"sem_reset", test_sem_reset},
	{"sleep", test_sleep},
	{"clock", test_clock},
	{"isr_tick", test_isr_tick},
	{"isr_alarm", test_isr_alarm},
	{"poll", test_poll},
	{"poll_order", test_poll_order},
	{"work", test_work},
	{"work_isr", test_work_isr},
	{"spinlock", test_spinlock},
	{"atomic", test_atomic},
	{"static_threads", test_static_threads},
	{"thread_name", test_thread_name},
	{"timeslice", test_timeslice},
	{"timeslice_preempt", test_timeslice_preempt},
};

int zephyr_selftest_run(void)
{
	int failed = 0;
	size_t count = sizeof(tests) / sizeof(tests[0]);

	st_printf("selftest: tick %d us, priority %lu (Zephyr 14 is %d)\n", TICK_US,
		  (unsigned long)uxTaskPriorityGet(NULL), K_PRIO_PREEMPT(14));
	st_flush();
	for (size_t i = 0; i < count; i++) {
		test_failures = 0;
		st_printf("%s\n", tests[i].name);
		tests[i].fn();
		st_printf("  %s\n", test_failures == 0 ? "ok" : "FAILED");
		failed += test_failures != 0;
		st_flush();
	}
	st_printf("selftest: %d of %u tests failed\n", failed, (unsigned int)count);
	st_flush();
	return failed;
}
