/* SPDX-License-Identifier: MIT */
/* Host test of common/drivers/nina/internal/nina_wait.c (the waits of the socket
 * loops) on fakes of nina_os.h: a clock in milliseconds that only the
 * sleeps and the waits move, and an IRQ line whose edges are planned at
 * given times.  nina_os_event_start() fails or succeeds as the test says.
 *
 * Checked:
 * - before the events are started, and after a start that failed (an OS
 *   without a usable interrupt, -ENOTSUP): each wait is a sleep of the
 *   loop's interval, nothing else, and the other event calls are not made;
 * - the start: nina_os_event_start() once, its failure keeps the sleeps;
 * - an edge after the mark ends the wait at the edge, within
 *   NINA_EVENT_TIMEOUT_MS;
 * - an edge between the mark and the wait (an edge from before the pass's
 *   command, or one during the pass) ends the wait at once: one more pass,
 *   nothing lost;
 * - no edge (one lost): NINA_EVENT_TIMEOUT_MS, then the pass asks again;
 * - the deadline of the call shortens the wait; a deadline that has passed
 *   does not wait;
 * - the back-off after waits in a row that an edge ended: 0, 0, 1, 2, 4,
 *   8, 16, 20, 20 ms of sleep before the wait, an edge during it still
 *   seen, the count started again by a wait that ends without an edge, the
 *   back-off never past the deadline.
 */

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>

#include "nina_os.h"
#include "nina_wait.h"

/* Fake clock, sleeps and line ------------------------------------------*/

static int64_t now = 1000;
static unsigned int sleeps, waits;
static uint32_t last_sleep, last_wait_ms;
static unsigned int starts;
static int start_result;
static int started; /* nina_os_event_start() has returned 0 */

#define MAX_EDGES 8
static int64_t edges[MAX_EDGES]; /* planned edge times, ascending; 0 none */
static unsigned int nedges;
static uint32_t count;           /* edges so far */

static void plan_edge(int64_t at)
{
	assert(nedges < MAX_EDGES);
	edges[nedges++] = at;
}

/* Runs the clock to T, counting the planned edges on the way. */
static void run_to(int64_t t)
{
	unsigned int i = 0, j;

	now = t;
	while (i < nedges && edges[i] <= now) {
		count++;
		i++;
	}
	for (j = 0; i + j < nedges; j++) {
		edges[j] = edges[i + j];
	}
	nedges = j;
}

int64_t nina_os_uptime_ms(void)
{
	return now;
}

void nina_os_sleep_ms(uint32_t ms)
{
	sleeps++;
	last_sleep = ms;
	run_to(now + ms);
}

int nina_os_event_start(void)
{
	assert(!started);
	starts++;
	if (!start_result) {
		started = 1;
	}
	return start_result;
}

/* Only after a start that succeeded (nina_os.h). */
uint32_t nina_os_event_seq(void)
{
	assert(started);
	return count;
}

/* Returns at once when the count moved; else at the next edge within MS,
 * else after MS. */
void nina_os_event_wait(uint32_t seq, uint32_t ms)
{
	assert(started);
	waits++;
	last_wait_ms = ms;
	if (count != seq) {
		return;
	}
	if (nedges && edges[0] <= now + ms) {
		run_to(edges[0] > now ? edges[0] : now);
		return;
	}
	run_to(now + ms);
}

/* Unused by nina_wait.c; the boundary's other functions. */
int nina_os_lock(void)
{
	return 0;
}

void nina_os_unlock(void)
{
}

static void reset_counts(void)
{
	sleeps = waits = 0;
	last_sleep = last_wait_ms = 0;
}

/* Tests ------------------------------------------------------------------*/

static void test_without_events(void)
{
	struct nina_wait w = NINA_WAIT_INIT;

	assert(!nina_wait_events());
	reset_counts();
	nina_wait_mark(&w);
	nina_wait(&w, NINA_POLL_INTERVAL_MS, INT64_MAX);
	assert(sleeps == 1 && last_sleep == NINA_POLL_INTERVAL_MS && waits == 0);
	nina_wait(&w, 1, now + 5);
	assert(sleeps == 2 && last_sleep == 1 && w.mark == 0 && w.woken == 0);
	/* An edge changes nothing. */
	plan_edge(now);
	run_to(now);
	nina_wait(&w, NINA_POLL_INTERVAL_MS, INT64_MAX);
	assert(sleeps == 3 && last_sleep == NINA_POLL_INTERVAL_MS && waits == 0);
}

static void test_backoff_values(void)
{
	static const uint32_t expected[] = {0, 0, 1, 2, 4, 8, 16, 20, 20, 20};

	for (unsigned int i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
		assert(nina_wait_backoff_ms(i) == expected[i]);
	}
	assert(nina_wait_backoff_ms(255) == NINA_POLL_INTERVAL_MS);
	assert(NINA_EVENT_TIMEOUT_MS == 100);
}

/* The OS cannot wait for the events: the waits stay sleeps. */
static void test_start_fails(void)
{
	start_result = -ENOTSUP;
	assert(nina_wait_start_events() == -ENOTSUP && starts == 1 && !nina_wait_events());
	test_without_events();
}

static void test_start(void)
{
	starts = 0;
	start_result = -EIO;
	assert(nina_wait_start_events() == -EIO && starts == 1 && !nina_wait_events());
	start_result = 0;
	assert(nina_wait_start_events() == 0 && starts == 2 && nina_wait_events());
	/* Once. */
	assert(nina_wait_start_events() == 0 && starts == 2);
}

static void test_edges(void)
{
	struct nina_wait w = NINA_WAIT_INIT;
	int64_t t;

	/* The mark is the count before the pass. */
	plan_edge(now);
	run_to(now);
	nina_wait_mark(&w);
	assert(w.mark == count);

	/* An edge 7 ms after the pass: the wait ends then, no sleep. */
	reset_counts();
	t = now;
	plan_edge(t + 7);
	nina_wait(&w, NINA_POLL_INTERVAL_MS, INT64_MAX);
	assert(now == t + 7 && waits == 1 && sleeps == 0 && last_wait_ms == NINA_EVENT_TIMEOUT_MS);
	assert(w.woken == 1);

	/* The edge came between the mark and the wait (during the pass, or
	 * left from before its command): at once; the next pass asks, and the
	 * wait after it (a new mark) waits for the next edge. */
	nina_wait_mark(&w);
	plan_edge(now);
	run_to(now);
	reset_counts();
	t = now;
	nina_wait(&w, NINA_POLL_INTERVAL_MS, INT64_MAX);
	assert(now == t && waits == 1 && sleeps == 0 && w.woken == 2);
	nina_wait_mark(&w);
	plan_edge(now + 30);
	/* Two edges in a row that did not help: 1 ms of back-off first. */
	nina_wait(&w, NINA_POLL_INTERVAL_MS, INT64_MAX);
	assert(now == t + 30 && sleeps == 1 && last_sleep == 1 && waits == 2 && w.woken == 3);

	/* No edge (lost): NINA_EVENT_TIMEOUT_MS after the back-off, and the
	 * count starts again. */
	nina_wait_mark(&w);
	reset_counts();
	t = now;
	nina_wait(&w, NINA_POLL_INTERVAL_MS, INT64_MAX);
	assert(sleeps == 1 && last_sleep == 2 && waits == 1);
	assert(now == t + NINA_EVENT_TIMEOUT_MS && w.woken == 0);
	reset_counts();
	t = now;
	nina_wait_mark(&w);
	nina_wait(&w, NINA_POLL_INTERVAL_MS, INT64_MAX);
	assert(sleeps == 0 && now == t + NINA_EVENT_TIMEOUT_MS);
}

static void test_deadline(void)
{
	struct nina_wait w = NINA_WAIT_INIT;
	int64_t t = now;

	/* The call's deadline in 30 ms: the wait is 30 ms. */
	nina_wait_mark(&w);
	reset_counts();
	nina_wait(&w, NINA_POLL_INTERVAL_MS, t + 30);
	assert(waits == 1 && last_wait_ms == 30 && now == t + 30 && sleeps == 0);
	/* Passed: no wait at all (the caller's deadline check ends the call). */
	reset_counts();
	nina_wait(&w, NINA_POLL_INTERVAL_MS, t + 10);
	assert(waits == 1 && last_wait_ms == 0 && now == t + 30);
	/* The back-off is not longer than the time left. */
	w.woken = 9;
	reset_counts();
	t = now;
	nina_wait(&w, NINA_POLL_INTERVAL_MS, t + 5);
	assert(sleeps == 1 && last_sleep == 5 && last_wait_ms == 0 && now == t + 5);
}

static void test_backoff_sequence(void)
{
	static const uint32_t expected[] = {0, 0, 1, 2, 4, 8, 16, 20, 20};
	struct nina_wait w = NINA_WAIT_INIT;

	/* Edges of other sockets, 3 ms after each pass, that never help: the
	 * waits before them grow to the poll interval. */
	for (unsigned int i = 0; i < sizeof(expected) / sizeof(expected[0]); i++) {
		int64_t t = now;

		nina_wait_mark(&w);
		reset_counts();
		plan_edge(t + 3);
		nina_wait(&w, NINA_POLL_INTERVAL_MS, INT64_MAX);
		assert(sleeps == (expected[i] ? 1u : 0u) && waits == 1);
		if (expected[i]) {
			assert(last_sleep == expected[i]);
		}
		/* The edge during the back-off is seen right after it. */
		assert(now == t + (expected[i] > 3 ? expected[i] : 3));
		assert(w.woken == i + 1);
	}
}

int main(void)
{
	test_backoff_values();
	test_without_events();
	test_start_fails();
	test_start();
	test_edges();
	test_deadline();
	test_backoff_sequence();
	puts("NINA socket waits: tests passed (start failed, then events)");
	return 0;
}
