/* SPDX-License-Identifier: MIT */
/* Host test of the kernel tick's catch-up (freertos/app/usb-console/src/
 * tick_catchup.h, which the board's tick hook, tick.c, runs).
 *
 * - The step itself: the first tick, the margin of a period and a
 *   microsecond, the move back of `due` on an earlier read, the TIMER's wrap.
 * - A model of SysTick and the CPU, in nanoseconds: period ends at a fixed
 *   period (the same crystal as the TIMER) and a random phase, one pending
 *   bit (an end while it is set is lost), the handler entered when the bit is
 *   set and interrupts are not masked, with random latencies, the TIMER read
 *   in the hook at a random time after the entry, and a pend
 *   (ICSR.PENDSTSET) when the step says so, which merges with the bit.
 *   Interrupts are masked from time to time for 0 to 60 ticks (critical
 *   sections, flash erases).  Over some million ticks (the TIMER wraps on the
 *   way) it checks the two properties of the header: the count never passes
 *   the period ends seen at any handler entry (never ahead), and where a
 *   catch-up stops, the count is at most one tick behind (bounded).  The same
 *   model without the pend loses ticks for good (the check has teeth).
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define configTICK_RATE_HZ 1000
#include "tick_catchup.h"

#define PERIOD_NS ((uint64_t)TICK_CATCHUP_PERIOD_US * 1000u)

static int failures;

#define CHECK(cond)                                                                                \
	do {                                                                                       \
		if (!(cond)) {                                                                     \
			printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                     \
			failures++;                                                                \
		}                                                                                  \
	} while (0)

/* The step -------------------------------------------------------------------*/

static void test_step(void)
{
	struct tick_catchup s = {0};
	const uint32_t p = TICK_CATCHUP_PERIOD_US;

	/* The first tick only sets the time. */
	CHECK(!tick_catchup_step(&s, 5000));
	CHECK(s.due == 5000);

	/* On time, or late by up to a period and a microsecond: nothing. */
	CHECK(!tick_catchup_step(&s, 5000 + p));
	CHECK(!tick_catchup_step(&s, 5000 + 2 * p + 300));
	CHECK(s.due == 5000 + 2 * p);
	CHECK(!tick_catchup_step(&s, 5000 + 3 * p + p));
	CHECK(s.due == 5000 + 3 * p);

	/* More than a period late: one tick more, and so on for each. */
	CHECK(tick_catchup_step(&s, 5000 + 4 * p + p + 1));
	CHECK(!tick_catchup_step(&s, 5000 + 4 * p + p + 2));

	/* An earlier read (less latency) moves the time back. */
	CHECK(!tick_catchup_step(&s, 5000 + 6 * p - 7));
	CHECK(s.due == 5000 + 6 * p - 7);

	/* Many lost ticks: true once per tick until caught up. */
	{
		uint32_t now = s.due + 45 * p + 500;
		int added = 0;

		while (tick_catchup_step(&s, now)) {
			added++;
		}
		CHECK(added == 44);
		CHECK(now - s.due == 500);
	}

	/* Across the TIMER's wrap. */
	s = (struct tick_catchup){0};
	CHECK(!tick_catchup_step(&s, UINT32_MAX - 1500));
	CHECK(!tick_catchup_step(&s, UINT32_MAX - 500));
	CHECK(!tick_catchup_step(&s, UINT32_MAX - 500 + p));
	CHECK(tick_catchup_step(&s, UINT32_MAX - 500 + 3 * p + 1));
	CHECK(!tick_catchup_step(&s, UINT32_MAX - 500 + 3 * p + 1));
	CHECK(s.due == UINT32_MAX - 500 + 3 * p);
}

/* The model ------------------------------------------------------------------*/

static uint64_t rng = 88172645463325252ull;

static uint64_t rnd(void)
{
	rng ^= rng << 13;
	rng ^= rng >> 7;
	rng ^= rng << 17;
	return rng;
}

/* 0 .. n - 1 */
static uint64_t rnd_below(uint64_t n)
{
	return n == 0 ? 0 : rnd() % n;
}

struct model {
	bool pend;        /* pend when the step says so (the hook), or not */
	bool edges;       /* tiny latencies, masks end just before a period end */
	uint64_t phase;   /* the first period end, ns */
	uint32_t timer0;  /* the TIMER at time 0, us */
	uint64_t next;    /* index of the next period end */
	bool pending;     /* SysTick's pending bit */
	uint64_t mask_on; /* masked in [mask_on, mask_off) */
	uint64_t mask_off;
	uint32_t count; /* the kernel's tick count */
	struct tick_catchup s;
	/* Results. */
	uint32_t ahead;        /* entries where the count passed the ends seen */
	uint32_t max_behind;   /* where a catch-up stopped */
	uint32_t behind_one;   /* stops one tick behind */
	uint32_t final_behind; /* at the end, the ticks lost and not added */
	uint32_t masks;
	uint32_t added;
};

static uint64_t period_end(const struct model *m, uint64_t n)
{
	return m->phase + (n - 1) * PERIOD_NS;
}

/* The period ends up to time t set the pending bit (the ones while it is set
 * are lost).
 */
static void ends_until(struct model *m, uint64_t t)
{
	while (period_end(m, m->next) <= t) {
		m->pending = true;
		m->next++;
	}
}

static uint64_t unmasked_at(const struct model *m, uint64_t t)
{
	return t >= m->mask_on && t < m->mask_off ? m->mask_off : t;
}

static uint32_t timer_at(const struct model *m, uint64_t t)
{
	return m->timer0 + (uint32_t)(t / 1000u);
}

/* Latencies in ns.  With edges, less than the TIMER's microsecond, which
 * the step's margin is for.
 */
static uint64_t entry_latency(const struct model *m)
{
	uint64_t r = rnd_below(1000);

	if (m->edges) {
		return 20 + rnd_below(200);
	}
	if (r < 10) {
		return 2000 + rnd_below(200000); /* a device interrupt first */
	}
	return 130 + rnd_below(2000);
}

static uint64_t read_delay(const struct model *m)
{
	uint64_t r = rnd_below(100);

	if (m->edges) {
		return 50 + rnd_below(300);
	}
	if (r < 5) {
		return 600 + rnd_below(30000); /* tasks woken before the hook */
	}
	return 600 + rnd_below(3000);
}

static void run_model(struct model *m, uint64_t ticks, uint64_t seed)
{
	uint64_t t = 0;

	rng = seed;
	m->phase = 1 + rnd_below(PERIOD_NS);
	m->next = 1;
	m->pending = false;
	m->mask_on = m->mask_off = 0;
	m->count = 0;
	m->s = (struct tick_catchup){0};

	/* Past the ticks asked for, up to the end of a catch-up. */
	for (uint64_t i = 0; i < ticks || m->pending; i++) {
		uint64_t entry;
		uint64_t read;
		uint32_t seen;
		uint32_t behind;

		/* Wait for the bit, then for the interrupts. */
		if (!m->pending) {
			t = t > period_end(m, m->next) ? t : period_end(m, m->next);
		}
		ends_until(m, t);
		entry = unmasked_at(m, t);
		entry += entry_latency(m);
		entry = unmasked_at(m, entry);
		ends_until(m, entry);

		/* The entry clears the bit; the kernel counts. */
		m->pending = false;
		m->count++;
		seen = (uint32_t)(m->next - 1);
		if ((int32_t)(m->count - seen) > 0) {
			m->ahead++;
		}

		/* The hook. */
		read = entry + read_delay(m);
		ends_until(m, read);
		if (tick_catchup_step(&m->s, timer_at(m, read))) {
			m->added++;
			if (m->pend) {
				m->pending = true;
			}
		} else if (i > 0) {
			behind = (uint32_t)(m->next - 1) - m->count;
			if ((int32_t)behind > 0 && behind > m->max_behind) {
				m->max_behind = behind;
			}
			m->behind_one += behind == 1;
		}
		t = read + (m->edges ? 50 + rnd_below(300) : 1000 + rnd_below(4000));
		ends_until(m, t);

		/* Sometimes the interrupts are masked after the handler, in a task:
		 * up to two ticks (a long critical section) or up to 60 (a flash
		 * erase).  Not before the first tick.
		 */
		if (i > 0 && i < ticks && t >= m->mask_off && rnd_below(50) == 0) {
			uint64_t span = rnd_below(2) ? 2 * PERIOD_NS : 60 * PERIOD_NS;

			m->mask_on = t + rnd_below(3 * PERIOD_NS);
			m->mask_off = m->mask_on + rnd_below(span);
			if (m->edges) {
				/* The handler then enters just before the next end. */
				uint64_t n = m->next + (m->mask_off - t) / PERIOD_NS + 1;

				m->mask_off = period_end(m, n) - 1 - rnd_below(1500);
				if (m->mask_off <= m->mask_on) {
					m->mask_off = m->mask_on;
				}
			}
			m->masks++;
		}
	}
	m->final_behind = (uint32_t)(m->next - 1) - m->count;
}

static void test_model(void)
{
	/* The TIMER wraps after about 10 minutes of the model's time. */
	const uint32_t timer0 = UINT32_MAX - 600u * 1000000u;
	const uint64_t ticks = 2000000; /* about 33 minutes and more */

	for (uint64_t seed = 1; seed <= 6; seed++) {
		struct model m = {.pend = true, .edges = seed > 4, .timer0 = timer0};

		run_model(&m, ticks, seed * 0x9e3779b97f4a7c15ull);
		printf("  seed %llu%s: %u masks, %u pends, behind at most %u where a catch-up "
		       "stopped (%u times by one), %u at the end, %u entries ahead\n",
		       (unsigned long long)seed, m.edges ? " (edges)" : "", m.masks, m.added,
		       m.max_behind, m.behind_one, m.final_behind, m.ahead);
		CHECK(m.ahead == 0);
		CHECK(m.max_behind <= 1);
		CHECK(m.final_behind <= 1);
		CHECK(m.masks > 1000);
		CHECK(m.added > 10000);
	}

	/* Without the pend the ticks lost stay lost. */
	{
		struct model m = {.pend = false, .timer0 = timer0};

		run_model(&m, ticks / 10, 12345);
		printf("  without the pend: %u masks, %u ticks behind at the end\n", m.masks,
		       m.final_behind);
		CHECK(m.ahead == 0);
		CHECK(m.final_behind > 1000);
	}
}

int main(void)
{
	test_step();
	test_model();
	if (failures != 0) {
		printf("tick: %d checks failed\n", failures);
		return 1;
	}
	printf("tick: ok\n");
	return 0;
}
