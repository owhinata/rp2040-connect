/* SPDX-License-Identifier: MIT */
/* The CPU column of the task lists ("ps" of the FreeRTOS build, "kernel ps"
 * of the Zephyr build): each task's share of the time since the previous
 * list (since the start for the first).  The OS gives counters that only
 * grow, one for all the time (idle included) and one per task; this keeps
 * the readings of the previous list and returns the differences.  Nothing
 * runs between two lists.  Free of OS calls, so the host tests
 * (common/tests/ps_interval.c) can check it.
 *
 * A list: ps_interval_begin() with the counter of all the time, then
 * ps_interval_task() for each task, then ps_interval_end(), all under the
 * caller's lock (one set of readings for every shell: the interval runs
 * from the previous list, whichever shell asked for it).
 *
 * Tasks are told apart by their handle (id) and, where the OS gives one, a
 * number that it does not reuse (serial; 0 where there is none), not by
 * name.  A task that was not in the previous list counts from 0 (its time
 * in the interval, as it started in it).  A task of the previous list that
 * is gone is dropped.  A counter lower than the previous reading of the
 * same id and serial is that of a new task in the place of the old one (a
 * handle used again), and counts from 0 too.  The 64-bit counters do not
 * wrap in practice (microseconds: 584,000 years; cycles at 125 MHz: 4,600
 * years).
 */

#ifndef COMMON_SHELL_INCLUDE_PS_INTERVAL_H
#define COMMON_SHELL_INCLUDE_PS_INTERVAL_H

#include <stddef.h>
#include <stdint.h>

/* The tasks a list can keep (more count from 0 at each list). */
#ifndef PS_INTERVAL_TASKS_MAX
#define PS_INTERVAL_TASKS_MAX 24
#endif

struct ps_interval_task {
	uintptr_t id;
	uint32_t serial;
	uint64_t time;
};

struct ps_interval {
	/* The readings of the previous list. */
	struct ps_interval_task prev[PS_INTERVAL_TASKS_MAX];
	size_t prev_count;
	uint64_t prev_total;
	/* Those of the list being made. */
	struct ps_interval_task next[PS_INTERVAL_TASKS_MAX];
	size_t next_count;
	uint64_t next_total;
};

/* Starts a list with the counter of all the time; returns the length of the
 * interval in the same unit (since the counter's start for the first list).
 */
static inline uint64_t ps_interval_begin(struct ps_interval *p, uint64_t total)
{
	p->next_count = 0;
	p->next_total = total;
	return total >= p->prev_total ? total - p->prev_total : total;
}

/* A task's counter; returns its time in the interval. */
static inline uint64_t ps_interval_task(struct ps_interval *p, uintptr_t id, uint32_t serial,
					uint64_t time)
{
	uint64_t delta = time;

	for (size_t i = 0; i < p->prev_count; i++) {
		const struct ps_interval_task *t = &p->prev[i];

		if (t->id == id && t->serial == serial) {
			if (time >= t->time) {
				delta = time - t->time;
			}
			break;
		}
	}
	if (p->next_count < PS_INTERVAL_TASKS_MAX) {
		p->next[p->next_count++] = (struct ps_interval_task){
			.id = id,
			.serial = serial,
			.time = time,
		};
	}
	return delta;
}

/* Ends the list: its readings become the previous ones. */
static inline void ps_interval_end(struct ps_interval *p)
{
	for (size_t i = 0; i < p->next_count; i++) {
		p->prev[i] = p->next[i];
	}
	p->prev_count = p->next_count;
	p->prev_total = p->next_total;
}

/* part of whole in tenths of a percent, rounded to the nearest, at most
 * 1000; 0 when whole is 0 (no time has passed).
 */
static inline unsigned int ps_interval_permille(uint64_t part, uint64_t whole)
{
	if (whole == 0) {
		return 0;
	}
	if (part >= whole) {
		return 1000;
	}
	/* part < whole; part * 1000 would overflow past 1.8e16 (cycles at
	 * 125 MHz: 4.6 years): drop low bits of both first.
	 */
	while (whole > UINT64_MAX / 1000u) {
		part >>= 1;
		whole >>= 1;
	}
	return (unsigned int)((part * 1000u + whole / 2u) / whole);
}

#endif /* COMMON_SHELL_INCLUDE_PS_INTERVAL_H */
