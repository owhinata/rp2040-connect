/* SPDX-License-Identifier: MIT */
/* The waits of the NINA socket loops (nina_wait.h), shared by the Zephyr,
 * NuttX and FreeRTOS builds.  The OS's events (nina_os_event_*() of
 * nina_os.h) are used once nina_wait_start_events() has succeeded; until
 * then, or if it fails, every wait is the loop's poll interval.  The host
 * test common/tests/nina_wait.c builds this file with fakes of nina_os.h,
 * whose nina_os_event_start() succeeds or fails.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "nina_os.h"
#include "nina_wait.h"

/* Set once by nina_wait_start_events(), read by every loop. */
static volatile bool events;

int nina_wait_start_events(void)
{
	int ret;

	if (events) {
		return 0;
	}
	ret = nina_os_event_start();
	if (!ret) {
		events = true;
	}
	return ret;
}

bool nina_wait_events(void)
{
	return events;
}

void nina_wait_mark(struct nina_wait *w)
{
	if (events) {
		w->mark = nina_os_event_seq();
	}
}

uint32_t nina_wait_backoff_ms(unsigned int woken)
{
	/* The first two waits that an edge ended are free. */
	if (woken < 2) {
		return 0;
	}
	if (woken - 2 >= 5) {
		return NINA_POLL_INTERVAL_MS;
	}
	return 1u << (woken - 2);
}

/* The milliseconds from now to END, 0 if it has passed, at most LIMIT. */
static uint32_t left_ms(int64_t end, uint32_t limit)
{
	int64_t left = end - nina_os_uptime_ms();

	if (left <= 0) {
		return 0;
	}
	return left < (int64_t)limit ? (uint32_t)left : limit;
}

void nina_wait(struct nina_wait *w, uint32_t poll_ms, int64_t deadline)
{
	if (events) {
		uint32_t ms = left_ms(deadline, NINA_EVENT_TIMEOUT_MS);
		int64_t end = nina_os_uptime_ms() + ms;
		uint32_t backoff = nina_wait_backoff_ms(w->woken);

		if (backoff > ms) {
			backoff = ms;
		}
		if (backoff) {
			nina_os_sleep_ms(backoff);
			ms = left_ms(end, ms);
		}
		nina_os_event_wait(w->mark, ms);
		if (nina_os_event_seq() != w->mark) {
			if (w->woken < UINT8_MAX) {
				w->woken++;
			}
		} else {
			w->woken = 0;
		}
		return;
	}
	nina_os_sleep_ms(poll_ms);
}
