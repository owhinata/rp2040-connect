/* SPDX-License-Identifier: Apache-2.0 */
#include <assert.h>
#include <stdlib.h>
#include "time_offset.h"

#define SEC 1000000LL

int main(void)
{
	struct time_offset t;
	int64_t server;
	uint32_t delay;
	const int64_t base = 1794000000LL * SEC;

	time_offset_init(&t);
	assert(!time_offset_get(&t, 0, &server, &delay));

	/* The shortest round trip in the window wins, not the newest. */
	time_offset_add(&t, &(struct time_offset_sample){1 * SEC, base + 9000, 20000});
	assert(time_offset_get(&t, 1 * SEC, &server, &delay) && server == 1 * SEC + base + 9000 &&
	       delay == 20000);
	time_offset_add(&t, &(struct time_offset_sample){3 * SEC, base + 100, 4000});
	time_offset_add(&t, &(struct time_offset_sample){5 * SEC, base - 7000, 30000});
	assert(time_offset_get(&t, 6 * SEC, &server, &delay) && server == 6 * SEC + base + 100 &&
	       delay == 4000);
	assert(t.last_residual_us == -7100 && t.min_delay_us == 4000 && t.samples == 3);

	/* A short round trip leaves the window after TIME_OFFSET_WINDOW samples. */
	for (int i = 0; i < TIME_OFFSET_WINDOW; i++) {
		time_offset_add(&t, &(struct time_offset_sample){(7 + 2 * i) * SEC, base + 500, 6000});
	}
	assert(time_offset_get(&t, 30 * SEC, &server, &delay) && delay == 6000 &&
	       server == 30 * SEC + base + 500);

	/* A board clock running 40 ppm slow: the offset grows 40 us per second.
	 * Noisy exchanges are later by up to half of their extra delay. */
	time_offset_init(&t);
	srand(1);
	for (int i = 0; i < 80; i++) {
		int64_t mono = (int64_t)i * 2 * SEC;
		uint32_t extra = i % 4 == 0 ? 0 : (uint32_t)(rand() % 20000);

		time_offset_add(&t, &(struct time_offset_sample){
			mono, base + mono * 40 / 1000000 + extra / 2, 3000 + extra});
	}
	assert(t.rate_valid && t.rate_ppb > 39000 && t.rate_ppb < 41000);
	/* Extrapolating 10 s past the last exchange stays within 0.1 ms. */
	int64_t mono = 80 * 2 * SEC + 10 * SEC;
	assert(time_offset_get(&t, mono, &server, &delay) && delay == 3000);
	assert(llabs(server - (mono + base + mono * 40 / 1000000)) < 100);

	/* A stepped server clock is not taken for drift. */
	for (int i = 80; i < 80 + 2 * TIME_OFFSET_WINDOW; i++) {
		int64_t m = (int64_t)i * 2 * SEC;

		time_offset_add(&t, &(struct time_offset_sample){m, base + 5 * SEC, 3000});
	}
	assert(!t.rate_valid);
	mono = (80 + 2 * TIME_OFFSET_WINDOW) * 2 * SEC;
	assert(time_offset_get(&t, mono, &server, &delay) && server == mono + base + 5 * SEC);

	/* Slewed clock: the first target is taken as it is. */
	struct time_slew s;
	int64_t previous, now;
#define GET(m) (assert(time_slew_get(&s, (m), &now)), now)

	time_slew_init(&s);
	assert(!time_slew_get(&s, 0, &now));
	time_slew_update(&s, 10 * SEC, base, 0);
	assert(GET(10 * SEC) == base && GET(11 * SEC) == base + SEC);
	assert(!time_slew_get(&s, 10 * SEC - 1, &now));
	/* The estimate drops by 3 ms: the clock keeps increasing and gets there
	 * within the span (3 ms over 8 s is 375 ppm, below the limit). */
	time_slew_update(&s, 12 * SEC, base + 2 * SEC - 3000, 0);
	previous = GET(12 * SEC);
	assert(previous == base + 2 * SEC && s.steps == 0);
	/* A time before the latest update is answered from the segment that
	 * was in effect then, continuously. */
	assert(GET(11 * SEC + 500000) == base + SEC + 500000 && GET(12 * SEC - 1) == previous - 1);
	for (int64_t m = 12 * SEC + 1000; m <= 30 * SEC; m += 1000) {
		assert(GET(m) > previous && now - previous <= 1001);
		previous = now;
	}
	assert(GET(20 * SEC) == base + 10 * SEC - 3000);
	/* No overshoot when no update follows. */
	assert(GET(30 * SEC) == base + 20 * SEC - 3000);
	/* A 20 ms error exceeds the limit: 500 ppm removes 1 ms per 2 s update
	 * until 4 ms remain. */
	time_slew_init(&s);
	time_slew_update(&s, 0, base, 0);
	for (int i = 1; i <= 30; i++) {
		int64_t m = (int64_t)i * 2 * SEC;

		previous = GET(m);
		time_slew_update(&s, m, base + m + 20000, 0);
		assert(GET(m) == previous);
		now = base + m + 20000 - now;
		assert(i > 17 || now == 20000 - (i - 1) * 1000);
	}
	/* Below 4 ms the correction is proportional and fades out. */
	assert(now >= 0 && now < 200 && s.steps == 0);
	/* Only one update back is kept. */
	assert(time_slew_get(&s, 58 * SEC, &now) && !time_slew_get(&s, 58 * SEC - 1, &now));
	/* The estimated rate is followed without using the slew budget. */
	time_slew_init(&s);
	time_slew_update(&s, 0, base, 40000);
	assert(GET(100 * SEC) == base + 100 * SEC + 4000);
	/* Half a second or more is a step, is counted, and cuts off the past. */
	time_slew_update(&s, 101 * SEC, base + 200 * SEC, 0);
	assert(s.steps == 1 && GET(101 * SEC) == base + 200 * SEC);
	assert(!time_slew_get(&s, 100 * SEC, &now));
	return 0;
}
