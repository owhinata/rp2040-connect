/* SPDX-License-Identifier: Apache-2.0 */
#ifndef TIME_OFFSET_H_
#define TIME_OFFSET_H_

#include <stdbool.h>
#include <stdint.h>

/* Estimates server time from the board's monotonic clock using repeated
 * request/response exchanges (NTP style).
 *
 * One exchange gives offset = server - board, wrong by up to half its round
 * trip when the two directions differ. The exchange with the shortest round
 * trip among the last TIME_OFFSET_WINDOW is therefore used as the estimate.
 * The rate between the clocks comes from the best exchanges of successive
 * windows. Nothing here can detect a constant asymmetry of the path.
 */
#define TIME_OFFSET_WINDOW  8
#define TIME_OFFSET_ANCHORS 6
/* Rate is only trusted over at least this span, and only within this bound;
 * crystal oscillators differ by tens of ppm. */
#define TIME_OFFSET_MIN_RATE_SPAN_US 30000000LL
#define TIME_OFFSET_MAX_RATE_PPB     500000LL

struct time_offset_sample {
	int64_t mono_us;   /* board monotonic time of the exchange */
	int64_t offset_us; /* server time minus board monotonic time */
	uint32_t delay_us; /* round trip without the server's processing time */
};

struct time_offset {
	struct time_offset_sample window[TIME_OFFSET_WINDOW];
	struct time_offset_sample anchors[TIME_OFFSET_ANCHORS];
	unsigned int count, next, anchor_count, since_anchor;
	int64_t rate_ppb; /* change of offset per second of board time */
	bool rate_valid;
	/* Diagnostics. */
	uint32_t samples, min_delay_us, last_delay_us;
	int64_t last_residual_us; /* newest offset minus the estimate before it */
};

void time_offset_init(struct time_offset *t);
void time_offset_add(struct time_offset *t, const struct time_offset_sample *sample);
/* Returns false until an exchange has been added. *delay_us is the round
 * trip of the exchange the estimate is based on; half of it bounds the error
 * from path asymmetry at that moment. */
bool time_offset_get(const struct time_offset *t, int64_t mono_us, int64_t *server_us,
		     uint32_t *delay_us);

/* A clock that follows the estimate without ever going backwards.
 *
 * The estimate jumps by a few milliseconds, in either direction, whenever it
 * switches to another exchange. This clock removes the difference gradually
 * instead: its speed differs from the board clock by the estimated rate plus
 * at most TIME_SLEW_MAX_PPB, so it is continuous and strictly increasing.
 * Only a difference of TIME_SLEW_STEP_US or more (first use, a stepped
 * server) is applied at once, and that is counted in `steps`.
 */
#define TIME_SLEW_MAX_PPB 500000LL
#define TIME_SLEW_SPAN_US 8000000LL
#define TIME_SLEW_STEP_US 500000LL

struct time_slew_segment {
	int64_t ref_mono_us, ref_time_us;
	int64_t rate_ppb, slew_ppb;
};

struct time_slew {
	bool valid, previous_valid;
	struct time_slew_segment now, previous;
	uint32_t steps;
};

void time_slew_init(struct time_slew *s);
/* target_us is the estimate for mono_us, rate_ppb its rate (0 if unknown). */
void time_slew_update(struct time_slew *s, int64_t mono_us, int64_t target_us, int64_t rate_ppb);
/* Events are converted a little after they happen, so times back to the
 * update before the last one are answered (false if earlier, or across a
 * step). The clock is continuous over both segments. */
bool time_slew_get(const struct time_slew *s, int64_t mono_us, int64_t *time_us);
#endif
