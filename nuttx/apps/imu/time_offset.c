/* SPDX-License-Identifier: Apache-2.0 */
#include "time_offset.h"

#include <string.h>

void time_offset_init(struct time_offset *t)
{
	memset(t, 0, sizeof(*t));
	t->min_delay_us = UINT32_MAX;
}

static const struct time_offset_sample *best(const struct time_offset *t)
{
	const struct time_offset_sample *b = NULL;

	for (unsigned int i = 0; i < t->count; i++) {
		if (!b || t->window[i].delay_us < b->delay_us) {
			b = &t->window[i];
		}
	}
	return b;
}

bool time_offset_get(const struct time_offset *t, int64_t mono_us, int64_t *server_us,
		     uint32_t *delay_us)
{
	const struct time_offset_sample *b = best(t);
	int64_t offset;

	if (!b) {
		return false;
	}
	offset = b->offset_us;
	if (t->rate_valid) {
		offset += (mono_us - b->mono_us) * t->rate_ppb / 1000000000LL;
	}
	*server_us = mono_us + offset;
	if (delay_us) {
		*delay_us = b->delay_us;
	}
	return true;
}

static void update_rate(struct time_offset *t)
{
	const struct time_offset_sample *first = &t->anchors[0];
	const struct time_offset_sample *last = &t->anchors[t->anchor_count - 1];
	int64_t span = last->mono_us - first->mono_us;
	int64_t rate;

	if (t->anchor_count < 2 || span < TIME_OFFSET_MIN_RATE_SPAN_US) {
		return;
	}
	rate = (last->offset_us - first->offset_us) * 1000000000LL / span;
	if (rate > TIME_OFFSET_MAX_RATE_PPB || rate < -TIME_OFFSET_MAX_RATE_PPB) {
		/* Not a plausible oscillator difference: a clock was stepped or
		 * the path changed. Start over from the newest point. */
		t->anchors[0] = *last;
		t->anchor_count = 1;
		t->rate_valid = false;
		return;
	}
	t->rate_ppb = rate;
	t->rate_valid = true;
}

void time_offset_add(struct time_offset *t, const struct time_offset_sample *sample)
{
	int64_t predicted;

	t->last_residual_us = time_offset_get(t, sample->mono_us, &predicted, NULL) ?
		sample->mono_us + sample->offset_us - predicted : 0;
	t->window[t->next] = *sample;
	t->next = (t->next + 1) % TIME_OFFSET_WINDOW;
	if (t->count < TIME_OFFSET_WINDOW) {
		t->count++;
	}
	t->samples++;
	t->last_delay_us = sample->delay_us;
	if (sample->delay_us < t->min_delay_us) {
		t->min_delay_us = sample->delay_us;
	}
	/* Once per full window, keep its best exchange for the rate. */
	if (++t->since_anchor == TIME_OFFSET_WINDOW) {
		t->since_anchor = 0;
		if (t->anchor_count == TIME_OFFSET_ANCHORS) {
			memmove(&t->anchors[0], &t->anchors[1],
				(TIME_OFFSET_ANCHORS - 1) * sizeof(t->anchors[0]));
			t->anchor_count--;
		}
		t->anchors[t->anchor_count++] = *best(t);
		update_rate(t);
	}
}

void time_slew_init(struct time_slew *s)
{
	memset(s, 0, sizeof(*s));
}

static int64_t segment_time(const struct time_slew_segment *g, int64_t mono_us)
{
	int64_t elapsed = mono_us - g->ref_mono_us;
	/* The correction ends after TIME_SLEW_SPAN_US even if no update follows,
	 * so a lost server cannot make the clock run away. */
	int64_t slewing = elapsed < TIME_SLEW_SPAN_US ? elapsed : TIME_SLEW_SPAN_US;

	return g->ref_time_us + elapsed + elapsed * g->rate_ppb / 1000000000LL +
	       slewing * g->slew_ppb / 1000000000LL;
}

bool time_slew_get(const struct time_slew *s, int64_t mono_us, int64_t *time_us)
{
	if (!s->valid) {
		return false;
	}
	if (mono_us >= s->now.ref_mono_us) {
		*time_us = segment_time(&s->now, mono_us);
		return true;
	}
	if (s->previous_valid && mono_us >= s->previous.ref_mono_us) {
		*time_us = segment_time(&s->previous, mono_us);
		return true;
	}
	return false;
}

void time_slew_update(struct time_slew *s, int64_t mono_us, int64_t target_us, int64_t rate_ppb)
{
	int64_t current, error, slew;

	if (!s->valid || mono_us < s->now.ref_mono_us) {
		s->previous_valid = false;
		s->now = (struct time_slew_segment){mono_us, target_us, rate_ppb, 0};
		if (s->valid) {
			s->steps++;
		}
		s->valid = true;
		return;
	}
	current = segment_time(&s->now, mono_us);
	error = target_us - current;
	if (error >= TIME_SLEW_STEP_US || error <= -TIME_SLEW_STEP_US) {
		/* Times before a step must not be converted with the new clock. */
		s->previous_valid = false;
		s->now = (struct time_slew_segment){mono_us, target_us, rate_ppb, 0};
		s->steps++;
		return;
	}
	slew = error * 1000000000LL / TIME_SLEW_SPAN_US;
	if (slew > TIME_SLEW_MAX_PPB) {
		slew = TIME_SLEW_MAX_PPB;
	} else if (slew < -TIME_SLEW_MAX_PPB) {
		slew = -TIME_SLEW_MAX_PPB;
	}
	s->previous = s->now;
	s->previous_valid = true;
	s->now = (struct time_slew_segment){mono_us, current, rate_ppb, slew};
}
