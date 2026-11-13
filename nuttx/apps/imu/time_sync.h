/* SPDX-License-Identifier: Apache-2.0 */
#ifndef TIME_SYNC_H_
#define TIME_SYNC_H_

#include <stdbool.h>
#include <stdint.h>

void time_sync_init(void);
/* True while the server time estimate is recent. */
bool time_sync_is_ready(void);
/* The board clock that the estimate refers to: microseconds since boot,
 * never stepped. */
int64_t time_sync_mono_us(void);
/* Server (Unix) time in microseconds for a board time. The result is
 * continuous and strictly increasing with mono_us: corrections are spread
 * over seconds, except for the first one and steps of half a second or more.
 * *delay_us is the round trip of the exchange the estimate rests on.
 * Returns false while no recent estimate exists or for a time before the
 * last correction. */
bool time_sync_unix_us(int64_t mono_us, int64_t *unix_us, uint32_t *delay_us);
/* The unsmoothed estimate for the same instant. It is the better value at
 * any one moment but jumps by a few milliseconds, in either direction, when
 * the underlying exchange changes; do not use it to order events. */
bool time_sync_estimate_us(int64_t mono_us, int64_t *unix_us, uint32_t *delay_us);

#endif
