/* SPDX-License-Identifier: MIT */
/* <zephyr/sys/clock.h> of the Zephyr compatibility layer on FreeRTOS: the
 * monotonic clock is the RP2040 TIMER (rp2040_timer_us()) and the realtime
 * clock is the TIMER plus a base, under a spinlock (the 64-bit base is two
 * words).  See the header.
 *
 * Who reads the realtime clock.  In the Zephyr build only time_sync.c sets
 * it (forward only) and reads it, and the shell's "date" shows it; in the
 * NuttX build the same, with nsh's "date" on NuttX's CLOCK_REALTIME.  No
 * code of the port uses it for anything else (every time on the batches is
 * time_sync_unix_us(), from the TIMER).  The FreeRTOS build has no "date",
 * so time_sync.c is its only user; newlib's gettimeofday() and time() are
 * put on the same clock (below), so that the build has one realtime clock,
 * as Zephyr's and NuttX's C libraries read their kernel's.  Without this they
 * would be the Pico SDK's own (pico_clib_interface/newlib_interface.c: the
 * TIMER plus a base of its own, which settimeofday() sets), a second clock
 * that time_sync.c never moves.
 */

#include <errno.h>
#include <stdint.h>
#include <time.h>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/clock.h>

static struct k_spinlock lock;
/* Realtime minus the TIMER, in microseconds.  0 until the first set: the
 * realtime clock then starts at 1970-01-01 at the start, as Zephyr's
 * (offset 0) and NuttX's (no RTC) do.
 */
static int64_t realtime_base_us;

static int64_t realtime_us(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	int64_t us = (int64_t)rp2040_timer_us() + realtime_base_us;

	k_spin_unlock(&lock, key);
	return us;
}

static void set_realtime_us(int64_t us)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	realtime_base_us = us - (int64_t)rp2040_timer_us();
	k_spin_unlock(&lock, key);
}

/* Microseconds to a timespec with 0 <= tv_nsec < NSEC_PER_SEC, also before
 * 1970 (a negative count), as Zephyr's normalized timespec.
 */
static void timespec_from_us(int64_t us, struct timespec *ts)
{
	int64_t sec = us / USEC_PER_SEC, rem = us % USEC_PER_SEC;

	if (rem < 0) {
		sec--;
		rem += USEC_PER_SEC;
	}
	ts->tv_sec = (time_t)sec;
	ts->tv_nsec = (long)(rem * NSEC_PER_USEC);
}

int sys_clock_gettime(int clock_id, struct timespec *ts)
{
	switch (clock_id) {
	case SYS_CLOCK_REALTIME:
		timespec_from_us(realtime_us(), ts);
		return 0;
	case SYS_CLOCK_MONOTONIC:
		timespec_from_us((int64_t)rp2040_timer_us(), ts);
		return 0;
	default:
		return -EINVAL;
	}
}

/* Zephyr: only SYS_CLOCK_REALTIME, and a valid timespec
 * (timespec_is_valid(): 0 <= tv_nsec < NSEC_PER_SEC).  Below a microsecond
 * is dropped (rounded toward the past).
 */
int sys_clock_settime(int clock_id, const struct timespec *ts)
{
	if (clock_id != SYS_CLOCK_REALTIME) {
		return -EINVAL;
	}
	if (ts->tv_nsec < 0 || ts->tv_nsec >= (long)NSEC_PER_SEC) {
		return -EINVAL;
	}

	set_realtime_us((int64_t)ts->tv_sec * USEC_PER_SEC + ts->tv_nsec / (long)NSEC_PER_USEC);
	return 0;
}

#ifdef __NEWLIB__
#include <sys/time.h>

/* newlib's gettimeofday() (and time(), which calls it) and settimeofday() on
 * the realtime clock above, in place of the Pico SDK's weak ones.  No time
 * zone: UTC, as the Zephyr and NuttX builds' clocks.
 */
int _gettimeofday(struct timeval *tv, void *tz)
{
	(void)tz;

	if (tv != NULL) {
		int64_t us = realtime_us(), sec = us / USEC_PER_SEC, rem = us % USEC_PER_SEC;

		if (rem < 0) {
			sec--;
			rem += USEC_PER_SEC;
		}
		tv->tv_sec = (time_t)sec;
		tv->tv_usec = (suseconds_t)rem;
	}
	return 0;
}

int settimeofday(const struct timeval *tv, const struct timezone *tz)
{
	(void)tz;

	if (tv != NULL) {
		if (tv->tv_usec < 0 || tv->tv_usec >= USEC_PER_SEC) {
			errno = EINVAL;
			return -1;
		}
		set_realtime_us((int64_t)tv->tv_sec * USEC_PER_SEC + tv->tv_usec);
	}
	return 0;
}
#endif /* __NEWLIB__ */
