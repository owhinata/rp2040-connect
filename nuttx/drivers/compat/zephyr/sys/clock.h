/* SPDX-License-Identifier: Apache-2.0 */
/* The realtime clock calls of Zephyr's <zephyr/sys/clock.h> (Zephyr
 * v4.5.0) that the time synchronization (time_sync.c of the Zephyr build)
 * uses, with Zephyr's names and results, on NuttX's clock_gettime() and
 * clock_settime():
 *
 *   SYS_CLOCK_REALTIME         CLOCK_REALTIME
 *   sys_clock_gettime(id, ts)  0, or -EINVAL for an unknown clock
 *   sys_clock_settime(id, ts)  0, or -EINVAL for an unknown clock or a bad
 *                              time (Zephyr: only SYS_CLOCK_REALTIME can be
 *                              set)
 *
 * and NSEC_PER_USEC, USEC_PER_SEC (NuttX's <nuttx/clock.h> defines both;
 * Zephyr's <zephyr/sys/time_units.h>).  NuttX's realtime clock is a base
 * time added to the monotonic clock (the TIMER, nano_oneshot.c of the
 * board); setting it moves only the base, as Zephyr's moves only its
 * offset from the uptime.  Part of the small Zephyr compatibility layer of
 * the port (nuttx/drivers/compat, see nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_CLOCK_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_CLOCK_H

#include <errno.h>
#include <time.h>

#ifdef __NuttX__
#  include <nuttx/clock.h>
#endif

#ifndef NSEC_PER_USEC
#  define NSEC_PER_USEC 1000
#endif

#ifndef USEC_PER_SEC
#  define USEC_PER_SEC  1000000
#endif

#define SYS_CLOCK_REALTIME CLOCK_REALTIME

static inline int sys_clock_gettime(int clock_id, struct timespec *ts)
{
  return clock_gettime(clock_id, ts) == 0 ? 0 : -EINVAL;
}

static inline int sys_clock_settime(int clock_id, const struct timespec *ts)
{
  if (clock_id != SYS_CLOCK_REALTIME)
    {
      return -EINVAL;
    }

  return clock_settime(clock_id, ts) == 0 ? 0 : -EINVAL;
}

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_CLOCK_H */
