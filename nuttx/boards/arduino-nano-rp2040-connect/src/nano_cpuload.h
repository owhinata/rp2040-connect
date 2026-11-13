/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_cpuload.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/* The arithmetic of cpu_load_get() (nano_idle.c), kept free of OS calls so
 * the host tests (nuttx/tests/nano_cpuload.c) can check it.  The steps are
 * those of Zephyr's cpu_load_get_cpu()
 * (zephyr/zephyr/lib/os/cpu_load/cpu_load_backend_idle.c) with the times
 * already in microseconds.
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_CPULOAD_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_CPULOAD_H

#include <stdint.h>

/* Idle accounting.  start: TIMER count at the last reset (0 = boot).
 * idle: idle microseconds since then, not counting an open window.
 * open/enter: an idle window is open since TIMER count enter.
 */

struct nano_cpuload_s
{
  uint64_t start;
  uint64_t idle;
  uint64_t enter;
  int open;
};

/* The window opens: Zephyr's cpu_load_on_enter_idle(). */

static inline void nano_cpuload_enter(struct nano_cpuload_s *s, uint64_t now)
{
  s->enter = now;
  s->open = 1;
}

/* The window closes, idempotent: Zephyr's cpu_load_on_exit_idle(). */

static inline void nano_cpuload_exit(struct nano_cpuload_s *s, uint64_t now)
{
  if (!s->open)
    {
      return;
    }

  s->idle += now - s->enter;
  s->open = 0;
}

/* The reading at TIMER count now (and idle_now, the second reading Zephyr
 * takes for an open window), with reset: the locked part of Zephyr's
 * cpu_load_get_cpu().  Sets *total and *idle (microseconds).
 */

static inline void nano_cpuload_sample(struct nano_cpuload_s *s,
                                       uint64_t now, uint64_t idle_now,
                                       int reset, uint64_t *total,
                                       uint64_t *idle)
{
  *total = now - s->start;
  *idle = s->idle;

  if (s->open)
    {
      *idle += idle_now - s->enter;
      if (reset)
        {
          /* Re-open the window at idle_now so the idle time just accounted
           * for is not counted again when it closes.
           */

          s->enter = idle_now;
        }
    }

  if (reset)
    {
      s->start = now;
      s->idle = 0;
    }
}

/* The rest of cpu_load_get_cpu(), after the lock: tenths of a percent,
 * rounded down; 0 when no time has passed.
 */

static inline int nano_cpuload_ratio(uint64_t total, uint64_t idle)
{
  uint64_t active;

  if (total == 0)
    {
      return 0;
    }

  if (idle > total)
    {
      idle = total;
    }

  active = total - idle;
  return (int)(uint32_t)((1000 * active) / total);
}

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_NANO_CPULOAD_H */
