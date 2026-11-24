/* SPDX-License-Identifier: Apache-2.0 */
/* CPU load from idle windows, the equivalent of Zephyr's CONFIG_CPU_LOAD
 * with CONFIG_CPU_LOAD_BACKEND_IDLE_HOOK (<zephyr/sys/cpu_load.h>,
 * lib/os/cpu_load/cpu_load_backend_idle.c).  The OS implements
 * cpu_load_get() in its idle loop with the arithmetic below (on NuttX the
 * board's up_idle(), boards/arduino-nano-rp2040-connect/src/
 * rp2040_idle_cpuload.c), which is kept free of OS calls so that the host
 * tests (common/tests/cpu_load.c) can check it.  Its steps are those of
 * Zephyr's cpu_load_get_cpu() (zephyr/zephyr/lib/os/cpu_load/
 * cpu_load_backend_idle.c) with the times already in microseconds.
 */

#ifndef COMMON_COMPAT_PORT_CPU_LOAD_H
#define COMMON_COMPAT_PORT_CPU_LOAD_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* The load since the last call with reset=true (since boot before the
 * first one), in tenths of a percent: 1000 * (total - idle) / total,
 * rounded down, both in microseconds of the TIMER.  0 when no time has
 * passed.  Idle time is the time the CPU spends waiting for an interrupt
 * in the idle loop; the window closes when the interrupt arrives, before
 * its handler runs, so interrupt handlers count as load even when they
 * interrupt idle.  reset=true starts a new measurement window at this
 * call.
 */

int cpu_load_get(bool reset);

/* The arithmetic ------------------------------------------------------- */

/* Idle accounting.  start: TIMER count at the last reset (0 = boot).
 * idle: idle microseconds since then, not counting an open window.
 * open/enter: an idle window is open since TIMER count enter.
 */

struct cpu_load_window
{
  uint64_t start;
  uint64_t idle;
  uint64_t enter;
  int open;
};

/* The window opens: Zephyr's cpu_load_on_enter_idle(). */

static inline void cpu_load_enter(struct cpu_load_window *s, uint64_t now)
{
  s->enter = now;
  s->open = 1;
}

/* The window closes, idempotent: Zephyr's cpu_load_on_exit_idle(). */

static inline void cpu_load_exit(struct cpu_load_window *s, uint64_t now)
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

static inline void cpu_load_sample(struct cpu_load_window *s,
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

static inline int cpu_load_ratio(uint64_t total, uint64_t idle)
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

#ifdef __cplusplus
}
#endif

#endif /* COMMON_COMPAT_PORT_CPU_LOAD_H */
