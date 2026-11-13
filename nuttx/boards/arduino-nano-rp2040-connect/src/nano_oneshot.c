/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_oneshot.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* Derived from arch/arm/src/rp23xx/rp23xx_oneshot.c (NuttX 13.1.0).
 *
 * A oneshot lower-half (CONFIG_ONESHOT_COUNT interface) backed by ALARM0 of
 * the RP2040 TIMER, used to drive the tickless scheduler through
 * up_alarm_set_lowerhalf() (drivers/timers/arch_alarm.c, see
 * nano_timerisr.c).  The TIMER is a free-running 64-bit counter incremented
 * once per microsecond (the watchdog tick, 1 MHz, rp2040_clock.c), and
 * serves directly as the monotonic time base returned by current().
 * ALARM1..3 are left free.
 *
 * Differences from the RP2350 original: the RP2040 has one TIMER block (no
 * TIMER0/TIMER1 choice) whose interrupt registers sit at other offsets
 * (nano_timer.h), its 1 MHz tick comes from the watchdog tick rather than
 * the TICKS block, and the interrupt handler returns without calling back
 * when the alarm is not actually pending (see nano_oneshot_interrupt()).
 *
 * As on the RP2350, the ALARM registers match only the low 32 bits of the
 * counter.  max_delay() is therefore capped below 2^32 counts so the
 * scheduler never asks for a longer interval, and any deadline that is
 * already due (or that the counter reaches while the alarm is being armed)
 * is raised at once through the INTF force register rather than waiting a
 * full 32-bit wrap (~71.6 minutes) for the compare to match again.
 *
 * This file also replaces the weak time functions of
 * drivers/timers/arch_alarm.c that the tickless scheduler calls on every
 * sleep, wake-up and context switch (up_timer_gettick(),
 * up_alarm_tick_start(), up_timer_gettime(), up_alarm_cancel()), and
 * up_udelay()/up_ndelay().  The generic versions convert through the
 * ONESHOT_COUNT layer (include/nuttx/timers/oneshot.h), whose 64-bit
 * divisions by the run-time frequency go to libgcc's __aeabi_uldivmod: on
 * the Cortex-M0+ (no divide instruction, no 32x32->64 multiply) about 6 us
 * per call.  The replacements read the TIMER directly and convert with
 * nano_timeconv.h, which is specialised for 1 MHz counts and 100 us ticks
 * and gives the same results for every 64-bit value.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/oneshot.h>
#include <arch/board/nano_timer.h>

#include "chip.h"
#include "arm_internal.h"

#include "nano_connect.h"
#include "nano_timeconv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This driver uses ALARM0; alarm 0 is bit 0 in the ARMED, INTE, INTF, INTR
 * and INTS registers.
 */

#define ONESHOT_ALARM0_BIT  NANO_TIMER_ALARM(0)
#define ONESHOT_IRQ         RP2040_TIMER_IRQ_0

#define ONESHOT_ARMED       (NANO_TIMER_BASE + NANO_TIMER_ARMED_OFFSET)
#define ONESHOT_ALARM0      (NANO_TIMER_BASE + NANO_TIMER_ALARM0_OFFSET)
#define ONESHOT_INTR        (NANO_TIMER_BASE + NANO_TIMER_INTR_OFFSET)
#define ONESHOT_INTE        (NANO_TIMER_BASE + NANO_TIMER_INTE_OFFSET)
#define ONESHOT_INTF        (NANO_TIMER_BASE + NANO_TIMER_INTF_OFFSET)
#define ONESHOT_INTS        (NANO_TIMER_BASE + NANO_TIMER_INTS_OFFSET)

/* The counter is clocked at 1 MHz (1 us per count). */

#define ONESHOT_FREQ_HZ     1000000

/* nano_timeconv.h converts counts to ticks of exactly 100 counts. */

#if CONFIG_USEC_PER_TICK != 100
#  error "nano_oneshot.c requires CONFIG_USEC_PER_TICK == 100"
#endif

/* The ALARM compare is only 32 bits wide, so never schedule further ahead
 * than this many counts.
 */

#define ONESHOT_MAX_COUNTS  ((clkcnt_t)UINT32_MAX)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static clkcnt_t nano_oneshot_max_delay(FAR struct oneshot_lowerhalf_s *l);
static clkcnt_t nano_oneshot_current(FAR struct oneshot_lowerhalf_s *l);
static void nano_oneshot_start_absolute(FAR struct oneshot_lowerhalf_s *l,
                                        clkcnt_t expected);
static void nano_oneshot_start(FAR struct oneshot_lowerhalf_s *l,
                               clkcnt_t delta);
static void nano_oneshot_cancel(FAR struct oneshot_lowerhalf_s *l);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_nano_oneshot_ops =
{
  .current        = nano_oneshot_current,
  .start          = nano_oneshot_start,
  .start_absolute = nano_oneshot_start_absolute,
  .cancel         = nano_oneshot_cancel,
  .max_delay      = nano_oneshot_max_delay,
};

static struct oneshot_lowerhalf_s g_nano_oneshot =
{
  .ops = &g_nano_oneshot_ops,
};

/* Set once nano_oneshot_initialize() has set up ALARM0, just before
 * up_alarm_set_lowerhalf().  Until then the time functions below return
 * -EAGAIN, as the generic ones do before a lower half is registered.
 */

static bool g_nano_oneshot_ready;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nano_oneshot_count
 *
 * Description:
 *   Read the free-running 64-bit microsecond counter: nano_mono_us()
 *   (include/nano_timer.h), the board's monotonic clock, which reads the
 *   raw (non-latching) registers and guards against a carry from the low
 *   word into the high word between the two reads.  Safe from any context.
 *
 ****************************************************************************/

static inline uint64_t nano_oneshot_count(void)
{
  return nano_mono_us();
}

/****************************************************************************
 * Name: nano_oneshot_setcompare
 *
 * Description:
 *   Arm ALARM0 for the absolute counter value "target".  Must be called with
 *   interrupts disabled.  If the target is already due -- or the counter
 *   reaches it while the alarm is being armed -- the interrupt is forced
 *   immediately through INTF instead of waiting a full 32-bit wrap for the
 *   compare to match.
 *
 ****************************************************************************/

static void nano_oneshot_setcompare(uint64_t target)
{
  /* Clear any stale force left from a previous expiry. */

  modifyreg32(ONESHOT_INTF, ONESHOT_ALARM0_BIT, 0);

  if ((int64_t)(target - nano_oneshot_count()) <= 0)
    {
      /* Already due: raise the interrupt now. */

      modifyreg32(ONESHOT_INTF, 0, ONESHOT_ALARM0_BIT);
    }
  else
    {
      /* Writing ALARM0 arms it against the low 32 bits of the counter. */

      putreg32((uint32_t)target, ONESHOT_ALARM0);

      /* Close the race: if the counter reached the target while we were
       * arming, force so we do not wait a full wrap for the match.
       */

      if ((int64_t)(nano_oneshot_count() - target) >= 0)
        {
          modifyreg32(ONESHOT_INTF, 0, ONESHOT_ALARM0_BIT);
        }
    }
}

/****************************************************************************
 * Name: nano_oneshot_interrupt
 ****************************************************************************/

static int nano_oneshot_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct oneshot_lowerhalf_s *lower = arg;

  /* The NVIC keeps an interrupt pending after the TIMER has dropped its
   * request (cancel() cleared INTR while interrupts were disabled).  Such
   * an entry has no expiry to report, and disarming here would drop an
   * alarm armed since; leave without calling back.
   */

  if ((getreg32(ONESHOT_INTS) & ONESHOT_ALARM0_BIT) == 0)
    {
      return OK;
    }

  /* Disarm the alarm, drop any force and clear the latched match. */

  putreg32(ONESHOT_ALARM0_BIT, ONESHOT_ARMED);
  modifyreg32(ONESHOT_INTF, ONESHOT_ALARM0_BIT, 0);
  putreg32(ONESHOT_ALARM0_BIT, ONESHOT_INTR);

  oneshot_process_callback(lower);
  return OK;
}

/****************************************************************************
 * Name: nano_oneshot_max_delay
 ****************************************************************************/

static clkcnt_t nano_oneshot_max_delay(FAR struct oneshot_lowerhalf_s *l)
{
  UNUSED(l);
  return ONESHOT_MAX_COUNTS;
}

/****************************************************************************
 * Name: nano_oneshot_current
 ****************************************************************************/

static clkcnt_t nano_oneshot_current(FAR struct oneshot_lowerhalf_s *l)
{
  UNUSED(l);
  return nano_oneshot_count();
}

/****************************************************************************
 * Name: nano_oneshot_start_absolute
 ****************************************************************************/

static void nano_oneshot_start_absolute(FAR struct oneshot_lowerhalf_s *l,
                                        clkcnt_t expected)
{
  irqstate_t flags = up_irq_save();

  UNUSED(l);
  nano_oneshot_setcompare(expected);
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: nano_oneshot_start
 ****************************************************************************/

static void nano_oneshot_start(FAR struct oneshot_lowerhalf_s *l,
                               clkcnt_t delta)
{
  irqstate_t flags = up_irq_save();

  UNUSED(l);
  nano_oneshot_setcompare(nano_oneshot_count() + delta);
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: nano_oneshot_cancel
 ****************************************************************************/

static void nano_oneshot_cancel(FAR struct oneshot_lowerhalf_s *l)
{
  irqstate_t flags = up_irq_save();

  UNUSED(l);
  putreg32(ONESHOT_ALARM0_BIT, ONESHOT_ARMED);
  modifyreg32(ONESHOT_INTF, ONESHOT_ALARM0_BIT, 0);
  putreg32(ONESHOT_ALARM0_BIT, ONESHOT_INTR);
  up_irq_restore(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nano_oneshot_initialize
 *
 * Description:
 *   Initialise ALARM0 of the TIMER as a oneshot lower-half.  Called once
 *   from up_timer_initialize() (nano_timerisr.c).
 *
 ****************************************************************************/

FAR struct oneshot_lowerhalf_s *nano_oneshot_initialize(void)
{
  FAR struct oneshot_lowerhalf_s *lower = &g_nano_oneshot;

  oneshot_count_init(lower, ONESHOT_FREQ_HZ);

  /* Start from a known-idle state: alarm disarmed, no force, status clear,
   * then enable the ALARM0 interrupt at the peripheral and the NVIC.
   */

  putreg32(ONESHOT_ALARM0_BIT, ONESHOT_ARMED);
  modifyreg32(ONESHOT_INTF, ONESHOT_ALARM0_BIT, 0);
  putreg32(ONESHOT_ALARM0_BIT, ONESHOT_INTR);
  modifyreg32(ONESHOT_INTE, 0, ONESHOT_ALARM0_BIT);

  irq_attach(ONESHOT_IRQ, nano_oneshot_interrupt, lower);
  up_enable_irq(ONESHOT_IRQ);

  g_nano_oneshot_ready = true;
  return lower;
}

/****************************************************************************
 * Name: up_timer_gettick
 *
 * Description:
 *   The current OS tick: the TIMER count over 100.  Replaces the weak
 *   function of arch_alarm.c (ONESHOT_TICK_CURRENT); called through
 *   clock_systime_ticks() on every sleep, wake-up and round-robin context
 *   switch, and by nxsched_process_timer().
 *
 ****************************************************************************/

int up_timer_gettick(FAR clock_t *ticks)
{
  if (!g_nano_oneshot_ready)
    {
      return -EAGAIN;
    }

  *ticks = (clock_t)nano_count_to_tick(nano_oneshot_count());
  return OK;
}

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   The time since boot (CLOCK_MONOTONIC, through clock_systime_timespec()).
 *   Replaces the weak function of arch_alarm.c (ONESHOT_CURRENT).
 *
 ****************************************************************************/

int up_timer_gettime(FAR struct timespec *ts)
{
  uint64_t sec;
  uint32_t nsec;

  if (!g_nano_oneshot_ready)
    {
      return -EAGAIN;
    }

  nano_count_to_time(nano_oneshot_count(), &sec, &nsec);
  ts->tv_sec  = (time_t)sec;
  ts->tv_nsec = (long)nsec;
  return OK;
}

/****************************************************************************
 * Name: up_alarm_tick_start
 *
 * Description:
 *   Raise the alarm (and so nxsched_process_timer()) at the start of the
 *   given tick, TIMER count ticks * 100.  Replaces the weak function of
 *   arch_alarm.c (ONESHOT_TICK_ABSOLUTE); the wdog code calls it whenever
 *   the earliest deadline changes.
 *
 ****************************************************************************/

int up_alarm_tick_start(clock_t ticks)
{
  if (!g_nano_oneshot_ready)
    {
      return -EAGAIN;
    }

  nano_oneshot_start_absolute(&g_nano_oneshot,
                              nano_tick_to_count((uint64_t)ticks));
  return OK;
}

/****************************************************************************
 * Name: up_alarm_cancel
 *
 * Description:
 *   Stop the alarm and return the current time.  Replaces the weak
 *   function of arch_alarm.c (ONESHOT_CANCEL); the wdog code calls it when
 *   the last watchdog is cancelled.
 *
 ****************************************************************************/

int up_alarm_cancel(FAR struct timespec *ts)
{
  if (!g_nano_oneshot_ready)
    {
      return -EAGAIN;
    }

  nano_oneshot_cancel(&g_nano_oneshot);
  if (ts != NULL)
    {
      up_timer_gettime(ts);
    }

  return OK;
}

/****************************************************************************
 * Name: up_udelay / up_ndelay
 *
 * Description:
 *   Busy-wait on the TIMER.  Replace the weak functions of arch_alarm.c,
 *   which read the time through ONESHOT_CURRENT (a 64-bit division per
 *   reading) or, before the timer is registered, count loops.  The TIMER
 *   runs from rp2040_clockconfig() on, so it can be used from early boot.
 *   The wait ends at the first count at or past the start plus the delay
 *   rounded up to whole microseconds, as in the generic ndelay_accurate().
 *
 ****************************************************************************/

static void nano_delay_counts(uint32_t counts)
{
  uint64_t start = nano_oneshot_count();

  while (nano_oneshot_count() - start < counts)
    {
    }
}

void up_udelay(useconds_t microseconds)
{
  nano_delay_counts(microseconds);
}

void up_ndelay(unsigned long nanoseconds)
{
  uint32_t usec = nanoseconds / NSEC_PER_USEC;

  if (usec * NSEC_PER_USEC != nanoseconds)
    {
      usec++;
    }

  nano_delay_counts(usec);
}
