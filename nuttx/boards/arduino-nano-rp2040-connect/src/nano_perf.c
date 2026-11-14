/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_perf.c
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

/* The perf counter (CONFIG_NANO_PERF, which selects ARCH_HAVE_PERF_EVENTS):
 * up_perf_*() on the RP2040 TIMER, 1 MHz from the crystal, 64 bits.  The
 * Cortex-M0+ has no cycle counter (no DWT), and without ARCH_HAVE_PERF_EVENTS
 * drivers/timers/arch_alarm.c would supply versions that go through the
 * oneshot's current() and a timespec in nanoseconds on every call.
 *
 * The user is the critical section monitor (SCHED_CRITMONITOR with
 * SCHED_CPULOAD_CRITMONITOR): sched/sched/sched_critmonitor.c reads
 * perf_gettime() at each context switch and adds the time since the last
 * switch to the outgoing task (tcb->run_time, /proc/<pid>/critmon) and to
 * its CPU load (/proc/<pid>/loadavg, the CPU column of ps,
 * /proc/cpuload); nuttx/patches/nuttx/0007 keeps that load in these
 * microseconds.  The Zephyr build's thread usage
 * (CONFIG_SCHED_THREAD_USAGE, kernel/usage.c) is the same kind of
 * accounting, at the PendSV switch, in k_cycle_get_32() cycles.
 *
 * nano_mono_us() (include/nano_timer.h) reads the raw registers without a
 * lock, so this is safe in any context: from boot (the TIMER runs from
 * reset; before rp2040_clockconfig() starts its tick it reads a constant),
 * at a switch inside arm_doirq() and in interrupt handlers.  The 64-bit
 * count does not wrap (PERF_OVERFLOW_CORRECTION is not needed).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <arch/board/nano_timer.h>

#ifndef CONFIG_ARCH_HAVE_PERF_EVENTS
#  error "nano_perf.c replaces the up_perf_*() of drivers/timers"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_perf_init(FAR void *arg)
{
  UNUSED(arg);
}

clock_t up_perf_gettime(void)
{
  return (clock_t)nano_mono_us();
}

unsigned long up_perf_getfreq(void)
{
  return USEC_PER_SEC;
}

/* Only for reading (procfs, clock_gettime(CLOCK_THREAD_CPUTIME_ID)). */

void up_perf_convert(clock_t elapsed, FAR struct timespec *ts)
{
  uint64_t us = (uint64_t)elapsed;

  ts->tv_sec  = (time_t)(us / USEC_PER_SEC);
  ts->tv_nsec = (long)(us % USEC_PER_SEC) * NSEC_PER_USEC;
}
