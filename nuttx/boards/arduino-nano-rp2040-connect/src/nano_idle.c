/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_idle.c
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

/* The idle loop body (CONFIG_ARCH_IDLE_CUSTOM; nuttx/patches/nuttx/0004
 * leaves arch/arm/src/rp2040/rp2040_idle.c out) and the CPU load measured
 * from idle windows, cpu_load_get().
 *
 * The same measurement as the Zephyr build (CONFIG_CPU_LOAD with
 * CONFIG_CPU_LOAD_BACKEND_IDLE_HOOK): Zephyr's idle thread locks interrupts
 * (PRIMASK on the Cortex-M0+), arch_cpu_idle() calls sys_trace_idle(),
 * which opens the window, waits with WFI, calls sys_trace_idle_exit(),
 * which closes it, and only then unlocks interrupts, so the handler of the
 * interrupt that woke the CPU runs outside the window and counts as load
 * (zephyr/zephyr/kernel/idle.c, arch/arm/core/cortex_m/cpu_idle.c,
 * subsys/tracing/tracing_none.c, lib/os/cpu_load/cpu_load_backend_idle.c).
 * up_idle() does the same.  WFI wakes on a pending interrupt even while
 * PRIMASK masks it (ARMv6-M), so an interrupt that arrives between
 * disabling interrupts and WFI is not missed: WFI returns at once.
 *
 * What rp2040_idle.c did: WFI, and the idle LED when CONFIG_ARCH_LEDS (not
 * used on this board).  Its CONFIG_SUPPRESS_INTERRUPTS /
 * SUPPRESS_TIMER_INTS branch (calling nxsched_process_timer()) is not
 * needed: the tickless timer interrupt (nano_oneshot.c) drives the
 * scheduler, and NuttX 13.1.0's idle loop needs nothing else
 * (sched/init/nx_start.c only calls up_idle() in a loop).
 *
 * Time base: the TIMER, 1 us (Zephyr: k_cycle_get_32(), CPU cycles at
 * 125 MHz, converted to microseconds rounding down in cpu_load_get()).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/board/nano_timer.h>

#include "cpu_load.h"
#include "nano_cpuload.h"

#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
#  error "nano_idle.c needs the timer interrupt"
#endif

#ifdef CONFIG_SMP
#  error "nano_idle.c measures one CPU"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Changed only with interrupts disabled. */

static struct nano_cpuload_s g_cpuload;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   Called in a loop by the idle task with interrupts enabled.  Waits for
 *   an interrupt and counts the wait as idle time.
 *
 ****************************************************************************/

void up_idle(void)
{
  irqstate_t flags = up_irq_save();

  nano_cpuload_enter(&g_cpuload, nano_mono_us());

  /* Zephyr: __DSB() then __WFI() (SLEEP_IF_ALLOWED). */

  __asm__ __volatile__("dsb\n\twfi" ::: "memory");

  nano_cpuload_exit(&g_cpuload, nano_mono_us());

  /* The pending interrupt is taken here, outside the window. */

  up_irq_restore(flags);
}

/****************************************************************************
 * Name: cpu_load_get
 *
 * Description:
 *   See cpu_load.h.  Zephyr's cpu_load_get(): reads the time before taking
 *   the lock, then the window state under it.
 *
 ****************************************************************************/

int cpu_load_get(bool reset)
{
  uint64_t now = nano_mono_us();
  uint64_t total;
  uint64_t idle;
  irqstate_t flags = up_irq_save();

  /* On one CPU the window is never open here (the caller is not idle);
   * the second reading is kept for the same steps as Zephyr's.
   */

  nano_cpuload_sample(&g_cpuload, now,
                      g_cpuload.open ? nano_mono_us() : now, reset,
                      &total, &idle);
  up_irq_restore(flags);

  /* The division (64-bit, libgcc on the M0+) runs with interrupts
   * enabled, as in Zephyr.
   */

  return nano_cpuload_ratio(total, idle);
}
