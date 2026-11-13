/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_stackcheck.c
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

/* The stack sentinel (CONFIG_NANO_STACK_SENTINEL): the counterpart of the
 * Zephyr build's CONFIG_STACK_SENTINEL.  Zephyr writes a magic word at the
 * lowest address of each thread stack and checks it in z_swap() for the
 * outgoing thread and at the end of every non-nested interrupt for the
 * current thread, i.e. the one the interrupt interrupted
 * (kernel/thread.c z_check_stack_sentinel(), kernel/include/kswap.h,
 * arch/arm/core/cortex_m/exc_exit.c); a changed word is a fatal error.
 *
 * Here the word is the lowest word of the stack as STACK_COLORATION fills
 * it (STACK_COLOR, 0xdeadbeef, at the aligned stack_base_ptr: the same word
 * up_check_tcbstack() starts from).  It is checked from two scheduler
 * instrumentation hooks (no note driver is built):
 *
 * - sched_note_suspend(): nxsched_switch_context() calls it for the
 *   outgoing task.  On the Cortex-M0+ that happens in arm_doirq()
 *   (arch/arm/src/armv6-m/arm_doirq.c), which only the exceptions up to
 *   PendSV go through (SVC for a switch from a thread, PendSV for a switch
 *   requested by an interrupt; arm_vectors.c), so this is the check at a
 *   switch.
 * - sched_note_irqhandler(): irq_dispatch() calls it before and after each
 *   handler, for every interrupt (peripheral interrupts and SysTick go
 *   through exception_direct(), which does not reach arm_doirq()).  After
 *   the handler (enter == false) the task in g_running_tasks[] is checked:
 *   the task that was running when the interrupt came, whose stack the
 *   handler ran on (NuttX has no interrupt stack on this chip), like
 *   Zephyr's _current at the end of an interrupt.  this_task() may already
 *   be a task the handler woke, which is switched to only afterwards.
 *   The Cortex-M0+ port runs all interrupts at one priority, so they do not
 *   nest and every handler exit is the end of a non-nested interrupt.
 *
 * CONFIG_STACKCHECK_MARGIN would check at a switch only, and needs
 * CONFIG_DEBUG_ASSERTIONS, which turns on every DEBUGASSERT in the OS; the
 * Zephyr build has CONFIG_ASSERT off.
 *
 * On a changed word, the word is restored (as Zephyr does, so further
 * checks do not trip on it again) and the system halts through PANIC(),
 * whose report (with the running task's name) goes to the RAM log (Zephyr:
 * fatal error, then halt; CONFIG_RESET_ON_FATAL_ERROR is off there).  The
 * report runs on the same, already overrun stack: there is no other.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>

#include "arm_internal.h"

#if !defined(CONFIG_STACK_COLORATION) || \
    !defined(CONFIG_SCHED_INSTRUMENTATION_SWITCH) || \
    !defined(CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER)
#  error "nano_stackcheck.c needs STACK_COLORATION and both hooks"
#endif

#ifdef CONFIG_SMP
#  error "nano_stackcheck.c checks one CPU"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void nano_check_sentinel(FAR struct tcb_s *tcb)
{
  FAR uint32_t *sentinel;

  if (tcb == NULL || tcb->stack_base_ptr == NULL)
    {
      return;
    }

  sentinel = (FAR uint32_t *)
             STACKFRAME_ALIGN_UP((uintptr_t)tcb->stack_base_ptr);
  if (*sentinel != STACK_COLOR)
    {
      *sentinel = STACK_COLOR;
      PANIC();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* The scheduler instrumentation hooks (include/nuttx/sched_note.h).  Only
 * the switch-out and interrupt-handler hooks do anything; there is no note
 * driver (DRIVERS_NOTE).
 */

void sched_note_add(FAR const void *note, size_t notelen)
{
  UNUSED(note);
  UNUSED(notelen);
}

void sched_note_start(FAR struct tcb_s *tcb)
{
  UNUSED(tcb);
}

void sched_note_stop(FAR struct tcb_s *tcb)
{
  UNUSED(tcb);
}

void sched_note_resume(FAR struct tcb_s *tcb)
{
  UNUSED(tcb);
}

/* At a switch: the outgoing task. */

void sched_note_suspend(FAR struct tcb_s *tcb)
{
  nano_check_sentinel(tcb);
}

/* At the end of each interrupt handler: the interrupted task. */

void sched_note_irqhandler(int irq, FAR void *handler, bool enter)
{
  UNUSED(irq);
  UNUSED(handler);

  if (!enter)
    {
      nano_check_sentinel(g_running_tasks[0]);
    }
}
