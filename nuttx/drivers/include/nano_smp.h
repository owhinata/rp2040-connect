/* SPDX-License-Identifier: Apache-2.0 */
/* What the port's own code uses where it relied on "one CPU": masking
 * interrupts excludes every interrupt handler and every other thread.
 *
 * With CONFIG_SMP the RP2040 port of NuttX runs every device interrupt on
 * core 0 (arch/arm/src/rp2040/rp2040_irq.c) and threads on both cores, so
 * up_irq_save() on one core excludes neither a thread on the other core nor
 * (from core 1) an interrupt handler.  The same exclusion is then NuttX's
 * critical section (enter_critical_section(): one lock for both cores,
 * recursive, usable in interrupt handlers), taken by
 *
 *   - nano_irq_save()/nano_irq_restore(), in place of up_irq_save()/
 *     up_irq_restore() around shared data,
 *   - the compatibility layer's k_spin_lock() (compat/zephyr/spinlock.h),
 *   - the port's interrupt handlers for their whole run (NANO_ISR_SMP()),
 *     which on one CPU run with nothing else in between.
 *
 * Without CONFIG_SMP everything here is what it was: up_irq_save(),
 * up_irq_restore() and the handler itself.
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_NANO_SMP_H
#define __NUTTX_DRIVERS_INCLUDE_NANO_SMP_H

#include <nuttx/config.h>
#include <nuttx/irq.h>

#ifdef CONFIG_SMP
#  define nano_irq_save()         enter_critical_section()
#  define nano_irq_restore(flags) leave_critical_section(flags)

/* Defines handler##_smp(), to be attached in place of handler. */

#  define NANO_ISR_SMP(handler) \
  static int handler##_smp(int irq, void *context, void *arg) \
  { \
    irqstate_t flags = enter_critical_section(); \
    int ret = handler(irq, context, arg); \
    leave_critical_section(flags); \
    return ret; \
  }
#  define NANO_ISR(handler)       handler##_smp
#else
#  define nano_irq_save()         up_irq_save()
#  define nano_irq_restore(flags) up_irq_restore(flags)
#  define NANO_ISR_SMP(handler)
#  define NANO_ISR(handler)       handler
#endif

#endif /* __NUTTX_DRIVERS_INCLUDE_NANO_SMP_H */
