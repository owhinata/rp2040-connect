/* SPDX-License-Identifier: Apache-2.0 */
/* The NuttX part of <zephyr/spinlock.h> of the Zephyr compatibility layer
 * (common/compat/include/zephyr/spinlock.h).  NuttX switches at once to a
 * thread that nxsem_post() makes ready, even with interrupts masked, so here the
 * lock is sched_lock() and up_irq_save(): the switch waits for
 * k_spin_unlock() (sched_unlock()), as in Zephyr.  In an interrupt handler
 * sched_lock() does nothing (NuttX switches at the end of the interrupt,
 * as Zephyr does).  The lock word is not used.  Part of the NuttX
 * implementation of the layer (common/compat/port/nuttx, see
 * nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_OS_SPINLOCK_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_OS_SPINLOCK_H

#include <sched.h>
#include <nuttx/irq.h>

struct k_spinlock
{
  uint8_t unused;
};

typedef irqstate_t k_spinlock_key_t;

static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *l)
{
  (void)l;
  sched_lock();
  return up_irq_save();
}

static inline void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t key)
{
  (void)l;
  up_irq_restore(key);
  sched_unlock();
}

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_OS_SPINLOCK_H */
