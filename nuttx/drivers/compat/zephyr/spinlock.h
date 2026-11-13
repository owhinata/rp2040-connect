/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's k_spinlock (<zephyr/spinlock.h>) for the compatibility layer.
 * The RP2040 build of Zephyr runs on one CPU (no CONFIG_SMP), where
 * k_spin_lock() masks interrupts and returns the previous state; with
 * interrupts masked Zephyr also switches to no other thread until the
 * lock is released (a thread made ready, e.g. by k_sem_give(), runs at
 * k_spin_unlock()).  NuttX switches at once to a thread that nxsem_post()
 * makes ready, even with interrupts masked, so here the lock is
 * sched_lock() and up_irq_save(): the switch waits for k_spin_unlock()
 * (sched_unlock()), as in Zephyr.  In an interrupt handler sched_lock()
 * does nothing (NuttX switches at the end of the interrupt, as Zephyr
 * does).  The lock word is not used.  Part of the small Zephyr
 * compatibility layer of the port (nuttx/drivers/compat, see
 * nuttx/drivers/README.md).
 *
 * A host test (not __NuttX__) provides struct k_spinlock, k_spinlock_key_t,
 * k_spin_lock() and k_spin_unlock() in its "zephyr_os_host.h".
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SPINLOCK_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SPINLOCK_H

#include <stdint.h>

#ifdef __NuttX__
#  include <sched.h>
#  include <nuttx/irq.h>

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
#else
#  include "zephyr_os_host.h"
#endif

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SPINLOCK_H */
