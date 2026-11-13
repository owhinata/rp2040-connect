/* SPDX-License-Identifier: Apache-2.0 */
/* The atomic operations of Zephyr's <zephyr/sys/atomic.h> that the ported
 * code uses, with Zephyr's types and return values.  The Cortex-M0+ has no
 * exclusive loads and stores; Zephyr's ARMv6-M build implements these
 * (CONFIG_ATOMIC_OPERATIONS_C) by masking interrupts around a plain access,
 * and so does this header (k_spin_lock()).  Part of the small Zephyr
 * compatibility layer of the port (nuttx/drivers/compat).
 *
 * NuttX's <nuttx/atomic.h> (pulled in by most kernel headers) has its own
 * atomic_t and atomic_set(), atomic_add(), atomic_or()... with other
 * meanings.  This header includes it first and then takes the names over
 * for the file: atomic_t is zephyr_atomic_t and NuttX's macros are
 * undefined.  A file that uses both must include the NuttX headers before
 * any header of the compatibility layer.
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_ATOMIC_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_ATOMIC_H

#include <stdbool.h>

#ifdef __NuttX__
#  include <nuttx/config.h>
#  include <nuttx/atomic.h>
#  undef atomic_set
#  undef atomic_add
#  undef atomic_sub
#  undef atomic_and
#  undef atomic_or
#  undef atomic_xor
#endif

#include <zephyr/spinlock.h>

#define atomic_t     zephyr_atomic_t
#define atomic_val_t zephyr_atomic_val_t

typedef long zephyr_atomic_t;
typedef zephyr_atomic_t zephyr_atomic_val_t;

#define ATOMIC_INIT(i) (i)

/* The lock word is not used (see spinlock.h); one per file is enough. */

static struct k_spinlock z_atomic_lock __attribute__((unused));

static inline atomic_val_t atomic_get(const atomic_t *target)
{
  return *(const volatile atomic_t *)target;
}

/* Each returns the previous value, as Zephyr's. */

static inline atomic_val_t atomic_set(atomic_t *target, atomic_val_t value)
{
  k_spinlock_key_t key = k_spin_lock(&z_atomic_lock);
  atomic_val_t old = *target;

  *target = value;
  k_spin_unlock(&z_atomic_lock, key);
  return old;
}

static inline atomic_val_t atomic_clear(atomic_t *target)
{
  return atomic_set(target, 0);
}

static inline bool atomic_cas(atomic_t *target, atomic_val_t old_value,
                              atomic_val_t new_value)
{
  k_spinlock_key_t key = k_spin_lock(&z_atomic_lock);
  bool ret = *target == old_value;

  if (ret)
    {
      *target = new_value;
    }

  k_spin_unlock(&z_atomic_lock, key);
  return ret;
}

static inline atomic_val_t atomic_add(atomic_t *target, atomic_val_t value)
{
  k_spinlock_key_t key = k_spin_lock(&z_atomic_lock);
  atomic_val_t old = *target;

  *target = old + value;
  k_spin_unlock(&z_atomic_lock, key);
  return old;
}

static inline atomic_val_t atomic_inc(atomic_t *target)
{
  return atomic_add(target, 1);
}

static inline atomic_val_t atomic_or(atomic_t *target, atomic_val_t value)
{
  k_spinlock_key_t key = k_spin_lock(&z_atomic_lock);
  atomic_val_t old = *target;

  *target = old | value;
  k_spin_unlock(&z_atomic_lock, key);
  return old;
}

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_ATOMIC_H */
