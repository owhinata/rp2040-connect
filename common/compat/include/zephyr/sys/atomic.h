/* SPDX-License-Identifier: Apache-2.0 */
/* The atomic operations of Zephyr's <zephyr/sys/atomic.h> that the ported
 * code uses, with Zephyr's types and return values.  The Cortex-M0+ has no
 * exclusive loads and stores; Zephyr's ARMv6-M build implements these
 * (CONFIG_ATOMIC_OPERATIONS_C) by masking interrupts around a plain access,
 * and so does this header (k_spin_lock()).  Part of the small Zephyr
 * compatibility layer of the port, its OS-independent part (common/compat).
 *
 * The names are taken over for the file: atomic_t is zephyr_atomic_t.  An
 * OS whose headers define names of their own (NuttX's <nuttx/atomic.h>:
 * atomic_t, atomic_set(), ... with other meanings) reads them in
 * "zephyr_os_atomic.h", included first, and undefines its macros there
 * (common/compat/port/nuttx on NuttX; empty in host tests,
 * common/tests/zephyr_fakes).  A file that uses both must include the OS's
 * headers before any header of the compatibility layer.
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_ATOMIC_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_ATOMIC_H

#include <stdbool.h>

#include "zephyr_os_atomic.h"

#include <zephyr/spinlock.h>

#define atomic_t     zephyr_atomic_t
#define atomic_val_t zephyr_atomic_val_t

typedef long zephyr_atomic_t;
typedef zephyr_atomic_t zephyr_atomic_val_t;

#define ATOMIC_INIT(i) (i)

/* One lock per file is enough (NuttX does not use the lock word, see its
 * zephyr_os_spinlock.h).
 */

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

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_SYS_ATOMIC_H */
