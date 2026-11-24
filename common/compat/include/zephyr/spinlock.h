/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's k_spinlock (<zephyr/spinlock.h>) for the compatibility layer.
 * The RP2040 build of Zephyr runs on one CPU (no CONFIG_SMP), where
 * k_spin_lock() masks interrupts and returns the previous state; with
 * interrupts masked Zephyr also switches to no other thread until the
 * lock is released (a thread made ready, e.g. by k_sem_give(), runs at
 * k_spin_unlock()).  Part of the small Zephyr compatibility layer of the
 * port, its OS-independent part (common/compat, see
 * common/compat/README.md).
 *
 * The OS part: "zephyr_os_spinlock.h" (common/compat/port/nuttx on NuttX,
 * common/tests/zephyr_fakes in host tests, common/compat/port/generic for
 * an OS on basic primitives) provides struct k_spinlock,
 * k_spinlock_key_t, k_spin_lock() and k_spin_unlock() with that meaning,
 * in threads and in interrupt handlers, nested in the same thread.
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_SPINLOCK_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_SPINLOCK_H

#include <stdint.h>

#include "zephyr_os_spinlock.h"

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_SPINLOCK_H */
