/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's k_spinlock (<zephyr/spinlock.h>) for the compatibility layer.
 * The RP2040 build of Zephyr runs on one CPU (no CONFIG_SMP), where
 * k_spin_lock() masks interrupts and returns the previous state; with
 * interrupts masked Zephyr also switches to no other thread until the
 * lock is released (a thread made ready, e.g. by k_sem_give(), runs at
 * k_spin_unlock()).  Part of the small Zephyr compatibility layer of the
 * port, its OS-independent part (common/compat, see
 * nuttx/drivers/README.md).
 *
 * The OS part: "zephyr_os_spinlock.h" (nuttx/drivers/compat on NuttX,
 * common/tests/zephyr_fakes in host tests) provides struct k_spinlock,
 * k_spinlock_key_t, k_spin_lock() and k_spin_unlock() with that meaning,
 * in threads and in interrupt handlers, nested in the same thread.
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SPINLOCK_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SPINLOCK_H

#include <stdint.h>

#include "zephyr_os_spinlock.h"

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SPINLOCK_H */
