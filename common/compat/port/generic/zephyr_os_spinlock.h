/* SPDX-License-Identifier: MIT */
/* The OS part of <zephyr/spinlock.h> of the Zephyr compatibility layer
 * (common/compat/include/zephyr/spinlock.h) on the primitives of zephyr_kernel_os.h:
 * k_spin_lock() masks interrupts and no other thread runs until
 * k_spin_unlock(), as in Zephyr on one CPU.
 *
 * - In a thread it is z_os_task_lock(), which nests and which the OS calls
 *   made inside (a wake in k_sem_give()) keep; a switch they make due
 *   happens at the unlock, as Zephyr switches at k_spin_unlock().  (On
 *   FreeRTOS it is the kernel's critical section, taskENTER_CRITICAL(): a
 *   plain PRIMASK lock would not do, since the FreeRTOS calls made inside
 *   leave their own critical section with taskEXIT_CRITICAL(), which enables
 *   interrupts when its nesting count drops to 0.)
 * - In an interrupt handler it is z_os_isr_lock(), which saves the
 *   interrupt mask and restores it.
 *
 * The key says which.  Nested locks in the same context work.  A thread
 * must not hold interrupts disabled by other means around k_spin_lock():
 * the unlock may enable them.  The lock word is not used.
 */

#ifndef COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_SPINLOCK_H
#define COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_SPINLOCK_H

#include <stdint.h>

#include "zephyr_kernel_os.h"

struct k_spinlock {
	uint8_t unused;
};

typedef uint32_t k_spinlock_key_t;

/* The key of a lock taken in a thread; in an interrupt handler the key is
 * the state z_os_isr_lock() saved, never this value.
 */
#define Z_SPIN_KEY_TASK 0xffffffffu

static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *l)
{
	(void)l;

	if (z_os_in_isr()) {
		return z_os_isr_lock();
	}
	z_os_task_lock();
	return Z_SPIN_KEY_TASK;
}

static inline void k_spin_unlock(struct k_spinlock *l, k_spinlock_key_t key)
{
	(void)l;

	if (key == Z_SPIN_KEY_TASK) {
		z_os_task_unlock();
	} else {
		z_os_isr_unlock(key);
	}
}

#endif /* COMMON_COMPAT_PORT_GENERIC_ZEPHYR_OS_SPINLOCK_H */
