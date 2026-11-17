/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of <zephyr/spinlock.h> of the Zephyr compatibility layer
 * (common/compat/zephyr/spinlock.h): k_spin_lock() masks interrupts and no
 * other task runs until k_spin_unlock(), as in Zephyr on one CPU.
 *
 * - In a task it is the kernel's critical section (z_os_task_lock(),
 *   taskENTER_CRITICAL() on the board).  A plain PRIMASK lock would not do:
 *   the FreeRTOS calls made inside (a task notification in k_sem_give())
 *   leave their own critical section with taskEXIT_CRITICAL(), which enables
 *   interrupts when its nesting count drops to 0.  A switch that they ask for
 *   is a pending PendSV, taken when the lock is released, as Zephyr switches
 *   at k_spin_unlock().
 * - In an interrupt handler it is taskENTER_CRITICAL_FROM_ISR(), which saves
 *   PRIMASK and restores it.
 *
 * The key says which.  Nested locks in the same context work.  A task must
 * not hold interrupts disabled by other means (save_and_disable_interrupts())
 * around k_spin_lock(): the unlock would enable them.  The lock word is not
 * used.  Part of the FreeRTOS implementation of the layer
 * (freertos/drivers/compat).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_SPINLOCK_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_SPINLOCK_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "zephyr_os_port.h"

struct k_spinlock {
	uint8_t unused;
};

typedef uint32_t k_spinlock_key_t;

/* The key of a lock taken in a task; in an interrupt handler the key is the
 * saved interrupt mask (0 or 1 on the board).
 */
#define Z_SPIN_KEY_TASK 0xffffffffu

static inline k_spinlock_key_t k_spin_lock(struct k_spinlock *l)
{
	(void)l;

	if (z_os_in_isr()) {
		return (k_spinlock_key_t)taskENTER_CRITICAL_FROM_ISR();
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
		taskEXIT_CRITICAL_FROM_ISR((UBaseType_t)key);
	}
}

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_OS_SPINLOCK_H */
