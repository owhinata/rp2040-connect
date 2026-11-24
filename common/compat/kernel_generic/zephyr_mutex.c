/* SPDX-License-Identifier: MIT */
/* k_mutex of <zephyr/kernel.h> for an OS that offers the primitives of
 * zephyr_kernel_os.h: the OS's own recursive mutex (z_os_mutex_init(),
 * z_os_mutex_lock(), z_os_mutex_unlock()), so that priority inheritance
 * is the OS's.  Here only the timeout and Zephyr's results (kernel/mutex.c):
 * K_NO_WAIT on a mutex another thread holds -EBUSY, a timeout -EAGAIN;
 * unlocking a mutex that nobody holds -EINVAL, one that another thread holds
 * -EPERM (z_os_mutex_unlock()).  Part of the generic kernel part of the
 * small Zephyr compatibility layer of the port (NuttX's is
 * common/compat/port/nuttx/zephyr_mutex.c).
 *
 * Zephyr's k_mutex is unlocked when all zero: k_mutex_lock() takes it when
 * lock_count is 0 (kernel/mutex.c), so a mutex in a zeroed static structure
 * works before k_mutex_init().  Zephyr's MQTT library relies on it
 * (mqtt_abort() locks the client's mutex, which only mqtt_client_init()
 * initializes), and so do K_MUTEX_DEFINE() and the settings subsystem's
 * K_MUTEX_DEFINE(settings_lock); the OS's struct k_mutex provides it.  Not
 * from an interrupt handler, as in Zephyr.
 *
 * Priority inheritance is the OS's (FreeRTOS's approximation and ThreadX's:
 * see common/compat/README.md).
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#include "zephyr_kernel_os.h"
#include "zephyr_os_internal.h"

int k_mutex_init(struct k_mutex *mutex)
{
	return z_os_mutex_init(mutex);
}

int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
	z_os_ticks_t ticks = z_wait_ticks(timeout);

	if (z_os_mutex_lock(mutex, ticks)) {
		return 0;
	}
	return ticks == 0 ? -EBUSY : -EAGAIN;
}

int k_mutex_unlock(struct k_mutex *mutex)
{
	return z_os_mutex_unlock(mutex);
}
