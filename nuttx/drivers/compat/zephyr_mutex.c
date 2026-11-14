/* SPDX-License-Identifier: Apache-2.0 */
/* k_mutex of <zephyr/kernel.h> on NuttX: a recursive mutex (rmutex_t) with
 * priority inheritance.  Part of the small Zephyr compatibility layer of
 * the port (nuttx/drivers/compat).
 *
 * Zephyr's k_mutex is unlocked when all zero: k_mutex_lock() takes it
 * when lock_count is 0 (kernel/mutex.c), so a mutex in a zeroed static
 * structure works before k_mutex_init().  Zephyr's MQTT library relies on
 * it: mqtt_abort() locks the client's mutex, which only mqtt_client_init()
 * initializes, and imu_raw_mqtt.c aborts after every attempt, also one
 * that ended before mqtt_client_init() (no default gateway yet).  NuttX's
 * all-zero mutex is held by thread 0 (mholder 0), so that lock waited for
 * ever.  Here a mutex that Z_MUTEX_INITIALIZER() or k_mutex_init() did not
 * set up (READY clear) is initialized by its first lock or unlock, under
 * the layer's spin lock (the scheduler locked and interrupts off), so two
 * first users cannot both initialize it.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/mutex.h>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

static struct k_spinlock g_init_lock;

static rmutex_t *mutex_of(struct k_mutex *mutex)
{
  k_spinlock_key_t key;

  if (!mutex->ready)
    {
      key = k_spin_lock(&g_init_lock);
      if (!mutex->ready)
        {
          (void)nxrmutex_init(&mutex->rmutex);
          mutex->ready = true;
        }

      k_spin_unlock(&g_init_lock, key);
    }

  return &mutex->rmutex;
}

int k_mutex_init(struct k_mutex *mutex)
{
  int ret = nxrmutex_init(&mutex->rmutex);

  mutex->ready = true;
  return ret;
}

int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
  rmutex_t *rmutex = mutex_of(mutex);
  int ret;

  if (K_TIMEOUT_EQ(timeout, K_NO_WAIT))
    {
      ret = nxrmutex_trylock(rmutex);
      return ret == 0 ? 0 : -EBUSY;
    }

  if (K_TIMEOUT_EQ(timeout, K_FOREVER))
    {
      do
        {
          ret = nxrmutex_lock(rmutex);
        }
      while (ret == -EINTR);

      return ret;
    }

  ret = nxrmutex_ticklock(rmutex, (uint32_t)timeout.ticks);
  return ret == -ETIMEDOUT ? -EAGAIN : ret;
}

int k_mutex_unlock(struct k_mutex *mutex)
{
  return nxrmutex_unlock(mutex_of(mutex));
}
