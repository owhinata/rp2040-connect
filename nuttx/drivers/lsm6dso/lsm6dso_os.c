/* SPDX-License-Identifier: Apache-2.0 */
/* The kernel services of lsm6dso.c on NuttX (see lsm6dso_os.h).
 *
 * Zephyr build (drivers/lsm6dso/lsm6dso.c):
 * - fifo_lock, zero_lock: k_mutex (recursive, priority inheritance).  Here
 *   rmutex_t; the wait is repeated when a signal ends it (-EINTR), so that
 *   it waits for ever as K_FOREVER does.
 * - cal_lock: k_spinlock, which on one CPU only disables interrupts.  Here
 *   up_irq_save()/up_irq_restore().
 * - k_busy_wait(): up_udelay(), which polls the TIMER (the board's
 *   nano_oneshot.c).
 * - k_msleep(), k_usleep(): Zephyr rounds a relative timeout up to ticks
 *   and adds one tick (kernel/timeout.c z_add_timeout()); nxsched_msleep()
 *   and nxsched_usleep() round up and wake at the current tick + ticks + 1
 *   (sched/clock clock_delay2abstick()): the same with the same 100 us
 *   tick.  Neither is ended by a signal.
 * - k_cycle_get_32(): the RP2040 TIMER's low word (TIMERAWL, 1 MHz) in
 *   place of SysTick's 125 MHz count, so k_cyc_to_ns_floor64() multiplies
 *   by 1000.
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/sched.h>
#include <arch/board/nano_timer.h>

#include "lsm6dso_os.h"

void lsm6dso_os_mutex_init(lsm6dso_mutex_t *mutex)
{
  nxrmutex_init(mutex);
}

void lsm6dso_os_mutex_lock(lsm6dso_mutex_t *mutex)
{
  int ret;

  do
    {
      ret = nxrmutex_lock(mutex);
    }
  while (ret == -EINTR);
}

void lsm6dso_os_mutex_unlock(lsm6dso_mutex_t *mutex)
{
  nxrmutex_unlock(mutex);
}

lsm6dso_spinlock_key_t lsm6dso_os_spin_lock(lsm6dso_spinlock_t *lock)
{
  (void)lock;
  return up_irq_save();
}

void lsm6dso_os_spin_unlock(lsm6dso_spinlock_t *lock,
                            lsm6dso_spinlock_key_t key)
{
  (void)lock;
  up_irq_restore(key);
}

void lsm6dso_os_busy_wait_us(uint32_t us)
{
  up_udelay(us);
}

void lsm6dso_os_sleep_ms(int32_t ms)
{
  if (ms > 0)
    {
      nxsched_msleep((unsigned int)ms);
    }
}

void lsm6dso_os_sleep_us(int32_t us)
{
  if (us > 0)
    {
      nxsched_usleep((useconds_t)us);
    }
}

uint32_t lsm6dso_os_cycle_get_32(void)
{
  return *(volatile uint32_t *)NANO_TIMER_TIMERAWL;
}

uint64_t lsm6dso_os_cyc_to_ns_floor64(uint32_t cycles)
{
  return (uint64_t)cycles * 1000U;
}
