/* SPDX-License-Identifier: Apache-2.0 */
/* The kernel services of lsm6dso.c on NuttX (see lsm6dso_os.h).
 *
 * Zephyr build (drivers/lsm6dso/lsm6dso.c):
 * - fifo_lock, zero_lock: k_mutex (recursive, priority inheritance).  Here
 *   the compatibility layer's k_mutex (an rmutex_t, <zephyr/kernel.h>),
 *   locked with K_FOREVER: a wait ended by a signal is repeated.
 * - cal_lock: k_spinlock, which on one CPU only disables interrupts; the
 *   compatibility layer's k_spin_lock() (up_irq_save()).
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
#include <nuttx/sched.h>
#include <arch/board/nano_timer.h>

#include "lsm6dso_os.h"

void lsm6dso_os_mutex_init(lsm6dso_mutex_t *mutex)
{
  (void)k_mutex_init(mutex);
}

void lsm6dso_os_mutex_lock(lsm6dso_mutex_t *mutex)
{
  (void)k_mutex_lock(mutex, K_FOREVER);
}

void lsm6dso_os_mutex_unlock(lsm6dso_mutex_t *mutex)
{
  (void)k_mutex_unlock(mutex);
}

lsm6dso_spinlock_key_t lsm6dso_os_spin_lock(lsm6dso_spinlock_t *lock)
{
  return k_spin_lock(lock);
}

void lsm6dso_os_spin_unlock(lsm6dso_spinlock_t *lock,
                            lsm6dso_spinlock_key_t key)
{
  k_spin_unlock(lock, key);
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
