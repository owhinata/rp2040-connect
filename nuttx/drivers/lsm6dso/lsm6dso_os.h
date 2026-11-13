/* SPDX-License-Identifier: Apache-2.0 */
/* The kernel services that lsm6dso.c uses, in the terms of the Zephyr
 * build's drivers/lsm6dso/lsm6dso.c.  lsm6dso_os.c implements them on
 * NuttX; a host test can build lsm6dso.c with fakes.  The kernel objects
 * are those of the port's Zephyr compatibility layer (<zephyr/kernel.h>),
 * which the FIFO stream (lsm6dso_stream.c, an unchanged copy) also locks
 * by their Zephyr names.
 *
 *   Zephyr                                    here
 *   struct k_mutex, k_mutex_init()            lsm6dso_mutex_t,
 *                                             lsm6dso_os_mutex_init()
 *   k_mutex_lock(.., K_FOREVER), _unlock()    lsm6dso_os_mutex_lock(),
 *                                             _unlock()
 *   struct k_spinlock, k_spin_lock(), _unlock lsm6dso_spinlock_t,
 *                                             lsm6dso_os_spin_lock(), _unlock
 *   k_busy_wait(us)                           lsm6dso_os_busy_wait_us()
 *   k_msleep(ms), k_usleep(us)                lsm6dso_os_sleep_ms(), _us()
 *   k_cycle_get_32(), k_cyc_to_ns_floor64()   lsm6dso_os_cycle_get_32(),
 *                                             lsm6dso_os_cyc_to_ns_floor64()
 */

#ifndef __NUTTX_DRIVERS_LSM6DSO_LSM6DSO_OS_H
#define __NUTTX_DRIVERS_LSM6DSO_LSM6DSO_OS_H

#include <stdint.h>

#include <zephyr/kernel.h>

/* Zephyr's k_mutex: recursive for its owner, priority inheritance. */

typedef struct k_mutex lsm6dso_mutex_t;

/* Zephyr's k_spinlock on one CPU: interrupts off. */

typedef struct k_spinlock lsm6dso_spinlock_t;
typedef k_spinlock_key_t lsm6dso_spinlock_key_t;

void lsm6dso_os_mutex_init(lsm6dso_mutex_t *mutex);

/* Waits for ever (K_FOREVER): a wait ended by a signal is repeated. */

void lsm6dso_os_mutex_lock(lsm6dso_mutex_t *mutex);
void lsm6dso_os_mutex_unlock(lsm6dso_mutex_t *mutex);

lsm6dso_spinlock_key_t lsm6dso_os_spin_lock(lsm6dso_spinlock_t *lock);
void lsm6dso_os_spin_unlock(lsm6dso_spinlock_t *lock,
                            lsm6dso_spinlock_key_t key);

/* k_busy_wait(): spins for at least us microseconds. */

void lsm6dso_os_busy_wait_us(uint32_t us);

/* k_msleep(), k_usleep(): the time rounded up to ticks plus one tick (both
 * kernels' rule for a relative timeout, with the same 100 us tick).  Not
 * ended by signals.
 */

void lsm6dso_os_sleep_ms(int32_t ms);
void lsm6dso_os_sleep_us(int32_t us);

/* k_cycle_get_32(), k_cyc_to_ns_floor64(): a free-running 32-bit counter
 * and its count in nanoseconds.  Zephyr's counts the CPU clock (SysTick,
 * 125 MHz); here it is the RP2040 TIMER (1 MHz, the board's monotonic
 * clock), so a time is a whole number of microseconds.
 */

uint32_t lsm6dso_os_cycle_get_32(void);
uint64_t lsm6dso_os_cyc_to_ns_floor64(uint32_t cycles);

#endif /* __NUTTX_DRIVERS_LSM6DSO_LSM6DSO_OS_H */
