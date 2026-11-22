/* SPDX-License-Identifier: Apache-2.0 */
/* The OS's part of the LSM6DSO's core (common/drivers/lsm6dso/port/lsm6dso_os.h) on the host's C
 * library alone, for the check that the core needs nothing else (common/tests/run.sh,
 * lsm6dso_core_independent): one thread, no interrupts, waits that return at once, a cycle counter
 * that stands still, a bus without a device and a log that prints nothing.
 */
#ifndef LSM6DSO_OS_IMPL_H_
#define LSM6DSO_OS_IMPL_H_

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include "lsm6dso_reg.h"

typedef struct {
	int depth;
} lsm6dso_os_mutex_t;
typedef struct {
	int locked;
} lsm6dso_os_spinlock_t;
typedef int lsm6dso_os_spinlock_key_t;

#define LSM6DSO_OS_LOG_LEVEL 3
#define LOG_MODULE_DECLARE(name, level) extern int lsm6dso_os_host_log_unused
#define LOG_ERR(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define LOG_DBG(...) ((void)0)

static inline void lsm6dso_os_mutex_init(lsm6dso_os_mutex_t *mutex)
{
	mutex->depth = 0;
}

static inline void lsm6dso_os_mutex_lock(lsm6dso_os_mutex_t *mutex)
{
	mutex->depth++;
}

static inline void lsm6dso_os_mutex_unlock(lsm6dso_os_mutex_t *mutex)
{
	mutex->depth--;
}

static inline lsm6dso_os_spinlock_key_t lsm6dso_os_spin_lock(lsm6dso_os_spinlock_t *lock)
{
	lock->locked = 1;
	return 0;
}

static inline void lsm6dso_os_spin_unlock(lsm6dso_os_spinlock_t *lock,
					  lsm6dso_os_spinlock_key_t key)
{
	(void)key;
	lock->locked = 0;
}

static inline void lsm6dso_os_busy_wait_us(uint32_t us)
{
	(void)us;
}

static inline void lsm6dso_os_msleep(int32_t ms)
{
	(void)ms;
}

static inline void lsm6dso_os_usleep(int32_t us)
{
	(void)us;
}

static inline uint32_t lsm6dso_os_cycle_get_32(void)
{
	return 0;
}

static inline uint64_t lsm6dso_os_cycles_to_ns(uint32_t cycles)
{
	return cycles;
}

static inline int lsm6dso_os_bus_read(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *buf,
				      size_t len)
{
	(void)ctx;
	(void)reg;
	(void)buf;
	(void)len;
	return -ENODEV;
}

#endif /* LSM6DSO_OS_IMPL_H_ */
