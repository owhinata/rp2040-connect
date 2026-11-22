/* SPDX-License-Identifier: Apache-2.0 */
/* The OS's part of the LSM6DSO's core on Zephyr (lsm6dso_os.h): Zephyr's
 * mutex, spin lock, waits, cycle counter, I2C and log.  The core's long
 * reads go to the I2C device of stmemsc's ctx (STMEMSC_CTX_I2C() makes
 * ctx->handle the instance's struct i2c_dt_spec); without an I2C instance
 * they fail with -ENOTSUP, and the binding starts no FIFO on another bus
 * (lsm6dso_rtio_stream.c).  The NuttX and FreeRTOS builds use this file on
 * their Zephyr compatibility layers.
 */

#ifndef LSM6DSO_OS_IMPL_H_
#define LSM6DSO_OS_IMPL_H_

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, i2c) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, i2c)
#include <zephyr/drivers/i2c.h>
#define LSM6DSO_OS_I2C 1
#endif

#include "lsm6dso_reg.h"

typedef struct k_mutex lsm6dso_os_mutex_t;
typedef struct k_spinlock lsm6dso_os_spinlock_t;
typedef k_spinlock_key_t lsm6dso_os_spinlock_key_t;

#define LSM6DSO_OS_LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL

static inline void lsm6dso_os_mutex_init(lsm6dso_os_mutex_t *mutex)
{
	k_mutex_init(mutex);
}

static inline void lsm6dso_os_mutex_lock(lsm6dso_os_mutex_t *mutex)
{
	k_mutex_lock(mutex, K_FOREVER);
}

static inline void lsm6dso_os_mutex_unlock(lsm6dso_os_mutex_t *mutex)
{
	k_mutex_unlock(mutex);
}

static inline lsm6dso_os_spinlock_key_t lsm6dso_os_spin_lock(lsm6dso_os_spinlock_t *lock)
{
	return k_spin_lock(lock);
}

static inline void lsm6dso_os_spin_unlock(lsm6dso_os_spinlock_t *lock,
					  lsm6dso_os_spinlock_key_t key)
{
	k_spin_unlock(lock, key);
}

static inline void lsm6dso_os_busy_wait_us(uint32_t us)
{
	k_busy_wait(us);
}

static inline void lsm6dso_os_msleep(int32_t ms)
{
	k_msleep(ms);
}

static inline void lsm6dso_os_usleep(int32_t us)
{
	k_usleep(us);
}

static inline uint32_t lsm6dso_os_cycle_get_32(void)
{
	return k_cycle_get_32();
}

static inline uint64_t lsm6dso_os_cycles_to_ns(uint32_t cycles)
{
	return k_cyc_to_ns_floor64(cycles);
}

static inline int lsm6dso_os_bus_read(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *buf,
				      size_t len)
{
#ifdef LSM6DSO_OS_I2C
	return i2c_burst_read_dt((const struct i2c_dt_spec *)ctx->handle, reg, buf, len);
#else
	ARG_UNUSED(ctx);
	ARG_UNUSED(reg);
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	return -ENOTSUP;
#endif
}

#endif /* LSM6DSO_OS_IMPL_H_ */
