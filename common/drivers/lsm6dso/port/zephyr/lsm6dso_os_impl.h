/* SPDX-License-Identifier: Apache-2.0 */
/* The OS's part of the LSM6DSO's core on Zephyr (lsm6dso_os.h): Zephyr's
 * mutex, spin lock, waits, cycle counter, I2C, delayable work, GPIO, sensor
 * clock and log, with lsm6dso_os.c (the stream's work queue and the INT1
 * line's interrupt).  The core's long reads go to the I2C device of
 * stmemsc's ctx (STMEMSC_CTX_I2C() makes ctx->handle the instance's struct
 * i2c_dt_spec); without an I2C instance they fail with -ENOTSUP, and the
 * stream starts on no other bus.  The OS's binding (lsm6dso_sensor.c) sets
 * the INT1 line with lsm6dso_os_int1_init().  The NuttX and FreeRTOS builds
 * use this file on their Zephyr compatibility layers.
 */

#ifndef LSM6DSO_OS_IMPL_H_
#define LSM6DSO_OS_IMPL_H_

#include <errno.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso, i2c) || \
	DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lsm6dso32, i2c)
#include <zephyr/drivers/i2c.h>
#include <stmemsc.h>
#define LSM6DSO_OS_I2C 1
#endif

#include "lsm6dso_reg.h"

typedef struct k_mutex lsm6dso_os_mutex_t;
typedef struct k_spinlock lsm6dso_os_spinlock_t;
typedef k_spinlock_key_t lsm6dso_os_spinlock_key_t;
typedef struct k_work_delayable lsm6dso_os_work_t;

/* INT1: the GPIO interrupt (lsm6dso_os.c) defers to the system work queue,
 * which calls lsm6dso_stream_int1() while enabled is set.
 */
typedef struct lsm6dso_os_int1 {
	struct gpio_dt_spec gpio;
	struct gpio_callback callback;
	struct k_work work;
	atomic_t enabled;
} lsm6dso_os_int1_t;

#define LSM6DSO_OS_LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL

/* The core's functions that this part calls (lsm6dso_stream.c). */
void lsm6dso_stream_run(lsm6dso_os_work_t *work);
int lsm6dso_stream_int1(lsm6dso_os_int1_t *int1);

/* lsm6dso_os.c: the stream's work queue, started once. */
extern struct k_work_q lsm6dso_os_work_q;
void lsm6dso_os_work_q_start(void);

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

static inline bool lsm6dso_os_bus_read_supported(const stmdev_ctx_t *ctx)
{
#ifdef LSM6DSO_OS_I2C
	/* Through void (*)(void), which matches any function type. */
	return ctx->read_reg == (stmdev_read_ptr)(void (*)(void))stmemsc_i2c_read;
#else
	ARG_UNUSED(ctx);
	return false;
#endif
}

/* The work item's handler: emitted where lsm6dso_os_work_init() is used,
 * next to lsm6dso_stream_run().
 */
static inline void lsm6dso_os_work_handler(struct k_work *work)
{
	lsm6dso_stream_run(k_work_delayable_from_work(work));
}

static inline void lsm6dso_os_work_init(lsm6dso_os_work_t *work)
{
	lsm6dso_os_work_q_start();
	k_work_init_delayable(work, lsm6dso_os_work_handler);
}

/* Inlined in every place, as the calls of the kernel they are (the stream's
 * every batch).
 */
static inline __attribute__((always_inline)) void lsm6dso_os_work_submit(lsm6dso_os_work_t *work)
{
	(void)k_work_reschedule_for_queue(&lsm6dso_os_work_q, work, K_NO_WAIT);
}

static inline __attribute__((always_inline)) void lsm6dso_os_work_schedule(lsm6dso_os_work_t *work,
									   int32_t ms)
{
	(void)k_work_schedule_for_queue(&lsm6dso_os_work_q, work, K_MSEC(ms));
}

/* The binding's: the line of the devicetree's irq-gpios. */
static inline void lsm6dso_os_int1_init(lsm6dso_os_int1_t *int1, const struct gpio_dt_spec *gpio)
{
	int1->gpio = *gpio;
}

static inline bool lsm6dso_os_int1_ready(const lsm6dso_os_int1_t *int1)
{
	return gpio_is_ready_dt(&int1->gpio);
}

static inline int lsm6dso_os_int1_get(const lsm6dso_os_int1_t *int1)
{
	return gpio_pin_get_dt(&int1->gpio);
}

static inline int lsm6dso_os_clock_ns(uint64_t *ns)
{
	uint64_t cycles;
	int ret;

	ret = sensor_clock_get_cycles(&cycles);
	if (ret != 0) {
		return ret;
	}
	*ns = sensor_clock_cycles_to_ns(cycles);
	return 0;
}

#endif /* LSM6DSO_OS_IMPL_H_ */
