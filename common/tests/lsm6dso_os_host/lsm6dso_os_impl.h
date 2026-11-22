/* SPDX-License-Identifier: Apache-2.0 */
/* The OS's part of the LSM6DSO's core (common/drivers/lsm6dso/port/lsm6dso_os.h) on the host's C
 * library alone, for the check that the core needs nothing else (common/tests/run.sh,
 * lsm6dso_core_independent) and the test of the FIFO stream (tests/lsm6dso_stream.c): one thread,
 * no interrupts, waits that return at once, a cycle counter that stands still, a bus without a
 * device that the stream may use, a work item that counts what it is asked, an INT1 line whose
 * state the test sets and reads, a clock that stands at one second and a log that prints nothing.
 */
#ifndef LSM6DSO_OS_IMPL_H_
#define LSM6DSO_OS_IMPL_H_

#include <errno.h>
#include <stdbool.h>
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
/* submitted: lsm6dso_os_work_submit() calls; scheduled: the calls of
 * lsm6dso_os_work_schedule() that found the work idle, the last one's delay.
 * The test runs the work (lsm6dso_stream_run()) and clears pending.
 */
typedef struct {
	unsigned int submitted;
	unsigned int scheduled;
	int32_t delay_ms;
	bool pending;
} lsm6dso_os_work_t;
/* ready, level, enable_error and disable_error are the test's; enabled and
 * the counts are the stream's calls.
 */
typedef struct {
	bool ready;
	int level;
	int enable_error;
	int disable_error;
	bool enabled;
	unsigned int enables;
	unsigned int disables;
} lsm6dso_os_int1_t;

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

static inline bool lsm6dso_os_bus_read_supported(const stmdev_ctx_t *ctx)
{
	(void)ctx;
	return true;
}

void lsm6dso_stream_run(lsm6dso_os_work_t *work);
int lsm6dso_stream_int1(lsm6dso_os_int1_t *int1);

static inline void lsm6dso_os_work_init(lsm6dso_os_work_t *work)
{
	*work = (lsm6dso_os_work_t){0};
}

static inline void lsm6dso_os_work_submit(lsm6dso_os_work_t *work)
{
	work->submitted++;
	work->delay_ms = 0;
	work->pending = true;
}

static inline void lsm6dso_os_work_schedule(lsm6dso_os_work_t *work, int32_t ms)
{
	if (!work->pending) {
		work->scheduled++;
		work->delay_ms = ms;
		work->pending = true;
	}
}

static inline bool lsm6dso_os_int1_ready(const lsm6dso_os_int1_t *int1)
{
	return int1->ready;
}

static inline int lsm6dso_os_int1_enable(lsm6dso_os_int1_t *int1)
{
	if (int1->enable_error != 0) {
		return int1->enable_error;
	}
	int1->enabled = true;
	int1->enables++;
	return 0;
}

static inline int lsm6dso_os_int1_get(const lsm6dso_os_int1_t *int1)
{
	return int1->level;
}

static inline int lsm6dso_os_int1_disable(lsm6dso_os_int1_t *int1)
{
	int1->enabled = false;
	int1->disables++;
	return int1->disable_error;
}

static inline int lsm6dso_os_clock_ns(uint64_t *ns)
{
	*ns = 1000000000;
	return 0;
}

#endif /* LSM6DSO_OS_IMPL_H_ */
