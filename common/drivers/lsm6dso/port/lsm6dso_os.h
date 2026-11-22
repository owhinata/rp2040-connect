/* SPDX-License-Identifier: Apache-2.0 */
/* What the LSM6DSO's core (lsm6dso_chip.c) needs from the OS.  The core
 * includes this header before anything else; it includes the OS's part,
 * "lsm6dso_os_impl.h", then declares the inline functions that part must
 * define, so that a missing or different one does not compile.
 *
 * The OS's part gives the core two kinds of lock, waits, a cycle counter,
 * the long reads of the sensor's FIFO and timestamp registers, and the log.
 * Everything else the core reaches through ST's register driver
 * (lsm6dso_reg.c), whose stmdev_ctx_t the OS's binding fills in: its
 * read_reg and write_reg move at most 255 bytes, enough for every register
 * but the FIFO's.  The boundary is made of static inline functions and
 * macros, so that the compiler makes of each call what a direct call of the
 * OS would be: the core's FIFO read, which runs once a watermark at up to
 * 6667 Hz, takes a lock, reads the cycle counter twice and reads the bus
 * twice.
 *
 * The Zephyr build's part is port/zephyr/lsm6dso_os_impl.h (in its patched
 * tree, drivers/sensor/st/lsm6dso/lsm6dso_os_impl.h), on Zephyr's kernel,
 * I2C and logging; the NuttX and FreeRTOS builds use the same part on their
 * Zephyr compatibility layers.  The host tests' part is
 * common/tests/lsm6dso_os_host/lsm6dso_os_impl.h.
 *
 * lsm6dso_os_impl.h: types and macros
 *
 *   lsm6dso_os_mutex_t     a mutex with priority inheritance (Zephyr's
 *                          k_mutex), which a thread may take again while it
 *                          holds it; lsm6dso_os_mutex_init() before any use
 *   lsm6dso_os_spinlock_t  a lock against the other threads and interrupts
 *                          of the CPU for a few instructions (Zephyr's
 *                          k_spinlock: interrupts off on the RP2040); a
 *                          zeroed object is an unlocked lock
 *   lsm6dso_os_spinlock_key_t  what lsm6dso_os_spin_lock() returns
 *   LSM6DSO_OS_LOG_LEVEL   the level of the core's log (Zephyr's
 *                          CONFIG_SENSOR_LOG_LEVEL), for
 *                          LOG_MODULE_DECLARE(LSM6DSO, LSM6DSO_OS_LOG_LEVEL)
 *   LOG_MODULE_DECLARE(), LOG_ERR(), LOG_INF() and LOG_DBG() as Zephyr's
 *                          (the module LSM6DSO is registered by the OS's
 *                          binding, lsm6dso.c)
 *
 * Nothing else: the core takes the rest from the C library (<errno.h>,
 * <stdbool.h>, <stddef.h>, <stdint.h>, <string.h>) and from ST's
 * lsm6dso_reg.h.
 */

#ifndef LSM6DSO_OS_H_
#define LSM6DSO_OS_H_

#include "lsm6dso_os_impl.h"

#include <stddef.h>
#include <stdint.h>

#include "lsm6dso_reg.h"

/* k_mutex_init(), k_mutex_lock(mutex, K_FOREVER) and k_mutex_unlock(). */

static inline void lsm6dso_os_mutex_init(lsm6dso_os_mutex_t *mutex);
static inline void lsm6dso_os_mutex_lock(lsm6dso_os_mutex_t *mutex);
static inline void lsm6dso_os_mutex_unlock(lsm6dso_os_mutex_t *mutex);

/* k_spin_lock() and k_spin_unlock(). */

static inline lsm6dso_os_spinlock_key_t lsm6dso_os_spin_lock(lsm6dso_os_spinlock_t *lock);
static inline void lsm6dso_os_spin_unlock(lsm6dso_os_spinlock_t *lock,
					  lsm6dso_os_spinlock_key_t key);

/* k_busy_wait(us): waits at least us microseconds without sleeping. */

static inline void lsm6dso_os_busy_wait_us(uint32_t us);

/* k_msleep(ms) and k_usleep(us): sleep at least that long. */

static inline void lsm6dso_os_msleep(int32_t ms);
static inline void lsm6dso_os_usleep(int32_t us);

/* k_cycle_get_32() and k_cyc_to_ns_floor64(): a free-running 32-bit cycle
 * counter, and a number of its cycles in nanoseconds, rounded down.
 */

static inline uint32_t lsm6dso_os_cycle_get_32(void);
static inline uint64_t lsm6dso_os_cycles_to_ns(uint32_t cycles);

/* i2c_burst_read_dt(): reads len bytes from the register reg on, on the bus
 * and at the address behind ctx (the device that ctx->read_reg reads), in one
 * transfer: the register, then the bytes with a repeated start.  len is at
 * most LSM6DSO_FIFO_MAX_WORDS * LSM6DSO_FIFO_WORD_SIZE.  0, or a negative
 * errno.  Thread context.
 */

static inline int lsm6dso_os_bus_read(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *buf,
				      size_t len);

#endif /* LSM6DSO_OS_H_ */
