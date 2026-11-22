/* SPDX-License-Identifier: Apache-2.0 */
/* What the LSM6DSO's core (lsm6dso.c, lsm6dso_stream.c) needs from the OS.
 * The core includes this header before anything else; it includes the OS's
 * part, "lsm6dso_os_impl.h", then declares the functions that part must
 * define, so that a missing or different one does not compile.
 *
 * The OS's part gives the core two kinds of lock, waits, a cycle counter,
 * the long reads of the sensor's FIFO and timestamp registers and the log
 * (lsm6dso.c), and for the FIFO stream (lsm6dso_stream.c) a work item on a
 * thread of the driver's own, the INT1 line and the clock of the batches'
 * time stamps.  Everything else the core reaches through ST's register
 * driver (lsm6dso_reg.c), whose stmdev_ctx_t the OS's binding fills in: its
 * read_reg and write_reg move at most 255 bytes, enough for every register
 * but the FIFO's.  The boundary is made mostly of static inline functions
 * and macros, so that the compiler makes of each call what a direct call of
 * the OS would be: once a watermark, at up to 6667 Hz, the stream's work
 * runs, takes its lock, reads the FIFO (a lock, the cycle counter twice,
 * the bus twice), reads the clock and schedules its next poll.  The INT1
 * line's start and stop are ordinary functions (once a stream), which the
 * OS's part may define in a file of its own.
 *
 * The Zephyr build's part is port/zephyr/lsm6dso_os_impl.h with
 * port/zephyr/lsm6dso_os.c, on Zephyr's kernel, GPIO, I2C, sensor clock
 * and logging; the NuttX and FreeRTOS builds use the same part on their
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
 *   lsm6dso_os_work_t      a work item that runs lsm6dso_stream_run() on a
 *                          thread of the driver's own, which does the
 *                          stream's I2C (Zephyr's k_work_delayable on the
 *                          work queue imu_fifo_worker); lsm6dso_os_work_init()
 *                          before any use
 *   lsm6dso_os_int1_t      the sensor's INT1 line: its GPIO, its edge
 *                          interrupt and what that interrupt defers; the
 *                          OS's binding sets the line before the stream
 *                          starts
 *   LSM6DSO_OS_LOG_LEVEL   the level of the core's log (Zephyr's
 *                          CONFIG_SENSOR_LOG_LEVEL), for
 *                          LOG_MODULE_DECLARE(LSM6DSO, LSM6DSO_OS_LOG_LEVEL)
 *   LOG_MODULE_DECLARE(), LOG_ERR(), LOG_INF() and LOG_DBG() as Zephyr's
 *                          (the module LSM6DSO is registered by the OS's
 *                          binding, lsm6dso_sensor.c)
 *
 * and declares the two functions of the core that it calls:
 *
 *   void lsm6dso_stream_run(lsm6dso_os_work_t *work);
 *                          from the work item's thread, each time the work
 *                          runs
 *   int lsm6dso_stream_int1(lsm6dso_os_int1_t *int1);
 *                          after each edge of INT1 to active, in a context
 *                          of the OS's part's choice (Zephyr's: the system
 *                          work queue), while the line is enabled; it
 *                          returns 0 (else the OS's part disables the line)
 *
 * Nothing else: the core takes the rest from the C library (<errno.h>,
 * <stdbool.h>, <stddef.h>, <stdint.h>, <string.h>) and from ST's
 * lsm6dso_reg.h.
 */

#ifndef LSM6DSO_OS_H_
#define LSM6DSO_OS_H_

#include "lsm6dso_os_impl.h"

#include <stdbool.h>
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
 * errno.  Thread context.  bus_read_supported: whether bus_read can reach
 * the device behind ctx (the stream starts on no other).
 */

static inline int lsm6dso_os_bus_read(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *buf,
				      size_t len);
static inline bool lsm6dso_os_bus_read_supported(const stmdev_ctx_t *ctx);

/* The stream's work item, on its own thread (Zephyr's work queue
 * imu_fifo_worker, a preemptible thread of priority 10, not the system
 * work queue).  init: before any use, also starts the thread if it is not
 * running.  submit: runs the work as soon as possible, also when it waits
 * for a delay (k_work_reschedule_for_queue(K_NO_WAIT)); a work that is
 * running runs again afterwards.  schedule: runs it after ms milliseconds,
 * unless it is already waiting to run (k_work_schedule_for_queue()).  Both
 * may be called from an interrupt.
 */

static inline void lsm6dso_os_work_init(lsm6dso_os_work_t *work);
static inline void lsm6dso_os_work_submit(lsm6dso_os_work_t *work);
static inline void lsm6dso_os_work_schedule(lsm6dso_os_work_t *work, int32_t ms);

/* INT1, the line of the FIFO threshold.  ready: the line's GPIO can be used
 * (gpio_is_ready_dt()).  enable: the line as an input and its edge interrupt
 * to active on; from then on, until disable, each edge leads to
 * lsm6dso_stream_int1().  0, or a negative errno with the line as it was.
 * get: the line's level, 1 for active, 0, or a negative errno.  disable: the
 * interrupt off, and no call of lsm6dso_stream_int1() runs or follows when
 * it returns.  0, or the first negative errno.  The stream's thread only.
 */

static inline bool lsm6dso_os_int1_ready(const lsm6dso_os_int1_t *int1);
int lsm6dso_os_int1_enable(lsm6dso_os_int1_t *int1);
static inline int lsm6dso_os_int1_get(const lsm6dso_os_int1_t *int1);
int lsm6dso_os_int1_disable(lsm6dso_os_int1_t *int1);

/* sensor_clock_get_cycles() and sensor_clock_cycles_to_ns(): the time of a
 * batch, in nanoseconds of the clock of the sensor API's time stamps.  0, or
 * a negative errno.
 */

static inline int lsm6dso_os_clock_ns(uint64_t *ns);

#endif /* LSM6DSO_OS_H_ */
