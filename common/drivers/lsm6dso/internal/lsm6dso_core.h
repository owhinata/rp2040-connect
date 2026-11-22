/* SPDX-License-Identifier: Apache-2.0 */
/* The LSM6DSO's core (lsm6dso.c): how the chip is operated, with no OS.
 * Its start and reset, the output data rates and full scales, the single
 * reads of the accelerometer and the gyro and their conversion, the FIFO
 * (its setup, pause, stop, reads and status, INT1's routing of its
 * threshold, the timestamp words' decimation, and the counters of struct
 * lsm6dso_fifo_stats), the read of the timestamp counter, and the
 * calibration in effect with the gyro bias tracked at rest.  It works on a
 * struct lsm6dso, which the OS's binding keeps for each device, and
 * reaches the chip through ST's lsm6dso_reg.c and the OS through
 * lsm6dso_os.h.  The FIFO stream on top of it is lsm6dso_stream.h.
 *
 * Three locks: fifo_lock serializes the accesses to the chip (all but INT1's
 * routing, which the stream makes while it owns the FIFO) and gives the
 * FIFO its owner (fifo_owned) while a stream runs; cal_lock guards the
 * conversions, which the stream's thread copies for each batch; zero_lock
 * guards the bias tracker.  The functions that take no lock say so: the
 * binding calls them between lsm6dso_acquire() and lsm6dso_release().
 *
 * The names: lsm6dso_*, but where that would repeat a call of the extended
 * API that takes a device (<zephyr/drivers/sensor/lsm6dso.h>), which keeps
 * its name, or fall among lsm6dso_cal.h's: lsm6dso_set_cal_lookup(),
 * lsm6dso_reload_cal(), lsm6dso_get_cal_status(),
 * lsm6dso_measure_gyro_bias() (lsm6dso_cal_zero()),
 * lsm6dso_track_gyro_bias(), lsm6dso_copy_cal(), lsm6dso_get_fifo_stats(),
 * lsm6dso_set_timestamp_decimation(), lsm6dso_get_timestamp_decimation()
 * and lsm6dso_read_timestamp().
 */
#ifndef LSM6DSO_CORE_H_
#define LSM6DSO_CORE_H_

#include "lsm6dso_os.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lsm6dso_reg.h"
#include "lsm6dso_cal.h"
#include "lsm6dso_fifo.h"

/* FIFO_STATUS1 and FIFO_STATUS2. */
struct lsm6dso_fifo_status {
	uint16_t words;
	bool watermark;
	bool full;
	bool overrun;
	bool overrun_latched;
};

/* What lsm6dso_init() sets the chip to: the devicetree's values of the
 * Zephyr build (st,lsm6dso-common.yaml), as register values.
 */
struct lsm6dso_init_config {
	/* ST's bus functions and their device; lsm6dso_os_bus_read() reads
	 * the FIFO on the same device.
	 */
	const stmdev_ctx_t *ctx;
	uint8_t accel_pm;
	uint8_t accel_odr;
	uint8_t accel_lp_filter;
	uint8_t accel_range;     /* the full-scale register value */
	bool accel_range_double; /* LSM6DSO32: twice the nominal ranges */
	uint8_t gyro_pm;
	uint8_t gyro_odr;
	uint8_t gyro_range;
};

struct lsm6dso {
	const stmdev_ctx_t *ctx;
	bool accel_range_double;
	lsm6dso_os_mutex_t fifo_lock;
	/* Nonzero also covers an error state until bypass has been restored. */
	bool fifo_owned;
	bool fifo_fault;
	uint32_t fifo_generation;
	struct lsm6dso_fifo_stats fifo_stats;
	/* The FIFO_CTRL4 encoding, for the next start; guarded by fifo_lock. */
	uint8_t timestamp_decimation;
	int16_t acc[3];
	uint32_t acc_gain;
	int16_t gyro[3];
	uint32_t gyro_gain;
	/* Conversions to physical units; guarded by cal_lock. */
	lsm6dso_os_spinlock_t cal_lock;
	struct lsm6dso_cal accel_cal;
	struct lsm6dso_cal gyro_cal;
	bool accel_calibrated;
	bool gyro_calibrated;
	lsm6dso_cal_lookup_t cal_lookup;
	/* Gyro bias tracked at rest; guarded by zero_lock. */
	lsm6dso_os_mutex_t zero_lock;
	struct lsm6dso_zero zero;
	uint16_t accel_freq;
	uint8_t accel_fs;
	uint16_t gyro_freq;
	uint8_t gyro_fs;
};

/* Initializes the locks and the settings of chip, then the chip itself:
 * the user bank, its identity, I3C off, the software reset, the power
 * modes, full scales and output data rates of config, the FIFO in bypass
 * mode, block data update and the accelerometer's low-pass filter.  Logs
 * what fails.  0, or -EIO.
 */
int lsm6dso_init(struct lsm6dso *core, const struct lsm6dso_init_config *config);

/* Takes fifo_lock for a single read or a setting: 0, or -EBUSY (the lock not
 * held) while the FIFO owns the sensor.
 */
static inline int lsm6dso_acquire(struct lsm6dso *core)
{
	lsm6dso_os_mutex_lock(&core->fifo_lock);
	if (core->fifo_owned) {
		lsm6dso_os_mutex_unlock(&core->fifo_lock);
		return -EBUSY;
	}
	return 0;
}

static inline void lsm6dso_release(struct lsm6dso *core)
{
	lsm6dso_os_mutex_unlock(&core->fifo_lock);
}

/* The settings, with fifo_lock held (lsm6dso_acquire()): the output
 * data rate in Hz (the first rate at or above freq; 0 is off) and the full
 * scale in g or dps.  A full scale also selects its conversion.  0,
 * -EINVAL for a value the chip does not have, or -EIO.
 */
int lsm6dso_accel_odr_set(struct lsm6dso *core, uint16_t freq);
int lsm6dso_accel_range_set(struct lsm6dso *core, int32_t range);
int lsm6dso_gyro_odr_set(struct lsm6dso *core, uint16_t freq);
int lsm6dso_gyro_range_set(struct lsm6dso *core, int32_t range);

/* The single reads into acc and gyro, with fifo_lock held.
 * 0, or -EIO.
 */
int lsm6dso_fetch_accel(struct lsm6dso *core);
int lsm6dso_fetch_gyro(struct lsm6dso *core);

/* Converts the last single read of kind (acc or gyro) with the conversion in
 * effect, the same as the FIFO decoder's (lsm6dso_cal_apply()).
 */
void lsm6dso_convert(struct lsm6dso *core, enum lsm6dso_cal_kind kind, int32_t out[3]);

/* The calibration (<zephyr/drivers/sensor/lsm6dso.h> describes the calls
 * that take a device).  lsm6dso_measure_gyro_bias() takes fifo_lock itself.
 */
void lsm6dso_set_cal_lookup(struct lsm6dso *core, lsm6dso_cal_lookup_t lookup);
void lsm6dso_reload_cal(struct lsm6dso *core);
void lsm6dso_get_cal_status(struct lsm6dso *core, struct lsm6dso_cal_status *status);
int lsm6dso_measure_gyro_bias(struct lsm6dso *core);
/* Feeds FIFO words to the gyro bias tracker. */
void lsm6dso_track_gyro_bias(struct lsm6dso *core, const uint8_t *words, size_t len);
/* Copies the conversions currently in effect. */
void lsm6dso_copy_cal(struct lsm6dso *core, struct lsm6dso_cal *accel, struct lsm6dso_cal *gyro);

/* The FIFO.  Each takes fifo_lock.  The caller sets the output data rates
 * first, and arranges the interrupt and error recovery.
 *
 * start: takes the FIFO for a stream (fifo_owned), clears the counters, and
 * batches the sensors asked for at their rates in stream mode with the
 * watermark (1 to 511 words) and the timestamp decimation.  -EBUSY if
 * owned, -EINVAL for a sensor that is off, no sensor or a watermark out of
 * range, -EIO (owned and faulty until stopped).
 * pause_batch: stops batching both sensors and keeps stream mode, so that
 * what is in the FIFO can be read out.
 * stop: bypass mode, which empties the FIFO, and releases it.
 * read: the status, then all the words in one burst if capacity_words
 * holds them (-ENOBUFS otherwise).  status: the status only.  Both -EIO
 * unless owned and without a fault.
 */
int lsm6dso_fifo_start(struct lsm6dso *core, uint16_t watermark, bool accel, bool gyro);
int lsm6dso_fifo_pause_batch(struct lsm6dso *core);
int lsm6dso_fifo_stop(struct lsm6dso *core);
int lsm6dso_fifo_read(struct lsm6dso *core, uint8_t *buffer, size_t capacity_words,
		      struct lsm6dso_fifo_status *status);
int lsm6dso_fifo_status(struct lsm6dso *core, struct lsm6dso_fifo_status *status);
void lsm6dso_get_fifo_stats(struct lsm6dso *core, struct lsm6dso_fifo_stats *stats);
/* Keeps the first nonzero error of a stream's stop in its counters. */
void lsm6dso_fifo_record_stop_error(struct lsm6dso *core, int error);
/* One timestamp word per 0 (none), 1, 8 or 32 sampling instants, for the
 * next start: -EINVAL for another value, -EBUSY while owned.
 */
int lsm6dso_set_timestamp_decimation(struct lsm6dso *core, unsigned int decimation);
unsigned int lsm6dso_get_timestamp_decimation(struct lsm6dso *core);
/* Routes the FIFO threshold to INT1 or takes it off, leaving INT1's other
 * sources as they are.  Takes no lock.  0, or -EIO.
 */
int lsm6dso_fifo_route_int1(struct lsm6dso *core, bool enabled);

/* lsm6dso_timestamp_read() of <zephyr/drivers/sensor/lsm6dso.h>. */
int lsm6dso_read_timestamp(struct lsm6dso *core, int64_t (*clock_us)(void), uint32_t *tick,
			   int64_t *before_us, int64_t *after_us);

#endif /* LSM6DSO_CORE_H_ */
