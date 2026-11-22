/* SPDX-License-Identifier: Apache-2.0 */
/* The extended API of the LSM6DSO driver (drivers/sensor/st/lsm6dso) on
 * Zephyr's sensor devices: the FIFO words and their parser
 * (lsm6dso_fifo.h), which also finds the words in a buffer of the FIFO
 * stream; the conversion to physical units (lsm6dso_cal.h) and its stored
 * calibration; the attributes, settings, counters and stop of the FIFO
 * stream; and the read of the sensor's timestamp counter.
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_LSM6DSO_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_LSM6DSO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>

#include "lsm6dso_cal.h"
#include "lsm6dso_fifo.h"

/* Finds the FIFO words in a buffer delivered by the driver's stream. The
 * buffer starts with a header whose layout belongs to the driver; use this
 * instead of reading it. Returns 0 and sets words and size (in bytes, a
 * multiple of LSM6DSO_FIFO_WORD_SIZE), or -EBADMSG if the buffer is not a
 * complete FIFO buffer of this driver.
 */
int lsm6dso_fifo_buffer_words(const uint8_t *buffer, size_t length, const uint8_t **words,
			      size_t *size);

void lsm6dso_cal_set_lookup(const struct device *dev, lsm6dso_cal_lookup_t lookup);
void lsm6dso_cal_reload(const struct device *dev);
void lsm6dso_cal_status(const struct device *dev, struct lsm6dso_cal_status *status);
/* Takes the gyro bias from the sensor at rest, now. Runs the sensor at a low
 * rate for up to three seconds if it is off, and switches it off again.
 * Returns 0 when the bias was updated, -EBUSY while a stream owns the sensor,
 * -EAGAIN if the sensor did not rest, -ERANGE if the reading was too far from
 * the stored calibration to be a bias.
 */
int lsm6dso_cal_zero(const struct device *dev);

/* Device-wide FIFO settings for SENSOR_CHAN_ALL. Both take effect at the next
 * stream start and cannot be changed while a stream is running. */
enum lsm6dso_sensor_attribute {
	/* FIFO threshold in FIFO words. */
	SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK = SENSOR_ATTR_PRIV_START,
	/* One timestamp word per this many sampling instants: 0 (none), 1, 8 or 32. */
	SENSOR_ATTR_LSM6DSO_FIFO_TIMESTAMP_DECIMATION,
};

int lsm6dso_fifo_set_watermark(const struct device *dev, uint16_t watermark);
uint16_t lsm6dso_fifo_get_watermark(const struct device *dev);
int lsm6dso_fifo_set_timestamp_decimation(const struct device *dev, unsigned int decimation);
unsigned int lsm6dso_fifo_get_timestamp_decimation(const struct device *dev);

void lsm6dso_fifo_get_stats(const struct device *dev, struct lsm6dso_fifo_stats *stats);
/* Reads the sensor's 32-bit timestamp counter (the clock of the FIFO
 * timestamp words) between two readings of the caller's clock, so that the
 * two clocks can be related. The counter was sampled somewhere between
 * *before_us and *after_us. Returns -EAGAIN if two consecutive reads
 * disagree, which would indicate a torn multi-byte read.
 * Thread context only; waits for a FIFO transfer in progress. */
int lsm6dso_timestamp_read(const struct device *dev, int64_t (*clock_us)(void), uint32_t *tick,
			   int64_t *before_us, int64_t *after_us);
/* Request a graceful stop of this stream, leaving its RTIO handle valid until
 * all pending FIFO words have been delivered and the terminal CQE is posted.
 */
int lsm6dso_fifo_request_stop(const struct device *dev, struct rtio *context,
			      rtio_sqe_handle_t handle);

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_LSM6DSO_H_ */
