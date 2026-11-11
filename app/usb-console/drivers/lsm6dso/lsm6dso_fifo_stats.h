/* SPDX-License-Identifier: Apache-2.0 */
#ifndef LSM6DSO_FIFO_STATS_H_
#define LSM6DSO_FIFO_STATS_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/rtio/rtio.h>

/* Limit the threshold to fit the sensor shell's 2048-byte RTIO pool. */
#define LSM6DSO_FIFO_DEFAULT_WATERMARK 32U
#define LSM6DSO_FIFO_MAX_WATERMARK 256U

int lsm6dso_fifo_set_watermark(const struct device *dev, uint16_t watermark);
uint16_t lsm6dso_fifo_get_watermark(const struct device *dev);
#if defined(CONFIG_IMU_TIMESTAMP_DIAGNOSTIC)
int lsm6dso_fifo_set_timestamp_decimation(const struct device *dev, unsigned int decimation);
unsigned int lsm6dso_fifo_get_timestamp_decimation(const struct device *dev);
#endif

struct lsm6dso_fifo_stats {
	uint32_t generation;
	uint64_t words_read;
	uint64_t transfer_time_ns;
	uint64_t max_transfer_time_ns;
	uint32_t transfers;
	uint32_t i2c_errors;
	uint32_t buffer_too_small;
	/* Observations, not a count of lost samples or distinct overruns. */
	uint32_t overrun_observations;
	int stop_error;
	uint16_t accel_hz;
	uint16_t gyro_hz;
	/* Sensitivity in effect when the FIFO was started. */
	uint32_t accel_ug_per_lsb;
	uint32_t gyro_udps_per_lsb;
	/* INTERNAL_FREQ_FINE read when the FIFO was started; 0 if unreadable. */
	int8_t freq_fine;
	bool active;
	bool fault;
};

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
#endif
