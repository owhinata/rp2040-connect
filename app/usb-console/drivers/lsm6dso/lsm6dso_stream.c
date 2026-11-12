/* SPDX-License-Identifier: Apache-2.0 */
#include "lsm6dso.h"
#include "lsm6dso_fifo.h"
#include "lsm6dso_fifo_hw.h"
#include "lsm6dso_fifo_decoder.h"

#include <zephyr/drivers/sensor_clock.h>
#include <zephyr/rtio/rtio.h>

#define STREAM_CANCEL_POLL_MS 20
#define STREAM_WORK_STACK_SIZE 2048

/* Keep FIFO I/O off Zephyr's cooperative system workqueue so a busy or
 * failing stream cannot monopolize USB and shell processing.
 */
static struct k_work_q stream_work_queue;
K_THREAD_STACK_DEFINE(stream_work_stack, STREAM_WORK_STACK_SIZE);
static atomic_t stream_queue_started;

void lsm6dso_submit_fallback(const struct device *dev, struct rtio_iodev_sqe *sqe);

static int stream_wakeup(void *context)
{
	struct lsm6dso_data *data = context;

	(void)k_work_reschedule_for_queue(&stream_work_queue, &data->stream_work, K_NO_WAIT);
	return 0;
}

static uint32_t period_ns(uint16_t freq)
{
	/* The driver's nominal 12 Hz entry corresponds to the 12.5 Hz ODR. */
	return freq == 12 ? 80000000U : freq ? 1000000000U / freq : 0;
}

static int stream_stop(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;
	int ret = 0, err;

	if (data->stream_active) {
		ret = lsm6dso_fifo_route_int1(&cfg->ctx, false);
	}
	if (data->stream_irq_active) {
		err = lsm6dso_fifo_irq_stop(&data->fifo_irq);
		if (ret == 0) {
			ret = err;
		}
		data->stream_irq_active = false;
	}
	err = lsm6dso_fifo_stop_locked_driver(dev);
	if (ret == 0) {
		ret = err;
	}
	lsm6dso_fifo_record_stop_error(dev, ret);
	data->stream_active = false;
	data->stream_batch_paused = false;
	return ret;
}

static int stream_start(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;
	int ret;

	if (!gpio_is_ready_dt(&cfg->fifo_gpio)) {
		return -ENODEV;
	}
	ret = lsm6dso_fifo_start_locked_driver(dev, data->stream_watermark,
					       data->accel_freq != 0, data->gyro_freq != 0);
	if (ret != 0) {
		(void)lsm6dso_fifo_stop_locked_driver(dev);
		return ret;
	}
	data->stream_active = true;
	ret = lsm6dso_fifo_irq_start(&data->fifo_irq, &cfg->fifo_gpio, stream_wakeup, data);
	if (ret == 0) {
		data->stream_irq_active = true;
		ret = lsm6dso_fifo_route_int1(&cfg->ctx, true);
	}
	if (ret != 0) {
		(void)stream_stop(dev);
	}
	return ret;
}

static int stream_read(const struct device *dev, struct rtio_iodev_sqe *sqe, bool drain)
{
	struct lsm6dso_data *data = dev->data;
	struct lsm6dso_fifo_status status;
	struct lsm6dso_fifo_packet *packet;
	struct lsm6dso_fifo_sample sample;
	struct lsm6dso_cal accel_cal, gyro_cal;
	uint8_t *buf;
	uint32_t len;
	uint64_t cycles, now;
	uint16_t accel_count = 0, gyro_count = 0;
	size_t cursor = 0;
	int ret;

	ret = lsm6dso_fifo_status_locked_driver(dev, &status);
	if (ret != 0) {
		return ret;
	}
	/* The number of lost samples is unknown. Terminate with an explicit
	 * discontinuity instead of returning an apparently continuous batch.
	 */
	if (status.overrun || status.overrun_latched) {
		return -EOVERFLOW;
	}
	if (status.words == 0 || (!drain && !status.watermark)) {
		return 0;
	}
	ret = rtio_sqe_rx_buf(sqe, sizeof(*packet) +
			      (drain ? 1 : data->stream_watermark) * LSM6DSO_FIFO_WORD_SIZE,
			      sizeof(*packet) + LSM6DSO_FIFO_MAX_WORDS * LSM6DSO_FIFO_WORD_SIZE,
			      &buf, &len);
	if (ret != 0) {
		return ret;
	}
	packet = (void *)buf;
	ret = lsm6dso_fifo_read_locked_driver(dev, packet->data,
			(len - sizeof(*packet)) / LSM6DSO_FIFO_WORD_SIZE, &status);
	if (ret != 0) {
		return ret;
	}
	if (status.overrun || status.overrun_latched) {
		return -EOVERFLOW;
	}
	if (status.words == 0) {
		return 0;
	}
	while (lsm6dso_fifo_next(packet->data, status.words * LSM6DSO_FIFO_WORD_SIZE,
				 &cursor, &sample) > 0) {
		accel_count += sample.kind == LSM6DSO_FIFO_ACCEL;
		gyro_count += sample.kind == LSM6DSO_FIFO_GYRO;
	}
	/* Before the snapshot below, so that this batch already uses a bias it
	 * has just updated.
	 */
	lsm6dso_cal_track(dev, packet->data, status.words * LSM6DSO_FIFO_WORD_SIZE);
	ret = sensor_clock_get_cycles(&cycles);
	if (ret != 0) {
		return ret;
	}
	now = sensor_clock_cycles_to_ns(cycles);
	packet->marker = LSM6DSO_FIFO_PACKET_MARKER;
	packet->words = status.words;
	packet->watermark = status.watermark;
	packet->accel_period_ns = period_ns(data->accel_freq);
	packet->gyro_period_ns = period_ns(data->gyro_freq);
	packet->accel_gain_ug = data->acc_gain;
	packet->gyro_gain_udps = data->gyro_gain;
	lsm6dso_cal_snapshot(dev, &accel_cal, &gyro_cal);
	memcpy(&packet->accel_cal, &accel_cal, sizeof(accel_cal));
	memcpy(&packet->gyro_cal, &gyro_cal, sizeof(gyro_cal));
	/* Host-read-time based estimate: FIFO and transfer latency are unknown. */
	uint64_t accel_age = accel_count ? (uint64_t)(accel_count - 1) * packet->accel_period_ns : 0;
	uint64_t gyro_age = gyro_count ? (uint64_t)(gyro_count - 1) * packet->gyro_period_ns : 0;
	packet->accel_first_timestamp_ns = now > accel_age ? now - accel_age : 0;
	packet->gyro_first_timestamp_ns = now > gyro_age ? now - gyro_age : 0;
	return 1;
}

static void stream_work(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);
	struct lsm6dso_data *data = CONTAINER_OF(delayable, struct lsm6dso_data, stream_work);
	struct rtio_iodev_sqe *sqe;
	k_spinlock_key_t key;
	bool stop_requested;
	int ret;

	key = k_spin_lock(&data->stream_lock);
	sqe = data->stream_sqe ? rtio_iodev_sqe_from_handle(data->stream_rtio,
							   data->stream_handle) : NULL;
	stop_requested = data->stream_stop_requested;
	k_spin_unlock(&data->stream_lock, key);
	if (!sqe) {
		/* The executor may have consumed a cancellation while completing a
		 * shot. In that case it frees the SQE without resubmitting it.
		 */
		if (data->stream_active) {
			(void)stream_stop(data->dev);
		}
		key = k_spin_lock(&data->stream_lock);
		data->stream_sqe = NULL;
		k_spin_unlock(&data->stream_lock, key);
		return;
	}
	if (rtio_iodev_sqe_is_canceled(sqe)) {
		ret = stream_stop(data->dev);
		key = k_spin_lock(&data->stream_lock);
		data->stream_sqe = NULL;
		k_spin_unlock(&data->stream_lock, key);
		rtio_iodev_sqe_err(sqe, ret ? ret : -ECANCELED);
		return;
	}
	if (stop_requested) {
		if (!data->stream_active) {
			ret = 0;
		} else if (!data->stream_batch_paused) {
			ret = lsm6dso_fifo_pause_batch_locked_driver(data->dev);
			if (ret == 0) {
				data->stream_batch_paused = true;
			}
		} else {
			ret = 0;
		}
		if (ret == 0 && data->stream_active) {
			ret = stream_read(data->dev, sqe, true);
		}
		if (ret <= 0) {
			int stop_ret = stream_stop(data->dev);

			key = k_spin_lock(&data->stream_lock);
			data->stream_sqe = NULL;
			k_spin_unlock(&data->stream_lock, key);
			rtio_iodev_sqe_err(sqe, ret < 0 ? ret : stop_ret ? stop_ret : -ECANCELED);
			return;
		}
		/* A final partial batch is delivered through the regular RTIO API.
		 * Its resubmission returns here to check that the FIFO is empty.
		 */
		rtio_iodev_sqe_ok(sqe, 0);
		return;
	}
	ret = data->stream_active ? 0 : stream_start(data->dev);
	if (ret == 0) {
		ret = stream_read(data->dev, sqe, false);
	}
	if (ret == 0) {
		(void)k_work_schedule_for_queue(&stream_work_queue, &data->stream_work,
					K_MSEC(STREAM_CANCEL_POLL_MS));
		return;
	}
	if (ret < 0) {
		(void)stream_stop(data->dev);
		key = k_spin_lock(&data->stream_lock);
		data->stream_sqe = NULL;
		k_spin_unlock(&data->stream_lock, key);
		rtio_iodev_sqe_err(sqe, ret);
		return;
	}
	/* A successful multishot completion immediately resubmits this SQE. */
	rtio_iodev_sqe_ok(sqe, 0);
	/* Also check if the executor consumed a concurrent cancellation. */
	(void)k_work_schedule_for_queue(&stream_work_queue, &data->stream_work,
					K_MSEC(STREAM_CANCEL_POLL_MS));
}

int lsm6dso_fifo_request_stop(const struct device *dev, struct rtio *context,
			      rtio_sqe_handle_t handle)
{
	struct lsm6dso_data *data = dev->data;
	k_spinlock_key_t key;
	bool found;

	key = k_spin_lock(&data->stream_lock);
	found = data->stream_sqe && data->stream_rtio == context && data->stream_handle == handle;
	if (found) {
		data->stream_stop_requested = true;
	}
	k_spin_unlock(&data->stream_lock, key);
	if (!found) {
		return -ENOENT;
	}
	(void)k_work_reschedule_for_queue(&stream_work_queue, &data->stream_work, K_NO_WAIT);
	return 0;
}

void lsm6dso_stream_init(const struct device *dev)
{
	struct lsm6dso_data *data = dev->data;

	if (atomic_cas(&stream_queue_started, 0, 1)) {
		k_work_queue_start(&stream_work_queue, stream_work_stack,
				   K_THREAD_STACK_SIZEOF(stream_work_stack),
				   K_PRIO_PREEMPT(10), NULL);
#if defined(CONFIG_THREAD_NAME)
		(void)k_thread_name_set(k_work_queue_thread_get(&stream_work_queue),
					"imu_fifo_worker");
#endif
	}
	data->stream_watermark = LSM6DSO_FIFO_DEFAULT_WATERMARK;
	data->timestamp_decimation = LSM6DSO_DEC_8;
	k_work_init_delayable(&data->stream_work, stream_work);
}

void lsm6dso_submit(const struct device *dev, struct rtio_iodev_sqe *sqe)
{
	struct lsm6dso_data *data = dev->data;
	const struct sensor_read_config *cfg = sqe->sqe.iodev->data;
	k_spinlock_key_t key;
	bool busy;

	if (!cfg->is_streaming) {
		lsm6dso_submit_fallback(dev, sqe);
		return;
	}
	if (cfg->count != 1 || cfg->triggers[0].trigger != SENSOR_TRIG_FIFO_WATERMARK ||
	    cfg->triggers[0].opt != SENSOR_STREAM_DATA_INCLUDE) {
		rtio_iodev_sqe_err(sqe, -ENOTSUP);
		return;
	}
	key = k_spin_lock(&data->stream_lock);
	busy = data->stream_sqe &&
		(data->stream_rtio != sqe->r ||
		 data->stream_handle != rtio_sqe_handle(sqe->r, &sqe->sqe));
	if (!busy) {
		if (!data->stream_sqe) {
			data->stream_stop_requested = false;
			data->stream_batch_paused = false;
		}
		data->stream_sqe = sqe;
		data->stream_rtio = sqe->r;
		data->stream_handle = rtio_sqe_handle(sqe->r, &sqe->sqe);
	}
	k_spin_unlock(&data->stream_lock, key);
	if (busy) {
		rtio_iodev_sqe_err(sqe, -EBUSY);
		return;
	}
	(void)k_work_reschedule_for_queue(&stream_work_queue, &data->stream_work, K_NO_WAIT);
}

int lsm6dso_fifo_set_watermark(const struct device *dev, uint16_t watermark)
{
	struct lsm6dso_data *data = dev->data;
	k_spinlock_key_t key;

	if (watermark == 0 || watermark > LSM6DSO_FIFO_MAX_WATERMARK) {
		return -EINVAL;
	}
	key = k_spin_lock(&data->stream_lock);
	if (data->stream_sqe) {
		k_spin_unlock(&data->stream_lock, key);
		return -EBUSY;
	}
	data->stream_watermark = watermark;
	k_spin_unlock(&data->stream_lock, key);
	return 0;
}

uint16_t lsm6dso_fifo_get_watermark(const struct device *dev)
{
	struct lsm6dso_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->stream_lock);
	uint16_t watermark = data->stream_watermark;

	k_spin_unlock(&data->stream_lock, key);
	return watermark;
}

int lsm6dso_fifo_set_timestamp_decimation(const struct device *dev, unsigned int decimation)
{
	struct lsm6dso_data *data = dev->data;
	uint8_t encoded;
	int ret = 0;

	switch (decimation) {
	case 0: encoded = LSM6DSO_NO_DECIMATION; break;
	case 1: encoded = LSM6DSO_DEC_1; break;
	case 8: encoded = LSM6DSO_DEC_8; break;
	case 32: encoded = LSM6DSO_DEC_32; break;
	default: return -EINVAL;
	}
	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	if (data->fifo_owned) {
		ret = -EBUSY;
	} else {
		data->timestamp_decimation = encoded;
	}
	k_mutex_unlock(&data->fifo_lock);
	return ret;
}

unsigned int lsm6dso_fifo_get_timestamp_decimation(const struct device *dev)
{
	static const unsigned int values[] = {0, 1, 8, 32};
	struct lsm6dso_data *data = dev->data;
	unsigned int value;

	k_mutex_lock(&data->fifo_lock, K_FOREVER);
	value = values[data->timestamp_decimation];
	k_mutex_unlock(&data->fifo_lock);
	return value;
}
