/* SPDX-License-Identifier: Apache-2.0 */
/* The FIFO stream of the sensor API: the core's stream (lsm6dso_stream.c)
 * of each device, on the devicetree's INT1 line, serving RTIO's multishot
 * read (lsm6dso_stream_req_impl.h); the stream's settings and stop of the
 * extended API (<zephyr/drivers/sensor/lsm6dso.h>).
 */
#include "lsm6dso_sensor.h"

#include <errno.h>
#include <zephyr/rtio/rtio.h>

void lsm6dso_sensor_stream_init(const struct device *dev)
{
	const struct lsm6dso_config *cfg = dev->config;
	struct lsm6dso_data *data = dev->data;

	lsm6dso_os_int1_init(&data->stream.int1, &cfg->fifo_gpio);
	lsm6dso_stream_init(&data->stream, &data->core);
}

void lsm6dso_sensor_submit_stream(const struct device *dev, struct rtio_iodev_sqe *sqe)
{
	struct lsm6dso_data *data = dev->data;
	const struct sensor_read_config *cfg = sqe->sqe.iodev->data;
	lsm6dso_stream_req_t req;

	if (cfg->count != 1 || cfg->triggers[0].trigger != SENSOR_TRIG_FIFO_WATERMARK ||
	    cfg->triggers[0].opt != SENSOR_STREAM_DATA_INCLUDE) {
		rtio_iodev_sqe_err(sqe, -ENOTSUP);
		return;
	}
	req.rtio = sqe->r;
	req.handle = rtio_sqe_handle(sqe->r, &sqe->sqe);
	if (lsm6dso_stream_submit(&data->stream, &req) != 0) {
		rtio_iodev_sqe_err(sqe, -EBUSY);
	}
}

int lsm6dso_fifo_request_stop(const struct device *dev, struct rtio *context,
			      rtio_sqe_handle_t handle)
{
	struct lsm6dso_data *data = dev->data;
	const lsm6dso_stream_req_t req = {.rtio = context, .handle = handle};

	return lsm6dso_stream_request_stop(&data->stream, &req);
}

int lsm6dso_fifo_set_watermark(const struct device *dev, uint16_t watermark)
{
	struct lsm6dso_data *data = dev->data;

	return lsm6dso_stream_set_watermark(&data->stream, watermark);
}

uint16_t lsm6dso_fifo_get_watermark(const struct device *dev)
{
	struct lsm6dso_data *data = dev->data;

	return lsm6dso_stream_get_watermark(&data->stream);
}

int lsm6dso_fifo_set_timestamp_decimation(const struct device *dev, unsigned int decimation)
{
	struct lsm6dso_data *data = dev->data;

	return lsm6dso_set_timestamp_decimation(&data->core, decimation);
}

unsigned int lsm6dso_fifo_get_timestamp_decimation(const struct device *dev)
{
	struct lsm6dso_data *data = dev->data;

	return lsm6dso_get_timestamp_decimation(&data->core);
}
