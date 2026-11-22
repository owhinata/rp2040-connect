/* SPDX-License-Identifier: Apache-2.0 */
/* The FIFO stream's user on Zephyr (lsm6dso_stream_req.h): RTIO's multishot
 * read of the sensor API's stream, which lsm6dso_rtio_stream.c hands to the
 * stream.  The NuttX and FreeRTOS builds use this file on their Zephyr
 * compatibility layers.
 */

#ifndef LSM6DSO_STREAM_REQ_IMPL_H_
#define LSM6DSO_STREAM_REQ_IMPL_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/rtio/rtio.h>

typedef struct {
	struct rtio *rtio;
	rtio_sqe_handle_t handle;
} lsm6dso_stream_req_t;

typedef struct rtio_iodev_sqe lsm6dso_stream_req_live_t;

static inline bool lsm6dso_stream_req_same(const lsm6dso_stream_req_t *a,
					   const lsm6dso_stream_req_t *b)
{
	return a->rtio == b->rtio && a->handle == b->handle;
}

static inline lsm6dso_stream_req_live_t *lsm6dso_stream_req_get(const lsm6dso_stream_req_t *req)
{
	return rtio_iodev_sqe_from_handle(req->rtio, req->handle);
}

static inline bool lsm6dso_stream_req_canceled(lsm6dso_stream_req_live_t *live)
{
	return rtio_iodev_sqe_is_canceled(live);
}

static inline int lsm6dso_stream_req_buffer(lsm6dso_stream_req_live_t *live, uint32_t min_len,
					    uint32_t max_len, uint8_t **buf, uint32_t *len)
{
	return rtio_sqe_rx_buf(live, min_len, max_len, buf, len);
}

static inline void lsm6dso_stream_req_done(lsm6dso_stream_req_live_t *live)
{
	rtio_iodev_sqe_ok(live, 0);
}

static inline void lsm6dso_stream_req_error(lsm6dso_stream_req_live_t *live, int error)
{
	rtio_iodev_sqe_err(live, error);
}

#endif /* LSM6DSO_STREAM_REQ_IMPL_H_ */
