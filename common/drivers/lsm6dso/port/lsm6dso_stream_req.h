/* SPDX-License-Identifier: Apache-2.0 */
/* What the LSM6DSO's FIFO stream (lsm6dso_stream.c) needs from its user:
 * the request it serves, a buffer for each batch, and the place to hand the
 * filled buffer or the error that ends the stream.  The stream includes this
 * header; it includes the user's part, "lsm6dso_stream_req_impl.h", then
 * declares the inline functions that part must define.  The stream calls
 * them on its own thread (lsm6dso_os.h), at most twice a batch besides the
 * lookup, so they are inline.
 *
 * The user's part of every build is RTIO's, port/zephyr/
 * lsm6dso_stream_req_impl.h (a multishot read of the sensor API's stream),
 * which the NuttX, FreeRTOS and ThreadX builds compile on their Zephyr
 * compatibility layers; the host tests' part is common/tests/lsm6dso_os_host/
 * lsm6dso_stream_req_impl.h.
 *
 * lsm6dso_stream_req_impl.h: types
 *
 *   lsm6dso_stream_req_t       what the stream keeps of the request it
 *                              serves: enough to find it again and to tell
 *                              it from another (RTIO: its context and SQE
 *                              handle); a value the stream copies
 *   lsm6dso_stream_req_live_t  the request as it is while the stream works
 *                              on it (RTIO: struct rtio_iodev_sqe)
 *
 * The user gives a request to lsm6dso_stream_submit(), asks for a graceful
 * end with lsm6dso_stream_request_stop(), and may end it on its own side
 * (RTIO: a cancellation), which the stream notices at its next poll.
 */

#ifndef LSM6DSO_STREAM_REQ_H_
#define LSM6DSO_STREAM_REQ_H_

#include "lsm6dso_stream_req_impl.h"

#include <stdbool.h>
#include <stdint.h>

/* Whether a and b are the same request (RTIO: the same context and handle). */

static inline bool lsm6dso_stream_req_same(const lsm6dso_stream_req_t *a,
					   const lsm6dso_stream_req_t *b);

/* The request, if it is still there, or NULL if the user has ended it
 * (RTIO: rtio_iodev_sqe_from_handle()).  Called with the stream's spin lock
 * held.
 */

static inline lsm6dso_stream_req_live_t *lsm6dso_stream_req_get(const lsm6dso_stream_req_t *req);

/* Whether the user has asked to end the request at once (RTIO:
 * rtio_iodev_sqe_is_canceled()); the stream stops and ends it with
 * -ECANCELED.
 */

static inline bool lsm6dso_stream_req_canceled(lsm6dso_stream_req_live_t *live);

/* A buffer of at least min_len and at most max_len bytes for the next batch
 * (RTIO: rtio_sqe_rx_buf()): 0 with *buf and *len set, or a negative errno,
 * which ends the stream.
 */

static inline int lsm6dso_stream_req_buffer(lsm6dso_stream_req_live_t *live, uint32_t min_len,
					    uint32_t max_len, uint8_t **buf, uint32_t *len);

/* done: the buffer holds a batch (struct lsm6dso_fifo_packet); the request
 * goes on (RTIO: rtio_iodev_sqe_ok(), after which the multishot SQE is
 * submitted again, which may call lsm6dso_stream_submit()).  error: the
 * request ends with error, a negative errno (RTIO: rtio_iodev_sqe_err()).
 */

static inline void lsm6dso_stream_req_done(lsm6dso_stream_req_live_t *live);
static inline void lsm6dso_stream_req_error(lsm6dso_stream_req_live_t *live, int error);

#endif /* LSM6DSO_STREAM_REQ_H_ */
