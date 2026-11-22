/* SPDX-License-Identifier: Apache-2.0 */
/* The LSM6DSO's FIFO stream (lsm6dso_stream.c), with no OS: the state
 * machine that serves one request of its user (lsm6dso_stream_req.h) from
 * the core's FIFO (lsm6dso_core.h).  It starts the FIFO at the first run,
 * routes the threshold to INT1 and enables the line (lsm6dso_os.h); each
 * edge, a level already active at the start, and a poll every
 * STREAM_CANCEL_POLL_MS run its work, which delivers each batch of at least
 * the watermark into a buffer of the user (struct lsm6dso_fifo_packet), and
 * notices the end of the request on the user's side.  A graceful stop stops
 * the batching, delivers what is left in the FIFO, then ends the request
 * with -ECANCELED.  An error (an overrun: -EOVERFLOW) stops the FIFO and
 * ends the request with it.
 *
 * The OS's binding keeps a struct lsm6dso_stream next to the device's
 * struct lsm6dso, sets its INT1 line, and calls lsm6dso_stream_init() before
 * any other.  lock (a spin lock) guards the request, requested,
 * stop_requested and the watermark's changes; the rest belongs to the work's
 * thread.
 */
#ifndef LSM6DSO_STREAM_H_
#define LSM6DSO_STREAM_H_

#include "lsm6dso_os.h"
#include "lsm6dso_stream_req.h"

#include <stdbool.h>
#include <stdint.h>

#include "lsm6dso_core.h"

struct lsm6dso_stream {
	struct lsm6dso *core;
	lsm6dso_os_work_t work;
	lsm6dso_os_int1_t int1;
	lsm6dso_os_spinlock_t lock;
	/* The request served, while requested. */
	lsm6dso_stream_req_t req;
	bool requested;
	bool stop_requested;
	/* The FIFO threshold in words for the next start. */
	uint16_t watermark;
	/* The FIFO is started (it may be faulty), INT1 is enabled, batching is
	 * stopped for a graceful stop.
	 */
	bool active;
	bool irq_active;
	bool batch_paused;
};

/* The stream of core, idle, with LSM6DSO_FIFO_DEFAULT_WATERMARK. */
void lsm6dso_stream_init(struct lsm6dso_stream *stream, struct lsm6dso *core);

/* Takes req: 0, or -EBUSY while another request is served.  The same
 * request again (RTIO's multishot resubmission) keeps the stream as it is.
 * Wakes the work.
 */
int lsm6dso_stream_submit(struct lsm6dso_stream *stream, const lsm6dso_stream_req_t *req);

/* A graceful stop of req: 0, or -ENOENT if it is not the request served. */
int lsm6dso_stream_request_stop(struct lsm6dso_stream *stream, const lsm6dso_stream_req_t *req);

/* 1 to LSM6DSO_FIFO_MAX_WATERMARK words: 0, -EINVAL, or -EBUSY while a
 * request is served.
 */
int lsm6dso_stream_set_watermark(struct lsm6dso_stream *stream, uint16_t watermark);
uint16_t lsm6dso_stream_get_watermark(struct lsm6dso_stream *stream);

/* The work (lsm6dso_os_work_t) and INT1's edge (lsm6dso_os_int1_t), which
 * the OS's part calls (lsm6dso_os.h).  lsm6dso_stream_int1() returns 0.
 */
void lsm6dso_stream_run(lsm6dso_os_work_t *work);
int lsm6dso_stream_int1(lsm6dso_os_int1_t *int1);

#endif /* LSM6DSO_STREAM_H_ */
