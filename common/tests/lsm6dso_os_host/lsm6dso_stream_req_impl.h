/* SPDX-License-Identifier: Apache-2.0 */
/* The FIFO stream's user (common/drivers/lsm6dso/port/lsm6dso_stream_req.h) on the host's C
 * library alone, for the check that the core needs nothing else (common/tests/run.sh,
 * lsm6dso_core_independent) and the test of the FIFO stream (tests/lsm6dso_stream.c), which
 * defines the functions below: a request is a number, which the test resolves to its own object.
 */
#ifndef LSM6DSO_STREAM_REQ_IMPL_H_
#define LSM6DSO_STREAM_REQ_IMPL_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	int id;
} lsm6dso_stream_req_t;

typedef struct lsm6dso_stream_req_host lsm6dso_stream_req_live_t;

lsm6dso_stream_req_live_t *lsm6dso_stream_req_host_get(int id);
bool lsm6dso_stream_req_host_canceled(lsm6dso_stream_req_live_t *live);
int lsm6dso_stream_req_host_buffer(lsm6dso_stream_req_live_t *live, uint32_t min_len,
				   uint32_t max_len, uint8_t **buf, uint32_t *len);
void lsm6dso_stream_req_host_done(lsm6dso_stream_req_live_t *live);
void lsm6dso_stream_req_host_error(lsm6dso_stream_req_live_t *live, int error);

static inline bool lsm6dso_stream_req_same(const lsm6dso_stream_req_t *a,
					   const lsm6dso_stream_req_t *b)
{
	return a->id == b->id;
}

static inline lsm6dso_stream_req_live_t *lsm6dso_stream_req_get(const lsm6dso_stream_req_t *req)
{
	return lsm6dso_stream_req_host_get(req->id);
}

static inline bool lsm6dso_stream_req_canceled(lsm6dso_stream_req_live_t *live)
{
	return lsm6dso_stream_req_host_canceled(live);
}

static inline int lsm6dso_stream_req_buffer(lsm6dso_stream_req_live_t *live, uint32_t min_len,
					    uint32_t max_len, uint8_t **buf, uint32_t *len)
{
	return lsm6dso_stream_req_host_buffer(live, min_len, max_len, buf, len);
}

static inline void lsm6dso_stream_req_done(lsm6dso_stream_req_live_t *live)
{
	lsm6dso_stream_req_host_done(live);
}

static inline void lsm6dso_stream_req_error(lsm6dso_stream_req_live_t *live, int error)
{
	lsm6dso_stream_req_host_error(live, error);
}

#endif /* LSM6DSO_STREAM_REQ_IMPL_H_ */
