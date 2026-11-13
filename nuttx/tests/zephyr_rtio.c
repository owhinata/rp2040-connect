/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the compatibility layer's RTIO (nuttx/drivers/compat:
 * <zephyr/rtio/rtio.h>, zephyr_rtio.c, zephyr_mem_blocks.c,
 * zephyr_rtio_workq.c) on the fake kernel (tests/zephyr_fakes), with a
 * context of imu_stream.c's sizes and a fake iodev.  It checks what the
 * port relies on, as Zephyr's RTIO does it:
 *
 * - the memory pool: block size, the largest run from max down to min,
 *   binding to the submission, first fit, exhaustion and release;
 * - submissions: the pool of 4, FIFO reuse, handles and their generations;
 * - completions: order, the 16 entries, a dropped one counted in xcqcnt
 *   (and its buffer left allocated, as in Zephyr), the consume semaphore;
 * - multishot: back to the iodev before the completion is posted, the
 *   buffer binding cleared, the handle kept; an error or a cancellation
 *   frees the submission and its buffer and posts no buffer;
 * - cancellation: the mark, the iodev's hook, stale and invalid handles,
 *   a submission canceled before rtio_submit() gets no completion;
 * - the RTIO work queue: 4 requests, run in order.
 */
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/rtio/work.h>

#include "zephyr_kernel_host.h"

/* imu_stream.c:64 */
RTIO_DEFINE_WITH_MEMPOOL(test_rtio, 4, 16, 128, 64, 4);

static struct rtio_iodev_sqe *held[32];
static int nheld, submits, cancels;
static rtio_sqe_handle_t cancel_handle;

static void fake_submit(struct rtio_iodev_sqe *sqe)
{
	submits++;
	assert(nheld < 32);
	held[nheld++] = sqe;
}

static void fake_cancel(const struct rtio_iodev *iodev, struct rtio *r, rtio_sqe_handle_t h)
{
	(void)iodev;
	assert(r == &test_rtio);
	cancels++;
	cancel_handle = h;
}

static const struct rtio_iodev_api fake_api = {.submit = fake_submit, .cancel = fake_cancel};
RTIO_IODEV_DEFINE(fake_iodev, &fake_api, NULL);

static struct rtio_iodev_sqe *take(void)
{
	assert(nheld > 0);
	struct rtio_iodev_sqe *s = held[0];

	memmove(held, held + 1, (size_t)(nheld - 1) * sizeof(held[0]));
	nheld--;
	return s;
}

static size_t used_blocks(void)
{
	size_t n = 0;

	for (int i = 0; i < 128; i++) {
		n += (test_rtio.block_pool->bitmap[i / 32] >> (i % 32)) & 1U;
	}
	return n;
}

static rtio_sqe_handle_t start_multishot(void *userdata)
{
	struct rtio_sqe *sqe = rtio_sqe_acquire(&test_rtio);
	rtio_sqe_handle_t h;

	assert(sqe != NULL);
	h = rtio_sqe_handle(&test_rtio, sqe);
	rtio_sqe_prep_read_multishot(sqe, &fake_iodev, RTIO_PRIO_NORM, userdata);
	assert(rtio_submit(&test_rtio, 0) == 0);
	return h;
}

static void test_pool_and_multishot(void)
{
	uint8_t *buf, *again, *cbuf;
	uint32_t len, again_len, clen;
	struct rtio_cqe *cqe;
	struct rtio_iodev_sqe *s;
	rtio_sqe_handle_t h;

	assert(rtio_mempool_block_size(&test_rtio) == 64);
	assert(rtio_sqe_acquirable(&test_rtio) == 4);
	h = start_multishot((void *)0x55);
	assert(submits == 1 && nheld == 1);
	s = take();
	assert(s->r == &test_rtio && rtio_iodev_sqe_from_handle(&test_rtio, h) == s);
	assert(rtio_sqe_acquirable(&test_rtio) == 3);
	/* The largest run first: max 3648 bytes = 57 blocks. */
	assert(rtio_sqe_rx_buf(s, 960, 3648, &buf, &len) == 0);
	assert(buf == test_rtio.block_pool->buffer && len == 57 * 64 && used_blocks() == 57);
	/* Bound: the same buffer again, or -ENOMEM if it is too small. */
	assert(rtio_sqe_rx_buf(s, 960, 3648, &again, &again_len) == 0);
	assert(again == buf && again_len == len);
	assert(rtio_sqe_rx_buf(s, 4000, 4096, &again, &again_len) == -ENOMEM);
	/* Success: back to the iodev first, then the completion. */
	rtio_iodev_sqe_ok(s, 0);
	assert(submits == 2 && nheld == 1 && held[0] == s);
	assert(s->rt.rx_bind.buf == NULL && s->rt.rx_bind.buf_len == 0);
	assert(rtio_iodev_sqe_from_handle(&test_rtio, h) == s);
	assert(k_sem_count_get(test_rtio.consume_sem) == 1);
	cqe = rtio_cqe_consume(&test_rtio);
	assert(cqe && cqe->result == 0 && cqe->userdata == (void *)0x55);
	assert(rtio_cqe_get_mempool_buffer(&test_rtio, cqe, &cbuf, &clen) == 0);
	assert(cbuf == buf && clen == len);
	rtio_cqe_release(&test_rtio, cqe);
	assert(rtio_cqe_consume(&test_rtio) == NULL);
	/* The next shot gets the next free run, while the first is held. */
	s = take();
	assert(rtio_sqe_rx_buf(s, 960, 3648, &again, &again_len) == 0);
	assert(again == buf + 57 * 64 && again_len == 57 * 64 && used_blocks() == 114);
	rtio_iodev_sqe_ok(s, 0);
	s = take();
	/* 14 blocks left: less than 960 bytes (15 blocks), so -ENOMEM. */
	assert(rtio_sqe_rx_buf(s, 960, 3648, &again, &again_len) == -ENOMEM);
	assert(rtio_sqe_rx_buf(s, 64, 3648, &again, &again_len) == 0 && again_len == 14 * 64);
	rtio_release_buffer(&test_rtio, again, again_len);
	s->rt.rx_bind.buf = NULL;
	s->rt.rx_bind.buf_len = 0;
	/* Releasing the first frees its run for the first fit. */
	rtio_release_buffer(&test_rtio, buf, len);
	assert(used_blocks() == 57);
	assert(rtio_sqe_rx_buf(s, 960, 3648, &again, &again_len) == 0 && again == buf);
	/* An error ends the multishot: buffer and submission freed, no buffer
	 * in the completion, the handle no longer resolves.
	 */
	rtio_iodev_sqe_err(s, -EIO);
	assert(nheld == 0 && submits == 3);
	assert(rtio_iodev_sqe_from_handle(&test_rtio, h) == NULL);
	assert(rtio_sqe_acquirable(&test_rtio) == 4);
	cqe = rtio_cqe_consume(&test_rtio);
	assert(cqe && cqe->result == 0);
	assert(rtio_cqe_get_mempool_buffer(&test_rtio, cqe, &cbuf, &clen) == 0);
	assert(cbuf == buf + 57 * 64);
	rtio_cqe_release(&test_rtio, cqe);
	rtio_release_buffer(&test_rtio, cbuf, clen);
	cqe = rtio_cqe_consume(&test_rtio);
	assert(cqe && cqe->result == -EIO);
	assert(rtio_cqe_get_mempool_buffer(&test_rtio, cqe, &cbuf, &clen) == -EINVAL);
	rtio_cqe_release(&test_rtio, cqe);
	assert(used_blocks() == 0);
}

static void test_cancel(void)
{
	struct rtio_iodev_sqe *s;
	struct rtio_cqe *cqe;
	rtio_sqe_handle_t h, stale;
	uint8_t *buf;
	uint32_t len;

	/* The handle of a recycled slot no longer resolves or cancels. */
	h = start_multishot(NULL);
	s = take();
	assert(rtio_sqe_rx_buf(s, 64, 128, &buf, &len) == 0);
	assert(rtio_sqe_cancel(&test_rtio, RTIO_SQE_HANDLE_INVALID) == 0 && cancels == 0);
	assert(rtio_iodev_sqe_from_handle(&test_rtio, RTIO_SQE_HANDLE_INVALID) == NULL);
	assert(!rtio_iodev_sqe_is_canceled(s));
	assert(rtio_sqe_cancel(&test_rtio, h) == 0);
	assert(rtio_iodev_sqe_is_canceled(s) && cancels == 1 && cancel_handle == h);
	/* The iodev ends it: -ECANCELED, nothing kept. */
	rtio_iodev_sqe_err(s, -ECANCELED);
	assert(rtio_iodev_sqe_from_handle(&test_rtio, h) == NULL && used_blocks() == 0);
	cqe = rtio_cqe_consume(&test_rtio);
	assert(cqe && cqe->result == -ECANCELED);
	assert(rtio_cqe_get_mempool_buffer(&test_rtio, cqe, &buf, &len) == -EINVAL);
	rtio_cqe_release(&test_rtio, cqe);
	stale = h;
	assert(rtio_sqe_cancel(&test_rtio, stale) == 0 && cancels == 1);

	/* A success on a canceled multishot frees it too: result 0, no buffer. */
	h = start_multishot(NULL);
	s = take();
	assert(rtio_sqe_rx_buf(s, 64, 128, &buf, &len) == 0);
	assert(rtio_sqe_cancel(&test_rtio, h) == 0 && cancels == 2);
	rtio_iodev_sqe_ok(s, 0);
	assert(nheld == 0 && used_blocks() == 0);
	cqe = rtio_cqe_consume(&test_rtio);
	assert(cqe && cqe->result == 0 && cqe->flags == 0);
	rtio_cqe_release(&test_rtio, cqe);

	/* A shot, then an error: the shot's buffer is the consumer's, the
	 * error's completion comes after it.
	 */
	h = start_multishot(NULL);
	s = take();
	assert(rtio_sqe_rx_buf(s, 64, 128, &buf, &len) == 0);
	rtio_iodev_sqe_ok(s, 0);
	s = take();
	rtio_iodev_sqe_err(s, -EIO);
	cqe = rtio_cqe_consume(&test_rtio);
	assert(cqe && cqe->result == 0);
	assert(rtio_cqe_get_mempool_buffer(&test_rtio, cqe, &buf, &len) == 0 && buf != NULL);
	rtio_cqe_release(&test_rtio, cqe);
	rtio_release_buffer(&test_rtio, buf, len);
	cqe = rtio_cqe_consume(&test_rtio);
	assert(cqe && cqe->result == -EIO);
	rtio_cqe_release(&test_rtio, cqe);
	assert(used_blocks() == 0);

	/* Canceled before rtio_submit(): it never reaches the iodev and posts
	 * nothing.
	 */
	struct rtio_sqe *sqe = rtio_sqe_acquire(&test_rtio);

	h = rtio_sqe_handle(&test_rtio, sqe);
	rtio_sqe_prep_read_with_pool(sqe, &fake_iodev, RTIO_PRIO_NORM, NULL);
	assert(rtio_sqe_cancel(&test_rtio, h) == 0 && cancels == 3);
	submits = 0;
	assert(rtio_submit(&test_rtio, 0) == 0);
	assert(submits == 0 && nheld == 0 && rtio_cqe_consume(&test_rtio) == NULL);
	assert(rtio_iodev_sqe_from_handle(&test_rtio, h) == NULL);
	assert(rtio_sqe_acquirable(&test_rtio) == 4);
}

static void test_completions(void)
{
	struct rtio_sqe *sqes[5];
	struct rtio_cqe *cqe;
	uint8_t *buf;
	uint32_t len;
	atomic_val_t count0 = atomic_get(&test_rtio.cq_count);

	/* Four submissions, then none: the pool is 4. */
	for (int i = 0; i < 4; i++) {
		sqes[i] = rtio_sqe_acquire(&test_rtio);
		assert(sqes[i] != NULL);
	}
	assert(rtio_sqe_acquire(&test_rtio) == NULL && rtio_sqe_acquirable(&test_rtio) == 0);
	for (int i = 0; i < 4; i++) {
		rtio_sqe_prep_read_with_pool(sqes[i], &fake_iodev, RTIO_PRIO_NORM,
					     (void *)(uintptr_t)(i + 1));
	}
	submits = 0;
	assert(rtio_submit(&test_rtio, 0) == 0 && submits == 4);
	/* Recycled in FIFO order: a new handle for the same slot. */
	struct rtio_iodev_sqe *first = take();
	rtio_sqe_handle_t old = rtio_sqe_handle(&test_rtio, &first->sqe);

	rtio_iodev_sqe_ok(first, 0);
	for (int i = 0; i < 3; i++) {
		rtio_iodev_sqe_ok(take(), 0);
	}
	assert(rtio_iodev_sqe_from_handle(&test_rtio, old) == NULL);
	/* 17 one-shot reads with buffers and nobody consuming: 16 completions
	 * kept in order, the 17th dropped and counted; its buffer stays
	 * allocated (nobody can release it), as in Zephyr.
	 */
	for (int i = 0; i < 13; i++) {
		struct rtio_sqe *sqe = rtio_sqe_acquire(&test_rtio);
		struct rtio_iodev_sqe *s;

		assert(sqe != NULL);
		rtio_sqe_prep_read_with_pool(sqe, &fake_iodev, RTIO_PRIO_NORM,
					     (void *)(uintptr_t)(i + 5));
		assert(rtio_submit(&test_rtio, 0) == 0);
		s = take();
		assert(rtio_sqe_rx_buf(s, 64, 64, &buf, &len) == 0);
		rtio_iodev_sqe_ok(s, 7);
	}
	assert(atomic_get(&test_rtio.xcqcnt) == 1);
	assert(atomic_get(&test_rtio.cq_count) == count0 + 17);
	assert(k_sem_count_get(test_rtio.consume_sem) == 16);
	assert(used_blocks() == 13);
	for (int i = 1; i <= 16; i++) {
		cqe = rtio_cqe_consume(&test_rtio);
		assert(cqe && cqe->userdata == (void *)(uintptr_t)i);
		assert(cqe->result == (i <= 4 ? 0 : 7));
		assert(rtio_cqe_get_mempool_buffer(&test_rtio, cqe, &buf, &len) == 0);
		assert(i <= 4 ? buf == NULL && len == 0 : buf != NULL && len == 64);
		rtio_cqe_release(&test_rtio, cqe);
		rtio_release_buffer(&test_rtio, buf, len);
	}
	assert(rtio_cqe_consume(&test_rtio) == NULL);
	assert(used_blocks() == 1);
	/* Give the leaked block back for the next tests. */
	for (int i = 0; i < 128; i++) {
		if ((test_rtio.block_pool->bitmap[i / 32] >> (i % 32)) & 1U) {
			rtio_release_buffer(&test_rtio, test_rtio.block_pool->buffer + i * 64, 64);
		}
	}
	assert(used_blocks() == 0);
	/* Double free and foreign pointers are refused by the pool. */
	assert(sys_mem_blocks_free_contiguous(test_rtio.block_pool,
					      test_rtio.block_pool->buffer, 1) == -EFAULT);
	assert(sys_mem_blocks_free_contiguous(test_rtio.block_pool, &len, 1) == -EFAULT);
	assert(rtio_flush_completion_queue(&test_rtio) == 0);
}

static int ran[8], nran;

static void work_handler(struct rtio_iodev_sqe *sqe)
{
	ran[nran++] = (int)(uintptr_t)sqe;
}

static void test_workq(void)
{
	struct rtio_work_req *reqs[5];

	for (int i = 0; i < 4; i++) {
		reqs[i] = rtio_work_req_alloc();
		assert(reqs[i] != NULL);
	}
	assert(rtio_work_req_alloc() == NULL && rtio_work_req_used_count_get() == 4);
	for (int i = 0; i < 4; i++) {
		rtio_work_req_submit(reqs[i], (struct rtio_iodev_sqe *)(uintptr_t)(i + 1),
				     work_handler);
	}
	assert(nran == 0);
	while (rtio_workq_process()) {
	}
	assert(nran == 4 && ran[0] == 1 && ran[3] == 4 && rtio_work_req_used_count_get() == 0);
	reqs[0] = rtio_work_req_alloc();
	rtio_work_req_submit(reqs[0], NULL, work_handler);
	assert(rtio_work_req_used_count_get() == 0 && !rtio_workq_process());
}

int main(void)
{
	test_pool_and_multishot();
	test_cancel();
	test_completions();
	test_workq();
	return 0;
}
