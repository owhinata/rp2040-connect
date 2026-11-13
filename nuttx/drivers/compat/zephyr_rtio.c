/* SPDX-License-Identifier: Apache-2.0 */
/* The RTIO executor: Zephyr's subsys/rtio/rtio_executor.c (Zephyr v4.5.0)
 * without chained submissions, transactions and the executor's own
 * operations (see <zephyr/rtio/rtio.h>).  OS-independent.  Part of the
 * small Zephyr compatibility layer of the port (nuttx/drivers/compat).
 *
 * - rtio_executor_submit(): every acquired submission, in order, goes to
 *   its iodev's submit function in the caller's context; one canceled
 *   before submission gets no completion.
 * - On success a multishot submission is handed straight back to its iodev
 *   (its pool buffer binding cleared first), then its completion is posted
 *   with the buffer; on an error or a cancellation its buffer and the
 *   submission are freed and the completion carries no buffer.
 * - A one-shot submission posts its completion (unless canceled or
 *   NO_RESPONSE) and is freed.
 */

#ifdef __NuttX__
#  include <nuttx/config.h>
#endif

#include <errno.h>
#include <stdbool.h>

#include <zephyr/rtio/rtio.h>
#include <zephyr/sys/__assert.h>

static inline void rtio_iodev_submit(struct rtio_iodev_sqe *iodev_sqe,
                                     int last_result)
{
  (void)last_result;

  if (rtio_iodev_sqe_is_canceled(iodev_sqe))
    {
      rtio_iodev_sqe_err(iodev_sqe, -ECANCELED);
      return;
    }

  /* No iodev means it is an executor specific operation, none of which
   * the port has (rtio_executor_op(): -EINVAL).
   */

  if (iodev_sqe->sqe.iodev == NULL)
    {
      rtio_iodev_sqe_err(iodev_sqe, -EINVAL);
      return;
    }

  iodev_sqe->sqe.iodev->api->submit(iodev_sqe);
}

void rtio_executor_submit(struct rtio *r)
{
  struct mpsc_node *node = mpsc_pop(&r->sq);

  while (node != NULL)
    {
      struct rtio_iodev_sqe *iodev_sqe =
        CONTAINER_OF(node, struct rtio_iodev_sqe, q);

      __ASSERT((iodev_sqe->sqe.flags &
                (RTIO_SQE_TRANSACTION | RTIO_SQE_CHAINED)) == 0,
               "chains and transactions are not supported");

      /* If this submission was cancelled before submit, then generate no
       * response
       */

      if (rtio_iodev_sqe_is_canceled(iodev_sqe))
        {
          iodev_sqe->sqe.flags |= RTIO_SQE_NO_RESPONSE;
        }

      iodev_sqe->r = r;
      iodev_sqe->next = NULL;

      rtio_iodev_submit(iodev_sqe, 0);

      node = mpsc_pop(&r->sq);
    }
}

static inline void rtio_executor_handle_multishot(struct rtio_iodev_sqe *iodev_sqe,
                                                  int result, bool is_ok)
{
  struct rtio *r = iodev_sqe->r;
  const bool is_canceled = rtio_iodev_sqe_is_canceled(iodev_sqe);
  const bool uses_mempool =
    FIELD_GET(RTIO_SQE_MEMPOOL_BUFFER, iodev_sqe->sqe.flags) == 1;
  const bool requires_response =
    FIELD_GET(RTIO_SQE_NO_RESPONSE, iodev_sqe->sqe.flags) == 0;
  uint32_t cqe_flags = rtio_cqe_compute_flags(iodev_sqe);
  void *userdata = iodev_sqe->sqe.userdata;

  /* We're releasing resources when erroring as an error handling scheme of
   * multi-shot submissions by requiring to stop re-submitting if something
   * goes wrong.
   */

  if (is_canceled || !is_ok)
    {
      rtio_release_buffer(r, iodev_sqe->rt.rx_bind.buf,
                          iodev_sqe->rt.rx_bind.buf_len);
      rtio_sqe_pool_free(r->sqe_pool, iodev_sqe);

      /* The blocks are back in the pool, so do not hand them out again */

      cqe_flags = 0;
    }
  else
    {
      /* Request was not canceled, re-arm it */

      if (iodev_sqe->sqe.op == RTIO_OP_RX && uses_mempool)
        {
          /* Reset the buffer info so the next request can get a new one */

          iodev_sqe->rt.rx_bind.buf = NULL;
          iodev_sqe->rt.rx_bind.buf_len = 0;
        }

      /* Straight back to the iodev. */

      rtio_iodev_submit(iodev_sqe, 0);
    }

  if (requires_response)
    {
      rtio_cqe_submit(r, result, userdata, cqe_flags);
    }
}

static inline void rtio_executor_handle_oneshot(struct rtio_iodev_sqe *iodev_sqe,
                                                int last_result, bool is_ok)
{
  const bool is_canceled = rtio_iodev_sqe_is_canceled(iodev_sqe);
  struct rtio_iodev_sqe *curr = iodev_sqe;
  struct rtio *r = iodev_sqe->r;
  void *userdata = curr->sqe.userdata;
  uint32_t cqe_flags = rtio_cqe_compute_flags(curr);
  uint32_t sqe_flags = curr->sqe.flags;

  (void)is_ok;

  if (!is_canceled && FIELD_GET(RTIO_SQE_NO_RESPONSE, sqe_flags) == 0)
    {
      /* Generate a result back to the client if need be. */

      rtio_cqe_submit(r, last_result, userdata, cqe_flags);
    }

  rtio_sqe_pool_free(r->sqe_pool, curr);
}

static inline void rtio_executor_done(struct rtio_iodev_sqe *iodev_sqe,
                                      int result, bool is_ok)
{
  const bool is_multishot =
    FIELD_GET(RTIO_SQE_MULTISHOT, iodev_sqe->sqe.flags) == 1;

  if (is_multishot)
    {
      rtio_executor_handle_multishot(iodev_sqe, result, is_ok);
    }
  else
    {
      rtio_executor_handle_oneshot(iodev_sqe, result, is_ok);
    }
}

void rtio_executor_ok(struct rtio_iodev_sqe *iodev_sqe, int result)
{
  rtio_executor_done(iodev_sqe, result, true);
}

void rtio_executor_err(struct rtio_iodev_sqe *iodev_sqe, int result)
{
  rtio_executor_done(iodev_sqe, result, false);
}
