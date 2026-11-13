/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's RTIO (<zephyr/rtio/rtio.h> with sqe.h, cqe.h and
 * iodev.h, Zephyr v4.5.0 as in zephyr/zephyr) that the sensor API, the
 * ported LSM6DSO stream and the IMU service use, with Zephyr's names,
 * types, flags and code.  The Zephyr build has CONFIG_RTIO_SUBMIT_SEM,
 * CONFIG_RTIO_CONSUME_SEM and CONFIG_RTIO_SYS_MEM_BLOCKS on and
 * CONFIG_RTIO_CQE_CALLBACK, CONFIG_USERSPACE off; this header is that
 * configuration.
 *
 * Kept (the functions are Zephyr's, written out):
 * - Contexts: RTIO_DEFINE_WITH_MEMPOOL() (the sizes come from the caller,
 *   e.g. imu_stream.c's 4 submissions, 16 completions, 128 blocks of 64
 *   bytes), RTIO_DEFINE().
 * - Submissions: rtio_sqe_acquire(), rtio_sqe_acquirable(), the prep
 *   functions for reads (rtio_sqe_prep_read(), _with_pool(),
 *   _multishot()), rtio_submit() (the submit semaphore), handles
 *   (rtio_sqe_handle(), rtio_iodev_sqe_from_handle(), the generation
 *   bumped on every free), cancellation (rtio_sqe_cancel(),
 *   rtio_iodev_sqe_is_canceled(), the iodev's cancel hook).
 * - The executor (zephyr_rtio.c): rtio_iodev_sqe_ok()/_err(); a multishot
 *   submission goes straight back to its iodev on success before its
 *   completion is posted, and is freed with its buffer on an error or a
 *   cancellation.
 * - Completions: rtio_cqe_submit() (a completion that finds no free entry
 *   is dropped and counted in xcqcnt), rtio_cqe_consume() (takes the
 *   consume semaphore without waiting), rtio_cqe_consume_block(),
 *   rtio_cqe_release(), rtio_flush_completion_queue().
 * - Memory pool: rtio_sqe_rx_buf() (the largest run of blocks from
 *   max_buf_len down to min_buf_len, else -ENOMEM; bound to the
 *   submission), rtio_cqe_get_mempool_buffer() (block index and count in
 *   the completion's flags), rtio_release_buffer(),
 *   rtio_mempool_block_size().
 * Not kept: chained submissions and transactions, the executor's own
 *   operations (RTIO_OP_CALLBACK, _AWAIT, _DELAY; a submission without an
 *   iodev completes with -EINVAL), the write, tiny write, transceive and
 *   I2C/I3C operations, rtio_sqe_copy_in*(), rtio_cqe_copy_out(), rtio pools, the completion
 *   callback, user mode and access grants, placement in special memory,
 *   the deprecated RTIO_SQE_CANCELED flag.
 *
 * Differences: the queues (struct mpsc) mask interrupts instead of being
 * lock-free, and the free lists of a context's pools are filled on their
 * first use instead of at boot (rtio_init(), SYS_INIT POST_KERNEL 0); the
 * order in which entries are handed out is the same.  The memory pool is
 * zephyr_mem_blocks.c.
 *
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat, see nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_RTIO_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_RTIO_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/mem_blocks.h>
#include <zephyr/sys/mpsc_lockfree.h>
#include <zephyr/sys/util.h>

struct rtio;
struct rtio_iodev;
struct rtio_iodev_sqe;
struct rtio_sqe;

/* sqe.h --------------------------------------------------------------------*/

#define RTIO_PRIO_LOW  0U
#define RTIO_PRIO_NORM 127U
#define RTIO_PRIO_HIGH 255U

#define RTIO_SQE_CHAINED        BIT(0)   /* not supported */
#define RTIO_SQE_TRANSACTION    BIT(1)   /* not supported */
#define RTIO_SQE_MEMPOOL_BUFFER BIT(2)
#define RTIO_SQE_MULTISHOT      BIT(4)
#define RTIO_SQE_NO_RESPONSE    BIT(5)

#define RTIO_OP_NOP 0
#define RTIO_OP_RX  (RTIO_OP_NOP + 1)
#define RTIO_OP_TX  (RTIO_OP_RX + 1)

typedef uint32_t rtio_sqe_handle_t;

/* Resolving it always yields NULL; cancelling it does nothing. */

#define RTIO_SQE_HANDLE_INVALID ((rtio_sqe_handle_t)UINT32_MAX)

struct rtio_sqe
{
  uint8_t op;
  uint8_t prio;
  uint16_t flags;
  uint32_t iodev_flags;
  const struct rtio_iodev *iodev;
  void *userdata;
  union
    {
      struct
        {
          uint32_t buf_len;
          const uint8_t *buf;
        } tx;

      struct
        {
          uint32_t buf_len;
          uint8_t *buf;
        } rx;
    };
};

/* The status word: generation | ALLOCD | CANCELED. */

#define RTIO_SQE_GEN_MASK        GENMASK(15, 0)
#define RTIO_SQE_ALLOCD          BIT(16)
#define RTIO_SQE_STATUS_CANCELED BIT(17)

static inline uint16_t rtio_sqe_handle_index(rtio_sqe_handle_t handle)
{
  return (uint16_t)(handle >> 16);
}

static inline uint16_t rtio_sqe_handle_generation(rtio_sqe_handle_t handle)
{
  return (uint16_t)(handle & RTIO_SQE_GEN_MASK);
}

struct rtio_iodev_sqe
{
  struct rtio_sqe sqe;
  struct mpsc_node q;
  struct rtio_iodev_sqe *next;
  struct rtio *r;
  atomic_t status;
  union
    {
      struct
        {
          uint8_t *buf;
          uint32_t buf_len;
        } rx_bind;
    } rt;
};

static inline void _rtio_sqe_prep_read(struct rtio_sqe *sqe,
                                       const struct rtio_iodev *iodev,
                                       int8_t prio, uint8_t *buf,
                                       uint32_t len, void *userdata)
{
  memset(sqe, 0, sizeof(struct rtio_sqe));
  sqe->op = RTIO_OP_RX;
  sqe->prio = prio;
  sqe->iodev = iodev;
  sqe->rx.buf_len = len;
  sqe->rx.buf = buf;
  sqe->userdata = userdata;
}

static inline void rtio_sqe_prep_read(struct rtio_sqe *sqe,
                                      const struct rtio_iodev *iodev,
                                      int8_t prio, uint8_t *buf,
                                      uint32_t len, void *userdata)
{
  __ASSERT_NO_MSG(len > 0 && buf != NULL);

  _rtio_sqe_prep_read(sqe, iodev, prio, buf, len, userdata);
}

static inline void rtio_sqe_prep_read_with_pool(struct rtio_sqe *sqe,
                                                const struct rtio_iodev *iodev,
                                                int8_t prio, void *userdata)
{
  _rtio_sqe_prep_read(sqe, iodev, prio, NULL, 0, userdata);
  sqe->flags = RTIO_SQE_MEMPOOL_BUFFER;
}

/* Keeps producing completions until it is canceled or fails. */

static inline void rtio_sqe_prep_read_multishot(struct rtio_sqe *sqe,
                                                const struct rtio_iodev *iodev,
                                                int8_t prio, void *userdata)
{
  rtio_sqe_prep_read_with_pool(sqe, iodev, prio, userdata);
  sqe->flags |= RTIO_SQE_MULTISHOT;
}

static inline struct rtio_iodev_sqe *
rtio_iodev_sqe_next(const struct rtio_iodev_sqe *iodev_sqe)
{
  return iodev_sqe->next;
}

/* Cancellation requested?  A driver serving a long submission (a stream)
 * checks this between steps.
 */

static inline bool
rtio_iodev_sqe_is_canceled(const struct rtio_iodev_sqe *iodev_sqe)
{
  return (atomic_get(&iodev_sqe->status) & RTIO_SQE_STATUS_CANCELED) != 0;
}

static inline void rtio_iodev_sqe_set_canceled(struct rtio_iodev_sqe *iodev_sqe)
{
  atomic_or(&iodev_sqe->status, RTIO_SQE_STATUS_CANCELED);
}

/* The pool of submissions.  Zephyr fills the free list at boot
 * (rtio_init()); here on the first allocation, in the same order.
 */

struct rtio_sqe_pool
{
  struct mpsc free_q;
  const uint16_t pool_size;
  uint16_t pool_free;
  struct rtio_iodev_sqe *pool;
  bool filled;
};

static inline void rtio_sqe_pool_fill(struct rtio_sqe_pool *pool)
{
  k_spinlock_key_t key = k_spin_lock(&pool->free_q.lock);
  bool fill = !pool->filled;

  pool->filled = true;
  k_spin_unlock(&pool->free_q.lock, key);
  if (fill)
    {
      for (int i = 0; i < pool->pool_size; i++)
        {
          mpsc_push(&pool->free_q, &pool->pool[i].q);
        }
    }
}

static inline struct rtio_iodev_sqe *
rtio_sqe_pool_alloc(struct rtio_sqe_pool *pool)
{
  struct mpsc_node *node;

  rtio_sqe_pool_fill(pool);
  node = mpsc_pop(&pool->free_q);
  if (node == NULL)
    {
      return NULL;
    }

  struct rtio_iodev_sqe *iodev_sqe =
    CONTAINER_OF(node, struct rtio_iodev_sqe, q);

  /* Reset the runtime scratch so per-op state starts clean. */

  memset(&iodev_sqe->rt, 0, sizeof(iodev_sqe->rt));
  iodev_sqe->next = NULL;

  /* Mark the slot live.  The generation persists from the previous free
   * so a handle captured against the prior occupant no longer matches.
   */

  atomic_or(&iodev_sqe->status, RTIO_SQE_ALLOCD);
  pool->pool_free--;
  return iodev_sqe;
}

static inline void rtio_sqe_pool_free(struct rtio_sqe_pool *pool,
                                      struct rtio_iodev_sqe *iodev_sqe)
{
  /* Bump the generation and clear ALLOCD. */

  atomic_val_t gen = (atomic_get(&iodev_sqe->status) + 1) &
                     RTIO_SQE_GEN_MASK;

  atomic_set(&iodev_sqe->status, gen);
  mpsc_push(&pool->free_q, &iodev_sqe->q);
  pool->pool_free++;
}

#define Z_RTIO_SQE_POOL_DEFINE(name, sz) \
  static struct rtio_iodev_sqe CONCAT(_sqe_pool_, name)[sz]; \
  static struct rtio_sqe_pool name = \
    { \
      .free_q = MPSC_INIT((name.free_q)), \
      .pool_size = (sz), \
      .pool_free = (sz), \
      .pool = CONCAT(_sqe_pool_, name), \
    }

/* cqe.h --------------------------------------------------------------------*/

#define RTIO_CQE_FLAG_MEMPOOL_BUFFER BIT(0)
#define RTIO_CQE_FLAG_GET(flags) FIELD_GET(GENMASK(7, 0), (flags))
#define RTIO_CQE_FLAG_MEMPOOL_GET_BLK_IDX(flags) \
  FIELD_GET(GENMASK(19, 8), (flags))
#define RTIO_CQE_FLAG_MEMPOOL_GET_BLK_CNT(flags) \
  FIELD_GET(GENMASK(31, 20), (flags))
#define RTIO_CQE_FLAG_PREP_MEMPOOL(blk_idx, blk_cnt) \
  (FIELD_PREP(GENMASK(7, 0), RTIO_CQE_FLAG_MEMPOOL_BUFFER) | \
   FIELD_PREP(GENMASK(19, 8), blk_idx) | FIELD_PREP(GENMASK(31, 20), blk_cnt))

struct rtio_cqe
{
  struct mpsc_node q;
  int32_t result;
  void *userdata;
  uint32_t flags;
};

struct rtio_cqe_pool
{
  struct mpsc free_q;
  const uint16_t pool_size;
  uint16_t pool_free;
  struct rtio_cqe *pool;
  bool filled;
};

static inline void rtio_cqe_pool_fill(struct rtio_cqe_pool *pool)
{
  k_spinlock_key_t key = k_spin_lock(&pool->free_q.lock);
  bool fill = !pool->filled;

  pool->filled = true;
  k_spin_unlock(&pool->free_q.lock, key);
  if (fill)
    {
      for (int i = 0; i < pool->pool_size; i++)
        {
          mpsc_push(&pool->free_q, &pool->pool[i].q);
        }
    }
}

static inline struct rtio_cqe *rtio_cqe_pool_alloc(struct rtio_cqe_pool *pool)
{
  struct mpsc_node *node;

  rtio_cqe_pool_fill(pool);
  node = mpsc_pop(&pool->free_q);
  if (node == NULL)
    {
      return NULL;
    }

  struct rtio_cqe *cqe = CONTAINER_OF(node, struct rtio_cqe, q);

  memset(cqe, 0, sizeof(struct rtio_cqe));
  pool->pool_free--;
  return cqe;
}

static inline void rtio_cqe_pool_free(struct rtio_cqe_pool *pool,
                                      struct rtio_cqe *cqe)
{
  mpsc_push(&pool->free_q, &cqe->q);
  pool->pool_free++;
}

#define Z_RTIO_CQE_POOL_DEFINE(name, sz) \
  static struct rtio_cqe CONCAT(_cqe_pool_, name)[sz]; \
  static struct rtio_cqe_pool name = \
    { \
      .free_q = MPSC_INIT((name.free_q)), \
      .pool_size = (sz), \
      .pool_free = (sz), \
      .pool = CONCAT(_cqe_pool_, name), \
    }

/* iodev.h ------------------------------------------------------------------*/

struct rtio_iodev_api
{
  /* Takes a submission; called in the submitter's context (rtio_submit())
   * or, for a multishot resubmission, in the completer's.
   */

  void (*submit)(struct rtio_iodev_sqe *iodev_sqe);

  /* Optional: abort a submission the iodev still holds. */

  void (*cancel)(const struct rtio_iodev *iodev, struct rtio *r,
                 rtio_sqe_handle_t handle);
};

struct rtio_iodev
{
  const struct rtio_iodev_api *api;
  void *data;
};

#define RTIO_IODEV_DEFINE(name, iodev_api, iodev_data) \
  struct rtio_iodev name = \
    { \
      .api = (iodev_api), \
      .data = (iodev_data), \
    }

/* rtio.h -------------------------------------------------------------------*/

struct rtio
{
  struct k_sem *submit_sem;     /* CONFIG_RTIO_SUBMIT_SEM */
  uint32_t submit_count;
  struct k_sem *consume_sem;    /* CONFIG_RTIO_CONSUME_SEM */
  atomic_t cq_count;            /* all completions */
  atomic_t xcqcnt;              /* completions dropped: no free CQE */
  struct rtio_sqe_pool *sqe_pool;
  struct rtio_cqe_pool *cqe_pool;
  struct sys_mem_blocks *block_pool;
  struct mpsc sq;
  struct mpsc cq;
};

#define Z_RTIO_DEFINE(name, _sqe_pool, _cqe_pool, _block_pool) \
  static K_SEM_DEFINE(CONCAT(_submit_sem_, name), 0, K_SEM_MAX_LIMIT); \
  static K_SEM_DEFINE(CONCAT(_consume_sem_, name), 0, K_SEM_MAX_LIMIT); \
  struct rtio name = \
    { \
      .submit_sem = &CONCAT(_submit_sem_, name), \
      .submit_count = 0, \
      .consume_sem = &CONCAT(_consume_sem_, name), \
      .cq_count = ATOMIC_INIT(0), \
      .xcqcnt = ATOMIC_INIT(0), \
      .sqe_pool = _sqe_pool, \
      .cqe_pool = _cqe_pool, \
      .block_pool = _block_pool, \
      .sq = MPSC_INIT((name.sq)), \
      .cq = MPSC_INIT((name.cq)), \
    }

#define RTIO_DEFINE(name, sq_sz, cq_sz) \
  Z_RTIO_SQE_POOL_DEFINE(CONCAT(name, _sqe_pool), sq_sz); \
  Z_RTIO_CQE_POOL_DEFINE(CONCAT(name, _cqe_pool), cq_sz); \
  Z_RTIO_DEFINE(name, &CONCAT(name, _sqe_pool), \
                &CONCAT(name, _cqe_pool), NULL)

static inline size_t rtio_mempool_block_size(const struct rtio *r)
{
  if (r == NULL || r->block_pool == NULL)
    {
      return 0;
    }

  return BIT(r->block_pool->info.blk_sz_shift);
}

static inline int rtio_block_pool_alloc(struct rtio *r, size_t min_sz,
                                        size_t max_sz, uint8_t **buf,
                                        uint32_t *buf_len)
{
  const uint32_t block_size = rtio_mempool_block_size(r);
  uint32_t bytes = max_sz;

  /* Not every context has a block pool and the block size may return 0 in
   * that case
   */

  if (block_size == 0)
    {
      return -ENOMEM;
    }

  do
    {
      size_t num_blks = DIV_ROUND_UP(bytes, block_size);
      int rc = sys_mem_blocks_alloc_contiguous(r->block_pool, num_blks,
                                               (void **)buf);

      if (rc == 0)
        {
          *buf_len = num_blks * block_size;
          return 0;
        }

      if (bytes <= block_size)
        {
          break;
        }

      bytes -= block_size;
    }
  while (bytes >= min_sz);

  return -ENOMEM;
}

static inline void rtio_block_pool_free(struct rtio *r, void *buf,
                                        uint32_t buf_len)
{
  /* Round up, a caller may name the bytes it used rather than the whole
   * buffer
   */

  size_t num_blks = DIV_ROUND_UP(buf_len,
                                 BIT(r->block_pool->info.blk_sz_shift));

  sys_mem_blocks_free_contiguous(r->block_pool, buf, num_blks);
}

#define Z_RTIO_BLOCK_POOL_DEFINE(name, blk_sz, blk_cnt, blk_align) \
  static uint8_t __aligned(WB_UP(blk_align)) \
  CONCAT(_block_pool_, name)[(blk_cnt) * WB_UP(blk_sz)]; \
  _SYS_MEM_BLOCKS_DEFINE_WITH_EXT_BUF(name, WB_UP(blk_sz), (blk_cnt), \
                                      CONCAT(_block_pool_, name), static)

/* sq_sz submissions, cq_sz completions, num_blks blocks of blk_size bytes
 * (a power of two) aligned to balign.
 */

#define RTIO_DEFINE_WITH_MEMPOOL(name, sq_sz, cq_sz, num_blks, blk_size, balign) \
  Z_RTIO_SQE_POOL_DEFINE(name##_sqe_pool, sq_sz); \
  Z_RTIO_CQE_POOL_DEFINE(name##_cqe_pool, cq_sz); \
  Z_RTIO_BLOCK_POOL_DEFINE(name##_block_pool, blk_size, num_blks, balign); \
  Z_RTIO_DEFINE(name, &name##_sqe_pool, &name##_cqe_pool, \
                &name##_block_pool)

static inline uint32_t rtio_sqe_acquirable(struct rtio *r)
{
  return r->sqe_pool->pool_free;
}

/* A free submission, queued for the next rtio_submit(); NULL if none. */

static inline struct rtio_sqe *rtio_sqe_acquire(struct rtio *r)
{
  struct rtio_iodev_sqe *iodev_sqe = rtio_sqe_pool_alloc(r->sqe_pool);

  if (iodev_sqe == NULL)
    {
      return NULL;
    }

  mpsc_push(&r->sq, &iodev_sqe->q);
  return &iodev_sqe->sqe;
}

static inline struct rtio_cqe *rtio_cqe_acquire(struct rtio *r)
{
  struct rtio_cqe *cqe = rtio_cqe_pool_alloc(r->cqe_pool);

  if (cqe == NULL)
    {
      return NULL;
    }

  memset(cqe, 0, sizeof(struct rtio_cqe));
  return cqe;
}

static inline void rtio_cqe_produce(struct rtio *r, struct rtio_cqe *cqe)
{
  mpsc_push(&r->cq, &cqe->q);
}

/* The oldest completion, or NULL at once if there is none.  Release it
 * with rtio_cqe_release().
 */

static inline struct rtio_cqe *rtio_cqe_consume(struct rtio *r)
{
  struct mpsc_node *node;
  struct rtio_cqe *cqe = NULL;

  if (k_sem_take(r->consume_sem, K_NO_WAIT) != 0)
    {
      return NULL;
    }

  node = mpsc_pop(&r->cq);
  if (node == NULL)
    {
      return NULL;
    }

  cqe = CONTAINER_OF(node, struct rtio_cqe, q);
  return cqe;
}

/* Waits for a completion. */

static inline struct rtio_cqe *rtio_cqe_consume_block(struct rtio *r)
{
  struct mpsc_node *node;
  struct rtio_cqe *cqe;

  k_sem_take(r->consume_sem, K_FOREVER);
  node = mpsc_pop(&r->cq);
  while (node == NULL)
    {
      node = mpsc_pop(&r->cq);
    }

  cqe = CONTAINER_OF(node, struct rtio_cqe, q);
  return cqe;
}

static inline void rtio_cqe_release(struct rtio *r, struct rtio_cqe *cqe)
{
  rtio_cqe_pool_free(r->cqe_pool, cqe);
}

static inline int rtio_flush_completion_queue(struct rtio *r)
{
  struct rtio_cqe *cqe;
  int res = 0;

  do
    {
      cqe = rtio_cqe_consume(r);
      if (cqe != NULL)
        {
          if ((cqe->result < 0) && (res == 0))
            {
              res = cqe->result;
            }

          rtio_cqe_release(r, cqe);
        }
    }
  while (cqe != NULL);

  return res;
}

static inline uint32_t rtio_cqe_compute_flags(struct rtio_iodev_sqe *iodev_sqe)
{
  uint32_t flags = 0;

  if (iodev_sqe->sqe.op == RTIO_OP_RX &&
      iodev_sqe->sqe.flags & RTIO_SQE_MEMPOOL_BUFFER)
    {
      struct rtio *r = iodev_sqe->r;
      struct sys_mem_blocks *mem_pool = r->block_pool;
      unsigned int blk_index = 0;
      unsigned int blk_count = 0;

      if (iodev_sqe->rt.rx_bind.buf)
        {
          blk_index = (iodev_sqe->rt.rx_bind.buf - mem_pool->buffer) >>
                      mem_pool->info.blk_sz_shift;
          blk_count = iodev_sqe->rt.rx_bind.buf_len >>
                      mem_pool->info.blk_sz_shift;
        }

      flags = RTIO_CQE_FLAG_PREP_MEMPOOL(blk_index, blk_count);
    }

  return flags;
}

/* The pool buffer of a completion: 0 (*buff NULL if it has none) or
 * -EINVAL if the submission did not use the pool.  Give the buffer back
 * with rtio_release_buffer().
 */

static inline int rtio_cqe_get_mempool_buffer(const struct rtio *r,
                                              struct rtio_cqe *cqe,
                                              uint8_t **buff,
                                              uint32_t *buff_len)
{
  if (RTIO_CQE_FLAG_GET(cqe->flags) == RTIO_CQE_FLAG_MEMPOOL_BUFFER)
    {
      unsigned int blk_idx = RTIO_CQE_FLAG_MEMPOOL_GET_BLK_IDX(cqe->flags);
      unsigned int blk_count = RTIO_CQE_FLAG_MEMPOOL_GET_BLK_CNT(cqe->flags);
      uint32_t blk_size = rtio_mempool_block_size(r);

      *buff_len = blk_count * blk_size;

      if (blk_count > 0)
        {
          *buff = r->block_pool->buffer + blk_idx * blk_size;

          __ASSERT_NO_MSG(*buff >= r->block_pool->buffer);
          __ASSERT_NO_MSG(*buff < r->block_pool->buffer +
                          blk_size * r->block_pool->info.num_blocks);
        }
      else
        {
          *buff = NULL;
        }

      return 0;
    }

  return -EINVAL;
}

/* zephyr_rtio.c (subsys/rtio/rtio_executor.c) */

void rtio_executor_submit(struct rtio *r);
void rtio_executor_ok(struct rtio_iodev_sqe *iodev_sqe, int result);
void rtio_executor_err(struct rtio_iodev_sqe *iodev_sqe, int result);

/* The iodev has finished a submission successfully.  A multishot
 * submission is handed back to its iodev before this returns.
 */

static inline void rtio_iodev_sqe_ok(struct rtio_iodev_sqe *iodev_sqe,
                                     int result)
{
  rtio_executor_ok(iodev_sqe, result);
}

/* The iodev has finished a submission with an error (or a cancellation,
 * -ECANCELED).  A multishot submission ends here.
 */

static inline void rtio_iodev_sqe_err(struct rtio_iodev_sqe *iodev_sqe,
                                      int result)
{
  rtio_executor_err(iodev_sqe, result);
}

/* Posts a completion: no free CQE drops it (xcqcnt).  Called by the
 * executor only.
 */

static inline void rtio_cqe_submit(struct rtio *r, int result,
                                   void *userdata, uint32_t flags)
{
  struct rtio_cqe *cqe = rtio_cqe_acquire(r);

  if (cqe == NULL)
    {
      atomic_inc(&r->xcqcnt);
    }
  else
    {
      cqe->result = result;
      cqe->userdata = userdata;
      cqe->flags = flags;
      rtio_cqe_produce(r, cqe);
      k_sem_give(r->consume_sem);
    }

  /* atomic_t isn't guaranteed to wrap correctly as it could be signed, so
   * we must resort to a cas loop.
   */

  atomic_t val;
  atomic_t new_val;

  do
    {
      val = atomic_get(&r->cq_count);
      new_val = (atomic_t)((uintptr_t)val + 1);
    }
  while (!atomic_cas(&r->cq_count, val, new_val));

  if (r->submit_count > 0)
    {
      r->submit_count--;
      if (r->submit_count == 0)
        {
          k_sem_give(r->submit_sem);
        }
    }
}

/* The read buffer of a submission: its own, or (pool submissions) the one
 * already bound to it, or a new run of blocks from max_buf_len down to
 * min_buf_len.  -ENOMEM if none fits.
 */

static inline int rtio_sqe_rx_buf(struct rtio_iodev_sqe *iodev_sqe,
                                  uint32_t min_buf_len,
                                  uint32_t max_buf_len,
                                  uint8_t **buf, uint32_t *buf_len)
{
  const struct rtio_sqe *sqe = &iodev_sqe->sqe;

  if (sqe->op == RTIO_OP_RX && sqe->flags & RTIO_SQE_MEMPOOL_BUFFER)
    {
      struct rtio *r = iodev_sqe->r;

      if (iodev_sqe->rt.rx_bind.buf != NULL)
        {
          if (iodev_sqe->rt.rx_bind.buf_len < min_buf_len)
            {
              return -ENOMEM;
            }

          *buf = iodev_sqe->rt.rx_bind.buf;
          *buf_len = iodev_sqe->rt.rx_bind.buf_len;
          return 0;
        }

      int rc = rtio_block_pool_alloc(r, min_buf_len, max_buf_len, buf,
                                     buf_len);

      if (rc == 0)
        {
          iodev_sqe->rt.rx_bind.buf = *buf;
          iodev_sqe->rt.rx_bind.buf_len = *buf_len;
          return 0;
        }

      return -ENOMEM;
    }

  if (sqe->rx.buf_len < min_buf_len)
    {
      return -ENOMEM;
    }

  *buf = sqe->rx.buf;
  *buf_len = sqe->rx.buf_len;
  return 0;
}

/* Gives a pool buffer back; ignores NULL and a context without a pool. */

static inline void rtio_release_buffer(struct rtio *r, void *buff,
                                       uint32_t buff_len)
{
  if (r == NULL || buff == NULL || r->block_pool == NULL || buff_len == 0)
    {
      return;
    }

  rtio_block_pool_free(r, buff, buff_len);
}

/* The handle of a submission: pool index and generation. */

static inline rtio_sqe_handle_t rtio_sqe_handle(const struct rtio *r,
                                                const struct rtio_sqe *sqe)
{
  const struct rtio_iodev_sqe *iodev_sqe =
    CONTAINER_OF(sqe, struct rtio_iodev_sqe, sqe);
  uint32_t blk_index = (uint32_t)(iodev_sqe - r->sqe_pool->pool);
  uint32_t gen = (uint32_t)(atomic_get(&iodev_sqe->status) &
                            RTIO_SQE_GEN_MASK);

  return (blk_index << 16) | gen;
}

/* The live submission a handle names, or NULL (out of range, free, or
 * recycled since).
 */

static inline struct rtio_iodev_sqe *
rtio_iodev_sqe_from_handle(struct rtio *r, rtio_sqe_handle_t handle)
{
  uint32_t blk_index = handle >> 16;
  uint32_t gen = handle & RTIO_SQE_GEN_MASK;

  if (blk_index >= r->sqe_pool->pool_size)
    {
      return NULL;
    }

  struct rtio_iodev_sqe *iodev_sqe = &r->sqe_pool->pool[blk_index];
  atomic_val_t status = atomic_get(&iodev_sqe->status);

  if ((status & RTIO_SQE_ALLOCD) == 0 ||
      (uint32_t)(status & RTIO_SQE_GEN_MASK) != gen)
    {
      return NULL;
    }

  return iodev_sqe;
}

/* Marks the submission canceled (if the handle still names it) and calls
 * the iodev's cancel hook.  Always 0.
 */

static inline int rtio_sqe_cancel(struct rtio *r, rtio_sqe_handle_t handle)
{
  uint16_t blk_index = rtio_sqe_handle_index(handle);
  uint16_t gen = rtio_sqe_handle_generation(handle);

  if (blk_index >= r->sqe_pool->pool_size)
    {
      return 0;
    }

  struct rtio_iodev_sqe *iodev_sqe = &r->sqe_pool->pool[blk_index];

  /* Validate identity and mark canceled in one atomic step. */

  while (true)
    {
      atomic_val_t status = atomic_get(&iodev_sqe->status);

      if ((status & RTIO_SQE_ALLOCD) == 0 ||
          (uint16_t)(status & RTIO_SQE_GEN_MASK) != gen)
        {
          return 0;
        }

      if (atomic_cas(&iodev_sqe->status, status,
                     status | RTIO_SQE_STATUS_CANCELED))
        {
          break;
        }
    }

  /* Give the iodev a chance to abort a submission it is already sitting
   * on, if the slot was not recycled in between.
   */

  const struct rtio_iodev *iodev = iodev_sqe->sqe.iodev;
  atomic_val_t status = atomic_get(&iodev_sqe->status);

  if ((status & RTIO_SQE_ALLOCD) == 0 ||
      (uint16_t)(status & RTIO_SQE_GEN_MASK) != gen)
    {
      return 0;
    }

  if (iodev != NULL && iodev->api->cancel != NULL)
    {
      iodev->api->cancel(iodev, r, handle);
    }

  return 0;
}

/* Hands the acquired submissions to their iodevs (in this context), then
 * waits for wait_count completions.
 */

static inline int rtio_submit(struct rtio *r, uint32_t wait_count)
{
  int res = 0;

  if (wait_count > 0)
    {
      k_sem_reset(r->submit_sem);
      r->submit_count = wait_count;
    }

  rtio_executor_submit(r);

  if (wait_count > 0)
    {
      res = k_sem_take(r->submit_sem, K_FOREVER);
      __ASSERT(res == 0, "semaphore was reset or timed out while waiting "
               "on completions!");
    }

  return res;
}

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_RTIO_RTIO_H */
