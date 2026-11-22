/* SPDX-License-Identifier: Apache-2.0 */
/* The parts of <zephyr/kernel.h> that start threads and what Zephyr starts
 * at boot, on the OS's start of a thread (z_thread_static_start()): the
 * thread of a work queue (k_work_queue_start(), kernel/work.c
 * work_queue_main() on zephyr_work.c's z_work_queue_process()), the system
 * work queue k_sys_work_q, zephyr_compat_initialize() of
 * "zephyr_compat.h", and k_uptime_get() on the cycle counter.  Part of the
 * small Zephyr compatibility layer of the port, its OS-independent part
 * (common/compat).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/work.h>

#include "priorities.h"

/* Work queues ------------------------------------------------------------*/

/* The thread of a work queue: kernel/work.c work_queue_main(). */

static void work_queue_main(void *p1, void *p2, void *p3)
{
  struct k_work_q *queue = p1;
  bool yield;

  (void)p2;
  (void)p3;

  for (; ; )
    {
      if (z_work_queue_process(queue, &yield))
        {
          if (yield)
            {
              k_yield();
            }

          continue;
        }

      /* Nothing pending: sleep until the next submission. */

      (void)Z_SEM_TAKE_FOREVER(&queue->notify);
    }
}

/* The queue keeps its thread's start data (z_thread_static_start() may
 * read it in the new thread).  Zephyr's k_work_queue_start() cannot fail;
 * a thread that did not start is the OS's assertion (Z_OS_ASSERT()), and
 * the queue's thread stays 0.
 */

void k_work_queue_start(struct k_work_q *queue, k_thread_stack_t *stack,
                        size_t stack_size, int prio,
                        const struct k_work_queue_config *cfg)
{
  int ret;

  z_work_queue_init(queue);
  queue->flags = BIT(K_WORK_QUEUE_STARTED_BIT);
  if (cfg != NULL && cfg->no_yield)
    {
      queue->flags |= BIT(K_WORK_QUEUE_NO_YIELD_BIT);
    }

  queue->thread_start = (struct z_thread_static)
    {
      .name = cfg != NULL && cfg->name != NULL ? cfg->name : "workq",
      .stack = stack,
      .stack_size = stack_size,
      .entry = work_queue_main,
      .p1 = queue,
      .prio = prio,
      .tid = &queue->thread,
    };

  ret = z_thread_static_start(&queue->thread_start);
  Z_OS_ASSERT(ret == 0);
  (void)ret;
}

/* Time -------------------------------------------------------------------*/

int64_t k_uptime_get(void)
{
  return Z_UPTIME_MS(k_cycle_get_64());
}

/* What Zephyr starts at boot ---------------------------------------------*/

struct k_work_q k_sys_work_q;

static K_THREAD_STACK_DEFINE(sys_work_q_stack, NANO_STACK_SYSWORKQ);

int zephyr_compat_initialize(void)
{
  static const struct k_work_queue_config cfg =
    {
      .name = "sysworkq",
      .no_yield = false,
    };

  int ret;

  /* kernel/system_work_q.c k_sys_work_q_init() (POST_KERNEL). */

  k_work_queue_start(&k_sys_work_q, sys_work_q_stack,
                     K_THREAD_STACK_SIZEOF(sys_work_q_stack),
                     NANO_PRIO_SYSWORKQ, &cfg);
  if (k_work_queue_thread_get(&k_sys_work_q) == (k_tid_t)0)
    {
      return -ENOMEM;
    }

  /* subsys/rtio/rtio_workq.c static_init() (POST_KERNEL). */

  ret = rtio_workq_initialize();
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_NANO_LSM6DSO_STREAM
  /* drivers/gpio/gpio_rpi_pico.c (POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY
   * 40): the gpio0 device of zephyr_gpio.c, which the LSM6DSO's FIFO stream
   * uses for INT1 (a build without the stream has no use for it; the NuttX
   * build compiles the layer's kernel part only with the stream).  Its bank
   * interrupt is added with the first pin's (the OS's nano_gpio.c).
   */

  return device_init(DEVICE_DT_GET(DT_NODELABEL(gpio0)));
#else
  return 0;
#endif
}
