/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's struct mpsc (<zephyr/sys/mpsc_lockfree.h>) as RTIO uses it: a
 * first-in first-out queue of nodes, any context may push, one consumer
 * pops.  Zephyr's is lock-free (atomic exchange); this one masks
 * interrupts around the list operations (k_spin_lock()), which gives the
 * same order and is safe from any context on one CPU.  Part of the small
 * Zephyr compatibility layer of the port (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_MPSC_LOCKFREE_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_MPSC_LOCKFREE_H

#include <stddef.h>

#include <zephyr/spinlock.h>

struct mpsc_node
{
  struct mpsc_node *next;
};

struct mpsc
{
  struct mpsc_node *head;   /* popped first */
  struct mpsc_node *tail;   /* pushed last */
  struct k_spinlock lock;
};

/* Zephyr's MPSC_INIT(symbol) names the object; this one is just empty. */

#define MPSC_INIT(symbol) { .head = NULL, .tail = NULL }

static inline void mpsc_push(struct mpsc *q, struct mpsc_node *n)
{
  k_spinlock_key_t key = k_spin_lock(&q->lock);

  n->next = NULL;
  if (q->tail != NULL)
    {
      q->tail->next = n;
    }
  else
    {
      q->head = n;
    }

  q->tail = n;
  k_spin_unlock(&q->lock, key);
}

static inline struct mpsc_node *mpsc_pop(struct mpsc *q)
{
  k_spinlock_key_t key = k_spin_lock(&q->lock);
  struct mpsc_node *n = q->head;

  if (n != NULL)
    {
      q->head = n->next;
      if (q->head == NULL)
        {
          q->tail = NULL;
        }

      n->next = NULL;
    }

  k_spin_unlock(&q->lock, key);
  return n;
}

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_SYS_MPSC_LOCKFREE_H */
