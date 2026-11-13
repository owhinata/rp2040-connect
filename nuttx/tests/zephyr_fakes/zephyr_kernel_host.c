/* SPDX-License-Identifier: Apache-2.0 */
/* A fake of the Zephyr compatibility layer's OS part (zephyr_kernel.c)
 * for host tests: one thread, nothing blocks.  Waiting for something that
 * is not there fails the test (a real thread would sleep).  Work queues
 * are run by the test (test_work_run()), timers fired by it
 * (test_timeout_fire()).
 */
#include <assert.h>
#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/rtio/work.h>

#include "zephyr_kernel_host.h"

int k_mutex_init(struct k_mutex *mutex)
{
	mutex->depth = 0;
	return 0;
}

int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout)
{
	(void)timeout;
	mutex->depth++;
	return 0;
}

int k_mutex_unlock(struct k_mutex *mutex)
{
	assert(mutex->depth > 0);
	mutex->depth--;
	return 0;
}

int k_sem_init(struct k_sem *sem, unsigned int initial_count, unsigned int limit)
{
	if (limit == 0 || initial_count > limit) {
		return -EINVAL;
	}
	*sem = (struct k_sem){.count = initial_count, .limit = limit};
	return 0;
}

int k_sem_take(struct k_sem *sem, k_timeout_t timeout)
{
	if (sem->count > 0) {
		sem->count--;
		return 0;
	}
	/* A K_FOREVER wait would never end in one thread. */
	assert(!K_TIMEOUT_EQ(timeout, K_FOREVER));
	return K_TIMEOUT_EQ(timeout, K_NO_WAIT) ? -EBUSY : -EAGAIN;
}

void k_sem_give(struct k_sem *sem)
{
	if (sem->count < sem->limit) {
		sem->count++;
	}
}

void k_sem_reset(struct k_sem *sem)
{
	sem->count = 0;
}

unsigned int k_sem_count_get(struct k_sem *sem)
{
	return sem->count;
}

void k_poll_event_init(struct k_poll_event *event, uint32_t type, int mode, void *obj)
{
	*event = (struct k_poll_event){.type = type, .mode = (uint32_t)mode, .sem = obj};
}

int k_poll(struct k_poll_event *events, int num_events, k_timeout_t timeout)
{
	int ready = 0;

	(void)timeout;
	for (int i = 0; i < num_events; i++) {
		events[i].state = K_POLL_STATE_NOT_READY;
		if (events[i].type == K_POLL_TYPE_SEM_AVAILABLE && events[i].sem->count > 0) {
			events[i].state = K_POLL_STATE_SEM_AVAILABLE;
			ready++;
		}
	}
	return ready ? 0 : -EAGAIN;
}

static struct {
	struct k_work_q *queue;
	const char *name;
	int prio;
	size_t stack;
} queues[8];
static int num_queues;

int k_thread_name_set(k_tid_t thread, const char *name)
{
	for (int i = 0; i < num_queues; i++) {
		if (queues[i].queue->thread == thread) {
			queues[i].name = name;
			return 0;
		}
	}
	return -EINVAL;
}

void k_work_queue_start(struct k_work_q *queue, k_thread_stack_t *stack, size_t stack_size,
			int prio, const struct k_work_queue_config *cfg)
{
	(void)stack;
	assert(num_queues < 8);
	z_work_queue_init(queue);
	queue->flags = BIT(K_WORK_QUEUE_STARTED_BIT);
	queue->thread = 100 + num_queues;
	queues[num_queues].queue = queue;
	queues[num_queues].name = cfg && cfg->name ? cfg->name : "";
	queues[num_queues].prio = prio;
	queues[num_queues].stack = stack_size;
	num_queues++;
}

int test_work_queues(void)
{
	return num_queues;
}

struct k_work_q *test_work_queue(int i, const char **name, int *prio, size_t *stack)
{
	*name = queues[i].name;
	*prio = queues[i].prio;
	*stack = queues[i].stack;
	return queues[i].queue;
}

int test_work_run(struct k_work_q *queue)
{
	bool yield;
	int n = 0;

	while (z_work_queue_process(queue, &yield)) {
		n++;
	}
	/* Every submission gave the thread's semaphore. */
	queue->notify.count = 0;
	return n;
}

int test_work_run_all(void)
{
	int total = 0, n;

	do {
		n = 0;
		for (int i = 0; i < num_queues; i++) {
			n += test_work_run(queues[i].queue);
		}
		while (rtio_workq_process()) {
			n++;
		}
		total += n;
	} while (n > 0);
	return total;
}

void z_add_timeout(struct _timeout *to, _timeout_func_t fn, k_timeout_t timeout)
{
	to->fn = fn;
	to->ticks = timeout.ticks;
	to->armed = true;
}

void z_abort_timeout(struct _timeout *to)
{
	to->armed = false;
}

void test_timeout_fire(struct _timeout *to)
{
	assert(to->armed);
	to->armed = false;
	to->fn(to);
}

struct k_work_q k_sys_work_q;

int zephyr_compat_initialize(void)
{
	static const struct k_work_queue_config cfg = {.name = "sysworkq"};

	k_work_queue_start(&k_sys_work_q, NULL, 1024, 119, &cfg);
	return 0;
}
