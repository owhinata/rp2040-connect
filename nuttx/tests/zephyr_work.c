/* SPDX-License-Identifier: Apache-2.0 */
/* Host test of the compatibility layer's work items (nuttx/drivers/compat/
 * zephyr_work.c) on the fake kernel (tests/zephyr_fakes): the return values
 * and flags of Zephyr's kernel/work.c that lsm6dso_stream.c and
 * lsm6dso_fifo_irq.c depend on.
 *
 * - k_work_submit_to_queue(): 1 queued, 0 already queued, 2 queued again
 *   while running (it runs once more), -ENODEV on a queue not started.
 * - k_work_schedule_for_queue(): 0 and no change while delayed or queued; a
 *   running item can be scheduled; K_NO_WAIT queues at once; K_MSEC(20)
 *   is 200 ticks of 100 us.
 * - k_work_reschedule_for_queue(): stops the delay; K_NO_WAIT queues now; a
 *   queued item stays queued once.
 * - the timer queues the item on the queue it was scheduled on; one that was
 *   unscheduled does nothing.
 * - k_work_cancel_sync(): removes a queued item (it does not run), false
 *   for an idle one.
 */
#include <assert.h>
#include <errno.h>

#include <zephyr/kernel.h>

#include "zephyr_kernel_host.h"

static struct k_work_q queue;
static struct k_work_q idle_queue;    /* never started */
static int runs;
static int (*in_handler)(void);

static void handler(struct k_work *work)
{
	(void)work;
	runs++;
	if (in_handler != NULL) {
		int (*f)(void) = in_handler;

		in_handler = NULL;
		(void)f();
	}
}

static struct k_work item;
static struct k_work_delayable ditem;
static int last_ret;

static int resubmit(void)
{
	assert(k_work_busy_get(&item) == K_WORK_RUNNING);
	last_ret = k_work_submit_to_queue(&queue, &item);
	return last_ret;
}

static int schedule_while_running(void)
{
	last_ret = k_work_schedule_for_queue(&queue, &ditem, K_MSEC(20));
	return last_ret;
}

static int wake_ret;

static int wake_then_poll(void)
{
	wake_ret = k_work_reschedule_for_queue(&queue, &ditem, K_NO_WAIT);
	return schedule_while_running();
}

static void test_ticks(void)
{
	assert(K_MSEC(20).ticks == 200);
	assert(K_MSEC(1).ticks == 10);
	assert(K_MSEC(0).ticks == 0 && K_TIMEOUT_EQ(K_MSEC(0), K_NO_WAIT));
	assert(K_FOREVER.ticks == -1);
}

static void test_submit(void)
{
	k_work_init(&item, handler);
	assert(k_work_submit_to_queue(&idle_queue, &item) == -ENODEV);
	assert(k_work_busy_get(&item) == 0);
	assert(k_work_submit_to_queue(&queue, &item) == 1);
	assert(k_work_submit_to_queue(&queue, &item) == 0);
	assert(k_work_busy_get(&item) == K_WORK_QUEUED);
	assert(test_work_run(&queue) == 1 && runs == 1);
	/* Queued again by its own handler: runs again after it. */
	in_handler = resubmit;
	assert(k_work_submit_to_queue(&queue, &item) == 1);
	assert(test_work_run(&queue) == 2 && runs == 3 && last_ret == 2);
	assert(k_work_busy_get(&item) == 0);
	/* k_work_submit(): the system work queue. */
	assert(k_work_submit(&item) == 1);
	assert(item.queue == &k_sys_work_q && test_work_run(&k_sys_work_q) == 1 && runs == 4);
}

static void test_cancel_sync(void)
{
	struct k_work_sync sync;

	assert(!k_work_cancel_sync(&item, &sync));
	assert(k_work_submit_to_queue(&queue, &item) == 1);
	assert(k_work_cancel_sync(&item, &sync));
	assert(k_work_busy_get(&item) == 0 && test_work_run(&queue) == 0 && runs == 4);
	/* It can be submitted again afterwards. */
	assert(k_work_submit_to_queue(&queue, &item) == 1 && test_work_run(&queue) == 1);
	runs = 0;
}

static void test_delayable(void)
{
	k_work_init_delayable(&ditem, handler);
	assert(k_work_delayable_from_work(&ditem.work) == &ditem);
	/* schedule: a delay, then nothing while delayed. */
	assert(k_work_schedule_for_queue(&queue, &ditem, K_MSEC(20)) == 1);
	assert(ditem.timeout.armed && ditem.timeout.ticks == 200);
	assert(k_work_busy_get(&ditem.work) == K_WORK_DELAYED);
	assert(k_work_schedule_for_queue(&queue, &ditem, K_NO_WAIT) == 0);
	assert(k_work_schedule_for_queue(&queue, &ditem, K_MSEC(5)) == 0);
	assert(ditem.timeout.ticks == 200 && test_work_run(&queue) == 0);
	/* The timer queues it. */
	test_timeout_fire(&ditem.timeout);
	assert(k_work_busy_get(&ditem.work) == K_WORK_QUEUED);
	assert(test_work_run(&queue) == 1 && runs == 1);
	/* reschedule(K_NO_WAIT) cuts a pending delay short. */
	assert(k_work_schedule_for_queue(&queue, &ditem, K_MSEC(20)) == 1);
	assert(k_work_reschedule_for_queue(&queue, &ditem, K_NO_WAIT) == 1);
	assert(!ditem.timeout.armed && k_work_busy_get(&ditem.work) == K_WORK_QUEUED);
	/* Queued: schedule does nothing, reschedule(K_NO_WAIT) leaves it queued
	 * once.
	 */
	assert(k_work_schedule_for_queue(&queue, &ditem, K_MSEC(20)) == 0);
	assert(!ditem.timeout.armed);
	assert(k_work_reschedule_for_queue(&queue, &ditem, K_NO_WAIT) == 0);
	assert(test_work_run(&queue) == 1 && runs == 2);
	/* Running: schedule arms the delay (the stream's 20 ms poll). */
	in_handler = schedule_while_running;
	assert(k_work_reschedule_for_queue(&queue, &ditem, K_NO_WAIT) == 1);
	assert(test_work_run(&queue) == 1 && runs == 3 && last_ret == 1);
	assert(ditem.timeout.armed && ditem.timeout.ticks == 200);
	/* reschedule with a delay replaces the delay. */
	assert(k_work_reschedule_for_queue(&queue, &ditem, K_MSEC(5)) == 1);
	assert(ditem.timeout.armed && ditem.timeout.ticks == 50);
	assert(k_work_reschedule_for_queue(&queue, &ditem, K_NO_WAIT) == 1);
	assert(!ditem.timeout.armed && test_work_run(&queue) == 1 && runs == 4);
	/* Running and queued again by a wakeup (the INT1 work's reschedule),
	 * then the poll's schedule: the schedule does nothing, the item runs
	 * again at once.
	 */
	in_handler = wake_then_poll;
	assert(k_work_reschedule_for_queue(&queue, &ditem, K_NO_WAIT) == 1);
	assert(test_work_run(&queue) == 2 && runs == 6);
	assert(wake_ret == 2 && last_ret == 0 && !ditem.timeout.armed);
	assert(k_work_busy_get(&ditem.work) == 0);
}

int main(void)
{
	assert(zephyr_compat_initialize() == 0);
	k_work_queue_start(&queue, NULL, 2048, K_PRIO_PREEMPT(10), NULL);
	assert(k_thread_name_set(k_work_queue_thread_get(&queue), "imu_fifo_worker") == 0);
	{
		const char *name;
		int prio;
		size_t stack;

		assert(test_work_queues() == 2);
		assert(test_work_queue(0, &name, &prio, &stack) == &k_sys_work_q);
		assert(prio == 119 && stack == 1024);
		assert(test_work_queue(1, &name, &prio, &stack) == &queue);
		assert(prio == 108 && stack == 2048 && name[0] == 'i');
	}
	test_ticks();
	test_submit();
	test_cancel_sync();
	test_delayable();
	return 0;
}
