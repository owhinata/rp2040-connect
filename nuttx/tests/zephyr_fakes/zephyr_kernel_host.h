/* SPDX-License-Identifier: Apache-2.0 */
/* What a host test can do with the fake kernel of zephyr_kernel_host.c. */
#ifndef TEST_ZEPHYR_KERNEL_HOST_H
#define TEST_ZEPHYR_KERNEL_HOST_H
#include <stdbool.h>
#include <zephyr/kernel.h>

/* Runs the pending items of every started queue (the system work queue
 * first) until none is left; returns how many ran.
 */
int test_work_run_all(void);
/* The same for one queue. */
int test_work_run(struct k_work_q *queue);
/* Fires the timer (as its interrupt would). */
void test_timeout_fire(struct _timeout *to);
/* The number of started queues, and one of them with its name and
 * priority.
 */
int test_work_queues(void);
struct k_work_q *test_work_queue(int i, const char **name, int *prio, size_t *stack);
/* Fires every armed timer of a delayable item on a started queue. */
#endif
