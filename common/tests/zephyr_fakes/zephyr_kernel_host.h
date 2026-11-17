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
/* The fake clock of k_uptime_get() and k_sleep(): set it (microseconds
 * since boot), and the ticks the last k_sleep() waited.
 */
void test_time_set_us(uint64_t us);
int64_t test_slept_ticks(void);
/* The i-th thread started with z_thread_static_start() (K_THREAD_DEFINE()
 * via K_THREAD_DEFINE_START()), NULL if none.
 */
const struct z_thread_static *test_thread(int i);
#endif
