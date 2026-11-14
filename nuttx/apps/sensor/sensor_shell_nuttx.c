/* SPDX-License-Identifier: Apache-2.0 */
/* NuttX port of the sensor shell: the wait of "sensor get" for its printed
 * result (see sensor_shell_nuttx.h).
 *
 * The read's completion reaches the processing thread
 * (sensor_shell_processing_tid, sensor_shell_stream.c), which prints the
 * result on the command's terminal with sensor_shell_processing_callback()
 * and then posts the semaphore that the command waits on.  The command
 * waits at most GET_WAIT_MS: the processing thread can be held up by a
 * terminal that does not take its output; the result is then printed
 * later, after the prompt, as in Zephyr.
 */

#include "sensor_shell_nuttx.h"

#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stddef.h>
#include <time.h>

#include "sensor_shell.h"

/* The host test (nuttx/tests/sensor_shell.c) waits less. */
#ifdef SENSOR_SHELL_GET_WAIT_MS
#define GET_WAIT_MS SENSOR_SHELL_GET_WAIT_MS
#else
#define GET_WAIT_MS 2000
#endif

static pthread_once_t g_get_once = PTHREAD_ONCE_INIT;
static sem_t g_get_done;
static void *volatile g_get_ctx;

static void get_init(void)
{
	sem_init(&g_get_done, 0, 0);
#ifdef SEM_PRIO_NONE
	/* A signal, not a lock: no priority inheritance. */
	sem_setprotocol(&g_get_done, SEM_PRIO_NONE);
#endif
}

void sensor_shell_nuttx_get_arm(void *ctx)
{
	pthread_once(&g_get_once, get_init);
	/* A post left over from a get that stopped waiting. */
	while (sem_trywait(&g_get_done) == 0) {
	}
	g_get_ctx = ctx;
}

void sensor_shell_nuttx_get_wait(void *ctx, int err)
{
	struct timespec abstime;

	if (err == 0) {
		clock_gettime(CLOCK_MONOTONIC, &abstime);
		abstime.tv_sec += GET_WAIT_MS / 1000;
		abstime.tv_nsec += (GET_WAIT_MS % 1000) * 1000000L;
		if (abstime.tv_nsec >= 1000000000L) {
			abstime.tv_sec++;
			abstime.tv_nsec -= 1000000000L;
		}
		while (sem_clockwait(&g_get_done, CLOCK_MONOTONIC, &abstime) != 0 &&
		       errno == EINTR) {
		}
	}
	if (g_get_ctx == ctx) {
		g_get_ctx = NULL;
	}
}

void sensor_shell_nuttx_processing_callback(int result, uint8_t *buf, uint32_t buf_len,
					    void *userdata)
{
	sensor_shell_processing_callback(result, buf, buf_len, userdata);
	if (userdata != NULL && userdata == g_get_ctx) {
		g_get_ctx = NULL;
		sem_post(&g_get_done);
	}
}
