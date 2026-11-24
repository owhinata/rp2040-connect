/* SPDX-License-Identifier: MIT */
/* The OS part of the sensor shell on the compatibility layer
 * (common/shell/port/generic): the wait of "sensor get" for its
 * printed result (sensor_shell_os.h), on the Zephyr compatibility layer's
 * semaphore.  The same behaviour as the NuttX build's
 * nuttx/apps/sensor/sensor_shell_nuttx.c.
 *
 * The read's completion reaches the processing thread
 * (sensor_shell_tid, common/compat/sensor_shell_stream.c), which
 * prints the result on the command's console with
 * sensor_shell_processing_callback() and then gives the semaphore that the
 * command waits on.  The command waits at most SENSOR_SHELL_GET_WAIT_MS: the
 * processing thread can be held up by a terminal that does not take its
 * output (a console's writer waits for its mutex at most 1 s,
 * common/compat/include/shell_console.h); the result is then printed later,
 * after the prompt, as in Zephyr.
 */

#include "sensor_shell_os.h"

#include <stddef.h>

#include <zephyr/kernel.h>

#include "sensor_shell.h"

/* The host test (freertos/tests/sensor_shell) may wait less. */
#ifndef SENSOR_SHELL_GET_WAIT_MS
#define SENSOR_SHELL_GET_WAIT_MS 2000
#endif

/* A signal, not a lock: a count of at most 1. */
static K_SEM_DEFINE(get_done, 0, 1);
/* The context of the "sensor get" that waits, or NULL. */
static void *volatile get_ctx;

void sensor_shell_os_get_arm(void *ctx)
{
	/* A give left over from a get that stopped waiting. */
	k_sem_reset(&get_done);
	get_ctx = ctx;
}

void sensor_shell_os_get_wait(void *ctx, int err)
{
	if (err == 0) {
		(void)k_sem_take(&get_done, K_MSEC(SENSOR_SHELL_GET_WAIT_MS));
	}
	if (get_ctx == ctx) {
		get_ctx = NULL;
	}
}

void sensor_shell_os_processing_callback(int result, uint8_t *buf, uint32_t buf_len, void *userdata)
{
	sensor_shell_processing_callback(result, buf, buf_len, userdata);
	if (userdata != NULL && userdata == get_ctx) {
		get_ctx = NULL;
		k_sem_give(&get_done);
	}
}
