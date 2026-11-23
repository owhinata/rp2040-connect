/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the sensor shell (common/shell/
 * sensor_shell.c and the copy of Zephyr's sensor_shell_stream.c in
 * common/compat, which include this header first): the Zephyr build's
 * CONFIG_SENSOR_SHELL_* values, the log, the wait of "sensor get" for its
 * result (sensor_shell_os_get_arm()/_wait()) and the processing thread's
 * callback.  The Zephyr build's header (drivers/sensor/sensor_shell_os.h of
 * its patched tree) does nothing for the wait; the NuttX
 * build's (nuttx/apps/sensor/sensor_shell_os.h) waits as this one does.
 *
 * Where the results go: the read's completion carries the command's
 * processing context (struct sensor_shell_processing_context), which holds
 * the command's shell backend (ctx.sh): on FreeRTOS the console of the
 * session that ran the command, the USB port or the Telnet client
 * (common/compat/shell_engine/zephyr_os_shell.h).  The processing thread
 * (sensor_shell_tid) prints with shell_print(ctx.sh, ...), which
 * writes straight to that console (common/compat/shell_engine/zshell_os_impl.h),
 * as Zephyr's processing thread prints on the shell instance that started
 * the read or the stream.  A console lives as long as the system: while
 * its session has no terminal (the USB port closed, the Telnet client
 * gone) the text is dropped and the thread goes on; the next Telnet client
 * gets the rest of a stream, as with Zephyr's Telnet backend.
 *
 * The wait (sensor_shell_os.c): Zephyr's "sensor get" returns at once and
 * the processing thread's lines come after the prompt, which Zephyr's shell
 * then draws again below them.  The FreeRTOS build's line editor
 * (common/compat/shell_engine/cli.h) does not draw the prompt again for the
 * output of another task, so the command waits until the processing thread
 * has printed the result (at most SENSOR_SHELL_GET_WAIT_MS) and returns to
 * the prompt then: the terminal shows what Zephyr's shows, the result and a
 * prompt below it.  The NuttX build does the same for nsh.
 */

#ifndef FREERTOS_DRIVERS_SENSOR_SHELL_SENSOR_SHELL_OS_H
#define FREERTOS_DRIVERS_SENSOR_SHELL_SENSOR_SHELL_OS_H

#include <stdint.h>

#include <zephyr/logging/log.h>

#include "priorities.h"

/* The Zephyr build's values (zephyr/build/usb-console/zephyr/.config):
 * CONFIG_SENSOR_SHELL_STREAM=y, CONFIG_SENSOR_SHELL_THREAD_STACK_SIZE=3072
 * (here APP_STACK_SENSOR_SHELL, 2048 as measured on this build:
 * app/usb-console/src/priorities_os.h),
 * CONFIG_SENSOR_SHELL_TRIG_PRINT_TIMEOUT_MS=5000,
 * CONFIG_SENSOR_SHELL_MAX_TRIGGER_DEVICES=1; CONFIG_SENSOR_INFO (so no
 * "sensor info") and CONFIG_SENSOR_SHELL_BATTERY are not set.  The thread's
 * priority is Zephyr's 0 (sensor_shell_stream.c: K_THREAD_DEFINE(..., 0, 0,
 * 0), APP_PRIO_SENSOR_SHELL = 20).  CONFIG_SENSOR_LOG_LEVEL (3) is in the
 * prelude of the shared sources (freertos/drivers/compat/common_prelude.h).
 */
#define CONFIG_SENSOR_SHELL_STREAM                1
#define CONFIG_SENSOR_SHELL_THREAD_STACK_SIZE     APP_STACK_SENSOR_SHELL
#define CONFIG_SENSOR_SHELL_TRIG_PRINT_TIMEOUT_MS 5000
#define CONFIG_SENSOR_SHELL_MAX_TRIGGER_DEVICES   1

/* "sensor get": get_arm() before the read is submitted, get_wait() after
 * the command's mutex is released (ERR: the command's result so far; no
 * wait unless 0).  CTX is the command's processing context, which the
 * read's completion carries as its userdata.
 */
void sensor_shell_freertos_get_arm(void *ctx);
void sensor_shell_freertos_get_wait(void *ctx, int err);

/* The processing thread's callback (sensor_shell_stream.c):
 * sensor_shell_processing_callback(), then the end of a "sensor get" that
 * waits for it.
 */
void sensor_shell_freertos_processing_callback(int result, uint8_t *buf, uint32_t buf_len,
					       void *userdata);

#define sensor_shell_os_get_arm(ctx)        sensor_shell_freertos_get_arm(ctx)
#define sensor_shell_os_get_wait(ctx, err)  sensor_shell_freertos_get_wait(ctx, err)
#define sensor_shell_os_processing_callback sensor_shell_freertos_processing_callback

#endif /* FREERTOS_DRIVERS_SENSOR_SHELL_SENSOR_SHELL_OS_H */
