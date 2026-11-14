/* SPDX-License-Identifier: Apache-2.0 */
/* NuttX port of the sensor shell: what the copies of Zephyr's sensor shell
 * (sensor_shell.c, sensor_shell_stream.c, sensor_shell.h) take from the
 * Zephyr build's configuration and logging, and the two additions of the
 * port (sensor_shell_nuttx.c).  Included first by both copies.
 */

#ifndef __NUTTX_APPS_SENSOR_SENSOR_SHELL_NUTTX_H
#define __NUTTX_APPS_SENSOR_SENSOR_SHELL_NUTTX_H

#include <nuttx/config.h>

#include <stdint.h>

#include "priorities.h"

/* The Zephyr build's values (zephyr/build/usb-console/zephyr/.config):
 * CONFIG_SENSOR_SHELL_STREAM=y, CONFIG_SENSOR_SHELL_THREAD_STACK_SIZE=3072,
 * CONFIG_SENSOR_SHELL_TRIG_PRINT_TIMEOUT_MS=5000,
 * CONFIG_SENSOR_SHELL_MAX_TRIGGER_DEVICES=1, CONFIG_SENSOR_LOG_LEVEL=3;
 * CONFIG_SENSOR_INFO and CONFIG_SENSOR_SHELL_BATTERY are not set.
 */

#define CONFIG_SENSOR_SHELL_STREAM                1
#define CONFIG_SENSOR_SHELL_THREAD_STACK_SIZE     NANO_STACK_SENSOR_SHELL
#define CONFIG_SENSOR_SHELL_TRIG_PRINT_TIMEOUT_MS 5000
#define CONFIG_SENSOR_SHELL_MAX_TRIGGER_DEVICES   1
#define CONFIG_SENSOR_LOG_LEVEL                   3

/* Zephyr's <zephyr/logging/log.h>, which the copies get through Zephyr's
 * headers: the port's RAM log ("dmesg").
 */

#include "app_log.h"

/* "sensor get" waits for its result to be printed (sensor_shell.c).  Zephyr
 * prints it from the processing thread after the command has returned and
 * draws the prompt again below it; nsh cannot draw its prompt again, so the
 * command returns to the prompt once the processing thread has printed the
 * result.  get_arm() before the read is submitted, get_wait() after the
 * command's mutex is released (ERR: the command's result so far; no wait
 * unless 0).  CTX is the command's processing context, which the read's
 * completion carries as its userdata.
 */

void sensor_shell_nuttx_get_arm(void *ctx);
void sensor_shell_nuttx_get_wait(void *ctx, int err);

/* The processing thread's callback (sensor_shell_stream.c):
 * sensor_shell_processing_callback(), then the end of a "sensor get" that
 * waits for it.
 */

void sensor_shell_nuttx_processing_callback(int result, uint8_t *buf,
                                            uint32_t buf_len,
                                            void *userdata);

#endif /* __NUTTX_APPS_SENSOR_SENSOR_SHELL_NUTTX_H */
