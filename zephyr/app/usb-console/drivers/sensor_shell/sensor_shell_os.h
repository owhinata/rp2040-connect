/* SPDX-License-Identifier: MIT */
/* The Zephyr part of the sensor shell (common/drivers/sensor_shell/
 * sensor_shell.c), which includes this header first.  "sensor get" returns
 * at once: the processing thread prints its result after the command has
 * returned, and the shell draws the prompt again below it.  The NuttX build
 * waits for the result here instead (nuttx/apps/sensor/sensor_shell_os.h).
 */

#ifndef SENSOR_SHELL_OS_H_
#define SENSOR_SHELL_OS_H_

#define sensor_shell_os_get_arm(ctx)       ((void)0)
#define sensor_shell_os_get_wait(ctx, err) ((void)0)

#endif /* SENSOR_SHELL_OS_H_ */
