/* SPDX-License-Identifier: MIT */
/* The NuttX part of the sensor shell (common/drivers/sensor_shell/
 * sensor_shell.c, which includes this header first): the Zephyr build's
 * CONFIG_SENSOR_SHELL_* values and the log (sensor_shell_nuttx.h), and the
 * wait of "sensor get" for its result to be printed before it returns to the
 * prompt (sensor_shell_nuttx_get_arm()/_wait(), see sensor_shell_nuttx.h).
 * The Zephyr build's header (zephyr/app/usb-console/drivers/sensor_shell/
 * sensor_shell_os.h) does nothing here.
 */

#ifndef __NUTTX_APPS_SENSOR_SENSOR_SHELL_OS_H
#define __NUTTX_APPS_SENSOR_SENSOR_SHELL_OS_H

#include "sensor_shell_nuttx.h"

#define sensor_shell_os_get_arm(ctx)       sensor_shell_nuttx_get_arm(ctx)
#define sensor_shell_os_get_wait(ctx, err) sensor_shell_nuttx_get_wait(ctx, err)

#endif /* __NUTTX_APPS_SENSOR_SENSOR_SHELL_OS_H */
