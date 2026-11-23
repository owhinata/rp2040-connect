/* SPDX-License-Identifier: MIT */
/* The NuttX part of the sensor shell (common/shell/
 * sensor_shell.c and common/compat/sensor_shell_stream.c, which include
 * this header first): the Zephyr build's CONFIG_SENSOR_SHELL_* values and
 * the log (sensor_shell_nuttx.h), the wait of "sensor get" for its result
 * to be printed before it returns to the prompt
 * (sensor_shell_nuttx_get_arm()/_wait(), see sensor_shell_nuttx.h), and the
 * processing thread's callback (sensor_shell_stream.c).  The Zephyr
 * build's header (drivers/sensor/sensor_shell_os.h of its patched tree)
 * does nothing here; the Zephyr build uses Zephyr's own
 * sensor_shell_stream.c.
 */

#ifndef __NUTTX_APPS_SENSOR_SENSOR_SHELL_OS_H
#define __NUTTX_APPS_SENSOR_SENSOR_SHELL_OS_H

#include "sensor_shell_nuttx.h"

#define sensor_shell_os_get_arm(ctx)       sensor_shell_nuttx_get_arm(ctx)
#define sensor_shell_os_get_wait(ctx, err) sensor_shell_nuttx_get_wait(ctx, err)
#define sensor_shell_os_processing_callback \
  sensor_shell_nuttx_processing_callback

#endif /* __NUTTX_APPS_SENSOR_SENSOR_SHELL_OS_H */
