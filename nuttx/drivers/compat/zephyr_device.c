/* SPDX-License-Identifier: Apache-2.0 */
/* The run-once device initialization of <zephyr/device.h> (Zephyr's
 * do_device_init()).  Part of the small Zephyr compatibility layer of the
 * port (nuttx/drivers/compat).
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>

/* kernel/device.c do_device_init(): run the init function, keep the
 * positive errno of a failure (at most 255), mark the device initialized
 * only on success.
 */

int device_init(const struct device *dev)
{
  int rc = 0;

  if (dev->init != NULL)
    {
      rc = dev->init(dev);
      if (rc != 0)
        {
          int res = rc < 0 ? -rc : rc;

          dev->state->init_res = res > UINT8_MAX ? UINT8_MAX : (uint8_t)res;
        }
      else
        {
          dev->state->init_res = 0;
        }
    }

  if (rc == 0)
    {
      dev->state->initialized = true;
    }

  return rc;
}
