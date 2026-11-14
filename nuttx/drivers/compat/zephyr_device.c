/* SPDX-License-Identifier: Apache-2.0 */
/* The run-once device initialization of <zephyr/device.h> (Zephyr's
 * do_device_init()), and the list of the port's devices with their node
 * labels and classes (device_get_binding(), device_get_by_dt_nodelabel(),
 * DEVICE_API_IS()).  Part of the small Zephyr compatibility layer of the
 * port (nuttx/drivers/compat).
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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

/* The devices, in the order of their initialization in the Zephyr build
 * (gpio: PRE_KERNEL_1; i2c0: POST_KERNEL 50; the LSM6DSOX: POST_KERNEL,
 * CONFIG_SENSOR_INIT_PRIORITY 90), with the node label of the Zephyr
 * build's devicetree and whether the API is a sensor API.  gpio0 exists
 * with the FIFO stream only (zephyr_gpio.c).
 */

static const struct
{
  const struct device *dev;
  const char *nodelabel;
  bool sensor;
} g_devices[] =
{
#ifdef CONFIG_NANO_LSM6DSO_STREAM
  { DEVICE_DT_GET(DT_NODELABEL(gpio0)), "gpio0", false },
#endif
  { DEVICE_DT_GET(DT_NODELABEL(i2c0)), "i2c0", false },
  { DEVICE_DT_GET(DT_NODELABEL(lsm6dsox)), "lsm6dsox", true },
};

size_t z_device_count(void)
{
  return ARRAY_SIZE(g_devices);
}

const struct device *z_device_get(size_t idx)
{
  return idx < ARRAY_SIZE(g_devices) ? g_devices[idx].dev : NULL;
}

/* kernel/device.c z_impl_device_get_binding(): the first device with that
 * name (the same pointer or the same text), NULL if it is not ready.
 */

const struct device *device_get_binding(const char *name)
{
  size_t i;

  if (name == NULL || name[0] == '\0')
    {
      return NULL;
    }

  for (i = 0; i < ARRAY_SIZE(g_devices); i++)
    {
      const struct device *dev = g_devices[i].dev;

      if (dev->name == name || strcmp(name, dev->name) == 0)
        {
          return device_is_ready(dev) ? dev : NULL;
        }
    }

  return NULL;
}

/* kernel/device.c z_impl_device_get_by_dt_nodelabel(): the first ready
 * device that has the node label; NULL for NULL or "".
 */

const struct device *device_get_by_dt_nodelabel(const char *name)
{
  size_t i;

  if (name == NULL || name[0] == '\0')
    {
      return NULL;
    }

  for (i = 0; i < ARRAY_SIZE(g_devices); i++)
    {
      if (device_is_ready(g_devices[i].dev) &&
          strcmp(name, g_devices[i].nodelabel) == 0)
        {
          return g_devices[i].dev;
        }
    }

  return NULL;
}

bool z_device_api_is_sensor(const struct device *dev)
{
  size_t i;

  for (i = 0; i < ARRAY_SIZE(g_devices); i++)
    {
      if (g_devices[i].dev == dev)
        {
          return g_devices[i].sensor;
        }
    }

  return false;
}
