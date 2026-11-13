/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's device model (<zephyr/device.h>) that the ported
 * driver and its callers use: struct device with its name, config, API and
 * data, device_is_ready(), the run-once initialization, and getting a
 * devicetree node's device with DEVICE_DT_GET(DT_NODELABEL(label)).  Part of
 * the small Zephyr compatibility layer of the port (nuttx/drivers/compat,
 * see nuttx/drivers/README.md).
 *
 * Zephyr initializes its devices by itself before main(); here the board
 * calls device_init() at the place of the same order in its bring-up
 * (boards/arduino-nano-rp2040-connect/src/nano_lsm6dso.c).  The devices
 * that exist are listed in <zephyr/devicetree.h>.
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_DEVICE_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_DEVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

/* zephyr/device.h struct device_state: the result of the initialization.
 * init_res is the positive errno of a failed init, initialized is set by a
 * successful one only (kernel/device.c, do_device_init()).
 */

struct device_state
{
  uint8_t init_res;
  bool initialized;
};

struct device
{
  const char *name;       /* the devicetree node name, e.g. "lsm6dsox@6a" */
  const void *config;
  const void *api;
  struct device_state *state;
  void *data;
  int (*init)(const struct device *dev);  /* Zephyr: ops.init */
};

/* kernel/device.c z_impl_device_is_ready(): false for NULL, else whether
 * the init function has succeeded.
 */

static inline bool device_is_ready(const struct device *dev)
{
  return dev != NULL && dev->state->initialized;
}

/* Runs the init function once and records the result, as Zephyr's
 * do_device_init() (kernel/device.c:18-63) does at boot.  Returns 0 or the
 * negative errno of the init function.  Thread context.
 */

int device_init(const struct device *dev);

/* DEVICE_API(sensor, name): the driver's API table. */

#define DEVICE_API(_class, _name)  const struct _class##_driver_api _name
#define DEVICE_API_GET(_class, _dev) \
  ((const struct _class##_driver_api *)(_dev)->api)

/* Devicetree nodes as devices.  A node is named by its label
 * (DT_NODELABEL(lsm6dsox) is the token lsm6dsox); the device of a node is
 * the object nano_device_<label>.  The expansion goes through a second
 * macro so that DEVICE_DT_GET(DT_BUS(...)) expands its argument first.
 */

#define DEVICE_DT_GET(node_id)      Z_DEVICE_DT_GET(node_id)
#define Z_DEVICE_DT_GET(node_id)    (&nano_device_##node_id)
#define DEVICE_DT_DECLARE(node_id)  Z_DEVICE_DT_DECLARE(node_id)
#define Z_DEVICE_DT_DECLARE(node_id) \
  extern const struct device nano_device_##node_id

/* DEVICE_DT_DEFINE(node_id, name, init, data, config, api): one device per
 * node, as Zephyr's DEVICE_DT_DEFINE() (without the init level and
 * priority, which the board's order of calls gives, and without PM).
 */

#define DEVICE_DT_DEFINE(node_id, _name, _init, _data, _config, _api) \
  Z_DEVICE_DT_DEFINE(node_id, _name, _init, _data, _config, _api)
#define Z_DEVICE_DT_DEFINE(node_id, _name, _init, _data, _config, _api) \
  static struct device_state nano_device_state_##node_id; \
  const struct device nano_device_##node_id = \
    { \
      .name = (_name), \
      .config = (_config), \
      .api = (_api), \
      .state = &nano_device_state_##node_id, \
      .data = (_data), \
      .init = (_init), \
    }

#include <zephyr/devicetree.h>

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_DEVICE_H */
