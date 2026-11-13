/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's GPIO API (<zephyr/drivers/gpio.h>, gpio_utils.h)
 * that the ported LSM6DSO FIFO interrupt (lsm6dso_fifo_irq.c) uses, with
 * Zephyr's names, flags and meanings: struct gpio_dt_spec,
 * gpio_is_ready_dt(), gpio_pin_configure_dt() (inputs), callbacks
 * (gpio_init_callback(), gpio_add_callback(), gpio_remove_callback(),
 * fired in the interrupt handler), gpio_pin_interrupt_configure_dt()
 * (edges and levels, logical or physical) and gpio_pin_get_dt() (the
 * logical level).  One port, gpio0 (GPIO0..29 of the RP2040), on the
 * board's <nano_gpio.h>; zephyr_gpio.c is Zephyr's gpio.h logic and
 * drivers/gpio/gpio_rpi_pico.c.  Outputs are not supported (-ENOTSUP).
 * Part of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_GPIO_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

/* zephyr/dt-bindings/gpio/gpio.h */

#define GPIO_ACTIVE_HIGH        (0 << 0)
#define GPIO_ACTIVE_LOW         (1 << 0)
#define GPIO_PULL_UP            (1 << 4)
#define GPIO_PULL_DOWN          (1 << 5)

/* zephyr/drivers/gpio.h */

#define GPIO_INPUT              BIT(16)
#define GPIO_OUTPUT             BIT(17)
#define GPIO_DISCONNECTED       0

#define GPIO_INT_DISABLE        BIT(21)
#define GPIO_INT_ENABLE         BIT(22)
#define GPIO_INT_LEVELS_LOGICAL BIT(23)
#define GPIO_INT_EDGE           BIT(24)
#define GPIO_INT_LOW_0          BIT(25)
#define GPIO_INT_HIGH_1         BIT(26)

#define GPIO_INT_EDGE_RISING    (GPIO_INT_ENABLE | GPIO_INT_EDGE | \
                                 GPIO_INT_HIGH_1)
#define GPIO_INT_EDGE_FALLING   (GPIO_INT_ENABLE | GPIO_INT_EDGE | \
                                 GPIO_INT_LOW_0)
#define GPIO_INT_EDGE_BOTH      (GPIO_INT_ENABLE | GPIO_INT_EDGE | \
                                 GPIO_INT_LOW_0 | GPIO_INT_HIGH_1)
#define GPIO_INT_EDGE_TO_ACTIVE (GPIO_INT_ENABLE | GPIO_INT_LEVELS_LOGICAL | \
                                 GPIO_INT_EDGE | GPIO_INT_HIGH_1)
#define GPIO_INT_EDGE_TO_INACTIVE (GPIO_INT_ENABLE | \
                                   GPIO_INT_LEVELS_LOGICAL | \
                                   GPIO_INT_EDGE | GPIO_INT_LOW_0)
#define GPIO_INT_LEVEL_ACTIVE   (GPIO_INT_ENABLE | GPIO_INT_LEVELS_LOGICAL | \
                                 GPIO_INT_HIGH_1)

typedef uint32_t gpio_port_pins_t;
typedef uint8_t gpio_pin_t;
typedef uint16_t gpio_dt_flags_t;
typedef uint32_t gpio_flags_t;

struct gpio_dt_spec
{
  const struct device *port;
  gpio_pin_t pin;
  gpio_dt_flags_t dt_flags;
};

struct gpio_callback;

typedef void (*gpio_callback_handler_t)(const struct device *port,
                                        struct gpio_callback *cb,
                                        gpio_port_pins_t pins);

struct gpio_callback
{
  struct gpio_callback *next;   /* Zephyr: sys_snode_t node */
  gpio_callback_handler_t handler;
  gpio_port_pins_t pin_mask;
};

/* The data of the gpio0 device (Zephyr: struct gpio_driver_data and the
 * driver's callback list).
 */

struct gpio_compat_data
{
  gpio_port_pins_t invert;        /* pins configured GPIO_ACTIVE_LOW */
  struct gpio_callback *callbacks;
};

static inline bool gpio_is_ready_dt(const struct gpio_dt_spec *spec)
{
  return device_is_ready(spec->port);
}

/* Inputs only: 0, -EINVAL (pin) or -ENOTSUP (output). */

int gpio_pin_configure(const struct device *port, gpio_pin_t pin,
                       gpio_flags_t flags);

static inline int gpio_pin_configure_dt(const struct gpio_dt_spec *spec,
                                        gpio_flags_t extra_flags)
{
  return gpio_pin_configure(spec->port, spec->pin,
                            spec->dt_flags | extra_flags);
}

int gpio_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
                                 gpio_flags_t flags);

static inline int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *spec,
                                                  gpio_flags_t flags)
{
  return gpio_pin_interrupt_configure(spec->port, spec->pin, flags);
}

/* The logical level: 0 or 1. */

int gpio_pin_get(const struct device *port, gpio_pin_t pin);

static inline int gpio_pin_get_dt(const struct gpio_dt_spec *spec)
{
  return gpio_pin_get(spec->port, spec->pin);
}

static inline void gpio_init_callback(struct gpio_callback *callback,
                                      gpio_callback_handler_t handler,
                                      gpio_port_pins_t pin_mask)
{
  callback->handler = handler;
  callback->pin_mask = pin_mask;
}

/* gpio_manage_callback(): adding moves the callback to the front of the
 * list; removing one that is not there is -EINVAL.
 */

int gpio_add_callback(const struct device *port,
                      struct gpio_callback *callback);
int gpio_remove_callback(const struct device *port,
                         struct gpio_callback *callback);

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_GPIO_H */
