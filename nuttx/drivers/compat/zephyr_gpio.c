/* SPDX-License-Identifier: Apache-2.0 */
/* The gpio0 device of <zephyr/drivers/gpio.h>: Zephyr's generic GPIO
 * calls (z_impl_gpio_pin_configure(), z_impl_gpio_pin_interrupt_configure(),
 * gpio_pin_get(), gpio_manage_callback(), gpio_fire_callbacks()) and the
 * RP2040 driver's (drivers/gpio/gpio_rpi_pico.c gpio_rpi_configure() for
 * inputs, gpio_rpi_pin_interrupt_configure(), gpio_rpi_isr()) on the
 * board's <nano_gpio.h>.  OS-independent apart from k_spin_lock().  Part
 * of the small Zephyr compatibility layer of the port
 * (nuttx/drivers/compat).
 */

#ifdef __NuttX__
#  include <nuttx/config.h>
#endif

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/spinlock.h>

#include "nano_gpio.h"

static struct gpio_compat_data gpio0_data;
static struct k_spinlock gpio0_lock;

/* drivers/gpio/gpio_rpi_pico.c gpio_rpi_bank_init(): connects and enables
 * the bank interrupt; the board attaches it with the first pin
 * (nano_gpio_irq_configure()).
 */

static int gpio0_init(const struct device *dev)
{
  (void)dev;
  return 0;
}

DEVICE_DT_DEFINE(gpio0, "gpio-port@0", gpio0_init, &gpio0_data, NULL, NULL);

int gpio_pin_configure(const struct device *port, gpio_pin_t pin,
                       gpio_flags_t flags)
{
  struct gpio_compat_data *data = port->data;
  k_spinlock_key_t key;

  if (pin >= NANO_GPIO_NUM)
    {
      return -EINVAL;
    }

  if ((flags & GPIO_OUTPUT) != 0 || (flags & GPIO_INPUT) == 0)
    {
      return -ENOTSUP;
    }

  key = k_spin_lock(&gpio0_lock);
  if ((flags & GPIO_ACTIVE_LOW) != 0)
    {
      data->invert |= (gpio_port_pins_t)BIT(pin);
    }
  else
    {
      data->invert &= ~(gpio_port_pins_t)BIT(pin);
    }

  k_spin_unlock(&gpio0_lock, key);

  return nano_gpio_configure_input(pin, (flags & GPIO_PULL_UP) != 0U,
                                   (flags & GPIO_PULL_DOWN) != 0U);
}

int gpio_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
                                 gpio_flags_t flags)
{
  const struct gpio_compat_data *data = port->data;
  uint32_t events = 0;

  if (pin >= NANO_GPIO_NUM)
    {
      return -EINVAL;
    }

  if (((flags & GPIO_INT_LEVELS_LOGICAL) != 0) &&
      ((data->invert & (gpio_port_pins_t)BIT(pin)) != 0))
    {
      /* Invert signal bits */

      flags ^= (GPIO_INT_LOW_0 | GPIO_INT_HIGH_1);
    }

  /* gpio_rpi_pin_interrupt_configure() */

  if ((flags & GPIO_INT_DISABLE) == 0)
    {
      if (flags & GPIO_INT_EDGE)
        {
          if (flags & GPIO_INT_LOW_0)
            {
              events |= NANO_GPIO_IRQ_EDGE_FALL;
            }

          if (flags & GPIO_INT_HIGH_1)
            {
              events |= NANO_GPIO_IRQ_EDGE_RISE;
            }
        }
      else
        {
          if (flags & GPIO_INT_LOW_0)
            {
              events |= NANO_GPIO_IRQ_LEVEL_LOW;
            }

          if (flags & GPIO_INT_HIGH_1)
            {
              events |= NANO_GPIO_IRQ_LEVEL_HIGH;
            }
        }
    }

  return nano_gpio_irq_configure(pin, events);
}

int gpio_pin_get(const struct device *port, gpio_pin_t pin)
{
  const struct gpio_compat_data *data = port->data;
  uint32_t value;

  if (pin >= NANO_GPIO_NUM)
    {
      return -EINVAL;
    }

  value = nano_gpio_get_all() ^ data->invert;
  return (value & BIT(pin)) != 0;
}

/* gpio_utils.h gpio_manage_callback() */

static int gpio_manage_callback(struct gpio_callback **callbacks,
                                struct gpio_callback *callback, bool set)
{
  k_spinlock_key_t key = k_spin_lock(&gpio0_lock);
  struct gpio_callback **link = callbacks;
  bool found = false;

  while (*link != NULL)
    {
      if (*link == callback)
        {
          *link = callback->next;
          found = true;
          break;
        }

      link = &(*link)->next;
    }

  if (!found && !set)
    {
      k_spin_unlock(&gpio0_lock, key);
      return -EINVAL;
    }

  if (set)
    {
      callback->next = *callbacks;
      *callbacks = callback;
    }

  k_spin_unlock(&gpio0_lock, key);
  return 0;
}

int gpio_add_callback(const struct device *port,
                      struct gpio_callback *callback)
{
  struct gpio_compat_data *data = port->data;

  return gpio_manage_callback(&data->callbacks, callback, true);
}

int gpio_remove_callback(const struct device *port,
                         struct gpio_callback *callback)
{
  struct gpio_compat_data *data = port->data;

  return gpio_manage_callback(&data->callbacks, callback, false);
}

/* gpio_rpi_isr() after clearing the events: gpio_utils.h
 * gpio_fire_callbacks() for the pin.  In the interrupt handler.
 */

void nano_gpio_irq_handler(unsigned int pin)
{
  const struct device *port = DEVICE_DT_GET(DT_NODELABEL(gpio0));
  struct gpio_compat_data *data = port->data;
  gpio_port_pins_t pins = BIT(pin);
  struct gpio_callback *cb;
  struct gpio_callback *tmp;

  for (cb = data->callbacks; cb != NULL; cb = tmp)
    {
      tmp = cb->next;
      if (cb->pin_mask & pins)
        {
          cb->handler(port, cb, cb->pin_mask & pins);
        }
    }
}
