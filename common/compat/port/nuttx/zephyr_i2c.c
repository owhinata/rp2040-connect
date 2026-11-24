/* SPDX-License-Identifier: Apache-2.0 */
/* Zephyr's I2C calls on a NuttX I2C master (see <zephyr/drivers/i2c.h> of
 * this directory).  Part of the small Zephyr compatibility layer of the
 * port (common/compat/port/nuttx).
 *
 * Zephyr (zephyr/include/zephyr/drivers/i2c.h, kernel/device.c)  here
 *   i2c_write(): 1 message WRITE|STOP            {flags 0}
 *   i2c_write_read(): WRITE, then                {I2C_M_NOSTOP},
 *     RESTART|READ|STOP                          {I2C_M_READ}
 *   i2c_read(): 1 message READ|STOP              {I2C_M_READ}
 *   i2c_transfer(): STOP on the last message,    the same messages with
 *     then the driver's transfer                 NuttX's flags (below)
 *   i2c_configure(): the bus driver's speed      the bus device's frequency,
 *                                                put in each message
 *   i2c_get_config(): the driver's get_config    I2C_MODE_CONTROLLER and the
 *                                                class of the frequency
 *   i2c_recover_bus(): the driver's recover_bus  I2C_RESET()
 *
 * The NuttX flags of a Zephyr message, the inverse of the board driver's
 * i2c_dw_msg_flags() (i2c_dw.h): I2C_M_READ for I2C_MSG_READ, I2C_M_TEN
 * for I2C_MSG_ADDR_10_BITS (which the driver refuses), I2C_M_NOSTOP on a
 * message without STOP (all but the last), and I2C_M_NOSTART on a message
 * that follows one without STOP and has no RESTART (a continuation).
 */

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* NuttX's header first: <zephyr/drivers/i2c.h> replaces its I2C_SPEED_*. */

#include <nuttx/i2c/i2c_master.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct i2c_compat_bus *bus_of(const struct device *dev)
{
  if (!device_is_ready(dev))
    {
      return NULL;
    }

  return (struct i2c_compat_bus *)dev->data;
}

/* The bit rate of a speed class (zephyr/drivers/i2c.h, I2C_BITRATE_*), for
 * the classes the RP2040's DesignWare driver accepts; 0 for the others.
 */

static uint32_t speed_frequency(uint32_t speed)
{
  switch (speed)
    {
      case I2C_SPEED_STANDARD:
        return I2C_BITRATE_STANDARD;

      case I2C_SPEED_FAST:
        return I2C_BITRATE_FAST;

      case I2C_SPEED_FAST_PLUS:
        return I2C_BITRATE_FAST_PLUS;

      default:
        return 0;
    }
}

static int transfer(const struct device *dev, struct i2c_msg_s *msgs,
                    int count)
{
  struct i2c_compat_bus *bus = bus_of(dev);
  int i;

  if (bus == NULL)
    {
      return -ENODEV;
    }

  for (i = 0; i < count; i++)
    {
      msgs[i].frequency = bus->frequency;
    }

  return I2C_TRANSFER(bus->master, msgs, count);
}

/* The speed class of a bit rate, the inverse of speed_frequency(). */

static uint32_t frequency_speed(uint32_t frequency)
{
  switch (frequency)
    {
      case I2C_BITRATE_STANDARD:
        return I2C_SPEED_STANDARD;

      case I2C_BITRATE_FAST:
        return I2C_SPEED_FAST;

      case I2C_BITRATE_FAST_PLUS:
        return I2C_SPEED_FAST_PLUS;

      default:
        return 0;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int i2c_compat_bind(const struct device *dev, struct i2c_master_s *master,
                    uint32_t frequency)
{
  struct i2c_compat_bus *bus;

  if (dev == NULL || master == NULL ||
      (frequency != I2C_BITRATE_STANDARD && frequency != I2C_BITRATE_FAST &&
       frequency != I2C_BITRATE_FAST_PLUS))
    {
      return -EINVAL;
    }

  bus = (struct i2c_compat_bus *)dev->data;
  bus->master = master;
  bus->frequency = frequency;
  dev->state->init_res = 0;
  dev->state->initialized = true;
  return 0;
}

int i2c_configure(const struct device *dev, uint32_t dev_config)
{
  struct i2c_compat_bus *bus = bus_of(dev);
  uint32_t frequency;

  if (bus == NULL)
    {
      return -ENODEV;
    }

  frequency = speed_frequency(I2C_SPEED_GET(dev_config));
  if (frequency == 0)
    {
      return -EINVAL;
    }

  bus->frequency = frequency;
  return 0;
}

int i2c_get_config(const struct device *dev, uint32_t *dev_config)
{
  struct i2c_compat_bus *bus = bus_of(dev);

  if (bus == NULL)
    {
      return -ENODEV;
    }

  *dev_config = I2C_MODE_CONTROLLER |
                I2C_SPEED_SET(frequency_speed(bus->frequency));
  return 0;
}

int i2c_recover_bus(const struct device *dev)
{
  struct i2c_compat_bus *bus = bus_of(dev);

  if (bus == NULL)
    {
      return -ENODEV;
    }

#ifdef CONFIG_I2C_RESET
  return I2C_RESET(bus->master);
#else
  return -ENOSYS;
#endif
}

int i2c_transfer(const struct device *dev, struct i2c_msg *msgs,
                 uint8_t num_msgs, uint16_t addr)
{
  struct i2c_msg_s nmsgs[I2C_COMPAT_MAX_MSGS];
  int i;

  if (num_msgs == 0)
    {
      return 0;
    }

  if (num_msgs > I2C_COMPAT_MAX_MSGS)
    {
      return -EINVAL;
    }

  msgs[num_msgs - 1].flags |= I2C_MSG_STOP;

  for (i = 0; i < num_msgs; i++)
    {
      uint8_t flags = msgs[i].flags;

      nmsgs[i].frequency = 0;
      nmsgs[i].addr = addr;
      nmsgs[i].flags = 0;
      nmsgs[i].buffer = msgs[i].buf;
      nmsgs[i].length = (ssize_t)msgs[i].len;

      if (flags & I2C_MSG_READ)
        {
          nmsgs[i].flags |= I2C_M_READ;
        }

      if (flags & I2C_MSG_ADDR_10_BITS)
        {
          nmsgs[i].flags |= I2C_M_TEN;
        }

      if (!(flags & I2C_MSG_STOP))
        {
          nmsgs[i].flags |= I2C_M_NOSTOP;
        }

      if (i > 0 && !(msgs[i - 1].flags & I2C_MSG_STOP) &&
          !(flags & I2C_MSG_RESTART))
        {
          nmsgs[i].flags |= I2C_M_NOSTART;
        }
    }

  return transfer(dev, nmsgs, num_msgs);
}

uint32_t i2c_compat_frequency(const struct device *dev)
{
  struct i2c_compat_bus *bus = bus_of(dev);

  return bus != NULL ? bus->frequency : 0;
}

int i2c_compat_write(const struct device *dev, const uint8_t *buf,
                     uint32_t num_bytes, uint16_t addr)
{
  struct i2c_msg_s msg;

  msg.frequency = 0;
  msg.addr = addr;
  msg.flags = 0;
  msg.buffer = (uint8_t *)buf;
  msg.length = (ssize_t)num_bytes;
  return transfer(dev, &msg, 1);
}

int i2c_compat_read(const struct device *dev, uint8_t *buf,
                    uint32_t num_bytes, uint16_t addr)
{
  struct i2c_msg_s msg;

  msg.frequency = 0;
  msg.addr = addr;
  msg.flags = I2C_M_READ;
  msg.buffer = buf;
  msg.length = (ssize_t)num_bytes;
  return transfer(dev, &msg, 1);
}

int i2c_compat_write_read(const struct device *dev, uint16_t addr,
                          const void *write_buf, size_t num_write,
                          void *read_buf, size_t num_read)
{
  struct i2c_msg_s msgs[2];

  msgs[0].frequency = 0;
  msgs[0].addr = addr;
  msgs[0].flags = I2C_M_NOSTOP;
  msgs[0].buffer = (uint8_t *)write_buf;
  msgs[0].length = (ssize_t)num_write;

  msgs[1].frequency = 0;
  msgs[1].addr = addr;
  msgs[1].flags = I2C_M_READ;
  msgs[1].buffer = (uint8_t *)read_buf;
  msgs[1].length = (ssize_t)num_read;

  return transfer(dev, msgs, 2);
}
