/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's I2C API (<zephyr/drivers/i2c.h>) that the ported
 * LSM6DSO driver, the IMU service and the I2C shell (the "i2c" command,
 * common/shell/i2c_shell.c) use, on a NuttX I2C master
 * (struct i2c_master_s, I2C_TRANSFER(), I2C_RESET()).  Part of the small Zephyr
 * compatibility layer of the port (common/compat/port/nuttx,
 * zephyr_i2c.c there; see common/compat/README.md).
 *
 * The bus is a struct device whose data is a struct i2c_compat_bus: the
 * NuttX I2C master and the bus speed.  Zephyr keeps the speed in the bus
 * driver (i2c_configure() changes it for every later transfer on the bus,
 * from any caller); a NuttX transfer carries its own frequency in each
 * message.  Here the bus device keeps the speed that i2c_configure() last
 * set (at first the devicetree's clock-frequency, BOARD_I2C0_FREQUENCY) and
 * every transfer made through this API uses it; i2c_get_config() reports
 * it.  Other users of the NuttX master pass their own frequency and are not
 * affected.
 *
 * Transfers become NuttX messages as Zephyr's helpers build theirs:
 *   i2c_write_read()/i2c_burst_read(): write (no STOP), then read with
 *     RESTART and STOP: msgs {I2C_M_NOSTOP}, {I2C_M_READ}
 *   i2c_write(): one write with STOP: msg {0}
 *   i2c_read(): one read with STOP: msg {I2C_M_READ}
 *   i2c_transfer(): Zephyr's messages, STOP added to the last one, each
 *     with the NuttX flags of the same bus sequence (see zephyr_i2c.c)
 * (the board's driver turns the NuttX flags back into Zephyr's,
 * i2c_dw.h i2c_dw_msg_flags()).  The result is the driver's: 0 or
 * a negative errno (-ENOTSUP for a message of no bytes, as in the Zephyr
 * build).  i2c_recover_bus() is I2C_RESET() (-ENOTSUP from the board's
 * driver, as Zephyr's i2c_dw_recovery_bus() without a recover-bus
 * callback).  No target mode: i2c_target_driver_register() and
 * _unregister() return -ENOSYS (CONFIG_I2C_TARGET is off in the Zephyr
 * build; the I2C shell builds no "i2c target").
 *
 * NuttX's <nuttx/i2c/i2c_master.h> has functions named i2c_write() and
 * i2c_read() of its own and I2C_SPEED_* in Hz: the Zephyr calls on a bus
 * device are i2c_compat_write(), i2c_compat_read() and
 * i2c_compat_write_read(), and a source that does not include NuttX's
 * header (the I2C shell) gets them under Zephyr's names i2c_write() and
 * i2c_read() too (a source that does includes it first).  i2c_write_read()
 * and the _dt forms keep Zephyr's names.  I2C_SPEED_* are Zephyr's speed
 * classes in any file that includes this header (after NuttX's, if both
 * are needed).
 */

#ifndef __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_DRIVERS_I2C_H
#define __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_DRIVERS_I2C_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>

/* zephyr/drivers/i2c.h: speed classes and the configuration word. */

#undef I2C_SPEED_STANDARD
#undef I2C_SPEED_FAST
#undef I2C_SPEED_FAST_PLUS
#undef I2C_SPEED_HIGH

#define I2C_SPEED_STANDARD   (0x1U)  /* 100 kbit/s */
#define I2C_SPEED_FAST       (0x2U)  /* 400 kbit/s */
#define I2C_SPEED_FAST_PLUS  (0x3U)  /* 1 Mbit/s */
#define I2C_SPEED_HIGH       (0x4U)  /* 3.4 Mbit/s */
#define I2C_SPEED_ULTRA      (0x5U)  /* 5 Mbit/s */
#define I2C_SPEED_DT         (0x7U)

#define I2C_SPEED_SHIFT      (1U)
#define I2C_SPEED_MASK       (0x7U << I2C_SPEED_SHIFT)
#define I2C_SPEED_SET(speed) (((speed) << I2C_SPEED_SHIFT) & I2C_SPEED_MASK)
#define I2C_SPEED_GET(cfg)   (((cfg) & I2C_SPEED_MASK) >> I2C_SPEED_SHIFT)
#define I2C_ADDR_10_BITS     BIT(0)
#define I2C_MODE_CONTROLLER  BIT(4)

#define I2C_BITRATE_STANDARD   100000U
#define I2C_BITRATE_FAST       400000U
#define I2C_BITRATE_FAST_PLUS  1000000U

/* The message flags and struct i2c_msg (zephyr/drivers/i2c.h). */

#define I2C_MSG_WRITE         (0U << 0U)
#define I2C_MSG_READ          BIT(0)
#define I2C_MSG_RW_MASK       BIT(0)
#define I2C_MSG_STOP          BIT(1)
#define I2C_MSG_RESTART       BIT(2)
#define I2C_MSG_ADDR_10_BITS  BIT(3)

struct i2c_msg
{
  uint8_t *buf;
  uint32_t len;
  uint8_t flags;
};

/* The most messages of one i2c_transfer() (-EINVAL above). */

#define I2C_COMPAT_MAX_MSGS   4

struct i2c_master_s;

/* The data of an I2C bus device. */

struct i2c_compat_bus
{
  struct i2c_master_s *master;   /* NULL until the board binds it */
  volatile uint32_t frequency;   /* Hz of the messages, i2c_configure() */
};

/* I2C_DT_SPEC_GET(): a device on a bus. */

struct i2c_dt_spec
{
  const struct device *bus;
  uint16_t addr;
};

/* The board makes a bus device ready: binds its NuttX master and sets the
 * devicetree's speed.  Returns 0 (the device is then ready), or -EINVAL for
 * a NULL master or a speed i2c_configure() would not take.
 */

int i2c_compat_bind(const struct device *dev, struct i2c_master_s *master,
                    uint32_t frequency);

/* i2c_configure(): sets the speed class of the bus for the later
 * transfers.  Standard (100 kHz), fast (400 kHz) and fast plus (1 MHz)
 * only, as the RP2040's DesignWare driver of the Zephyr build takes them
 * (i2c_dw.c i2c_dw_configure(): high speed is refused on the RP2040, ultra
 * and the rest are -EINVAL); the mode bits are ignored (that driver forces
 * controller mode).  Returns 0, -EINVAL (nothing changed) or -ENODEV.
 */

int i2c_configure(const struct device *dev, uint32_t dev_config);

/* i2c_get_config(): the configuration word of the bus,
 * I2C_MODE_CONTROLLER and the speed class of its frequency.  Returns 0 or
 * -ENODEV.
 */

int i2c_get_config(const struct device *dev, uint32_t *dev_config);

/* The speed of the bus in Hz (not a Zephyr call: i2c_get_config() gives
 * the configuration word).
 */

uint32_t i2c_compat_frequency(const struct device *dev);

/* Zephyr's i2c_transfer(), i2c_write(), i2c_read() and i2c_write_read(). */

int i2c_transfer(const struct device *dev, struct i2c_msg *msgs,
                 uint8_t num_msgs, uint16_t addr);
int i2c_compat_write(const struct device *dev, const uint8_t *buf,
                     uint32_t num_bytes, uint16_t addr);
int i2c_compat_read(const struct device *dev, uint8_t *buf,
                    uint32_t num_bytes, uint16_t addr);
int i2c_compat_write_read(const struct device *dev, uint16_t addr,
                          const void *write_buf, size_t num_write,
                          void *read_buf, size_t num_read);

static inline int i2c_write_read(const struct device *dev, uint16_t addr,
                                 const void *write_buf, size_t num_write,
                                 void *read_buf, size_t num_read)
{
  return i2c_compat_write_read(dev, addr, write_buf, num_write, read_buf,
                               num_read);
}

#ifndef __INCLUDE_NUTTX_I2C_I2C_MASTER_H
static inline int i2c_write(const struct device *dev, const uint8_t *buf,
                            uint32_t num_bytes, uint16_t addr)
{
  return i2c_compat_write(dev, buf, num_bytes, addr);
}

static inline int i2c_read(const struct device *dev, uint8_t *buf,
                           uint32_t num_bytes, uint16_t addr)
{
  return i2c_compat_read(dev, buf, num_bytes, addr);
}
#endif

/* i2c_recover_bus(): I2C_RESET() of the bus's master (CONFIG_I2C_RESET;
 * -ENOSYS without it, as Zephyr's for a driver without recover_bus).
 * Returns the driver's result or -ENODEV.
 */

int i2c_recover_bus(const struct device *dev);

/* Target mode: no target drivers. */

static inline int i2c_target_driver_register(const struct device *dev)
{
  (void)dev;
  return -ENOSYS;
}

static inline int i2c_target_driver_unregister(const struct device *dev)
{
  (void)dev;
  return -ENOSYS;
}

static inline int i2c_write_dt(const struct i2c_dt_spec *spec,
                               const uint8_t *buf, uint32_t num_bytes)
{
  return i2c_compat_write(spec->bus, buf, num_bytes, spec->addr);
}

static inline int i2c_write_read_dt(const struct i2c_dt_spec *spec,
                                    const void *write_buf, size_t num_write,
                                    void *read_buf, size_t num_read)
{
  return i2c_compat_write_read(spec->bus, spec->addr, write_buf, num_write,
                               read_buf, num_read);
}

/* i2c_burst_read(): the register address, then the bytes from it. */

static inline int i2c_burst_read_dt(const struct i2c_dt_spec *spec,
                                    uint8_t start_addr, uint8_t *buf,
                                    uint32_t num_bytes)
{
  return i2c_compat_write_read(spec->bus, spec->addr, &start_addr,
                               sizeof(start_addr), buf, num_bytes);
}

static inline bool i2c_is_ready_dt(const struct i2c_dt_spec *spec)
{
  return device_is_ready(spec->bus);
}

#endif /* __COMMON_COMPAT_PORT_NUTTX_ZEPHYR_DRIVERS_I2C_H */
