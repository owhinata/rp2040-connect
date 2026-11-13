/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's I2C API (<zephyr/drivers/i2c.h>) that the ported
 * LSM6DSO driver and the IMU service use, on a NuttX I2C master
 * (struct i2c_master_s, I2C_TRANSFER()).  Part of the small Zephyr
 * compatibility layer of the port (nuttx/drivers/compat,
 * compat/zephyr_i2c.c; see nuttx/drivers/README.md).
 *
 * The bus is a struct device whose data is a struct i2c_compat_bus: the
 * NuttX I2C master and the bus speed.  Zephyr keeps the speed in the bus
 * driver (i2c_configure() changes it for every later transfer on the bus,
 * from any caller); a NuttX transfer carries its own frequency in each
 * message.  Here the bus device keeps the speed that i2c_configure() last
 * set (at first the devicetree's clock-frequency, BOARD_I2C0_FREQUENCY) and
 * every transfer made through this API uses it.  Other users of the NuttX
 * master (the "i2c" tool) pass their own frequency and are not affected.
 *
 * Transfers become NuttX messages as Zephyr's helpers build theirs:
 *   i2c_write_read()/i2c_burst_read(): write (no STOP), then read with
 *     RESTART and STOP: msgs {I2C_M_NOSTOP}, {I2C_M_READ}
 *   i2c_write(): one write with STOP: msg {0}
 * (the board's driver turns the NuttX flags back into Zephyr's,
 * nano_i2c_dw.h nano_i2c_dw_msg_flags()).  The result is the driver's: 0 or
 * a negative errno.
 *
 * NuttX's <nuttx/i2c/i2c_master.h> has functions named i2c_write() and
 * i2c_read() of its own and I2C_SPEED_* in Hz: the Zephyr calls on a bus
 * device (i2c_write(), i2c_write_read(), i2c_burst_read()) are here
 * i2c_compat_write() and i2c_compat_write_read(), and the _dt forms keep
 * Zephyr's names.  I2C_SPEED_* are Zephyr's speed classes in any file that
 * includes this header (after NuttX's, if both are needed).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_I2C_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_I2C_H

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

/* The speed of the bus in Hz (not a Zephyr call: i2c_get_config() gives
 * the configuration word).
 */

uint32_t i2c_compat_frequency(const struct device *dev);

/* Zephyr's i2c_write() and i2c_write_read(). */

int i2c_compat_write(const struct device *dev, const uint8_t *buf,
                     uint32_t num_bytes, uint16_t addr);
int i2c_compat_write_read(const struct device *dev, uint16_t addr,
                          const void *write_buf, size_t num_write,
                          void *read_buf, size_t num_read);

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

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_I2C_H */
