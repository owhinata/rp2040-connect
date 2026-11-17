/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's I2C API (<zephyr/drivers/i2c.h>, Zephyr v4.5.0) that
 * the shared sources use (the LSM6DSO driver's register access and FIFO
 * reads, the IMU service's i2c_configure()), with Zephyr's names, types,
 * values and helpers: the speed classes and the configuration word, the
 * message flags, struct i2c_msg, struct i2c_dt_spec, the driver's API table
 * (configure and transfer), i2c_configure(), i2c_transfer() and the
 * helpers built on it, as Zephyr's header has them.  Part of the FreeRTOS
 * implementation of the Zephyr compatibility layer (freertos/drivers/compat;
 * the NuttX build has its own, nuttx/drivers/compat/zephyr/drivers/i2c.h,
 * on a NuttX I2C master).
 *
 * The bus is the device of its driver, as in Zephyr: i2c0 is the I2C0
 * controller of the shared core (common/drivers/i2c_dw/nano_i2c.c), whose
 * FreeRTOS part (freertos/drivers/i2c_dw/nano_i2c_instance.h) defines the
 * device and its API table.  The calls go to the driver directly, without
 * the conversion of the NuttX build:
 *
 *   i2c_write():            1 message WRITE|STOP
 *   i2c_read():             1 message READ|STOP
 *   i2c_write_read():       WRITE, then RESTART|READ|STOP
 *   i2c_burst_read():       i2c_write_read() of the register address
 *   i2c_transfer():         0 for no message; otherwise STOP added to the
 *                           last message (CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS
 *                           off), then the driver's transfer
 *   i2c_configure():        the driver's configure (the speed class for
 *                           the later transfers on the bus, from any caller)
 *
 * Not here: target mode, callbacks and RTIO (CONFIG_I2C_CALLBACK,
 * CONFIG_I2C_RTIO: off in the Zephyr build; the LSM6DSO's FIFO stream reads
 * the FIFO with i2c_burst_read_dt() in a work item), statistics, the dump of
 * messages, i2c_get_config(), i2c_recover_bus() and the other helpers (the
 * shared sources call none of them).
 */

#ifndef FREERTOS_DRIVERS_COMPAT_ZEPHYR_DRIVERS_I2C_H
#define FREERTOS_DRIVERS_COMPAT_ZEPHYR_DRIVERS_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

/* zephyr/drivers/i2c.h: speed classes and the configuration word. */

#define I2C_SPEED_STANDARD  (0x1U) /* 100 kbit/s */
#define I2C_SPEED_FAST      (0x2U) /* 400 kbit/s */
#define I2C_SPEED_FAST_PLUS (0x3U) /* 1 Mbit/s */
#define I2C_SPEED_HIGH      (0x4U) /* 3.4 Mbit/s */
#define I2C_SPEED_ULTRA     (0x5U) /* 5 Mbit/s */
#define I2C_SPEED_DT        (0x7U)

#define I2C_SPEED_SHIFT      (1U)
#define I2C_SPEED_MASK       (0x7U << I2C_SPEED_SHIFT) /* 3 bits */
#define I2C_SPEED_SET(speed) (((speed) << I2C_SPEED_SHIFT) & I2C_SPEED_MASK)
#define I2C_SPEED_GET(cfg)   (((cfg) & I2C_SPEED_MASK) >> I2C_SPEED_SHIFT)
#define I2C_ADDR_10_BITS     BIT(0)
#define I2C_MODE_CONTROLLER  BIT(4)

#define I2C_BITRATE_STANDARD  100000U
#define I2C_BITRATE_FAST      400000U
#define I2C_BITRATE_FAST_PLUS 1000000U

/* The message flags (the same definitions as the shared core's
 * common/drivers/i2c_dw/nano_i2c_dw.h, which reads them).
 */

#define I2C_MSG_WRITE        (0U << 0U)
#define I2C_MSG_READ         BIT(0)
#define I2C_MSG_RW_MASK      BIT(0)
#define I2C_MSG_STOP         BIT(1)
#define I2C_MSG_RESTART      BIT(2)
#define I2C_MSG_ADDR_10_BITS BIT(3)

struct i2c_msg {
	uint8_t *buf;
	uint32_t len;
	uint8_t flags;
};

/* The driver's API (struct i2c_driver_api, DEVICE_API(i2c, ...)). */

struct i2c_driver_api {
	int (*configure)(const struct device *dev, uint32_t dev_config);
	int (*transfer)(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			uint16_t addr);
};

/* I2C_DT_SPEC_GET(): a device on a bus. */

struct i2c_dt_spec {
	const struct device *bus;
	uint16_t addr;
};

static inline bool i2c_is_ready_dt(const struct i2c_dt_spec *spec)
{
	return device_is_ready(spec->bus);
}

static inline int i2c_configure(const struct device *dev, uint32_t dev_config)
{
	return DEVICE_API_GET(i2c, dev)->configure(dev, dev_config);
}

static inline int i2c_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			       uint16_t addr)
{
	if (!num_msgs) {
		return 0;
	}

	msgs[num_msgs - 1].flags |= I2C_MSG_STOP;

	return DEVICE_API_GET(i2c, dev)->transfer(dev, msgs, num_msgs, addr);
}

static inline int i2c_transfer_dt(const struct i2c_dt_spec *spec, struct i2c_msg *msgs,
				  uint8_t num_msgs)
{
	return i2c_transfer(spec->bus, msgs, num_msgs, spec->addr);
}

static inline int i2c_write(const struct device *dev, const uint8_t *buf, uint32_t num_bytes,
			    uint16_t addr)
{
	struct i2c_msg msg;

	msg.buf = (uint8_t *)buf;
	msg.len = num_bytes;
	msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	return i2c_transfer(dev, &msg, 1, addr);
}

static inline int i2c_write_dt(const struct i2c_dt_spec *spec, const uint8_t *buf,
			       uint32_t num_bytes)
{
	return i2c_write(spec->bus, buf, num_bytes, spec->addr);
}

static inline int i2c_read(const struct device *dev, uint8_t *buf, uint32_t num_bytes,
			   uint16_t addr)
{
	struct i2c_msg msg;

	msg.buf = buf;
	msg.len = num_bytes;
	msg.flags = I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(dev, &msg, 1, addr);
}

static inline int i2c_read_dt(const struct i2c_dt_spec *spec, uint8_t *buf, uint32_t num_bytes)
{
	return i2c_read(spec->bus, buf, num_bytes, spec->addr);
}

static inline int i2c_write_read(const struct device *dev, uint16_t addr, const void *write_buf,
				 size_t num_write, void *read_buf, size_t num_read)
{
	struct i2c_msg msg[2];

	msg[0].buf = (uint8_t *)write_buf;
	msg[0].len = num_write;
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = (uint8_t *)read_buf;
	msg[1].len = num_read;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	return i2c_transfer(dev, msg, 2, addr);
}

static inline int i2c_write_read_dt(const struct i2c_dt_spec *spec, const void *write_buf,
				    size_t num_write, void *read_buf, size_t num_read)
{
	return i2c_write_read(spec->bus, spec->addr, write_buf, num_write, read_buf, num_read);
}

static inline int i2c_burst_read(const struct device *dev, uint16_t dev_addr, uint8_t start_addr,
				 uint8_t *buf, uint32_t num_bytes)
{
	return i2c_write_read(dev, dev_addr, &start_addr, sizeof(start_addr), buf, num_bytes);
}

static inline int i2c_burst_read_dt(const struct i2c_dt_spec *spec, uint8_t start_addr,
				    uint8_t *buf, uint32_t num_bytes)
{
	return i2c_burst_read(spec->bus, spec->addr, start_addr, buf, num_bytes);
}

#endif /* FREERTOS_DRIVERS_COMPAT_ZEPHYR_DRIVERS_I2C_H */
