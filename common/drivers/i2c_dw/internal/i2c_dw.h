/*
 * SPDX-License-Identifier: Apache-2.0
 */

/* Definitions and pure functions of the I2C driver (i2c_dw.c), without
 * OS headers, so the host tests (common/tests/i2c_dw.c) can check them:
 *
 * - the definitions of the Zephyr build's i2c_dw.h (Zephyr's, with the
 *   local patch 0003, see i2c_dw.c; i2c_dw_rp2040.h) that the controller
 *   path uses: state bits, interrupt sets, the SCL count formulas and the
 *   DesignWare speed codes, with Zephyr's message flags and speed classes
 *   (zephyr/include/zephyr/drivers/i2c.h);
 * - the SCL counts of i2c_dw_configure() (i2c_dw.c:1144-1214);
 * - the command words that i2c_dw_data_send() and i2c_dw_data_ask() write
 *   to IC_DATA_CMD (i2c_dw.c:497-527, 409-437), and the RESTART that
 *   i2c_dw_transfer() adds when the direction changes (i2c_dw.c:1021-1031);
 * - what the NuttX build adds: the NuttX message flags (I2C_M_*) turned
 *   into Zephyr's (I2C_MSG_*) for one message of a transfer, and the NuttX
 *   frequency turned into a Zephyr speed class (its part of the driver,
 *   i2c_dw_instance.h and i2c_dw_os_impl.h of port/nuttx, uses them).
 *
 * The Zephyr build's values: CONFIG_I2C_DW_CLOCK_SPEED=125 (MHz, clk_peri),
 * CONFIG_I2C_DW_IC_CLK_FREQ_OPTIMIZATION off, no lcnt-offset/hcnt-offset/
 * fs-spike-len/hs-spike-len in the devicetree (so 0, 0, 1, 1).
 */

#ifndef COMMON_DRIVERS_I2C_DW_I2C_DW_H_
#define COMMON_DRIVERS_I2C_DW_I2C_DW_H_

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "i2c_dw_registers.h"

/* zephyr/sys/util.h: the core's own, unless the OS's headers, read first,
 * already have them.
 */

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#endif

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/* Zephyr's message flags (zephyr/include/zephyr/drivers/i2c.h:204-228). */

#define I2C_MSG_WRITE        (0U << 0U)
#define I2C_MSG_READ         BIT(0)
#define I2C_MSG_RW_MASK      BIT(0)
#define I2C_MSG_STOP         BIT(1)
#define I2C_MSG_RESTART      BIT(2)
#define I2C_MSG_ADDR_10_BITS BIT(3)

/* Zephyr's speed classes (zephyr/include/zephyr/drivers/i2c.h:43-55),
 * written as Zephyr writes them, so that a compatibility layer's
 * <zephyr/drivers/i2c.h> with the same definitions can be included too (as
 * for I2C_MSG_*).  NuttX's <nuttx/i2c/i2c_master.h> uses these names for
 * frequencies; the NuttX build's i2c_dw_os_impl.h drops those.
 */

#define I2C_SPEED_STANDARD  (0x1U) /* 100 kbit/s */
#define I2C_SPEED_FAST      (0x2U) /* 400 kbit/s */
#define I2C_SPEED_FAST_PLUS (0x3U) /* 1 Mbit/s */
#define I2C_SPEED_HIGH      (0x4U) /* 3.4 Mbit/s */
#define I2C_SPEED_ULTRA     (0x5U) /* 5 Mbit/s */

/* The NuttX message flags (nuttx/include/nuttx/i2c/i2c_master.h:95-98);
 * the NuttX build's i2c_dw_instance.h checks that they match.
 */

#define I2C_DW_M_READ    0x0001
#define I2C_DW_M_TEN     0x0002
#define I2C_DW_M_NOSTOP  0x0040
#define I2C_DW_M_NOSTART 0x0080

/* i2c_dw.h:34 */

#define I2C_DW_MAGIC_KEY 0x44570140

/* i2c_dw.h:39-56: dw->state */

#define I2C_DW_STATE_READY (0)
#define I2C_DW_CMD_SEND    (1 << 0)
#define I2C_DW_CMD_RECV    (1 << 1)
#define I2C_DW_CMD_ERROR   (1 << 2)
#define I2C_DW_BUSY        (1 << 3)
#define I2C_DW_USER_ABRT   (1 << 4)
#define I2C_DW_NACK        (1 << 5)
#define I2C_DW_SCL_STUCK   (1 << 6)
#define I2C_DW_SDA_STUCK   (1 << 7)

#define I2C_DW_ERR_MASK (I2C_DW_CMD_ERROR | I2C_DW_SCL_STUCK | I2C_DW_SDA_STUCK | I2C_DW_NACK)

#define I2C_DW_STUCK_ERR_MASK (I2C_DW_SCL_STUCK | I2C_DW_SDA_STUCK | I2C_DW_USER_ABRT)

/* i2c_dw.h:62-71 (CONFIG_I2C_DW_EXTENDED_SUPPORT off) */

#define DW_ENABLE_TX_INT_I2C_CONTROLLER                                                            \
	(DW_INTR_STAT_TX_OVER | DW_INTR_STAT_TX_EMPTY | DW_INTR_STAT_TX_ABRT |                     \
	 DW_INTR_STAT_STOP_DET)
#define DW_ENABLE_RX_INT_I2C_CONTROLLER                                                            \
	(DW_INTR_STAT_RX_UNDER | DW_INTR_STAT_RX_OVER | DW_INTR_STAT_RX_FULL |                     \
	 DW_INTR_STAT_STOP_DET)

#define DW_DISABLE_ALL_I2C_INT 0x00000000

/* i2c_dw.h:78-85, with CONFIG_I2C_DW_CLOCK_SPEED as an argument (MHz). */

#define I2C_STD_HCNT(mhz) ((mhz) * 4)
#define I2C_STD_LCNT(mhz) ((mhz) * 5)
#define I2C_FS_HCNT(mhz)  (((mhz) * 6) / 8)
#define I2C_FS_LCNT(mhz)  (((mhz) * 7) / 8)
#define I2C_FSP_HCNT(mhz) (((mhz) * 2) / 8)
#define I2C_FSP_LCNT(mhz) (((mhz) * 2) / 8)
#define I2C_HS_HCNT(mhz)  (((mhz) * 6) / 8)
#define I2C_HS_LCNT(mhz)  (((mhz) * 7) / 8)

/* i2c_dw.h:109-129 (CONFIG_I2C_DW_IC_CLK_FREQ_OPTIMIZATION off) */

#define I2C_MIN_SCL_LCNT(spk)  ((spk) + 8)
#define I2C_MIN_SCL_HCNT(spk)  ((spk) + 6)

#define I2C_ENSURE_MIN_SCL_HCNT(x, spk_len)	\
	(((x) < I2C_MIN_SCL_HCNT(spk_len)) ?	\
	I2C_MIN_SCL_HCNT(spk_len) : (x))

#define I2C_ENSURE_MIN_SCL_LCNT(x, spk_len)	\
	(((x) < I2C_MIN_SCL_LCNT(spk_len)) ?	\
	I2C_MIN_SCL_LCNT(spk_len) : (x))

/* i2c_dw.h:135-138 */

#define I2C_DW_SPEED_STANDARD  0x1
#define I2C_DW_SPEED_FAST      0x2
#define I2C_DW_SPEED_FAST_PLUS 0x2
#define I2C_DW_SPEED_HIGH      0x3

/* i2c_dw.h:148 */

#define SDA_HOLD_INVALID UINT32_MAX

/* The constant part of the Zephyr build's i2c_dw_rom_config for i2c0, the
 * same for i2c1, whose node has none of these properties either
 * (i2c_dw.c:1757-1771 with the devicetree of the build): no SDA hold
 * property (SDA_HOLD_INVALID: IC_SDA_HOLD keeps its reset value), offsets
 * 0, spike lengths MAX(0, DW_IC_SPKLEN_MIN) = 1, transfer timeout
 * CONFIG_I2C_TRANSFER_TIMEOUT_MS = 1000 ms.
 */

#define I2C_DW_LCNT_OFFSET       0
#define I2C_DW_HCNT_OFFSET       0
#define I2C_DW_FS_SPK_LEN        DW_IC_SPKLEN_MIN
#define I2C_DW_HS_SPK_LEN        DW_IC_SPKLEN_MIN
#define I2C_DW_TRANSFER_TIMEOUT_MS 1000

/* The SCL counts of i2c_dw_configure() (i2c_dw.c:1156-1194) for a speed
 * class.  Returns 0, or -EINVAL for a class the controller does not run
 * (as i2c_dw_configure() does; it still stores the minimum counts then,
 * see i2c_dw.c).
 */

static inline int i2c_dw_scl_counts(uint32_t speed, uint32_t clock_mhz, bool support_hs_mode,
				    uint16_t *hcnt, uint16_t *lcnt)
{
	uint32_t lcnt_val = 0U;
	uint32_t hcnt_val = 0U;
	int rc = 0;

	switch (speed) {
	case I2C_SPEED_STANDARD:
		lcnt_val = I2C_STD_LCNT(clock_mhz) + I2C_DW_LCNT_OFFSET;
		hcnt_val = I2C_STD_HCNT(clock_mhz) + I2C_DW_HCNT_OFFSET;
		break;
	case I2C_SPEED_FAST:
		lcnt_val = I2C_FS_LCNT(clock_mhz) + I2C_DW_LCNT_OFFSET;
		hcnt_val = I2C_FS_HCNT(clock_mhz) + I2C_DW_HCNT_OFFSET;
		break;
	case I2C_SPEED_FAST_PLUS:
		lcnt_val = I2C_FSP_LCNT(clock_mhz) + I2C_DW_LCNT_OFFSET;
		hcnt_val = I2C_FSP_HCNT(clock_mhz) + I2C_DW_HCNT_OFFSET;
		break;
	case I2C_SPEED_HIGH:
		if (support_hs_mode) {
			lcnt_val = I2C_HS_LCNT(clock_mhz) + I2C_DW_LCNT_OFFSET;
			hcnt_val = I2C_HS_HCNT(clock_mhz) + I2C_DW_HCNT_OFFSET;
		} else {
			rc = -EINVAL;
		}
		break;
	default:
		rc = -EINVAL;
	}

	if (speed == I2C_SPEED_HIGH) {
		lcnt_val = I2C_ENSURE_MIN_SCL_LCNT(lcnt_val, I2C_DW_HS_SPK_LEN);
		hcnt_val = I2C_ENSURE_MIN_SCL_HCNT(hcnt_val, I2C_DW_HS_SPK_LEN);
	} else {
		lcnt_val = I2C_ENSURE_MIN_SCL_LCNT(lcnt_val, I2C_DW_FS_SPK_LEN);
		hcnt_val = I2C_ENSURE_MIN_SCL_HCNT(hcnt_val, I2C_DW_FS_SPK_LEN);
	}
	*lcnt = (uint16_t)lcnt_val;
	*hcnt = (uint16_t)hcnt_val;
	return rc;
}

/* The speed class of a NuttX message frequency: the bit rates that
 * Zephyr's i2c_map_dt_bitrate() (zephyr/drivers/i2c/i2c-priv.h:25-41)
 * accepts, and nothing else (the Zephyr API has no other speeds).  Returns
 * 0 for any other frequency.
 */

static inline uint32_t i2c_dw_speed(uint32_t frequency)
{
	switch (frequency) {
	case 100000:
		return I2C_SPEED_STANDARD;
	case 400000:
		return I2C_SPEED_FAST;
	case 1000000:
		return I2C_SPEED_FAST_PLUS;
	case 3400000:
		return I2C_SPEED_HIGH;
	case 5000000:
		return I2C_SPEED_ULTRA;
	}
	return 0;
}

/* Zephyr's flags for message cur of a NuttX transfer, from the NuttX
 * flags of the previous (prev), this and the next (next) message; first
 * and last tell where it is (prev and next are not used there).
 *
 * - READ from I2C_M_READ.
 * - STOP after the last message (z_impl_i2c_transfer() sets I2C_MSG_STOP
 *   on the last message, CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS is off), and
 *   after any other message unless it has I2C_M_NOSTOP or the next one has
 *   I2C_M_NOSTART (NuttX's rule, i2c_master.h:64-92).
 * - RESTART on a message that follows one without STOP, unless it has
 *   I2C_M_NOSTART (a continuation).  After a STOP the controller starts
 *   the next command with a START by itself.  i2c_dw_transfer() adds a
 *   RESTART on a change of direction in any case.
 */

static inline uint8_t i2c_dw_msg_flags(uint16_t prev, uint16_t cur, uint16_t next, bool first,
				       bool last)
{
	uint8_t flags = (cur & I2C_DW_M_READ) ? I2C_MSG_READ : I2C_MSG_WRITE;

	if (last || (!(cur & I2C_DW_M_NOSTOP) && !(next & I2C_DW_M_NOSTART))) {
		flags |= I2C_MSG_STOP;
	}
	if (!first && !(cur & I2C_DW_M_NOSTART) && (prev & I2C_DW_M_NOSTOP)) {
		flags |= I2C_MSG_RESTART;
	}
	return flags;
}

/* i2c_dw_transfer() (i2c_dw.c:1021-1031): the flags of the next message,
 * with a RESTART when the direction differs from the message before it
 * (pflags: dw->xfr_flags, which keeps the last message of the previous
 * transfer at the start of a transfer).
 */

static inline uint8_t i2c_dw_next_flags(uint8_t pflags, uint8_t flags)
{
	if ((pflags & I2C_MSG_RW_MASK) != (flags & I2C_MSG_RW_MASK)) {
		flags |= I2C_MSG_RESTART;
	}
	return flags;
}

/* One write command of i2c_dw_data_send() (i2c_dw.c:499-510): the byte,
 * RESTART on the first command of the message if asked (and cleared from
 * *flags), STOP on the last byte (len: bytes left, this one included).
 */

static inline uint32_t i2c_dw_write_cmd(uint8_t *flags, uint8_t byte, uint32_t len)
{
	uint32_t data = byte;

	if (*flags & I2C_MSG_RESTART) {
		data |= IC_DATA_CMD_RESTART;
		*flags &= ~(I2C_MSG_RESTART);
	}
	if ((len == 1U) && (*flags & I2C_MSG_STOP)) {
		data |= IC_DATA_CMD_STOP;
	}
	return data;
}

/* One read request of i2c_dw_data_ask() (i2c_dw.c:411-422); request_bytes:
 * requests left, this one included.
 */

static inline uint32_t i2c_dw_read_cmd(uint8_t *flags, uint32_t request_bytes)
{
	uint32_t data = IC_DATA_CMD_CMD;

	if (*flags & I2C_MSG_RESTART) {
		data |= IC_DATA_CMD_RESTART;
		*flags &= ~(I2C_MSG_RESTART);
	}
	if ((*flags & I2C_MSG_STOP) && (request_bytes == 1U)) {
		data |= IC_DATA_CMD_STOP;
	}
	return data;
}

#endif /* COMMON_DRIVERS_I2C_DW_I2C_DW_H_ */
