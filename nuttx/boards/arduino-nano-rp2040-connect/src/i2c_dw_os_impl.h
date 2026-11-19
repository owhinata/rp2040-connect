/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/i2c_dw_os_impl.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2022 Andrei-Edward Popa
 * Copyright (c) 2023 TOKITA Hiroshi <tokita.hiroshi@fujitsu.com>
 *
 ****************************************************************************/

/* The NuttX part of the I2C0 core's boundary
 * (common/drivers/i2c_dw/i2c_dw_os.h, which includes this file first in
 * i2c_dw.c): the NuttX headers, and the types and calls
 * behind the boundary's names:
 *
 *   Zephyr                       here
 *   k_sem (no priority inh.)     sem_t
 *   k_sem_give()                 nxsem_post()
 *   k_sem_take(.., K_MSEC(ms))   nxsem_tickwait_uninterruptible(..,
 *                                MSEC2TICK(ms))
 *   k_sem_take(.., K_FOREVER)    nxsem_wait_uninterruptible()
 *   k_sem_reset()                nxsem_reset(.., 0)
 *   k_timepoint_t                clock_t (the system ticks)
 *   k_msleep()                   nxsched_msleep()
 *   k_uptime_get()               TICK2MSEC(clock_systime_ticks())
 *   irq_lock() (atomic_or())     up_irq_save(), up_irq_restore()
 *   struct i2c_msg               struct i2c_msg_s (i2c_dw_msg_flags()
 *                                turns the NuttX flags into I2C_MSG_*)
 *   the device                   struct i2c_master_s first in the
 *                                controller's data (i2c_dw_instance.h)
 */

#ifndef __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_I2C_DW_OS_IMPL_H
#define __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_I2C_DW_OS_IMPL_H

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/sched.h>
#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rp2040_memorymap.h"
#include "hardware/rp2040_pads_bank0.h"
#include "hardware/rp2040_resets.h"
#include "rp2040_gpio.h"

#include "i2c_dw_i2c0.h"

/* <nuttx/i2c/i2c_master.h> names the bus frequencies I2C_SPEED_*; the core
 * uses these names for Zephyr's speed classes (i2c_dw.h), as the
 * compatibility layer's <zephyr/drivers/i2c.h> does.  Nothing here uses the
 * frequencies.
 */

#undef I2C_SPEED_STANDARD
#undef I2C_SPEED_FAST
#undef I2C_SPEED_FAST_PLUS
#undef I2C_SPEED_HIGH

/* CONFIG_I2C_DW_CLOCK_SPEED of the Zephyr build: clk_peri in MHz. */

#if BOARD_PERI_FREQ % 1000000 != 0
#  error "clk_peri is not a whole number of MHz"
#endif
#define I2C_DW_CLOCK_SPEED (BOARD_PERI_FREQ / 1000000)

typedef sem_t i2c_dw_os_sem_t;
typedef clock_t i2c_dw_os_timepoint_t;
typedef irqstate_t i2c_dw_os_irqstate_t;

/* The messages of I2C_TRANSFER(): Zephyr's flags from the NuttX flags of
 * the message and its neighbours (i2c_dw.h).
 */

typedef struct i2c_msg_s i2c_dw_os_msg_t;

#define I2C_DW_OS_MSG_BUF(m) ((m)->buffer)
#define I2C_DW_OS_MSG_LEN(m) ((uint32_t)(m)->length)
#define I2C_DW_OS_MSG_FLAGS(msgs, cur, i, left)                                \
	i2c_dw_msg_flags((i) > 0 ? (msgs)[(i) - 1].flags : 0, (cur)->flags,       \
			 (left) > 1 ? (msgs)[(i) + 1].flags : 0, (i) == 0,           \
			 (left) == 1)

/* NuttX's I2C master: first, so that the driver's operations find the
 * controller from it (i2c_dw_instance.h).
 */

#define I2C_DW_OS_DEV_HEAD struct i2c_master_s dev;

static inline irqstate_t i2c_dw_os_irq_lock(void)
{
	return up_irq_save();
}

static inline void i2c_dw_os_irq_unlock(irqstate_t state)
{
	up_irq_restore(state);
}

static inline void i2c_dw_os_sem_give(sem_t *sem)
{
	nxsem_post(sem);
}

static inline int i2c_dw_os_sem_take_ms(sem_t *sem, uint32_t ms)
{
	return nxsem_tickwait_uninterruptible(sem, MSEC2TICK(ms));
}

static inline clock_t i2c_dw_os_timepoint_calc(uint32_t ms)
{
	return clock_systime_ticks() + MSEC2TICK(ms);
}

/* sys_timepoint_timeout(end): K_NO_WAIT once it has passed (a last try
 * without waiting).
 */

static inline int i2c_dw_os_sem_take_until(sem_t *sem, clock_t end)
{
	int64_t left;

	left = (int64_t)(end - clock_systime_ticks());
	if (left < 0) {
		left = 0;
	}
	return nxsem_tickwait_uninterruptible(sem, (uint32_t)left);
}

static inline void i2c_dw_os_sem_reset(sem_t *sem)
{
	nxsem_reset(sem, 0);
}

static inline int i2c_dw_os_bus_lock(sem_t *sem)
{
	return nxsem_wait_uninterruptible(sem);
}

static inline void i2c_dw_os_bus_unlock(sem_t *sem)
{
	nxsem_post(sem);
}

static inline void i2c_dw_os_msleep(uint32_t ms)
{
	nxsched_msleep(ms);
}

#define i2c_dw_os_uptime_ms() ((int64_t)TICK2MSEC(clock_systime_ticks()))

#endif /* __BOARDS_ARDUINO_NANO_RP2040_CONNECT_SRC_I2C_DW_OS_IMPL_H */
