/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the I2C core's boundary
 * (common/drivers/i2c_dw/i2c_dw_os.h, which includes this file first in
 * i2c_dw.c): the types and calls behind the boundary's names, on the
 * FreeRTOS part of the Zephyr compatibility layer (freertos/drivers/compat)
 * and the Pico SDK:
 *
 *   Zephyr                       here
 *   k_sem (no priority inh.)     the layer's k_sem (Zephyr's rules: a give
 *                                hands the count to the first waiter; from
 *                                an interrupt handler it wakes the task with
 *                                the FromISR task notification and a switch
 *                                at the end of the handler, not through the
 *                                timer task "Tmr Svc"; zephyr_kernel.c)
 *   k_sem_give()                 k_sem_give()
 *   k_sem_take(.., K_MSEC(ms))   k_sem_take(.., K_MSEC(ms)): ms rounded up
 *                                to ticks (1 ms), plus one tick (Zephyr's
 *                                rounding and wake-up)
 *   k_sem_take(.., K_FOREVER)    k_sem_take(.., K_FOREVER) (never fails)
 *   k_sem_reset()                k_sem_reset()
 *   k_timepoint_t                the FreeRTOS tick count (32 bits; the
 *                                differences are taken as signed, so the
 *                                1 s deadline is far from the wrap)
 *   sys_timepoint_calc(K_MSEC)   now + the ticks of K_MSEC(ms) (at least
 *                                one, as Zephyr's MAX(1, dt))
 *   k_sem_take(.., sys_timepoint_timeout(end))
 *                                k_sem_take(.., K_TICKS(end - now)), 0 once
 *                                end has passed (K_NO_WAIT: one try)
 *   k_msleep()                   k_msleep()
 *   k_uptime_get()               k_uptime_get() (the TIMER in ms)
 *   irq_lock() (atomic_or())     save_and_disable_interrupts(),
 *                                restore_interrupts(): PRIMASK, as Zephyr's
 *                                irq_lock() on the Cortex-M0+; nothing inside
 *                                calls FreeRTOS (see zephyr_os_spinlock.h)
 *   struct i2c_msg               Zephyr's struct i2c_msg of the layer
 *                                (<zephyr/drivers/i2c.h>): its buf, len and
 *                                flags; i2c_transfer() has put STOP on the
 *                                last message, as z_impl_i2c_transfer()
 *   the devices                  the layer's struct device i2c0 and i2c1
 *                                (i2c_dw_instance.h); nothing in the
 *                                controller's data (I2C_DW_OS_DEV_HEAD empty)
 *   the nodes i2c0, i2c1         CONFIG_I2C_DW_I2C0 (always) and
 *                                CONFIG_I2C_DW_I2C1 (the CMake option
 *                                I2C_DW_I2C1), defined for every source of
 *                                the build by i2c_dw.cmake
 *   CONFIG_I2C_DW_CLOCK_SPEED    the I2C block's clock, clk_sys
 *                                (SYS_CLK_HZ: 125 MHz, the Pico SDK's
 *                                default for the RP2040, which its start-up
 *                                code sets; the Zephyr build's 125)
 *   CONFIG_I2C_DW_RP2040_DMA=y   the same symbol, defined below with
 *                                CONFIG_I2C_DW_I2C0 (the DMA reads of i2c0)
 */

#ifndef FREERTOS_DRIVERS_I2C_DW_I2C_DW_OS_IMPL_H
#define FREERTOS_DRIVERS_I2C_DW_I2C_DW_OS_IMPL_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "pico.h"
#include "hardware/sync.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

#if defined(CONFIG_I2C_DW_I2C0) && !defined(CONFIG_I2C_DW_RP2040_DMA)
#define CONFIG_I2C_DW_RP2040_DMA 1
#endif

#if SYS_CLK_HZ % 1000000 != 0
#error "clk_sys is not a whole number of MHz"
#endif
#define I2C_DW_CLOCK_SPEED (SYS_CLK_HZ / 1000000)

typedef struct k_sem i2c_dw_os_sem_t;
typedef TickType_t i2c_dw_os_timepoint_t;
typedef uint32_t i2c_dw_os_irqstate_t;

typedef struct i2c_msg i2c_dw_os_msg_t;

/* The flags are the message's own (the NuttX build makes Zephyr's flags
 * from its own and the neighbours' and needs the other arguments).
 */
#define I2C_DW_OS_MSG_BUF(m) ((m)->buf)
#define I2C_DW_OS_MSG_LEN(m) ((m)->len)
#define I2C_DW_OS_MSG_FLAGS(msgs, cur, i, left)                                                    \
	((void)(msgs), (void)(i), (void)(left), (uint8_t)(cur)->flags)

#define I2C_DW_OS_DEV_HEAD

static inline uint32_t i2c_dw_os_irq_lock(void)
{
	return save_and_disable_interrupts();
}

static inline void i2c_dw_os_irq_unlock(uint32_t state)
{
	restore_interrupts(state);
}

static inline void i2c_dw_os_sem_give(struct k_sem *sem)
{
	k_sem_give(sem);
}

static inline int i2c_dw_os_sem_take_ms(struct k_sem *sem, uint32_t ms)
{
	return k_sem_take(sem, K_MSEC(ms));
}

static inline TickType_t i2c_dw_os_timepoint_calc(uint32_t ms)
{
	int64_t dt = K_MSEC(ms).ticks;

	return xTaskGetTickCount() + (TickType_t)MAX(1, dt);
}

static inline int i2c_dw_os_sem_take_until(struct k_sem *sem, TickType_t end)
{
	int32_t left = (int32_t)(end - xTaskGetTickCount());

	if (left < 0) {
		left = 0;
	}
	return k_sem_take(sem, K_TICKS(left));
}

static inline void i2c_dw_os_sem_reset(struct k_sem *sem)
{
	k_sem_reset(sem);
}

static inline int i2c_dw_os_bus_lock(struct k_sem *sem)
{
	return k_sem_take(sem, K_FOREVER);
}

static inline void i2c_dw_os_bus_unlock(struct k_sem *sem)
{
	k_sem_give(sem);
}

static inline void i2c_dw_os_msleep(uint32_t ms)
{
	(void)k_msleep(ms > INT32_MAX ? INT32_MAX : (int32_t)ms);
}

#define i2c_dw_os_uptime_ms() k_uptime_get()

#endif /* FREERTOS_DRIVERS_I2C_DW_I2C_DW_OS_IMPL_H */
