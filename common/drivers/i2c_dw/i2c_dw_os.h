/* SPDX-License-Identifier: MIT */
/* What the I2C0 and DMA core (nano_i2c.c and nano_dma.c here) needs from
 * the OS.  Both files include this header before anything else; it
 * includes the OS's part, "i2c_dw_os_impl.h", then declares the inline
 * functions that part must define, so a missing or different one does not
 * compile.
 *
 * The core is the register work of the Zephyr build's I2C0 driver
 * (zephyr/app/usb-console/drivers/i2c_dw/i2c_dw.c) and of its DMA driver
 * (Zephyr's dma_rpi_pico.c): the transfer's state machine, the interrupt
 * handlers' bodies, the DMA reads, the DMA channels and the i2c_dma
 * counters (nano_i2c_dma.h).  The OS's part gives it semaphores, a clock,
 * sleeps and a critical section, connects the interrupts, resets the
 * blocks, sets the pins, and makes the I2C0 device of the OS's driver
 * model.  The boundary is made of static inline functions and macros, and
 * the OS's interrupt handlers and transfer entry call the core's static
 * inline functions, so that the compiler makes one function of each, as
 * if the core called the OS directly (the interrupt and transfer paths run
 * for every FIFO read at 6667 Hz).
 *
 * Each OS's build puts these files on the include path of nano_i2c.c and
 * nano_dma.c (the NuttX build's are in nuttx/boards/arduino-nano-rp2040-
 * connect/src; nuttx/drivers/README.md, "I2C0"):
 *
 *   i2c_dw_os_impl.h     the types, macros and inline functions below, and
 *                        CONFIG_NANO_I2C0_DMA defined to build the DMA
 *                        reads (the Zephyr build's CONFIG_I2C_DW_RP2040_DMA,
 *                        on there; nano_dma.c is then built too)
 *   app_log.h            LOG_MODULE_REGISTER(name), LOG_ERR() and
 *                        LOG_ERR_RATELIMIT() as Zephyr's (the rate limit:
 *                        LOG_RATELIMIT_INTERVAL_MS), at the level of
 *                        LOG_LEVEL, which the core defines (3) before it
 *                        includes the file
 *   nano_i2c_instance.h  read at the end of nano_i2c.c (below)
 *   nano_dma_instance.h  read at the end of nano_dma.c (below)
 *
 * i2c_dw_os_impl.h: types and macros
 *
 *   i2c_dw_os_sem_t        a counting semaphore (Zephyr's k_sem with
 *                          K_SEM_MAX_LIMIT; bus_sem is used as a binary
 *                          one, without priority inheritance, as in the
 *                          Zephyr build)
 *   i2c_dw_os_timepoint_t  an absolute time for i2c_dw_os_sem_take_until()
 *                          (Zephyr's k_timepoint_t)
 *   i2c_dw_os_irqstate_t   what i2c_dw_os_irq_lock() returns
 *   i2c_dw_os_msg_t        one message of a transfer in the OS's form, read
 *                          by the core only through these macros:
 *     I2C_DW_OS_MSG_BUF(m)  its buffer (uint8_t *; NULL for an empty
 *                           message without one)
 *     I2C_DW_OS_MSG_LEN(m)  its length (uint32_t)
 *     I2C_DW_OS_MSG_FLAGS(msgs, cur, i, left)
 *                           Zephyr's flags (a uint8_t of I2C_MSG_READ,
 *                           I2C_MSG_STOP, I2C_MSG_RESTART, nano_i2c_dw.h)
 *                           of message cur = msgs + i, left being the
 *                           number of messages from cur to the end.  For
 *                           Zephyr's own messages these are their flags,
 *                           with I2C_MSG_STOP on the last one (as
 *                           z_impl_i2c_transfer() adds it)
 *   I2C_DW_OS_DEV_HEAD     members put first in struct i2c_dw_dev_config
 *                          (the OS's device object if it has to be first,
 *                          as NuttX's struct i2c_master_s; may be empty)
 *   I2C_DW_CLOCK_SPEED     the clock of the I2C block in MHz: 125, the
 *                          Zephyr build's CONFIG_I2C_DW_CLOCK_SPEED
 *
 * i2c_dw_os_impl.h: functions (declared below; "interrupt" marks the
 * ones that the interrupt handlers call, the others run in the thread that
 * makes the transfer), and i2c_dw_os_uptime_ms(), which may be a macro.
 *
 * nano_i2c_instance.h (end of nano_i2c.c) defines:
 *
 *   static int i2c_dw_os_prepare(struct i2c_dw_dev_config *dw)
 *       reset_line_toggle_dt() and pinctrl_apply_state(): reset I2C0 (set
 *       its RESETS bit, clear it, wait for RESET_DONE), then GPIO12 (SDA)
 *       and GPIO13 (SCL) as Zephyr's pinctrl_rpi_pico.c sets the board's
 *       i2c0_default: function I2C, no pulls, 4 mA, slow slew, Schmitt
 *       trigger and input enabled.  0, or a negative errno
 *   static void i2c_dw_os_irq_enable(struct i2c_dw_dev_config *dw)
 *       IRQ_CONNECT() and irq_enable(): I2C0_IRQ on a handler that calls
 *       i2c_dw_isr_handler(dw), then enabled
 *
 *   and the OS's device: it sets dw->base of i2c_0_runtime (0x40044000),
 *   initializes device_sync_sem and rp_dma_sem to 0 and bus_sem to 1,
 *   calls i2c_dw_turn_on(dw) once at start-up (after nano_dma_initialize(),
 *   as Zephyr initializes the DMA device first) and sets dw->ready if it
 *   returned 0.  Its transfer entry refuses what the Zephyr API cannot
 *   express (no message array, more than 255 messages, a 10-bit address,
 *   a missing buffer, a device that is not ready), returns 0 for no
 *   message, and otherwise returns i2c_dw_transfer_msgs(dw, msgs, count,
 *   target_address, speed) with the speed class of the transfer
 *   (NANO_I2C_SPEED_STANDARD, _FAST or _FAST_PLUS of nano_i2c_dw.h: the OS's
 *   part keeps what i2c_configure() set, or takes it from the messages).
 *
 * nano_dma_instance.h (end of nano_dma.c) defines:
 *
 *   static void nano_dma_os_reset(void)
 *       reset the DMA block (set its RESETS bit, clear it, wait for
 *       RESET_DONE)
 *   static void nano_dma_os_irq_enable(void)
 *       DMA_IRQ_0 and DMA_IRQ_1 on one handler that calls
 *       dma_rpi_pico_isr_handler(), then both enabled
 *
 *   The OS's start-up calls nano_dma_initialize() before
 *   i2c_dw_turn_on().  No other user may own the DMA's interrupts or
 *   channels (the channel allocation of nano_dma.c knows no other users).
 *
 * The I2C0 and DMA interrupts run at the same priority (as in the Zephyr
 * and NuttX builds), and the interrupt handlers and the transfers run on
 * one core.
 */

#ifndef __COMMON_DRIVERS_I2C_DW_I2C_DW_OS_H
#define __COMMON_DRIVERS_I2C_DW_I2C_DW_OS_H

#include "i2c_dw_os_impl.h"

#include <stdint.h>

/* Critical section against the interrupts of this core (Zephyr's
 * irq_lock()/atomic_*() on the single-core RP2040): disables interrupts
 * and returns the previous state; i2c_dw_os_irq_unlock() restores it.
 * Thread and interrupt.  Nests.
 */

static inline i2c_dw_os_irqstate_t i2c_dw_os_irq_lock(void);
static inline void i2c_dw_os_irq_unlock(i2c_dw_os_irqstate_t state);

/* k_sem_give().  Interrupt and thread. */

static inline void i2c_dw_os_sem_give(i2c_dw_os_sem_t *sem);

/* k_sem_take(sem, K_MSEC(ms)): 0 once taken, nonzero (a negative errno)
 * when ms milliseconds (rounded up to the OS's ticks, as Zephyr's K_MSEC())
 * pass first.  Not ended by signals.
 */

static inline int i2c_dw_os_sem_take_ms(i2c_dw_os_sem_t *sem, uint32_t ms);

/* sys_timepoint_calc(K_MSEC(ms)): now plus ms (rounded up as above). */

static inline i2c_dw_os_timepoint_t i2c_dw_os_timepoint_calc(uint32_t ms);

/* k_sem_take(sem, sys_timepoint_timeout(end)): waits until end; once end
 * has passed, it still takes the semaphore without waiting if it can.
 * 0 once taken, nonzero otherwise.  Not ended by signals.
 */

static inline int i2c_dw_os_sem_take_until(i2c_dw_os_sem_t *sem,
					   i2c_dw_os_timepoint_t end);

/* k_sem_reset(): the count becomes 0. */

static inline void i2c_dw_os_sem_reset(i2c_dw_os_sem_t *sem);

/* k_sem_take(&bus_sem, K_FOREVER) and k_sem_give(&bus_sem): the
 * transfers' lock.  The lock returns 0, or a negative errno if it could
 * not wait (the transfer then returns that errno).
 */

static inline int i2c_dw_os_bus_lock(i2c_dw_os_sem_t *sem);
static inline void i2c_dw_os_bus_unlock(i2c_dw_os_sem_t *sem);

/* k_msleep(ms): sleep at least ms milliseconds. */

static inline void i2c_dw_os_msleep(uint32_t ms);

/* k_uptime_get(): int64_t i2c_dw_os_uptime_ms(void), the milliseconds
 * since boot.  Not declared here, as it may be a macro.  (The NuttX
 * build's is one: as an inline function it changes the order in which
 * the compiler inlines the core's functions, and with it the order of the
 * functions in the image.)
 */

#endif /* __COMMON_DRIVERS_I2C_DW_I2C_DW_OS_H */
