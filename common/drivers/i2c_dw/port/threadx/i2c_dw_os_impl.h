/* SPDX-License-Identifier: MIT */
/* The ThreadX part of the I2C core's boundary
 * (common/drivers/i2c_dw/port/i2c_dw_os.h, which includes this file first in
 * i2c_dw.c): the types and calls behind the boundary's names, on the
 * ThreadX kernel and the Pico SDK:
 *
 *   Zephyr                       here
 *   k_sem (no priority inh.)     a ThreadX counting semaphore (TX_SEMAPHORE,
 *                                in the structure's own memory) and its
 *                                limit: no object of the compatibility
 *                                layer, no task notification as on
 *                                FreeRTOS: ThreadX's semaphore is its
 *                                lightest object a thread waits on (a count
 *                                and a list of the waiting threads), and an
 *                                interrupt handler may put it.  Made by
 *                                i2c_dw_os_sem_init() at the start of the
 *                                instance (i2c_dw_instance.h):
 *                                device_sync_sem and rp2040_dma_sem from 0
 *                                with Zephyr's K_SEM_MAX_LIMIT (one thread
 *                                waits, the interrupt handler gives), bus_sem
 *                                from 1 with the limit 1 (the transfers'
 *                                lock: no priority inheritance, as Zephyr's
 *                                k_sem; the waiter of the highest priority
 *                                first, then first come, as Zephyr's wait
 *                                queue: ThreadX queues the waiters first
 *                                come, and the unlock moves the most urgent
 *                                to the front, tx_semaphore_prioritize())
 *   k_sem_give()                 tx_semaphore_ceiling_put() with the limit:
 *                                Zephyr's rule, as ThreadX hands the count to
 *                                the first waiter at once (it returns
 *                                TX_SUCCESS), or the count goes up, not past
 *                                the limit (TX_CEILING_EXCEEDED, ignored).
 *                                From an interrupt handler too: ThreadX's
 *                                Cortex-M0 port pends PendSV when the woken
 *                                thread is more urgent, and the switch
 *                                comes at the end of the handler
 *   k_sem_take(.., K_MSEC(ms))   tx_semaphore_get() for ms rounded up to
 *                                ticks (k_ms_to_ticks_ceil64()) plus one
 *                                tick: Zephyr wakes a relative timeout at the
 *                                current tick + ticks + 1, ThreadX a wait of
 *                                n at the current tick + n.  With the board's
 *                                1 ms tick: the transfer's 1000 ms 1001
 *                                ticks, the abort's 1 ms 2; 0 does not
 *                                block.  -EAGAIN once the time is up (-EBUSY
 *                                without a wait), as Zephyr's
 *   k_sem_take(.., K_FOREVER)    the same with TX_WAIT_FOREVER
 *   k_sem_reset()                the count becomes 0 (ThreadX has no such
 *                                service: the semaphore's count member is
 *                                written with interrupts masked, as
 *                                ThreadX's own services change it).  Only
 *                                the thread that waits on the semaphore
 *                                resets it, so no thread waits then
 *                                (Zephyr's would end such waits with
 *                                -EAGAIN)
 *   k_timepoint_t                ThreadX's tick count (tx_time_get(), 32
 *                                bits; the differences are taken as signed,
 *                                so the 1 s deadline is far from the wrap)
 *   sys_timepoint_calc(K_MSEC)   now + the ticks of ms (at least one, as
 *                                Zephyr's MAX(1, dt))
 *   k_sem_take(.., sys_timepoint_timeout(end))
 *                                a wait for the ticks to end plus one, 0
 *                                once end has passed (one try)
 *   k_msleep()                   tx_thread_sleep() for the ticks plus one, as
 *                                the waits; 0 ticks yields
 *                                (tx_thread_relinquish())
 *   k_uptime_get()               the RP2040 TIMER (time_us_64()) in ms,
 *                                rounded down: not the tick, which stands
 *                                still while interrupts are masked for
 *                                longer than a tick (a flash erase) until the
 *                                application's tick catches up
 *   irq_lock() (atomic_or())     save_and_disable_interrupts(),
 *                                restore_interrupts(): PRIMASK, as Zephyr's
 *                                irq_lock() on the Cortex-M0+
 *   struct i2c_msg               Zephyr's struct i2c_msg of the
 *                                compatibility layer (<zephyr/drivers/i2c.h>,
 *                                the API the driver is used with): its buf,
 *                                len and flags; i2c_transfer() has put STOP
 *                                on the last message, as
 *                                z_impl_i2c_transfer()
 *   the devices                  the layer's struct device i2c0 and i2c1
 *                                (i2c_dw_instance.h); nothing in the
 *                                controller's data (I2C_DW_OS_DEV_HEAD empty)
 *   the nodes i2c0, i2c1         CONFIG_I2C_DW_I2C0 (always) and
 *                                CONFIG_I2C_DW_I2C1, defined for every source
 *                                of the build by the build's definitions
 *   CONFIG_I2C_DW_CLOCK_SPEED    the I2C block's clock, clk_sys
 *                                (SYS_CLK_HZ: 125 MHz, the Pico SDK's
 *                                default for the RP2040, which its start-up
 *                                code sets; the Zephyr build's 125)
 *   CONFIG_I2C_DW_RP2040_DMA=y   the same symbol, defined below with
 *                                CONFIG_I2C_DW_I2C0 (the DMA reads of i2c0)
 *
 * The semaphores' waits.  ThreadX does the whole of a give and a take in its
 * own critical sections (interrupts masked), so nothing here masks
 * interrupts around them; a give between the waiter's look at the count and
 * its suspension is not lost (ThreadX looks at the count and suspends the
 * thread in one critical section).  A wait that ends by its time has left
 * the list in ThreadX's timeout; a give after that counts up.  The waits
 * are those of a thread: ThreadX refuses a wait before the kernel runs or in
 * an interrupt handler (TX_WAIT_ERROR, with its error checking on), and
 * there a take has the count if there is one (bus_lock() of a free bus
 * before the kernel runs) and returns -EAGAIN otherwise.
 *
 * The semaphores are ThreadX objects of their own, apart from the
 * compatibility layer's wake flag of each thread (zephyr_kernel_os_impl.h
 * of threadx/drivers/compat), so that a wake-up of the one never ends a wait
 * of the other, as the FreeRTOS build keeps them apart on two task
 * notification indices (1 the layer's, 3 the I2C's).  They are made once
 * and never zeroed, so they stay in ThreadX's list of created semaphores.
 *
 * The interrupt handlers that give the semaphores run at the Pico SDK's
 * default priority for the peripherals (0x80).  ThreadX's Cortex-M0 port
 * masks all interrupts (PRIMASK) in its critical sections, so a service
 * may be called from an interrupt of any priority.
 */

#ifndef I2C_DW_OS_IMPL_H_
#define I2C_DW_OS_IMPL_H_

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "tx_api.h"

#include "pico.h"
#include "hardware/sync.h"
#include "hardware/timer.h"

#include <zephyr/drivers/i2c.h>

#if defined(CONFIG_I2C_DW_I2C0) && !defined(CONFIG_I2C_DW_RP2040_DMA)
#define CONFIG_I2C_DW_RP2040_DMA 1
#endif

#if SYS_CLK_HZ % 1000000 != 0
#error "clk_sys is not a whole number of MHz"
#endif
#define I2C_DW_CLOCK_SPEED (SYS_CLK_HZ / 1000000)

#if TX_TIMER_TICKS_PER_SECOND != 1000
#error "the I2C driver's waits are written for ThreadX's 1 ms tick (tx_user.h)"
#endif

/* The semaphore and its limit (0 until i2c_dw_os_sem_init()). */
typedef struct {
	TX_SEMAPHORE sem;
	ULONG limit;
} i2c_dw_os_sem_t;

typedef ULONG i2c_dw_os_timepoint_t;
typedef uint32_t i2c_dw_os_irqstate_t;

typedef struct i2c_msg i2c_dw_os_msg_t;

/* Zephyr's K_SEM_MAX_LIMIT. */
#define I2C_DW_OS_SEM_MAX_LIMIT UINT_MAX

/* The flags are the message's own (the NuttX build makes Zephyr's flags
 * from its own and the neighbours' and needs the other arguments).
 */
#define I2C_DW_OS_MSG_BUF(m) ((m)->buf)
#define I2C_DW_OS_MSG_LEN(m) ((m)->len)
#define I2C_DW_OS_MSG_FLAGS(msgs, cur, i, left)                                                    \
	((void)(msgs), (void)(i), (void)(left), (uint8_t)(cur)->flags)

#define I2C_DW_OS_DEV_HEAD

/* k_sem_init(sem, initial, limit).  Not part of the boundary: the start of
 * the instance calls it (i2c_dw_instance.h), before anything waits on or
 * gives the semaphore, from a thread or before the kernel runs.  A second
 * start of the instance (its controller did not start the first time)
 * keeps the ThreadX object made by the first (ThreadX refuses to create
 * one twice, and to delete one before the kernel runs) and sets its count
 * and limit anew, with interrupts masked as ThreadX's own services change
 * the count; nothing waits on it then.
 */
static inline void i2c_dw_os_sem_init(i2c_dw_os_sem_t *sem, unsigned int initial,
				      unsigned int limit)
{
	if (sem->limit != 0) {
		uint32_t state = save_and_disable_interrupts();

		sem->sem.tx_semaphore_count = initial;
		sem->limit = limit;
		restore_interrupts(state);
		return;
	}
	sem->limit = limit;
	(void)tx_semaphore_create(&sem->sem, (CHAR *)"i2c_dw", initial);
}

/* MS in ticks, rounded up (Zephyr's K_MSEC(): k_ms_to_ticks_ceil64()). */
static inline uint64_t i2c_dw_os_ms_to_ticks(uint32_t ms)
{
	return ((uint64_t)ms * TX_TIMER_TICKS_PER_SECOND + 999u) / 1000u;
}

/* The wait option of a wait of TICKS: 0 does not block, else ticks + 1, at
 * most TX_WAIT_FOREVER - 1 (TX_WAIT_FOREVER would be for ever).
 */
static inline ULONG i2c_dw_os_block_ticks(uint64_t ticks)
{
	if (ticks == 0) {
		return TX_NO_WAIT;
	}
	if (ticks >= (uint64_t)(TX_WAIT_FOREVER - 2)) {
		return TX_WAIT_FOREVER - 1;
	}
	return (ULONG)ticks + 1;
}

/* k_sem_take() for the wait option TICKS (TX_WAIT_FOREVER: for ever).
 * ThreadX refuses a wait where no thread can block (TX_WAIT_ERROR: before
 * the kernel runs, in an interrupt handler) even when the count is there:
 * then one try without a wait.
 */
static inline int i2c_dw_os_sem_wait(i2c_dw_os_sem_t *sem, ULONG ticks)
{
	UINT ret = tx_semaphore_get(&sem->sem, ticks);

	if (ret == TX_WAIT_ERROR) {
		ret = tx_semaphore_get(&sem->sem, TX_NO_WAIT);
	}
	if (ret == TX_SUCCESS) {
		return 0;
	}
	return ticks == TX_NO_WAIT ? -EBUSY : -EAGAIN;
}

static inline uint32_t i2c_dw_os_irq_lock(void)
{
	return save_and_disable_interrupts();
}

static inline void i2c_dw_os_irq_unlock(uint32_t state)
{
	restore_interrupts(state);
}

static inline void i2c_dw_os_sem_give(i2c_dw_os_sem_t *sem)
{
	(void)tx_semaphore_ceiling_put(&sem->sem, sem->limit);
}

static inline int i2c_dw_os_sem_take_ms(i2c_dw_os_sem_t *sem, uint32_t ms)
{
	return i2c_dw_os_sem_wait(sem, i2c_dw_os_block_ticks(i2c_dw_os_ms_to_ticks(ms)));
}

static inline ULONG i2c_dw_os_timepoint_calc(uint32_t ms)
{
	uint64_t dt = i2c_dw_os_ms_to_ticks(ms);

	return tx_time_get() + (ULONG)(dt > 1 ? dt : 1);
}

static inline int i2c_dw_os_sem_take_until(i2c_dw_os_sem_t *sem, ULONG end)
{
	int32_t left = (int32_t)(end - tx_time_get());

	return i2c_dw_os_sem_wait(sem, i2c_dw_os_block_ticks(left > 0 ? (uint64_t)left : 0));
}

static inline void i2c_dw_os_sem_reset(i2c_dw_os_sem_t *sem)
{
	uint32_t state = save_and_disable_interrupts();

	sem->sem.tx_semaphore_count = 0;
	restore_interrupts(state);
}

/* The lock: the count at once when it is free (also before the kernel
 * runs, see i2c_dw_os_sem_wait()); otherwise a wait in ThreadX's list.
 */
static inline int i2c_dw_os_bus_lock(i2c_dw_os_sem_t *sem)
{
	return i2c_dw_os_sem_wait(sem, TX_WAIT_FOREVER);
}

/* With two or more waiters, the most urgent (the first of them among
 * equals) goes to the front before the put hands it the count.  A thread
 * that joins the list after the look at the count (an interrupt readied it
 * and it ran first) is behind the one the put hands the count to, as if it
 * had come after the give; only when no waiter was there to look at does
 * the put take the first that came.
 */
static inline void i2c_dw_os_bus_unlock(i2c_dw_os_sem_t *sem)
{
	if (sem->sem.tx_semaphore_suspended_count > 1) {
		(void)tx_semaphore_prioritize(&sem->sem);
	}
	i2c_dw_os_sem_give(sem);
}

static inline void i2c_dw_os_msleep(uint32_t ms)
{
	uint64_t ticks = i2c_dw_os_ms_to_ticks(ms > INT32_MAX ? INT32_MAX : ms);

	if (ticks == 0) {
		/* Behind the other ready threads of the priority, as Zephyr's yield. */
		tx_thread_relinquish();
		return;
	}
	(void)tx_thread_sleep(i2c_dw_os_block_ticks(ticks));
}

#define i2c_dw_os_uptime_ms() ((int64_t)(time_us_64() / 1000u))

#endif /* I2C_DW_OS_IMPL_H_ */
