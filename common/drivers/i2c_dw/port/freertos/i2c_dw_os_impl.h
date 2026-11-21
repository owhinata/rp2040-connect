/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the I2C core's boundary
 * (common/drivers/i2c_dw/port/i2c_dw_os.h, which includes this file first in
 * i2c_dw.c): the types and calls behind the boundary's names, on FreeRTOS
 * and the Pico SDK:
 *
 *   Zephyr                       here
 *   k_sem (no priority inh.)     a count with its limit and a list of the
 *                                waiting tasks (each on its own stack), in
 *                                the structure's own memory, the tasks woken
 *                                through their task notification
 *                                I2C_DW_OS_NOTIFY_INDEX (no FreeRTOS queue:
 *                                these waits run for every FIFO read).  Made
 *                                by i2c_dw_os_sem_init() at the start of the
 *                                instance (i2c_dw_instance.h):
 *                                device_sync_sem and rp2040_dma_sem from 0
 *                                with Zephyr's K_SEM_MAX_LIMIT (one task
 *                                waits, the interrupt handler gives), bus_sem
 *                                from 1 with the limit 1 (the transfers'
 *                                lock: no priority inheritance, as Zephyr's
 *                                k_sem; its waiters by priority, then first
 *                                come, as Zephyr's wait queue)
 *   k_sem_give()                 Zephyr's rule: a waiter takes the count at
 *                                once (it is marked done and notified), or
 *                                the count goes up, not past the limit.  The
 *                                notification is sent after the critical
 *                                section: vTaskNotifyGiveIndexedFromISR() in
 *                                an interrupt handler (the Pico SDK's
 *                                __get_current_exception(), IPSR) with the
 *                                switch to the woken task at the end of the
 *                                handler (portYIELD_FROM_ISR(): PendSV),
 *                                xTaskNotifyGiveIndexed() in a task
 *   k_sem_take(.., K_MSEC(ms))   the count if there is one; otherwise the
 *                                task joins the list and waits on its
 *                                notification for ms rounded up to ticks
 *                                (k_ms_to_ticks_ceil64()) plus one tick:
 *                                Zephyr wakes a relative timeout at the
 *                                current tick + ticks + 1, FreeRTOS a block
 *                                time of n at the current tick + n.  With
 *                                the board's 1 ms tick: the transfer's
 *                                1000 ms 1001 ticks, the abort's 1 ms 2; 0
 *                                does not block.  -EAGAIN once the time is
 *                                up (-EBUSY without a wait), as Zephyr's
 *   k_sem_take(.., K_FOREVER)    the same without an end (cannot fail)
 *   k_sem_reset()                the count becomes 0.  Only the task that
 *                                waits on the semaphore resets it, so no
 *                                task waits then (Zephyr's would end such
 *                                waits with -EAGAIN)
 *   k_timepoint_t                the FreeRTOS tick count (32 bits; the
 *                                differences are taken as signed, so the
 *                                1 s deadline is far from the wrap)
 *   sys_timepoint_calc(K_MSEC)   now + the ticks of ms (at least one, as
 *                                Zephyr's MAX(1, dt))
 *   k_sem_take(.., sys_timepoint_timeout(end))
 *                                a wait for the ticks to end plus one, 0
 *                                once end has passed (one try)
 *   k_msleep()                   vTaskDelay() for the ticks plus one, as the
 *                                waits; 0 ticks yields (taskYIELD())
 *   k_uptime_get()               the RP2040 TIMER (time_us_64()) in ms,
 *                                rounded down: not the tick, which stands
 *                                still while interrupts are masked for
 *                                longer than a tick (a flash erase) until the
 *                                application's tick hook catches up
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
 *                                CONFIG_I2C_DW_I2C1 (the CMake option
 *                                I2C_DW_I2C1), defined for every source of
 *                                the build by i2c_dw.cmake
 *   CONFIG_I2C_DW_CLOCK_SPEED    the I2C block's clock, clk_sys
 *                                (SYS_CLK_HZ: 125 MHz, the Pico SDK's
 *                                default for the RP2040, which its start-up
 *                                code sets; the Zephyr build's 125)
 *   CONFIG_I2C_DW_RP2040_DMA=y   the same symbol, defined below with
 *                                CONFIG_I2C_DW_I2C0 (the DMA reads of i2c0)
 *
 * The semaphores' waits.  A waiting task's entry is on its stack; the count,
 * the list and the entries' done flags change only in the kernel's critical
 * section (taskENTER_CRITICAL(), _FROM_ISR() in an interrupt handler: PRIMASK
 * on the RP2040), and no FreeRTOS call is made inside it.  A give marks the
 * first waiter done and takes it off the list in the critical section, and
 * notifies its task after it: a give between the waiter's look at the count
 * and its wait is not lost (the waiter is already on the list, and the
 * notification is pending when it waits).  A wake-up does not decide the
 * result: the waiter looks at its done flag, and if it is not set (a
 * notification left from an earlier wait whose give came as it timed out, or
 * from the giver of a done wait that was preempted before it notified) it
 * waits again for the rest of its time.  When its time is up it leaves the
 * list in the critical section; a give that marked it done before that is
 * its count (0).
 *
 * The notification index is the driver's own (I2C_DW_OS_NOTIFY_INDEX, 3):
 * index 0 is the Pico SDK's and the application's, 1 the compatibility
 * layer's waits, 2 the NINA's socket event waits (FreeRTOSConfig.h).  An
 * index of its own keeps other users' notifications from waking these waits
 * at all (they would only cost a second wait).  It adds a notification
 * value and state (5 bytes) to each task's control block; with this build's
 * configuration they fit in the block's padding (the static tasks' buffers
 * keep their sizes).
 *
 * The interrupt handlers that give the semaphores run at the Pico SDK's
 * default priority for the peripherals (0x80).  The RP2040 port of FreeRTOS
 * (Cortex-M0+, no BASEPRI) masks all interrupts (PRIMASK) in its critical
 * sections, so a FromISR call is safe from an interrupt of any priority.
 */

#ifndef I2C_DW_OS_IMPL_H_
#define I2C_DW_OS_IMPL_H_

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

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

/* The task notification of the semaphores' waits (see above). */
#define I2C_DW_OS_NOTIFY_INDEX 3

#if configTASK_NOTIFICATION_ARRAY_ENTRIES <= I2C_DW_OS_NOTIFY_INDEX
#error "the I2C driver's semaphores wait on task notification index 3"
#endif

/* A task waiting on a semaphore, on its stack. */
struct i2c_dw_os_waiter {
	struct i2c_dw_os_waiter *next;
	TaskHandle_t task;
	UBaseType_t prio;
	volatile bool done; /* the count is the waiter's (set by a give) */
};

typedef struct {
	volatile uint32_t count;
	uint32_t limit;
	struct i2c_dw_os_waiter *waiters;
} i2c_dw_os_sem_t;

typedef TickType_t i2c_dw_os_timepoint_t;
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
 * gives the semaphore.
 */
static inline void i2c_dw_os_sem_init(i2c_dw_os_sem_t *sem, unsigned int initial,
				      unsigned int limit)
{
	sem->count = initial;
	sem->limit = limit;
	sem->waiters = NULL;
}

/* MS in ticks, rounded up (Zephyr's K_MSEC(): k_ms_to_ticks_ceil64()). */
static inline uint64_t i2c_dw_os_ms_to_ticks(uint32_t ms)
{
	return ((uint64_t)ms * configTICK_RATE_HZ + 999u) / 1000u;
}

/* The block time of a wait of TICKS: 0 does not block, else ticks + 1, at
 * most portMAX_DELAY - 1 (portMAX_DELAY would be for ever).
 */
static inline TickType_t i2c_dw_os_block_ticks(uint64_t ticks)
{
	if (ticks == 0) {
		return 0;
	}
	if (ticks >= (uint64_t)(portMAX_DELAY - 2)) {
		return portMAX_DELAY - 1;
	}
	return (TickType_t)ticks + 1;
}

/* The critical section of the semaphores (see above). */
static inline UBaseType_t i2c_dw_os_sem_lock(void)
{
	if (__get_current_exception() != 0) {
		return taskENTER_CRITICAL_FROM_ISR();
	}
	taskENTER_CRITICAL();
	return 0;
}

static inline void i2c_dw_os_sem_unlock(UBaseType_t key)
{
	if (__get_current_exception() != 0) {
		taskEXIT_CRITICAL_FROM_ISR(key);
	} else {
		taskEXIT_CRITICAL();
	}
}

/* Takes the count if there is one (in the critical section). */
static inline bool i2c_dw_os_sem_try(i2c_dw_os_sem_t *sem)
{
	UBaseType_t key = i2c_dw_os_sem_lock();
	bool taken = sem->count > 0;

	if (taken) {
		sem->count--;
	}
	i2c_dw_os_sem_unlock(key);
	return taken;
}

/* k_sem_take() for a block time of TICKS (portMAX_DELAY: for ever), the
 * waiter at priority PRIO in the list (higher first, then first come).
 * From a task.
 */
static inline int i2c_dw_os_sem_wait(i2c_dw_os_sem_t *sem, TickType_t ticks, UBaseType_t prio)
{
	struct i2c_dw_os_waiter waiter = {
		.next = NULL,
		.task = xTaskGetCurrentTaskHandle(),
		.prio = prio,
		.done = false,
	};
	struct i2c_dw_os_waiter **link;
	TickType_t start, left = ticks;
	UBaseType_t key;

	key = i2c_dw_os_sem_lock();
	if (sem->count > 0) {
		sem->count--;
		i2c_dw_os_sem_unlock(key);
		return 0;
	}
	if (ticks == 0) {
		i2c_dw_os_sem_unlock(key);
		return -EBUSY;
	}
	for (link = &sem->waiters; *link != NULL && (*link)->prio >= prio; link = &(*link)->next) {
	}
	waiter.next = *link;
	*link = &waiter;
	i2c_dw_os_sem_unlock(key);

	start = xTaskGetTickCount();
	for (;;) {
		TickType_t elapsed;

		(void)ulTaskNotifyTakeIndexed(I2C_DW_OS_NOTIFY_INDEX, pdTRUE, left);
		if (waiter.done) {
			return 0;
		}
		if (ticks == portMAX_DELAY) {
			continue;
		}
		elapsed = xTaskGetTickCount() - start;
		if (elapsed >= ticks) {
			break;
		}
		/* An early wake-up (see above): the rest of the time. */
		left = ticks - elapsed;
	}

	key = i2c_dw_os_sem_lock();
	if (!waiter.done) {
		for (link = &sem->waiters; *link != NULL; link = &(*link)->next) {
			if (*link == &waiter) {
				*link = waiter.next;
				break;
			}
		}
	}
	i2c_dw_os_sem_unlock(key);
	return waiter.done ? 0 : -EAGAIN;
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
	UBaseType_t key = i2c_dw_os_sem_lock();
	struct i2c_dw_os_waiter *waiter = sem->waiters;
	TaskHandle_t task = NULL;

	if (waiter != NULL) {
		sem->waiters = waiter->next;
		waiter->next = NULL;
		task = waiter->task;
		waiter->done = true;
	} else if (sem->count != sem->limit) {
		sem->count++;
	}
	i2c_dw_os_sem_unlock(key);

	/* The waiter's entry may be gone now (it may see done and return):
	 * only its task is used.
	 */
	if (task == NULL) {
		return;
	}
	if (__get_current_exception() != 0) {
		BaseType_t woken = pdFALSE;

		vTaskNotifyGiveIndexedFromISR(task, I2C_DW_OS_NOTIFY_INDEX, &woken);
		portYIELD_FROM_ISR(woken);
	} else {
		(void)xTaskNotifyGiveIndexed(task, I2C_DW_OS_NOTIFY_INDEX);
	}
}

static inline int i2c_dw_os_sem_take_ms(i2c_dw_os_sem_t *sem, uint32_t ms)
{
	return i2c_dw_os_sem_wait(sem, i2c_dw_os_block_ticks(i2c_dw_os_ms_to_ticks(ms)), 0);
}

static inline TickType_t i2c_dw_os_timepoint_calc(uint32_t ms)
{
	uint64_t dt = i2c_dw_os_ms_to_ticks(ms);

	return xTaskGetTickCount() + (TickType_t)(dt > 1 ? dt : 1);
}

static inline int i2c_dw_os_sem_take_until(i2c_dw_os_sem_t *sem, TickType_t end)
{
	int32_t left = (int32_t)(end - xTaskGetTickCount());

	return i2c_dw_os_sem_wait(sem, i2c_dw_os_block_ticks(left > 0 ? (uint64_t)left : 0), 0);
}

static inline void i2c_dw_os_sem_reset(i2c_dw_os_sem_t *sem)
{
	UBaseType_t key = i2c_dw_os_sem_lock();

	sem->count = 0;
	i2c_dw_os_sem_unlock(key);
}

/* The lock: the count at once when it is free (no priority to look up);
 * otherwise a wait in the list by the task's priority.
 */
static inline int i2c_dw_os_bus_lock(i2c_dw_os_sem_t *sem)
{
	if (i2c_dw_os_sem_try(sem)) {
		return 0;
	}
	return i2c_dw_os_sem_wait(sem, portMAX_DELAY, uxTaskPriorityGet(NULL));
}

static inline void i2c_dw_os_bus_unlock(i2c_dw_os_sem_t *sem)
{
	i2c_dw_os_sem_give(sem);
}

static inline void i2c_dw_os_msleep(uint32_t ms)
{
	uint64_t ticks = i2c_dw_os_ms_to_ticks(ms > INT32_MAX ? INT32_MAX : ms);

	if (ticks == 0) {
		/* Behind the other ready tasks of the priority, as Zephyr's yield. */
		taskYIELD();
		return;
	}
	vTaskDelay(i2c_dw_os_block_ticks(ticks));
}

#define i2c_dw_os_uptime_ms() ((int64_t)(time_us_64() / 1000u))

#endif /* I2C_DW_OS_IMPL_H_ */
