/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the I2C test (i2c_dw_test_os.h) on the FreeRTOS
 * POSIX port: the runner (main.c's tests) and the test's hardware are
 * tasks, the hardware's at APP_PRIO_SHELL + 4, which runs
 * i2c_dw_test_hw_step() after every tick.  An interrupt handler is a call
 * from a task counted in z_host_isr_depth, which the Pico SDK's
 * __get_current_exception() (IPSR) of the binding reads; the tick hook of
 * the board (tick.c) counts as one too.  The checks of the binding's own
 * parts (common/drivers/i2c_dw/port/freertos/i2c_dw_os_impl.h): its count
 * and list of waiters, a notification left on its index.
 */

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"

#include "pico_i2c_fake.h"

#include "i2c_dw_os_impl.h"
#include "i2c_dw_test_os.h"
#include "priorities.h"

#define RUNNER_STACK 16384
#define HW_STACK     16384

_Thread_local int z_host_isr_depth;

/* tick.c's vApplicationTickHook(), counted as an interrupt handler. */
void app_tick_hook(void);

void vApplicationTickHook(void)
{
	z_host_isr_depth++;
	app_tick_hook();
	z_host_isr_depth--;
}

uint __get_current_exception(void)
{
	return z_host_isr_depth > 0 ? 16u + 23u : 0u;
}

uint32_t i2c_dw_test_os_ticks(void)
{
	return xTaskGetTickCount();
}

void i2c_dw_test_os_delay(uint32_t ticks)
{
	vTaskDelay(ticks);
}

void i2c_dw_test_os_thread_start(void (*fn)(void *arg), void *arg, int above)
{
	assert(xTaskCreate(fn, "test", HW_STACK / sizeof(StackType_t), arg,
			   (UBaseType_t)(APP_PRIO_SHELL + above), NULL) == pdPASS);
}

void i2c_dw_test_os_thread_exit(void)
{
	vTaskDelete(NULL);
}

void i2c_dw_test_os_isr(void (*fn)(void *arg), void *arg)
{
	z_host_isr_depth++;
	fn(arg);
	z_host_isr_depth--;
}

bool i2c_dw_test_os_in_isr(void)
{
	return z_host_isr_depth > 0;
}

/* The port's tasks are threads of which one runs at a time: nothing to
 * mask.
 */
uint32_t i2c_dw_test_os_irq_disable(void)
{
	return 0;
}

void i2c_dw_test_os_irq_restore(uint32_t state)
{
	(void)state;
}

void i2c_dw_test_os_sched_lock(void)
{
	vTaskSuspendAll();
}

void i2c_dw_test_os_sched_unlock(void)
{
	(void)xTaskResumeAll();
}

uint32_t i2c_dw_test_os_sem_count(i2c_dw_os_sem_t *sem)
{
	return sem->count;
}

unsigned int i2c_dw_test_os_sem_waiters(i2c_dw_os_sem_t *sem)
{
	unsigned int n = 0;

	for (struct i2c_dw_os_waiter *w = sem->waiters; w != NULL; w = w->next) {
		n++;
	}
	return n;
}

/* The window between a wait's look at the count and its block: the wait
 * reads the tick count there (the only FreeRTOS call between the two), so
 * the wrapped xTaskGetTickCount() (-Wl,--wrap, host_tests.sh) gives the
 * armed semaphore once, as an interrupt handler would at that moment, while
 * the waiter is on its list.
 */
static i2c_dw_os_sem_t *volatile race_sem;

TickType_t __real_xTaskGetTickCount(void);
TickType_t __wrap_xTaskGetTickCount(void);

TickType_t __wrap_xTaskGetTickCount(void)
{
	i2c_dw_os_sem_t *sem = race_sem;

	if (sem != NULL && sem->waiters != NULL &&
	    sem->waiters->task == xTaskGetCurrentTaskHandle()) {
		race_sem = NULL;
		i2c_dw_test_give_from_isr(sem);
	}
	return __real_xTaskGetTickCount();
}

void i2c_dw_test_os_race(i2c_dw_os_sem_t *sem)
{
	race_sem = sem;
}

bool i2c_dw_test_os_race_pending(void)
{
	return race_sem != NULL;
}

static void giver_main(void *arg)
{
	vTaskDelay(20);
	i2c_dw_test_give_from_isr(arg);
	vTaskDelete(NULL);
}

/* A notification left on the binding's index (another wait's, or a stray
 * one) does not end a wait: it waits for the rest of its time, -EAGAIN, or
 * until a give.
 */
void i2c_dw_test_os_boundary(void)
{
	static i2c_dw_os_sem_t sem;
	TickType_t start, ticks;

	i2c_dw_os_sem_init(&sem, 0, I2C_DW_OS_SEM_MAX_LIMIT);
	(void)xTaskNotifyGiveIndexed(xTaskGetCurrentTaskHandle(), I2C_DW_OS_NOTIFY_INDEX);
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_ms(&sem, 10) == -EAGAIN);
	ticks = xTaskGetTickCount() - start;
	assert(ticks >= 11 && ticks <= 13);
	assert(sem.waiters == NULL);
	(void)xTaskNotifyGiveIndexed(xTaskGetCurrentTaskHandle(), I2C_DW_OS_NOTIFY_INDEX);
	assert(xTaskCreate(giver_main, "giver", HW_STACK / sizeof(StackType_t), &sem,
			   APP_PRIO_SHELL + 1, NULL) == pdPASS);
	start = xTaskGetTickCount();
	assert(i2c_dw_os_sem_take_ms(&sem, 1000) == 0);
	ticks = xTaskGetTickCount() - start;
	assert(ticks >= 19 && ticks <= 30);
	assert(sem.count == 0 && sem.waiters == NULL);
}

static void hw_main(void *arg)
{
	(void)arg;
	for (;;) {
		vTaskDelay(1);
		i2c_dw_test_hw_step();
	}
}

static void runner(void *arg)
{
	(void)arg;
	i2c_dw_test_run();
	_exit(0);
}

int main(void)
{
	/* printf() must not allocate in a task (the port can switch threads
	 * while the C library holds its locks).
	 */
	static char out[1 << 12];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, APP_PRIO_SHELL,
			NULL) != pdPASS ||
	    xTaskCreate(hw_main, "hw", HW_STACK / sizeof(StackType_t), NULL, APP_PRIO_SHELL + 4,
			NULL) != pdPASS) {
		return 1;
	}
	vTaskStartScheduler();
	return 1;
}
