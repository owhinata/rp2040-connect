/* SPDX-License-Identifier: MIT */
/* The FreeRTOS part of the NINA test (nina_test_os.h) on the FreeRTOS POSIX
 * port: the runner at the shell's priority (APP_PRIO_SHELL, 6: "nina
 * version" runs in the console's task) and the DMA engine, a task above
 * every other that moves the bytes a tick after the TX channel's start.
 * An interrupt handler is a plain call from a task (the binding's FromISR
 * calls work there).  A wake-up that is not a wait's own is a notification
 * on the NINA's index (2), which the binding's waits share.  The windows
 * before a wait's block (nina_test_os_dma_window() and the others) are
 * wrapped calls of the kernel (-Wl,--wrap, host_tests.sh): the
 * notification take of the DMA and socket event waits
 * (ulTaskGenericNotifyTake()), the semaphore take of READY's
 * (xQueueSemaphoreTake()).
 *
 * The binding's own behaviour, tested here: the Wi-Fi thread's wake-up
 * (nina_os_wifi_post(), _wait()): a post before any wait is not kept, and
 * the first wait returns at once; then a waiting task runs at the post; two
 * posts before the wait end one wait, and the next waits its whole time
 * (rounded up to ticks, plus one).
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "nina_os.h"
#include "nina_test_os.h"

#define RUNNER_STACK 16384
#define HELPER_STACK 16384

/* The shell's priority (APP_PRIO_SHELL). */
#define RUNNER_PRIORITY 6

/* The NINA's task notification (port/freertos/nina_os.c). */
#define NINA_NOTIFY_INDEX 2

static TaskHandle_t dma_engine_task, runner_task;

/* The configuration has a tick hook (the board's catch-up, not used here). */
void vApplicationTickHook(void)
{
}

uint32_t nina_test_os_ticks(void)
{
	return (uint32_t)xTaskGetTickCount();
}

unsigned int nina_test_os_tick_hz(void)
{
	return configTICK_RATE_HZ;
}

void nina_test_os_delay(uint32_t ticks)
{
	vTaskDelay(ticks);
}

nina_test_os_thread_t nina_test_os_thread_start(void (*fn)(void *arg), void *arg, int above)
{
	TaskHandle_t task;

	assert(xTaskCreate(fn, "helper", HELPER_STACK / sizeof(StackType_t), arg,
			   (UBaseType_t)(RUNNER_PRIORITY + above), &task) == pdPASS);
	return task;
}

void nina_test_os_thread_exit(void)
{
	vTaskDelete(NULL);
}

void nina_test_os_thread_delete(nina_test_os_thread_t thread)
{
	vTaskDelete(thread);
}

bool nina_test_os_thread_blocked(nina_test_os_thread_t thread)
{
	return eTaskGetState(thread) == eBlocked;
}

nina_test_os_thread_t nina_test_os_runner(void)
{
	return runner_task;
}

int nina_test_os_priority(void)
{
	return (int)uxTaskPriorityGet(NULL) - RUNNER_PRIORITY;
}

void nina_test_os_isr(void (*fn)(void *arg), void *arg)
{
	fn(arg);
}

/* The window armed for the runner's next blocking wait, and whether the
 * DMA engine's move runs in it.
 */
enum window {
	WINDOW_NONE,
	WINDOW_DMA,
	WINDOW_READY,
	WINDOW_EVENT,
};

static volatile enum window window;
static volatile bool in_window;

void nina_test_os_dma_window(void)
{
	window = WINDOW_DMA;
}

void nina_test_os_ready_window(void)
{
	window = WINDOW_READY;
}

void nina_test_os_event_window(void)
{
	window = WINDOW_EVENT;
}

uint32_t __real_ulTaskGenericNotifyTake(UBaseType_t index, BaseType_t clear, TickType_t ticks);
uint32_t __wrap_ulTaskGenericNotifyTake(UBaseType_t index, BaseType_t clear, TickType_t ticks);

uint32_t __wrap_ulTaskGenericNotifyTake(UBaseType_t index, BaseType_t clear, TickType_t ticks)
{
	if (index == NINA_NOTIFY_INDEX && ticks != 0 &&
	    xTaskGetCurrentTaskHandle() == runner_task) {
		if (window == WINDOW_DMA) {
			window = WINDOW_NONE;
			in_window = true;
			nina_test_dma_move();
			in_window = false;
		} else if (window == WINDOW_EVENT) {
			window = WINDOW_NONE;
			nina_test_irq_rise();
		}
	}
	return __real_ulTaskGenericNotifyTake(index, clear, ticks);
}

BaseType_t __real_xQueueSemaphoreTake(QueueHandle_t queue, TickType_t ticks);
BaseType_t __wrap_xQueueSemaphoreTake(QueueHandle_t queue, TickType_t ticks);

BaseType_t __wrap_xQueueSemaphoreTake(QueueHandle_t queue, TickType_t ticks)
{
	if (window == WINDOW_READY && ticks != 0 && xTaskGetCurrentTaskHandle() == runner_task) {
		window = WINDOW_NONE;
		nina_test_module_alive(true);
	}
	return __real_xQueueSemaphoreTake(queue, ticks);
}

/* In the DMA window the wrapper moves the bytes instead. */
void nina_test_os_dma_kick(void)
{
	if (window != WINDOW_DMA) {
		xTaskNotifyGive(dma_engine_task);
	}
}

bool nina_test_os_dma_caller_waits(void)
{
	return in_window || eTaskGetState(runner_task) == eBlocked;
}

void nina_test_os_stray_wake(void)
{
	(void)xTaskNotifyGiveIndexed(xTaskGetCurrentTaskHandle(), NINA_NOTIFY_INDEX);
}

static volatile unsigned int wakes;
static volatile TickType_t wake_ticks;

/* The Wi-Fi thread: waits 1000 ms again and again, and notes how many
 * waits ended and the ticks of the last.
 */
static void wifi_wake_helper(void *arg)
{
	(void)arg;
	for (;;) {
		TickType_t t = xTaskGetTickCount();

		nina_os_wifi_wait(1000);
		wake_ticks = xTaskGetTickCount() - t;
		wakes++;
	}
}

/* The time rounded up to ticks, plus one tick. */
static TickType_t sleep_ticks(uint64_t us)
{
	return (TickType_t)((us * configTICK_RATE_HZ + 999999u) / 1000000u + 1);
}

static void test_wifi_wake(void)
{
	UBaseType_t mine = uxTaskPriorityGet(NULL);
	TaskHandle_t task;
	unsigned int n;

	/* Before any wait: nothing to wake. */
	nina_os_wifi_post();
	wakes = 0;
	/* The first wait returns at once; the second waits. */
	assert(xTaskCreate(wifi_wake_helper, "wifi", HELPER_STACK / sizeof(StackType_t), NULL,
			   mine + 1, &task) == pdPASS);
	assert(wakes == 1 && wake_ticks == 0);
	/* A post wakes it at once. */
	nina_os_wifi_post();
	assert(wakes == 2 && wake_ticks <= 1);
	/* Two posts while it cannot run: one wait ends at once, the next takes
	 * its whole time. */
	vTaskPrioritySet(task, mine - 1);
	nina_os_wifi_post();
	nina_os_wifi_post();
	assert(wakes == 2);
	vTaskDelay(2);
	assert(wakes == 3);
	vTaskPrioritySet(task, mine + 1);
	n = wakes;
	vTaskDelay(sleep_ticks(1000000) + 10);
	assert(wakes == n + 1);
	assert(wake_ticks >= sleep_ticks(1000000) && wake_ticks <= sleep_ticks(1000000) + 5);
	vTaskDelete(task);
}

void nina_test_os_tests(void)
{
	test_wifi_wake();
}

static void dma_engine(void *arg)
{
	(void)arg;
	for (;;) {
		(void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		vTaskDelay(1);
		nina_test_dma_move();
	}
}

static void runner(void *arg)
{
	(void)arg;
	nina_test_run();
	_exit(0);
}

int main(void)
{
	/* printf() must not allocate in a task (the port can switch threads
	 * while the C library holds its locks).
	 */
	static char out[1 << 12];

	setvbuf(stdout, out, _IOLBF, sizeof(out));
	nina_test_setup();
	if (xTaskCreate(runner, "runner", RUNNER_STACK / sizeof(StackType_t), NULL, RUNNER_PRIORITY,
			&runner_task) != pdPASS ||
	    xTaskCreate(dma_engine, "dma", HELPER_STACK / sizeof(StackType_t), NULL,
			configMAX_PRIORITIES - 1, &dma_engine_task) != pdPASS) {
		return 2;
	}
	vTaskStartScheduler();
	return 3;
}
