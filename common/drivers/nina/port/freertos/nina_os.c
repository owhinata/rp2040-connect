/* SPDX-License-Identifier: MIT */
/* The kernel services of the shared NINA core on FreeRTOS (nina_os.h),
 * written on FreeRTOS and the Pico SDK.  The calls of the Zephyr build
 * (common/drivers/nina/port/zephyr/nina_os.c) and what stands for them here:
 *
 *   K_MUTEX_DEFINE(nina_lock),      a FreeRTOS recursive mutex
 *   nina_dns_lock and               (xSemaphoreCreateRecursiveMutexStatic()),
 *   connect_state_lock;             taken with xSemaphoreTakeRecursive(
 *   k_mutex_lock(K_FOREVER)         portMAX_DELAY): recursive for its owner,
 *                                   with FreeRTOS's priority inheritance
 *                                   (the holder is raised to its highest
 *                                   waiter, one level, and drops back when it
 *                                   has released every mutex it holds).  A
 *                                   wait for ever cannot fail
 *                                   (INCLUDE_vTaskSuspend): nina_os_lock()
 *                                   returns 0.  FreeRTOS refuses the unlock
 *                                   of a mutex the caller does not hold
 *   K_SEM_DEFINE(scan_request,      a FreeRTOS binary semaphore each: a give
 *   0, 1) and connect_request       while one is pending leaves one; the wait
 *                                   is for ever
 *   k_sleep(K_USEC(us)),            vTaskDelay(ticks + 1) for the time
 *   k_sleep(K_MSEC(ms))             rounded up to ticks (Zephyr wakes a
 *                                   relative timeout at the current tick +
 *                                   ticks + 1, FreeRTOS a delay of n at the
 *                                   current tick + n); 0 ticks yields
 *                                   (taskYIELD()).  With the board's 1 ms
 *                                   tick: 100 us 2 ticks, 10 ms 11, 750 ms
 *                                   751.  The argument is clamped to
 *                                   INT32_MAX, as k_msleep()/k_usleep() take
 *                                   an int32_t
 *   k_uptime_get()                  the RP2040 TIMER (time_us_64(), 64 bits
 *                                   of 1 us) in ms, rounded down: not the
 *                                   tick, which is 32 bits wide (it wraps
 *                                   after about 49.7 days) and stands still
 *                                   while interrupts are masked for longer
 *                                   than a tick (a flash erase) until the
 *                                   application's tick hook catches up
 *
 * Every timed wait below (READY, the socket events) blocks for the time
 * rounded up to ticks, plus one tick, as the sleeps; 0 does not block.
 *
 * The objects are static (no heap) and made at the first use of any of
 * them (objects() below), from a task or before the scheduler starts.
 *
 * The core's waits for READY (nina_proto.c wait_ready()) sleep on an edge
 * interrupt of the line instead of sampling it (nina_os_ready_*() of
 * nina_os.h; the Zephyr and NuttX builds wait on its interrupt in the same
 * way):
 *
 *   nina_os_ready_start()       adds this file's handler of IO_IRQ_BANK0
 *                               (below) and enables the interrupt; 0
 *   nina_os_ready_arm(desired)  drops a wake-up left from before (takes
 *                               the semaphore without waiting), then enables
 *                               the edge of READY (GPIO10, active low:
 *                               nina_bus_rp2040.c) towards DESIRED, falling for 1
 *                               (ready), rising for 0: all of the pin's
 *                               events off, then the edge on
 *                               (gpio_set_irq_enabled(), which clears the
 *                               events it names before it changes the
 *                               enable, so an edge latched while the event
 *                               was off does not fire)
 *   the interrupt               the handler acknowledges the edge and gives
 *                               the semaphore (xSemaphoreGiveFromISR()); the
 *                               waiting task runs at the handler's end
 *                               (portYIELD_FROM_ISR(): PendSV)
 *   nina_os_ready_wait(ms)      xSemaphoreTake() for at most
 *                               READY_WAIT_SLICE_MS at a time.  The core's
 *                               deadline is on the TIMER
 *                               (nina_os_uptime_ms()), the wait on the tick,
 *                               which stands still while interrupts are
 *                               masked; in slices, as a sampling would (one
 *                               read of READY per slice while the module
 *                               stays busy)
 *   nina_os_ready_disarm()      all of the pin's events off (no interrupts
 *                               while CS toggles READY during the transfers)
 *
 * Nothing is lost between the core's read of READY and its wait: the edge
 * is enabled before the read, so a change after the read gives the
 * semaphore, and the wait returns at once.  A change before the read is
 * seen by the read.  One waiter at a time: the core calls these with
 * nina_lock held.
 *
 * The waits of the socket loops for the module's socket events on its IRQ
 * line (nina_os_event_*() of nina_os.h, nina_wait.c):
 *
 *   nina_os_event_start()       adds the handler (if nina_os_ready_start()
 *                               has not) and enables the rising edge of
 *                               GPIO2 (the IRQ line, active high:
 *                               nina_bus_rp2040.c) as READY's; the line stays an
 *                               input with its pull-down.  The probe calls
 *                               it after the module's reset, during which
 *                               the line is driven; 0
 *   the interrupt               the count goes up and every task that waits
 *                               is woken (a broadcast): each waiter is on a
 *                               list (on its stack), which the handler
 *                               empties, notifying each task on its task
 *                               notification NINA_OS_NOTIFY_INDEX
 *   nina_os_event_seq()         the count (written by the interrupt only)
 *   nina_os_event_wait()        while the count is SEQ and the time
 *                               (nina_os_uptime_ms(), the TIMER) is not up:
 *                               joins the list, in one critical section
 *                               with a last look at the count (an edge after
 *                               the caller read SEQ either is seen there or
 *                               wakes the task), and waits on the
 *                               notification for the time left; leaves the
 *                               list when it times out
 *
 * The handler shares IO_IRQ_BANK0 with the board's GPIO bank 0 backend
 * (freertos/drivers/compat/rp2040/nano_gpio.c, the LSM6DSOX's INT1, GPIO24)
 * the SDK's way: each adds a raw handler with
 * gpio_add_raw_irq_handler_masked() for its own pins (the SDK asserts that
 * no pin is in two masks), and each looks at and acknowledges only its own
 * pins.  One handler for GPIO10 and GPIO2 (the SDK has a few shared
 * handler slots for all interrupts).  It runs at the SDK's default priority
 * (0x80), as every peripheral interrupt of the build.  The RP2040 port of
 * FreeRTOS (Cortex-M0+, no BASEPRI) masks all interrupts (PRIMASK) in its
 * critical sections, so a FromISR call is safe from an interrupt of any
 * priority (configMAX_SYSCALL_INTERRUPT_PRIORITY plays no part).
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "nina_os.h"

/* The task notification of the socket events' waits; index 0 is the Pico
 * SDK's and the application's, 1 the compatibility layer's
 * (FreeRTOSConfig.h).
 */
#define NINA_OS_NOTIFY_INDEX 2

#if configTASK_NOTIFICATION_ARRAY_ENTRIES <= NINA_OS_NOTIFY_INDEX
#error "the NINA's socket event waits use task notification index 2"
#endif

/* READY: GPIO10, active low; the IRQ line: GPIO2, active high
 * (nina_bus_rp2040.c, the devicetree's ready-gpios and irq-gpios).
 */
#define READY_GPIO 10
#define IRQ_GPIO   2

#define NINA_GPIO_IRQ_PINS ((1u << READY_GPIO) | (1u << IRQ_GPIO))

#define GPIO_EVENTS_ALL                                                                            \
	(GPIO_IRQ_LEVEL_LOW | GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)

/* The longest single wait for READY (see above). */
#define READY_WAIT_SLICE_MS 10

static StaticSemaphore_t nina_lock_buffer;
static StaticSemaphore_t nina_dns_lock_buffer;
static StaticSemaphore_t connect_state_lock_buffer;
static StaticSemaphore_t scan_request_buffer;
static StaticSemaphore_t connect_request_buffer;
static StaticSemaphore_t ready_sem_buffer;

static SemaphoreHandle_t nina_lock;
static SemaphoreHandle_t nina_dns_lock;
static SemaphoreHandle_t connect_state_lock;
static SemaphoreHandle_t scan_request;
static SemaphoreHandle_t connect_request;
static SemaphoreHandle_t ready_sem;

/* Makes the objects once: the first lock or request of any task finds
 * ready_sem, the last one made, still NULL.  The critical section keeps a
 * second task from making them again.
 */
static void make_objects(void)
{
	taskENTER_CRITICAL();
	if (ready_sem == NULL) {
		nina_lock = xSemaphoreCreateRecursiveMutexStatic(&nina_lock_buffer);
		nina_dns_lock = xSemaphoreCreateRecursiveMutexStatic(&nina_dns_lock_buffer);
		connect_state_lock =
			xSemaphoreCreateRecursiveMutexStatic(&connect_state_lock_buffer);
		scan_request = xSemaphoreCreateBinaryStatic(&scan_request_buffer);
		connect_request = xSemaphoreCreateBinaryStatic(&connect_request_buffer);
		ready_sem = xSemaphoreCreateBinaryStatic(&ready_sem_buffer);
	}
	taskEXIT_CRITICAL();
}

static inline void objects(void)
{
	if (ready_sem == NULL) {
		make_objects();
	}
}

/* Time ----------------------------------------------------------------------*/

/* US in ticks, rounded up (Zephyr's K_USEC() and K_MSEC():
 * k_us_to_ticks_ceil64()).  US is at most UINT32_MAX * 1000.
 */
static uint64_t ticks_ceil(uint64_t us)
{
	return (us * configTICK_RATE_HZ + 999999u) / 1000000u;
}

/* The block time of a wait of TICKS: 0 does not block, else ticks + 1, at
 * most portMAX_DELAY - 1 (portMAX_DELAY would be for ever).
 */
static TickType_t block_ticks(uint64_t ticks)
{
	if (ticks == 0) {
		return 0;
	}
	if (ticks >= (uint64_t)(portMAX_DELAY - 2)) {
		return portMAX_DELAY - 1;
	}
	return (TickType_t)ticks + 1;
}

static void sleep_ticks(uint64_t ticks)
{
	if (ticks == 0) {
		/* Behind the other ready tasks of the priority, as Zephyr's yield. */
		taskYIELD();
		return;
	}
	/* The current tick + ticks + 1, in steps that a delay can hold. */
	for (ticks += 1; ticks > 0;) {
		TickType_t step = ticks < (uint64_t)(portMAX_DELAY - 1) ? (TickType_t)ticks
									: portMAX_DELAY - 1;

		vTaskDelay(step);
		ticks -= step;
	}
}

void nina_os_sleep_us(uint32_t us)
{
	sleep_ticks(ticks_ceil(us > INT32_MAX ? INT32_MAX : us));
}

void nina_os_sleep_ms(uint32_t ms)
{
	sleep_ticks(ticks_ceil((uint64_t)(ms > INT32_MAX ? INT32_MAX : ms) * 1000u));
}

int64_t nina_os_uptime_ms(void)
{
	return (int64_t)(time_us_64() / 1000u);
}

/* Locks and requests ----------------------------------------------------------*/

int nina_os_lock(void)
{
	objects();
	return xSemaphoreTakeRecursive(nina_lock, portMAX_DELAY) == pdPASS ? 0 : -EAGAIN;
}

void nina_os_unlock(void)
{
	(void)xSemaphoreGiveRecursive(nina_lock);
}

void nina_os_dns_lock(void)
{
	objects();
	(void)xSemaphoreTakeRecursive(nina_dns_lock, portMAX_DELAY);
}

void nina_os_dns_unlock(void)
{
	(void)xSemaphoreGiveRecursive(nina_dns_lock);
}

void nina_os_wifi_lock(void)
{
	objects();
	(void)xSemaphoreTakeRecursive(connect_state_lock, portMAX_DELAY);
}

void nina_os_wifi_unlock(void)
{
	(void)xSemaphoreGiveRecursive(connect_state_lock);
}

static SemaphoreHandle_t wifi_request(unsigned int which)
{
	return which == NINA_OS_WIFI_SCAN ? scan_request : connect_request;
}

void nina_os_wifi_post(unsigned int which)
{
	objects();
	(void)xSemaphoreGive(wifi_request(which));
}

void nina_os_wifi_wait(unsigned int which)
{
	objects();
	(void)xSemaphoreTake(wifi_request(which), portMAX_DELAY);
}

/* The lines' interrupt ----------------------------------------------------------*/

/* A task in nina_os_event_wait(), on its stack. */
struct event_waiter {
	struct event_waiter *next;
	TaskHandle_t task;
};

static volatile uint32_t event_count;
static struct event_waiter *event_waiters;
static bool attached;

/* In the interrupt handler: one more edge, every waiter woken. */
static void event_edge(BaseType_t *woken)
{
	UBaseType_t key = taskENTER_CRITICAL_FROM_ISR();
	struct event_waiter *w = event_waiters;

	event_count++;
	event_waiters = NULL;
	while (w != NULL) {
		struct event_waiter *next = w->next;

		w->next = NULL;
		vTaskNotifyGiveIndexedFromISR(w->task, NINA_OS_NOTIFY_INDEX, woken);
		w = next;
	}
	taskEXIT_CRITICAL_FROM_ISR(key);
}

/* IO_IRQ_BANK0, for the pins of NINA_GPIO_IRQ_PINS only. */
static void nina_os_gpio_isr(void)
{
	BaseType_t woken = pdFALSE;

	if (gpio_get_irq_event_mask(IRQ_GPIO) != 0) {
		gpio_acknowledge_irq(IRQ_GPIO, GPIO_EVENTS_ALL);
		event_edge(&woken);
	}
	if (gpio_get_irq_event_mask(READY_GPIO) != 0) {
		gpio_acknowledge_irq(READY_GPIO, GPIO_EVENTS_ALL);
		(void)xSemaphoreGiveFromISR(ready_sem, &woken);
	}
	portYIELD_FROM_ISR(woken);
}

/* Adds the handler once (from a task; the critical section keeps two first
 * callers from adding it twice).
 */
static void gpio_attach(void)
{
	configASSERT(ready_sem != NULL);
	taskENTER_CRITICAL();
	if (!attached) {
		gpio_add_raw_irq_handler_masked(NINA_GPIO_IRQ_PINS, nina_os_gpio_isr);
		irq_set_enabled(IO_IRQ_BANK0, true);
		attached = true;
	}
	taskEXIT_CRITICAL();
}

/* All of PIN's events off, then EVENTS (0: none) on.  The pins' enables
 * are set and cleared through the registers' atomic aliases.
 */
static void gpio_events(unsigned int pin, uint32_t events)
{
	gpio_set_irq_enabled(pin, GPIO_EVENTS_ALL, false);
	if (events != 0) {
		gpio_set_irq_enabled(pin, events, true);
	}
}

/* READY ---------------------------------------------------------------------*/

int nina_os_ready_start(void)
{
	gpio_attach();
	return 0;
}

void nina_os_ready_arm(int desired)
{
	/* Active low: ready (1) is the falling edge. */
	uint32_t edge = desired ? GPIO_IRQ_EDGE_FALL : GPIO_IRQ_EDGE_RISE;

	(void)xSemaphoreTake(ready_sem, 0);
	gpio_events(READY_GPIO, edge);
}

void nina_os_ready_wait(uint32_t ms)
{
	uint32_t slice = ms < READY_WAIT_SLICE_MS ? ms : READY_WAIT_SLICE_MS;

	(void)xSemaphoreTake(ready_sem, block_ticks(ticks_ceil((uint64_t)slice * 1000u)));
}

void nina_os_ready_disarm(void)
{
	gpio_events(READY_GPIO, 0);
}

/* The socket events -------------------------------------------------------------*/

int nina_os_event_start(void)
{
	gpio_attach();
	gpio_events(IRQ_GPIO, GPIO_IRQ_EDGE_RISE);
	return 0;
}

uint32_t nina_os_event_seq(void)
{
	return event_count;
}

static void event_unlink(struct event_waiter *waiter)
{
	struct event_waiter **link;

	for (link = &event_waiters; *link != NULL; link = &(*link)->next) {
		if (*link == waiter) {
			*link = waiter->next;
			waiter->next = NULL;
			return;
		}
	}
}

void nina_os_event_wait(uint32_t seq, uint32_t ms)
{
	struct event_waiter waiter = {.next = NULL, .task = xTaskGetCurrentTaskHandle()};
	int64_t end = nina_os_uptime_ms() + ms;
	int64_t left = ms;

	while (event_count == seq && left > 0) {
		taskENTER_CRITICAL();
		if (event_count != seq) {
			taskEXIT_CRITICAL();
			break;
		}
		/* A notification left by an earlier wait that timed out as it was
		 * woken.
		 */
		(void)xTaskNotifyStateClearIndexed(NULL, NINA_OS_NOTIFY_INDEX);
		(void)ulTaskNotifyValueClearIndexed(NULL, NINA_OS_NOTIFY_INDEX, UINT32_MAX);
		waiter.next = event_waiters;
		event_waiters = &waiter;
		taskEXIT_CRITICAL();

		(void)ulTaskNotifyTakeIndexed(NINA_OS_NOTIFY_INDEX, pdTRUE,
					      block_ticks(ticks_ceil((uint64_t)left * 1000u)));

		taskENTER_CRITICAL();
		event_unlink(&waiter);
		taskEXIT_CRITICAL();
		left = end - nina_os_uptime_ms();
	}
}
