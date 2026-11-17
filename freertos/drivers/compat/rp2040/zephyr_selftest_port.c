/* SPDX-License-Identifier: MIT */
/* The board's part of the compatibility layer's self-test
 * (zephyr_selftest.h): a hardware interrupt from an alarm of the Pico SDK's
 * default alarm pool (a TIMER alarm interrupt), the wait until the USB
 * console has sent what the test wrote, and real-time checks on.
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdio.h"
#include "pico/time.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"

#include "zephyr_selftest.h"

static void (*alarm_fn)(void *arg);

/* In the TIMER alarm's interrupt. */
static int64_t alarm_fired(alarm_id_t id, void *user_data)
{
	(void)id;
	alarm_fn(user_data);
	return 0; /* once */
}

int zephyr_selftest_port_alarm(uint32_t us, void (*fn)(void *arg), void *arg)
{
	alarm_fn = fn;
	return add_alarm_in_us(us, alarm_fired, arg, true) > 0 ? 0 : -EBUSY;
}

/* The longest wait for the host to read the console, and the time left after
 * TinyUSB's transmit buffer has emptied for the last packet's transfer to end.
 */
#define DRAIN_TIMEOUT_MS 500
#define DRAIN_LAST_MS    5

/* pico_stdio_usb hands the text to TinyUSB's transmit buffer (64 bytes) and
 * returns; the transfers end later in USB interrupts, and each run of the
 * SDK's USB worker interrupt releases the stdio mutex there, which wakes the
 * timer task (priority 31) through xEventGroupSetBitsFromISR()
 * (FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/port.c
 * vPortLockInternalSpinUnlockWithNotify()).  Both would preempt the test's
 * tasks.  The wait sleeps, so they run now.
 */
void zephyr_selftest_port_quiet(void)
{
	stdio_flush();
	for (int ms = 0; ms < DRAIN_TIMEOUT_MS && tud_cdc_connected() &&
			 tud_cdc_write_available() < CFG_TUD_CDC_TX_BUFSIZE;
	     ms++) {
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	vTaskDelay(pdMS_TO_TICKS(DRAIN_LAST_MS));
}

bool zephyr_selftest_port_realtime(void)
{
	return true;
}
