/* SPDX-License-Identifier: MIT */
/* The board's platform part of the layer's kernel test
 * (common/tests/zephyr_kernel/zephyr_kernel_test.h): a hardware interrupt
 * from an alarm of the Pico SDK's default alarm pool (a TIMER alarm
 * interrupt), the wait until the USB console has sent what the test wrote,
 * real-time checks on (SysTick counts the processor clock), and interrupts
 * masked for a while, which the tick catches up (src/tick.c).
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/stdio.h"
#include "pico/time.h"
#include "tusb.h"

#include <zephyr/kernel.h>

#include "zephyr_kernel_os.h"
#include "zephyr_kernel_test.h"

static void (*alarm_fn)(void *arg);

/* In the TIMER alarm's interrupt. */
static int64_t alarm_fired(alarm_id_t id, void *user_data)
{
	(void)id;
	alarm_fn(user_data);
	return 0; /* once */
}

int zephyr_kernel_test_port_alarm(uint32_t us, void (*fn)(void *arg), void *arg)
{
	alarm_fn = fn;
	return add_alarm_in_us(us, alarm_fired, arg, true) > 0 ? 0 : -EBUSY;
}

/* The longest wait for the host to read the console, and the time left after
 * TinyUSB's transmit buffer has emptied for the last packet's transfer to end.
 */
#define DRAIN_TIMEOUT_MS 500
#define DRAIN_LAST_MS    5

/* pico_stdio_usb hands the text to TinyUSB's transmit buffer and returns;
 * the transfers end later in USB interrupts (and the SDK's USB worker
 * interrupt), which would land in the measurements.  The wait sleeps, so
 * they run now.
 */
void zephyr_kernel_test_port_quiet(void)
{
	stdio_flush();
	for (int ms = 0; ms < DRAIN_TIMEOUT_MS && tud_cdc_connected() &&
			 tud_cdc_write_available() < CFG_TUD_CDC_TX_BUFSIZE;
	     ms++) {
		z_os_sleep(1);
	}
	z_os_sleep(DRAIN_LAST_MS);
}

bool zephyr_kernel_test_port_realtime(void)
{
	return true;
}

/* PRIMASK, as flash_safe_execute() masks them for a flash erase. */
int zephyr_kernel_test_port_mask_irqs(uint32_t us)
{
	uint32_t primask = save_and_disable_interrupts();

	busy_wait_us_32(us);
	restore_interrupts(primask);
	return 0;
}
