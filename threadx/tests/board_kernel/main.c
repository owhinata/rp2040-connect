/* SPDX-License-Identifier: MIT */
/* The ThreadX kernel test firmware: a bring-up tool for the ThreadX build
 * on the board, not the product firmware.  It runs the Zephyr compatibility
 * layer's kernel test (common/tests/zephyr_kernel/zephyr_kernel_test.c, on
 * the ThreadX primitives, threadx/drivers/compat) on ThreadX's Cortex-M0
 * port with the board's kernel part (threadx/app/usb-console: tx_user.h,
 * the low-level initialization and the tick), with the platform part of the
 * board (test_port.c), so the checks of interrupts, real time and the tick's
 * catch-up run too, and the board's direct checks (lock_test.c).
 *
 * The start is the product's: main() sets up the board, then ThreadX
 * without the scheduler (_tx_initialize_kernel_setup(): the low-level and
 * the high-level initialization, interrupts masked from here), then the
 * layer (zephyr_compat_initialize()) and the first thread, then the
 * scheduler (tx_kernel_enter(), which skips the initialization done and
 * calls tx_application_define(), empty here).
 *
 * Output on the USB CDC port (product string "ThreadX kernel test"), once
 * the host has opened it (DTR): the ThreadX version, the tick measured
 * against the TIMER, the direct checks, the test's lines and a summary,
 * which is printed again every SUMMARY_PERIOD_MS.  The LED (GPIO6) shows the
 * state without USB (board_kernel.h).
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/structs/systick.h"
#include "hardware/sync.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/time.h"

#include "tx_api.h"
#include "tx_initialize.h"

#include <zephyr/kernel.h>

#include "board_kernel.h"
#include "priorities.h"
#include "zephyr_compat.h"
#include "zephyr_kernel_os.h"
#include "zephyr_kernel_test.h"

#define RUNNER_STACK      4096
#define SUMMARY_PERIOD_MS 5000
/* After the host opens the port, before the first line. */
#define HOST_SETTLE_MS    500

static K_THREAD_STACK_DEFINE(runner_stack, RUNNER_STACK);
static z_os_thread_t runner_tid;

/* The LED ----------------------------------------------------------------*/

/* 20 slots of 100 ms, bit n for slot n; a TIMER alarm (the default alarm
 * pool) steps them, so the LED needs neither the tick nor a thread.  When
 * the ThreadX clock has not moved since the last slot (and a thread has
 * run), the LED is off: the tick has stopped.
 */
#define LED_SLOT_MS 100
#define LED_SLOTS   20

static const uint32_t led_patterns[] = {
	[BOARD_LED_BOOT] = 0xfffff,    /* on */
	[BOARD_LED_WAITING] = 0x07c1f, /* 500 ms on, 500 ms off */
	[BOARD_LED_RUNNING] = 0x55555, /* 100 ms on, 100 ms off */
	[BOARD_LED_PASSED] = 0x00001,  /* 100 ms on in 2 s */
	[BOARD_LED_FAILED] = 0x00015,  /* 3 x 100 ms on in 2 s */
	[BOARD_LED_FAULT] = 0x003ff,   /* 1 s on, 1 s off */
};

static volatile enum board_led led_state = BOARD_LED_BOOT;
static repeating_timer_t led_timer;
static unsigned int led_slot;
static ULONG led_clock;

void board_led_set(enum board_led state)
{
	led_state = state;
}

/* In the alarm's interrupt. */
static bool led_step(repeating_timer_t *t)
{
	ULONG clock = tx_time_get();
	enum board_led state = led_state;
	bool on = (led_patterns[state] >> led_slot) & 1u;

	(void)t;
	if (state != BOARD_LED_BOOT && clock == led_clock) {
		on = false;
	}
	led_clock = clock;
	led_slot = (led_slot + 1) % LED_SLOTS;
	gpio_put(PICO_DEFAULT_LED_PIN, on);
	return true;
}

/* The fault pattern by spinning, with interrupts masked: for a fault in an
 * interrupt, before the kernel runs, or inside the thread's lock.
 */
static void __attribute__((noreturn)) led_fault_forever(void)
{
	(void)save_and_disable_interrupts();
	for (;;) {
		gpio_put(PICO_DEFAULT_LED_PIN, true);
		busy_wait_us_32(1000000);
		gpio_put(PICO_DEFAULT_LED_PIN, false);
		busy_wait_us_32(1000000);
	}
}

/* Faults -----------------------------------------------------------------*/

extern UINT z_tx_lock_depth;

/* Z_TX_ASSERT() (zephyr_os_port.h) and this firmware's own checks.  In a
 * thread outside the lock, the line is printed again every
 * SUMMARY_PERIOD_MS and the thread stops there; elsewhere only the LED
 * tells.
 */
void app_assert_failed(const char *file, int line)
{
	board_led_set(BOARD_LED_FAULT);
	if (z_os_in_isr() || z_os_pre_kernel() || z_tx_lock_depth != 0) {
		led_fault_forever();
	}
	for (;;) {
		printf("ASSERTION FAILED: %s:%d\n", file, line);
		z_os_sleep(SUMMARY_PERIOD_MS);
	}
}

/* The Pico SDK's is a breakpoint. */
void isr_hardfault(void);

void isr_hardfault(void)
{
	led_fault_forever();
}

/* The test's output ------------------------------------------------------*/

/* The test writes each item's lines with fputs() to stdout (st_flush());
 * they go to the Pico SDK's stdio directly (newlib's fputs() would
 * allocate a buffer for stdout; no other stream is used), and are scanned
 * for the summary: an item's name (a line that does not start with a
 * space), "  skipped: ..." and its end, "  ok" or "  FAILED".
 */
int __wrap_fputs(const char *s, FILE *stream);

static bool scan_on;
static char scan_line[40];
static size_t scan_len;
static char scan_item[32];
static bool scan_skipped;
static int kt_passed;
static int kt_failed;
static int kt_skipped;
static char kt_failed_names[128];

static void scan_line_end(void)
{
	const char *l = scan_line;

	if (strncmp(l, "zephyr_kernel_test:", 19) == 0) {
		return;
	}
	if (l[0] != ' ' && l[0] != '\0') {
		strncpy(scan_item, l, sizeof(scan_item) - 1);
		scan_item[sizeof(scan_item) - 1] = '\0';
		scan_skipped = false;
	} else if (strncmp(l, "  skipped", 9) == 0) {
		scan_skipped = true;
	} else if (strcmp(l, "  ok") == 0) {
		if (scan_skipped) {
			kt_skipped++;
		} else {
			kt_passed++;
		}
	} else if (strcmp(l, "  FAILED") == 0) {
		size_t n = strlen(kt_failed_names);

		kt_failed++;
		snprintf(kt_failed_names + n, sizeof(kt_failed_names) - n, " %s", scan_item);
	}
}

static void scan(const char *s)
{
	for (; *s != '\0'; s++) {
		if (*s == '\n') {
			scan_line[scan_len] = '\0';
			scan_line_end();
			scan_len = 0;
		} else if (scan_len < sizeof(scan_line) - 1) {
			scan_line[scan_len++] = *s;
		}
	}
}

int __wrap_fputs(const char *s, FILE *stream)
{
	if (stream != stdout && stream != stderr) {
		return EOF;
	}
	if (scan_on) {
		scan(s);
	}
	(void)stdio_put_string(s, -1, false, true);
	return 0;
}

/* The run -------------------------------------------------------------------*/

static int board_passed;
static int board_failed;
static int kt_returned;

static void print_summary(void)
{
	printf("%s\n", board_tick_line);
	printf("board_test: %d passed, %d failed\n", board_passed, board_failed);
	printf("zephyr_kernel_test: %d passed, %d failed, %d skipped\n", kt_passed, kt_failed,
	       kt_skipped);
	if (kt_failed != 0) {
		printf("  failed:%s\n", kt_failed_names);
	}
	if (kt_returned != kt_failed) {
		printf("  (the run counted %d failed)\n", kt_returned);
	}
}

static void runner(void *arg)
{
	(void)arg;
	board_led_set(BOARD_LED_WAITING);
	while (!stdio_usb_connected()) {
		z_os_sleep(10);
	}
	z_os_sleep(HOST_SETTLE_MS);
	board_led_set(BOARD_LED_RUNNING);

	printf("\nThreadX kernel test (threadx/tests/board_kernel)\n");
	printf("ThreadX %d.%d.%d.%d%c: %s\n", THREADX_MAJOR_VERSION, THREADX_MINOR_VERSION,
	       THREADX_PATCH_VERSION, THREADX_BUILD_VERSION, THREADX_HOTFIX_VERSION,
	       _tx_version_id);
	printf("clk_sys %lu Hz, SysTick reload %lu, %d ticks per second\n",
	       (unsigned long)clock_get_hz(clk_sys), (unsigned long)systick_hw->rvr,
	       TX_TIMER_TICKS_PER_SECOND);

	board_tick_test(&board_passed, &board_failed);
	board_lock_test(&board_passed, &board_failed);
	printf("board_test: %d passed, %d failed\n", board_passed, board_failed);
	zephyr_kernel_test_port_quiet();

	scan_on = true;
	kt_returned = zephyr_kernel_test_run();
	scan_on = false;

	printf("\n");
	print_summary();
	board_led_set(board_failed == 0 && kt_failed == 0 && kt_returned == 0 && kt_passed > 0
			      ? BOARD_LED_PASSED
			      : BOARD_LED_FAILED);
	for (;;) {
		z_os_sleep(SUMMARY_PERIOD_MS);
		printf("\n--- summary, %lu s after the start\n",
		       (unsigned long)(time_us_64() / 1000000u));
		print_summary();
	}
}

void tx_application_define(void *first_unused_memory)
{
	/* Everything was created by main() before tx_kernel_enter(). */
	(void)first_unused_memory;
}

int main(void)
{
	stdio_init_all();

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
	gpio_put(PICO_DEFAULT_LED_PIN, true);
	if (!add_repeating_timer_ms(-LED_SLOT_MS, led_step, NULL, &led_timer)) {
		app_assert_failed(__FILE__, __LINE__);
	}

	/* The tick's catch-up takes SysTick's period for exactly one tick of
	 * the TIMER (tx_initialize_low_level.c).
	 */
	if (clock_get_hz(clk_sys) % TX_TIMER_TICKS_PER_SECOND != 0) {
		app_assert_failed(__FILE__, __LINE__);
	}

	_tx_initialize_kernel_setup();
	if (zephyr_compat_initialize() != 0) {
		app_assert_failed(__FILE__, __LINE__);
	}
	if (z_os_thread_start(runner, "runner", runner_stack, K_THREAD_STACK_SIZEOF(runner_stack),
			      APP_PRIO_SHELL, NULL, &runner_tid) != 0) {
		app_assert_failed(__FILE__, __LINE__);
	}

	tx_kernel_enter();
	app_assert_failed(__FILE__, __LINE__);
	return 0;
}
