/* SPDX-License-Identifier: MIT */
/* The parts of the ThreadX kernel test firmware (main.c, lock_test.c,
 * test_port.c; see main.c).
 */

#ifndef THREADX_TESTS_BOARD_KERNEL_BOARD_KERNEL_H
#define THREADX_TESTS_BOARD_KERNEL_BOARD_KERNEL_H

#include <stdbool.h>
#include <stdint.h>

/* The LED's patterns (main.c). */
enum board_led {
	BOARD_LED_BOOT,    /* on: main() has not seen a thread run yet */
	BOARD_LED_WAITING, /* 1 Hz: the kernel runs, waiting for the USB host */
	BOARD_LED_RUNNING, /* 5 Hz: the tests run */
	BOARD_LED_PASSED,  /* one short flash every 2 s: all passed */
	BOARD_LED_FAILED,  /* three short flashes every 2 s: a test failed */
	BOARD_LED_FAULT,   /* 1 s on, 1 s off: an assertion or a HardFault */
};

void board_led_set(enum board_led state);

/* The direct checks of the thread's lock on the Cortex-M0 port
 * (lock_test.c), and of the tick against the TIMER: they print a line
 * each and add to *passed and *failed.  From a thread of the layer below
 * Zephyr priority 10.
 */
void board_tick_test(int *passed, int *failed);
void board_lock_test(int *passed, int *failed);

/* The line of the tick's measurement, kept for the summary (main.c). */
extern char board_tick_line[96];

#endif /* THREADX_TESTS_BOARD_KERNEL_BOARD_KERNEL_H */
