/* SPDX-License-Identifier: MIT */
/* The host test's part of the generic shell commands
 * (common/shell/kernel_shell_os.h): a fixed tick count, restarts that are
 * only counted, and the record the test sets (freertos/tests/shell/main.c).
 */

#ifndef TESTS_SHELL_KERNEL_SHELL_OS_IMPL_H
#define TESTS_SHELL_KERNEL_SHELL_OS_IMPL_H

#include <stdint.h>

#include "fault_record.h"

#define TEST_TICK_COUNT 1234u

extern unsigned int test_reboots;
extern unsigned int test_bootsels;
extern uint32_t test_restart_delay_ms;
extern const struct fault_record *test_fault;

static inline uint32_t kernel_shell_os_tick_count(void)
{
	return TEST_TICK_COUNT;
}

static inline void kernel_shell_os_reboot(uint32_t delay_ms)
{
	test_reboots++;
	test_restart_delay_ms = delay_ms;
}

static inline void kernel_shell_os_bootsel(uint32_t delay_ms)
{
	test_bootsels++;
	test_restart_delay_ms = delay_ms;
}

static inline const struct fault_record *kernel_shell_os_fault_last(void)
{
	return test_fault;
}

#endif /* TESTS_SHELL_KERNEL_SHELL_OS_IMPL_H */
