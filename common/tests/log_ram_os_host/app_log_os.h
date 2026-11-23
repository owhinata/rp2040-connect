/* SPDX-License-Identifier: MIT */
/* The OS part of common/compat/log/app_log.h for the host tests of the RAM log
 * (tests/log_ram.c, tests/log_ring.c): nothing to read first and no options
 * (the defaults, as the FreeRTOS build's); the ring's size is given by the
 * test's build (-DLOG_RAM_TEST_SIZE=4096 or 8192, the FreeRTOS and the
 * NuttX build's, tests/run.sh).
 */
#ifndef TEST_LOG_RAM_APP_LOG_OS_H
#define TEST_LOG_RAM_APP_LOG_OS_H
#include <stdarg.h>
#ifndef LOG_RAM_TEST_SIZE
#error "build the test with -DLOG_RAM_TEST_SIZE=<bytes>"
#endif
#define LOG_RAM_SIZE LOG_RAM_TEST_SIZE
#endif
