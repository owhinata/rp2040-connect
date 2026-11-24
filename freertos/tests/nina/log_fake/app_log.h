/* SPDX-License-Identifier: MIT */
/* The host test's stand-in for the RAM log's macros (common/compat/log/
 * app_log.h) in common/drivers/nina/port/pico_sdk/nina_bus_dma_rp2040.c:
 * each message goes to test_log() of main.c, which keeps it for the test's
 * checks; a rate-limited one is marked "rl " (the limit itself is the RAM
 * log's, tested by common/tests/log_ram.c).
 */

#ifndef FREERTOS_TESTS_NINA_LOG_FAKE_APP_LOG_H
#define FREERTOS_TESTS_NINA_LOG_FAKE_APP_LOG_H

#define LOG_LEVEL_ERR 1
#define LOG_LEVEL_WRN 2
#define LOG_LEVEL_INF 3
#define LOG_LEVEL_DBG 4

void test_log(int level, const char *module, int ratelimited, const char *fmt, ...)
	__attribute__((format(printf, 4, 5)));

#define LOG_MODULE_REGISTER(name, level)                                                           \
	static const char test_log_module_[] __attribute__((unused)) = #name

#define LOG_ERR(...)           test_log(LOG_LEVEL_ERR, test_log_module_, 0, __VA_ARGS__)
#define LOG_ERR_RATELIMIT(...) test_log(LOG_LEVEL_ERR, test_log_module_, 1, __VA_ARGS__)

#endif /* FREERTOS_TESTS_NINA_LOG_FAKE_APP_LOG_H */
