/* SPDX-License-Identifier: Apache-2.0 */
/* The LOG_* macros of nuttx/drivers/include/app_log.h for the host test:
 * each message enabled by the module's level becomes one line
 * "<lvl> module: message" in a buffer the test reads (test_log_line()).
 * LOG_DBG is compiled out above the level, as in app_log.h.
 */
#ifndef TEST_FAKE_APP_LOG_H
#define TEST_FAKE_APP_LOG_H
#define LOG_LEVEL_NONE 0
#define LOG_LEVEL_ERR  1
#define LOG_LEVEL_WRN  2
#define LOG_LEVEL_INF  3
#define LOG_LEVEL_DBG  4
void test_log(int level, const char *module, const char *fmt, ...)
	__attribute__((format(printf, 3, 4)));
#define LOG_MODULE_REGISTER(name, level) \
	static const char test_log_module_[] __attribute__((unused)) = #name; \
	enum { test_log_level_ = (level) }
#define TEST_LOG_(level, ...) \
	do { if ((level) <= test_log_level_) { test_log((level), test_log_module_, __VA_ARGS__); } } while (0)
#define LOG_ERR(...) TEST_LOG_(LOG_LEVEL_ERR, __VA_ARGS__)
#define LOG_WRN(...) TEST_LOG_(LOG_LEVEL_WRN, __VA_ARGS__)
#define LOG_INF(...) TEST_LOG_(LOG_LEVEL_INF, __VA_ARGS__)
#define LOG_DBG(...) TEST_LOG_(LOG_LEVEL_DBG, __VA_ARGS__)
#endif
