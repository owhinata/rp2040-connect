/* SPDX-License-Identifier: Apache-2.0 */
/* The logging macros of the port: LOG_MODULE_REGISTER(), LOG_ERR(),
 * LOG_WRN(), LOG_INF(), LOG_DBG() and the *_RATELIMIT() forms, used the way
 * Zephyr's <zephyr/logging/log.h> is used by zephyr/app/usb-console, so its
 * sources move over with only the include changed.  Each message becomes
 * one line in the RAM log (drivers/log/log_ram.c, shown by "dmesg") in
 * Zephyr's format:
 *
 *   [HH:MM:SS.mmm,uuu] <inf> module: message\n
 *
 * with <err>, <wrn>, <inf> and <dbg>, and "module.function: " for <dbg>
 * (Zephyr's CONFIG_LOG_FUNC_NAME_PREFIX_DBG).  The time is the board's
 * monotonic clock, microseconds since boot (nano_mono_us()).
 *
 * Usable from the kernel side (nuttx/drivers, the board code) and from the
 * applications (flat build), in threads and in interrupt handlers.  The
 * include path is $(TOPDIR)/external/include.
 *
 * Levels are checked at compile time only: a message above the level of
 * its file (or above CONFIG_NANO_LOG_MAX_LEVEL) is not compiled in.
 * Zephyr's run-time filtering (the "log" shell command) is not provided.
 *
 * Unlike Zephyr's deferred mode, the line is formatted at once, in the
 * caller's context and on its stack (see app_log_printf() in log_ram.c).
 *
 * <syslog.h> defines LOG_ERR as a syslog priority.  This header includes
 * <syslog.h> first and replaces LOG_ERR with the function-like macro, so a
 * file that includes app_log.h cannot use syslog(LOG_ERR, ...); it can use
 * APP_LOG_SYSLOG_ERR instead.
 */

#ifndef __NUTTX_DRIVERS_INCLUDE_APP_LOG_H
#define __NUTTX_DRIVERS_INCLUDE_APP_LOG_H

#include <nuttx/config.h>

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/compiler.h>

/* The syslog priority that <syslog.h> calls LOG_ERR. */

#define APP_LOG_SYSLOG_ERR 3
#undef LOG_ERR

/* Zephyr's levels (zephyr/logging/log_core.h). */

#define LOG_LEVEL_NONE 0
#define LOG_LEVEL_ERR  1
#define LOG_LEVEL_WRN  2
#define LOG_LEVEL_INF  3
#define LOG_LEVEL_DBG  4

/* Zephyr: CONFIG_LOG_DEFAULT_LEVEL=3, CONFIG_LOG_MAX_LEVEL=4,
 * CONFIG_LOG_RATELIMIT_INTERVAL_MS=5000 (zephyr/build/usb-console).
 */

#ifdef CONFIG_NANO_LOG_DEFAULT_LEVEL
#  define APP_LOG_DEFAULT_LEVEL CONFIG_NANO_LOG_DEFAULT_LEVEL
#else
#  define APP_LOG_DEFAULT_LEVEL LOG_LEVEL_INF
#endif

#ifdef CONFIG_NANO_LOG_MAX_LEVEL
#  define APP_LOG_MAX_LEVEL CONFIG_NANO_LOG_MAX_LEVEL
#else
#  define APP_LOG_MAX_LEVEL LOG_LEVEL_DBG
#endif

#define LOG_RATELIMIT_INTERVAL_MS 5000

/* The level of a module registered without one: LOG_LEVEL if the file
 * defines it before including this header (as Zephyr's i2c_dw.c does),
 * else the default.
 */

#ifdef LOG_LEVEL
#  define APP_LOG_FILE_LEVEL LOG_LEVEL
#else
#  define APP_LOG_FILE_LEVEL APP_LOG_DEFAULT_LEVEL
#endif

/* LOG_MODULE_REGISTER(name) or LOG_MODULE_REGISTER(name, level), once per
 * file, at file scope.  LOG_MODULE_DECLARE() is the same here: each file
 * keeps its own copy of the name.
 */

#define APP_LOG_MODULE_(name, level, ...) \
  static const char app_log_module_name_[] unused_data = #name; \
  enum { app_log_module_level_ = (level) }

#define LOG_MODULE_REGISTER(...) \
  APP_LOG_MODULE_(__VA_ARGS__, APP_LOG_FILE_LEVEL, 0)
#define LOG_MODULE_DECLARE(...) \
  APP_LOG_MODULE_(__VA_ARGS__, APP_LOG_FILE_LEVEL, 0)

/* Compile-time check, a constant expression. */

#define APP_LOG_ENABLED(level) \
  ((level) <= app_log_module_level_ && (level) <= APP_LOG_MAX_LEVEL)

#define APP_LOG_(level, func, ...) \
  do \
    { \
      if (APP_LOG_ENABLED(level)) \
        { \
          app_log_printf((level), app_log_module_name_, (func), \
                         __VA_ARGS__); \
        } \
    } \
  while (0)

#define LOG_ERR(...) APP_LOG_(LOG_LEVEL_ERR, NULL, __VA_ARGS__)
#define LOG_WRN(...) APP_LOG_(LOG_LEVEL_WRN, NULL, __VA_ARGS__)
#define LOG_INF(...) APP_LOG_(LOG_LEVEL_INF, NULL, __VA_ARGS__)
#define LOG_DBG(...) APP_LOG_(LOG_LEVEL_DBG, __func__, __VA_ARGS__)

/* Rate-limited forms (Zephyr's _LOG_RATELIMIT_CORE with
 * CONFIG_LOG_RATELIMIT=y): per call site, at most one message per interval
 * (milliseconds of uptime, wrapping at 32 bits); the first one always goes
 * out.  The messages held back in between are counted, and the next one
 * that goes out is preceded by "Skipped N messages" at the same level.
 * Zephyr uses atomics; here the check runs with interrupts disabled.
 */

#define APP_LOG_RATELIMIT_(level, func, rate_ms, ...) \
  do \
    { \
      if (APP_LOG_ENABLED(level)) \
        { \
          static uint32_t app_log_last_; \
          static uint32_t app_log_skipped_; \
          uint32_t app_log_n_; \
          if (app_log_ratelimit_pass(&app_log_last_, &app_log_skipped_, \
                                     (rate_ms), &app_log_n_)) \
            { \
              if (app_log_n_ > 0) \
                { \
                  app_log_printf((level), app_log_module_name_, (func), \
                                 "Skipped %d messages", (int)app_log_n_); \
                } \
              app_log_printf((level), app_log_module_name_, (func), \
                             __VA_ARGS__); \
            } \
        } \
    } \
  while (0)

#define LOG_ERR_RATELIMIT(...) \
  APP_LOG_RATELIMIT_(LOG_LEVEL_ERR, NULL, LOG_RATELIMIT_INTERVAL_MS, \
                     __VA_ARGS__)
#define LOG_WRN_RATELIMIT(...) \
  APP_LOG_RATELIMIT_(LOG_LEVEL_WRN, NULL, LOG_RATELIMIT_INTERVAL_MS, \
                     __VA_ARGS__)
#define LOG_INF_RATELIMIT(...) \
  APP_LOG_RATELIMIT_(LOG_LEVEL_INF, NULL, LOG_RATELIMIT_INTERVAL_MS, \
                     __VA_ARGS__)
#define LOG_DBG_RATELIMIT(...) \
  APP_LOG_RATELIMIT_(LOG_LEVEL_DBG, __func__, LOG_RATELIMIT_INTERVAL_MS, \
                     __VA_ARGS__)

#ifdef __cplusplus
extern "C"
{
#endif

/* Format one line and store it in the RAM log (drivers/log/log_ram.c).
 * level is LOG_LEVEL_ERR..LOG_LEVEL_DBG; func is NULL, or the function
 * name to put after the module name.  The line is built in a buffer of
 * APP_LOG_LINE_MAX bytes on the caller's stack; a longer message is cut
 * (the line still ends with '\n').
 */

#define APP_LOG_LINE_MAX 192

void app_log_printf(int level, FAR const char *module, FAR const char *func,
                    FAR const char *fmt, ...) printf_like(4, 5);
void app_log_vprintf(int level, FAR const char *module, FAR const char *func,
                     FAR const char *fmt, va_list ap) printf_like(4, 0);

/* The rate-limit decision of APP_LOG_RATELIMIT_(): returns true if the
 * message may go out (and sets *skipped_out to the number of messages held
 * back since the last one, clearing it), false if it is held back (and
 * counts it).
 */

bool app_log_ratelimit_pass(FAR uint32_t *last_ms, FAR uint32_t *skipped,
                            uint32_t rate_ms, FAR uint32_t *skipped_out);

#ifdef __cplusplus
}
#endif

#endif /* __NUTTX_DRIVERS_INCLUDE_APP_LOG_H */
