/* SPDX-License-Identifier: Apache-2.0 */
/* The part of Zephyr's <zephyr/shell/shell.h> (Zephyr v4.5.0) that the
 * port's commands use, for command handlers ported with their Zephyr
 * signature
 *
 *   int handler(const struct shell *sh, size_t argc, char **argv)
 *
 * Part of the small Zephyr compatibility layer of the port: its
 * OS-independent part (common/compat; see common/compat/README.md), with
 * the command handling in common/shell/zshell.c.
 *
 * Output: shell_fprintf() and shell_print(), shell_info(), shell_warn(),
 * shell_error() (each of the last four adds "\n", as Zephyr's).
 *   - sh == NULL: the console of the task that calls it, where the port's
 *     other commands write (nuttx/apps/kv): stdout for shell_print() and
 *     shell_info(), stderr for shell_warn() and shell_error() (Zephyr
 *     prints the last ones in yellow and red).  The commands whose sets are
 *     written with common/shell/include/subcmd.h pass NULL.
 *   - sh != NULL: a shell backend, the terminal of a shell session (on
 *     NuttX the nsh session's /dev/ttyACM0 or /dev/telnet0), which the
 *     commands registered with SHELL_CMD_REGISTER() below get as sh.  As in
 *     Zephyr the object outlives the command: a thread that keeps it (the
 *     sensor shell's processing thread, nuttx/apps/sensor) writes to that
 *     terminal later, after the command has returned to the prompt.  The
 *     output to a backend is the OS's z_shell_backend_vfprintf() (below).
 *
 * Commands: SHELL_CMD_REGISTER() with SHELL_STATIC_SUBCMD_SET_CREATE(),
 * SHELL_DYNAMIC_CMD_CREATE(), SHELL_CMD_ARG(), SHELL_CMD(),
 * SHELL_COND_CMD(), SHELL_EXPR_CMD_ARG(), SHELL_HELP() and
 * SHELL_SUBCMD_SET_END make the command's entry: main() (on NuttX the
 * builtin application's, when the source is its MAINSRC), else
 * z_shell_main_<syntax>() when the source is compiled with
 * Z_SHELL_CMD_NAMED_MAIN (below).  The entry runs the command with the
 * Zephyr shell's word matching, wildcards, argument count check and help
 * (common/shell/zshell.c).  Differences from Zephyr: the subcommand set of
 * an entry is a struct shell_cmd_entry that says whether it is dynamic
 * (Zephyr: union shell_cmd_entry in a linker section of its own); no
 * select mode, aliases, remote commands, raw arguments or Tab completion
 * (on NuttX, nsh's).
 *
 * shell_hexdump() and shell_hexdump_line() are Zephyr's, with its output.
 * shell_device_get_binding() and shell_device_filter() are Zephyr's, on
 * the devices of <zephyr/device.h>.
 *
 * The OS part: "zephyr_os_shell.h", from the OS's directory of the layer
 * (common/compat/port/nuttx on NuttX), defines struct shell, a shell backend
 * (Zephyr: the instance of a shell transport), with whatever the OS's
 * backend needs to find the terminal of a session and to tell output from
 * the session's running command from output of other threads.  The OS
 * implements z_shell_backend_vfprintf() (below) and the boundary of
 * common/shell/port/zshell_os.h.
 */

#ifndef COMMON_COMPAT_INCLUDE_ZEPHYR_SHELL_SHELL_H
#define COMMON_COMPAT_INCLUDE_ZEPHYR_SHELL_SHELL_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/sys/util.h>

/* A shell backend (Zephyr: the instance of a shell transport): struct
 * shell of the OS part.
 */

#include "zephyr_os_shell.h"

/* zephyr/shell/shell_types.h */

enum shell_vt100_color
{
  SHELL_VT100_COLOR_DEFAULT,
  SHELL_VT100_COLOR_BLACK,
  SHELL_VT100_COLOR_RED,
  SHELL_VT100_COLOR_GREEN,
  SHELL_VT100_COLOR_YELLOW,
  SHELL_VT100_COLOR_BLUE,
  SHELL_VT100_COLOR_MAGENTA,
  SHELL_VT100_COLOR_CYAN,
  SHELL_VT100_COLOR_WHITE,
  VT100_COLOR_END
};

#define SHELL_NORMAL   SHELL_VT100_COLOR_DEFAULT
#define SHELL_INFO     SHELL_VT100_COLOR_GREEN
#define SHELL_OPTION   SHELL_VT100_COLOR_CYAN
#define SHELL_WARNING  SHELL_VT100_COLOR_YELLOW
#define SHELL_ERROR    SHELL_VT100_COLOR_RED

/* The output to a backend (the OS's part of zshell.c, see
 * common/shell/port/zshell_os.h): COLOR is SHELL_NORMAL, SHELL_INFO,
 * SHELL_WARNING or SHELL_ERROR as the macros below pass it.  Weak, so that
 * the sources that only pass NULL link without zshell.c.
 */

void z_shell_backend_vfprintf(const struct shell *sh,
                              enum shell_vt100_color color,
                              const char *fmt, va_list args)
  __attribute__((weak));

static inline void shell_vfprintf(const struct shell *sh,
                                  enum shell_vt100_color color,
                                  const char *fmt, va_list args)
{
  if (sh != NULL && z_shell_backend_vfprintf != NULL)
    {
      z_shell_backend_vfprintf(sh, color, fmt, args);
      return;
    }

  vfprintf(color == SHELL_WARNING || color == SHELL_ERROR ? stderr : stdout,
           fmt, args);
}

/* shell_fprintf() under a second name, which the macros below use: a
 * source may define shell_fprintf() as a macro of its own after this
 * header (nuttx/apps/wifi/wifi_credentials_shell.c).
 */

static inline __attribute__((format(printf, 3, 4)))
void z_shell_fprintf_(const struct shell *sh, enum shell_vt100_color color,
                      const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  shell_vfprintf(sh, color, fmt, args);
  va_end(args);
}

static inline __attribute__((format(printf, 3, 4)))
void shell_fprintf(const struct shell *sh, enum shell_vt100_color color,
                   const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  shell_vfprintf(sh, color, fmt, args);
  va_end(args);
}

#define shell_fprintf_normal(sh, fmt, ...) \
  z_shell_fprintf_(sh, SHELL_NORMAL, fmt, ##__VA_ARGS__)
#define shell_fprintf_info(sh, fmt, ...) \
  z_shell_fprintf_(sh, SHELL_INFO, fmt, ##__VA_ARGS__)
#define shell_fprintf_warn(sh, fmt, ...) \
  z_shell_fprintf_(sh, SHELL_WARNING, fmt, ##__VA_ARGS__)
#define shell_fprintf_error(sh, fmt, ...) \
  z_shell_fprintf_(sh, SHELL_ERROR, fmt, ##__VA_ARGS__)

#define shell_print(sh, fmt, ...) \
  shell_fprintf_normal(sh, fmt "\n", ##__VA_ARGS__)
#define shell_info(sh, fmt, ...) \
  shell_fprintf_info(sh, fmt "\n", ##__VA_ARGS__)
#define shell_warn(sh, fmt, ...) \
  shell_fprintf_warn(sh, fmt "\n", ##__VA_ARGS__)
#define shell_error(sh, fmt, ...) \
  shell_fprintf_error(sh, fmt "\n", ##__VA_ARGS__)

/* Command sets ------------------------------------------------------------*/

#define SHELL_OPT_ARG_RAW        (0xFE)
#define SHELL_OPT_ARG_CHECK_SKIP (0xFF)
#define SHELL_OPT_ARG_MAX        (0xFD)

/* SHELL_CMD_HELP_PRINTED: a handler's value after printing its help. */

#define SHELL_CMD_HELP_PRINTED   (1)

struct shell_static_args
{
  uint8_t mandatory;   /* number of mandatory arguments, with the command */
  uint8_t optional;    /* number of optional arguments */
};

struct shell_static_entry;

typedef void (*shell_dynamic_get)(size_t idx,
                                  struct shell_static_entry *entry);
typedef int (*shell_cmd_handler)(const struct shell *sh, size_t argc,
                                 char **argv);

/* Zephyr's union shell_cmd_entry, with the kind that Zephyr reads from the
 * linker section the set is in.
 */

struct shell_cmd_entry
{
  bool is_dynamic;
  union
    {
      shell_dynamic_get dynamic_get;              /* dynamic set */
      const struct shell_static_entry *entry;     /* static set */
    };
};

struct shell_static_entry
{
  const char *syntax;
  const char *help;
  const struct shell_cmd_entry *subcmd;
  shell_cmd_handler handler;
  struct shell_static_args args;
};

/* SHELL_HELP(): a help text with a separate usage line. */

#define SHELL_STRUCTURED_HELP_MAGIC 0x86D20BC4

struct shell_cmd_help
{
  uint32_t magic;
  const char *description;
  const char *usage;
};

#define SHELL_HELP(_description, _usage) \
  ((const char *)&(const struct shell_cmd_help){ \
    .magic = SHELL_STRUCTURED_HELP_MAGIC, \
    .description = (_description), \
    .usage = (_usage), \
  })

static inline bool shell_help_is_structured(const char *help)
{
  const struct shell_cmd_help *structured =
    (const struct shell_cmd_help *)help;

  return structured != NULL &&
         ((uintptr_t)structured % __alignof__(struct shell_cmd_help)) == 0 &&
         structured->magic == SHELL_STRUCTURED_HELP_MAGIC;
}

#define SHELL_EXPR_CMD_ARG(_expr, _syntax, _subcmd, _help, _handler, \
                           _mand, _opt) \
  { \
    .syntax = (_expr) ? (const char *)STRINGIFY(_syntax) : "", \
    .help = (_expr) ? (const char *)(_help) : NULL, \
    .subcmd = (const struct shell_cmd_entry *)((_expr) ? (_subcmd) : NULL), \
    .handler = (shell_cmd_handler)((_expr) ? (_handler) : NULL), \
    .args = { .mandatory = (_mand), .optional = (_opt) }, \
  }

#define SHELL_CMD_ARG(syntax, subcmd, help, handler, mand, opt) \
  SHELL_EXPR_CMD_ARG(1, syntax, subcmd, help, handler, mand, opt)
#define SHELL_CMD(_syntax, _subcmd, _help, _handler) \
  SHELL_CMD_ARG(_syntax, _subcmd, _help, _handler, 0, 0)
#define SHELL_COND_CMD_ARG(_flag, _syntax, _subcmd, _help, _handler, \
                           _mand, _opt) \
  SHELL_EXPR_CMD_ARG(IS_ENABLED(_flag), _syntax, _subcmd, _help, _handler, \
                     _mand, _opt)
#define SHELL_COND_CMD(_flag, _syntax, _subcmd, _help, _handler) \
  SHELL_COND_CMD_ARG(_flag, _syntax, _subcmd, _help, _handler, 0, 0)

#define SHELL_SUBCMD_SET_END { 0 }

#define SHELL_STATIC_SUBCMD_SET_CREATE(name, ...) \
  static const struct shell_static_entry shell_##name[] = \
    { \
      __VA_ARGS__ \
    }; \
  static const struct shell_cmd_entry name = \
    { \
      .is_dynamic = false, \
      .entry = shell_##name, \
    }

#define SHELL_DYNAMIC_CMD_CREATE(name, get) \
  static const struct shell_cmd_entry name = \
    { \
      .is_dynamic = true, \
      .dynamic_get = (get), \
    }

/* Runs the root command ROOT with the words of main() (ARGV[0] is the
 * command's name) as the Zephyr shell runs a command line, on the backend
 * of the calling task's terminal.  Returns the exit status: EXIT_SUCCESS
 * when the handler returned 0, else EXIT_FAILURE.  common/shell/zshell.c.
 */

int z_shell_cmd_main(const struct shell_static_entry *root, int argc,
                     char **argv);

/* SHELL_CMD_ARG_REGISTER() / SHELL_CMD_REGISTER(): the command's entry,
 * int Z_SHELL_CMD_MAIN(syntax)(int argc, char *argv[]), which runs it with
 * z_shell_cmd_main().  By default the entry is main() (on NuttX the
 * builtin application's, for the source that is the application's
 * MAINSRC).  A source that is not (its build defines
 * Z_SHELL_CMD_NAMED_MAIN for it) gets
 * z_shell_main_<syntax>() instead, which the application's main() calls
 * (common/imu/time_sync.c: z_shell_main_ntp(), nuttx/apps/ntp/ntp_main.c).
 */

#ifdef Z_SHELL_CMD_NAMED_MAIN
#  define Z_SHELL_CMD_MAIN(syntax) z_shell_main_##syntax
#else
#  define Z_SHELL_CMD_MAIN(syntax) main
#endif

/* With Z_SHELL_CMD_ROOT_GLOBAL (the FreeRTOS build defines it for every
 * source) the registration makes no entry function: the root entry itself,
 * const struct shell_static_entry z_shell_root_<syntax>, is global, and the
 * build's table of commands lists it (the OS's shell runs it with
 * z_shell_cmd_main()).
 */

#ifdef Z_SHELL_CMD_ROOT_GLOBAL
#define SHELL_CMD_ARG_REGISTER(syntax, subcmd, help, handler, mandatory, \
                               optional) \
  extern const struct shell_static_entry z_shell_root_##syntax; \
  const struct shell_static_entry z_shell_root_##syntax = \
    SHELL_CMD_ARG(syntax, subcmd, help, handler, mandatory, optional)
#else
#define SHELL_CMD_ARG_REGISTER(syntax, subcmd, help, handler, mandatory, \
                               optional) \
  static const struct shell_static_entry z_shell_root_##syntax = \
    SHELL_CMD_ARG(syntax, subcmd, help, handler, mandatory, optional); \
  int Z_SHELL_CMD_MAIN(syntax)(int argc, char *argv[]); \
  int Z_SHELL_CMD_MAIN(syntax)(int argc, char *argv[]) \
  { \
    return z_shell_cmd_main(&z_shell_root_##syntax, argc, argv); \
  }
#endif

#define SHELL_CMD_REGISTER(syntax, subcmd, help, handler) \
  SHELL_CMD_ARG_REGISTER(syntax, subcmd, help, handler, 0, 0)

/* SHELL_SUBCMD_ADD((parent), syntax, subcmd, help, handler, mand, opt):
 * Zephyr puts the entry in a linker section of the parent command, from
 * which SHELL_SUBCMD_SET_CREATE(name, (parent)) makes the parent's set,
 * sorted by name, so that subcommands can come from several sources.  Here
 * it defines the entry as the global
 *
 *   const struct shell_static_entry z_shell_subcmd_<parent>_<syntax>
 *
 * and the source that makes the parent's set lists the entries itself, in
 * Zephyr's order (one parent level only; common/compat/zephyr_wifi_shell.c
 * for "wifi").
 */

#define Z_SHELL_UNPAREN(...) __VA_ARGS__
#define Z_SHELL_SUBCMD_NAME_(parent, syntax) z_shell_subcmd_##parent##_##syntax
#define Z_SHELL_SUBCMD_NAME(parent, syntax) Z_SHELL_SUBCMD_NAME_(parent, syntax)

#define SHELL_SUBCMD_ADD(_parent, _syntax, _subcmd, _help, _handler, _mand, \
                         _opt) \
  extern const struct shell_static_entry \
    Z_SHELL_SUBCMD_NAME(Z_SHELL_UNPAREN _parent, _syntax); \
  const struct shell_static_entry \
    Z_SHELL_SUBCMD_NAME(Z_SHELL_UNPAREN _parent, _syntax) = \
    SHELL_CMD_ARG(_syntax, _subcmd, _help, _handler, _mand, _opt)

/* Zephyr's shell_help(sh): prints the help of the command that runs (the
 * deepest one with a handler), as a wrong argument count does.  From the
 * handler only, in its task.  common/shell/zshell.c, where the OS's part
 * keeps the running command (ZSHELL_OS_ACTIVE_CMD, common/shell/port/zshell_os.h;
 * the FreeRTOS build).
 */

void shell_help(const struct shell *sh);

/* shell_hexdump(): DATA as lines of SHELL_HEXDUMP_BYTES_IN_LINE bytes, each
 * the offset, the bytes in hex and as text, in Zephyr's format
 * (subsys/shell/shell_utils.c); shell_hexdump_line() prints one line.
 * common/shell/zshell.c.
 */

#define SHELL_HEXDUMP_BYTES_IN_LINE 16

void shell_hexdump_line(const struct shell *sh, unsigned int offset,
                        const uint8_t *data, size_t len);
void shell_hexdump(const struct shell *sh, const uint8_t *data, size_t len);

/* Devices (common/shell/zshell.c, on the devices of <zephyr/device.h>). */

struct device;
typedef bool (*shell_device_filter_t)(const struct device *dev);

/* The IDX-th ready device with a name that FILTER (if not NULL) accepts. */

const struct device *shell_device_filter(size_t idx,
                                         shell_device_filter_t filter);

/* device_get_binding(NAME), else the device with the devicetree node label
 * NAME (CONFIG_DEVICE_DT_METADATA of the Zephyr build).
 */

const struct device *shell_device_get_binding(const char *name);

#endif /* COMMON_COMPAT_INCLUDE_ZEPHYR_SHELL_SHELL_H */
