# SPDX-License-Identifier: MIT
# The FreeRTOS implementation of the Zephyr compatibility layer
# (freertos/drivers/compat), the shell, the RAM log and the shared sources
# they build (common/), for an application of freertos/app.  include() this
# file from the application's CMakeLists.txt, then call
#
#   freertos_zephyr_compat(<target> [SELFTEST])
#
# after add_executable().  It adds to the target:
#
# - the FreeRTOS part of the layer (zephyr_kernel.c, zephyr_mutex.c) and the
#   OS-independent part that links at this stage (common/compat:
#   zephyr_work.c, zephyr_rtio.c, zephyr_rtio_workq.c, zephyr_mem_blocks.c);
# - the RAM log and "dmesg" (freertos/drivers/log/log_ram.c, on
#   common/log/log_ring.c);
# - the shell (freertos/drivers/shell: consoles, the stdio of each task, the
#   line editor, the command table and the backend of common/shell/zshell.c;
#   common/shell/subcmd.c; the copies of Zephyr's fnmatch() and getopt in
#   common/compat) and newlib's glue (freertos/drivers/libc: the console
#   streams, -Wl,--wrap=...; the types and constants of the network
#   headers newlib lacks, <sys/socket.h>, <netinet/in.h>, <netinet/tcp.h>,
#   <arpa/inet.h>, <poll.h>, <netdb.h>, <sys/uio.h>, and inet_pton() and
#   inet_ntop(), inet.c: the layer's <zephyr/net/socket.h> is on them, its
#   socket calls are the NINA's, freertos/drivers/nina/nina.cmake);
#   malloc() is the Pico SDK's, with its mutex
#   (pico_malloc, PICO_USE_MALLOC_MUTEX with pico_multicore, which the
#   kernel's port links);
# - with SELFTEST, the layer's self-test (zephyr_selftest.c and the board's
#   part, rp2040/zephyr_selftest_port.c) and NANO_SELFTEST=1;
# - the include directories, the board's port header first
#   (rp2040/zephyr_os_port.h), then the FreeRTOS part (zephyr_os.h, ...),
#   freertos/drivers/shell, freertos/drivers/include (app_log_os.h),
#   freertos/drivers/libc/include (headers newlib does not have, so none
#   hides one of newlib's), common/compat (the Zephyr headers),
#   common/log and common/shell;
# - the definitions every source sees: Z_SHELL_CMD_ROOT_GLOBAL (a
#   SHELL_CMD_REGISTER() makes the root z_shell_root_<syntax> for the
#   application's table, nano_shell.h), CONFIG_GETOPT_LONG=1 (Zephyr's
#   option, which changes struct sys_getopt_state), PICO_STDOUT_MUTEX=0
#   (each console has its own, nano_stdio.h);
# - the prelude of the shared sources (common_prelude.h, -include on each
#   source under common/ only, as the NuttX build does).
#
# The application provides FreeRTOSConfig.h (with
# configNUM_THREAD_LOCAL_STORAGE_POINTERS >= 1), priorities_os.h and
# os_clock.h on its include path, a tick hook that calls
# zephyr_compat_tick() (zephyr_compat_os.h), the table of commands
# (nano_shell_cmds[]), and calls zephyr_compat_initialize()
# (zephyr_compat.h) before the scheduler starts.

get_filename_component(FREERTOS_COMPAT_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
get_filename_component(FREERTOS_DRIVERS_DIR ${CMAKE_CURRENT_LIST_DIR}/.. ABSOLUTE)
get_filename_component(FREERTOS_COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../common ABSOLUTE)

function(freertos_zephyr_compat target)
  cmake_parse_arguments(ARG "SELFTEST" "" "" ${ARGN})

  set(common_sources
    ${FREERTOS_COMMON_DIR}/compat/zephyr_work.c
    ${FREERTOS_COMMON_DIR}/compat/zephyr_rtio.c
    ${FREERTOS_COMMON_DIR}/compat/zephyr_rtio_workq.c
    ${FREERTOS_COMMON_DIR}/compat/zephyr_mem_blocks.c
    ${FREERTOS_COMMON_DIR}/log/log_ring.c
    ${FREERTOS_COMMON_DIR}/shell/zshell.c
    ${FREERTOS_COMMON_DIR}/shell/subcmd.c
  )

  # Copies of Zephyr's files (v4.5.0, unchanged; common/tests/run.sh
  # compares them with the Zephyr tree).
  set(zephyr_copies
    ${FREERTOS_COMMON_DIR}/compat/subsys/portability/posix/c_lib_ext/fnmatch.c
    ${FREERTOS_COMMON_DIR}/compat/lib/utils/getopt/getopt.c
    ${FREERTOS_COMMON_DIR}/compat/lib/utils/getopt/getopt_long.c
  )

  target_sources(${target} PRIVATE
    ${FREERTOS_COMPAT_DIR}/zephyr_kernel.c
    ${FREERTOS_COMPAT_DIR}/zephyr_mutex.c
    ${FREERTOS_DRIVERS_DIR}/log/log_ram.c
    ${FREERTOS_DRIVERS_DIR}/shell/cli.c
    ${FREERTOS_DRIVERS_DIR}/shell/getopt_state.c
    ${FREERTOS_DRIVERS_DIR}/shell/nano_console.c
    ${FREERTOS_DRIVERS_DIR}/shell/nano_shell.c
    ${FREERTOS_DRIVERS_DIR}/shell/nano_stdio.c
    ${FREERTOS_DRIVERS_DIR}/shell/shell_argv.c
    ${FREERTOS_DRIVERS_DIR}/libc/stdio_wrap.c
    ${FREERTOS_DRIVERS_DIR}/libc/inet.c
    ${common_sources}
    ${zephyr_copies}
  )

  if(ARG_SELFTEST)
    target_sources(${target} PRIVATE
      ${FREERTOS_COMPAT_DIR}/zephyr_selftest.c
      ${FREERTOS_COMPAT_DIR}/rp2040/zephyr_selftest_port.c
    )
    target_compile_definitions(${target} PRIVATE NANO_SELFTEST=1)
  endif()

  target_include_directories(${target} PRIVATE
    ${FREERTOS_COMPAT_DIR}/rp2040
    ${FREERTOS_COMPAT_DIR}
    ${FREERTOS_DRIVERS_DIR}/shell
    ${FREERTOS_DRIVERS_DIR}/include
    ${FREERTOS_DRIVERS_DIR}/libc/include
    ${FREERTOS_COMMON_DIR}/compat
    ${FREERTOS_COMMON_DIR}/log
    ${FREERTOS_COMMON_DIR}/shell
    ${FREERTOS_COMMON_DIR}/compat/lib/utils/getopt
  )

  target_compile_definitions(${target} PRIVATE
    Z_SHELL_CMD_ROOT_GLOBAL
    CONFIG_GETOPT_LONG=1
    PICO_STDOUT_MUTEX=0
  )

  # freertos/drivers/libc/stdio_wrap.c
  target_link_options(${target} PRIVATE
    -Wl,--wrap=fputs -Wl,--wrap=fwrite -Wl,--wrap=fputc -Wl,--wrap=putc
    -Wl,--wrap=fprintf -Wl,--wrap=vfprintf -Wl,--wrap=fflush
  )

  # Source file properties are scoped to the directory that calls this
  # function, the application's, which is the target's.
  set_source_files_properties(${common_sources} ${zephyr_copies} PROPERTIES
    COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h"
  )
endfunction()
