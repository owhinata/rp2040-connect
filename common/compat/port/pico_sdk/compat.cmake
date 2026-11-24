# SPDX-License-Identifier: MIT
# The Zephyr compatibility layer of a build on the Pico SDK with the generic
# kernel part (the FreeRTOS and ThreadX builds), the shell, the RAM log and
# the shared sources they build (common/).  The build's CMakeLists.txt
# sets, before it include()s this file:
#
#   PICO_SDK_OS               the name of the OS's directories: its part of
#                             the layer, common/compat/port/<PICO_SDK_OS>
#                             (with the port header in rp2040/), and the
#                             drivers' bindings,
#                             common/drivers/<driver>/port/<PICO_SDK_OS>
#                             (freertos, threadx)
#   PICO_SDK_STMEMSC_DIR      ST's register driver of the LSM6DSO, fetched by
#                             the build's setup.sh (lsm6dso.cmake)
#
# then calls
#
#   pico_sdk_zephyr_compat(<target>)
#
# after add_executable().  It adds to the target:
#
# - the kernel part of the layer, the generic one for an OS that offers
#   only basic primitives (common/compat/kernel_generic: zephyr_kernel.c,
#   zephyr_mutex.c) on the OS's primitives (its zephyr_kernel_os_impl.h),
#   the realtime clock of <zephyr/sys/clock.h> on the RP2040 TIMER
#   (common/compat/port/pico_sdk/zephyr_sys_clock.c, which newlib's
#   gettimeofday() and time() read too) and the
#   OS-independent part that links at this stage (common/compat:
#   zephyr_init.c, on the kernel part's start of a thread, right after it;
#   k_heap, zephyr_heap.c, after the mutexes; zephyr_work.c, zephyr_rtio.c,
#   zephyr_rtio_workq.c, zephyr_mem_blocks.c);
# - the RAM log (common/compat/log/log_ram.c, on common/compat/log/log_ring.c
#   and the Pico SDK part of its boundary,
#   common/compat/port/pico_sdk/log_ram_os_impl.h) and "dmesg"
#   (common/shell/dmesg_shell.c);
# - the shell: the shell engine for an OS without a shell of its own
#   (common/compat/shell_engine, on the layer: the line editor, the command
#   table and its execution with "help", the copy of Zephyr's word split,
#   the consoles, the getopt state of each session, the backend of
#   common/shell/zshell.c and the serial session; the Telnet shell is
#   telnet.cmake's), common/shell/subcmd.c, the copies of Zephyr's
#   fnmatch() and getopt in common/compat, and the console transport on the
#   Pico SDK (common/compat/port/pico_sdk: shell_serial_os_impl.h on the
#   SDK's stdio_usb, the SDK's stdio on the consoles, shell_stdio.c;
#   newlib's glue, stdio_wrap.c: the console streams,
#   -Wl,--wrap=...); the types and constants of the network headers newlib
#   lacks, <sys/socket.h>, <netinet/in.h>, <netinet/tcp.h>, <arpa/inet.h>,
#   <poll.h>, <netdb.h>, <sys/uio.h>, and inet_pton() and inet_ntop()
#   (common/compat/libc_net, include/ and inet.c: the layer's
#   <zephyr/net/socket.h> is on them); malloc() is the Pico SDK's, with its
#   mutex (pico_malloc, PICO_USE_MALLOC_MUTEX with pico_multicore when the
#   kernel's port links it);
# - the layer's socket part, Zephyr's socket layer down to its list of
#   socket implementations (<zephyr/net/socket.h>, <zephyr/net/socket_offload.h>,
#   <zephyr/sys/fdtable.h>), OS-independent, for an OS without a network
#   stack of its own (common/compat/net_offload): reduced copies of Zephyr's
#   subsys/net/lib/sockets/sockets.c, getaddrinfo.c and socket_offload.c and
#   of lib/os/zvfs/zvfs_fdtable.c and zvfs_poll.c (v4.5.0, see their
#   headers); here, the linker script's section of the registered
#   implementations (ld/section_extra_post_text.incl,
#   pico_add_linker_script_override_path()); the NINA's sockets register
#   themselves (nina.cmake);
# - the include directories: the OS's part first (common/compat/port/<OS>/
#   rp2040: its port header zephyr_os_port.h; common/compat/port/<OS>:
#   zephyr_kernel_os_impl.h, priorities_os.h), the part shared by the OSes
#   on the Pico SDK (common/compat/port/pico_sdk: the port header's
#   zephyr_os_port_rp2040.h, the console transport's shell_serial_os_impl.h
#   and shell_stdio.h, log_ram_os_impl.h), the layer's parts for an OS without
#   the model of its own (common/compat/port/generic: <zephyr/drivers/i2c.h>,
#   <zephyr/sys/clock.h>, <zephyr/sys/__assert.h>, app_log_os.h, the generic
#   kernel part's zephyr_os.h and default zephyr_os_atomic.h, the shell
#   engine's zephyr_os_shell.h, ...), common/compat/net_offload/include (the
#   network part's Zephyr headers, before common/compat/include: its
#   <zephyr/net/net_ip.h> takes the place of the layer's),
#   common/compat/libc_net/include (headers newlib does not have, so none
#   hides one of newlib's), the layer's headers (common/compat/include: the
#   Zephyr headers and the others; internal; port: the boundaries) and the
#   shell's (common/shell/include: subcmd.h, ps_interval.h; port: its
#   boundaries zshell_os.h and kernel_shell_os.h; port/<OS>: the OS's
#   kernel_shell_os_impl.h; port/generic: zshell_os_impl.h, the shell
#   engine's backends of zshell.c, and sensor_shell_os.h);
# - the definitions every source sees: Z_SHELL_CMD_ROOT_GLOBAL (a
#   SHELL_CMD_REGISTER() makes the root z_shell_root_<syntax> for the
#   application's table, shell_engine.h), CONFIG_GETOPT_LONG=1 (Zephyr's
#   option, which changes struct sys_getopt_state), PICO_STDOUT_MUTEX=0
#   (each console has its own, shell_stdio.h);
# - the prelude of the shared sources (common/compat/port/generic/common_prelude.h,
#   PICO_SDK_PRELUDE: -include on each source under common/ only, as the
#   NuttX build does; not on the network part, common/compat/net_offload
#   and common/compat/libc_net, nor on the shell engine,
#   common/compat/shell_engine, which are written without it).
#
# The application provides the OS's configuration with a thread pointer of
# the layer's (FreeRTOSConfig.h's configNUM_THREAD_LOCAL_STORAGE_POINTERS >= 1,
# tx_user.h's TX_THREAD_USER_EXTENSION), a tick hook that calls
# zephyr_compat_tick() (common/compat/include/zephyr_compat_os.h),
# the table of commands
# (shell_engine_cmds[]), and calls zephyr_compat_initialize()
# (zephyr_compat.h) before the scheduler starts.

foreach(var PICO_SDK_OS PICO_SDK_STMEMSC_DIR)
  if(NOT DEFINED ${var})
    message(FATAL_ERROR "${var} must be set before ${CMAKE_CURRENT_LIST_FILE} is included")
  endif()
endforeach()

get_filename_component(PICO_SDK_COMPAT_PORT_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
get_filename_component(PICO_SDK_COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)
set(PICO_SDK_COMPAT_OS_DIR ${PICO_SDK_COMMON_DIR}/compat/port/${PICO_SDK_OS})
set(PICO_SDK_NET_OFFLOAD_DIR ${PICO_SDK_COMMON_DIR}/compat/net_offload)
set(PICO_SDK_LIBC_NET_DIR ${PICO_SDK_COMMON_DIR}/compat/libc_net)
set(PICO_SDK_SHELL_ENGINE_DIR ${PICO_SDK_COMMON_DIR}/compat/shell_engine)
set(PICO_SDK_KERNEL_GENERIC_DIR ${PICO_SDK_COMMON_DIR}/compat/kernel_generic)
set(PICO_SDK_COMPAT_GENERIC_DIR ${PICO_SDK_COMMON_DIR}/compat/port/generic)
set(PICO_SDK_PRELUDE ${PICO_SDK_COMPAT_GENERIC_DIR}/common_prelude.h)

function(pico_sdk_zephyr_compat target)
  # The work queues' threads, the system work queue, zephyr_compat_initialize()
  # and k_uptime_get(), linked right after zephyr_kernel.c, where they were.
  set(common_kernel_sources
    ${PICO_SDK_COMMON_DIR}/compat/zephyr_init.c
  )

  # k_heap, linked after the mutexes, where it was.
  set(common_heap_sources
    ${PICO_SDK_COMMON_DIR}/compat/zephyr_heap.c
  )

  # The RAM log and "dmesg".
  set(common_log_sources
    ${PICO_SDK_COMMON_DIR}/compat/log/log_ram.c
    ${PICO_SDK_COMMON_DIR}/shell/dmesg_shell.c
  )

  set(common_sources
    ${PICO_SDK_COMMON_DIR}/compat/zephyr_work.c
    ${PICO_SDK_COMMON_DIR}/compat/zephyr_rtio.c
    ${PICO_SDK_COMMON_DIR}/compat/zephyr_rtio_workq.c
    ${PICO_SDK_COMMON_DIR}/compat/zephyr_mem_blocks.c
    ${PICO_SDK_COMMON_DIR}/compat/log/log_ring.c
    ${PICO_SDK_COMMON_DIR}/shell/zshell.c
    ${PICO_SDK_COMMON_DIR}/shell/subcmd.c
  )

  # Copies of Zephyr's files (v4.5.0, unchanged; common/tests/run.sh
  # compares them with the Zephyr tree).
  set(zephyr_copies
    ${PICO_SDK_COMMON_DIR}/compat/subsys/portability/posix/c_lib_ext/fnmatch.c
    ${PICO_SDK_COMMON_DIR}/compat/lib/utils/getopt/getopt.c
    ${PICO_SDK_COMMON_DIR}/compat/lib/utils/getopt/getopt_long.c
  )

  # Reduced copies of Zephyr's files (v4.5.0; what was left out is in each
  # header).  Their loops compare an int with ARRAY_SIZE() as Zephyr's do.
  set(zephyr_socket_copies
    ${PICO_SDK_NET_OFFLOAD_DIR}/lib/os/zvfs/zvfs_fdtable.c
    ${PICO_SDK_NET_OFFLOAD_DIR}/lib/os/zvfs/zvfs_poll.c
    ${PICO_SDK_NET_OFFLOAD_DIR}/subsys/net/lib/sockets/sockets.c
    ${PICO_SDK_NET_OFFLOAD_DIR}/subsys/net/lib/sockets/getaddrinfo.c
    ${PICO_SDK_NET_OFFLOAD_DIR}/subsys/net/lib/sockets/socket_offload.c
  )

  target_sources(${target} PRIVATE
    ${PICO_SDK_KERNEL_GENERIC_DIR}/zephyr_kernel.c
    ${common_kernel_sources}
    ${PICO_SDK_KERNEL_GENERIC_DIR}/zephyr_mutex.c
    ${common_heap_sources}
    ${PICO_SDK_COMPAT_PORT_DIR}/zephyr_sys_clock.c
    ${zephyr_socket_copies}
    ${common_log_sources}
    ${PICO_SDK_SHELL_ENGINE_DIR}/cli.c
    ${PICO_SDK_SHELL_ENGINE_DIR}/getopt_state.c
    ${PICO_SDK_SHELL_ENGINE_DIR}/shell_console.c
    ${PICO_SDK_SHELL_ENGINE_DIR}/shell_engine.c
    ${PICO_SDK_COMPAT_PORT_DIR}/shell_stdio.c
    ${PICO_SDK_SHELL_ENGINE_DIR}/shell_argv.c
    ${PICO_SDK_SHELL_ENGINE_DIR}/shell_serial.c
    ${PICO_SDK_COMPAT_PORT_DIR}/stdio_wrap.c
    ${PICO_SDK_LIBC_NET_DIR}/inet.c
    ${common_sources}
    ${zephyr_copies}
  )

  target_include_directories(${target} PRIVATE
    ${PICO_SDK_COMPAT_OS_DIR}/rp2040
    ${PICO_SDK_COMPAT_OS_DIR}
    ${PICO_SDK_COMPAT_PORT_DIR}
    ${PICO_SDK_COMPAT_GENERIC_DIR}
    ${PICO_SDK_NET_OFFLOAD_DIR}/include
    ${PICO_SDK_LIBC_NET_DIR}/include
    ${PICO_SDK_COMMON_DIR}/compat/include
    ${PICO_SDK_COMMON_DIR}/compat/port
    ${PICO_SDK_COMMON_DIR}/compat/internal
    ${PICO_SDK_COMMON_DIR}/shell/include
    ${PICO_SDK_COMMON_DIR}/shell/port
    ${PICO_SDK_COMMON_DIR}/shell/port/${PICO_SDK_OS}
    ${PICO_SDK_COMMON_DIR}/shell/port/generic
    ${PICO_SDK_COMMON_DIR}/compat/lib/utils/getopt
  )

  target_compile_definitions(${target} PRIVATE
    Z_SHELL_CMD_ROOT_GLOBAL
    CONFIG_GETOPT_LONG=1
    PICO_STDOUT_MUTEX=0
  )

  # stdio_wrap.c
  target_link_options(${target} PRIVATE
    -Wl,--wrap=fputs -Wl,--wrap=fwrite -Wl,--wrap=fputc -Wl,--wrap=putc
    -Wl,--wrap=fprintf -Wl,--wrap=vfprintf -Wl,--wrap=fflush
  )

  pico_add_linker_script_override_path(${target} ${PICO_SDK_COMPAT_PORT_DIR}/ld
    FILES section_extra_post_text.incl
  )

  set_source_files_properties(${zephyr_socket_copies} PROPERTIES
    COMPILE_OPTIONS "-Wno-sign-compare"
  )

  # Source file properties are scoped to the directory that calls this
  # function, the application's, which is the target's.
  set_source_files_properties(${common_kernel_sources} ${common_heap_sources}
    ${common_log_sources} ${common_sources} ${zephyr_copies} PROPERTIES
    COMPILE_OPTIONS "-include;${PICO_SDK_PRELUDE}"
  )
endfunction()
