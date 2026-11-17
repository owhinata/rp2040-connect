# SPDX-License-Identifier: MIT
# The SPI link to the NINA-W102 module on the FreeRTOS build, the "nina"
# command, and the module's sockets and name resolution.  include() this
# file from the application's CMakeLists.txt, then call
#
#   freertos_nina(<target> [READY_IRQ])
#
# after freertos_zephyr_compat(<target>) (freertos/drivers/compat/compat.cmake,
# whose layer and shell it needs).  It adds to the target:
#
# - the shared core of the link (common/drivers/nina/nina_spi.c:
#   nina_request() and nina_ip_config(), the WiFiNINA SPI protocol with the
#   Zephyr build's framing, timeouts and lazy reset of the module) and the
#   "nina version" command (common/drivers/nina/nina_shell.c, the Zephyr
#   build's text);
# - the sockets: the shared socket table and commands
#   (common/drivers/nina/nina_sock.c) and resolver (nina_dns.c), and the
#   zsock_*() calls of the layer's <zephyr/net/socket.h> on them
#   (nina_socket.c: the descriptors, the Zephyr build's wait loops), which
#   the time synchronization, the MQTT transport, Telnet and the speed test
#   use; nothing calls them yet, so --gc-sections leaves most out;
# - the two boundaries of the core on FreeRTOS: nina_os.c (nina_os.h: the
#   locks, sleeps and clock on the Zephyr compatibility layer) and
#   nano_nina.c (nano_nina.h: SPI1 and the module's lines on the Pico SDK's
#   hardware_spi and hardware_gpio), with those libraries;
# - the include directory common/drivers/nina (nina_os.h, nano_nina.h,
#   nina_proto.h, nina_sock.h, nina_dns.h).
#
# nina_sock.c and nina_dns.c take the defaults of CONFIG_NANO_NINA_TCP_MAX_IO
# (3584) and CONFIG_NANO_NINA_SEND_RETRY_MS (1), the Zephyr build's values.
#
# READY_IRQ (the application's option NANO_NINA_READY_IRQ): the core's waits
# for the module's READY line sleep until an edge interrupt of the line
# (GPIO10) instead of sampling it every 100 us.  It defines, for every
# source of the target, NINA_OS_WAIT_READY (nina_spi.c takes the waits of
# nina_os.h, nina_os_ready_*()) and NANO_NINA_READY_IRQ (nina_os.c
# implements them; freertos/drivers/compat/rp2040/nano_gpio.c adds GPIO10 to
# the pins whose interrupt it handles).
#
# The application calls nano_nina_initialize() once at the start, before
# the tasks (the Zephyr build initializes spi1 at boot); the module itself
# is reset by the first request.  Each shared source is compiled with the
# prelude of the shared sources (common_prelude.h), as in compat.cmake.

get_filename_component(FREERTOS_NINA_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

function(freertos_nina target)
  cmake_parse_arguments(ARG "READY_IRQ" "" "" ${ARGN})

  set(nina_common_sources
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_spi.c
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_shell.c
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_sock.c
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_dns.c
  )

  target_sources(${target} PRIVATE
    ${FREERTOS_NINA_DIR}/nano_nina.c
    ${FREERTOS_NINA_DIR}/nina_os.c
    ${FREERTOS_NINA_DIR}/nina_socket.c
    ${nina_common_sources}
  )

  target_include_directories(${target} PRIVATE
    ${FREERTOS_COMMON_DIR}/drivers/nina
  )

  target_link_libraries(${target}
    hardware_gpio
    hardware_resets
    hardware_spi
  )

  if(ARG_READY_IRQ)
    target_compile_definitions(${target} PRIVATE
      NANO_NINA_READY_IRQ=1
      NINA_OS_WAIT_READY=1
    )
  endif()

  set_source_files_properties(${nina_common_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h"
  )
endfunction()
