# SPDX-License-Identifier: MIT
# The SPI link to the NINA-W102 module on the FreeRTOS build and the "nina"
# command.  include() this file from the application's CMakeLists.txt, then
# call
#
#   freertos_nina(<target>)
#
# after freertos_zephyr_compat(<target>) (freertos/drivers/compat/compat.cmake,
# whose layer and shell it needs).  It adds to the target:
#
# - the shared core of the link (common/drivers/nina/nina_spi.c:
#   nina_request() and nina_ip_config(), the WiFiNINA SPI protocol with the
#   Zephyr build's framing, timeouts and lazy reset of the module) and the
#   "nina version" command (common/drivers/nina/nina_shell.c, the Zephyr
#   build's text);
# - the two boundaries of the core on FreeRTOS: nina_os.c (nina_os.h: the
#   locks, sleeps and clock on the Zephyr compatibility layer) and
#   nano_nina.c (nano_nina.h: SPI1 and the module's lines on the Pico SDK's
#   hardware_spi and hardware_gpio), with those libraries;
# - the include directory common/drivers/nina (nina_os.h, nano_nina.h,
#   nina_proto.h).
#
# The application calls nano_nina_initialize() once at the start, before
# the tasks (the Zephyr build initializes spi1 at boot); the module itself
# is reset by the first request.  Each shared source is compiled with the
# prelude of the shared sources (common_prelude.h), as in compat.cmake.

get_filename_component(FREERTOS_NINA_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

function(freertos_nina target)
  set(nina_common_sources
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_spi.c
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_shell.c
  )

  target_sources(${target} PRIVATE
    ${FREERTOS_NINA_DIR}/nano_nina.c
    ${FREERTOS_NINA_DIR}/nina_os.c
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

  set_source_files_properties(${nina_common_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h"
  )
endfunction()
