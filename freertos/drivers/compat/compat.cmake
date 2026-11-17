# SPDX-License-Identifier: MIT
# The FreeRTOS implementation of the Zephyr compatibility layer
# (freertos/drivers/compat) and the shared sources it builds (common/), for
# an application of freertos/app.  include() this file from the
# application's CMakeLists.txt, then call
#
#   freertos_zephyr_compat(<target> [SELFTEST])
#
# after add_executable().  It adds to the target:
#
# - the FreeRTOS part of the layer (zephyr_kernel.c, zephyr_mutex.c), the
#   OS-independent part that links at this stage (common/compat:
#   zephyr_work.c, zephyr_rtio.c, zephyr_rtio_workq.c, zephyr_mem_blocks.c)
#   and the smallest receiver of the LOG_* macros (freertos/drivers/log, on
#   common/log/log_ring.c);
# - with SELFTEST, the layer's self-test (zephyr_selftest.c and the board's
#   part, rp2040/zephyr_selftest_port.c) and NANO_SELFTEST=1;
# - the include directories, the board's port header first
#   (rp2040/zephyr_os_port.h), then the FreeRTOS part (zephyr_os.h, ...),
#   freertos/drivers/include (app_log_os.h), common/compat (the Zephyr
#   headers) and common/log;
# - the prelude of the shared sources (common_prelude.h, -include on each
#   source under common/ only, as the NuttX build does).
#
# The application provides FreeRTOSConfig.h, priorities_os.h and os_clock.h
# on its include path, a tick hook that calls zephyr_compat_tick()
# (zephyr_compat_os.h), and calls zephyr_compat_initialize()
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
  )

  target_sources(${target} PRIVATE
    ${FREERTOS_COMPAT_DIR}/zephyr_kernel.c
    ${FREERTOS_COMPAT_DIR}/zephyr_mutex.c
    ${FREERTOS_DRIVERS_DIR}/log/app_log_min.c
    ${common_sources}
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
    ${FREERTOS_DRIVERS_DIR}/include
    ${FREERTOS_COMMON_DIR}/compat
    ${FREERTOS_COMMON_DIR}/log
  )

  # Source file properties are scoped to the directory that calls this
  # function, the application's, which is the target's.
  set_source_files_properties(${common_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h"
  )
endfunction()
