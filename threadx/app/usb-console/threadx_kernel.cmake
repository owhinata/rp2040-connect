# SPDX-License-Identifier: MIT
# The Eclipse ThreadX kernel on the Pico SDK for the RP2040, and the board's
# part of it, for a firmware under threadx/ (the USB console, the test
# firmware threadx/tests/board_kernel).  include() this file after
# pico_sdk_init(), then call
#
#   threadx_kernel(<target>)
#
# after add_executable().  THREADX_PATH is ThreadX's tree
# (threadx/script/setup.sh).  It makes:
#
# - the library threadx_kernel: ThreadX's kernel (common/src, with the API's
#   error checking, txe_*.c) and its Cortex-M0 port (ports/cortex_m0/gnu/src),
#   as they are (no warnings checked), with tx_user.h of this directory
#   (TX_INCLUDE_USER_DEFINE_FILE); the link takes the services used;
# - in the target: the board's part, src/tx_initialize_low_level.c
#   (_tx_initialize_low_level(): SysTick, the priorities of PendSV, SVCall
#   and SysTick) and src/tick.c (isr_systick: ThreadX's tick, the layer's
#   tick and the catch-up of lost ticks, with tick_catchup.h of the FreeRTOS
#   build, found below), ThreadX's scheduler on the vectors of the Pico
#   SDK's crt0 (its isr_pendsv and isr_svcall are weak breakpoints; the
#   link defines them as ThreadX's handlers, so the vector table holds
#   ThreadX's own addresses), and the include directories of ThreadX and
#   tx_user.h.

get_filename_component(THREADX_APP_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
get_filename_component(THREADX_REPO_DIR ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)

if(NOT DEFINED ENV{THREADX_PATH})
  message(FATAL_ERROR "THREADX_PATH is not set (source threadx/script/setup.sh)")
endif()
set(THREADX_KERNEL_DIR $ENV{THREADX_PATH})
set(THREADX_PORT_DIR ${THREADX_KERNEL_DIR}/ports/cortex_m0/gnu)

file(GLOB threadx_kernel_sources ${THREADX_KERNEL_DIR}/common/src/*.c)
file(GLOB threadx_port_sources ${THREADX_PORT_DIR}/src/*.S)
if(NOT threadx_kernel_sources OR NOT threadx_port_sources)
  message(FATAL_ERROR "ThreadX's sources are missing under ${THREADX_KERNEL_DIR}")
endif()

add_library(threadx_kernel STATIC ${threadx_kernel_sources} ${threadx_port_sources})
target_include_directories(threadx_kernel PUBLIC
  ${THREADX_APP_DIR}
  ${THREADX_KERNEL_DIR}/common/inc
  ${THREADX_PORT_DIR}/inc
)
target_compile_definitions(threadx_kernel PUBLIC TX_INCLUDE_USER_DEFINE_FILE)
target_compile_options(threadx_kernel PRIVATE -w)

# tick_catchup.h, the FreeRTOS build's, read by its path: under common/ once
# it is shared, else in the FreeRTOS application.  Its directory is not put
# on the include path, which has headers of the same names as ours.
file(GLOB_RECURSE threadx_tick_catchup_header ${THREADX_REPO_DIR}/common/tick_catchup.h)
if(NOT threadx_tick_catchup_header)
  set(threadx_tick_catchup_header
    ${THREADX_REPO_DIR}/freertos/app/usb-console/src/tick_catchup.h)
endif()
list(LENGTH threadx_tick_catchup_header threadx_tick_catchup_count)
list(GET threadx_tick_catchup_header 0 threadx_tick_catchup_header)
if(NOT threadx_tick_catchup_count EQUAL 1 OR NOT EXISTS ${threadx_tick_catchup_header})
  message(FATAL_ERROR "tick_catchup.h not found (or found twice) under ${THREADX_REPO_DIR}")
endif()

function(threadx_kernel target)
  target_sources(${target} PRIVATE
    ${THREADX_APP_DIR}/src/tx_initialize_low_level.c
    ${THREADX_APP_DIR}/src/tick.c
  )
  set_source_files_properties(${THREADX_APP_DIR}/src/tick.c PROPERTIES
    COMPILE_DEFINITIONS "TICK_CATCHUP_HEADER=\"${threadx_tick_catchup_header}\""
  )
  target_link_libraries(${target} threadx_kernel hardware_exception hardware_timer)
  target_link_options(${target} PRIVATE
    -Wl,--defsym=isr_pendsv=__tx_PendSVHandler
    -Wl,--defsym=isr_svcall=__tx_SVCallHandler
  )
endfunction()
