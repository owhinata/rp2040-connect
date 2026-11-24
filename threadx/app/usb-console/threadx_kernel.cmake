# SPDX-License-Identifier: MIT
# The Eclipse ThreadX kernel on the Pico SDK for the RP2040, and the board's
# part of it, for a firmware under threadx/ (the USB console, the test
# firmware threadx/tests/board_kernel).  include() this file after
# pico_sdk_init(), then call
#
#   threadx_kernel(<target>)
#
# after add_executable().  THREADX_PATH is ThreadX's tree
# (threadx/script/setup.sh).  It adds to the target:
#
# - ThreadX's kernel (common/src, with the API's error checking, txe_*.c),
#   its Cortex-M0 port (ports/cortex_m0/gnu/src) and the execution profile
#   kit (utility/execution_profile_kit), as they are (no warnings checked),
#   with tx_user.h of this directory (TX_INCLUDE_USER_DEFINE_FILE, for the
#   C and the assembly sources alike: the port's .S files read
#   TX_LOW_POWER, TX_ENABLE_WFI and TX_EXECUTION_PROFILE_ENABLE from it).
#   They are the target's own sources, as the FreeRTOS kernel's are in the
#   FreeRTOS build, so that their objects can be named in the list of code
#   that runs from RAM (common/pico_sdk/app/usb-console/ram_code.cmake);
#   the link drops the functions that are not called (each in a section
#   of its own, -ffunction-sections, and --gc-sections, as the Pico SDK
#   builds);
# - the board's part, src/tx_initialize_low_level.c
#   (_tx_initialize_low_level(): SysTick, the priorities of PendSV, SVCall
#   and SysTick) and src/tick.c (isr_systick: ThreadX's tick, the layer's
#   tick and the catch-up of lost ticks, with tick_catchup.h of
#   common/compat/include, on the target's include path);
# - ThreadX's scheduler on the vectors of the Pico SDK's crt0 (its
#   isr_pendsv and isr_svcall are weak breakpoints; the link defines them
#   as ThreadX's handlers, so the vector table holds ThreadX's own
#   addresses);
# - the include directories of ThreadX and tx_user.h, and the Pico SDK's
#   locks and sleeps on ThreadX (threadx/drivers/pico_sdk_interop).
#
# The idle wait's hooks (the halves of common/pico_sdk/app/usb-console/src/
# idle.c named by src/idle_os.h) and the threads' run time (src/run_time.c)
# are the firmware's to add: the test firmware has its own LED.

get_filename_component(THREADX_APP_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
get_filename_component(THREADX_REPO_DIR ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)

if(NOT DEFINED ENV{THREADX_PATH})
  message(FATAL_ERROR "THREADX_PATH is not set (source threadx/script/setup.sh)")
endif()
set(THREADX_KERNEL_DIR $ENV{THREADX_PATH})
set(THREADX_PORT_DIR ${THREADX_KERNEL_DIR}/ports/cortex_m0/gnu)
set(THREADX_PROFILE_DIR ${THREADX_KERNEL_DIR}/utility/execution_profile_kit)

file(GLOB threadx_kernel_sources ${THREADX_KERNEL_DIR}/common/src/*.c)
file(GLOB threadx_port_sources ${THREADX_PORT_DIR}/src/*.S)
if(NOT threadx_kernel_sources OR NOT threadx_port_sources)
  message(FATAL_ERROR "ThreadX's sources are missing under ${THREADX_KERNEL_DIR}")
endif()
set(THREADX_KERNEL_SOURCES
  ${threadx_kernel_sources}
  ${threadx_port_sources}
  ${THREADX_PROFILE_DIR}/tx_execution_profile.c
)

include(${THREADX_REPO_DIR}/threadx/drivers/pico_sdk_interop/pico_sdk_interop.cmake)

function(threadx_kernel target)
  target_sources(${target} PRIVATE
    ${THREADX_KERNEL_SOURCES}
    ${THREADX_APP_DIR}/src/tx_initialize_low_level.c
    ${THREADX_APP_DIR}/src/tick.c
  )
  # tx_user.h reads rp2040_timer_us.h (the kit's clock).
  target_include_directories(${target} PRIVATE
    ${THREADX_APP_DIR}
    ${THREADX_KERNEL_DIR}/common/inc
    ${THREADX_PORT_DIR}/inc
    ${THREADX_PROFILE_DIR}
    ${THREADX_REPO_DIR}/common/chip/rp2040/include
  )
  target_compile_definitions(${target} PRIVATE TX_INCLUDE_USER_DEFINE_FILE)
  # ThreadX's own sources are compiled as they are, without the target's
  # warnings.  Source file properties are scoped to the directory that
  # calls this function, the target's.
  set_source_files_properties(${THREADX_KERNEL_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")
  target_link_libraries(${target} hardware_exception hardware_timer)
  target_link_options(${target} PRIVATE
    -Wl,--defsym=isr_pendsv=__tx_PendSVHandler
    -Wl,--defsym=isr_svcall=__tx_SVCallHandler
  )
  pico_sdk_interop(${target})
endfunction()
