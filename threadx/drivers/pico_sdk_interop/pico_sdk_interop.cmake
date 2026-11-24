# SPDX-License-Identifier: MIT
# The Pico SDK's locks and sleeps on ThreadX (pico_sdk_interop.h): for a
# firmware on the Pico SDK and ThreadX, call
#
#   pico_sdk_interop(<target>)
#
# (threadx_kernel() does).  It adds pico_sdk_interop.c and gives every
# source of the target, the SDK's included, the adapter header
# (PICO_CONFIG_RTOS_ADAPTER_HEADER, read by pico/config.h): the SDK's
# mutex_t and its users must all see the same lock_owner_id_t.

get_filename_component(PICO_SDK_INTEROP_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

function(pico_sdk_interop target)
  target_sources(${target} PRIVATE ${PICO_SDK_INTEROP_DIR}/pico_sdk_interop.c)
  target_include_directories(${target} PRIVATE ${PICO_SDK_INTEROP_DIR})
  target_compile_definitions(${target} PRIVATE
    PICO_CONFIG_RTOS_ADAPTER_HEADER=${PICO_SDK_INTEROP_DIR}/pico_sdk_interop.h
  )
  target_link_libraries(${target} pico_sync pico_time hardware_sync)
endfunction()
