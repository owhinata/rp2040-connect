# SPDX-License-Identifier: MIT
# The ThreadX kernel test firmware (main.c) on the Raspberry Pi Pico SDK and
# the Eclipse ThreadX kernel, for the Arduino Nano RP2040 Connect.  Built by
# threadx/script/build.sh board_kernel, which sets PICO_SDK_PATH,
# THREADX_PATH and PICO_TOOLCHAIN_PATH (threadx/script/setup.sh).

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD arduino_nano_rp2040_connect)

# Before project(): it sets up the toolchain.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(board_kernel C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

get_filename_component(REPO_DIR ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)
set(COMMON_DIR ${REPO_DIR}/common)
set(THREADX_DIR ${REPO_DIR}/threadx)

# ThreadX's kernel and the board's part of it: tx_user.h, the low-level
# initialization, the tick and the vectors (threadx/app/usb-console).
include(${THREADX_DIR}/app/usb-console/threadx_kernel.cmake)

# The layer's kernel part (the generic one, common/compat/kernel_generic, on
# the ThreadX primitives, common/compat/port/threadx), the OS-independent part
# the test uses (common/compat: zephyr_init.c, zephyr_work.c,
# zephyr_rtio_workq.c), the product's idle wait (the halves of
# common/pico_sdk/app/usb-console/src/idle.c named by
# threadx/app/usb-console/src/idle_os.h) and run time
# (threadx/app/usb-console/src/run_time.c), and the test.
add_executable(board_kernel
  main.c
  lock_test.c
  idle_test.c
  sdk_test.c
  test_port.c
  ${COMMON_DIR}/pico_sdk/app/usb-console/src/idle.c
  ${THREADX_DIR}/app/usb-console/src/run_time.c
  ${COMMON_DIR}/tests/zephyr_kernel/zephyr_kernel_test.c
  ${COMMON_DIR}/compat/kernel_generic/zephyr_kernel.c
  ${COMMON_DIR}/compat/zephyr_init.c
  ${COMMON_DIR}/compat/kernel_generic/zephyr_mutex.c
  ${COMMON_DIR}/compat/zephyr_work.c
  ${COMMON_DIR}/compat/zephyr_rtio_workq.c
  ${COMMON_DIR}/compat/port/threadx/zephyr_kernel_os.c
)

threadx_kernel(board_kernel)

# This directory (zephyr_kernel_test_os_impl.h), the board's idle_os.h and
# run_time.h (threadx/app/usb-console/src), the board's port header first
# (common/compat/port/threadx/rp2040) with the part shared by the OSes on the
# Pico SDK (common/compat/port/pico_sdk), then the ThreadX part of the layer
# (common/compat/port/threadx, with the board's priorities_os.h), the
# layer's implementations for an OS without its own
# (common/compat/port/generic: <zephyr/sys/clock.h>, <zephyr/sys/__assert.h>,
# the generic kernel part's zephyr_os.h with the default zephyr_os_atomic.h),
# the layer's headers (common/compat/include, port and internal), the
# RP2040's TIMER (common/chip/rp2040/include), the shared application
# headers idle.c reads (common/pico_sdk/app/usb-console/src: app.h,
# idle_blink.h) and the test.
target_include_directories(board_kernel PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${THREADX_DIR}/app/usb-console/src
  ${COMMON_DIR}/compat/port/threadx/rp2040
  ${COMMON_DIR}/compat/port/pico_sdk
  ${COMMON_DIR}/compat/port/threadx
  ${COMMON_DIR}/compat/port/generic
  ${COMMON_DIR}/compat/include
  ${COMMON_DIR}/compat/port
  ${COMMON_DIR}/compat/internal
  ${COMMON_DIR}/chip/rp2040/include
  ${COMMON_DIR}/pico_sdk/app/usb-console/src
  ${COMMON_DIR}/tests/zephyr_kernel
  ${COMMON_DIR}/tests/zephyr_kernel/kernel_generic
)

target_compile_options(board_kernel PRIVATE -Wall -Wextra)

# The name under /dev/serial/by-id.  The Pico SDK cuts USB strings to
# USBD_DESC_STR_MAX - 1 characters (19 by default).
target_compile_definitions(board_kernel PRIVATE
  USBD_PRODUCT="ThreadX kernel test"
  USBD_DESC_STR_MAX=32
)

# The LED shows this firmware's states (main.c): no blink from the idle
# wait.
set_source_files_properties(${COMMON_DIR}/pico_sdk/app/usb-console/src/idle.c
  PROPERTIES COMPILE_DEFINITIONS IDLE_BLINK=0
)

# The test's fputs() to the Pico SDK's stdio (main.c), and the count of the
# idle wait's passes (idle_test.c).
target_link_options(board_kernel PRIVATE
  -Wl,--wrap=fputs
  -Wl,--wrap=tx_low_power_enter
)

target_link_libraries(board_kernel
  pico_stdlib
  hardware_sync
  hardware_timer
)

# The console is the USB CDC ACM port; UART0 (GPIO0/1) is left alone.
pico_enable_stdio_usb(board_kernel 1)
pico_enable_stdio_uart(board_kernel 0)

# board_kernel.uf2, .bin, .hex, .dis and .map
pico_add_extra_outputs(board_kernel)
