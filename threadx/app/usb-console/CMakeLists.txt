# SPDX-License-Identifier: MIT
# The USB console on the Raspberry Pi Pico SDK and the Eclipse ThreadX
# kernel, for the Arduino Nano RP2040 Connect.  Built by
# threadx/script/build.sh, which sets PICO_SDK_PATH, THREADX_PATH and
# PICO_TOOLCHAIN_PATH (threadx/script/setup.sh).  What the builds on the
# Pico SDK share is common/pico_sdk/app/usb-console/usb-console.cmake; this
# file gives it the ThreadX parts.

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD arduino_nano_rp2040_connect)

# Before project(): it sets up the toolchain.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(usb_console C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

# ThreadX's kernel and the board's part of it: tx_user.h, the low-level
# initialization, the tick, the vectors and the Pico SDK's locks and sleeps
# on ThreadX (threadx_kernel.cmake).
include(${CMAKE_CURRENT_LIST_DIR}/threadx_kernel.cmake)

# The sources shared with the Zephyr and NuttX builds (common/ at the root of
# the repository), with the parts shared by the builds on the Pico SDK
# (common/pico_sdk), and the ThreadX part of the Zephyr compatibility layer
# that runs them (threadx/drivers/compat: zephyr_kernel_os_impl.h, the
# primitives of the generic kernel part, and rp2040/zephyr_os_port.h).
get_filename_component(COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../common ABSOLUTE)
set(PICO_SDK_OS threadx)
get_filename_component(THREADX_COMPAT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../drivers/compat ABSOLUTE)
set(PICO_SDK_OS_COMPAT_DIRS ${THREADX_COMPAT_DIR}/rp2040 ${THREADX_COMPAT_DIR})
get_filename_component(PICO_SDK_STMEMSC_DIR ${CMAKE_CURRENT_LIST_DIR}/../../stmemsc ABSOLUTE)
# priorities_os.h and the ThreadX parts of the application's boundaries
# (src/); tx_user.h comes with the kernel (threadx_kernel()).
set(PICO_SDK_OS_APP_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
set(PICO_SDK_OS_LIBRARIES)
# The name under /dev/serial/by-id (threadx/script/flash.sh looks for it).
set(PICO_SDK_USB_PRODUCT "ThreadX USB Console")
# The NINA core's binding (its locks, waits and READY's interrupt) runs from
# RAM with the shared code (ram_code.cmake), as in the FreeRTOS build.  The
# kernel stays in the flash, as in the other builds.
set(THREADX_SRC $ENV{THREADX_PATH}/common/src)
set(THREADX_PORT_SRC $ENV{THREADX_PATH}/ports/cortex_m0/gnu/src)
set(RAM_CODE_OS_SOURCES
  common/drivers/nina/port/threadx/nina_os.c
)
include(${COMMON_DIR}/pico_sdk/app/usb-console/usb-console.cmake)

# The start: main() (src/main.c) with ThreadX's set-up, the thread "main"
# and the scheduler, which calls the shared steps (src/usb_console.c of
# PICO_SDK_APP_DIR) and, through them, the application's start shared by
# the three builds (common/imu/app_main.h: app_main.c, app_init.c); the
# commands (src/commands.c, with "ps" and the kernel's part of "version" of
# src/commands_os.c here) and the generic commands
# (common/shell/kernel_shell.c, on src/kernel_shell_os_impl.h); the fatal
# errors (with the stack check's handler and the running thread's name,
# src/fault_os.c), the idle wait (its halves, src/idle_os.h) and the
# interrupt stack; the run times of "ps" (src/run_time.c); the variables
# of the layer's ThreadX part; the records of fatal errors.  threadx_kernel() adds
# ThreadX and its tick.
add_executable(usb-console
  src/main.c
  ${PICO_SDK_APP_DIR}/src/usb_console.c
  ${COMMON_DIR}/imu/app_main.c
  ${COMMON_DIR}/imu/app_init.c
  ${PICO_SDK_APP_DIR}/src/commands.c
  src/commands_os.c
  ${COMMON_DIR}/shell/kernel_shell.c
  ${PICO_SDK_APP_DIR}/src/fault.c
  src/fault_os.c
  ${PICO_SDK_APP_DIR}/src/idle.c
  ${PICO_SDK_APP_DIR}/src/irq_stack.c
  src/run_time.c
  ${THREADX_COMPAT_DIR}/zephyr_kernel_os.c
  ${COMMON_DIR}/compat/log/fault_record.c
)

threadx_kernel(usb-console)
pico_sdk_usb_console(usb-console)
