# SPDX-License-Identifier: MIT
# The USB console on the Raspberry Pi Pico SDK and the Eclipse ThreadX
# kernel, for the Arduino Nano RP2040 Connect.  Built by
# threadx/script/build.sh, which sets PICO_SDK_PATH, THREADX_PATH and
# PICO_TOOLCHAIN_PATH (threadx/script/setup.sh).  What the builds on the
# Pico SDK share is common/imu/port/pico_sdk/usb-console.cmake; this
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
# the repository), with the parts shared by the builds on the Pico SDK (the
# port/pico_sdk directories of common/), and the ThreadX parts that run
# them, found after PICO_SDK_OS: the Zephyr compatibility layer's
# (common/compat/port/threadx: zephyr_kernel_os_impl.h, the primitives of
# the generic kernel part, with its variables in zephyr_kernel_os.c,
# priorities_os.h, and rp2040/zephyr_os_port.h), the shell's
# (common/shell/port/threadx: kernel_shell_os_impl.h, the generic commands'
# OS part) and the drivers' bindings (common/drivers/<driver>/port/threadx).
get_filename_component(COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../common ABSOLUTE)
set(PICO_SDK_OS threadx)
get_filename_component(PICO_SDK_STMEMSC_DIR ${CMAKE_CURRENT_LIST_DIR}/../../stmemsc ABSOLUTE)
# The ThreadX parts of the application's boundaries (src/); tx_user.h comes
# with the kernel (threadx_kernel()).
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
include(${COMMON_DIR}/imu/port/pico_sdk/usb-console.cmake)

# The start: main() (src/main.c) with ThreadX's set-up, the thread "main"
# and the scheduler, which calls the shared steps (usb_console.c of
# PICO_SDK_APP_DIR, common/imu/port/pico_sdk) and, through them, the
# application's start shared by the three builds (common/imu/app_main.h:
# app_main.c, app_init.c); the commands (commands.c there, with "ps" and
# the kernel's part of "version" of src/commands_os.c here) and the generic
# commands (common/shell/kernel_shell.c, on kernel_shell_os_impl.h); the
# fatal errors (fault.c there, with the stack check's handler and the
# running thread's name, src/fault_os.c), the idle wait (idle.c there, its
# halves in src/idle_os.h) and the interrupt stack (irq_stack.c there); the
# run times of "ps" (src/run_time.c); the variables of the layer's ThreadX
# part; the records of fatal errors.  threadx_kernel() adds ThreadX and its
# tick.
add_executable(usb-console
  src/main.c
  ${PICO_SDK_APP_DIR}/usb_console.c
  ${COMMON_DIR}/imu/app_main.c
  ${COMMON_DIR}/imu/app_init.c
  ${PICO_SDK_APP_DIR}/commands.c
  src/commands_os.c
  ${COMMON_DIR}/shell/kernel_shell.c
  ${PICO_SDK_APP_DIR}/fault.c
  src/fault_os.c
  ${PICO_SDK_APP_DIR}/idle.c
  ${PICO_SDK_APP_DIR}/irq_stack.c
  src/run_time.c
  ${COMMON_DIR}/compat/port/threadx/zephyr_kernel_os.c
  ${COMMON_DIR}/compat/log/fault_record.c
)

threadx_kernel(usb-console)
pico_sdk_usb_console(usb-console)
