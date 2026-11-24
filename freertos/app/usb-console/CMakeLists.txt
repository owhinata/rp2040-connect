# SPDX-License-Identifier: MIT
# The USB console on the Raspberry Pi Pico SDK and the FreeRTOS kernel, for
# the Arduino Nano RP2040 Connect.  Built by freertos/script/build.sh, which
# sets PICO_SDK_PATH, FREERTOS_KERNEL_PATH and PICO_TOOLCHAIN_PATH
# (freertos/script/setup.sh).  What the builds on the Pico SDK share is
# common/imu/port/pico_sdk/usb-console.cmake; this file gives it the
# FreeRTOS parts.

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD arduino_nano_rp2040_connect)

# Both must come before project(): the first sets up the toolchain.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(usb_console C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

# The sources shared with the Zephyr and NuttX builds (common/ at the root of
# the repository), with the parts shared by the builds on the Pico SDK (the
# port/pico_sdk directories of common/), and the FreeRTOS parts that run
# them, found after PICO_SDK_OS: the Zephyr compatibility layer's
# (common/compat/port/freertos: zephyr_kernel_os_impl.h, the primitives of
# the generic kernel part, priorities_os.h, and rp2040/zephyr_os_port.h),
# the shell's (common/shell/port/freertos: kernel_shell_os_impl.h, the
# generic commands' OS part) and the drivers' bindings
# (common/drivers/<driver>/port/freertos).
get_filename_component(COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../common ABSOLUTE)
set(PICO_SDK_OS freertos)
get_filename_component(PICO_SDK_STMEMSC_DIR ${CMAKE_CURRENT_LIST_DIR}/../../stmemsc ABSOLUTE)
# FreeRTOSConfig.h and the FreeRTOS parts of the application's boundaries
# (src/).
set(PICO_SDK_OS_APP_DIRS ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/src)
set(PICO_SDK_OS_LIBRARIES FreeRTOS-Kernel)
# The name under /dev/serial/by-id (freertos/script/flash.sh looks for it).
set(PICO_SDK_USB_PRODUCT "FreeRTOS USB Console")
# The NINA core's FreeRTOS binding (its locks, waits and READY's interrupt)
# runs from RAM with the shared code (ram_code.cmake).
set(RAM_CODE_OS_SOURCES
  common/drivers/nina/port/freertos/nina_os.c
)
include(${COMMON_DIR}/imu/port/pico_sdk/usb-console.cmake)

# The start: main() (src/main.c) with the task "main" and the scheduler,
# which calls the shared steps (usb_console.c of PICO_SDK_APP_DIR,
# common/imu/port/pico_sdk) and, through them, the application's start
# shared by the three builds (common/imu/app_main.h: app_main.c,
# app_init.c); the commands (commands.c there, with "ps" and the kernel's
# part of "version" of src/commands_os.c here, which also has the run time
# counters' clock) and the generic commands (common/shell/kernel_shell.c,
# on kernel_shell_os_impl.h); the fatal errors (fault.c there, with
# FreeRTOS's stack overflow hook and the running task's name,
# src/fault_os.c), the idle pass and the interrupt stack (idle.c,
# irq_stack.c there); FreeRTOS's tick hook (src/tick.c); the records of
# fatal errors.
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
  src/tick.c
  ${COMMON_DIR}/compat/log/fault_record.c
)

pico_sdk_usb_console(usb-console)
