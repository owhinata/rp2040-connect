# SPDX-License-Identifier: MIT
# The USB console on the Raspberry Pi Pico SDK and the FreeRTOS kernel, for
# the Arduino Nano RP2040 Connect.  Built by freertos/script/build.sh, which
# sets PICO_SDK_PATH, FREERTOS_KERNEL_PATH and PICO_TOOLCHAIN_PATH
# (freertos/script/setup.sh).  What the builds on the Pico SDK share is
# common/pico_sdk/app/usb-console/usb-console.cmake; this file gives it the
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
# the repository), with the parts shared by the builds on the Pico SDK
# (common/pico_sdk), and the FreeRTOS part of the Zephyr compatibility layer
# that runs them (freertos/drivers/compat: zephyr_kernel_os_impl.h, the
# primitives of the generic kernel part, and rp2040/zephyr_os_port.h).
get_filename_component(COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../common ABSOLUTE)
set(PICO_SDK_OS freertos)
get_filename_component(FREERTOS_COMPAT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../drivers/compat ABSOLUTE)
set(PICO_SDK_OS_COMPAT_DIRS ${FREERTOS_COMPAT_DIR}/rp2040 ${FREERTOS_COMPAT_DIR})
get_filename_component(PICO_SDK_STMEMSC_DIR ${CMAKE_CURRENT_LIST_DIR}/../../stmemsc ABSOLUTE)
# FreeRTOSConfig.h; priorities_os.h and the FreeRTOS parts of the
# application's boundaries (src/).
set(PICO_SDK_OS_APP_DIRS ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/src)
set(PICO_SDK_OS_LIBRARIES FreeRTOS-Kernel)
# The name under /dev/serial/by-id (freertos/script/flash.sh looks for it).
set(PICO_SDK_USB_PRODUCT "FreeRTOS USB Console")
# The NINA core's FreeRTOS binding (its locks, waits and READY's interrupt)
# runs from RAM with the shared code (ram_code.cmake).
set(RAM_CODE_OS_SOURCES
  common/drivers/nina/port/freertos/nina_os.c
)
include(${COMMON_DIR}/pico_sdk/app/usb-console/usb-console.cmake)

# The start: main() (src/main.c) with the task "main" and the scheduler,
# which calls the shared steps (src/usb_console.c of PICO_SDK_APP_DIR) and,
# through them, the application's start shared by the three builds
# (common/imu/app_main.h: app_main.c, app_init.c); the commands (src/commands.c, with "ps" and
# "version" of src/commands_os_impl.h) and the generic commands
# (common/shell/kernel_shell.c, on src/kernel_shell_os_impl.h); the fatal
# errors, the idle pass and the interrupt stack; FreeRTOS's tick hook
# (src/tick.c); the records of fatal errors.
add_executable(usb-console
  src/main.c
  ${PICO_SDK_APP_DIR}/src/usb_console.c
  ${COMMON_DIR}/imu/app_main.c
  ${COMMON_DIR}/imu/app_init.c
  ${PICO_SDK_APP_DIR}/src/commands.c
  ${COMMON_DIR}/shell/kernel_shell.c
  ${PICO_SDK_APP_DIR}/src/fault.c
  ${PICO_SDK_APP_DIR}/src/idle.c
  ${PICO_SDK_APP_DIR}/src/irq_stack.c
  src/tick.c
  ${COMMON_DIR}/compat/log/fault_record.c
)

pico_sdk_usb_console(usb-console)
