# SPDX-License-Identifier: MIT
# The USB console on the Raspberry Pi Pico SDK and the FreeRTOS kernel, for
# the Arduino Nano RP2040 Connect.  Built by freertos/script/build.sh, which
# sets PICO_SDK_PATH, FREERTOS_KERNEL_PATH and PICO_TOOLCHAIN_PATH
# (freertos/script/setup.sh).

cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD arduino_nano_rp2040_connect)

# Both must come before project(): the first sets up the toolchain.
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(usb_console C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(usb-console
  src/main.c
  src/cli.c
  src/commands.c
)

# FreeRTOSConfig.h
target_include_directories(usb-console PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_compile_options(usb-console PRIVATE -Wall -Wextra)

# The name under /dev/serial/by-id (freertos/script/flash.sh looks for it).
# The Pico SDK cuts USB strings to USBD_DESC_STR_MAX - 1 characters (19 by
# default, one short of this name).
target_compile_definitions(usb-console PRIVATE
  USBD_PRODUCT="FreeRTOS USB Console"
  USBD_DESC_STR_MAX=32
)

target_link_libraries(usb-console
  pico_stdlib
  pico_bootrom
  pico_unique_id
  hardware_watchdog
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
)

# The console is the USB CDC ACM port; UART0 (GPIO0/1) is left alone.
pico_enable_stdio_usb(usb-console 1)
pico_enable_stdio_uart(usb-console 0)

# usb-console.uf2, .bin, .hex, .dis and .map
pico_add_extra_outputs(usb-console)
