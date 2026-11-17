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

# The sources shared with the Zephyr and NuttX builds (common/ at the root of
# the repository), and the FreeRTOS implementation of the Zephyr
# compatibility layer that runs them (freertos/drivers/compat).
get_filename_component(COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../common ABSOLUTE)
include(${CMAKE_CURRENT_LIST_DIR}/../../drivers/compat/compat.cmake)
# The key-value store and the "kv" command, on the last 8 KiB of the flash,
# which the linker script keeps the image out of (freertos/drivers/kv).
include(${CMAKE_CURRENT_LIST_DIR}/../../drivers/kv/kv.cmake)
# The SPI link to the NINA-W102 module and the "nina" command
# (freertos/drivers/nina).
include(${CMAKE_CURRENT_LIST_DIR}/../../drivers/nina/nina.cmake)
# I2C0 with its DMA reads (the device i2c0 of the compatibility layer) and the
# "i2c_dma" command (freertos/drivers/i2c_dw).
include(${CMAKE_CURRENT_LIST_DIR}/../../drivers/i2c_dw/i2c_dw.cmake)
# The LSM6DSOX with its FIFO stream, on i2c0 and the layer's gpio0 and RTIO
# (freertos/drivers/lsm6dso).
include(${CMAKE_CURRENT_LIST_DIR}/../../drivers/lsm6dso/lsm6dso.cmake)
# The IMU acquisition service and the "imu" command, with its MQTT transport,
# and the time synchronization with the "ntp" command (freertos/drivers/imu).
include(${CMAKE_CURRENT_LIST_DIR}/../../drivers/imu/imu.cmake)
# Wi-Fi: the NINA's network interface, the net_mgmt events, the stored
# credentials with the automatic reconnection and the "wifi" command
# (freertos/drivers/wifi).
include(${CMAKE_CURRENT_LIST_DIR}/../../drivers/wifi/wifi.cmake)
# The Telnet shell on TCP port 23 (freertos/drivers/telnet).
include(${CMAKE_CURRENT_LIST_DIR}/../../drivers/telnet/telnet.cmake)
# The "zperf" command: TCP tests against iperf3 (freertos/drivers/zperf).
include(${CMAKE_CURRENT_LIST_DIR}/../../drivers/zperf/zperf.cmake)
# The "sensor" command: the Zephyr build's sensor shell with its processing
# thread (freertos/drivers/sensor_shell).
include(${CMAKE_CURRENT_LIST_DIR}/../../drivers/sensor_shell/sensor_shell.cmake)

# The compatibility layer's self-test, the "selftest" command
# (freertos/drivers/compat/zephyr_selftest.h): a test feature, off by default
# (freertos/script/build.sh -DNANO_SELFTEST=ON).
option(NANO_SELFTEST "Build the selftest command" OFF)

# Zephyr's choice among the tasks of one priority (a preempted task goes on;
# freertos/drivers/compat/zephyr_os_sched.h).  OFF leaves FreeRTOS's own
# choice, for comparing the two (freertos/script/build.sh
# -DNANO_SCHED_ZEPHYR=OFF).  Defined for every target, the kernel included.
option(NANO_SCHED_ZEPHYR "Keep a preempted task's turn, as Zephyr does" ON)
if(NOT NANO_SCHED_ZEPHYR)
  add_compile_definitions(NANO_SCHED_FREERTOS=1)
endif()

# The kernel tick in Hz (FreeRTOSConfig.h configTICK_RATE_HZ): 10000 (100 us,
# the Zephyr and NuttX builds' tick, the default) or 1000 (1 ms: fewer tick
# interrupts; the layer's timeouts then count in 1 ms ticks, so a sleep
# shorter than 1 ms lasts 1 to 2 ms).  For comparing the two
# (freertos/script/build.sh -DNANO_TICK_HZ=1000).  Defined for every target,
# the kernel included.
set(NANO_TICK_HZ 10000 CACHE STRING "Kernel tick in Hz: 10000 or 1000")
set_property(CACHE NANO_TICK_HZ PROPERTY STRINGS 10000 1000)
if(NOT NANO_TICK_HZ STREQUAL "10000" AND NOT NANO_TICK_HZ STREQUAL "1000")
  message(FATAL_ERROR "NANO_TICK_HZ must be 10000 or 1000, not ${NANO_TICK_HZ}")
endif()
add_compile_definitions(NANO_TICK_HZ=${NANO_TICK_HZ})

# The NINA-W102's READY line waited for on its edge interrupt (GPIO10)
# instead of sampled every 100 us, as the Zephyr and NuttX builds do
# (freertos/drivers/nina/nina.cmake, READY_IRQ).  Off by default, for
# comparing the two (freertos/script/build.sh -DNANO_NINA_READY_IRQ=ON).
option(NANO_NINA_READY_IRQ "Wait for the NINA's READY line on its interrupt" OFF)

add_executable(usb-console
  src/main.c
  src/commands.c
  src/fault.c
  src/idle.c
  src/irq_stack.c
  src/tick.c
  ${COMMON_DIR}/log/fault_record.c
)

if(NANO_SELFTEST)
  freertos_zephyr_compat(usb-console SELFTEST)
else()
  freertos_zephyr_compat(usb-console)
endif()
freertos_kv(usb-console)
if(NANO_NINA_READY_IRQ)
  freertos_nina(usb-console READY_IRQ)
else()
  freertos_nina(usb-console)
endif()
if(NANO_SELFTEST)
  freertos_i2c_dw(usb-console SELFTEST)
  freertos_lsm6dso(usb-console SELFTEST)
else()
  freertos_i2c_dw(usb-console)
  freertos_lsm6dso(usb-console)
endif()
freertos_imu(usb-console)
freertos_wifi(usb-console)
freertos_telnet(usb-console)
freertos_zperf(usb-console)
freertos_sensor_shell(usb-console)

# FreeRTOSConfig.h; priorities_os.h (src/, which FreeRTOSConfig.h includes);
# priorities.h, cpu_load.h and nano_cpuload.h (common/compat);
# fault_record.h (common/log).
target_include_directories(usb-console PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/src
  ${COMMON_DIR}/compat
  ${COMMON_DIR}/log
)

target_compile_options(usb-console PRIVATE -Wall -Wextra)

# The name under /dev/serial/by-id (freertos/script/flash.sh looks for it).
# The Pico SDK cuts USB strings to USBD_DESC_STR_MAX - 1 characters (19 by
# default, one short of this name).
target_compile_definitions(usb-console PRIVATE
  USBD_PRODUCT="FreeRTOS USB Console"
  USBD_DESC_STR_MAX=32
)

# Fatal errors end in src/fault.c, which defines panic() itself: an empty
# PICO_PANIC_FUNCTION leaves the SDK's panic() (pico_platform_panic/panic.c)
# out, and the weak stand-in of custom_panic_function.S gives way to ours.
target_compile_definitions(usb-console PRIVATE
  PICO_PANIC_FUNCTION=
)

target_link_libraries(usb-console
  pico_stdlib
  pico_bootrom
  pico_unique_id
  hardware_sync
  hardware_timer
  hardware_watchdog
  FreeRTOS-Kernel
  FreeRTOS-Kernel-Heap4
)

# The console is the USB CDC ACM port; UART0 (GPIO0/1) is left alone.
pico_enable_stdio_usb(usb-console 1)
pico_enable_stdio_uart(usb-console 0)

# usb-console.uf2, .bin, .hex, .dis and .map
pico_add_extra_outputs(usb-console)
