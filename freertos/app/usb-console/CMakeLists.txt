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
# I2C0 with its DMA reads (the device i2c0 of the compatibility layer), I2C1
# as an option (the device i2c1) and the "i2c_dma" command
# (freertos/drivers/i2c_dw).
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
# (common/compat/selftest/zephyr_selftest.h): a test feature, off by default
# (freertos/script/build.sh -DSELFTEST=ON).
option(SELFTEST "Build the selftest command" OFF)

# The code of the sensor's and the send's paths, which runs from RAM: the
# flash's XIP cache is too small for it (ram_code.cmake).
include(${CMAKE_CURRENT_LIST_DIR}/ram_code.cmake)

# I2C1 (the device i2c1 on GPIO18/D6 and GPIO19/D7, freertos/drivers/i2c_dw):
# off by default (freertos/script/build.sh -DI2C_DW_I2C1=ON).  I2C0 has no
# option: the LSM6DSOX is on it.
option(I2C_DW_I2C1 "Build the I2C1 controller (device i2c1)" OFF)
set(i2c_dw_options)
if(I2C_DW_I2C1)
  list(APPEND i2c_dw_options I2C1)
endif()

# The generic commands (common/shell/kernel_shell.c, on src/kernel_shell_os_impl.h)
# are linked where this build's own were, after src/commands.c.  The
# application's start shared by the three builds (common/imu/app_main.h:
# app_main.c, app_init.c) follows src/main.c, which calls it.
add_executable(usb-console
  src/main.c
  ${COMMON_DIR}/imu/app_main.c
  ${COMMON_DIR}/imu/app_init.c
  src/commands.c
  ${COMMON_DIR}/shell/kernel_shell.c
  src/fault.c
  src/idle.c
  src/irq_stack.c
  src/tick.c
  ${COMMON_DIR}/compat/log/fault_record.c
)

if(SELFTEST)
  freertos_zephyr_compat(usb-console SELFTEST)
else()
  freertos_zephyr_compat(usb-console)
endif()
freertos_kv(usb-console)
freertos_nina(usb-console)
if(SELFTEST)
  freertos_i2c_dw(usb-console SELFTEST ${i2c_dw_options})
  freertos_lsm6dso(usb-console SELFTEST)
else()
  freertos_i2c_dw(usb-console ${i2c_dw_options})
  freertos_lsm6dso(usb-console)
endif()
freertos_imu(usb-console)
freertos_wifi(usb-console)
freertos_telnet(usb-console)
freertos_zperf(usb-console)
freertos_sensor_shell(usb-console)
freertos_ram_code(usb-console)

# The stack of the start-up task "main" in SCRATCH_X, which only core 1's
# stack would use (ld/section_extra_post_scratch.incl, src/main.c).
pico_add_linker_script_override_path(usb-console ${CMAKE_CURRENT_LIST_DIR}/ld
  FILES section_extra_post_scratch.incl
)

# FreeRTOSConfig.h; priorities_os.h (src/, which FreeRTOSConfig.h includes);
# priorities.h and cpu_load.h (common/compat);
# fault_record.h (common/compat/log); the RP2040's code that no OS owns
# (common/chip/rp2040/include: rp2040_timer_us.h, which src/os_clock.h reads;
# rp2040_flash_kv.h and rp2040_flash_kv_os.h of the key-value area's flash,
# drivers/kv/kv.cmake).
target_include_directories(usb-console PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/src
  ${COMMON_DIR}/compat
  ${COMMON_DIR}/compat/log
  ${COMMON_DIR}/chip/rp2040/include
)

target_compile_options(usb-console PRIVATE -Wall -Wextra)
set_source_files_properties(${COMMON_DIR}/imu/app_main.c ${COMMON_DIR}/imu/app_init.c PROPERTIES
  COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h"
)

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

# malloc() returns NULL when it finds no room, as in the Zephyr and NuttX
# builds, instead of the Pico SDK's default panic (pico_malloc): its users
# check (the shell's command state and long lines, zperf's buffers, the
# NINA passthrough mode's buffer, newlib's own).
target_compile_definitions(usb-console PRIVATE
  PICO_MALLOC_PANIC=0
)

# The SDK's chain slots of shared interrupt handlers (12 bytes of RAM each),
# for every core and line together: its default with stdio_usb, 6, is
# taken by USBCTRL_IRQ's two (TinyUSB's and stdio_usb's), IO_IRQ_BANK0's two
# raw handlers (drivers/compat/rp2040/gpio_os_rp2040.c and the NINA's
# nina_os.c) and DMA_IRQ_1's two (I2C0's reads and the NINA's SPI DMA); one
# more would end in the SDK's hard_assert() when it is added.  8 leaves room
# for two.
target_compile_definitions(usb-console PRIVATE
  PICO_MAX_SHARED_IRQ_HANDLERS=8
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
