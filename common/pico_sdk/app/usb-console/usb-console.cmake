# SPDX-License-Identifier: MIT
# The USB console on the Pico SDK for the Arduino Nano RP2040 Connect: what
# the builds on the Pico SDK share of the application's build.  The OS's
# CMakeLists.txt (freertos/ or threadx/app/usb-console) sets, after
# pico_sdk_init():
#
#   COMMON_DIR                 common/ at the root of the repository
#   PICO_SDK_OS, PICO_SDK_OS_COMPAT_DIRS, PICO_SDK_STMEMSC_DIR
#                              the OS's part of the layer and the drivers'
#                              bindings (common/pico_sdk/drivers/compat/compat.cmake)
#   PICO_SDK_OS_APP_DIRS       the OS's application directories, put first on
#                              the include path (FreeRTOSConfig.h on
#                              FreeRTOS, priorities_os.h, the OS's parts of the
#                              application's boundaries: idle_os.h,
#                              fault_os_impl.h, main_os_impl.h,
#                              commands_os_impl.h, kernel_shell_os_impl.h)
#   PICO_SDK_OS_LIBRARIES      the OS's libraries (FreeRTOS-Kernel; none for
#                              ThreadX), linked after the SDK's
#   PICO_SDK_USB_PRODUCT       the USB product string, the name under
#                              /dev/serial/by-id ("FreeRTOS USB Console",
#                              "ThreadX USB Console"),
#                              at most 31 characters
#   RAM_CODE_OS_SOURCES        the OS's sources whose code runs from RAM
#                              (ram_code.cmake)
#
# then include()s this file, adds the executable with its sources
# (add_executable(): the shared ones of src/ here, PICO_SDK_APP_DIR, with
# the OS's own, in the order of the link) and calls
#
#   pico_sdk_usb_console(<target>)
#
# which adds the drivers (common/pico_sdk/drivers/*: the layer and its shell,
# the key-value store, the NINA, I2C0 and the option I2C1, the LSM6DSOX,
# the IMU service, Wi-Fi, Telnet, zperf, the "sensor" and "i2c" commands),
# the code that runs from RAM (ram_code.cmake), the start-up thread's stack
# in SCRATCH_X (ld/section_extra_post_scratch.incl), the include
# directories, the options of the SDK and the outputs (.uf2, .bin, .hex,
# .dis, .map).

get_filename_component(PICO_SDK_APP_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

include(${COMMON_DIR}/pico_sdk/drivers/compat/compat.cmake)
# The key-value store and the "kv" command, on the last 8 KiB of the flash,
# which the linker script keeps the image out of (common/pico_sdk/drivers/kv).
include(${COMMON_DIR}/pico_sdk/drivers/kv/kv.cmake)
# The SPI link to the NINA-W102 module and the "nina" command
# (common/pico_sdk/drivers/nina).
include(${COMMON_DIR}/pico_sdk/drivers/nina/nina.cmake)
# I2C0 with its DMA reads (the device i2c0 of the compatibility layer), I2C1
# as an option (the device i2c1) and the "i2c_dma" command
# (common/pico_sdk/drivers/i2c_dw).
include(${COMMON_DIR}/pico_sdk/drivers/i2c_dw/i2c_dw.cmake)
# The LSM6DSOX with its FIFO stream, on i2c0 and the layer's gpio0 and RTIO
# (common/pico_sdk/drivers/lsm6dso).
include(${COMMON_DIR}/pico_sdk/drivers/lsm6dso/lsm6dso.cmake)
# The IMU acquisition service and the "imu" command, with its MQTT transport,
# and the time synchronization with the "ntp" command
# (common/pico_sdk/drivers/imu).
include(${COMMON_DIR}/pico_sdk/drivers/imu/imu.cmake)
# Wi-Fi: the NINA's network interface, the net_mgmt events, the stored
# credentials with the automatic reconnection and the "wifi" command
# (common/pico_sdk/drivers/wifi).
include(${COMMON_DIR}/pico_sdk/drivers/wifi/wifi.cmake)
# The Telnet shell on TCP port 23 (common/pico_sdk/drivers/telnet).
include(${COMMON_DIR}/pico_sdk/drivers/telnet/telnet.cmake)
# The "zperf" command: TCP tests against iperf3 (common/pico_sdk/drivers/zperf).
include(${COMMON_DIR}/pico_sdk/drivers/zperf/zperf.cmake)
# The "sensor" command: the Zephyr build's sensor shell with its processing
# thread (common/pico_sdk/drivers/sensor_shell).
include(${COMMON_DIR}/pico_sdk/drivers/sensor_shell/sensor_shell.cmake)
# The "i2c" command: the Zephyr build's I2C shell (common/pico_sdk/drivers/i2c_shell).
include(${COMMON_DIR}/pico_sdk/drivers/i2c_shell/i2c_shell.cmake)

# The code of the sensor's and the send's paths, which runs from RAM: the
# flash's XIP cache is too small for it (ram_code.cmake).
include(${CMAKE_CURRENT_LIST_DIR}/ram_code.cmake)

# I2C1 (the device i2c1 on GPIO18/D6 and GPIO19/D7,
# common/pico_sdk/drivers/i2c_dw): off by default (the build's build.sh
# -DI2C_DW_I2C1=ON).  I2C0 has no option: the LSM6DSOX is on it.
option(I2C_DW_I2C1 "Build the I2C1 controller (device i2c1)" OFF)

function(pico_sdk_usb_console target)
  set(i2c_dw_options)
  if(I2C_DW_I2C1)
    list(APPEND i2c_dw_options I2C1)
  endif()

  pico_sdk_zephyr_compat(${target})
  pico_sdk_kv(${target})
  pico_sdk_nina(${target})
  pico_sdk_i2c_dw(${target} ${i2c_dw_options})
  pico_sdk_lsm6dso(${target})
  pico_sdk_imu(${target})
  pico_sdk_wifi(${target})
  pico_sdk_telnet(${target})
  pico_sdk_zperf(${target})
  pico_sdk_sensor_shell(${target})
  pico_sdk_i2c_shell(${target})
  pico_sdk_ram_code(${target})

  # The stack of the start-up thread "main" in SCRATCH_X, which only core 1's
  # stack would use (ld/section_extra_post_scratch.incl, main_os.h).
  pico_add_linker_script_override_path(${target} ${PICO_SDK_APP_DIR}/ld
    FILES section_extra_post_scratch.incl
  )

  # The OS's directories (PICO_SDK_OS_APP_DIRS); src/ here (app.h, fault.h,
  # the application's boundaries main_os.h, fault_os.h, commands_os.h);
  # priorities.h and cpu_load.h (common/compat); fault_record.h
  # (common/compat/log); the RP2040's code that no OS owns
  # (common/chip/rp2040/include: rp2040_timer_us.h; rp2040_flash_kv.h and
  # rp2040_flash_kv_os.h of the key-value area's flash,
  # common/pico_sdk/drivers/kv/kv.cmake).
  target_include_directories(${target} PRIVATE
    ${PICO_SDK_OS_APP_DIRS}
    ${PICO_SDK_APP_DIR}/src
    ${COMMON_DIR}/compat
    ${COMMON_DIR}/compat/log
    ${COMMON_DIR}/chip/rp2040/include
  )

  target_compile_options(${target} PRIVATE -Wall -Wextra)
  set_source_files_properties(${COMMON_DIR}/imu/app_main.c ${COMMON_DIR}/imu/app_init.c PROPERTIES
    COMPILE_OPTIONS "-include;${PICO_SDK_PRELUDE}"
  )

  # The name under /dev/serial/by-id (the build's flash.sh looks for it).
  # The Pico SDK cuts USB strings to USBD_DESC_STR_MAX - 1 characters (19 by
  # default).
  target_compile_definitions(${target} PRIVATE
    USBD_PRODUCT="${PICO_SDK_USB_PRODUCT}"
    USBD_DESC_STR_MAX=32
  )

  # Fatal errors end in src/fault.c, which defines panic() itself: an empty
  # PICO_PANIC_FUNCTION leaves the SDK's panic() (pico_platform_panic/panic.c)
  # out, and the weak stand-in of custom_panic_function.S gives way to ours.
  target_compile_definitions(${target} PRIVATE
    PICO_PANIC_FUNCTION=
  )

  # malloc() returns NULL when it finds no room, as in the Zephyr and NuttX
  # builds, instead of the Pico SDK's default panic (pico_malloc): its users
  # check (the shell's command state and long lines, zperf's buffers, the
  # NINA passthrough mode's buffer, newlib's own).
  target_compile_definitions(${target} PRIVATE
    PICO_MALLOC_PANIC=0
  )

  # The SDK's chain slots of shared interrupt handlers (12 bytes of RAM each),
  # for every core and line together: its default with stdio_usb, 6, is
  # taken by USBCTRL_IRQ's two (TinyUSB's and stdio_usb's), IO_IRQ_BANK0's two
  # raw handlers (common/pico_sdk/drivers/compat/rp2040/gpio_os_rp2040.c and
  # the NINA's nina_os.c) and DMA_IRQ_1's two (I2C0's reads and the NINA's
  # SPI DMA); one more would end in the SDK's hard_assert() when it is added.
  # 8 leaves room for two.
  target_compile_definitions(${target} PRIVATE
    PICO_MAX_SHARED_IRQ_HANDLERS=8
  )

  target_link_libraries(${target}
    pico_stdlib
    pico_bootrom
    pico_unique_id
    hardware_sync
    hardware_timer
    hardware_watchdog
    ${PICO_SDK_OS_LIBRARIES}
  )

  # The console is the USB CDC ACM port; UART0 (GPIO0/1) is left alone.
  pico_enable_stdio_usb(${target} 1)
  pico_enable_stdio_uart(${target} 0)

  # <target>.uf2, .bin, .hex, .dis and .map
  pico_add_extra_outputs(${target})
endfunction()
