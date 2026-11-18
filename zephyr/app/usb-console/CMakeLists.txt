cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_console)

# Sources shared with the NuttX build.
set(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../common)
cmake_path(NORMAL_PATH COMMON_DIR)

target_sources(app PRIVATE src/main.c src/log_ram.c ${COMMON_DIR}/imu/kv_store.c ${COMMON_DIR}/imu/imu_cal.c
  ${COMMON_DIR}/imu/imu_cal_codec.c ${COMMON_DIR}/imu/wifi_profile.c
  ${COMMON_DIR}/imu/time_sync.c ${COMMON_DIR}/imu/time_offset.c
  ${COMMON_DIR}/imu/imu_stream.c ${COMMON_DIR}/imu/imu_raw_batch.c ${COMMON_DIR}/imu/imu_raw_packet.c
  ${COMMON_DIR}/imu/imu_raw_time.c ${COMMON_DIR}/imu/imu_tick_clock.c ${COMMON_DIR}/imu/imu_raw_queue.c
  ${COMMON_DIR}/imu/imu_raw_mqtt.c ${COMMON_DIR}/imu/imu_raw_status.c)
target_include_directories(app PRIVATE src drivers/lsm6dso ${COMMON_DIR}/imu ${COMMON_DIR}/drivers/lsm6dso
  ${COMMON_DIR}/drivers/nina)

# NINA-W102: the shared core of common/drivers/nina (the SPI link nina_spi.c, the Wi-Fi
# interface nina_net.c, the socket table and commands nina_sock.c, the resolver nina_dns.c, the
# "nina" command nina_shell.c and the socket loops' waits nina_wait.c) and the Zephyr side in
# drivers/nina: the core's two boundaries (nano_nina.c: spi1 and the module's lines from the
# devicetree; nina_os.c: locks, sleeps, clock, the READY and IRQ interrupts) and the socket
# offload (nina_socket.c).  The core's
# POSIX socket names come from drivers/nina/posix, on the include path of these three sources
# only.
set(nina_dir ${COMMON_DIR}/drivers/nina)
target_sources(app PRIVATE ${nina_dir}/nina_spi.c drivers/nina/nano_nina.c drivers/nina/nina_os.c
  ${nina_dir}/nina_net.c ${nina_dir}/nina_sock.c drivers/nina/nina_socket.c ${nina_dir}/nina_dns.c
  ${nina_dir}/nina_shell.c ${nina_dir}/nina_wait.c)
# The NINA passthrough mode for esptool (drivers/nina/nina_passthrough.c, on the Pico SDK's
# registers of hal_rpi_pico): main() runs it on request, and its "nina passthrough" is an entry of
# nina_shell.c's set; both read drivers/nina/nina_passthrough.h.
if(CONFIG_NANO_NINA_PASSTHROUGH)
  target_sources(app PRIVATE drivers/nina/nina_passthrough.c)
endif()
set_property(SOURCE src/main.c ${nina_dir}/nina_shell.c APPEND PROPERTY INCLUDE_DIRECTORIES
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/nina)
set(nina_socket_sources ${nina_dir}/nina_sock.c drivers/nina/nina_socket.c ${nina_dir}/nina_dns.c)
set_property(SOURCE ${nina_socket_sources} APPEND PROPERTY INCLUDE_DIRECTORIES
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/nina/posix)
# The core's names of CONFIG_NINA_TCP_MAX_IO and CONFIG_NINA_SEND_RETRY_MS (nina_sock.h), and the
# resolver registered as the offload's, its results on drivers/nina's heap (nina_os.h).
set_property(SOURCE ${nina_socket_sources} APPEND PROPERTY COMPILE_DEFINITIONS
  CONFIG_NANO_NINA_TCP_MAX_IO=CONFIG_NINA_TCP_MAX_IO
  CONFIG_NANO_NINA_SEND_RETRY_MS=CONFIG_NINA_SEND_RETRY_MS NINA_OS_DNS_OFFLOAD)
# The waits for READY sleep until the line's edge interrupt (nina_os.c) instead of sampling it
# every 100 us.  Without this line they sample.
set_property(SOURCE ${nina_dir}/nina_spi.c drivers/nina/nina_os.c APPEND PROPERTY
  COMPILE_DEFINITIONS NINA_OS_WAIT_READY)
# The socket loops wait for the edges of the module's IRQ line when its firmware reports socket
# events (nina_wait.c, nina_os.c's GPIO2 interrupt).  Without this line they keep their poll
# intervals whatever the firmware.
set_property(SOURCE ${nina_dir}/nina_wait.c drivers/nina/nina_os.c APPEND PROPERTY
  COMPILE_DEFINITIONS NINA_OS_EVENTS)
# Integer division through the SIO divider: the Pico SDK's pico_divider (hal_rpi_pico), as the
# SDK's pico_divider_hardware links it.  libgcc defines the plain names, so the references are
# wrapped to the SDK's __wrap___aeabi_idiv, ...  The routines save the divider when they
# interrupt a division (PICO_DIVIDER_DISABLE_INTERRUPTS=0); the thread switches save it
# (CONFIG_ARM_RP2040_DIVIDER_SAVE, manifest/zephyr/patches).  Division by zero still calls
# libgcc's __aeabi_idiv0 and __aeabi_ldiv0, which return; the 64-bit results are libgcc's, but a
# 32-bit quotient is then saturated (libgcc: 0) and the remainder 0 (libgcc: the dividend).
if(CONFIG_NANO_HW_DIVIDER)
  if(NOT CONFIG_ARM_RP2040_DIVIDER_SAVE)
    message(FATAL_ERROR "CONFIG_NANO_HW_DIVIDER needs CONFIG_ARM_RP2040_DIVIDER_SAVE: "
      "apply the local patches (source script/setup.sh)")
  endif()
  set(pico_src_dir ${ZEPHYR_HAL_RPI_PICO_MODULE_DIR}/src)
  set(divider_asm ${pico_src_dir}/rp2_common/pico_divider/divider_hardware.S)
  target_sources(app PRIVATE ${divider_asm})
  set_source_files_properties(${divider_asm} PROPERTIES
    COMPILE_DEFINITIONS PICO_DIVIDER_DISABLE_INTERRUPTS=0
    INCLUDE_DIRECTORIES "${pico_src_dir}/rp2_common/hardware_divider/include;${pico_src_dir}/rp2040/pico_platform/include")
  foreach(sym __aeabi_idiv __aeabi_idivmod __aeabi_uidiv __aeabi_uidivmod __aeabi_ldivmod
          __aeabi_uldivmod)
    zephyr_ld_options(-Wl,--wrap=${sym})
  endforeach()
endif()

# Keep Zephyr's Kconfig and devicetree bindings while using local driver copies.
function(use_local_driver_sources target pattern replacement_dir)
  get_filename_component(dir "${replacement_dir}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
  get_target_property(sources ${target} SOURCES)
  set(updated_sources)
  set(replaced 0)
  foreach(source IN LISTS sources)
    if(source MATCHES "${pattern}")
      get_filename_component(filename "${source}" NAME)
      list(APPEND updated_sources "${dir}/${filename}")
      math(EXPR replaced "${replaced} + 1")
    else()
      list(APPEND updated_sources "${source}")
    endif()
  endforeach()
  if(replaced EQUAL 0)
    message(FATAL_ERROR "No sources replaced in ${target}; check the Zephyr version and configuration")
  endif()
  set_property(TARGET ${target} PROPERTY SOURCES "${updated_sources}")
endfunction()

# The DesignWare I2C driver is Zephyr's with a local patch (manifest/zephyr/patches/zephyr/
# 0003-i2c-dw-rp2040-fifo-and-dma.patch: the RP2040's FIFO depth and CONFIG_I2C_DW_RP2040_DMA).
# The DMA reads take their command words from common/drivers/i2c_dw/i2c_dw_rp2040_commands.c; the
# "i2c_dma" command is the shared one (common/drivers/i2c_dw/i2c_dma_shell.c, on the functions of
# nano_i2c_dma.h that the patched driver defines).
if(CONFIG_I2C_DW_RP2040_DMA)
  target_sources(drivers__i2c PRIVATE ${COMMON_DIR}/drivers/i2c_dw/i2c_dw_rp2040_commands.c)
  if(CONFIG_SHELL)
    target_sources(drivers__i2c PRIVATE ${COMMON_DIR}/drivers/i2c_dw/i2c_dma_shell.c)
  endif()
  target_include_directories(drivers__i2c PRIVATE ${COMMON_DIR}/drivers/i2c_dw)
endif()

# lsm6dso.c is in common/drivers/lsm6dso; lsm6dso_shub.c and lsm6dso_trigger.c, which Zephyr
# builds only with CONFIG_LSM6DSO_SENSORHUB and CONFIG_LSM6DSO_TRIGGER (off here), stay in
# drivers/lsm6dso.
use_local_driver_sources(drivers__sensor__st__lsm6dso "(^|/)lsm6dso\\.c$" ${COMMON_DIR}/drivers/lsm6dso)
if(CONFIG_LSM6DSO_SENSORHUB OR CONFIG_LSM6DSO_TRIGGER)
  use_local_driver_sources(drivers__sensor__st__lsm6dso "(^|/)lsm6dso_(shub|trigger)\\.c$" drivers/lsm6dso)
endif()
use_local_driver_sources(drivers__sensor "(^|/)sensor_shell\\.c$" ${COMMON_DIR}/drivers/sensor_shell)
# sensor_shell.c reads lsm6dso_attributes.h of common/drivers/lsm6dso and the Zephyr part,
# sensor_shell_os.h of drivers/sensor_shell.
target_include_directories(drivers__sensor PRIVATE ${ZEPHYR_BASE}/drivers/sensor ${COMMON_DIR}/drivers/lsm6dso
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/sensor_shell)
# The driver's files are in drivers/lsm6dso (lsm6dso_instance.h, the devices, which lsm6dso.c
# includes) and common/drivers/lsm6dso.
target_include_directories(drivers__sensor__st__lsm6dso PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers/lsm6dso ${COMMON_DIR}/drivers/lsm6dso)
target_sources(drivers__sensor__st__lsm6dso PRIVATE ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_fifo.c)
target_sources(drivers__sensor__st__lsm6dso PRIVATE ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_cal.c)
target_sources(drivers__sensor__st__lsm6dso PRIVATE ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_zero.c)
target_sources(drivers__sensor__st__lsm6dso PRIVATE ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_fifo_hw.c)
target_sources(drivers__sensor__st__lsm6dso PRIVATE ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_fifo_irq.c)
if(CONFIG_SENSOR_ASYNC_API)
  target_sources(drivers__sensor__st__lsm6dso PRIVATE
    ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_fifo_decoder.c
    ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_compat.c
    ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_stream.c)
endif()
