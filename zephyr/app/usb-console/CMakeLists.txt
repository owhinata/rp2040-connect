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
# interface nina_net.c, the socket table and commands nina_sock.c, the resolver nina_dns.c and
# the "nina" command nina_shell.c) and the Zephyr side in drivers/nina: the core's two
# boundaries (nano_nina.c: spi1 and the module's lines from the devicetree; nina_os.c: locks,
# sleeps, clock and the READY interrupt) and the socket offload (nina_socket.c).  The core's
# POSIX socket names come from drivers/nina/posix, on the include path of these three sources
# only.
set(nina_dir ${COMMON_DIR}/drivers/nina)
target_sources(app PRIVATE ${nina_dir}/nina_spi.c drivers/nina/nano_nina.c drivers/nina/nina_os.c
  ${nina_dir}/nina_net.c ${nina_dir}/nina_sock.c drivers/nina/nina_socket.c ${nina_dir}/nina_dns.c
  ${nina_dir}/nina_shell.c)
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
# memcpy() and memset() through the bootrom (common/drivers/rp2040_rom, shared with the NuttX
# build): picolibc defines the plain names, so the references are wrapped to __wrap_memcpy, ...
if(CONFIG_NANO_ROM_MEM_OPS)
  set(rom_mem_dir ${COMMON_DIR}/drivers/rp2040_rom)
  target_sources(app PRIVATE src/rom_mem_early.c ${rom_mem_dir}/rp2040_rom_mem_lookup.c
    ${rom_mem_dir}/rp2040_rom_mem_ops.S)
  set_source_files_properties(${rom_mem_dir}/rp2040_rom_mem_ops.S PROPERTIES
    COMPILE_DEFINITIONS RP2040_ROM_MEM_WRAP)
  foreach(sym memcpy memset __aeabi_memcpy __aeabi_memcpy4 __aeabi_memcpy8 __aeabi_memset
          __aeabi_memset4 __aeabi_memset8)
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

# boot2: keep the board's choice (boot2_at25sf128a.S) but run the flash at clk_sys / 2 (62.5 MHz)
# instead of that file's default clk_sys / 4, as the Pico SDK boards do.  hal_rpi_pico has no
# option for PICO_FLASH_SPI_CLKDIV, so build its bootloader project once more with the define
# and link that boot2 in place of the one hal_rpi_pico builds.
if(CONFIG_RP2_REQUIRES_SECOND_STAGE_BOOT)
  if(NOT CONFIG_RP2_FLASH_AT25SF128A)
    message(FATAL_ERROR "boot2: expected CONFIG_RP2_FLASH_AT25SF128A")
  endif()
  include(ExternalProject)
  set(nano_boot2_dir ${CMAKE_BINARY_DIR}/nano_boot2)
  ExternalProject_Add(
    nano_boot2
    SOURCE_DIR ${ZEPHYR_BASE}/modules/hal_rpi_pico/bootloader
    BINARY_DIR ${nano_boot2_dir}
    CMAKE_ARGS
      -DCMAKE_SYSTEM_NAME=${CMAKE_SYSTEM_NAME}
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_ASM_COMPILER=${CMAKE_ASM_COMPILER}
      -DCMAKE_ASM_FLAGS=-DPICO_FLASH_SPI_CLKDIV=2
      -DZEPHYR_HAL_RPI_PICO_MODULE_DIR=${ZEPHYR_HAL_RPI_PICO_MODULE_DIR}
      -DZEPHYR_BASE=${ZEPHYR_BASE}
      -DFLASH_TYPE=AT25SF128A
      -DPYTHON_EXECUTABLE=${Python3_EXECUTABLE}
      -DRP2_BOOTLOADER_BYPRODUCT=${nano_boot2_dir}/boot_stage2.S
      -DCONFIG_PICOLIBC=${CONFIG_PICOLIBC}
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS ${nano_boot2_dir}/boot_stage2.S
    BUILD_ALWAYS TRUE)
  use_local_driver_sources(modules__hal_rpi_pico "(^|/)bootloader/boot_stage2\\.S$" ${nano_boot2_dir})
  add_dependencies(modules__hal_rpi_pico nano_boot2)
endif()

use_local_driver_sources(drivers__i2c "(^|/)i2c_dw\\.c$" drivers/i2c_dw)
if(CONFIG_I2C_DW_RP2040_DMA)
  target_sources(drivers__i2c PRIVATE ${COMMON_DIR}/drivers/i2c_dw/i2c_dw_rp2040_commands.c)
endif()
target_include_directories(drivers__i2c PRIVATE "${ZEPHYR_BASE}/drivers/i2c" ${COMMON_DIR}/drivers/i2c_dw)

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
