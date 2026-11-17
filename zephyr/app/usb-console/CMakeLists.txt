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
  ${COMMON_DIR}/imu/imu_raw_mqtt.c ${COMMON_DIR}/imu/imu_raw_status.c
  drivers/nina/nina_wifi.c ${COMMON_DIR}/drivers/nina/nina_net.c drivers/nina/nina_socket.c
  drivers/nina/nina_dns.c)
# common/drivers/nina: nina_proto.h for src/main.c and drivers/nina.
target_include_directories(app PRIVATE src drivers/lsm6dso ${COMMON_DIR}/imu ${COMMON_DIR}/drivers/lsm6dso
  ${COMMON_DIR}/drivers/nina)
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
