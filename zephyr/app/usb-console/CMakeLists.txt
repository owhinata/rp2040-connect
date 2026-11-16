cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_console)

# Sources shared with the NuttX build.
set(COMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../common)
cmake_path(NORMAL_PATH COMMON_DIR)

target_sources(app PRIVATE src/main.c src/log_ram.c src/kv_store.c src/imu_cal.c
  ${COMMON_DIR}/imu/imu_cal_codec.c src/wifi_profile.c
  src/time_sync.c ${COMMON_DIR}/imu/time_offset.c
  src/imu_stream.c ${COMMON_DIR}/imu/imu_raw_batch.c ${COMMON_DIR}/imu/imu_raw_packet.c
  ${COMMON_DIR}/imu/imu_raw_time.c ${COMMON_DIR}/imu/imu_tick_clock.c ${COMMON_DIR}/imu/imu_raw_queue.c
  src/imu_raw_mqtt.c ${COMMON_DIR}/imu/imu_raw_status.c
  drivers/nina/nina_wifi.c drivers/nina/nina_net.c drivers/nina/nina_socket.c
  drivers/nina/nina_dns.c)
target_include_directories(app PRIVATE src drivers/lsm6dso ${COMMON_DIR}/imu ${COMMON_DIR}/drivers/lsm6dso)
# Keep Zephyr's Kconfig and devicetree bindings while using local driver copies.
function(use_local_driver_sources target pattern replacement_dir)
  get_target_property(sources ${target} SOURCES)
  set(updated_sources)
  set(replaced 0)
  foreach(source IN LISTS sources)
    if(source MATCHES "${pattern}")
      get_filename_component(filename "${source}" NAME)
      list(APPEND updated_sources "${CMAKE_CURRENT_SOURCE_DIR}/${replacement_dir}/${filename}")
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

use_local_driver_sources(drivers__i2c "(^|/)i2c_dw\\.c$" drivers/i2c_dw)
if(CONFIG_I2C_DW_RP2040_DMA)
  target_sources(drivers__i2c PRIVATE ${COMMON_DIR}/drivers/i2c_dw/i2c_dw_rp2040_commands.c)
endif()
target_include_directories(drivers__i2c PRIVATE "${ZEPHYR_BASE}/drivers/i2c" ${COMMON_DIR}/drivers/i2c_dw)

use_local_driver_sources(drivers__sensor__st__lsm6dso "(^|/)lsm6dso(_shub|_trigger)?\\.c$" drivers/lsm6dso)
use_local_driver_sources(drivers__sensor "(^|/)sensor_shell\\.c$" drivers/sensor_shell)
# sensor_shell.c reads lsm6dso_attributes.h of common/drivers/lsm6dso.
target_include_directories(drivers__sensor PRIVATE ${ZEPHYR_BASE}/drivers/sensor ${COMMON_DIR}/drivers/lsm6dso)
# The driver's files are in drivers/lsm6dso and common/drivers/lsm6dso.
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
    drivers/lsm6dso/lsm6dso_compat.c
    ${COMMON_DIR}/drivers/lsm6dso/lsm6dso_stream.c)
endif()
