cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_console)

target_sources(app PRIVATE src/main.c src/log_ram.c src/kv_store.c src/wifi_profile.c
  src/time_sync.c src/time_offset.c
  src/imu_stream.c src/imu_raw_batch.c src/imu_raw_packet.c src/imu_raw_time.c
  src/imu_tick_clock.c src/imu_raw_queue.c src/imu_raw_mqtt.c src/imu_raw_status.c
  drivers/nina/nina_wifi.c drivers/nina/nina_net.c drivers/nina/nina_socket.c
  drivers/nina/nina_dns.c)
target_include_directories(app PRIVATE src)
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
  target_sources(drivers__i2c PRIVATE drivers/i2c_dw/i2c_dw_rp2040_commands.c)
endif()
target_include_directories(drivers__i2c PRIVATE "${ZEPHYR_BASE}/drivers/i2c")

use_local_driver_sources(drivers__sensor__st__lsm6dso "(^|/)lsm6dso(_shub|_trigger)?\\.c$" drivers/lsm6dso)
use_local_driver_sources(drivers__sensor "(^|/)sensor_shell\\.c$" drivers/sensor_shell)
target_include_directories(drivers__sensor PRIVATE ${ZEPHYR_BASE}/drivers/sensor)
target_sources(drivers__sensor__st__lsm6dso PRIVATE drivers/lsm6dso/lsm6dso_fifo.c)
target_sources(drivers__sensor__st__lsm6dso PRIVATE drivers/lsm6dso/lsm6dso_fifo_hw.c)
target_sources(drivers__sensor__st__lsm6dso PRIVATE drivers/lsm6dso/lsm6dso_fifo_irq.c)
if(CONFIG_SENSOR_ASYNC_API)
  target_sources(drivers__sensor__st__lsm6dso PRIVATE
    drivers/lsm6dso/lsm6dso_fifo_decoder.c
    drivers/lsm6dso/lsm6dso_compat.c
    drivers/lsm6dso/lsm6dso_stream.c)
endif()
