cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_console)

target_sources(app PRIVATE src/main.c src/log_ram.c src/imu_fifo_shell.c src/imu_measure.c
  src/imu_measure_packet.c)

# Keep Zephyr's Kconfig and devicetree bindings, but use our sensor copies.
# The I2C driver is compiled unmodified from Zephyr.
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
