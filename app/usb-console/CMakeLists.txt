cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_console)

target_sources(app PRIVATE src/main.c src/log_ram.c)

# Keep Zephyr's Kconfig and devicetree bindings, but compile only our copies.
# Replace the sources in their original libraries so all DW instances use the
# same implementation and the standard sensor driver cannot bind a second time.
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
use_local_driver_sources(drivers__sensor__st__lsm6dso "(^|/)lsm6dso(_shub|_trigger)?\\.c$" drivers/lsm6dso)
target_include_directories(drivers__i2c PRIVATE ${ZEPHYR_BASE}/drivers/i2c)
