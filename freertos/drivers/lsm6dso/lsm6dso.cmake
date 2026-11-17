# SPDX-License-Identifier: MIT
# The LSM6DSOX on the FreeRTOS build: the shared driver with its FIFO stream,
# and the parts of the Zephyr compatibility layer that it needs.  include()
# this file from the application's CMakeLists.txt, then call
#
#   freertos_lsm6dso(<target> [SELFTEST])
#
# after freertos_zephyr_compat(<target>) and freertos_i2c_dw(<target>)
# (freertos/drivers/compat/compat.cmake, freertos/drivers/i2c_dw/
# i2c_dw.cmake: the layer's kernel, work queues and RTIO, and the device
# i2c0).  It adds to the target the same sources as the NuttX build's
# CONFIG_NANO_LSM6DSO and CONFIG_NANO_LSM6DSO_STREAM (nuttx/drivers/lsm6dso/
# Make.defs, nuttx/drivers/compat/Make.defs):
#
# - the driver, shared with the Zephyr and NuttX builds
#   (common/drivers/lsm6dso: lsm6dso.c, which ends with this directory's
#   lsm6dso_instance.h, the device lsm6dsox; lsm6dso_fifo_hw.c, _fifo.c,
#   _cal.c, _zero.c; the FIFO stream lsm6dso_stream.c, _fifo_irq.c,
#   _compat.c, _fifo_decoder.c);
# - ST's glue to the Zephyr I2C API (common/drivers/stmemsc, shared with the
#   NuttX build) and ST's register driver lsm6dso_reg.c (freertos/stmemsc,
#   fetched by freertos/script/setup.sh, compiled as it is);
# - the layer's devices and sensor API (common/compat: zephyr_device.c, the
#   devices gpio0, i2c0 and lsm6dsox; zephyr_gpio.c, the device gpio0;
#   sensor_default_rtio.c, the default iodev and decoder of the sensor API)
#   and their FreeRTOS parts (freertos/drivers/compat: zephyr_sensor_clock.c,
#   the sensor clock on the TIMER; rp2040/nano_gpio.c, GPIO bank 0 and its
#   interrupt on the Pico SDK);
# - the start of the sensor (this directory: nano_lsm6dso.c, nano_lsm6dso.h);
# - CONFIG_NANO_LSM6DSO_STREAM=1 for every source of the target (the NuttX
#   build's Kconfig option of that name: zephyr_device.c lists gpio0,
#   zephyr_compat_initialize() starts it, and the prelude turns on
#   CONFIG_SENSOR_ASYNC_API, the Zephyr build's option);
# - with SELFTEST, the board's check of the sensor (lsm6dso_selftest.c, part
#   of the "selftest" command).
#
# Each shared source is compiled with the prelude of the shared sources
# (common_prelude.h), as in compat.cmake; so is the self-test, which includes
# lsm6dso.h (whose structures depend on CONFIG_SENSOR_ASYNC_API).  The
# Zephyr build's code keeps its casts of the bus functions (lsm6dso.c, with
# stmemsc.h), unused parameters and comparisons of signed with unsigned
# (lsm6dso.c, lsm6dso_compat.c, sensor_default_rtio.c), which -Wextra
# reports: those warnings are off for these three files only (the NuttX host
# tests do the same).  ST's file is compiled without warnings, as it is.

get_filename_component(FREERTOS_LSM6DSO_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)
get_filename_component(FREERTOS_STMEMSC_DIR ${CMAKE_CURRENT_LIST_DIR}/../../stmemsc ABSOLUTE)

function(freertos_lsm6dso target)
  cmake_parse_arguments(ARG "SELFTEST" "" "" ${ARGN})

  if(NOT EXISTS ${FREERTOS_STMEMSC_DIR}/lsm6dso_reg.c)
    message(FATAL_ERROR "${FREERTOS_STMEMSC_DIR}/lsm6dso_reg.c is missing; "
      "run freertos/script/setup.sh")
  endif()

  set(lsm6dso_driver_sources
    ${FREERTOS_COMMON_DIR}/drivers/lsm6dso/lsm6dso.c
    ${FREERTOS_COMMON_DIR}/drivers/lsm6dso/lsm6dso_fifo_hw.c
    ${FREERTOS_COMMON_DIR}/drivers/lsm6dso/lsm6dso_fifo.c
    ${FREERTOS_COMMON_DIR}/drivers/lsm6dso/lsm6dso_cal.c
    ${FREERTOS_COMMON_DIR}/drivers/lsm6dso/lsm6dso_zero.c
    ${FREERTOS_COMMON_DIR}/drivers/lsm6dso/lsm6dso_stream.c
    ${FREERTOS_COMMON_DIR}/drivers/lsm6dso/lsm6dso_fifo_irq.c
    ${FREERTOS_COMMON_DIR}/drivers/lsm6dso/lsm6dso_compat.c
    ${FREERTOS_COMMON_DIR}/drivers/lsm6dso/lsm6dso_fifo_decoder.c
    ${FREERTOS_COMMON_DIR}/drivers/stmemsc/stmemsc_i2c.c
    ${FREERTOS_COMMON_DIR}/drivers/stmemsc/stmemsc_mdelay.c
  )

  set(lsm6dso_compat_sources
    ${FREERTOS_COMMON_DIR}/compat/zephyr_device.c
    ${FREERTOS_COMMON_DIR}/compat/zephyr_gpio.c
    ${FREERTOS_COMMON_DIR}/compat/sensor_default_rtio.c
  )

  target_sources(${target} PRIVATE
    ${lsm6dso_driver_sources}
    ${lsm6dso_compat_sources}
    ${FREERTOS_STMEMSC_DIR}/lsm6dso_reg.c
    ${FREERTOS_COMPAT_DIR}/zephyr_sensor_clock.c
    ${FREERTOS_COMPAT_DIR}/rp2040/nano_gpio.c
    ${FREERTOS_LSM6DSO_DIR}/nano_lsm6dso.c
  )

  set(lsm6dso_prelude_sources ${lsm6dso_driver_sources} ${lsm6dso_compat_sources})
  if(ARG_SELFTEST)
    target_sources(${target} PRIVATE ${FREERTOS_LSM6DSO_DIR}/lsm6dso_selftest.c)
    list(APPEND lsm6dso_prelude_sources ${FREERTOS_LSM6DSO_DIR}/lsm6dso_selftest.c)
  endif()

  target_include_directories(${target} PRIVATE
    ${FREERTOS_LSM6DSO_DIR}
    ${FREERTOS_COMMON_DIR}/drivers/lsm6dso
    ${FREERTOS_COMMON_DIR}/drivers/stmemsc
    ${FREERTOS_STMEMSC_DIR}
  )

  target_compile_definitions(${target} PRIVATE CONFIG_NANO_LSM6DSO_STREAM=1)

  target_link_libraries(${target}
    hardware_gpio
    hardware_irq
  )

  set_source_files_properties(${lsm6dso_prelude_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h"
  )
  # The warnings of -Wextra that the shared sources give (see above).
  set_property(SOURCE ${FREERTOS_COMMON_DIR}/drivers/lsm6dso/lsm6dso.c APPEND PROPERTY
    COMPILE_OPTIONS -Wno-unused-parameter -Wno-sign-compare -Wno-cast-function-type
  )
  set_property(SOURCE ${FREERTOS_COMMON_DIR}/drivers/lsm6dso/lsm6dso_compat.c
    ${FREERTOS_COMMON_DIR}/compat/sensor_default_rtio.c APPEND PROPERTY
    COMPILE_OPTIONS -Wno-unused-parameter -Wno-sign-compare
  )
  set_source_files_properties(${FREERTOS_STMEMSC_DIR}/lsm6dso_reg.c PROPERTIES
    COMPILE_OPTIONS "-w"
  )
endfunction()
