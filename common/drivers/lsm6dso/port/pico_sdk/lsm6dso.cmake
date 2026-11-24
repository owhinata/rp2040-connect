# SPDX-License-Identifier: MIT
# The LSM6DSOX on the Pico SDK: the shared driver with its FIFO stream,
# and the parts of the Zephyr compatibility layer that it needs.  include()
# this file from the application's CMakeLists.txt, then call
#
#   pico_sdk_lsm6dso(<target>)
#
# after pico_sdk_zephyr_compat(<target>) and pico_sdk_i2c_dw(<target>)
# (common/compat/port/pico_sdk/compat.cmake, common/drivers/i2c_dw/port/pico_sdk/
# i2c_dw.cmake: the layer's kernel, work queues and RTIO, and the device
# i2c0; the configuration stops if CONFIG_I2C_DW_I2C0 is not on, as the
# NuttX build's LSM6DSO_LOCAL depends on I2C_DW_I2C0).  It adds to the target the same sources as the NuttX build's
# CONFIG_LSM6DSO_LOCAL and CONFIG_LSM6DSO_STREAM (nuttx/drivers/lsm6dso/
# Make.defs, nuttx/drivers/compat/Make.defs):
#
# - the driver, shared with the Zephyr and NuttX builds
#   (common/drivers/lsm6dso, the files of the Zephyr build's module): its
#   core (lsm6dso.c, lsm6dso_cal.c, lsm6dso_fifo.c, and the FIFO stream
#   lsm6dso_stream.c) and the Zephyr build's binding (port/zephyr:
#   lsm6dso_sensor.c, which ends with this build's
#   port/pico_sdk/lsm6dso_instance.h, the device lsm6dsox; the stream's RTIO
#   side lsm6dso_rtio_stream.c, _rtio.c, _decoder.c; the core's OS part
#   lsm6dso_os_impl.h with lsm6dso_os.c, the stream's work queue and INT1
#   interrupt, and its user part lsm6dso_stream_req_impl.h); its extended API
#   <zephyr/drivers/sensor/lsm6dso.h> is
#   common/drivers/lsm6dso/include/lsm6dso.h, through common/compat;
# - ST's glue to the Zephyr I2C API (common/drivers/stmemsc, shared with the
#   NuttX build) and ST's register driver lsm6dso_reg.c (PICO_SDK_STMEMSC_DIR:
#   freertos/stmemsc, fetched by freertos/script/setup.sh; compiled as it is);
# - the layer's devices and sensor API (common/compat: zephyr_device.c, the
#   devices gpio0, i2c0 and lsm6dsox; zephyr_gpio.c, the device gpio0;
#   sensor_default_rtio.c, the default iodev and decoder of the sensor API;
#   zephyr_sensor_clock.c, the sensor clock on the TIMER) and their part
#   on the Pico SDK (common/compat/port/pico_sdk/gpio_os_rp2040.c, GPIO bank 0 and its
#   interrupt on the Pico SDK);
# - the start of the sensor (common/drivers/lsm6dso/port/pico_sdk:
#   lsm6dso_board.c, lsm6dso_board.h);
# - CONFIG_LSM6DSO_STREAM=1 for every source of the target (the NuttX
#   build's Kconfig option of that name: zephyr_device.c lists gpio0,
#   zephyr_compat_initialize() starts it, and the prelude turns on
#   CONFIG_SENSOR_ASYNC_API, the Zephyr build's option).
#
# Each shared source is compiled with the prelude of the shared sources
# (common_prelude.h), as in compat.cmake.
# The Zephyr build's code keeps its casts of the bus functions
# (lsm6dso_sensor.c, with stmemsc.h), unused parameters and comparisons of
# signed with unsigned (lsm6dso_sensor.c, lsm6dso_rtio.c,
# sensor_default_rtio.c), which -Wextra reports: those warnings are off for
# these three files only (the NuttX host tests do the same).  ST's file is compiled without warnings, as it is.


function(pico_sdk_lsm6dso target)
  get_target_property(defs ${target} COMPILE_DEFINITIONS)
  if(NOT "CONFIG_I2C_DW_I2C0=1" IN_LIST defs)
    message(FATAL_ERROR "pico_sdk_lsm6dso: the LSM6DSOX is on i2c0, but "
      "CONFIG_I2C_DW_I2C0 is not on for ${target} (call pico_sdk_i2c_dw() "
      "first; common/drivers/i2c_dw/port/pico_sdk/i2c_dw.cmake)")
  endif()

  if(NOT EXISTS ${PICO_SDK_STMEMSC_DIR}/lsm6dso_reg.c)
    message(FATAL_ERROR "${PICO_SDK_STMEMSC_DIR}/lsm6dso_reg.c is missing; "
      "run the build's setup.sh")
  endif()

  set(lsm6dso_dir ${PICO_SDK_COMMON_DIR}/drivers/lsm6dso)
  set(lsm6dso_driver_sources
    ${lsm6dso_dir}/port/zephyr/lsm6dso_sensor.c
    ${lsm6dso_dir}/lsm6dso.c
    ${lsm6dso_dir}/lsm6dso_cal.c
    ${lsm6dso_dir}/lsm6dso_fifo.c
    ${lsm6dso_dir}/lsm6dso_stream.c
    ${lsm6dso_dir}/port/zephyr/lsm6dso_rtio_stream.c
    ${lsm6dso_dir}/port/zephyr/lsm6dso_rtio.c
    ${lsm6dso_dir}/port/zephyr/lsm6dso_decoder.c
    ${lsm6dso_dir}/port/zephyr/lsm6dso_os.c
    ${PICO_SDK_COMMON_DIR}/drivers/stmemsc/stmemsc_i2c.c
    ${PICO_SDK_COMMON_DIR}/drivers/stmemsc/stmemsc_mdelay.c
  )

  set(lsm6dso_compat_sources
    ${PICO_SDK_COMMON_DIR}/compat/zephyr_device.c
    ${PICO_SDK_COMMON_DIR}/compat/zephyr_gpio.c
    ${PICO_SDK_COMMON_DIR}/compat/sensor_default_rtio.c
    ${PICO_SDK_COMMON_DIR}/compat/zephyr_sensor_clock.c
  )

  target_sources(${target} PRIVATE
    ${lsm6dso_driver_sources}
    ${lsm6dso_compat_sources}
    ${PICO_SDK_STMEMSC_DIR}/lsm6dso_reg.c
    ${PICO_SDK_COMPAT_PORT_DIR}/gpio_os_rp2040.c
    ${lsm6dso_dir}/port/pico_sdk/lsm6dso_board.c
  )

  set(lsm6dso_prelude_sources ${lsm6dso_driver_sources} ${lsm6dso_compat_sources})

  # The driver's directories; port/pico_sdk before port/zephyr, so that this
  # build's lsm6dso_instance.h is found.
  target_include_directories(${target} PRIVATE
    ${lsm6dso_dir}/include
    ${lsm6dso_dir}/internal
    ${lsm6dso_dir}/port
    ${lsm6dso_dir}/port/pico_sdk
    ${lsm6dso_dir}/port/zephyr
    ${PICO_SDK_COMMON_DIR}/drivers/stmemsc
    ${PICO_SDK_STMEMSC_DIR}
  )

  target_compile_definitions(${target} PRIVATE CONFIG_LSM6DSO_STREAM=1)

  target_link_libraries(${target}
    hardware_gpio
    hardware_irq
  )

  set_source_files_properties(${lsm6dso_prelude_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${PICO_SDK_PRELUDE}"
  )
  # The warnings of -Wextra that the shared sources give (see above).
  set_property(SOURCE ${lsm6dso_dir}/port/zephyr/lsm6dso_sensor.c APPEND PROPERTY
    COMPILE_OPTIONS -Wno-unused-parameter -Wno-sign-compare -Wno-cast-function-type
  )
  set_property(SOURCE ${lsm6dso_dir}/port/zephyr/lsm6dso_rtio.c
    ${PICO_SDK_COMMON_DIR}/compat/sensor_default_rtio.c APPEND PROPERTY
    COMPILE_OPTIONS -Wno-unused-parameter -Wno-sign-compare
  )
  set_source_files_properties(${PICO_SDK_STMEMSC_DIR}/lsm6dso_reg.c PROPERTIES
    COMPILE_OPTIONS "-w"
  )
endfunction()
