# SPDX-License-Identifier: MIT
# The "sensor" command of the builds on the Pico SDK: the Zephyr build's sensor shell
# with the same subcommands, syntax and output.  include() this file from
# the application's CMakeLists.txt, then call
#
#   pico_sdk_sensor_shell(<target>)
#
# after pico_sdk_zephyr_compat() and pico_sdk_lsm6dso()
# (common/pico_sdk/drivers/compat/compat.cmake, common/pico_sdk/drivers/lsm6dso/
# lsm6dso.cmake: the layer with its shell, RTIO and RTIO work queue, the
# device lsm6dsox with its FIFO stream and the default iodev and decoder of
# the sensor API).  It adds to the target the NuttX build's sources of
# CONFIG_APP_SENSOR (nuttx/apps/sensor/Makefile), in its order:
#
# - the command (common/shell/sensor_shell.c, shared with the
#   Zephyr and NuttX builds: Zephyr v4.5.0's drivers/sensor/sensor_shell.c
#   with the LSM6DSO's FIFO attributes, the file of the Zephyr build's
#   patched tree; its SHELL_CMD_REGISTER() makes the
#   root z_shell_root_sensor for the application's table, and its RTIO
#   context sensor_read_rtio with the Zephyr build's pool,
#   RTIO_DEFINE_WITH_MEMPOOL(sensor_read_rtio, 8, 8, 32, 64, 4));
# - the copy of Zephyr's sensor_shell_stream.c (common/compat: "sensor
#   stream" and the processing thread sensor_shell_tid, Zephyr
#   priority 0, 3072 bytes, which the application starts with
#   K_THREAD_DEFINE_START());
# - the OS part on the layer (common/compat/os_generic: include/sensor_shell_os.h,
#   which the two include first, and sensor_shell_os.c, the wait of
#   "sensor get").
#
# The shared sources are compiled with the prelude of the shared sources
# (common_prelude.h), as in compat.cmake; so is sensor_shell_os.c, which
# includes Zephyr's sensor_shell.h (<zephyr/drivers/sensor.h>, whose
# structures depend on the prelude's CONFIG_SENSOR_ASYNC_API).  Include directories:
# common/compat (Zephyr's sensor_shell.h, and the LSM6DSO's
# <zephyr/drivers/sensor/lsm6dso.h>; sensor_shell_os.h is on the include path
# through compat.cmake).  sensor_shell.c is Zephyr's text, which under
# -Wextra compares signed with unsigned (15 places: an int index against
# ARRAY_SIZE() or a count, the sample window's int64_t against uint64_t)
# and leaves 3 parameters unused (device_is_sensor() without
# CONFIG_SENSOR_INFO, cmd_get_sensor_info(), data_ready_trigger_handler()):
# those two warnings are off for this file only, as in the NuttX host tests
# (nuttx/script/host_tests.sh).  sensor_shell_stream.c builds without
# warnings.


function(pico_sdk_sensor_shell target)
  set(sensor_shell_common_sources
    ${PICO_SDK_COMMON_DIR}/shell/sensor_shell.c
    ${PICO_SDK_COMMON_DIR}/compat/sensor_shell_stream.c
  )

  target_sources(${target} PRIVATE
    ${sensor_shell_common_sources}
    ${PICO_SDK_OS_GENERIC_DIR}/sensor_shell_os.c
  )

  target_include_directories(${target} PRIVATE
    ${PICO_SDK_COMMON_DIR}/compat
  )

  # The NuttX build's option of the "sensor" command, which the
  # application's start (common/imu/app_init.c) reads.
  target_compile_definitions(${target} PRIVATE
    CONFIG_APP_SENSOR=1
  )

  set_source_files_properties(${sensor_shell_common_sources}
    ${PICO_SDK_OS_GENERIC_DIR}/sensor_shell_os.c PROPERTIES
    COMPILE_OPTIONS "-include;${PICO_SDK_PRELUDE}"
  )
  set_property(SOURCE ${PICO_SDK_COMMON_DIR}/shell/sensor_shell.c APPEND PROPERTY
    COMPILE_OPTIONS -Wno-sign-compare -Wno-unused-parameter
  )
endfunction()
