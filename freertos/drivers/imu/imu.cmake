# SPDX-License-Identifier: MIT
# The IMU acquisition service of the FreeRTOS build and its "imu" command
# (start, stop, status, cal), shared with the Zephyr and NuttX builds.
# include() this file from the application's CMakeLists.txt, then call
#
#   freertos_imu(<target>)
#
# after freertos_zephyr_compat(), freertos_kv() and freertos_lsm6dso()
# (freertos/drivers/compat/compat.cmake, freertos/drivers/kv/kv.cmake,
# freertos/drivers/lsm6dso/lsm6dso.cmake: the layer, the key-value store and
# the device lsm6dsox with its FIFO stream).  It adds to the target the
# NuttX build's sources of CONFIG_NANO_APP_IMU without CONFIG_NANO_IMU_RAW_MQTT
# and CONFIG_NANO_TIME_SYNC (nuttx/apps/imu/Makefile), in its order:
#
# - the service (common/imu: imu_stream.c, the "imu" command and the thread
#   imu_stream_tid, which the application starts with
#   K_THREAD_DEFINE_START(); the batches imu_raw_batch.c, imu_raw_packet.c,
#   imu_raw_time.c, imu_tick_clock.c, imu_raw_queue.c), and the calibration
#   (imu_cal.c, "imu cal" and the lookup of the stored values for the
#   driver, imu_cal_init(); imu_cal_codec.c), each with the prelude of the
#   shared sources (common_prelude.h, -include), as on NuttX;
# - STUBS for the MQTT transport (imu_raw_mqtt_null.c: every batch is
#   counted as dropped, nothing is sent; a warning in "dmesg" at every
#   start) and for the time synchronization (time_sync_null.c: never
#   synchronized; time_sync_mono_us() is the real clock), copies of the
#   NuttX build's.  The build's MQTT transport replaces the first with
#   common/imu/imu_raw_mqtt.c and imu_raw_status.c (and the MQTT client), its
#   time synchronization the second with common/imu/time_sync.c and
#   time_offset.c (and the "ntp" command);
# - the part of the layer they need: hwinfo_get_device_id()
#   (freertos/drivers/compat/rp2040/zephyr_hwinfo.c, on pico_unique_id);
#   the address conversions of <zephyr/net/socket.h> and newlib's missing
#   network headers are the layer's (compat.cmake);
# - the include directories: common/imu and common/drivers/lsm6dso (already
#   the driver's).
#
# The shared sources build without warnings under the application's -Wall
# -Wextra, so no option of theirs is turned off.

get_filename_component(FREERTOS_IMU_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

function(freertos_imu target)
  set(imu_common_sources
    ${FREERTOS_COMMON_DIR}/imu/imu_stream.c
    ${FREERTOS_COMMON_DIR}/imu/imu_raw_batch.c
    ${FREERTOS_COMMON_DIR}/imu/imu_raw_packet.c
    ${FREERTOS_COMMON_DIR}/imu/imu_raw_time.c
    ${FREERTOS_COMMON_DIR}/imu/imu_tick_clock.c
    ${FREERTOS_COMMON_DIR}/imu/imu_raw_queue.c
    ${FREERTOS_COMMON_DIR}/imu/imu_cal.c
    ${FREERTOS_COMMON_DIR}/imu/imu_cal_codec.c
  )

  target_sources(${target} PRIVATE
    ${imu_common_sources}
    ${FREERTOS_IMU_DIR}/imu_raw_mqtt_null.c
    ${FREERTOS_IMU_DIR}/time_sync_null.c
    ${FREERTOS_COMPAT_DIR}/rp2040/zephyr_hwinfo.c
  )

  target_include_directories(${target} PRIVATE
    ${FREERTOS_COMMON_DIR}/imu
    ${FREERTOS_COMMON_DIR}/drivers/lsm6dso
  )

  target_link_libraries(${target}
    pico_unique_id
  )

  set_source_files_properties(${imu_common_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h"
  )
endfunction()
