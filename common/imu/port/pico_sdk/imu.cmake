# SPDX-License-Identifier: MIT
# The IMU acquisition service of the builds on the Pico SDK and its "imu" command
# (start, stop, status, cal), shared with the Zephyr and NuttX builds.
# include() this file from the application's CMakeLists.txt, then call
#
#   pico_sdk_imu(<target>)
#
# after pico_sdk_zephyr_compat(), pico_sdk_kv() and pico_sdk_lsm6dso()
# (common/compat/port/pico_sdk/compat.cmake, common/compat/port/pico_sdk/kv.cmake,
# common/drivers/lsm6dso/port/pico_sdk/lsm6dso.cmake: the layer, the key-value store and
# the device lsm6dsox with its FIFO stream), with pico_sdk_nina() and
# pico_sdk_wifi() in the same target (the sockets of the MQTT transport and
# of the time synchronization, the default gateway of the interface, the
# Wi-Fi events).  It adds to the target the NuttX build's sources of
# CONFIG_APP_IMU with CONFIG_IMU_RAW_MQTT and CONFIG_TIME_SYNC
# (nuttx/apps/imu/Makefile), in its order:
#
# - the service (common/imu: imu_stream.c, the "imu" command and the thread
#   imu_stream_tid, which the application starts with
#   K_THREAD_DEFINE_START(); the batches imu_raw_batch.c, imu_raw_packet.c,
#   imu_raw_time.c, imu_tick_clock.c, imu_raw_queue.c), and the calibration
#   (imu_cal.c, "imu cal" and the lookup of the stored values for the
#   driver, imu_cal_init(); imu_cal_codec.c), each with the prelude of the
#   shared sources (common_prelude.h, -include), as on NuttX;
# - the MQTT transport (common/imu/imu_raw_mqtt.c: the thread
#   imu_raw_mqtt_tid, which the application starts with
#   K_THREAD_DEFINE_START(); its JSON messages, imu_raw_status.c), on the
#   port's minimal MQTT client (common/imu/mqtt_min.c, the part of Zephyr's
#   MQTT library the transport uses, with the Zephyr build's settings,
#   mqtt_min.h) and the client's socket layer on the layer's zsock_*()
#   calls (common/imu/mqtt_min_sock.c, the NuttX build's file; the NINA's
#   common/drivers/nina/port/zephyr/nina_socket.c).  mqtt_min.c is Zephyr's text,
#   which compares signed with unsigned (13 places under -Wextra, e.g.
#   "(end - cur) < sizeof(uint8_t)"), as Zephyr's library does: the file is
#   compiled with -Wno-sign-compare, as in the host tests of the NuttX
#   build and of common/tests;
# - the time synchronization (common/imu/time_sync.c with its estimate,
#   time_offset.c; CONFIG_TIME_SYNC of the NuttX build): the thread
#   time_sync_tid, which the application starts with
#   K_THREAD_DEFINE_START() and whose Wi-Fi events time_sync_init() asks
#   for, and the "ntp" command (its SHELL_CMD_REGISTER() makes the root
#   z_shell_root_ntp for the application's table).  It reads the RP2040
#   TIMER with rp2040_timer_us() (common/chip/rp2040, on the application's
#   include path), the realtime clock of the layer's <zephyr/sys/clock.h>
#   (common/compat/port/pico_sdk/zephyr_sys_clock.c) and the CONFIG_NINA_NTP_*
#   values of the prelude;
# - the part of the layer they need: hwinfo_get_device_id()
#   (common/compat/port/pico_sdk/zephyr_hwinfo.c, on pico_unique_id);
#   the address conversions of <zephyr/net/socket.h> and newlib's missing
#   network headers are the layer's (compat.cmake);
# - the include directory common/imu (the LSM6DSO driver's
#   <zephyr/drivers/sensor/lsm6dso.h>, which the service reads, is
#   common/compat's).
#
# The service needs the transport's and the time synchronization's API in
# every build: without the MQTT transport, common/imu/imu_raw_mqtt_null.c
# takes the place of imu_raw_mqtt.c, imu_raw_status.c, mqtt_min.c and
# mqtt_min_sock.c (and CONFIG_IMU_RAW_MQTT is not defined); without the time
# synchronization, common/imu/time_sync_null.c that of time_sync.c and
# time_offset.c (no CONFIG_TIME_SYNC), as in the NuttX build.  This file
# builds both modules.
#
# Besides mqtt_min.c's -Wno-sign-compare, the shared sources build without
# warnings under the application's -Wall -Wextra.

function(pico_sdk_imu target)
  set(imu_common_sources
    ${PICO_SDK_COMMON_DIR}/imu/imu_stream.c
    ${PICO_SDK_COMMON_DIR}/imu/imu_raw_batch.c
    ${PICO_SDK_COMMON_DIR}/imu/imu_raw_packet.c
    ${PICO_SDK_COMMON_DIR}/imu/imu_raw_time.c
    ${PICO_SDK_COMMON_DIR}/imu/imu_tick_clock.c
    ${PICO_SDK_COMMON_DIR}/imu/imu_raw_queue.c
    ${PICO_SDK_COMMON_DIR}/imu/imu_cal.c
    ${PICO_SDK_COMMON_DIR}/imu/imu_cal_codec.c
    ${PICO_SDK_COMMON_DIR}/imu/imu_raw_mqtt.c
    ${PICO_SDK_COMMON_DIR}/imu/imu_raw_status.c
    ${PICO_SDK_COMMON_DIR}/imu/mqtt_min.c
    ${PICO_SDK_COMMON_DIR}/imu/mqtt_min_sock.c
  )

  target_sources(${target} PRIVATE
    ${imu_common_sources}
    ${PICO_SDK_COMMON_DIR}/imu/time_sync.c
    ${PICO_SDK_COMMON_DIR}/imu/time_offset.c
    ${PICO_SDK_COMPAT_PORT_DIR}/zephyr_hwinfo.c
  )

  target_include_directories(${target} PRIVATE
    ${PICO_SDK_COMMON_DIR}/imu
  )

  # The NuttX build's options of these modules, which the application's
  # start (common/imu/app_main.c, app_init.c) reads.
  target_compile_definitions(${target} PRIVATE
    CONFIG_APP_IMU=1
    CONFIG_IMU_RAW_MQTT=1
    CONFIG_TIME_SYNC=1
  )

  target_link_libraries(${target}
    pico_unique_id
  )

  set_source_files_properties(${imu_common_sources} ${PICO_SDK_COMMON_DIR}/imu/time_offset.c
    ${PICO_SDK_COMMON_DIR}/imu/time_sync.c PROPERTIES
    COMPILE_OPTIONS "-include;${PICO_SDK_PRELUDE}"
  )
  set_source_files_properties(${PICO_SDK_COMMON_DIR}/imu/mqtt_min.c PROPERTIES
    COMPILE_OPTIONS "-include;${PICO_SDK_PRELUDE};-Wno-sign-compare"
  )
endfunction()
