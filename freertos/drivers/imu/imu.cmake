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
# the device lsm6dsox with its FIFO stream), with freertos_nina() and
# freertos_wifi() in the same target (the sockets of the MQTT transport and
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
#   (freertos/drivers/compat/zephyr_sys_clock.c) and the CONFIG_NINA_NTP_*
#   values of the prelude;
# - the part of the layer they need: hwinfo_get_device_id()
#   (freertos/drivers/compat/rp2040/zephyr_hwinfo.c, on pico_unique_id);
#   the address conversions of <zephyr/net/socket.h> and newlib's missing
#   network headers are the layer's (compat.cmake);
# - the include directory common/imu (the LSM6DSO driver's
#   <zephyr/drivers/sensor/lsm6dso.h>, which the service reads, is
#   common/compat's).
#
# Besides mqtt_min.c's -Wno-sign-compare, the shared sources build without
# warnings under the application's -Wall -Wextra.

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
    ${FREERTOS_COMMON_DIR}/imu/imu_raw_mqtt.c
    ${FREERTOS_COMMON_DIR}/imu/imu_raw_status.c
    ${FREERTOS_COMMON_DIR}/imu/mqtt_min.c
    ${FREERTOS_COMMON_DIR}/imu/mqtt_min_sock.c
  )

  target_sources(${target} PRIVATE
    ${imu_common_sources}
    ${FREERTOS_COMMON_DIR}/imu/time_sync.c
    ${FREERTOS_COMMON_DIR}/imu/time_offset.c
    ${FREERTOS_COMPAT_DIR}/rp2040/zephyr_hwinfo.c
  )

  target_include_directories(${target} PRIVATE
    ${FREERTOS_COMMON_DIR}/imu
  )

  target_link_libraries(${target}
    pico_unique_id
  )

  set_source_files_properties(${imu_common_sources} ${FREERTOS_COMMON_DIR}/imu/time_offset.c
    ${FREERTOS_COMMON_DIR}/imu/time_sync.c PROPERTIES
    COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h"
  )
  set_source_files_properties(${FREERTOS_COMMON_DIR}/imu/mqtt_min.c PROPERTIES
    COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h;-Wno-sign-compare"
  )
endfunction()
