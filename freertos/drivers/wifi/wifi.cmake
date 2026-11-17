# SPDX-License-Identifier: MIT
# Wi-Fi on the FreeRTOS build: the NINA-W102's network interface, the
# net_mgmt requests and events, the stored credentials with the automatic
# (re)connection, and the "wifi" command.  include() this file from the
# application's CMakeLists.txt, then call
#
#   freertos_wifi(<target>)
#
# after freertos_zephyr_compat(), freertos_kv() and freertos_nina()
# (freertos/drivers/compat/compat.cmake, freertos/drivers/kv/kv.cmake,
# freertos/drivers/nina/nina.cmake: the layer and its shell, the settings
# on the key-value area, the NINA link).  It adds to the target what the
# NuttX build has with CONFIG_NANO_NINA_NET, CONFIG_NANO_WIFI_CREDENTIALS
# and CONFIG_NANO_WIFI_PROFILE (nuttx/drivers/nina/Make.defs,
# nuttx/drivers/compat/Make.defs, nuttx/apps/wifi/Makefile):
#
# - the NINA's network interface: the driver (common/drivers/nina/
#   nina_net.c: the Wi-Fi operations and the threads nina_scan_tid,
#   nina_connect_tid and nina_monitor_tid) and its interface on FreeRTOS
#   (freertos/drivers/nina/nina_netif.c: nina_net_initialize());
# - the layer's network part: the interface (freertos/drivers/compat/
#   zephyr_net_if.c, <zephyr/net/net_if.h>), the net_mgmt events
#   (common/compat/zephyr_net_mgmt.c with the thread "net_mgmt" of
#   freertos/drivers/compat/zephyr_net_mgmt_os.h), the Wi-Fi requests
#   (common/compat/zephyr_wifi_mgmt.c) and helpers (zephyr_wifi_utils.c);
# - the stored credentials: the copies of Zephyr's wifi_credentials.c and
#   its settings backend (common/compat; the entries are the settings
#   "wifi_cred/<n>" on the key-value area, as in the Zephyr and NuttX
#   builds), with CONFIG_NANO_WIFI_CREDENTIALS for every source of the target
#   (it adds NET_REQUEST_WIFI_CONNECT_STORED to <zephyr/net/wifi_mgmt.h>);
#   the configuration is the Zephyr build's (2 entries, 128-byte password
#   room, 30 s connection timeout: the defaults of
#   <zephyr/net/wifi_credentials.h>);
# - the Wi-Fi profile (common/imu/wifi_profile.c: the migration of the old
#   profile, the thread wifi_reconnect_tid; its FreeRTOS part
#   wifi_profile_os.h here);
# - the "wifi" command: connect, disconnect, scan and status
#   (common/compat/zephyr_wifi_shell.c, the part of Zephyr's wifi_shell.c the
#   driver serves; wifi_shell_init() at boot) and "wifi cred" (the unchanged
#   copy of Zephyr's wifi_credentials_shell.c in
#   common/compat/subsys/net/lib/wifi_credentials);
# - the "ifconfig" command (nano_ifconfig.c here: the NuttX build's lines
#   for wlan0, on the interface's state);
# - the include directories: this directory, freertos/drivers/nina
#   (nina_net.h), common/imu (wifi_profile.h, kv_store.h).
#
# The application calls nina_net_initialize(), wifi_credentials_init() and
# wifi_shell_init() in its task "main", starts the threads with
# K_THREAD_DEFINE_START() and calls wifi_profile_init() after
# kv_store_init(), in the order of the Zephyr build's boot (see main.c).
#
# Each shared source is compiled with the prelude of the shared sources
# (common_prelude.h), as in compat.cmake.  The copy of Zephyr's
# wifi_credentials_shell.c is compiled unchanged, so the warnings of the
# application's -Wall -Wextra that its code gives are turned off for that
# file only: it prints size_t and uint32_t values with %d (-Wno-format;
# the values are small, and size_t, uint32_t and int are all 32 bits and
# passed alike on this target) and its list handler ignores its arguments
# (-Wno-unused-parameter).  Zephyr's build does not enable -Wextra.

get_filename_component(FREERTOS_WIFI_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

function(freertos_wifi target)
  set(wifi_common_sources
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_net.c
    ${FREERTOS_COMMON_DIR}/compat/zephyr_net_mgmt.c
    ${FREERTOS_COMMON_DIR}/compat/zephyr_wifi_mgmt.c
    ${FREERTOS_COMMON_DIR}/compat/zephyr_wifi_utils.c
    ${FREERTOS_COMMON_DIR}/compat/wifi_credentials.c
    ${FREERTOS_COMMON_DIR}/compat/wifi_credentials_backend_settings.c
    ${FREERTOS_COMMON_DIR}/imu/wifi_profile.c
    ${FREERTOS_COMMON_DIR}/compat/zephyr_wifi_shell.c
  )

  # Copy of Zephyr's file (v4.5.0, unchanged; common/tests/run.sh compares
  # it with the Zephyr tree).
  set(wifi_zephyr_copies
    ${FREERTOS_COMMON_DIR}/compat/subsys/net/lib/wifi_credentials/wifi_credentials_shell.c
  )

  target_sources(${target} PRIVATE
    ${FREERTOS_COMPAT_DIR}/zephyr_net_if.c
    ${FREERTOS_NINA_DIR}/nina_netif.c
    ${FREERTOS_WIFI_DIR}/nano_ifconfig.c
    ${wifi_common_sources}
    ${wifi_zephyr_copies}
  )

  target_include_directories(${target} PRIVATE
    ${FREERTOS_WIFI_DIR}
    ${FREERTOS_NINA_DIR}
    ${FREERTOS_COMMON_DIR}/imu
  )

  target_compile_definitions(${target} PRIVATE
    CONFIG_NANO_WIFI_CREDENTIALS=1
  )

  set(prelude "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h")
  set_source_files_properties(${wifi_common_sources} PROPERTIES
    COMPILE_OPTIONS "${prelude}"
  )
  set_source_files_properties(${FREERTOS_COMMON_DIR}/compat/wifi_credentials.c PROPERTIES
    COMPILE_OPTIONS "${prelude};-Wno-type-limits"
  )
  set_source_files_properties(
    ${FREERTOS_COMMON_DIR}/compat/wifi_credentials_backend_settings.c PROPERTIES
    COMPILE_OPTIONS "${prelude};-Wno-sign-compare"
  )
  set_source_files_properties(${wifi_zephyr_copies} PROPERTIES
    COMPILE_OPTIONS "${prelude};-Wno-format;-Wno-unused-parameter"
  )
endfunction()
