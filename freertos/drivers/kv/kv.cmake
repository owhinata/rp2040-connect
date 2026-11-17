# SPDX-License-Identifier: MIT
# The key-value store of the FreeRTOS build and its "kv" command, on the
# flash of the key-value area (the last 8 KiB of the 16 MiB flash,
# common/compat/nano_flash_kv.h), in the Zephyr build's format.  include()
# this file from the application's CMakeLists.txt, then call
#
#   freertos_kv(<target>)
#
# after freertos_zephyr_compat(<target>) (freertos/drivers/compat/compat.cmake,
# whose layer and shell it needs).  It adds to the target:
#
# - the store and the "kv" command (common/imu/kv_store.c: set, sethex, get,
#   delete; "kv wipe" of common/compat/kv_store_wipe.c, kv_store_os.h), the
#   hex conversions of the command (the copy of Zephyr's lib/utils/hex.c),
#   the flash device and flash map of the area (common/compat/zephyr_flash_kv.c)
#   and the copies of Zephyr's settings and NVS (common/compat/subsys:
#   settings.c, settings_store.c, settings_init.c, settings_nvs.c, nvs.c,
#   crc8_sw.c), as the NuttX build has them (nuttx/apps/imu/Makefile);
# - the boundary of the area on the Pico SDK (nano_flash_kv.c, with
#   hardware_flash and pico_flash), and PICO_FLASH_ASSUME_CORE1_SAFE=1 for
#   every source of the target, so that flash_safe_execute() only disables
#   the interrupts (core 1 is not used; without it the call always fails on
#   this build);
# - the linker script's parts (ld/, pico_add_linker_script_override_path()):
#   the FLASH region ends where the area starts, the area's symbols
#   __kv_start and __kv_end, and the check that the image ends before it;
# - the include directories: this directory (nano_flash_kv_os.h),
#   common/imu (kv_store.h), the settings' own headers
#   (<settings/settings_nvs.h>; settings_priv.h for kv_store_wipe.c).
#
# Each shared source is compiled with the prelude of the shared sources
# (common_prelude.h), as in compat.cmake.  The copies of Zephyr's settings
# and NVS are read with the Zephyr build's Kconfig values first
# (common/compat/zephyr_kv_config.h, -include); they test unset options with
# #if, which -Wundef would report (not part of -Wall -Wextra; also turned off
# here, as on NuttX).  They are compiled unchanged, so the warnings of the
# application's -Wall -Wextra that Zephyr's code gives are turned off for
# the one file that gives them, nvs.c: it prints uint32_t with %d, %u and
# %x (Zephyr's uint32_t is unsigned int, newlib's unsigned long; both are
# 32 bits and passed alike, so the output is right: -Wno-format, as on
# NuttX), and -Wextra's -Wsign-compare and -Wunused-parameter (one each),
# which Zephyr's build does not enable.  The copy of hex.c, like the
# port's own files, needs neither the Kconfig values nor any of this.

get_filename_component(FREERTOS_KV_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

function(freertos_kv target)
  set(kv_common_sources
    ${FREERTOS_COMMON_DIR}/imu/kv_store.c
    ${FREERTOS_COMMON_DIR}/compat/kv_store_wipe.c
    ${FREERTOS_COMMON_DIR}/compat/zephyr_flash_kv.c
    ${FREERTOS_COMMON_DIR}/compat/lib/utils/hex.c
  )

  # Copies of Zephyr's files (v4.5.0, unchanged; common/tests/run.sh
  # compares them with the Zephyr tree).
  set(kv_zephyr_copies
    ${FREERTOS_COMMON_DIR}/compat/subsys/settings/src/settings.c
    ${FREERTOS_COMMON_DIR}/compat/subsys/settings/src/settings_store.c
    ${FREERTOS_COMMON_DIR}/compat/subsys/settings/src/settings_init.c
    ${FREERTOS_COMMON_DIR}/compat/subsys/settings/src/settings_nvs.c
    ${FREERTOS_COMMON_DIR}/compat/subsys/crc/crc8_sw.c
  )
  set(kv_nvs ${FREERTOS_COMMON_DIR}/compat/subsys/kvss/nvs/nvs.c)

  target_sources(${target} PRIVATE
    ${FREERTOS_KV_DIR}/nano_flash_kv.c
    ${kv_common_sources}
    ${kv_zephyr_copies}
    ${kv_nvs}
  )

  target_include_directories(${target} PRIVATE
    ${FREERTOS_KV_DIR}
    ${FREERTOS_COMMON_DIR}/imu
    ${FREERTOS_COMMON_DIR}/compat/subsys/settings/include
    ${FREERTOS_COMMON_DIR}/compat/subsys/settings/src
  )

  target_compile_definitions(${target} PRIVATE
    PICO_FLASH_ASSUME_CORE1_SAFE=1
  )

  # The plain signature, as the SDK's examples and the application use it.
  target_link_libraries(${target}
    hardware_flash
    pico_flash
  )

  pico_add_linker_script_override_path(${target} ${FREERTOS_KV_DIR}/ld
    FILES pico_flash_region.ld section_extra_post_end.incl
  )

  set(prelude "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h")
  set(kv_config "-include;${FREERTOS_COMMON_DIR}/compat/zephyr_kv_config.h;-Wno-undef")
  set(nvs_warnings "-Wno-format;-Wno-sign-compare;-Wno-unused-parameter")
  set_source_files_properties(${kv_common_sources} PROPERTIES
    COMPILE_OPTIONS "${prelude}"
  )
  set_source_files_properties(${kv_zephyr_copies} PROPERTIES
    COMPILE_OPTIONS "${prelude};${kv_config}"
  )
  set_source_files_properties(${kv_nvs} PROPERTIES
    COMPILE_OPTIONS "${prelude};${kv_config};${nvs_warnings}"
  )
endfunction()
