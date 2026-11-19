# SPDX-License-Identifier: MIT
# I2C0 and its DMA reads on the FreeRTOS build, and the "i2c_dma" command.
# include() this file from the application's CMakeLists.txt, then call
#
#   freertos_i2c_dw(<target> [SELFTEST])
#
# after freertos_zephyr_compat(<target>) (freertos/drivers/compat/compat.cmake,
# whose layer and shell it needs).  It adds to the target:
#
# - the shared core of the controller and its DMA (common/drivers/i2c_dw:
#   i2c_dw.c, the Zephyr build's i2c_dw.c with its RP2040 DMA reads, and
#   dma_rpi_pico.c, the Zephyr build's dma_rpi_pico.c) and the "i2c_dma" command
#   (common/shell/i2c_dma_shell.c, the Zephyr build's lines);
# - the core's boundary on FreeRTOS and the Pico SDK (this directory:
#   i2c_dw_os_impl.h, read first by both core files; i2c_dw_instance.h and
#   dma_rpi_pico_instance.h, read at their ends: the device i2c0 with
#   Zephyr's I2C API, the interrupts, resets and pins; dma_rpi_pico_os.c,
#   the claim of the DMA channels in the SDK's record), with the SDK's
#   libraries for them;
# - the include directories: this directory (i2c_dw_i2c0.h, the boundary's
#   parts) and common/drivers/i2c_dw (dma_rpi_pico.h, i2c_dw_rp2040_dma.h:
#   the "i2c_dma" state, which <zephyr/drivers/i2c/i2c_dw_rp2040_dma.h> of
#   common/compat includes);
# - with SELFTEST, the board's check of I2C0 (i2c_dw_selftest.c, part of the
#   "selftest" command).
#
# The application calls dma_rpi_pico_initialize() and then
# i2c_dw_i2c0_initialize() once at the start, before the scheduler
# (i2c_dw_i2c0.h).  Each shared source is compiled with the prelude of the
# shared sources (common_prelude.h), as in compat.cmake.

get_filename_component(FREERTOS_I2C_DW_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

function(freertos_i2c_dw target)
  cmake_parse_arguments(ARG "SELFTEST" "" "" ${ARGN})

  set(i2c_dw_common_sources
    ${FREERTOS_COMMON_DIR}/drivers/i2c_dw/i2c_dw.c
    ${FREERTOS_COMMON_DIR}/drivers/i2c_dw/dma_rpi_pico.c
    ${FREERTOS_COMMON_DIR}/shell/i2c_dma_shell.c
  )

  target_sources(${target} PRIVATE
    ${FREERTOS_I2C_DW_DIR}/dma_rpi_pico_os.c
    ${i2c_dw_common_sources}
  )

  if(ARG_SELFTEST)
    target_sources(${target} PRIVATE ${FREERTOS_I2C_DW_DIR}/i2c_dw_selftest.c)
  endif()

  target_include_directories(${target} PRIVATE
    ${FREERTOS_I2C_DW_DIR}
    ${FREERTOS_COMMON_DIR}/drivers/i2c_dw
  )

  target_link_libraries(${target}
    hardware_dma
    hardware_gpio
    hardware_irq
    hardware_resets
    hardware_sync
  )

  set_source_files_properties(${i2c_dw_common_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h"
  )
endfunction()
