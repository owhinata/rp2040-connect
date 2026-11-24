# SPDX-License-Identifier: MIT
# I2C0 with its DMA reads and, as an option, I2C1 on the Pico SDK, and
# the "i2c_dma" command.  include() this file from the application's
# CMakeLists.txt, then call
#
#   pico_sdk_i2c_dw(<target> [I2C1])
#
# after pico_sdk_zephyr_compat(<target>) (common/pico_sdk/drivers/compat/compat.cmake,
# whose layer and shell it needs).  It adds to the target:
#
# - the shared core of the controller (common/drivers/i2c_dw: i2c_dw.c, the
#   Zephyr build's i2c_dw.c with its RP2040 DMA reads) and the "i2c_dma"
#   command (common/shell/i2c_dma_shell.c, the Zephyr build's lines);
# - the core's binding: the OS's part (common/drivers/i2c_dw/port/<OS>,
#   PICO_SDK_OS: i2c_dw_os_impl.h, read first by the core) and the Pico SDK
#   part (common/drivers/i2c_dw/port/pico_sdk: i2c_dw_instance.h, read at its
#   end: the devices i2c0 and i2c1 with Zephyr's I2C API, the interrupts,
#   resets and pins; i2c_dw_dma_os_rp2040.c, the DMA reads' channels on the
#   SDK's hardware_dma, i2c_dw_dma_os.h of the core's port/), with the SDK's
#   libraries for them;
# - the controllers, as the NuttX build's Kconfig options of the same names
#   (I2C_DW_I2C0, I2C_DW_I2C1): CONFIG_I2C_DW_I2C0=1 always, and
#   CONFIG_I2C_DW_I2C1=1 with I2C1, for every source of the target;
# - the include directories of the core: include/ (i2c_dw_rp2040_dma.h: the
#   "i2c_dma" state, which <zephyr/drivers/i2c/i2c_dw_rp2040_dma.h> of
#   common/compat includes), internal/ (the core's own headers), port/ (the
#   two boundaries), port/<OS> and port/pico_sdk (the binding's headers,
#   among them i2c_dw_controllers_rp2040.h, the start of the controllers).
#
# The application calls i2c_dw_i2c0_initialize() and, with I2C1,
# i2c_dw_i2c1_initialize() once at the start, before the scheduler
# (i2c_dw_controllers_rp2040.h).  Each shared source is compiled with the
# prelude of the shared sources (common_prelude.h), as in compat.cmake;
# i2c_dw_dma_os_rp2040.c, a source of the binding, without it.

function(pico_sdk_i2c_dw target)
  cmake_parse_arguments(ARG "I2C1" "" "" ${ARGN})

  # i2c0 is always built: the LSM6DSOX is on it (lsm6dso.cmake
  # checks this), as the NuttX build's LSM6DSO_LOCAL depends on I2C_DW_I2C0.
  target_compile_definitions(${target} PRIVATE CONFIG_I2C_DW_I2C0=1)
  if(ARG_I2C1)
    target_compile_definitions(${target} PRIVATE CONFIG_I2C_DW_I2C1=1)
  endif()

  set(i2c_dw_common_sources
    ${PICO_SDK_COMMON_DIR}/drivers/i2c_dw/i2c_dw.c
    ${PICO_SDK_COMMON_DIR}/shell/i2c_dma_shell.c
  )

  target_sources(${target} PRIVATE
    ${PICO_SDK_COMMON_DIR}/drivers/i2c_dw/port/pico_sdk/i2c_dw_dma_os_rp2040.c
    ${i2c_dw_common_sources}
  )

  target_include_directories(${target} PRIVATE
    ${PICO_SDK_COMMON_DIR}/drivers/i2c_dw/include
    ${PICO_SDK_COMMON_DIR}/drivers/i2c_dw/internal
    ${PICO_SDK_COMMON_DIR}/drivers/i2c_dw/port
    ${PICO_SDK_COMMON_DIR}/drivers/i2c_dw/port/${PICO_SDK_OS}
    ${PICO_SDK_COMMON_DIR}/drivers/i2c_dw/port/pico_sdk
  )

  target_link_libraries(${target}
    hardware_dma
    hardware_gpio
    hardware_irq
    hardware_resets
    hardware_sync
  )

  set_source_files_properties(${i2c_dw_common_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${PICO_SDK_PRELUDE}"
  )
endfunction()
