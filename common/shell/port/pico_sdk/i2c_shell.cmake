# SPDX-License-Identifier: MIT
# The "i2c" command of the builds on the Pico SDK: the Zephyr build's I2C shell with
# the same subcommands, syntax and output.  include() this file from the
# application's CMakeLists.txt, then call
#
#   pico_sdk_i2c_shell(<target>)
#
# after pico_sdk_zephyr_compat() and pico_sdk_i2c_dw()
# (common/compat/port/pico_sdk/compat.cmake, common/drivers/i2c_dw/port/pico_sdk/
# i2c_dw.cmake: the layer with its shell and <zephyr/drivers/i2c.h>, the
# device i2c0 "i2c@40044000").  It adds to the target the command,
# common/shell/i2c_shell.c: Zephyr v4.5.0's
# drivers/i2c/i2c_shell.c with the local patch 0004 ("i2c scan" probes with
# one-byte reads on a controller that cannot send a write of no bytes), the
# file of the Zephyr build's patched tree (common/tests/run.sh compares the
# two).  Its SHELL_CMD_REGISTER() makes the root z_shell_root_i2c for the
# application's table.  The commands of "i2c target" are not built
# (CONFIG_I2C_TARGET, off in the Zephyr build).
#
# The source is compiled with the prelude of the shared sources
# (common_prelude.h), as in compat.cmake, and with the Zephyr build's
# CONFIG_I2C_SHELL_BUFFER_SIZE (64: the most bytes of one "i2c read" or
# "i2c write") and CONFIG_LOG_DEFAULT_LEVEL (3, its LOG_MODULE_REGISTER(),
# which logs nothing).  i2c_shell.c is Zephyr's text, which under -Wextra
# compares signed with unsigned and leaves the handlers' argc unused:
# those two warnings are off for this file only.

function(pico_sdk_i2c_shell target)
  set(i2c_shell_source ${PICO_SDK_COMMON_DIR}/shell/i2c_shell.c)

  target_sources(${target} PRIVATE ${i2c_shell_source})

  set_source_files_properties(${i2c_shell_source} PROPERTIES
    COMPILE_OPTIONS "-include;${PICO_SDK_PRELUDE};-Wno-sign-compare;-Wno-unused-parameter"
    COMPILE_DEFINITIONS "CONFIG_I2C_SHELL_BUFFER_SIZE=64;CONFIG_LOG_DEFAULT_LEVEL=3"
  )
endfunction()
