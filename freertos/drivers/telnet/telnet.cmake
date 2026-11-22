# SPDX-License-Identifier: MIT
# The Telnet shell (common/compat/shell_engine/shell_telnet.c, the shell
# engine's, on the layer and Zephyr's sockets): the USB console's shell on
# TCP port 23 for one client at a time, the Zephyr build's
# CONFIG_SHELL_BACKEND_TELNET.  include() this file from the application's
# CMakeLists.txt, then call
#
#   freertos_telnet(<target>)
#
# after freertos_zephyr_compat() and freertos_nina() (the layer with its
# shell engine, whose include directory has shell_telnet.h; the NINA's
# sockets).  The application calls shell_telnet_init() in its task "main"
# once the NINA's interface is set up and before the threads that use the
# network start (see main.c).

function(freertos_telnet target)
  target_sources(${target} PRIVATE
    ${FREERTOS_SHELL_ENGINE_DIR}/shell_telnet.c
  )
endfunction()
