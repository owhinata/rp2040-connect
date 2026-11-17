# SPDX-License-Identifier: MIT
# The Telnet shell of the FreeRTOS build (freertos/drivers/telnet/
# nano_telnet.c): the USB console's shell on TCP port 23 for one client at
# a time, the Zephyr build's CONFIG_SHELL_BACKEND_TELNET.  include() this
# file from the application's CMakeLists.txt, then call
#
#   freertos_telnet(<target>)
#
# after freertos_zephyr_compat() and freertos_nina() (the layer with its
# shell and consoles, the NINA's sockets).  The application calls
# nano_telnet_init() in its task "main" once the NINA's interface is set up
# and before the threads that use the network start (see main.c).

get_filename_component(FREERTOS_TELNET_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

function(freertos_telnet target)
  target_sources(${target} PRIVATE
    ${FREERTOS_TELNET_DIR}/nano_telnet.c
  )

  target_include_directories(${target} PRIVATE
    ${FREERTOS_TELNET_DIR}
  )
endfunction()
