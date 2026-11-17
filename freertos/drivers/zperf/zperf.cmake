# SPDX-License-Identifier: MIT
# The "zperf" command of the FreeRTOS build: the TCP tests of the iperf3
# protocol both ways (the PC runs "iperf3 -s" or "iperf3 -c <board>"), with
# the syntax and output of the Zephyr build's zperf.  include() this file
# from the application's CMakeLists.txt, then call
#
#   freertos_zperf(<target>)
#
# after freertos_zephyr_compat() and freertos_nina() (the layer with its
# shell, the NINA's sockets).  It adds the port's minimal iperf3
# (common/compat/zperf_iperf3.c, the layer's <zephyr/net/zperf.h>) and the
# command (common/compat/zephyr_zperf_shell.c, the TCP part of Zephyr's
# zperf_shell.c), each with the prelude of the shared sources
# (common_prelude.h), as in compat.cmake.  The application starts the
# server's thread with K_THREAD_DEFINE_START(zperf_server_tid) and lists
# the command in its table (NANO_SHELL_CMD(zperf)).

function(freertos_zperf target)
  set(zperf_common_sources
    ${FREERTOS_COMMON_DIR}/compat/zperf_iperf3.c
    ${FREERTOS_COMMON_DIR}/compat/zephyr_zperf_shell.c
  )

  target_sources(${target} PRIVATE
    ${zperf_common_sources}
  )

  set_source_files_properties(${zperf_common_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h"
  )
endfunction()
