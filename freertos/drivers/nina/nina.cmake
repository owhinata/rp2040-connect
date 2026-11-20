# SPDX-License-Identifier: MIT
# The SPI link to the NINA-W102 module on the FreeRTOS build, the "nina"
# command, and the module's sockets and name resolution.  include() this
# file from the application's CMakeLists.txt, then call
#
#   freertos_nina(<target>)
#
# after freertos_zephyr_compat(<target>) (freertos/drivers/compat/compat.cmake,
# whose layer and shell it needs).  It adds to the target:
#
# - the shared core of the link (common/drivers/nina/nina_spi.c:
#   nina_request() and nina_ip_config(), the WiFiNINA SPI protocol with the
#   Zephyr build's framing, timeouts and lazy reset of the module) and the
#   "nina version" command (common/shell/nina_shell.c, the Zephyr
#   build's text), and the waits of the socket loops
#   (common/drivers/nina/nina_wait.c);
# - the sockets: the shared socket table and commands
#   (common/drivers/nina/nina_sock.c) and resolver (nina_dns.c), and the
#   core's Zephyr binding of them, compiled as the Zephyr build compiles it
#   (common/drivers/nina/zephyr/nina_socket.c: Zephyr's socket offload, the
#   wait loops, the resolver registered as the offload's with its results on
#   a 2048-byte k_heap; NINA_OS_DNS_OFFLOAD), on the layer's socket part
#   (freertos/drivers/compat/compat.cmake): the time synchronization, the
#   MQTT transport, Telnet and the speed test call the layer's zsock_*();
# - the two boundaries of the core on FreeRTOS: the core's FreeRTOS binding
#   common/drivers/nina/freertos/nina_os.c (nina_os.h: the locks, requests,
#   sleeps, clock and the waits on the lines' edge interrupts, on FreeRTOS
#   and the Pico SDK's hardware_gpio, hardware_irq and hardware_timer; the
#   application calls its nina_os_initialize() of nina_freertos.h first) and
#   nina_bus.c (nina_bus.h: SPI1 and the module's lines on the Pico SDK's
#   hardware_spi and hardware_gpio), with those libraries;
# - the NINA passthrough mode and this build's "nina" root with its
#   "passthrough" subcommand (nina_passthrough.c: USB CDC <-> the module's
#   UART for esptool, a start of its own that main() runs on request;
#   common/shell/nina_shell.h merges the subcommand in), on hardware_uart,
#   hardware_irq and hardware_watchdog;
# - the include directories freertos/drivers/nina (nina_passthrough.h),
#   common/drivers/nina (nina_os.h, nina_bus.h, nina_proto.h, nina_cmd.h,
#   nina_sock.h, nina_dns.h, nina_passthrough_logic.h) and
#   common/drivers/nina/freertos (nina_freertos.h); common/shell is on the
#   include path through compat.cmake.
#
# CONFIG_NINA_TCP_MAX_IO (3584), CONFIG_NINA_SEND_RETRY_MS (1) and
# CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY (40) are the Zephyr build's values, in
# the prelude of the shared sources (common_prelude.h).  The core's POSIX
# names are the build's freertos/drivers/libc/include, the binding's net_*
# names the layer's <zephyr/net/net_ip.h> on them (the Zephyr build's
# common/drivers/nina/zephyr/posix maps the other way).
#
# The core's waits for the module's READY line sleep until an edge interrupt
# of the line (GPIO10; the binding's own handler of IO_IRQ_BANK0) instead of
# sampling it (nina_os_ready_*() of nina_os.h).
#
# The socket loops (zephyr/nina_socket.c) wait between their passes with
# common/drivers/nina/nina_wait.c: they wait for the rising edges of the
# module's IRQ line (GPIO2, the same handler) when its firmware reports
# socket events (nina_proto.h, NINA_CAP_SOCKET_EVENTS); with an older
# firmware they sleep their poll intervals.
#
# The application calls nina_bus_initialize() once at the start, before
# the tasks (the Zephyr build initializes spi1 at boot); the module itself
# is reset by the first request.  Each shared source is compiled with the
# prelude of the shared sources (common_prelude.h), as in compat.cmake.

get_filename_component(FREERTOS_NINA_DIR ${CMAKE_CURRENT_LIST_DIR} ABSOLUTE)

function(freertos_nina target)
  set(nina_common_sources
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_spi.c
    ${FREERTOS_COMMON_DIR}/shell/nina_shell.c
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_sock.c
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_dns.c
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_wait.c
    ${FREERTOS_COMMON_DIR}/drivers/nina/zephyr/nina_socket.c
  )

  target_sources(${target} PRIVATE
    ${FREERTOS_COMMON_DIR}/drivers/nina/freertos/nina_os.c
    ${FREERTOS_NINA_DIR}/nina_bus.c
    ${FREERTOS_NINA_DIR}/nina_passthrough.c
    ${nina_common_sources}
  )

  target_include_directories(${target} PRIVATE
    ${FREERTOS_NINA_DIR}
    ${FREERTOS_COMMON_DIR}/drivers/nina
    ${FREERTOS_COMMON_DIR}/drivers/nina/freertos
  )

  target_link_libraries(${target}
    hardware_gpio
    hardware_irq
    hardware_resets
    hardware_spi
    hardware_timer
    hardware_uart
    hardware_watchdog
  )

  set_source_files_properties(${nina_common_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${FREERTOS_COMPAT_DIR}/common_prelude.h"
  )
  # The resolver registered as the offload's, its results on the binding's
  # heap (nina_os.h), as in the Zephyr build (zephyr/app/usb-console/
  # CMakeLists.txt).
  set_source_files_properties(
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_sock.c
    ${FREERTOS_COMMON_DIR}/drivers/nina/nina_dns.c
    ${FREERTOS_COMMON_DIR}/drivers/nina/zephyr/nina_socket.c
    PROPERTIES COMPILE_DEFINITIONS NINA_OS_DNS_OFFLOAD
  )
endfunction()
