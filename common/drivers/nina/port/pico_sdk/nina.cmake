# SPDX-License-Identifier: MIT
# The SPI link to the NINA-W102 module on the Pico SDK, the "nina"
# command, and the module's sockets and name resolution.  include() this
# file from the application's CMakeLists.txt, then call
#
#   pico_sdk_nina(<target>)
#
# after pico_sdk_zephyr_compat(<target>) (common/compat/port/pico_sdk/compat.cmake,
# whose layer and shell it needs).  It adds to the target:
#
# - the shared core of the link (common/drivers/nina/internal/nina_proto.c:
#   nina_proto_request() and nina_proto_ip_config(), the WiFiNINA SPI protocol with the
#   Zephyr build's framing, timeouts and lazy reset of the module) and the
#   "nina version" command (common/shell/nina_shell.c, the Zephyr
#   build's text), and the waits of the socket loops
#   (common/drivers/nina/internal/nina_wait.c);
# - the Wi-Fi operations (common/drivers/nina/nina_wifi.c: scan, connect,
#   disconnect, status, the link monitor) and their Zephyr binding on the
#   layer (common/drivers/nina/port/zephyr/nina_net.c: Zephyr's Wi-Fi
#   management and interface, the thread nina_wifi_tid); pico_sdk_wifi()
#   (common/compat/port/pico_sdk/wifi.cmake) adds the layer's network part that
#   they run on;
# - the sockets: the shared socket table and commands
#   (common/drivers/nina/nina_sock.c) and resolver (nina_dns.c), and the
#   core's Zephyr binding of them, compiled as the Zephyr build compiles it
#   (common/drivers/nina/port/zephyr/nina_socket.c: Zephyr's socket offload, the
#   wait loops, the resolver registered as the offload's with its results on
#   a 2048-byte k_heap; NINA_OS_DNS_OFFLOAD), on the layer's socket part
#   (common/compat/port/pico_sdk/compat.cmake): the time synchronization, the
#   MQTT transport, Telnet and the speed test call the layer's zsock_*();
# - the two boundaries of the core: the OS's binding
#   common/drivers/nina/port/<OS>/nina_os.c (PICO_SDK_OS; nina_os.h: the
#   locks, requests, sleeps, clock and the waits on the lines' edge
#   interrupts, on the OS and the Pico SDK's hardware_gpio, hardware_irq and
#   hardware_timer) and the Pico SDK's port/pico_sdk/nina_bus_rp2040.c
#   (nina_bus.h: SPI1 and the module's lines on the Pico SDK's hardware_spi
#   and hardware_gpio; the long transfers by DMA in nina_bus_dma_rp2040.c,
#   on hardware_dma and hardware_irq, with the OS's waits of
#   port/pico_sdk/nina_bus_dma_os.h in port/<OS>/nina_bus_dma_os_impl.h),
#   with those libraries;
# - the NINA passthrough mode (USB CDC <-> the module's UART for esptool, a
#   start of its own that main() runs on request; "nina passthrough" of
#   nina_shell.c, CONFIG_NINA_PASSTHROUGH of the prelude): the shared core
#   common/drivers/nina/nina_passthrough.c, its host's side on TinyUSB
#   common/drivers/nina/port/pico_sdk/nina_passthrough_os.c and its module's
#   side nina_passthrough_bus_rp2040.c there (UART1, the module's lines,
#   the watchdog's scratch registers, the LED), on hardware_uart,
#   hardware_irq and hardware_watchdog.  The core's receive path runs from RAM, as the
#   bus's interrupt handler (NINA_PASSTHROUGH_ISR_SECTION);
# - the include directories common/drivers/nina (nina_sock.h, nina_dns.h,
#   nina_passthrough.h), its internal/ (nina_proto.h, nina_wait.h), its port/
#   (nina_os.h, nina_bus.h and the passthrough mode's two boundaries), its
#   port/<OS> and port/pico_sdk (the bindings' headers); common/shell is on the include path through compat.cmake.
#
# CONFIG_NINA_TCP_MAX_IO (3584), CONFIG_NINA_SEND_RETRY_MS (1) and
# CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY (40) are the Zephyr build's values, in
# the prelude of the shared sources (common_prelude.h).  The core's POSIX
# names are the build's common/compat/libc_net/include, the binding's net_*
# names the layer's <zephyr/net/net_ip.h> on them (the Zephyr build's
# common/drivers/nina/port/zephyr/posix maps the other way).
#
# The core's waits for the module's READY line sleep until an edge interrupt
# of the line (GPIO10; the binding's own handler of IO_IRQ_BANK0) instead of
# sampling it (nina_os_ready_*() of nina_os.h).
#
# The socket loops (port/zephyr/nina_socket.c) wait between their passes with
# common/drivers/nina/internal/nina_wait.c: they wait for the rising edges of the
# module's IRQ line (GPIO2, the same handler) when its firmware reports
# socket events (nina_proto.h, NINA_CAP_SOCKET_EVENTS); with an older
# firmware they sleep their poll intervals.
#
# The core sets up SPI1 and resets the module at its first request; the
# application calls nothing of the driver at its start.  Each shared source is compiled with the
# prelude of the shared sources (common_prelude.h), as in compat.cmake.

function(pico_sdk_nina target)
  set(nina_common_sources
    ${PICO_SDK_COMMON_DIR}/drivers/nina/internal/nina_proto.c
    ${PICO_SDK_COMMON_DIR}/shell/nina_shell.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/nina_wifi.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/port/zephyr/nina_net.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/nina_sock.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/nina_dns.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/internal/nina_wait.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/port/zephyr/nina_socket.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/nina_passthrough.c
  )

  target_sources(${target} PRIVATE
    ${PICO_SDK_COMMON_DIR}/drivers/nina/port/${PICO_SDK_OS}/nina_os.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/port/pico_sdk/nina_passthrough_os.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/port/pico_sdk/nina_bus_rp2040.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/port/pico_sdk/nina_bus_dma_rp2040.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/port/pico_sdk/nina_passthrough_bus_rp2040.c
    ${nina_common_sources}
  )

  target_include_directories(${target} PRIVATE
    ${PICO_SDK_COMMON_DIR}/drivers/nina/include
    ${PICO_SDK_COMMON_DIR}/drivers/nina/internal
    ${PICO_SDK_COMMON_DIR}/drivers/nina/port
    ${PICO_SDK_COMMON_DIR}/drivers/nina/port/${PICO_SDK_OS}
    ${PICO_SDK_COMMON_DIR}/drivers/nina/port/pico_sdk
  )

  # The Zephyr binding's option (port/zephyr/Kconfig), whose threads the
  # application's start (common/imu/app_init.c) starts.
  target_compile_definitions(${target} PRIVATE
    CONFIG_WIFI_NINA_LOCAL=1
  )

  target_link_libraries(${target}
    hardware_dma
    hardware_gpio
    hardware_irq
    hardware_resets
    hardware_spi
    hardware_timer
    hardware_uart
    hardware_watchdog
  )

  set_source_files_properties(${nina_common_sources} PROPERTIES
    COMPILE_OPTIONS "-include;${PICO_SDK_PRELUDE}"
  )
  # The resolver registered as the offload's, its results on the binding's
  # heap (nina_os.h), as in the Zephyr build (zephyr/app/usb-console/
  # CMakeLists.txt).
  set_source_files_properties(
    ${PICO_SDK_COMMON_DIR}/drivers/nina/nina_sock.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/nina_dns.c
    ${PICO_SDK_COMMON_DIR}/drivers/nina/port/zephyr/nina_socket.c
    PROPERTIES COMPILE_DEFINITIONS NINA_OS_DNS_OFFLOAD
  )
  # The core's part of UART1's interrupt in RAM, as the Pico SDK's
  # __not_in_flash_func() places it.
  set_source_files_properties(
    ${PICO_SDK_COMMON_DIR}/drivers/nina/nina_passthrough.c
    PROPERTIES COMPILE_DEFINITIONS "NINA_PASSTHROUGH_ISR_SECTION=\".time_critical.nina_passthrough_uart_isr\""
  )
endfunction()
