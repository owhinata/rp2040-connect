# The NINA-W102 driver (`nina`)

English | [日本語](README_ja.md)

The firmware of this repository is one firmware built on four OSes, Zephyr, NuttX, the Pico SDK with FreeRTOS and the
Pico SDK with ThreadX: the Zephyr, NuttX, FreeRTOS and ThreadX builds. This document describes the driver of the
NINA-W102 (an ESP32) that the four builds share.

The driver lets the firmware use the NINA-W102 as Wi-Fi, sockets and name resolution over SPI, and has a passthrough mode
for reading and writing the NINA's flash with esptool on the PC. Its body is a core with no OS of its own (the top of this
directory, `include/` and `internal/`); each OS writes only the bindings (`port/<OS>/`, `port/pico_sdk/`) that implement the
narrow boundaries of `port/`. The binding to Zephyr's socket and Wi-Fi management APIs (part of `port/zephyr/`) is also
built by the FreeRTOS and ThreadX builds, on their compatibility layer (the layer that provides Zephyr's APIs under Zephyr's
names, [`common/compat/`](../../compat/README.md)).

The module is expected to run Arduino NINA firmware 3.0.2 with patches 0001 to 0006 (see
[firmware/nina-fw](../../../firmware/nina-fw/README.md), which also tells how to build and write it); without 0003 to 0006 it
works without those additions ("Firmware additions" below). The design's summary is in §9.1 of the
[design document](../../../design/imu-raw-mqtt-design.md) ("The NINA driver", "The NINA passthrough", "The NINA
firmware's additions").

## Directories

| Place | Files | Role |
|---|---|---|
| top | `nina_wifi.c` | The Wi-Fi operations: connect (and its wait for the connection and the DHCP result), disconnect, scan, status, the link monitor; the body of the one thread `nina_wifi_tid`, which does them one at a time (the monitor's check every 2 s also during a scan or an attempt); what happens goes to the binding's callbacks (`struct nina_wifi_ops`) |
| top | `nina_sock.c` | The socket table (10 sockets) and the NINA's socket commands: `nina_socket_create()`, `do_bind()`, `do_listen()`, `do_accept()`, `do_connect()`, `do_close()` and `do_setsockopt()`, and for sending and receiving `udp_send()`, `do_send()`, `do_sendmsg()`, `udp_recv()` and `do_recv()` (one pass of a send or receive wait loop per request). `nina_sock_poll_revents()` (`do_poll()` for one socket) is used by the Zephyr, FreeRTOS and ThreadX builds only |
| top | `nina_dns.c` | Name resolution (the module's resolver) |
| top | `nina_passthrough.c` | The esptool passthrough: the restart's request, DTR and RTS on the module's lines, the rate, the data both ways with the 16 KiB buffer, the way out, the counts, the LED; on its two boundaries `nina_passthrough_bus.h` (the module's side) and `nina_passthrough_os.h` (the host's side) |
| `include/` | `nina_wifi.h`, `nina_dns.h`, `nina_passthrough.h` | The headers for the driver's users |
| `internal/` | `nina_proto.c` | Framing of SPI commands and replies (`nina_proto_request()`, `nina_proto_ip_config()`), the module's reset, the waits for READY |
| `internal/` | `nina_proto.h` | The names of the protocol's bytes (framing, the commands the driver sends, the values of their replies) and the declarations of `nina_proto.c`'s requests and of the module's capabilities |
| `internal/` | `nina_sock.h` | The declarations of `nina_sock.c` (for the bindings) |
| `internal/` | `nina_wait.c`, `nina_wait.h` | The waits between the passes of the socket loops (an interval, or the IRQ line's edge) |
| `port/` | `nina_os.h`, `nina_bus.h`, `nina_passthrough_os.h`, `nina_passthrough_bus.h` | The boundaries the OS and the board implement ("The boundaries the OS implements" below) |
| `port/zephyr/` | `nina_os.c`, `nina_bus.c`, `nina_net.c`, `nina_socket.c`, `nina_passthrough_os.c`, `nina_passthrough_bus_rp2040.c`, `posix/`, `Kconfig`, `dts/bindings/wifi/arduino,nina-spi.yaml` | The binding to Zephyr's APIs. The FreeRTOS and ThreadX builds compile `nina_net.c` and `nina_socket.c` too, on the compatibility layer |
| `port/nuttx/` | `nina_os.c`, `nina_bus_rp2040.c`, `nina_usrsock.c` and `nina_usrsock.h`, `nina_poller.c`, `nina_netdev.c`, `nina_wifi_mgmt.c` and `nina_wifi_mgmt.h`, `nina_net.h`, `nina_passthrough_os.c`, `nina_passthrough_bus_rp2040.c` | The binding to NuttX's APIs |
| `port/pico_sdk/` | `nina.cmake`, `nina_bus_rp2040.c`, `nina_bus_dma_rp2040.c` and `nina_bus_dma_rp2040.h`, `nina_bus_dma_os.h`, `nina_board_rp2040.h`, `nina_passthrough_os.c`, `nina_passthrough_bus_rp2040.c` | The binding shared by the OSes on the Pico SDK (the FreeRTOS and ThreadX builds) |
| `port/freertos/` | `nina_os.c`, `nina_bus_dma_os_impl.h` | FreeRTOS's part (locks and waits) |
| `port/threadx/` | `nina_os.c`, `nina_bus_dma_os_impl.h` | ThreadX's part (locks and waits) |

The `nina` command (`nina version`, `nina passthrough`) is `common/shell/nina_shell.c`, used by the Zephyr, FreeRTOS and
ThreadX builds (the NuttX build has `nuttx/apps/nina`, with the same syntax and output). What is the chip's and the board's
is in the same place as the binding, with `_rp2040` in its name (SPI and the lines, `nina_bus_rp2040.c`; the passthrough's
module side, `nina_passthrough_bus_rp2040.c`). The Zephyr build takes SPI and the lines from the devicetree, so its
`nina_bus.c` does not depend on the chip.

## How the core works

### Division of work

Wi-Fi and TCP/IP are handled by the NINA firmware (lwIP). The RP2040 only issues SPI commands to manage the Wi-Fi
connection and sockets; it never sees packets. To Zephyr this is a Wi-Fi management API plus socket offload.

### SPI transfer layer

- Commands are handled one at a time. A request is handled synchronously in the calling thread, and the NINA lock
  (`nina_os_lock()`, recursive with priority inheritance) serializes requests. CS stays asserted from the start of a phase
  until its last byte, including the wait for READY. The bus is taken only between `nina_bus_select()` and
  `nina_bus_deselect()`.
- The NINA is reset once, on the first request (10 ms, 750 ms). At boot the binding only sets up SPI1 and its pins and
  drives CS inactive (the core calls `nina_bus_initialize()` at its first request).
- SPI runs in mode 0, 8 bits, MSB first, at `CONFIG_NINA_SPI_FREQUENCY` (8 MHz by default). The PL022 divides clk_peri
  (125 MHz) by an even number, so it runs at 7.8125 MHz.
- A transfer of 64 bytes or more (the long reads in 128-byte pieces, the socket writes' parameters) goes by DMA while the
  calling thread sleeps; a shorter one, the core's single bytes among them, is polled. The four builds draw the same line
  (how each does it: "The bindings of each OS" below).
- Changes of the module's READY line (GPIO10) are waited for on a GPIO edge interrupt. The interrupt is enabled before
  READY is read, so a change after the read is not missed; it is off during the transfers. Past the deadline (2 s before a
  command, 10 s for the reply (2 s for `nina_proto_ip_config()`), 20 ms after CS is asserted) the result is
  `-ETIMEDOUT`. Whether the interrupt is used is decided at run time: after the first reset `nina_os_ready_start()`
  registers READY's callback, and if that fails READY is read every 100 µs instead (same deadlines and results).
- One TCP send carries at most 3584 bytes (`CONFIG_NINA_TCP_MAX_IO`), the most that fits the module's command buffer. A
  RAW batch of up to 2448 bytes goes out in one command.

### Sockets

Every socket uses the BSD-style socket commands of firmware 3.0.2 (0x70 to 0x7f: socket, close, bind, listen, accept,
connect, send, recv, sendto, recvfrom, ioctl, poll, getsockopt). Each command is a single call of an lwIP socket function
inside the module, and sockets are independent of each other. The table and the commands are `nina_sock.c`; the bindings
turn them into the OS's socket calls (the Zephyr, FreeRTOS and ThreadX builds: Zephyr's socket offload
`port/zephyr/nina_socket.c`; the NuttX build: usrsock).

- **Ten sockets**, the limit of the module's lwIP (`CONFIG_LWIP_MAX_SOCKETS=10`), shared by the Telnet listener, MQTT, NTP
  and the speed test. An eleventh gets `EMFILE`.
- **Always non-blocking.** Commands run inside the module's SPI handler, so a blocking call stalls everything including
  Wi-Fi management. The NINA's socket is created on the first bind or connect and made non-blocking as soon as it is
  created, and the waits for connect completion, readable data and send space repeat the poll command every 20 ms (the
  calling thread asks in Zephyr's offload, the thread `nina_event` in the NuttX build; with a firmware that reports socket
  events, on the edges of the IRQ line instead: see "Firmware additions" below).
- **A reply cannot tell a failure from zero bytes.** Receiving polls first (readable with zero bytes means the peer
  closed). When a TCP send returns zero, a poll command `CONFIG_NINA_SEND_RETRY_MS` (1 ms by default) later checks whether
  the socket is writable, and the data goes again only once it is (the full data is not sent over SPI at every try; this
  does not depend on the firmware). The poll's error flag tells a broken connection. Connect results are read from
  SO_ERROR.
- **TCP connect** waits up to 10 s. A send fails when no buffer space appears for 10 s; this is why a vanished access
  point takes about 9 s to be recognized as a disconnection.
- **UDP** carries up to 512 bytes per datagram. What does not fit the receive buffer is dropped by lwIP, which keeps
  datagram boundaries. A connected UDP socket has datagrams from other senders discarded by the driver. A receive takes at
  most 512 bytes.
- `bind` accepts the any address (0.0.0.0) only, IPv4 only. Multicast is not supported.
- Two socket options (`setsockopt`) are accepted.
  - The receive timeout (`SO_RCVTIMEO`) applies to the driver's own receive wait (in the NuttX build NuttX handles it). It
    is not passed to the module.
  - `TCP_NODELAY` is accepted but **has no effect**. The module's command cannot set TCP-level options, so Nagle's
    algorithm stays on. The iperf3 client asks for it on its control connection and gives up if it is refused, which is
    the only reason it is accepted.

  Any other option returns `ENOPROTOOPT`.
- Not verified: several TCP clients at once, ten sockets at once, and the timeout path when a server does not answer.

### Name resolution

`nina_dns.c` resolves a name with the NINA's resolver commands (0x34 with the name, 0x35 to read the address) to one IPv4
address. A numeric address is not sent to the NINA. A lock (`nina_os_dns_lock()`) covers the two commands (the NINA keeps
one result). The only user is the time synchronization when its NTP server is given by name (the default is the gateway).
The Zephyr, FreeRTOS and ThreadX builds register it as the socket offload's `getaddrinfo()` (`NINA_OS_DNS_OFFLOAD`, with
the results on a 2048-byte heap of its own); the NuttX build uses it through functions of its own ("NuttX" below).

### Wi-Fi

- Open and WPA2-PSK networks on 2.4 GHz. Open networks have not been tried on hardware. WPA, WPA3 and WEP are refused by
  the core (`-ENOTSUP`).
- The operations run in one thread (`nina_wifi_tid`). Every 2 seconds, while connected, the monitor reads the NINA's state
  (0x20), also during a scan or a connection attempt (its check runs within their polls' sleeps); after two disconnected
  readings in a row it treats the link as lost, clears the IP configuration, raises the disconnect event and asks for a
  reconnection. A connection hands the SSID (and passphrase) to the NINA, then reads the state every 500 ms up to 60 times;
  once associated it turns power saving off (0x17), reads the IP configuration (0x21) into the interface, and raises the
  result event (failures as `-ETIMEDOUT` and so on). A connection whose request generation
  (`generation`/`request_generation`) has changed ends without a result. The passphrase is never logged.
- Power saving is switched off on every connection and reconnection, because with it sending can stall for hundreds of
  milliseconds. Confirming that the setting took effect needs the patched firmware.
- Credentials are stored with Zephyr's `wifi cred` (in the format of Settings on NVS). At boot and after an unexpected
  disconnection the driver keeps trying the stored settings. It does not reconnect after a manual disconnection
  (`wifi disconnect`): the core asks `wifi_profile_connection_allowed()` (`common/imu/wifi_profile.c`) for every connection
  request and refuses one from the reconnection thread with `-ECANCELED` after a disconnection asked for. With several
  stored networks, which one is used is not guaranteed.
- Firmware 3.0.2 stops Wi-Fi while scanning, so the connection drops briefly (the driver treats it as a disconnection and
  connects again after the scan). A firmware that keeps the link does not drop it (next section). At most 10 results.
- At boot, the interface's initialization reads the MAC address (0x22) and the capabilities from the NINA
  (`nina_wifi_init()`). That is the first command to the NINA, so the NINA reset happens here.

### Firmware additions (patches 0003 to 0006)

A firmware with the patches 0003 to 0006 of `firmware/nina-fw/` answers command 0x6A with the bits of what it adds (the
specification: "Host interface of 0003 to 0006" in [`firmware/nina-fw/README.md`](../../../firmware/nina-fw/README.md)).
The driver asks once, when the interface is initialized (after reading the MAC address), and switches its behaviour bit by
bit. A firmware without the command (3.0.2 and the like) answers `0xEF 0x00 0xEE`; after those three bytes the driver takes
it as "no additions" and works without them. The second line of `nina version` shows the module's bits and those in use.
The four builds behave the same.

| Bit | Addition | What the driver does |
|---|---|---|
| 0 | A scan while connected keeps the link | A scan while connected is not treated as a disconnection (the address stays, no disconnection event, no reconnection). A loss of the link during the scan is found by the link monitor (every 2 s) as any other |
| 1 | Asynchronous scan | 0x36 starts it, 0x6B is asked every 100 ms, 0x27 reads the results when it is done. The NINA lock is held for one command at a time, so sending goes on during the scan. A scan still running after 10 s fails (`-ETIMEDOUT`) without 0x27. Could not start or failed: `-EIO`; stopped by a connection or disconnection: `-ECANCELED` |
| 2 | Socket events on the IRQ line | The socket waits (connect, receive, `poll`, the send retries, the NuttX build's `nina_event`) wake on GPIO2's rising edge interrupt instead of every 20 ms (1 ms for sends), asking again after 100 ms at most |

How bit 2 is used (`internal/nina_wait.h`; the passes of the NuttX build's `nina_event` are under "NuttX" below):

- As the firmware's rule says: the edge count is noted before a command that may arm an event (poll, recv, accept, send,
  connect); after a "not yet" answer the driver waits until the count changes or 100 ms pass, and asks again. An edge from
  before the command costs one more ask; a lost edge costs at most 100 ms (the asking keeps it correct).
- One line serves every socket, so each edge wakes every waiting thread (the binding counts the edges and wakes every
  waiting thread). A thread whose wake-ups keep finding nothing rests a little before it waits again (1, 2, 4, 8, 16 ms,
  then 20 ms).
- A send waits 1 ms after a pass that sent data and got zero bytes, as without bit 2, and for the edge after a pass that
  only polled (the socket was not writable). No wait goes past the call's deadline (`SO_RCVTIMEO` for receiving, the `poll`
  timeout, 10 s for connect and send).
- When the edge interrupt cannot be used (`nina_os_event_start()` fails to register the IRQ line's callback or to enable
  its edge), the waits keep the intervals whatever the firmware, and bit 2 is dropped from the bits in use.

### The esptool passthrough (`nina passthrough confirm`)

`nina passthrough confirm` restarts the board into a mode that relays the USB serial port (CDC ACM) to the NINA's UART (the
ESP32's UART0, which its ROM bootloader uses), so that esptool on the PC reads and writes the NINA's flash, instead of
Arduino's SerialNINAPassthrough sketch (the RP2040's firmware stays). The mode itself (the request's mark, the lines, the
rate, the data both ways with the buffer, the way out, the counts, the LED) is `nina_passthrough.c`; its host side (the
USB serial port, the clock) and its module side (UART1, the module's lines, the watchdog's scratch registers) are each OS's
bindings. `CONFIG_NINA_PASSTHROUGH` (on by default).

| RP2040 | Passthrough mode | Normal firmware |
|---|---|---|
| GPIO8 | UART1 TX -> the module's UART0 RX | SPI1 RX (MISO) |
| GPIO9 | UART1 RX <- the module's UART0 TX | CS |
| GPIO2 | the module's GPIO0 (boot strap) | IRQ |
| GPIO3 | the module's RESETN (active low) | RESET |
| GPIO6 | the LED | the LED |

- Wiring: UART1 uses the SPI link's MISO and CS, so the NINA's SPI cannot run meanwhile. In every build the passthrough is
  a start of its own, apart from the normal start, and does not start the NINA's SPI.
- Lines: RTS set releases the reset, DTR set pulls GPIO0 low (the sketch's mbed branch). Opening the port starts the ROM's
  download mode, closing it holds the module in reset. When the reset is released with DTR clear (the NINA's own firmware
  starts), GPIO0 is released after 750 ms and GPIO8 is not driven. UART1's rate follows the host's line coding (8N1).
- The way in: the command leaves a request in the watchdog's scratch registers 0..3 and restarts with the watchdog
  (`rp2040_watchdog_reboot()` of `common/chip/rp2040`). The start reads and clears it at once, so RESET and any later
  restart are normal starts. Nothing is written to the flash (the key-value store).
- No byte lost: module to PC, UART1's receive interrupt moves the bytes into a 16 KiB buffer (allocated on entering the
  mode), which holds 178 ms at 921600 bps while the PC does not read; what does not fit is counted. PC to module, the
  host's receive side stops while the UART's FIFO cannot take more (the host waits), so nothing is lost.
- The way out: close the port at 300 bps (for example `stty -F /dev/ttyACM0 300`), or RESET. The NINA is started again into
  its own firmware (GPIO0 high, RESETN low for 10 ms, GPIO0 released after 750 ms), then the watchdog restarts the board.
- The RAM log (`dmesg` afterwards) gets a line at the start, each time the port closes and at the way out (bytes each way,
  bytes dropped while USB was not configured, bytes lost to a full buffer and the most waiting, the UART's overruns, breaks
  and framing errors, the rate).
- esptool 5.3.0 does not connect with `--before no-reset`. Use `custom_reset_sequence = R0|D1|W0.2|R1|W0.6` in
  `ESPTOOL_CFGFILE` and `--before default-reset --after no-reset`, which keep RTS set (the procedure is in "Reading and
  writing the NINA firmware" in the [root README](../../../README.md)).

### TLS

TLS is not supported. These points are unresolved.

- **Random source**: the build uses Zephyr's non-cryptographic test random generator, and the RP2040 has no cryptographic
  random source. The ATECC608A on the board has its configuration and data unlocked and returned the same answer to two
  Random commands, so it cannot be used as it is. Getting the ESP32's hardware random numbers over SPI needs a new firmware
  command.
- **Certificate verification**: TLS on the module cannot verify the host name for connections made by IP address, and
  certificate date checking is not enabled there.
- **Time**: checking certificate validity needs trustworthy time, and NTP replies are not authenticated.

An implementation must not fall back silently to a test random generator when no proper source is available.

### Looking at the radio link

[Tracing TCP ACKs with a Wi-Fi monitor](../../../design/wifi-monitor-debug.md) describes how to examine sending delays and
ACK behaviour at the level of radio frames.

## The boundaries the OS implements

What the core needs from the OS and the board are the headers of `port/`. Each is written in the terms of the Zephyr calls
the core used before it was shared.

| Boundary | What it must do |
|---|---|
| `port/nina_os.h` | `nina_os_lock()`, `nina_os_unlock()`: a recursive lock with priority inheritance (the Zephyr build's `K_MUTEX_DEFINE(nina_lock)`); `nina_os_lock()` returns 0 or a negative errno, which `nina_proto_request()` and `nina_proto_ip_config()` return untouched. `nina_os_wifi_lock()`, `nina_os_wifi_unlock()`: the Wi-Fi operations' lock. `nina_os_wifi_post()`, `nina_os_wifi_wait()`: the Wi-Fi thread's wake-up (at most one count; the wait ends at a post or after its time, rounded up to ticks plus one tick; the core keeps its requests itself). `nina_os_dns_lock()`, `nina_os_dns_unlock()`: a lock with priority inheritance held across the two resolver commands. `nina_os_sleep_us()`, `nina_os_sleep_ms()`: sleep for the time rounded up to ticks plus one tick, as Zephyr's `k_sleep()` (not ended by signals). `nina_os_uptime_ms()`: a clock in milliseconds, rounded down (`k_uptime_get()`). `nina_os_ready_start()`, `nina_os_ready_arm()`, `nina_os_ready_wait()`, `nina_os_ready_disarm()`: waiting for a change of READY on its interrupt (`nina_os_ready_start()` registers the callback; if it fails, the core samples READY every 100 µs). `nina_os_event_start()`, `nina_os_event_seq()`, `nina_os_event_wait()`: counting the rising edges of the IRQ line (GPIO2) and waking every waiting thread (if `nina_os_event_start()` fails, the waits keep their intervals) |
| `port/nina_bus.h` | `nina_bus_initialize()`: called by the core at its first request, sets up the SPI and its pins and drives CS inactive (leaves READY, RESET and IRQ alone; a later call does nothing); `-ENODEV` when the bus cannot be used (while the passthrough mode runs). `nina_bus_pin_configure()`, `nina_bus_pin_set()`, `nina_bus_pin_get()`: logical values (1 is active); the board knows each line's polarity and pull (CS GPIO9 active low, READY GPIO10 active low with pull-up, RESET GPIO3 active low, IRQ GPIO2 active high with pull-down). `nina_bus_select()`, `nina_bus_deselect()`: for one phase of a request (the command, the reply), take the bus from its other users where the OS has them (Zephyr's and NuttX's bus locks), set it to the module's mode and rate and drive CS active; `nina_bus_deselect()` drives CS inactive and gives the bus back. Between the two the bus is the module's. `nina_bus_transceive()`: a synchronous transfer in mode 0, 8 bits, MSB first (only a write when RX is NULL), without touching CS |
| `port/nina_passthrough_os.h` | The passthrough's host side: the OS and its USB serial port (the clock, the host's writes, the log). The binding also implements `nina_passthrough_run()` and calls the core at its events (`nina_passthrough.h`) |
| `port/nina_passthrough_bus.h` | The passthrough's module side: the request's words and the restart, the module's RESETN and GPIO0, the UART to the module's UART0, the LED. It is the chip's and the board's, so each OS implements it for its chip (on the RP2040's registers or its SDK). The receive side's functions are also called from the UART's interrupt |
| Configuration | `CONFIG_NINA_TCP_MAX_IO` (default 3584) and `CONFIG_NINA_SEND_RETRY_MS` (default 1), read by `nina_sock.h`, which uses the defaults without them |
| C library | `nina_sock.[ch]` and `nina_dns.[ch]` use the types and constants of POSIX's `<sys/socket.h>`, `<sys/types.h>`, `<sys/uio.h>`, `<netinet/in.h>`, `<netinet/tcp.h>`, `<arpa/inet.h>`, `<poll.h>` and `<netdb.h>` (`struct sockaddr_in`, `struct iovec`, `struct addrinfo`, `AF_INET`, `MSG_DONTWAIT`, `POLLIN`, `EAI_*` and so on) and `inet_pton()`, `htons()`, `malloc()` and `free()`. The `POLL*` bits of `nina_sock_poll_revents()` and the `EAI_*` values are those of the same headers as the callers' |

The OSes on the Pico SDK also implement the wait of the long transfers by DMA, `port/pico_sdk/nina_bus_dma_os.h`, in
`port/<OS>/nina_bus_dma_os_impl.h` ("Pico SDK" below).

## The bindings of each OS

### Zephyr

The core's binding to Zephyr's APIs is `port/zephyr/`. The application's `CMakeLists.txt` builds it from there; the
binding's devicetree binding is found through `DTS_ROOT` and its options through the application's `Kconfig` (`rsource`).

| File | Role |
|---|---|
| `nina_os.c` | Zephyr's side of `nina_os.h`: mutexes, the Wi-Fi thread's wake-up (a semaphore with a timed take), sleeps, clock, the READY interrupt, the waits for the IRQ line's (GPIO2) edges |
| `nina_bus.c` | Zephyr's side of `nina_bus.h`: the devicetree's `spi1` and the four lines (CS, READY, RESET, IRQ), Zephyr's GPIO and SPI calls; with `CONFIG_NINA_PASSTHROUGH`, no bus while `nina_passthrough_active()` is true (the esptool passthrough) |
| `nina_net.c` | Zephyr's Wi-Fi management and network interface on the core: the device (`NET_DEVICE_DT_OFFLOAD_DEFINE()`), Zephyr's parameters checked and passed on, the results as Zephyr's structures and events, the address, netmask, gateway and dormant state on the interface, the Wi-Fi profile's reconnection as the core's hooks, the thread `nina_wifi_tid` (`K_THREAD_DEFINE()`, 2048 bytes, priority 8) |
| `nina_socket.c` | Socket offload (file descriptors, errno, `O_NONBLOCK`, `SO_RCVTIMEO`, the wait loops and their sleeps, poll), and the resolver registered as the `getaddrinfo` offload |
| `nina_passthrough_os.c` | Zephyr's side of the passthrough's `nina_passthrough_os.h`: the CDC ACM port and the USB stack's messages, the 1 ms tick, the start (`SYS_INIT`) and `main()`'s way out |
| `nina_passthrough_bus_rp2040.c` | The passthrough's module side (`nina_passthrough_bus.h`) on the RP2040's registers: the watchdog's scratch registers and restart, UART1 with its interrupt, GPIO2, GPIO3, GPIO8, the LED |
| `posix/` | Headers that map the POSIX socket names the core uses (`struct sockaddr_in`, `AF_INET`, `POLLIN`, ...) to Zephyr's (read by the core and `nina_socket.c` only) |
| `Kconfig`, `dts/bindings/wifi/arduino,nina-spi.yaml` | The driver's options (`CONFIG_WIFI_NINA_LOCAL`, `CONFIG_NINA_SPI_FREQUENCY`, `CONFIG_NINA_TCP_MAX_IO`, `CONFIG_NINA_SEND_RETRY_MS`) and the devicetree binding of the `nina_w102` node. `CONFIG_NINA_PASSTHROUGH` is in the application's `Kconfig` |

Zephyr's Telnet shell, MQTT client and zperf run on these sockets.

- **SPI**: the frequency is `spi-max-frequency` of the NINA's devicetree node; the SPI bus and the chip select come from
  that node's parent and `reg` too. The PL022 driver runs in its DMA mode (`CONFIG_SPI_PL022_DMA`, with `spi1`'s `dmas` in
  `app.overlay`: channels 10 and 11, requested at boot; the I2C0 reads take free channels from 0 up). A transfer of
  `CONFIG_SPI_PL022_DMA_MIN_LEN` bytes (64) or more goes through the two channels while the thread waits for their
  interrupt; a shorter one is polled as without the DMA (the local patch `zephyr/0007-spi-pl022-dma-min-len.patch`). The
  bus stays the module's while CS is low in both paths: `nina_bus_select()` takes the driver's lock with `SPI_LOCK_ON`
  through a transfer of no bytes, which the threshold keeps on the polled path (in the DMA mode without one, that transfer
  would start a DMA; `nina_bus.c` does not build that way). A DMA error (the driver logs
  `<err> spi_pl022: dma:... callback gets error`) or a transfer that does not end within its length's time plus 200 ms
  (`CONFIG_SPI_COMPLETION_TIMEOUT_TOLERANCE`; `Timeout waiting for transfer complete`; a byte lost to an RX overrun ends
  this way) fails the request with that error; the patch then stops the channels.
- **Waiting for READY** (`nina_os.c`): each wait lasts at most 10 ms, and READY is read again after it.
- **Sockets**: Zephyr's file descriptor table is sized with `CONFIG_ZVFS_OPEN_MAX=16`. By default its size follows
  `CONFIG_NET_MAX_CONTEXTS`, so lowering only that value leaves too few descriptors and accepting a Telnet client then
  fails over and over. Zephyr's own TCP/IP stack carries no traffic (`net conn` stays empty). Its packet buffers and
  connection tables are kept at a minimum in `prj.conf`. The calling thread does the waiting (connect polls every 20 ms for
  up to 10 s, poll every 20 ms; MQTT's input every 100 ms, the time synchronization's receive every 1 ms while it waits for
  a reply). `poll` does not fail as a whole when asking the module about one socket fails: that socket gets `POLLERR`. A
  socket that another thread closes while the poll runs gets `POLLNVAL`. A `poll` that includes a descriptor other than a
  NINA socket looks again at least every 20 ms even with bit 2 (for that descriptor, as without bit 2).
- **Passthrough mode**: the host side is `nina_passthrough_os.c` on Zephyr's USB device stack, the module side
  `nina_passthrough_bus_rp2040.c` on the RP2040's registers.
  - The start reads and clears the request at `POST_KERNEL` 0.
  - Zephyr starts as usual in this mode, but the NINA core sees no SPI bus (`nina_bus_initialize()` fails): the Wi-Fi
    interface's initialization, the Telnet server's socket, `nina version` and the like fail with `-ENODEV` without
    touching SPI1 or the NINA's lines (two `<err>` lines of the Telnet listener stay in the RAM log). `main()` runs the relay
    without settings, calibration, time synchronization or a Wi-Fi connection. The CDC ACM interrupt callback is replaced by
    the relay's (`POST_KERNEL` 99), so the shell gets no input and its output does not reach the port. The log goes to RAM
    only.
  - UART1's receive interrupt runs at priority 0, at 4 bytes in the FIFO; the buffer comes from `malloc()`. PC to module,
    one packet (64 bytes) at a time; what the UART's FIFO cannot take is written by its transmit interrupt, and the CDC ACM
    receive side stops meanwhile. DTR, RTS and the rate are taken from the USB stack's messages (the system work queue) and
    checked every millisecond. The data move in the CDC ACM callback (on its own work queue, next item).
- **The CDC ACM work queue**: see [the Zephyr port](../../../design/zephyr-port.md#the-cdc-acm-work-queue).

### NuttX

The core's binding to NuttX's APIs is `port/nuttx/` (which also has the RP2040's SPI and lines, `nina_bus_rp2040.c`).
`nuttx/drivers/nina/` keeps only `Kconfig` and `Make.defs`, because NuttX builds `nuttx/drivers` as its `external/`
directory (`nuttx/nuttx/external` is a symlink to it, made by `script/setup.sh`): its `Makefile` includes `nina/Make.defs`
and its `Kconfig` sources `nina/Kconfig`. `nina/Make.defs` finds the `.c` files of the core and of the binding through
`VPATH` (paths relative to `nuttx/drivers`), and the objects are made in `nuttx/drivers`. Neither is compiled with the
prelude (the configuration values come in through `<nuttx/config.h>`, which NuttX's headers read). The core's headers are in
`include/`, `internal/` and `port/`, the binding's in `port/nuttx/`; the kernel side (`nina/Make.defs`), the board and the
`nina`, `imu` and `init` applications add those directories to their include paths.

| Boundary | NuttX implementation |
|---|---|
| `nina_os.h` | `port/nuttx/nina_os.c`. The Wi-Fi operations' lock is an `rmutex_t`, the Wi-Fi thread's wake-up one semaphore |
| `nina_bus.h` | `port/nuttx/nina_bus_rp2040.c` |
| `nina_passthrough_os.h` | `port/nuttx/nina_passthrough_os.c` (on cdcacm) |
| `nina_passthrough_bus.h` | `port/nuttx/nina_passthrough_bus_rp2040.c` (on the RP2040's registers) |
| Configuration | `nuttx/drivers/nina/Kconfig` |
| C library | NuttX's libc |

**SPI transfer layer**

- The SPI is NuttX's `rp2040_spi.c`. Each transfer takes `SPI_LOCK`; the first one sets mode 0, 8 bits and
  `CONFIG_NINA_SPI_FREQUENCY`. `SPI_SELECT` is not used.
- With `CONFIG_RP2040_SPI_DMA` (on in the `imu` configuration) the driver moves an exchange of more than
  `CONFIG_RP2040_SPI_DMATHRESHOLD` bytes (63, so from 64 bytes) through two channels of NuttX's DMA driver while the thread
  waits on the driver's semaphore (two completions, TX and RX, on DMA_IRQ_0; no time limit); shorter ones are polled. The
  driver takes the two channels at its initialization, the NINA's first request, with `rp2040_dmachannel_trywait()` (the
  local patch `nuttx/0013-rp2040-spi-dma-channels-trywait`: without two free channels every exchange is polled).
  `SPI_EXCHANGE()` has no result, so the core cannot see a DMA error; the driver reports one only with `spierr()`
  (`CONFIG_DEBUG_SPI_ERROR`, off).
- Waiting for READY: `nina_os_ready_arm()` sets the semaphore to 0, then enables the edge towards the desired level (ready:
  falling, busy: rising) through `common/compat/port/nuttx/rp2040_gpio_os.c` (`gpio_os_irq_configure()`, which clears an
  edge latched while the event was off before enabling it); the core reads READY after that. A change after the read stays
  in the semaphore, so the wait returns at once. The interrupt is GPIO bank 0's single IRQ, shared with the LSM6DSOX's INT1
  (GPIO24); a callback on the compatibility layer's `gpio0` only posts the semaphore (at most one count).
  `nina_os_ready_wait()` waits once with a timeout (the milliseconds rounded up to ticks, plus one tick), without slices of
  10 ms. The deadline (`nina_os_uptime_ms()`) and the timeout are both on the tickless tick read from the TIMER, so they do
  not drift apart while interrupts are masked (a flash erase). `nina_os_ready_disarm()` disables the edge, so CS toggling
  READY during the transfers raises no interrupts. `gpio0` and `rp2040_gpio_os.c` are built with `CONFIG_LSM6DSO_STREAM`,
  so `CONFIG_NINA` depends on it. The callback is added to `gpio0` by `nina_os_ready_start()` after the first reset.

**Sockets (usrsock)**

The NuttX build provides the sockets through usrsock, with `usrsock_request()` in the kernel (NuttX's
`CONFIG_NET_USRSOCK_CUSTOM`). NuttX's own TCP/IP stack is not used (`NET_TCP_NO_STACK`, `NET_UDP_NO_STACK`).

| File | Role |
|---|---|
| `nina_sock.c` and `internal/nina_sock.h` | The socket table and commands (the core) |
| `port/nuttx/nina_usrsock.c` | Dispatches usrsock's requests to the functions above and answers them (ACK) |
| `port/nuttx/nina_poller.c` | The thread `nina_event`: turns the state of the NINA's sockets into usrsock events, and times the retries of a waiting send; with a firmware that reports socket events, it wakes on the IRQ line's edges |
| `port/nuttx/nina_usrsock.h` | The declarations between the two |

- A request is handled synchronously in the calling thread, at its priority (as the Zephyr build's socket calls).
  usrsock passes one request at a time and holds the usrsock lock meanwhile; the NINA lock inside `nina_proto_request()`
  serializes the NINA commands. The lock order is always "usrsock lock, then NINA lock" (`nina_event` takes them in the
  same order).
- A socket's number (usrsock's usockid) is its place in the table (0 to 9). usrsock has 12 connections
  (`CONFIG_NET_USRSOCK_PREALLOC_CONNS`): the 10, plus an 11th request, the new connection of an `accept()` and a
  `SOCK_CTRL` attempt.
- Waiting: the Zephyr build waits in the caller. NuttX's usrsock waits for events from the driver, so `nina_event` makes the
  same queries (the NINA's poll command) every 20 ms and turns the answers into events: readable → `RECVFROM_AVAIL` (on a
  listening socket: a connection is waiting), writable → `SENDTO_READY`, the error flag or a failed poll of a connected TCP
  socket → `REMOTE_CLOSED`. A UDP socket's error flag is not passed on (the Zephyr build's datagram calls also carry on
  after it). A socket whose events usrsock already holds is not queried; with nothing to query the thread sleeps until a
  request wakes it. With a firmware that reports socket events, the queries come at the IRQ line's edges (100 ms at most)
  instead of every 20 ms.
- Every socket is handled so: the MQTT and time synchronization sockets (the compatibility layer's `zsock_*()` calls are
  NuttX's socket calls) as well as Telnet's, iperf's and the nsh commands'. `nina_event` runs below their threads'
  priorities (106; MQTT 107, time synchronization 109), so its polls come while they sleep or wait. The Zephyr build asks
  the NINA only from the threads that use a socket; here `nina_event` asks about each socket that has something to report,
  every 20 ms or at the IRQ line's edges.
- connect: the state is checked once right after the connect command (the first pass of the Zephyr build's loop); if the
  handshake is not done, the request is answered "in progress" and `nina_event` checks every 20 ms, up to 10 s after the
  connect command (`ETIMEDOUT`). Success is reported with `SENDTO_READY`, failure with the final answer to the connect
  request, so `connect()` sets the Zephyr build's errno (SO_ERROR 104/113/116/118 → `ECONNRESET`/`ECONNABORTED`/
  `ETIMEDOUT`/`EHOSTUNREACH`; anything else and the error flag → `ECONNREFUSED`). The failed NINA socket is closed.
- accept: the request comes once the listening socket has `RECVFROM_AVAIL` (usrsock's rule). If the NINA has no connection
  the answer is `EAGAIN`, and on a blocking socket usrsock waits for the next event. The new socket gets `SENDTO_READY`.
- Unsupported operations give the Zephyr build's errno: `getsockopt`, `getsockname`, `getpeername` and `shutdown` give
  `EOPNOTSUPP`; `setsockopt` other than `TCP_NODELAY` gives `ENOPROTOOPT` (NuttX handles `SO_RCVTIMEO` and `SO_SNDTIMEO`
  itself). `ioctl` answers `ENOTTY`, which leaves the command to NuttX's own handling (the network device's `SIOC*` and so
  on). `socket(AF_INET, SOCK_CTRL)` (used by `ifconfig` and others) is answered `ENOTSUP`, which leaves it to NuttX.
- Sending and receiving use the Zephyr build's values (a datagram of more than 512 bytes gives `EMSGSIZE`). `sendmsg` joins
  several vectors, up to 3584 bytes, and writes them with one command (so that MQTT's header and payload go out together;
  NuttX's usrsock passes the vectors separately). A send with one vector is treated as `send`/`sendto`, with more as
  `sendmsg` (UDP `sendmsg` gives `EOPNOTSUPP`, as in the Zephyr build).
- How sending and receiving wait: each request makes one pass of the Zephyr build's wait loop in the calling thread, and
  usrsock waits between passes.
  - When a waiting TCP send finds that the NINA took nothing, the answer is `EAGAIN` with `SENDTO_READY` dropped; after
    `CONFIG_NINA_SEND_RETRY_MS` `nina_event` reports `SENDTO_READY` and usrsock repeats the request (the next pass). The
    state is checked with the NINA's poll at the first pass and then every 100 ms (the error flag gives `EPIPE`), and the
    call gives `EAGAIN` 10 s after its first pass. The deadline and the next check are kept per socket; a continued call is
    told from a new one by the data's address and length.
  - A send that does not wait (`MSG_DONTWAIT`, a non-blocking socket) gives `EAGAIN` at once when nothing is taken, as in
    the Zephyr build. A datagram the NINA did not take (the NINA reports every failure as zero bytes) gives `EAGAIN` at
    once whether the call waits or not. In these two cases and at the 10 s deadline the answer is `-ETIME` with
    `SENDTO_READY` kept, which patch 0005 turns into `EAGAIN` that ends the call (stock usrsock repeats a waiting call's
    `EAGAIN`). `SENDTO_READY` is dropped only between the passes of a waiting send: with it dropped, a following blocking
    send would wait in usrsock without asking the driver, with no passes and no deadline. So a TCP socket always looks
    writable to `poll()` (the Zephyr build's `poll()` asks the NINA).
  - A receive first checks with the NINA's poll that data is there (as in the Zephyr build: the answer cannot tell
    "nothing" from an empty datagram); if not, the answer is `EAGAIN` with `RECVFROM_AVAIL` dropped. A waiting receive
    waits in usrsock for `RECVFROM_AVAIL`, which `nina_event` reports from its polls every 20 ms (NuttX handles
    `SO_RCVTIMEO`). A TCP receive that filled its request (all it asked for, up to 512 bytes) keeps `RECVFROM_AVAIL`: more
    may be there, and the next receive asks the NINA (as every receive of the Zephyr build does). After other data
    `RECVFROM_AVAIL` is dropped and `nina_event` polls again at once. usrsock passes a receive to the driver only with
    `RECVFROM_AVAIL` (without it, a receive with `MSG_DONTWAIT` gets `EAGAIN` from usrsock itself), so a receive or a
    `poll()` learns of new data only from `nina_event`'s poll (up to 20 ms later, or at the IRQ line's edge; the Zephyr
    build asks there and then). Without the kept `RECVFROM_AVAIL`, a packet read in pieces (the MQTT client's CONNACK: 2
    bytes, then 2) would stop after the first piece, as `nina_event` may come only after the next receive.
  - A UDP socket's first send, which creates its NINA socket, makes `nina_event` poll it at once (with the socket events,
    that poll arms the event of the reply, such as the time synchronization's).
  - A poll that found data (`nina_event`'s) stands for the next receive's first poll (nothing takes the data in between).
- A TCP receive of 0 means the peer closed or the connection failed (the NINA reports both as zero bytes, as in the Zephyr
  build).

**Bit 2's waits**

- The GPIO2 interrupt of `port/nuttx/nina_os.c` (`common/compat/port/nuttx/rp2040_gpio_os.c`, a callback of the layer's
  `gpio0`) counts the edges, posts the `sem_t` of every waiting thread (a broadcast) and wakes `nina_event`
  (`nina_poller_edge()`).
- `nina_event` starts its next pass as soon as the count has changed since the count noted before its last pass (or before
  the commands of the receive request that scheduled it), instead of after 20 ms; otherwise after 100 ms (or a connection
  in progress's deadline, if earlier). When passes started by an edge keep reporting nothing, it waits a little before the
  next edge counts (1, 2, 4, 8, 16 ms, then 20 ms).
- A waiting TCP send's next pass: after a pass that only polled (the socket was not writable), when the count has changed
  since the request noted it before its commands (100 ms at most, not past the send's deadline); after a pass that sent and
  got zero bytes, 1 ms later, as without bit 2.

**Name resolution**

`nina_dns.c` is used through functions of its own, `nina_dns_getaddrinfo()` and `nina_dns_freeaddrinfo()`
(`include/nina_dns.h`), and the compatibility layer's `<zephyr/net/socket.h>` maps `zsock_getaddrinfo()`,
`zsock_freeaddrinfo()` and `struct zsock_addrinfo` to them. NuttX's `getaddrinfo()` (`CONFIG_LIBC_NETDB`, which queries a
DNS server over UDP itself) is not used; `CONFIG_NINA_DNS` cannot be on together with `CONFIG_LIBC_NETDB`. Errors are
NuttX's `EAI_*` values under the Zephyr build's `DNS_EAI_*` names (`EAI_NONAME` for a name that does not resolve,
`EAI_AGAIN` for an SPI failure, `EAI_FAIL` for an address of the wrong length, `EAI_BADFLAGS`/`EAI_FAMILY`/`EAI_SOCKTYPE`
for bad hints, `EAI_SERVICE` for a bad service, `EAI_MEMORY`). The result comes from `malloc()`, not the Zephyr build's
2 KB heap.

**Wi-Fi (network device `wlan0`)**

| File | Role |
|---|---|
| `nina_wifi.c` and `include/nina_wifi.h` | The core's Wi-Fi operations |
| `port/nuttx/nina_netdev.c` | Registers the NINA as the NuttX network device `wlan0`, turns wapi's wireless ioctls into the core's calls, puts the core's results on `wlan0` (addresses, carrier) and starts the thread |
| `port/nuttx/nina_wifi_mgmt.[ch]` | The Zephyr Wi-Fi management that the applications use (the compatibility layer's `net_mgmt()`), on the core: the interface of `net_if_get_default()`, `NET_REQUEST_WIFI_CONNECT` (which `NET_REQUEST_WIFI_CONNECT_STORED` makes for each stored network), the connection's events, and the Wi-Fi profile's reconnection as the core's hooks (`common/imu/wifi_profile.c` with `CONFIG_WIFI_PROFILE`; without it every connection is allowed and nothing reconnects) |
| `port/nuttx/nina_net.h` | `nina_net_initialize()` (the board's bring-up) and `nina_net_start()` (the application's table of threads, `common/imu/app_init.c`, which the init task runs) |

- Start-up: the board's bring-up calls `nina_net_initialize()` after the LSM6DSOX and before the KV store (the Zephyr build
  initializes the interface at `POST_KERNEL` after the sensor and sets up NVS in `main()`). It starts the `net_mgmt` thread
  and registers the compatibility layer's interface, registers `wlan0` as `NET_LL_IEEE80211` with `IFF_UP` and no carrier,
  and reads the MAC address and the capabilities from the NINA, in Zephyr's order. The init task starts the thread
  `nina_wifi_tid` with `nina_net_start()` where Zephyr starts its static threads (the first entry of the application's
  table of threads, `app_threads_start()` of `common/imu/app_init.c`): a kernel thread on a static stack
  (`kthread_create_with_stack()`) at the priority and with the stack of `priorities.h` (110; 2048 bytes), running before
  `nina_net_start()` returns.
- Interface values: the IPv4 address, netmask and gateway are `wlan0`'s `d_ipaddr`, `d_netmask` and `d_draddr`, the MAC
  address is `d_mac`. NuttX's `ifconfig` (`/proc/net/wlan0`), `SIOCGIFADDR` and so on read them there. Zephyr's "dormant" is
  `IFF_RUNNING` clear (`RUNNING` appears when connected). `wlan0` has no `d_ifup`/`d_ifdown`, so `ifconfig wlan0 down`/`up`
  do nothing (the Zephyr build's interface is always up as well). NuttX's own TCP/IP is not used, so ARP and ICMP are off
  (`CONFIG_NET_ARP`, `CONFIG_NET_ICMP`). `CONFIG_NET_ETHERNET` stays: registering a `NET_LL_IEEE80211` device needs it
  (`netdev_register()` uses `CONFIG_NET_ETH_PKTSIZE`), and so does the MAC address field (`d_mac.ether`).
- The stored credentials and automatic reconnection (`nuttx/apps/wifi/`: the Zephyr build's own file
  `common/imu/wifi_profile.c`, copies of Zephyr's `subsys/net/lib/wifi_credentials/`, `wifi cred`; "Stored Wi-Fi
  credentials and automatic reconnection (`wifi cred`)" in [`design/nuttx-porting.md`](../../../design/nuttx-porting.md)) use
  the layer's `net_mgmt()` (`NET_REQUEST_WIFI_CONNECT_STORED`), `net_mgmt_add_event_callback()`
  (`NET_EVENT_WIFI_CONNECT_RESULT`), `net_if_get_wifi_sta()`, `net_if_ipv4_get_global_addr()` and `k_current_get()` as the
  Zephyr build does; `nina_wifi_mgmt.c` serves them on the core.

wapi's ioctls (`d_ioctl`, handled in the calling thread):

| ioctl | Handling |
|---|---|
| `SIOCSIWSCAN` | `nina_wifi_scan()`. An ESSID, a passive scan or channels are refused with `-ENOTSUP`, as the Zephyr build's driver refuses them |
| `SIOCGIWSCAN` | `-EAGAIN` while scanning, the scan's error if it failed, else the results (up to 10 collected from the core's scan callback, in the NINA's order, as wireless events: BSSID, channel, RSSI in dBm, SSID, encrypted or not; nothing about encryption when the type is unknown). `-E2BIG` if the buffer is too small |
| `SIOCSIWAUTH` | Keeps the WPA version and the pairwise cipher |
| `SIOCSIWENCODEEXT` | Keeps the passphrase. What these two keep is used and forgotten by the next `SIOCSIWESSID` (the passphrase is wiped), so a `wapi essid` without a `wapi psk` before it connects without encryption |
| `SIOCSIWESSID` | `nina_wifi_connect()` with the kept settings: WPA2 → PSK, none → open, WPA, WPA3 and WEP → refused by the core (`-ENOTSUP`), after the checks of Zephyr's Wi-Fi management on the key (`-EINVAL`: a WEP key of other than 5, 10, 13 or 26 bytes, another key of other than 8 to 64 bytes, no key for a security that needs one). With a passphrase but no version set: PSK (as Zephyr's `wifi connect` without `-k`). Flags 0 (the random SSID of wapi's `disconnect`) does nothing |
| `SIOCSIWAP` | 00:00:00:00:00:00 or ff:ff:ff:ff:ff:ff: `nina_wifi_disconnect()`; anything else `-ENOTSUP` |
| `SIOCGIWESSID`, `SIOCGIWAP`, `SIOCGIWSENS` | SSID, BSSID and RSSI from `nina_wifi_status()` (asks the NINA). While disconnected: empty SSID, zero BSSID, `-ENOTCONN` for the RSSI |
| `SIOCGIWMODE`, `SIOCSIWMODE` | Managed (`IW_MODE_INFRA`) only |

Other wireless ioctls return `-ENOTTY`. The connection's result is not what `wapi essid` returns (the Zephyr build's
`wifi connect` also returns once the request is made); check `RUNNING` and the IP address in `ifconfig`, or the ESSID in
`wapi show`.

**Passthrough mode**

The host side is `port/nuttx/nina_passthrough_os.c` on cdcacm (built with the NINA driver, `nuttx/drivers/nina/Make.defs`),
the module side `port/nuttx/nina_passthrough_bus_rp2040.c` on the RP2040's registers, with `CONFIG_NINA_PASSTHROUGH`
(default y).

- Way in: `nina passthrough confirm` (`nuttx/apps/nina`) writes the mark and restarts (as the Pico SDK's
  `watchdog_reboot(0, 0, 0)`). `rp2040_boardinitialize()` reads and clears the mark first, so RESET, any later restart and a
  fatal error start the normal firmware.
- The start of the mode: `rp2040_bringup()` sets up the USB device only (CDC ACM, the same descriptors and serial number),
  none of SPI1, the NINA, I2C0, the sensor, `wlan0` or the key-value flash. The init task runs `nina_passthrough_run()`
  (priority 120) instead of nsh, Telnet, Wi-Fi and the threads.
- Lines and rate: cdcacm's callback (`CAIOC_REGISTERCB`, called from the USB interrupt) reads the new state with
  `CAIOC_GETCTRLLINE` and `CAIOC_GETLINECODING` and applies it to the pins at once, so every change of the host is applied
  in its order, however short. The release of GPIO0 after 750 ms, the log of a close, the LED (GPIO6) blinking at 5 Hz and
  the way out are done by the init task, which waits on a semaphore.
- Module to PC: UART1's receive interrupt (at 4 bytes in the FIFO, or after 32 bit times without more) moves the bytes into
  the ring (taken from the heap in this mode only); the kernel thread `nina_pt_rx` writes it to `/dev/ttyACM0`. At
  921600 bps the 28 bytes left in the FIFO (300 us) are the margin for the interrupt's latency.
- PC to module: `nina_pt_tx` reads `/dev/ttyACM0` and puts the bytes into UART1's transmit FIFO, waiting for the transmit
  interrupt while it is full. It reads no more until they are in, so the serial driver's receive buffer fills, cdcacm stops
  taking packets (the host waits) and nothing is lost.
- `/dev/ttyACM0` is not a console: no translation, no echo (the termios flags are cleared as well).
- At the way out the module is started into its own firmware as `init_nina()` does it (GPIO0 high, RESETN low for 10 ms,
  750 ms, GPIO0 an input).
- No signal is used (the waits are semaphores, sleeps and blocking reads and writes). NuttX's UART1 driver
  (`CONFIG_RP2040_UART1`) would take the pins at boot and is not used (UART1's registers are driven directly).

### Pico SDK (shared by FreeRTOS and ThreadX)

The FreeRTOS and ThreadX builds share the part on the Pico SDK, `port/pico_sdk/`; only the OS's part differs (`nina_os.c`
and `nina_bus_dma_os_impl.h` of `port/freertos/` and `port/threadx/`, written on that OS and the Pico SDK without the
compatibility layer). `port/pico_sdk/nina.cmake` (`pico_sdk_nina()`) adds the driver to the build. The application's start
calls nothing of the driver (the core sets up SPI1 and resets the module at its first request).

| File | Role |
|---|---|
| `nina_bus_rp2040.c` | The boundary `nina_bus.h`: SPI1 at 7.8125 MHz and the CS, READY, RESET and IRQ lines (the Pico SDK's `hardware_spi` and `hardware_gpio`; set up in the order and with the values of Zephyr's drivers). Transfers shorter than 64 bytes polled, longer ones by `nina_bus_dma_rp2040.c` |
| `nina_bus_dma_rp2040.c`, `.h` | The long transfers by DMA (`hardware_dma`, `hardware_irq`), waiting on `nina_bus_dma_os.h` |
| `nina_bus_dma_os.h` | The boundary of the OS's part of the DMA's wait: the calling thread sleeps until the DMA interrupt handler wakes it or a time runs out. The OS's part `port/<OS>/nina_bus_dma_os_impl.h` defines its types, macros and inline functions |
| `nina_board_rp2040.h` | The board's lines (as in Zephyr's `arduino_nano_connect.dts` and `-pinctrl.dtsi` and the NuttX board's `board.h`, `BOARD_NINA_*`) |
| `nina_passthrough_os.c` | The passthrough's host side (TinyUSB and the Pico SDK, without the scheduler) |
| `nina_passthrough_bus_rp2040.c` | The passthrough's module side: UART1, the module's lines, the watchdog's scratch registers, the LED (`hardware_uart`, `hardware_irq`, `hardware_watchdog`) |
| `nina.cmake` | The build definitions |

- **DMA** (`nina_bus_dma_rp2040.c`): a transfer of 64 bytes or more (SOCKET_SEND's payload, the long reads) goes through
  two channels that the binding claims with `dma_claim_unused_channel(false)` at the bus's first use (in the normal mode
  only; without two free channels every transfer stays polled) and keeps. RX first (SSPDR to the buffer, or to a one-byte
  sink; DREQ 19), then SSPDMACR's TXDMAE and RXDMAE, then TX (the buffer to SSPDR; DREQ 18), both 8-bit. The calling thread
  sleeps until the RX channel's DMA_IRQ_1 interrupt (a handler added with `irq_add_shared_handler()`, which looks at its own
  channels only). SSPDMACR is then cleared and SPI1 left as after a polled transfer (RX FIFO empty, no overrun flag). A
  transfer that does not end within 2 ms + length / 256 ms aborts both channels and fails with `-EIO`; so do a lost byte
  (the overrun flag) and a channel's bus error. The core returns it as the request's error (the socket call fails with
  `EIO`). All three are logged to `dmesg` (`<err> nina_bus: SPI1 DMA did not end in ...`, `SPI1 RX overrun by DMA ...`,
  `SPI1 DMA bus error ...`, at most one line each per 5 s, with the count of those held back). A channel that does not stop
  when aborted, and no two free channels at the start, are logged once and make every later transfer polled. Short
  transfers and transfers before the scheduler runs are polled.
- **The READY and IRQ interrupts**: the core's waits for READY sleep on READY's edge interrupt (GPIO10), in waits of at most
  10 ms, after which READY is read again (the tick stands still while interrupts are masked and catches up with the TIMER
  afterwards, hence the slices of 10 ms). The binding has its own raw handler of `IO_IRQ_BANK0` for GPIO10 and GPIO2
  (`gpio_add_raw_irq_handler_masked()`, beside the GPIO bank 0 backend's for GPIO24,
  `common/compat/port/pico_sdk/gpio_os_rp2040.c`; each handler acknowledges only its own pins). These two and the DMA's
  handler take slots of the Pico SDK's shared interrupt handlers (the number of slots and the interrupt priorities are in
  [the FreeRTOS port's "Interrupts"](../../../design/freertos-port.md#interrupts)). The FreeRTOS and ThreadX bindings' `nina_os_ready_start()`
  does not fail.
- **Sockets and Wi-Fi**: the Zephyr binding's `port/zephyr/nina_socket.c` (socket offload, priority 40) and
  `port/zephyr/nina_net.c` (Zephyr's Wi-Fi management and interface, the thread `nina_wifi_tid`) are compiled unchanged on
  the compatibility layer. As in the Zephyr build, the calling thread asks the NINA and waits (there is no event thread like
  the NuttX build's usrsock and `nina_event`). The layer's socket layer, descriptor table, `net_if`, `net_mgmt` and `wifi`
  command are described in [the compatibility layer's notes](../../compat/README.md). The core's POSIX names are the layer's
  `common/compat/libc_net/include/`, the binding's `net_*` names the layer's `<zephyr/net/net_ip.h>` (the Zephyr build's
  `port/zephyr/posix/` maps the other way). `CONFIG_NINA_TCP_MAX_IO` (3584), `CONFIG_NINA_SEND_RETRY_MS` (1) and
  `CONFIG_NET_SOCKETS_OFFLOAD_PRIORITY` (40) are the Zephyr build's values, in the prelude of the shared sources
  (`common_prelude.h`). Name resolution's results come from a 2048-byte heap (`EAI_MEMORY` when about 36 results are not
  freed).
- **Passthrough mode**: `main()` sees the mark before anything else and, with no task (thread), scheduler, SPI1, NINA, I2C
  or KV store, relays between TinyUSB (`tud_task()`) and UART1 in one loop. From the module to the PC, UART1's receive
  interrupt and the core's part of it run from RAM (`NINA_PASSTHROUGH_ISR_SECTION`). The LED blinks at 5 Hz.
- The NINA core's binding (`nina_os.c`) is on the list of code in RAM ([the FreeRTOS port's notes](../../../design/freertos-port.md)).

### FreeRTOS

`port/freertos/nina_os.c` (the boundary `nina_os.h`) and `port/freertos/nina_bus_dma_os_impl.h`.

- The NINA lock, the name resolution lock and the Wi-Fi operations' lock are FreeRTOS recursive mutexes with priority
  inheritance.
- The Wi-Fi thread's wake-up is its task notification 2 (the core keeps its requests itself).
- The waits for READY sleep on a binary semaphore that READY's edge interrupt gives.
- Socket events: on the rising edges of the IRQ line (GPIO2) on its interrupt, the binding counts them and wakes every
  waiting task (the waiters are on a list, each woken on its task notification 2).
- The SPI DMA's wait is the calling task's NINA task notification (index 2).
- Sleeps are "rounded up, plus one tick"; the clock is the TIMER in ms.
- The binding's objects are static and made at the first use of any of them.

### ThreadX

`port/threadx/nina_os.c` (the boundary `nina_os.h`) and `port/threadx/nina_bus_dma_os_impl.h`, written on ThreadX and the
Pico SDK directly, not on the layer. What they wait on are `TX_SEMAPHORE`s the driver owns.

- Why the waits are `TX_SEMAPHORE`s and not a wake-up of the thread's own or a suspend and resume: see [the ThreadX port](../../../design/threadx-port.md#the-layers-kernel-part-and-the-drivers-waits).
- The three locks are `TX_MUTEX`es with priority inheritance, the READY wait and the Wi-Fi thread's wake-up
  `TX_SEMAPHORE`s. A post to the Wi-Fi thread before its first wait stays (as in the Zephyr build).
- A socket event wait makes a semaphore on its stack for the call and deletes it at the end (the event interrupt walks the
  list of waiters and wakes them all).
- The SPI DMA's wait (`nina_bus_dma_os_impl.h`) is one semaphore of the transfers (one at a time, inside `nina_os_lock()`).
- The objects are made at the first use and never deleted (they stay in the lists of created objects).
- Timed waits and sleeps last the time rounded up to ticks plus one tick (Zephyr's rule; ThreadX's wait of n ends at the
  current tick + n). The clock is the TIMER in ms, not the tick.

## Tests

`common/tests/run.sh` runs the core's tests, and each OS's `host_tests.sh` runs its bindings' tests after them. None needs
the board.

| Test | Place | What it checks |
|---|---|---|
| `nina_core_independent` | `common/tests/run.sh` | Each file of the core (the top, `include/`, `internal/` and the boundary headers of `port/`) compiles with the C library, the POSIX socket types and its own headers only |
| `nina_public_headers` | `common/tests/run.sh` | The headers of `include/` compile with that directory alone (none reads `internal/` or `port/`) |
| `nina_proto` | `common/tests/nina_proto.c` | SPI framing, parsing of answers (including the firmware's error answer 0xEF), READY timeouts, the reset sequence, the CS order, the capabilities query (0x6A) (fake boundary and clock); each with the wait for READY on its interrupt and with READY sampled every 100 µs when that cannot start |
| `nina_wait` | `common/tests/nina_wait.c` | The socket waits (`nina_wait.c`): the intervals (before the events start, and after they could not), the IRQ line's edges, the 100 ms timeout, deadlines, the rests (fake clock and line) |
| `nina_sock` | `common/tests/nina_sock.c` | The socket table and commands (fake `nina_proto_request()` and clock) |
| `nina_wifi` | `common/tests/nina_wifi.c` | The Wi-Fi operations (`nina_wifi.c`) with a fake boundary and binding: connect and its check, its thread (the check every 500 ms, power saving, the address, the 60 tries, a disconnection during an attempt), disconnect, the monitor (lost after two readings), the scans of 3.0.2 and of the patched firmware and how they end, status. The combinations of the one thread's states (the monitor's checks on time during a scan and during an attempt, a disconnection found during a scan, a request refused while the results are read, a request at the deadline, the 10 s and 30 s limits across checks) |
| `nina_dns`, `nina_dns_offload` | `common/tests/nina_dns.c` | Name resolution (`nina_dns_offload` in the form `NINA_OS_DNS_OFFLOAD` of the Zephyr, FreeRTOS and ThreadX builds) |
| `nina_passthrough` | `common/tests/nina_passthrough.c` | The passthrough's core (`nina_passthrough.c`) on fakes of its boundaries: the restart's request told from a fatal error record, DTR and RTS on the module's lines, the rate and the way out, the buffer (also on two threads), the start, the data both ways with their counts (breaks, overruns, a full buffer, no host), the tick (the time GPIO0 is released, the LED), the way out's sequence |
| `nina_boundary` | `zephyr/script/host_tests.sh` | The shared `nina_proto.c` on `nina_bus.c` and `nina_os.c` of `port/zephyr/` (with the Wi-Fi operations' lock and the Wi-Fi thread's wake-up): framing, the lines and their settings, the reset sequence, the wait for READY on its interrupt (registering the callback, resetting the semaphore, the edge's direction, disabling before transfers, the 10 ms limit) and the sampling when the callback cannot be registered. The waits for the IRQ line's edges (when they cannot start, GPIO2's callback and rising edge, the edge count, a wait ended by an edge, the timeout, an edge right before a wait) |
| `nina_boundary_passthrough` | `zephyr/script/host_tests.sh` | The same with `CONFIG_NINA_PASSTHROUGH`: no bus during the passthrough, `-ENODEV` without touching the lines or SPI, the normal behaviour once it is over |
| `nina_boundary_spi_dma`, `nina_bus_spi_dma_threshold` | `zephyr/script/host_tests.sh` | The same with the bus configuration of `prj.conf` (the PL022's DMA mode and `CONFIG_SPI_PL022_DMA_MIN_LEN` 64); `nina_bus.c` does not build in the DMA mode without a threshold |
| `passthrough` | `zephyr/script/host_tests.sh` | The passthrough's Zephyr side `port/zephyr/nina_passthrough_os.c` and `nina passthrough` of `nina_shell.c` with the real core, on a fake Zephyr and a fake module side: the request at the start, the lines and the rate from the USB messages and the tick, the data both ways and the hand-over from the interrupts, the bytes dropped without USB, the way out's sequence, the command |
| `nina_socket` | `zephyr/script/host_tests.sh` | The socket offload `port/zephyr/nina_socket.c` on the shared core: file descriptors, the wait loops and their sleeps (after the events could not start), options, accept, sendmsg, datagrams, poll, the resolver's registration. The waits for the IRQ line's edges once the events started (an edge between passes, the timeout, deadlines, how a send waits, a `poll` with a descriptor other than a NINA socket) |
| `nina_net` | `zephyr/script/host_tests.sh` | The Zephyr binding `port/zephyr/nina_net.c` with fake Zephyr declarations and a fake core: the device, the interface's initialization, the scan's parameters and results, connect, disconnect, status, the core's callbacks on the interface and as Zephyr's events, the profile's hooks, the thread's stack and priority (one) |
| `nina_os` | `nuttx/tests/nina_os.c` | The wait for READY on its interrupt of `port/nuttx/nina_os.c` (with the core's `wait_ready()`), the GPIO2 interrupt and its waits, the Wi-Fi operations' lock and the Wi-Fi thread's wake-up (fake GPIO bank and kernel). The passes of `nina_event` at the edges have no host test |
| `nina_netdev` | `nuttx/tests/nina_netdev.c` | `wlan0` and wapi's ioctls on the core |
| `nina_wifi_mgmt` | `nuttx/tests/nina_wifi_mgmt.c` | The applications' Wi-Fi management |
| `nina_passthrough` | `nuttx/tests/nina_passthrough.c` | The confirmation word, argument counts and messages of `nina passthrough`. The NuttX side (the interrupts, threads and cdcacm together) has no host test |
| `nina` | `freertos/tests/nina/` | The NINA's SPI and line boundary (with the long transfers by DMA on a fake DMA: the threshold, SPI1's state at the end, the shared DMA_IRQ_1, timeout and abort, a lost byte, a bus error, the transfer after each, a channel that does not stop, no free channel, the logged messages) and the FreeRTOS binding of `nina_os.h` (no compatibility layer; built twice, with a 1 ms and a 4 ms tick): the wait for READY on its interrupt, the waits for the IRQ line's edges (two tasks woken by one edge, the timeout), the shared bank interrupt, the locks, the Wi-Fi thread's wake-up, the sleeps' tick counts, the clock across the tick's wrap |
| `socket` | `freertos/tests/socket/` | The NINA's sockets on the compatibility layer: descriptors, the wait loops, options, poll, the descriptor locks, name resolution |
| `wifi` | `freertos/tests/wifi/` | The interface, events, credentials, reconnection, `wifi` and `ifconfig`, the scans by the firmware's bits (keeping the link, asynchronous, failing, not ending, a disconnection during the scan, 3.0.2) |
| `nina` | `threadx/tests/nina/` | The FreeRTOS build's test body (`freertos/tests/nina/main.c`) with `threadx/tests/nina/nina_test_os.c` (on ThreadX's Linux port): the SPI and line boundary and the DMA, the READY and event interrupts, the locks, sleeps, the clock; and also calls before the kernel, a post to the Wi-Fi thread before its first wait, priority inheritance across two mutexes, the window right before a wait |
