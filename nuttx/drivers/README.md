# Drivers (NuttX port)

English | [日本語](README_ja.md)

`nuttx/drivers/` holds the port's code that NuttX builds on the kernel side (through the symlink
`nuttx/nuttx/external`, into `libexternal.a`). The board's part is in
`nuttx/boards/arduino-nano-rp2040-connect/src/`. The Zephyr version's driver notes are
`zephyr/app/usb-console/drivers/README.md`. How to use the commands, and the differences from the
Zephyr version, are in `nuttx/README.md`, and how each part works in detail is in
`design/nuttx-porting.md`.

## NINA-W102 (`nina`)

The NINA driver is in two parts: an OS-independent core (the repository's `common/drivers/nina/`,
shared with the other builds) and its connection to NuttX (`nuttx/drivers/nina/` and the board's
`src/nano_nina.c`). `nina/Make.defs` finds the core's `.c` files through `VPATH`, and the objects are
made in `nuttx/drivers`. The core is compiled without the prelude (the configuration values come in
through `<nuttx/config.h>`, which NuttX's headers read). Its headers are in `common/drivers/nina/`;
the kernel side (`nina/Make.defs`), the board and the `nina` and `imu` applications add that
directory to their include paths.

What the core needs from the OS and the board (its boundary):

| Boundary | NuttX implementation | What it must do |
|---|---|---|
| `common/drivers/nina/nina_os.h` | `nina/nina_os.c` | `nina_os_lock()`, `nina_os_unlock()`: a recursive lock with priority inheritance (the Zephyr version's `K_MUTEX_DEFINE(nina_lock)`); `nina_os_lock()` returns 0 or a negative errno, which `nina_request()` and `nina_ip_config()` return untouched. `nina_os_dns_lock()`, `nina_os_dns_unlock()`: a lock with priority inheritance held across the two resolver commands. `nina_os_sleep_us()`, `nina_os_sleep_ms()`: sleep for the time rounded up to ticks plus one tick, as Zephyr's `k_sleep()`, not ended by signals. `nina_os_uptime_ms()`: the kernel clock in milliseconds, rounded down (`k_uptime_get()`). `nina_os_ready_arm()`, `nina_os_ready_wait()`, `nina_os_ready_disarm()`: waiting for a change of READY on its interrupt (`NINA_OS_WAIT_READY`, defined by `nina/Make.defs`; see "Waiting for READY" below). `nina_os_event_start()`, `nina_os_event_seq()`, `nina_os_event_wait()`: counting the rising edges of the IRQ line (GPIO2) and waking every waiting thread (`NINA_OS_EVENTS`, defined by `nina/Make.defs`; see "Firmware additions" below) |
| `common/drivers/nina/nano_nina.h` | the board's `src/nano_nina.c` | `nano_nina_initialize()`: called once at boot, sets up the SPI and its pins and drives CS inactive (leaves READY, RESET and IRQ alone). `nano_nina_bus_ready()`: whether the SPI and the four lines can be used. `nano_nina_pin_configure()`, `nano_nina_pin_set()`, `nano_nina_pin_get()`: logical values (1 is active); the board knows each line's polarity and pull (CS GPIO9 active low, READY GPIO10 active low with pull-up, RESET GPIO3 active low, IRQ GPIO2 active high with pull-down). `nano_nina_transceive()`: a synchronous transfer in mode 0, 8 bits, MSB first (only a write when RX is NULL), without touching CS |
| Configuration | `nina/Kconfig` | `CONFIG_NANO_NINA_TCP_MAX_IO` (default 3584) and `CONFIG_NANO_NINA_SEND_RETRY_MS` (default 1), read by `nina_sock.h`, which uses the defaults without them |
| C library | NuttX's libc | `nina_sock.[ch]` and `nina_dns.[ch]` use the types and constants of POSIX's `<sys/socket.h>`, `<sys/types.h>`, `<sys/uio.h>`, `<netinet/in.h>`, `<netinet/tcp.h>`, `<arpa/inet.h>`, `<poll.h>` and `<netdb.h>` (`struct sockaddr_in`, `struct iovec`, `struct addrinfo`, `AF_INET`, `MSG_DONTWAIT`, `POLLIN`, `EAI_*` and so on) and `inet_pton()`, `htons()`, `malloc()` and `free()`. The `POLL*` bits of `nina_sock_poll_revents()` and the `EAI_*` values are those of the same headers as the callers' |

### SPI transfer layer

| File | Role |
|---|---|
| `common/drivers/nina/nina_spi.c` | Framing of commands and answers (`nina_request()`, `nina_ip_config()`). Originally a port of the functions of the same names in the Zephyr version's `nina_wifi.c`, with the branches, order, constants and error values matching one to one; now used by the three builds |
| `common/drivers/nina/nina_proto.h` | The public declarations; the Zephyr version uses the same file |
| `common/drivers/nina/nina_os.h`, `nina/nina_os.c` | The lock (a recursive mutex with priority inheritance), sleeps, the millisecond clock and the wait for READY on its interrupt: Zephyr's `K_MUTEX`, `k_sleep()` and `k_uptime_get()` |
| `common/drivers/nina/nano_nina.h`, the board's `src/nano_nina.c` | SPI1 and the four lines (CS, READY, RESET, IRQ): what Zephyr's devicetree and its SPI and GPIO drivers set up |

- A request is handled synchronously in the calling thread. The mutex serializes requests, and CS
  stays asserted from the start of a phase until its last byte, including the wait for READY.
- The NINA is reset once, on the first request (10 ms, 750 ms). At boot the board only sets up
  SPI1 and its pins and drives CS inactive.
- The SPI is NuttX's `rp2040_spi.c`, polled (no DMA). Each transfer takes `SPI_LOCK`; the first
  one sets mode 0, 8 bits and `CONFIG_NANO_NINA_SPI_FREQUENCY`. `SPI_SELECT` is not used.
- Waiting for READY: instead of sampling READY (GPIO10, active low) every 100 µs, the core's
  `wait_ready()` sleeps until the line's interrupt reports a change (as in the Zephyr and FreeRTOS
  builds). `nina_os_ready_arm()` sets the semaphore to 0, then enables
  the edge towards the desired level (ready: falling, busy: rising) through the board's
  `src/nano_gpio.c` (`nano_gpio_irq_configure()`, which clears an edge latched while the event
  was off before enabling it); the core reads READY after that. A change after the read stays in
  the semaphore, so the wait returns at once. The interrupt is GPIO bank 0's single IRQ, shared
  with the LSM6DSOX's INT1 (GPIO24); a callback on the compatibility layer's `gpio0` only posts
  the semaphore (at most one count). `nina_os_ready_wait()` waits once with a timeout (the
  milliseconds rounded up to ticks, plus one tick). The deadline (`nina_os_uptime_ms()`) and the
  timeout are both on the tickless tick read from the TIMER, so they do not drift apart while
  interrupts are masked (a flash erase); the FreeRTOS build waits in slices of 10 ms because its
  tick falls behind. `nina_os_ready_disarm()` disables the edge, so CS toggling READY during
  the transfers raises no interrupts. `gpio0` and `src/nano_gpio.c` are built with
  `CONFIG_NANO_LSM6DSO_STREAM`, so `CONFIG_NANO_NINA` depends on it.
- The time limits and the wiring are in the "NINA-W102 (`nina`)" section of
  `design/nuttx-porting.md`.
- `common/drivers/nina/nina_spi.c` is checked by a host test (`common/tests/nina_spi.c`, run by
  `common/tests/run.sh`) with a fake board and clock. The wait for READY on its interrupt
  (`nina/nina_os.c` with the core's `wait_ready()`) is checked by `nuttx/tests/nina_os.c`
  (`script/host_tests.sh`) on a fake GPIO bank and kernel.

### Sockets (usrsock)

The Zephyr version uses a socket offload (`drivers/nina/nina_socket.c`); the NuttX version uses
usrsock, with `usrsock_request()` in the kernel (NuttX's `CONFIG_NET_USRSOCK_CUSTOM`). NuttX's own
TCP/IP stack is not used (`NET_TCP_NO_STACK`, `NET_UDP_NO_STACK`).

| File | Role |
|---|---|
| `common/drivers/nina/nina_sock.[ch]` | The socket table (10 sockets) and the NINA's socket commands: `nina_socket_create()`, `do_bind()`, `do_listen()`, `do_accept()`, `do_connect()`, `do_close()` and `do_setsockopt()`, originally of the Zephyr version's `nina_socket.c`, and for sending and receiving `udp_send()`, `do_send()`, `do_sendmsg()`, `udp_recv()` and `do_recv()`, with the names, branches, order, constants and errno names kept (one pass of a send or receive wait loop per request); the Zephyr version now uses this file too. `FIONREAD` (`nina_sock_fionread()`) and the driver's own socket options `NINA_SO_CALLER_POLLS` and `NINA_SO_POLL` (`nina_sock_poll_revents()`, `common/drivers/nina/nina_sockopt.h`) are not used by the Zephyr version. OS-independent |
| `nina/nina_usrsock.c` | Dispatches usrsock's requests to the functions above and answers them (ACK) |
| `nina/nina_poller.c` | The thread `nina_event`: turns the state of the NINA's sockets into usrsock events (for a socket whose callers poll, only while it connects), and times the retries of a waiting send; with a firmware that reports socket events, it wakes on the IRQ line's edges |
| `nina/nina_usrsock.h` | The declarations between the two |

- A request is handled synchronously in the calling thread, at its priority (as the Zephyr
  version's socket calls). usrsock passes one request at a time and holds the usrsock lock
  meanwhile; the NINA lock inside `nina_request()` serializes the NINA commands. The lock order is
  always "usrsock lock, then NINA lock" (`nina_event` takes them in the same order).
- A socket's number (usrsock's usockid) is its place in the table (0 to 9). As in the Zephyr
  version, the NINA's socket is created on the first bind or connect and made non-blocking at
  once. An 11th `socket()` fails with `EMFILE`. usrsock has 12 connections
  (`CONFIG_NET_USRSOCK_PREALLOC_CONNS`): the 10, plus an 11th request, the new connection of an
  `accept()` and a `SOCK_CTRL` attempt.
- Waiting: the Zephyr version waits in the caller (connect polls every 20 ms for up to 10 s, poll
  every 20 ms). NuttX's usrsock waits for events from the driver, so `nina_event` makes the same
  queries (the NINA's poll command) every 20 ms and turns the answers into events: readable →
  `RECVFROM_AVAIL` (on a listening socket: a connection is waiting), writable → `SENDTO_READY`,
  the error flag or a failed poll of a connected TCP socket → `REMOTE_CLOSED`. A UDP socket's error
  flag is not passed on (the Zephyr version's datagram calls also carry on after it). A socket
  whose events usrsock already holds is not queried; with nothing to query the thread sleeps until
  a request wakes it. With a firmware that reports socket events ("Firmware additions" below), the
  queries come at the IRQ line's edges (100 ms at most) instead of every 20 ms.
- Sockets whose callers poll: the compatibility layer's `zsock_socket()` marks the socket it makes
  with the driver's own socket option `NINA_SO_CALLER_POLLS` (level `NINA_SOL`,
  `nina_sockopt.h`): the MQTT and time synchronization sockets. `nina_event` does not query
  a marked socket except to check a connection in progress, nor after a receive. As with the
  Zephyr version's NINA sockets, only its users (the layer's `zsock_recv()` and `zsock_poll()`)
  ask the NINA, in the calling thread (see the layer's "`zsock_poll()`" below). Unmarked sockets
  (Telnet, iperf, nsh commands) are queried by `nina_event` as above (the Zephyr version's Telnet is
  also asked every 20 ms, by the socket service's `poll()`). For a marked socket too, `nina_event`
  checks a connection (every 20 ms, up to 10 s) and times the wait before a waiting send's next pass
  (1 ms): under usrsock, a request held while waiting would stop the other sockets' requests too;
  the NINA commands of each pass are sent from the calling thread. A waiting receive (`recv()`
  without `MSG_DONTWAIT`) or NuttX's `poll()` on a marked socket would wait for an event that does
  not come: use the layer's calls on these sockets.
- connect: the state is checked once right after the connect command (the first pass of the
  Zephyr version's loop); if the handshake is not done, the request is answered "in progress" and
  `nina_event` checks every 20 ms, up to 10 s after the connect command (`ETIMEDOUT`). Success is
  reported with `SENDTO_READY`, failure with the final answer to the connect request, so
  `connect()` sets the Zephyr version's errno (SO_ERROR 104/113/116/118 → `ECONNRESET`/
  `ECONNABORTED`/`ETIMEDOUT`/`EHOSTUNREACH`; anything else and the error flag → `ECONNREFUSED`).
  The failed NINA socket is closed.
- accept: the request comes once the listening socket has `RECVFROM_AVAIL` (usrsock's rule). If
  the NINA has no connection the answer is `EAGAIN`, and on a blocking socket usrsock waits for the
  next event. The new socket gets `SENDTO_READY`.
- Unsupported operations give the Zephyr version's errno: `getsockopt`, `getsockname`,
  `getpeername` and `shutdown` give `EOPNOTSUPP`; `setsockopt` other than `TCP_NODELAY` gives
  `ENOPROTOOPT` (NuttX handles `SO_RCVTIMEO` and `SO_SNDTIMEO` itself). The exceptions are the
  driver's own two at `NINA_SOL` (`NINA_SO_CALLER_POLLS` for `setsockopt`, `NINA_SO_POLL` for
  `getsockopt`). `ioctl` other than `FIONREAD`
  (below) answers `ENOTTY`, which leaves the command to NuttX's own handling (the network device's
  `SIOC*` and so on). `socket(AF_INET, SOCK_CTRL)` (used by `ifconfig` and others) is answered
  `ENOTSUP`, which leaves it to NuttX.
- Sending and receiving use the Zephyr version's values: a TCP send takes at most 3584 bytes a
  call (`CONFIG_NANO_NINA_TCP_MAX_IO`), a datagram at most 512 (more gives `EMSGSIZE`), a receive at
  most 512. `sendmsg` joins several vectors, up to 3584 bytes, and writes them with one command (so
  that MQTT's header and payload go out together; NuttX's usrsock passes the vectors separately). A
  send with one vector is treated as `send`/`sendto`, with more as `sendmsg` (UDP `sendmsg` gives
  `EOPNOTSUPP`, as in the Zephyr version).
- How sending and receiving wait: each request makes one pass of the Zephyr version's wait loop in
  the calling thread, and usrsock waits between passes.
  - When a waiting TCP send finds that the NINA took nothing, the answer is `EAGAIN` with
    `SENDTO_READY` dropped; after `CONFIG_NANO_NINA_SEND_RETRY_MS` (default 1 ms, the Zephyr
    version's `CONFIG_NINA_SEND_RETRY_MS`) `nina_event` reports `SENDTO_READY` and usrsock repeats
    the request (the next pass). The state is checked with the NINA's poll at the first pass and
    then every 100 ms (the error flag gives `EPIPE`), and the call gives `EAGAIN` 10 s after its
    first pass. The deadline and the next check are kept per socket; a continued call is told from
    a new one by the data's address and length.
  - A send that does not wait (`MSG_DONTWAIT`, a non-blocking socket) gives `EAGAIN` at once when
    nothing is taken, as in the Zephyr version. A datagram the NINA did not take (the NINA reports
    every failure as zero bytes) gives `EAGAIN` at once whether the call waits or not. In these two
    cases and at the 10 s deadline the answer is `-ETIME` with `SENDTO_READY` kept, which patch 0005
    turns into `EAGAIN` that ends the call (stock usrsock repeats a waiting call's `EAGAIN`).
    `SENDTO_READY` is dropped only between the passes of a waiting send: with it dropped, a
    following blocking send would wait in usrsock without asking the driver, with no passes and no
    deadline. So a TCP socket always looks writable to `poll()` (the Zephyr version's `poll()` asks
    the NINA).
  - A receive first checks with the NINA's poll that data is there (as in the Zephyr version: the
    answer cannot tell "nothing" from an empty datagram); if not, the answer is `EAGAIN` with
    `RECVFROM_AVAIL` dropped. A waiting receive waits in usrsock for `RECVFROM_AVAIL`, which
    `nina_event` reports from its polls every 20 ms (NuttX handles `SO_RCVTIMEO`). After data is
    received `RECVFROM_AVAIL` is dropped too and `nina_event` polls again at once. usrsock passes a
    receive to the driver only with `RECVFROM_AVAIL`, so on an unmarked socket the first query of a
    receive or a `poll()` can wait for `nina_event`'s poll (up to 20 ms; the Zephyr version asks
    there and then). On a marked socket the layer asks first with `FIONREAD`, without that delay.
  - A poll that found data (`nina_event`'s, `FIONREAD`'s) stands for the next receive's first poll
    (nothing takes the data in between).
- A TCP receive of 0 means the peer closed or the connection failed (the NINA reports both as zero
  bytes, as in the Zephyr version). A UDP socket connected with `connect()` drops datagrams from
  other senders (as in the Zephyr version).
- `ioctl(FIONREAD)` is the NuttX version's own: one NINA poll there and then, in the calling thread;
  1 when data (for TCP also the peer's close) is there, 0 when not (the NINA does not give a byte
  count). With 1 the answer carries `RECVFROM_AVAIL`, so a following `recv(MSG_DONTWAIT)` reaches
  the NINA. The compatibility layer's `zsock_recv()` uses it (the Zephyr version's NINA `recv()`
  sends one poll on every call before it reads: the time synchronization's receive every 1 ms and
  the MQTT library's `recv(MSG_DONTWAIT)`; see the layer's table below).
- `getsockopt(NINA_SOL, NINA_SO_POLL)` (an int) is the NuttX version's own too: one pass of the
  Zephyr version's `do_poll()` for one socket. Without a NINA socket yet the NINA is not asked: TCP
  gives `POLLHUP`, UDP `POLLOUT`. Otherwise one poll: readable → `POLLIN`, writable → `POLLOUT`, the
  error flag → `POLLERR`; a failed poll gives its errno. The layer's `zsock_poll()` uses it.
- `common/drivers/nina/nina_sock.c` is checked by a host test (`common/tests/nina_sock.c`) with a fake
  `nina_request()` and clock.

### Name resolution

`common/drivers/nina/nina_dns.c` is a line-by-line port of the Zephyr version's `nina_dns.c` (the Zephyr version now uses this file too): it
resolves a name with the NINA's resolver commands (0x34 with the name, 0x35 to read the address) to
one IPv4 address. A numeric address is not sent to the NINA. The Zephyr version registers it as the
socket offload's `getaddrinfo()`, used by `zsock_getaddrinfo()`; the NuttX version has functions of
its own, `nina_getaddrinfo()` and `nina_freeaddrinfo()` (`common/drivers/nina/nina_dns.h`), and the compatibility
layer's `<zephyr/net/socket.h>` maps `zsock_getaddrinfo()`, `zsock_freeaddrinfo()` and
`struct zsock_addrinfo` to them. NuttX's `getaddrinfo()` (`CONFIG_LIBC_NETDB`, which queries a DNS
server over UDP itself) is not used; `CONFIG_NANO_NINA_DNS` cannot be on together with
`CONFIG_LIBC_NETDB`. Errors are NuttX's `EAI_*` values under the Zephyr version's `DNS_EAI_*` names
(`EAI_NONAME` for a name that does not resolve, `EAI_AGAIN` for an SPI failure, `EAI_FAIL` for an
address of the wrong length, `EAI_BADFLAGS`/`EAI_FAMILY`/`EAI_SOCKTYPE` for bad hints, `EAI_SERVICE`
for a bad service, `EAI_MEMORY`). A mutex covers the two commands (the NINA keeps one result). The
result comes from `malloc()`, not the Zephyr version's 2 KB heap. The host test is
`common/tests/nina_dns.c`. The only user now is the time synchronization when its NTP server is
given by name (the default is the gateway).

### Wi-Fi (network device `wlan0`)

| File | Role |
|---|---|
| `common/drivers/nina/nina_net.c` | The same file as the Zephyr version's (the repository's `common/`; `nina/Make.defs` finds it through `VPATH` and compiles it with the prelude, and the object is made in `nuttx/drivers`): scan, connect, disconnect, status, and the three threads `nina_scan_tid`, `nina_connect_tid` and `nina_monitor_tid` |
| `nina/nina_netdev.c` | Registers the NINA as the NuttX network device `wlan0` and turns wapi's wireless ioctls into the Zephyr Wi-Fi requests (`net_mgmt()`). NuttX port only |
| `nina/wifi_profile_null.c` | Empty versions of the Wi-Fi profile functions `nina_net.c` calls (`wifi_profile_pause_reconnect()` and so on), used only in a build without the stored credentials and automatic reconnection (`common/imu/wifi_profile.c`, the same file as the Zephyr version's, built by `nuttx/apps/wifi/`; when `CONFIG_NANO_WIFI_PROFILE` is not set) |
| `include/nina_net.h` | `nina_net_initialize()`. The `wifi_profile.h` that `nina_net.c` includes is the Zephyr version's own file, `common/imu/wifi_profile.h` (`nina/Make.defs` adds it to the include path) |

- `nina_net.c` uses Zephyr's APIs (`net_if`, `net_mgmt`, `wifi_mgmt`, kernel objects) through the
  compatibility layer (see "Zephyr API compatibility layer" below), so it runs as the same code as in
  the Zephyr version. Values and behaviour are the Zephyr version's: every 2 seconds, while
  connected, the monitor reads the NINA's state (0x20); after two disconnected readings in a row it
  treats the link as lost, clears the IP configuration, raises the disconnect event and asks for a
  reconnection. A connection hands the SSID (and passphrase) to the NINA, then reads the state every
  500 ms up to 60 times; once associated it turns power saving off (0x17), reads the IP
  configuration (0x21) into the interface, and raises the result event (failures as `-ETIMEDOUT` and
  so on). A connection whose request generation (`generation`/`request_generation`) has changed ends
  without a result. A scan while connected first treats the link as disconnected and asks for a
  reconnection when it ends (unless the firmware keeps the link: "Firmware additions" below). A manual disconnect stops reconnection (`wifi_profile_pause_reconnect()`).
  The passphrase is never logged.
- Start-up: the board's bring-up calls `nina_net_initialize()` after the LSM6DSOX and before the KV
  store (the Zephyr version initializes the interface at `POST_KERNEL` after the sensor and sets up
  NVS in `main()`). It starts the `net_mgmt` thread, registers `wlan0` as `NET_LL_IEEE80211` with
  `IFF_UP`, and calls `driver_init()` and `iface_init()` (dormant, MAC address read from the NINA,
  0x22) in Zephyr's order. That is the first command to the NINA, so the NINA reset (10 ms, 750 ms)
  happens here. The init task starts the three threads with `K_THREAD_DEFINE_START()`.
- Interface values: the IPv4 address, netmask and gateway are `wlan0`'s `d_ipaddr`, `d_netmask` and
  `d_draddr`, the MAC address is `d_mac`. NuttX's `ifconfig` (`/proc/net/wlan0`), `SIOCGIFADDR` and
  so on read them there. Zephyr's "dormant" is `IFF_RUNNING` clear (`RUNNING` appears when
  connected). `wlan0` has no `d_ifup`/`d_ifdown`, so `ifconfig wlan0 down`/`up` do nothing (the
  Zephyr version's interface is always up as well). NuttX's own TCP/IP is not used, so ARP and ICMP
  are off (`CONFIG_NET_ARP`, `CONFIG_NET_ICMP`). `CONFIG_NET_ETHERNET` stays: registering a
  `NET_LL_IEEE80211` device needs it (`netdev_register()` uses `CONFIG_NET_ETH_PKTSIZE`), and so does
  the MAC address field (`d_mac.ether`).

wapi's ioctls (`d_ioctl`, handled in the calling thread):

| ioctl | Handling |
|---|---|
| `SIOCSIWSCAN` | `NET_REQUEST_WIFI_SCAN`. An ESSID, a passive scan or channels are passed on, and the driver refuses them with `-ENOTSUP` as in the Zephyr version |
| `SIOCGIWSCAN` | `-EAGAIN` while scanning, the scan's error if it failed, else the results (up to 10 collected from the `SCAN_RESULT` events, in the NINA's order, as wireless events: BSSID, channel, RSSI in dBm, SSID, encrypted or not; nothing about encryption when the type is unknown). `-E2BIG` if the buffer is too small |
| `SIOCSIWAUTH` | Keeps the WPA version and the pairwise cipher |
| `SIOCSIWENCODEEXT` | Keeps the passphrase. What these two keep is used and forgotten by the next `SIOCSIWESSID` (the passphrase is wiped), so a `wapi essid` without a `wapi psk` before it connects without encryption |
| `SIOCSIWESSID` | `NET_REQUEST_WIFI_CONNECT` with the kept settings: WPA2 → `WIFI_SECURITY_TYPE_PSK`, none → `_NONE`, WPA → `_WPA_PSK`, WPA3 → `_SAE`, WEP → `_WEP` (the driver takes only NONE and PSK). With a passphrase but no version set: PSK (as Zephyr's `wifi connect` without `-k`). Flags 0 (the random SSID of wapi's `disconnect`) does nothing |
| `SIOCSIWAP` | 00:00:00:00:00:00 or ff:ff:ff:ff:ff:ff: `NET_REQUEST_WIFI_DISCONNECT`; anything else `-ENOTSUP` |
| `SIOCGIWESSID`, `SIOCGIWAP`, `SIOCGIWSENS` | SSID, BSSID and RSSI from `NET_REQUEST_WIFI_IFACE_STATUS` (asks the NINA). While disconnected: empty SSID, zero BSSID, `-ENOTCONN` for the RSSI |
| `SIOCGIWMODE`, `SIOCSIWMODE` | Managed (`IW_MODE_INFRA`) only |

Other wireless ioctls return `-ENOTTY`. The connection's result is not what `wapi essid` returns
(the Zephyr version's `wifi connect` also returns once the request is made); check `RUNNING` and
the IP address in `ifconfig`, or the ESSID in `wapi show`.

- The stored credentials and automatic reconnection (`nuttx/apps/wifi/`: the Zephyr version's own
  file `common/imu/wifi_profile.c`, copies of Zephyr's `subsys/net/lib/wifi_credentials/`, `wifi cred`;
  "Stored Wi-Fi credentials and automatic reconnection (`wifi cred`)" in `design/nuttx-porting.md`) use the layer's
  `net_mgmt()` (`NET_REQUEST_WIFI_CONNECT_STORED`), `net_mgmt_add_event_callback()`
  (`NET_EVENT_WIFI_CONNECT_RESULT`), `net_if_get_wifi_sta()`, `net_if_ipv4_get_global_addr()` and
  `k_current_get()` as the Zephyr version does. `nina_net.c` calls
  `wifi_profile_connection_allowed()` for every connection request and refuses one from the
  reconnection thread with `-ECANCELED` after a disconnection asked for (as in the Zephyr version).

### Firmware additions (patches 0003 to 0006)

The core (`nina_spi.c`, `nina_net.c`, `nina_sock.c`, `nina_wait.c`) behaves as in the Zephyr version
(described in the section of the same name in `zephyr/app/usb-console/drivers/README.md`; the
specification is "Host interface of 0003 to 0006" in `firmware/nina-fw/README.md`): 0x6A is asked
once when the interface is initialized (an older firmware's `0xEF` answer means no additions); with
bit 0 a scan while connected is not treated as a disconnection; with bit 1 the scan is asynchronous
(0x36, then 0x6B every 100 ms for up to 10 s, then 0x27); with bit 2 the socket waits wake on the
IRQ line's (GPIO2) edges. `nina version` shows the module's bits and those in use on its second
line. Checking with a poll command that the socket is writable before a TCP send's data goes
again is a change that does not depend on the firmware.

Bit 2 on NuttX:

- The GPIO2 interrupt of `nina/nina_os.c` (the board's `src/nano_gpio.c`, a callback of the layer's
  `gpio0`) counts the edges, posts the `sem_t` of every waiting thread (a broadcast) and wakes
  `nina_event` (`nina_poller_edge()`).
- `nina_event` starts its next pass as soon as the count has changed since the count noted before
  its last pass (or before the commands of the receive request that scheduled it), instead of after
  20 ms; otherwise after 100 ms (or a connection in progress's deadline, if earlier). When passes
  started by an edge keep reporting nothing, it waits a little before the next edge counts (1, 2,
  4, 8, 16 ms, then 20 ms).
- A waiting TCP send's next pass: after a pass that only polled (the socket was not writable),
  when the count has changed since the request noted it before its commands (100 ms at most, not
  past the send's deadline); after a pass that sent and got zero bytes, 1 ms later as before.
- The layer's `zsock_recv()` and `zsock_poll()` use the same waits (`nina_wait()`). A
  `zsock_poll()` that includes a NuttX socket (not a NINA one) looks again at least every 20 ms.
- Without `-DNINA_OS_EVENTS` in `nina/Make.defs`, the waits keep their intervals whatever the
  firmware.
- `nuttx/tests/nina_os.c` (`nina_os_events`) tests the interrupt and the waits of `nina/nina_os.c`.
  The passes of `nina_event` at the edges and the layer's waits have no host test.

### Passthrough mode (`nina passthrough confirm`)

A relay between the USB CDC ACM port and the NINA's UART (the ESP32's UART0), for reading and writing the
module's flash with esptool on the PC. It follows the FreeRTOS build's
`freertos/drivers/nina/nina_passthrough.c`, and its decisions (the request's mark, the lines, the rate,
the way out, the ring) are the ones shared by the three builds,
`common/drivers/nina/nina_passthrough_logic.h`. The NuttX side is the board's
`src/nano_nina_passthrough.c` (its parts without the hardware in `src/nano_nina_passthrough_glue.h`)
and `include/nano_nina_passthrough.h`, with `CONFIG_NANO_NINA_PASSTHROUGH` (default y).

| RP2040 | Passthrough mode | Normal firmware |
|---|---|---|
| GPIO8 | UART1 TX -> the module's UART0 RX | SPI1 RX (MISO) |
| GPIO9 | UART1 RX <- the module's UART0 TX | CS |
| GPIO2 | the module's GPIO0 (boot strap) | IRQ |
| GPIO3 | the module's RESETN (active low) | RESET |

- Way in: `nina passthrough confirm` (`nuttx/apps/nina`) writes a mark into the watchdog's scratch
  registers 0..3 and restarts through the watchdog (`nano_watchdog_reboot()` of the board's
  `src/nano_reset.c`, as the Pico SDK's `watchdog_reboot(0, 0, 0)`). `rp2040_boardinitialize()` reads
  and clears the mark first, so RESET, any later restart and a fatal error start the normal firmware.
  Nothing is written to the flash (key-value store).
- The start of the mode: `nano_bringup()` sets up the USB device only (CDC ACM, the same descriptors and
  serial number), none of SPI1, the NINA, I2C0, the sensor, `wlan0` or the key-value flash. The init
  task runs `nano_nina_passthrough_run()` (priority 120) instead of nsh, Telnet, Wi-Fi and the threads.
- Lines and rate: cdcacm's callback (`CAIOC_REGISTERCB`, called from the USB interrupt) reads the new
  state with `CAIOC_GETCTRLLINE` and `CAIOC_GETLINECODING` and applies it to the pins at once (RTS set
  releases the reset, DTR set pulls GPIO0 low), so every change of the host is applied in its order,
  however short. UART1 follows the line coding's rate (8N1 always). The release of GPIO0 after 750 ms,
  the log of a close, the LED (GPIO6) blinking at 5 Hz and the way out are done by the init task,
  which waits on a semaphore.
- Module to PC: UART1's receive interrupt (at 4 bytes in the FIFO, or after 32 bit times without more)
  moves the bytes into a 16 KiB ring (taken from the heap in this mode only); the kernel thread
  `nina_pt_rx` writes it to `/dev/ttyACM0`. At 921600 bps the 28 bytes left in the FIFO (300 us) are
  the margin for the interrupt's latency, and the ring holds 178 ms for the times the PC does not read.
  Bytes that find it full are counted.
- PC to module: `nina_pt_tx` reads `/dev/ttyACM0` and puts the bytes into UART1's transmit FIFO,
  waiting for the transmit interrupt while it is full. It reads no more until they are in, so the
  serial driver's receive buffer fills, cdcacm stops taking packets (the host waits) and nothing is lost.
- `/dev/ttyACM0` is not a console: no translation, no echo (the termios flags are cleared as well).
- Way out: closing the port at 300 bps (`stty -F /dev/ttyACM0 300`), or RESET. The module is started
  into its own firmware as `init_nina()` does it (GPIO0 high, RESETN low for 10 ms, 750 ms, GPIO0 an
  input), then the watchdog restarts the board.
- The RAM log (`dmesg` after the way out) gets a line at the start, at each close of the port and at the
  end, with the bytes each way, the bytes lost, and UART1's overruns, breaks and framing errors.
- No signal is used (the waits are semaphores, sleeps and blocking reads and writes). NuttX's UART1
  driver (`CONFIG_RP2040_UART1`) would take the pins at boot and is not used (UART1's registers are
  driven directly).
- The host test `nuttx/tests/nina_passthrough.c` (`script/host_tests.sh`) checks the conversion of
  cdcacm's values into the pins, the way out and the rate, the handling of the words of UART1's data
  register, and the confirmation word, argument counts and messages of `nina passthrough`. The shared
  decisions are tested by `freertos/tests/passthrough`. The interrupts, threads and cdcacm together
  have no host test.

The `bootsel` command (`nuttx/apps/bootsel`) restarts into BOOTSEL (`RPI-RP2`) through the board's
`nano_reset_usb_boot()` (`include/nano_reset.h`), the boot ROM's `reset_usb_boot(0, 0)` (no activity
LED, both the mass storage and PICOBOOT interfaces on).

## I2C0

The core is in the repository's `common/drivers/i2c_dw/` (it does not depend on the OS; the board's
`src/Make.defs` finds it through `VPATH`, and the objects are made in the board's directory); the
NuttX part is in the board's `src/`.

| File | Role |
|---|---|
| `common/drivers/i2c_dw/nano_i2c.c` | The core of the I2C0 controller. A copy of the controller part of the Zephyr version's `drivers/i2c_dw/i2c_dw.c` (the transfer's state machine, the body of the interrupt handler, `i2c_dw_data_ask()`, `_read()`, `_send()`, `i2c_dw_setup()`, `i2c_dw_configure()`, the register work of the initialization), of the DMA reads (`i2c_dw_rp_dma_read()` and the rest) and of the `i2c_dma` counters, with the same functions, branches, order, register values, time limit and error values. It includes the board's `nano_i2c_instance.h` at its end |
| `common/drivers/i2c_dw/nano_dma.c`, `nano_dma.h` | The core of the RP2040 DMA used by the DMA reads: Zephyr's `drivers/dma/dma_rpi_pico.c` (`dma_config()`, `dma_start()`, `dma_stop()`, the body of the interrupt handler), the channel allocation of the DMA API (`dma_request_channel()`, `dma_release_channel()`) and the pico-sdk calls that `i2c_dw.c` makes itself (`dma_channel_is_busy()`, `dma_irqn_acknowledge_channel()`, `dma_hw->abort`), written on the registers (their addresses are in `nano_dma.h`). It includes the board's `nano_dma_instance.h` at its end |
| `common/drivers/i2c_dw/i2c_dw_os.h` | The boundary: what the two cores need from the OS (the table below). It includes the board's `i2c_dw_os_impl.h` first and declares the functions that it must define |
| `common/drivers/i2c_dw/nano_i2c_dw.h` | Definitions and pure functions without OS headers: the definitions of the Zephyr version's `i2c_dw.h`, the SCL counts, the speed class of a frequency, the NuttX flags (`I2C_M_*`) turned into Zephyr's (`I2C_MSG_*`) and the words written to `IC_DATA_CMD`. Host test `common/tests/nano_i2c_dw.c` |
| `common/drivers/i2c_dw/nano_i2c_dw_registers.h` | A copy of the Zephyr version's `i2c_dw_registers.h` (the accessors are defined on plain volatile 32-bit accesses) |
| `common/drivers/i2c_dw/i2c_dw_rp2040_commands.[ch]` | The command words sent by DMA. The same files as the Zephyr version's. Host test `common/tests/i2c_dma_commands.c` |
| `common/drivers/i2c_dw/nano_i2c_dma.h` | The state that the `i2c_dma` command (`nuttx/apps/i2c_dma`) reads and sets: DMA on or off and the four counters |
| The board's `src/i2c_dw_os_impl.h` | The NuttX implementation of the boundary: semaphores (`sem_t`: `nxsem_post()`, `nxsem_tickwait_uninterruptible()`, `nxsem_wait_uninterruptible()`, `nxsem_reset()`), time (the system ticks), `nxsched_msleep()`, `up_irq_save()`, the messages (`struct i2c_msg_s`, with Zephyr's flags made by `nano_i2c_dw_msg_flags()`), `I2C_DW_CLOCK_SPEED` (`BOARD_PERI_FREQ`) |
| The board's `src/nano_i2c_instance.h` | The NuttX part of I2C0: the operations of `struct i2c_master_s` (`I2C_TRANSFER()` checks the request, takes the speed from the frequency and calls the core's transfer; `I2C_RESET()`), the interrupt handler attached with `irq_attach()`, the reset and the pins (`i2c_dw_pinctrl()`), `nano_i2c0_initialize()` |
| The board's `src/nano_dma_instance.h` | The NuttX part of the DMA: the reset of the DMA block and the handler of DMA_IRQ_0 and 1 |
| The board's `src/nano_i2c.h` | `nano_i2c0_initialize()` (called by `nano_bringup()` at boot, which registers `/dev/i2c0`) |

What the OS provides at the boundary (`i2c_dw_os.h`). All are static inline functions or macros, and
the OS's interrupt handlers and transfer entry call static inline functions of the core, so the
compiler makes one function of each (no extra calls on the transfer and interrupt paths). Those
marked "interrupt" are called from the interrupt handlers.

| Boundary | Meaning (Zephyr) | NuttX build |
|---|---|---|
| `i2c_dw_os_sem_t` | A counting semaphore (`k_sem`; `bus_sem` is used as a binary one without priority inheritance) | `sem_t` |
| `i2c_dw_os_sem_give()` (interrupt) | `k_sem_give()` | `nxsem_post()` |
| `i2c_dw_os_sem_take_ms(sem, ms)` | `k_sem_take(sem, K_MSEC(ms))`: 0 when taken, nonzero when the time (rounded up to ticks) passes first; not ended by signals | `nxsem_tickwait_uninterruptible(sem, MSEC2TICK(ms))` |
| `i2c_dw_os_timepoint_t`, `i2c_dw_os_timepoint_calc(ms)`, `i2c_dw_os_sem_take_until(sem, end)` | `sys_timepoint_calc()` and `k_sem_take(sem, sys_timepoint_timeout(end))` (after the deadline, one try without waiting) | The system ticks (`clock_systime_ticks()`) and `nxsem_tickwait_uninterruptible()` with what is left, at least 0 |
| `i2c_dw_os_sem_reset()` | `k_sem_reset()` | `nxsem_reset(sem, 0)` |
| `i2c_dw_os_bus_lock()`, `_unlock()` | `k_sem_take(K_FOREVER)` and `k_sem_give()` on `bus_sem`; the errno of a failed lock is what the transfer returns | `nxsem_wait_uninterruptible()`, `nxsem_post()` |
| `i2c_dw_os_msleep(ms)` | `k_msleep()` | `nxsched_msleep()` |
| `i2c_dw_os_uptime_ms()` | `k_uptime_get()` (may be a macro) | `TICK2MSEC(clock_systime_ticks())` (a macro) |
| `i2c_dw_os_irq_lock()`, `_unlock()` (interrupt) | `irq_lock()` (for `atomic_or()` and the DMA's channel table); nests | `up_irq_save()`, `up_irq_restore()` |
| `i2c_dw_os_msg_t`, `I2C_DW_OS_MSG_BUF()`, `_LEN()`, `_FLAGS()` | One message of a transfer (for Zephyr's `struct i2c_msg`: `buf`, `len`, `flags`, with STOP on the last message) | `struct i2c_msg_s`; the flags are made from it and its neighbours by `nano_i2c_dw_msg_flags()` |
| `I2C_DW_OS_DEV_HEAD` | The OS's device at the start of the controller's data (may be empty) | `struct i2c_master_s dev;` |
| `I2C_DW_CLOCK_SPEED` | `CONFIG_I2C_DW_CLOCK_SPEED` (MHz, 125) | `BOARD_PERI_FREQ / 1000000` |
| `i2c_dw_os_prepare(dw)` (`nano_i2c_instance.h`) | `reset_line_toggle_dt()` and `pinctrl_apply_state()`: reset I2C0, GPIO12 and 13 as I2C (no pulls, 4 mA, slow slew, Schmitt trigger and input enabled) | RESETS and `rp2040_gpio_*()` (`i2c_dw_pinctrl()`, the steps of Zephyr's `pinctrl_rpi_pico.c`) |
| `i2c_dw_os_irq_enable(dw)` (same) | `IRQ_CONNECT()`, `irq_enable()`: a handler of I2C0_IRQ that calls `i2c_dw_isr_handler(dw)` | `irq_attach()`, `up_enable_irq()` |
| `nano_dma_os_reset()`, `nano_dma_os_irq_enable()` (`nano_dma_instance.h`) | The reset of the DMA block; a handler of DMA_IRQ_0 and 1 that calls `dma_rpi_pico_isr_handler()` | RESETS, `irq_attach()`, `up_enable_irq()` |
| `app_log.h` | `LOG_MODULE_REGISTER()`, `LOG_ERR()`, `LOG_ERR_RATELIMIT()` (at the level 3 that the core sets) | `common/log/app_log.h` with its NuttX part `nuttx/drivers/include/app_log_os.h` (the RAM log) |
| `CONFIG_NANO_I2C0_DMA` | Build the DMA reads (the Zephyr version's `CONFIG_I2C_DW_RP2040_DMA`) | Kconfig |

The OS's I2C0 device (`nano_i2c_instance.h`) sets `base` of `i2c_0_runtime` (0x40044000), initializes
the semaphores (`device_sync_sem` and `rp_dma_sem` to 0, `bus_sem` to 1), calls `i2c_dw_turn_on()`
once at boot after `nano_dma_initialize()`, and sets `ready` when that succeeds. Its transfer entry
refuses what the Zephyr API cannot express (no message array, more than 255 messages, addresses or
speeds that differ between messages, a 10-bit address, a missing buffer, a device that is not ready),
returns 0 for no message, and otherwise returns `i2c_dw_transfer_msgs(dw, msgs, count, address,
speed)` (`speed` is Zephyr's speed class). The I2C0 and DMA interrupts run at the same priority, and
the handlers and the transfers run on one core. The DMA's channel allocation knows no other users and
the initialization resets the DMA block, so nothing else may use the DMA.

- The calling thread drives a transfer and the interrupt handler moves the bytes. The caller waits
  for the end on a semaphore (1 s; after that the transfer is aborted and returns `-ETIMEDOUT`). A
  bus semaphore serializes whole transfers (no priority inheritance, as in the Zephyr version).
- NuttX messages are handled as Zephyr messages: a STOP after the last message and after any
  message without `I2C_M_NOSTOP` whose next message has no `I2C_M_NOSTART`, a RESTART at the start
  of a message without `I2C_M_NOSTART` after one with `I2C_M_NOSTOP`. A change of direction (also
  from the last message of the previous transfer) adds a RESTART, as in the Zephyr version.
- The speed comes from the messages' frequency (100 kHz, 400 kHz, 1 MHz); when it changes, the
  counts are computed as Zephyr's `i2c_configure()` does. A speed that is not taken changes nothing
  and gives `-EINVAL` (the Zephyr version keeps the minimum counts then).
- Not ported: target (slave) mode, Intel LPSS DMA, `CONFIG_I2C_ALLOW_NO_STOP_TRANSACTIONS`, the SDA/SCL
  timeouts (`CONFIG_I2C_DW_EXTENDED_SUPPORT`), power management, PCIe, code for other SoCs, the bus
  recovery and check callbacks (not registered on this board; recovery does nothing), i2c1.
- The DMA reads (`CONFIG_NANO_I2C0_DMA`, on by default; the Zephyr version's
  `CONFIG_I2C_DW_RP2040_DMA`) copy the DMA part of `i2c_dw.c` with the same branches, order and
  values:
  - Conditions: DMA on (`i2c_dma mode`), the DMA ready, 32 to 3584 bytes
    (`I2C_DW_RP2040_DMA_MAX_READ`), ending with STOP, the last message of the transfer, a 7-bit
    address. Otherwise `-EAGAIN` sends the read to the interrupt path. When a channel cannot be had
    or configured, `dma_fallbacks` is counted and the read also goes to the interrupt path (nothing
    has been started yet).
  - The TX channel writes the read command words (16 bits: READ on each, RESTART on the first only
    when one is needed, STOP on the last) to `IC_DATA_CMD`; the RX channel takes the data from
    `IC_DATA_CMD` 8 bits at a time. `IC_DMA_TDLR` is 8, `IC_DMA_RDLR` 0, the only interrupts are
    STOP_DET, TX_ABRT, TX_OVER, RX_OVER, RX_UNDER and SCL_STUCK_LOW. The RX channel starts first,
    then TX, then `IC_DMA_CR` gets 3.
  - The read ends when TX done, RX done and STOP have all come, or on an error (an I2C error
    interrupt, a DMA bus error). The wait uses the transfer time limit (1 s) as one deadline and
    waits for what is left of it. An error gives `-EIO`, the deadline `-ETIMEDOUT`.
  - Clean-up: `IC_DMA_CR` 0, interrupt mask 0; on failure abort the I2C and stop both DMA channels
    (interrupt off, CHAN_ABORT, wait in 1 ms steps for at most 100 ms until BUSY and CHAN_ABORT clear,
    then acknowledge late completions (RP2040-E13) on both DMA_IRQ_0 and DMA_IRQ_1). If they do not
    stop, the controller is quarantined (`<err> i2c_dw: DMA channels did not stop; I2C controller
    quarantined`, `-EIO`, the channels are not released) and every later transfer gives `-EIO` until
    a reboot. Otherwise the interrupts are cleared and, on failure, the abort is given at most 100 ms
    to finish (else quarantine and `<err> i2c_dw: I2C abort did not finish; controller quarantined`).
  - As in Zephyr's RP2040 DMA driver, channel 0 is on DMA_IRQ_0 and the others on DMA_IRQ_1, a
    channel request never waits and a stop does not wait (the I2C side above waits). NuttX's
    `rp2040_dmac.c` (`CONFIG_RP2040_DMAC`) uses DMA_IRQ_0 for every channel, waits for a free channel
    and waits without a limit in its stop, so it is off (these reads are the only DMA user; the
    NINA's SPI does not use DMA).
  - `rp_dma_events` is updated with interrupts off (Zephyr's `atomic_or()`); the four counters are
    written only by the transfer, which holds the bus semaphore (32 bits, wrapping).

## LSM6DSOX (`lsm6dso`)

| File | Role |
|---|---|
| `common/drivers/lsm6dso/lsm6dso.[ch]` | The driver. The same files as the Zephyr version's (the repository's `common/`, compiled with the prelude; Zephyr's kernel, device, sensor, GPIO, I2C and log calls come from the compatibility layer, below). At its end `lsm6dso.c` includes `lsm6dso_instance.h` of the build's own driver directory, so `lsm6dso_init` and `lsm6dso_driver_api` stay `static` |
| `lsm6dso/lsm6dso_instance.h` | The device, in place of the Zephyr version's devicetree instance macros: one `DEVICE_DT_DEFINE()` with the board's values (INT1's `.fifo_gpio` is `<&gpio0 24 0>`) |
| `common/drivers/stmemsc/stmemsc.h`, `stmemsc_i2c.c`, `stmemsc_mdelay.c` | A copy of the I2C part of Zephyr's `drivers/sensor/st/stmemsc/` (the glue between ST's `stmdev_ctx_t` and Zephyr's I2C API; `stmemsc_mdelay()` is the layer's `k_msleep()`), shared with the FreeRTOS version (compiled without the prelude). `lsm6dso.h` includes `<stmemsc.h>`, which is this one through the include path |
| `lsm6dso/st_lsm6dso_reg.c` | A one-line wrapper that builds ST's `lsm6dso_reg.c` (`nuttx/stmemsc/`, not in Git) as it is |
| `common/drivers/lsm6dso/lsm6dso_compat.c` | Single reads through RTIO (the fallback for a submission that is not a stream). The same file as the Zephyr version's (the log is the layer's `<zephyr/logging/log.h>`, the level `CONFIG_SENSOR_LOG_LEVEL` is in the prelude) |
| `common/drivers/lsm6dso/lsm6dso_fifo.[ch]`, `lsm6dso_fifo_packet.h` | Parsing of the FIFO words and the format of the buffer's header. The same files as the Zephyr version's (the repository's `common/`; `lsm6dso/Make.defs` finds them through `VPATH`, and the objects are made in `nuttx/drivers`; the same for the next 4 rows) |
| `common/drivers/lsm6dso/lsm6dso_fifo_hw.[ch]`, `lsm6dso_fifo_stats.h`, `lsm6dso_attributes.h` | FIFO setup, status and burst reads, the FIFO statistics, the private attributes. The same files as the Zephyr version's (they use Zephyr's API through the compatibility layer) |
| `common/drivers/lsm6dso/lsm6dso_stream.c`, `lsm6dso_fifo_irq.[ch]` | The FIFO stream (below). The same files as the Zephyr version's |
| `common/drivers/lsm6dso/lsm6dso_fifo_decoder.[ch]` | The decoders of the stream's and the single reads' buffers (`get_decoder`). The same files as the Zephyr version's |
| `common/drivers/lsm6dso/lsm6dso_cal.[ch]`, `lsm6dso_zero.[ch]` | The conversion to physical units with the calibration, and the tracking of the gyro bias at rest (below). The same files as the Zephyr version's |
| the board's `src/nano_lsm6dso.[ch]` | Makes I2C0 the compatibility layer's bus device (`i2c0`, 100 kHz) and initializes `lsm6dsox` (from `nano_bringup()`) |
| the board's `src/nano_gpio.c` | GPIO bank 0 for the compatibility layer's `gpio0` (input set-up, levels, edge and level interrupts); `<nano_gpio.h>` (`common/compat`) |

What the driver's calls into the system are on NuttX:

| Zephyr | NuttX port |
|---|---|
| `<zephyr/logging/log.h>`, `CONFIG_SENSOR_LOG_LEVEL` (3) | the layer's `<zephyr/logging/log.h>` (`app_log.h`: the same lines in the RAM log), level 3 from the prelude |
| `k_mutex_init/_lock/_unlock` | the layer's `k_mutex` (an `rmutex_t`; a `K_FOREVER` wait ended by a signal is repeated) |
| `k_spin_lock/_unlock` | the layer's `k_spin_lock()` (`sched_lock()` and interrupts off) |
| `k_busy_wait()` | the layer's, `up_udelay()` |
| `k_msleep()`, `k_usleep()` | the layer's, `nxsched_msleep()`, `nxsched_usleep()` (rounded up to ticks plus one tick: Zephyr's rule, the same 100 µs tick) |
| `k_cycle_get_32()`, `k_cyc_to_ns_floor64()` | the layer's: the TIMER's low 32 bits (1 MHz) and ×1000 |
| `sys_get_le32()` | the layer's `<zephyr/sys/byteorder.h>` |
| `CONFIG_SENSOR_ASYNC_API` | defined by the prelude when `CONFIG_NANO_LSM6DSO_STREAM` is set (always on with `NANO_LSM6DSO`; as in Zephyr, `lsm6dso.c` does not build without it) |
| `DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` | the layer's `<zephyr/devicetree.h>`: 1 for `st_lsm6dso` on `i2c` only (the board has one LSM6DSOX, on I2C0) |
| the devicetree instance, `POST_KERNEL`, priority 90 | `DEVICE_DT_DEFINE(lsm6dsox, ...)` (`lsm6dso_instance.h`), `device_init()` in the board's bring-up |

- The board's `nano_bringup()` initializes the sensor right after I2C0 and before mounting the
  key-value store (NVS), the Zephyr version's order. A failure only leaves the device not ready
  (`device_is_ready()` is false, as with Zephyr's `do_device_init()`); the boot goes on.
- Not ported: SPI, the sensor hub (`lsm6dso_shub.c`), triggers (`lsm6dso_trigger.c`) and the die
  temperature. All are off in the Zephyr version's configuration (`CONFIG_LSM6DSO_SENSORHUB`,
  `_TRIGGER`, `_ENABLE_TEMP`) and not built there. Their `#ifdef` parts are in the shared
  `lsm6dso.c` and `lsm6dso.h`; the two files are only in the Zephyr version.
- The FIFO read time (`transfer_time_ns` in the statistics) is measured with the TIMER (1 µs
  steps); the Zephyr version uses the CPU clock (SysTick, 8 ns steps).

### Conversion to physical units and calibration

The conversion (`lsm6dso_cal.c`), the calibration lookup (`lsm6dso_cal_set_lookup()`,
`lsm6dso_cal_reload()`, `lsm6dso_cal_status()`), taking the bias again on the spot
(`lsm6dso_cal_zero()`) and the tracking during a stream (`lsm6dso_zero.c`) are the Zephyr
version's code and behave the same (the Zephyr version's driver notes, "Conversion to physical
units and calibration"). The driver does not read the KV store. The lookup function is registered
by `common/imu/imu_cal.c` (the same file as the Zephyr version's, built by `nuttx/apps/imu/`) in init's
`imu_cal_init()`; the driver calls it when it is registered, on `imu cal reload` and when a range
is set. The calibration applies to the output of the single reads (`sensor_channel_get()`) and of
the FIFO decoder, not to the RAW the acquisition service sends over MQTT.

### FIFO stream

`lsm6dso_stream.c` and `lsm6dso_fifo_irq.[ch]` are the same files as the Zephyr version's
(`common/drivers/lsm6dso/`); they use the kernel, RTIO, GPIO and sensor API under Zephyr's names from the
compatibility layer (next section). They behave as in the Zephyr version:

- `sensor_stream()` (`SENSOR_DT_STREAM_IODEV`; one trigger, `SENSOR_TRIG_FIFO_WATERMARK` with
  `SENSOR_STREAM_DATA_INCLUDE`, anything else is `-ENOTSUP`, a second stream `-EBUSY`) submits one
  multishot read; `lsm6dso_submit()` wakes the worker (`imu_fifo_worker`) in the caller.
- On its first run the worker starts the FIFO (watermark, accelerometer and gyroscope BDR, one
  timestamp every 8, stream mode), enables the rising-edge interrupt of INT1 and then routes the
  FIFO threshold to INT1. If INT1 is already high when enabled, it queues one run at once.
- An INT1 edge → the GPIO interrupt queues work on the system work queue (`sysworkq`,
  `k_work_submit()`) → there `stream_wakeup()` wakes the worker
  (`k_work_reschedule_for_queue(.., K_NO_WAIT)`) → the worker reads the FIFO status and, at the
  watermark or above, reads all words in one transfer and completes one batch. The completed
  multishot goes straight back to the worker (`lsm6dso_submit()`), which looks at the status again
  at once; with nothing there it schedules another look in 20 ms
  (`k_work_schedule_for_queue(.., K_MSEC(20))`), which also catches a missed edge.
- Buffers come from the context's memory pool (at least the packet header plus the watermark's
  words, at most the header plus 512 words; none available ends the stream with `-ENOMEM`). An
  overrun ends it with `-EOVERFLOW`, an I2C failure with that error. Ending puts the FIFO back in
  bypass and turns the INT1 routing and the interrupt off.
- Graceful stop (`lsm6dso_fifo_request_stop()`): the worker stops batching (BDR 0), delivers the
  remaining words as a last partial batch, then puts the FIFO in bypass and ends with
  `-ECANCELED`. A cancellation (`rtio_sqe_cancel()`) is noticed at the worker's next run (INT1 or
  the 20 ms look) and stops at once, without pausing, ending with `-ECANCELED` (or the stop's
  error); the words left in the FIFO are dropped.
- The stream does not change the ODRs (`imu start` sets them and they stay after the stop, as in
  the Zephyr version).

## Zephyr API compatibility layer (`compat`)

The compatibility layer provides, under Zephyr's names and no more than needed, the part of
Zephyr's API that the copied driver and its callers (the acquisition service, the calibration, the
sensor command) use. Zephyr's sensor subsystem, RTIO and kernel as a whole are not ported. The layer
is in two places:

- **The OS-independent part** (the repository's `common/compat/`): the headers of Zephyr's API
  (`common/compat/zephyr/...`), the work items, RTIO, the block pool, the device list, `gpio0`, the
  `net_mgmt` events, the Wi-Fi requests, the sensors' default iodev, and the copies of Zephyr's
  libraries that the builds on the layer use (`sensor_shell_stream.c` and `sensor_shell.h`; the
  stored Wi-Fi credentials, `wifi_credentials.c`, `wifi_credentials_backend_settings.c` and
  `wifi_credentials_internal.h`; the key-value store's NVS and settings, see "Key-value store:
  copies of Zephyr's NVS and settings" below; the MQTT client is `common/imu/mqtt_min.c`), and
  boundary headers (the table of thread priorities and stacks `priorities.h`, `cpu_load.h` of
  `cpu_load_get()` with its arithmetic `nano_cpuload.h`, `nano_gpio.h` under `gpio0`, the
  key-value area's flash `nano_flash_kv.h`). The command handling of the shell's
  `zephyr/shell/shell.h` is `common/shell/zshell.c` ("Shell" below). The Zephyr build does not use
  it (it has Zephyr itself).
- **The NuttX implementation** (`nuttx/drivers/compat/`): the kernel part (`zephyr_kernel.c`,
  `zephyr_mutex.c`), the OS headers (`zephyr_os.h`, `zephyr_os_spinlock.h`, `zephyr_os_atomic.h`,
  `zephyr_os_shell.h`), the OS parts that shared `.c` files include at their end
  (`zephyr_rtio_workq_os.h`, `zephyr_net_mgmt_os.h`), the headers that depend on NuttX as a whole
  (`zephyr/net/socket.h`, `net_if.h`, `zephyr/sys/clock.h`, `__assert.h`, `zephyr/drivers/i2c.h`),
  the `.c` files of I2C, the interface, the chip ID and the sensor clock, and the prelude
  (`common_prelude.h`).

The include path is the NuttX implementation (`$(TOPDIR)/external/compat`), then the shared part
(`common/compat`). The kernel side (`compat/Make.defs`) and the applications (`apps/sensor`,
`apps/wifi`) find the shared `.c` files through `VPATH` and compile them with the prelude. The host
tests have a fake kernel (the OS headers and their functions) in `common/tests/zephyr_fakes`, ahead
of the NuttX implementation on the include path.

| Header (in `common/compat/` unless marked) | Contents |
|---|---|
| `zephyr/device.h` | `struct device` (name, config, API, state, data, init function), `device_is_ready()`, `device_init()` (the meaning of Zephyr's `do_device_init()`), `DEVICE_DT_GET()`, `DEVICE_DT_DEFINE()`, `DEVICE_API()`, `device_get_binding()` (node name), `device_get_by_dt_nodelabel()` (node label), `DEVICE_API_IS(sensor, dev)` (below) |
| `zephyr/devicetree.h` | `DT_NODELABEL()`, `DT_BUS()`, `DT_REG_ADDR()`; the nodes are `gpio0`, `i2c0`, `lsm6dsox` and `timer`, which has no device (the TIMER's address 0x40054000, for `time_sync_mono_us()` of `time_sync.c`), only. `DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` (Zephyr's definition): 1 for `st_lsm6dso` on `i2c`, 0 for any other compatible or bus (`lsm6dso.h`). `DT_HAS_CHOSEN()` is always 0 (the Zephyr build chooses no `zephyr,settings-partition` either) |
| `zephyr/kernel.h` | `k_timeout_t` (`K_NO_WAIT`, `K_FOREVER`, `K_MSEC()`, `K_SECONDS()`; ticks of 100 µs), `k_mutex`, `k_sem` (`K_SEM_DEFINE`), `k_poll()` on semaphores (`K_POLL_TYPE_SEM_AVAILABLE`, `K_POLL_MODE_NOTIFY_ONLY`), work (`k_work`, `k_work_delayable`, `k_work_q`, `k_work_submit()`, `k_work_cancel_sync()`, `k_work_schedule_for_queue()`, `k_work_reschedule_for_queue()`, `k_work_queue_start()`), `K_THREAD_STACK_DEFINE()`, `K_PRIO_PREEMPT()`, `k_thread_name_set()`, the system work queue `k_sys_work_q`, `K_MUTEX_DEFINE()`, `K_THREAD_DEFINE()` (below), `k_current_get()` (the calling thread's NuttX thread ID, comparable with the `k_tid_t` of `K_THREAD_DEFINE()`), `k_uptime_get()`, `k_sleep()`, `k_msleep()`, `k_usleep()`, `k_busy_wait()` (below), `k_cycle_get_32()`, `k_cycle_get_64()`, `k_cyc_to_us_floor64()`, `k_cyc_to_us_ceil64()`, `k_cyc_to_ns_floor64()`, `sys_clock_hw_cycles_per_sec()` (below) |
| `zephyr/spinlock.h`, `zephyr/sys/atomic.h` | `k_spin_lock()` (below), `atomic_t` and `atomic_get/set/clear/cas/or/inc/add()` |
| `zephyr/rtio/rtio.h`, `zephyr/rtio/work.h` | RTIO (below) and the RTIO work queue (`rtio_work_req_alloc()`, `_submit()`) |
| `zephyr/drivers/gpio.h` | `struct gpio_dt_spec`, `gpio_is_ready_dt()`, `gpio_pin_configure_dt()` (inputs only), `gpio_pin_interrupt_configure_dt()`, `gpio_pin_get_dt()`, callbacks (`gpio_init_callback()`, `gpio_add_callback()`, `gpio_remove_callback()`) |
| `zephyr/drivers/sensor.h` | `struct sensor_value`, the channel, attribute and trigger enumerations (Zephyr's numbers), `struct sensor_driver_api`, `sensor_attr_set()`/`_get()`, `sensor_sample_fetch()`/`_chan()`, `sensor_channel_get()`, `sensor_trigger_set()`, the unit conversions. The async API: `struct sensor_chan_spec`, the decoders (`struct sensor_decoder_api`, `sensor_three_axis_data` and others), `SENSOR_DT_READ_IODEV()`, `SENSOR_DT_STREAM_IODEV()`, `sensor_get_decoder()`, `sensor_stream()`, `sensor_read()`, `sensor_read_async_mempool()`, `sensor_processing_with_callback()`. The print formats of the decoded data, `PRIsensor_*_data` (Zephyr's `sensor_data_types.h`; the Q value format is in `dsp/print_format.h`) |
| `zephyr/drivers/sensor_clock.h` | `sensor_clock_get_cycles()`, `sensor_clock_cycles_to_ns()` (below) |
| `zephyr/drivers/i2c.h` (NuttX) | `struct i2c_dt_spec`, `i2c_burst_read_dt()`, `i2c_write_dt()`, `i2c_write_read_dt()`, `i2c_configure()`, the speed classes (`I2C_SPEED_*`, `I2C_SPEED_SET()`) |
| `zephyr/sys/util.h`, `sys/mpsc_lockfree.h`, `sys/mem_blocks.h`, `dsp/types.h` | `BIT()`, `ARRAY_SIZE()`, `MIN()`/`MAX()`, `IS_ENABLED()`, `CONTAINER_OF()`, `GENMASK()`/`GENMASK64()`/`FIELD_GET()`, `STRINGIFY()`, `KB()`, `ZTESTABLE_STATIC`, `hex2bin()`/`bin2hex()` (implemented by the copy of `lib/utils/hex.c`) and others (it also includes `<string.h>`, as Zephyr's does), RTIO's queues and block pool, `q31_t` |
| `zephyr/sys/__assert.h` (NuttX) | `__ASSERT()`, `__ASSERT_NO_MSG()` (NuttX's `DEBUGASSERT()`; `assert()` in the host tests) |
| `zephyr/dsp/print_format.h` | A copy of Zephyr's header of that name (with only a leading comment and the include of `<zephyr/sys/util.h>` added): the Q value print format `PRIq()`, `PRIq_arg()` |
| `zephyr/types.h` | `<stddef.h>` and `<stdint.h>` (what `lsm6dso.h` reads it for) |
| `zephyr/init.h` | An empty header (`lsm6dso.c` includes it; `SYS_INIT()` and the init levels are used only by the Zephyr version's instance macros) |
| `zephyr/sys/iterable_sections.h` | `STRUCT_SECTION_FOREACH()` only, over an always empty section (the port has no Zephyr linker sections). The sensor shell includes it (and uses it only with `CONFIG_SENSOR_INFO`, which the Zephyr build does not set); the settings walk the section of static handlers (none in the port) |
| `zephyr/sys/sys_io.h` | `sys_read32()` (one volatile 32-bit read without a barrier, as Zephyr's on Arm), `mem_addr_t` |
| `zephyr/sys/printk.h`, `zephyr/sys/byteorder.h`, `zephyr/sys/cpu_load.h` | `snprintk()` (`snprintf()`, as in the Zephyr version's configuration with `CONFIG_PICOLIBC=y`), `sys_put_be16/32()`/`sys_get_be16/32()`/`sys_get_le32()`, `cpu_load_get()` (`cpu_load.h`; the board implements it) |
| `zephyr/drivers/hwinfo.h` | `hwinfo_get_device_id()` (below) |
| `zephyr/shell/shell.h` | `shell_print()`, `shell_info()`, `shell_warn()`, `shell_error()`, `shell_fprintf()`, `struct shell` (a shell backend; its members are the OS part `zephyr_os_shell.h`), the command table macros (`SHELL_CMD_REGISTER()`, `SHELL_STATIC_SUBCMD_SET_CREATE()`, `SHELL_DYNAMIC_CMD_CREATE()`, `SHELL_CMD_ARG()`, `SHELL_CMD()`, `SHELL_COND_CMD()`, `SHELL_EXPR_CMD_ARG()`, `SHELL_HELP()`, `SHELL_SUBCMD_SET_END`), `shell_device_get_binding()`, `shell_device_filter()` (below) |
| `zephyr/net/socket.h` (NuttX) | `zsock_inet_pton()`, `zsock_inet_ntop()` (the C library's `inet_pton()`, `inet_ntop()`), `zsock_getaddrinfo()`, `zsock_freeaddrinfo()` (the NINA's resolver, `nina_getaddrinfo()`; errors are NuttX's `EAI_*`), `zsock_socket()` (NuttX's `socket()` marked `NINA_SO_CALLER_POLLS`; if the mark is refused, closed and -1), `zsock_sendto()`, `zsock_close()` (NuttX's `sendto()`, `close()`), `zsock_recv()` (asks the NINA there and then with `ioctl(FIONREAD)` and reads with `recv(MSG_DONTWAIT)` when something is there; nothing there: `EAGAIN` with `ZSOCK_MSG_DONTWAIT`, otherwise asks again every 20 ms (with socket events, at the IRQ line's edge, 100 ms at most); as the Zephyr version's NINA `recv()`, `do_recv()` and `udp_recv()`, asks on every call and every 20 ms while it waits; `SO_RCVTIMEO` and `O_NONBLOCK` are not looked at), `ZSOCK_MSG_DONTWAIT`, `zsock_connect()`, `zsock_send()`, `zsock_sendmsg()` (`connect()`, `send()`, `sendmsg()`), `zsock_poll()`, `struct zsock_pollfd`, `ZSOCK_POLLIN` and the others (`poll()` and `struct pollfd`; how the NINA is asked, below) |
| `zephyr/sys/clock.h` (NuttX) | `sys_clock_gettime()`, `sys_clock_settime()` (`SYS_CLOCK_REALTIME`: NuttX's `clock_gettime()`, `clock_settime()` with `CLOCK_REALTIME`; 0 or `-EINVAL`), `NSEC_PER_USEC`, `USEC_PER_SEC` |
| `zephyr/net/net_if.h` (NuttX) | `struct net_if` (one NuttX network device), `net_if_get_default()`, `net_if_get_wifi_sta()`, the IPv4 address, netmask and gateway (`net_if_ipv4_addr_add()`, `_rm()`, `net_if_ipv4_get_global_addr()`, `net_if_ipv4_set_netmask_by_addr()`, `net_if_ipv4_set_gw()`, `net_if_ipv4_get_gw()`), `net_if_dormant_on()`/`_off()`, `net_if_is_admin_up()`, `net_if_set_link_addr()` (below) |
| `zephyr/net/net_mgmt.h` | `net_mgmt()` (requests), `struct net_mgmt_event_callback`, `net_mgmt_init_event_callback()`, `net_mgmt_add_event_callback()`, `_del_event_callback()`, `net_mgmt_event_notify_with_info()` (below) |
| `zephyr/net/wifi_mgmt.h` | The Wi-Fi types (security, band, MFP, state, mode; Zephyr's values), `struct wifi_scan_params`, `wifi_scan_result`, `wifi_connect_req_params`, `wifi_iface_status`, `wifi_status`, `struct wifi_mgmt_ops`, the offloaded interface API and `NET_DEVICE_DT_OFFLOAD_DEFINE()`, the requests `NET_REQUEST_WIFI_SCAN`, `_CONNECT`, `_DISCONNECT`, `_IFACE_STATUS`, `_CONNECT_STORED` (with `CONFIG_NANO_WIFI_CREDENTIALS`), the events `NET_EVENT_WIFI_SCAN_RESULT`, `_SCAN_DONE`, `_CONNECT_RESULT`, `_DISCONNECT_RESULT`, `wifi_mgmt_raise_connect_result_event()`, `_disconnect_result_event()` (below), `wifi_security_txt()`, `wifi_band_txt()` |
| `zephyr/net/wifi_credentials.h` | A copy of Zephyr's header of that name: `struct wifi_credentials_header`, `_personal`, `WIFI_CREDENTIALS_FLAG_*`, `wifi_credentials_set_personal_struct()`, `_get_by_ssid_personal_struct()`, `_delete_by_ssid()`, `_for_each_ssid()`, `_is_empty()` and the rest. The configuration values (`CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES` and so on) come from the OS's `CONFIG_NANO_WIFI_CREDENTIALS_*` (in the NuttX build, `nuttx/apps/wifi/Kconfig`). The port's own `wifi_credentials_init()` (called by init; Zephyr's `SYS_INIT()`). The functions are in `common/compat/wifi_credentials.c` (the copy of Zephyr's library) and `wifi_credentials_backend_settings.c`, which stores them in the settings |
| `zephyr/net/wifi_utils.h` | `wifi_utils_validate_chan()` (`_2g`, `_5g`, `_6g`), `net_bytes_from_str()` (in Zephyr's `<zephyr/net/net_ip.h>`) |
| `zephyr/settings/settings.h`, `zephyr/kvss/nvs.h`, `zephyr/sys/slist.h`, `zephyr/sys/list_gen.h` | Unchanged copies of Zephyr's headers of the same names ("Key-value store" below): the settings API (`settings_subsys_init()`, `settings_save_one()`, `settings_load_one()`, `settings_delete()`, `settings_load_subtree_direct()` and others), NVS's API and `struct nvs_fs`, singly linked lists |
| `zephyr/drivers/flash.h`, `zephyr/storage/flash_map.h` | The flash API that NVS and the settings use (`flash_read()`, `flash_write()`, `flash_erase()`, `flash_flatten()`, `flash_get_parameters()`, `flash_get_write_block_size()`, `flash_get_page_info_by_offs()`, `struct flash_parameters`) and the flash map (`flash_area_open()`, `flash_area_close()`, `flash_area_get_sectors()`, `PARTITION_ID(storage_partition)`). The only device is the key-value area's flash (`zephyr_flash_kv.c`, below) |
| `zephyr/sys/crc.h`, `zephyr/toolchain.h` | Declarations of `crc8_ccitt()` and others (implemented by the copy of `subsys/crc/crc8_sw.c`), `__packed`, `__weak` |
| `nano_flash_kv.h` | The key-value area's constants (8 KiB from `0xffe000`, 256-byte pages, 4 KiB sectors), the OS boundary ("Key-value store" below), the range checks, the split into pages and the check of what the area holds (Zephyr's NVS, the earlier NuttX build's format, erased, other) |
| `zephyr_kv_config.h` | The Zephyr build's Kconfig values read before the copied NVS and settings (`CONFIG_SETTINGS_NVS_SECTOR_COUNT` and others; "Key-value store" below) |
| `kv_store_os.h` | The OS part of `common/imu/kv_store.c`: the `kv wipe` subcommand (the Zephyr build's `zephyr/app/usb-console/src/kv_store_os.h` is empty) |
| `zephyr/logging/log.h` | `LOG_MODULE_REGISTER()`, `LOG_ERR()` and the other logging macros. It only includes the port's `app_log.h` (`common/log/`; the RAM log of `dmesg`) |
| `zephyr/net/net_ip.h` | The IPv4 address type `struct in_addr` (`imu_raw_mqtt.h`). It only includes NuttX's `<netinet/in.h>` (the same member `s_addr`, network byte order) |
| `zephyr_compat.h` | `zephyr_compat_initialize()` (called by the board's bring-up, below), `K_THREAD_DEFINE_START()` (called by init) |
| `sensor_shell.h` | A copy of Zephyr's `drivers/sensor/sensor_shell.h` (the sensor shell's shared types; the Zephyr build reads Zephyr's) |
| `zephyr_os.h`, `zephyr_os_spinlock.h`, `zephyr_os_atomic.h`, `zephyr_os_shell.h` (NuttX) | The OS headers (below): NuttX's types (`rmutex_t`, `sem_t`, a wdog, `pid_t`), the tick, the priorities, the TIMER, `sched_lock()` and `up_irq_save()`, the takeover of `<nuttx/atomic.h>`'s names, `struct shell` (the terminal's path and the command's task) |
| `zephyr_rtio_workq_os.h`, `zephyr_net_mgmt_os.h` (NuttX) | The OS parts that `zephyr_rtio_workq.c` and `zephyr_net_mgmt.c` include at their end: the entries of the threads `RTIO WQ 0` and `net_mgmt` and their start as kernel threads (`kthread_create_with_stack()`) |
| `common_prelude.h` (NuttX) | The prelude (below) |

| Source (in `common/compat/` unless marked) | Contents |
|---|---|
| `zephyr_mutex.c` (NuttX) | `k_mutex` on NuttX (`rmutex_t`). Zephyr's `k_mutex` is unlocked when all zero (`lock_count` 0), and Zephyr's MQTT library locks the client's mutex in `mqtt_abort()` before `mqtt_client_init()`; NuttX's all-zero mutex counts as held by thread 0, and the lock never returns. So `K_MUTEX_DEFINE()` and `k_mutex_init()` set a mark (`ready`), and one without it (all zero) is initialized by its first lock or unlock (once, under the layer's spin lock) |
| `zephyr_kernel.c` (NuttX) | The OS part on NuttX: `k_sem` (`sem_t`, the limit checked in `k_sem_give()`), `k_poll()`, the work queue threads (kernel threads on static stacks), the timer of delayable work (a wdog), thread names, the start of `K_THREAD_DEFINE()` threads, `k_uptime_get()`, `k_sleep()`, `k_msleep()`, `k_usleep()`, `k_busy_wait()`, `zephyr_compat_initialize()` |
| `zephyr_work.c` | Work items and queues: the used part of Zephyr's `kernel/work.c` (OS-independent) |
| `zephyr_rtio.c`, `zephyr_rtio_workq.c`, `zephyr_mem_blocks.c` | The RTIO executor (`subsys/rtio/rtio_executor.c`), the RTIO work queue (`rtio_workq.c`; its thread is the OS part `zephyr_rtio_workq_os.h`), the block pool (`lib/mem_blocks`) |
| `zephyr_gpio.c` | The `gpio0` device (Zephyr's `gpio.h` logic and `gpio_rpi_pico.c`) on the board's `<nano_gpio.h>` |
| `zephyr_sensor_clock.c` (NuttX) | The sensor clock on the board's TIMER (`nano_mono_us()`) |
| `sensor_default_rtio.c` | A copy of Zephyr's `drivers/sensor/default_rtio_sensor.c` (only the log replaced): the sensors' iodev API, the fallback for single reads, the default decoder |
| `zephyr_device.c` | Device initialization and the list of devices (`gpio0`, `i2c0`, `lsm6dsox` in the Zephyr build's order of initialization, with their node labels and whether the API is a sensor API; `gpio0` with the OS's configuration `CONFIG_NANO_LSM6DSO_STREAM`) |
| `zephyr_i2c.c` (NuttX) | I2C (on a NuttX I2C master) |
| `zephyr_hwinfo.c` (NuttX) | `hwinfo_get_device_id()` on the board's chip ID (`nano_chip_id()`) |
| `zephyr_net_if.c` (NuttX), `zephyr_net_mgmt.c`, `zephyr_wifi_mgmt.c` | The interface (values of the NuttX network device), the `net_mgmt` event queue and thread (the thread is the OS part `zephyr_net_mgmt_os.h`; its stack size is `NANO_STACK_NET_MGMT` of `priorities.h`), the Wi-Fi requests (a copy of the checks of Zephyr's `subsys/net/l2/wifi/wifi_mgmt.c`) and events, `NET_REQUEST_WIFI_CONNECT_STORED` (a copy of `connect_stored_command()` of the same file; with `CONFIG_NANO_WIFI_CREDENTIALS`). Only with `CONFIG_NANO_NINA_NET` |
| `zephyr_wifi_utils.c` | Copies of `wifi_security_txt()` and `wifi_band_txt()` (Zephyr's `wifi_mgmt.c`), the channel checks (`wifi_utils.c`) and `net_bytes_from_str()` (`subsys/net/ip/utils.c`), for `wifi cred`. OS-independent. Only with `CONFIG_NANO_NINA_NET` |
| `sensor_shell_stream.c` | A copy of Zephyr's `drivers/sensor/sensor_shell_stream.c` (the sensor shell's processing thread). It includes the OS's `sensor_shell_os.h` first, and the processing thread's callback is `sensor_shell_os_processing_callback()` (in the NuttX build, `nuttx/apps/sensor/sensor_shell_nuttx.c`). Built by `apps/sensor` |
| `wifi_credentials.c`, `wifi_credentials_backend_settings.c` | Copies of Zephyr's `subsys/net/lib/wifi_credentials/` (the stored Wi-Fi credentials). Built by `apps/wifi`. Two changes in the backend: the read buffer is aligned to 8 bytes (it is read as a struct; the Cortex-M0+ faults on an unaligned access) and the index formats are `%zu` |
| `subsys/kvss/nvs/`, `subsys/settings/`, `subsys/crc/crc8_sw.c`, `lib/utils/hex.c` | Unchanged copies of Zephyr v4.5.0's NVS, settings, CRC-8 and hex conversions ("Key-value store" below). Built by `apps/imu` |
| `zephyr_flash_kv.c` | The key-value area's flash device and flash map ("Key-value store" below). Built by `apps/imu` |
| `kv_store_wipe.c` | `kv wipe confirm` (erases the area and initializes the settings again). Built by `apps/imu` |

- **Work**: the flags (RUNNING, CANCELING, QUEUED, DELAYED) and return values are those of
  `kernel/work.c`. `submit` does nothing if already queued (0) and queues a running item again on
  its queue (2). `schedule` does nothing while delayed or queued (a running item can be
  scheduled); `reschedule` stops the delay and schedules again (`K_NO_WAIT` queues at once).
  `k_work_cancel_sync()` removes a queued item and waits for a running one (one waiting thread
  only; Zephyr allows several). After an item, the queue's thread calls `sched_yield()` if more are
  pending (Zephyr's `k_yield()`). The delay timer is a wdog; a delay ends at the current tick +
  ticks + 1, as in Zephyr. Not ported: flush, drain, plug, stopping a queue, the work timeout,
  `k_work_cancel()`, `k_work_cancel_delayable()`.
- **OS headers** (of the layer): the shared headers hold no OS types; they include these headers
  from the OS's directory (`nuttx/drivers/compat/` in the NuttX build, `common/tests/zephyr_fakes/`
  in the host tests). The list of what each must provide is in the leading comment of the shared
  header that includes it.
  - `zephyr_os.h` (included first by `<zephyr/kernel.h>`): the OS headers needed before the layer,
    `Z_USEC_PER_TICK`, `struct k_mutex` and `Z_MUTEX_INITIALIZER()` (an all-zero one must work as
    an unlocked mutex), `struct k_sem`, `Z_SEM_INITIALIZER()`, `K_SEM_MAX_LIMIT`, `struct _timeout`,
    `_timeout_func_t`, `k_tid_t`, `k_current_get()`, `K_PRIO_PREEMPT()` (Zephyr's priority as the
    OS's, keeping the order), `k_cycle_get_32()`, `k_cycle_get_64()` (1 MHz). The functions
    (`k_sem_*()`, `k_mutex_*()`, `k_poll()`, `k_sleep()` and the others, `k_work_queue_start()`,
    `z_add_timeout()`, `z_abort_timeout()`, `z_thread_static_start()`,
    `zephyr_compat_initialize()`) are the OS's `.c` files (`zephyr_kernel.c` and `zephyr_mutex.c`
    in the NuttX build).
  - `zephyr_os_spinlock.h` (included by `<zephyr/spinlock.h>`): `struct k_spinlock`,
    `k_spinlock_key_t`, `k_spin_lock()`, `k_spin_unlock()` (below).
  - `zephyr_os_atomic.h` (included by `<zephyr/sys/atomic.h>` before it defines its names): the OS
    headers that have names the layer takes over, and the `#undef` of their macros (below). Empty in
    the host tests.
  - `zephyr_os_shell.h` (included by `<zephyr/shell/shell.h>`): `struct shell` ("Shell" below).
  - `zephyr_rtio_workq_os.h`, `zephyr_net_mgmt_os.h` (included by the shared `.c` files at their
    end): the threads' entries and their start (`rtio_workq_initialize()`,
    `net_mgmt_event_init()`), on the files' `static` variables (semaphores, stacks).
  - Headers that are the OS's as a whole: `zephyr/net/socket.h`, `zephyr/net/net_if.h`,
    `zephyr/sys/clock.h`, `zephyr/sys/__assert.h`, `zephyr/drivers/i2c.h`.
  - Boundary headers that the shared files read: `priorities.h` (`NANO_STACK_NET_MGMT`; in
    `common/compat/`, on `NANO_PRIO_FROM_ZEPHYR()` of the OS part `priorities_os.h`, in the NuttX
    build in `nuttx/drivers/include/`), `nano_gpio.h` (`zephyr_gpio.c`; in `common/compat/`, the
    board implements it), `app_log.h` (`<zephyr/logging/log.h>`, `sensor_default_rtio.c`, the
    credentials; in `common/log/`, on the OS part `app_log_os.h`), `<cpu_load.h>`
    (`<zephyr/sys/cpu_load.h>`; `common/compat/`), `sensor_shell_os.h` (`sensor_shell_stream.c`). The configuration
    values `CONFIG_NANO_LSM6DSO_STREAM` (`zephyr_device.c`) and `CONFIG_NANO_WIFI_CREDENTIALS`,
    `_MAX_ENTRIES`, `_SAE_PASSWORD_LENGTH`, `_CONNECTION_TIMEOUT` (the Wi-Fi headers and
    `zephyr_wifi_mgmt.c`) must be defined before the layer's headers (in the NuttX build by
    `<nuttx/config.h>`, which the prelude reads for the shared `.c` files).
- **Spin locks**: Zephyr's single-CPU `k_spin_lock()` masks interrupts, and no other thread runs
  until it is released (a thread made ready by `k_sem_give()` or the like runs at the release).
  NuttX switches to a thread that `nxsem_post()` makes ready at once, even with interrupts masked,
  so the NuttX build's `k_spin_lock()` (`zephyr_os_spinlock.h`) is `sched_lock()` plus
  `up_irq_save()` (`sched_lock()` does
  nothing in an interrupt handler). The `atomic_*()` functions use it too (the Cortex-M0+ has no
  exclusive loads and stores; Zephyr's ARMv6-M build masks interrupts as well).
- **Clashing NuttX names**: NuttX's `<nuttx/atomic.h>` (read by many kernel headers) has its own
  `atomic_t`, `atomic_set()` and others with other meanings. The layer's `<zephyr/sys/atomic.h>`
  reads it first (`zephyr_os_atomic.h`), then takes the names over for the file (`atomic_t` is `zephyr_atomic_t`, NuttX's
  macros are undefined). A file that uses both includes the NuttX headers first (for the files of
  `common/`, the prelude does). The board's bring-up does not include the Zephyr headers, only
  `zephyr_compat.h`.
- **Prelude** (`common_prelude.h`): when the NuttX build compiles the sources shared with the Zephyr
  build and the layer's shared `.c` files (the repository's `common/`), its build definitions (the
  `Makefile` of `apps/imu`, `apps/wifi` and `apps/sensor`, `compat/Make.defs`, `lsm6dso/Make.defs`,
  `nina/Make.defs`, the board's `src/Make.defs`) include it before each file with `-include` (as the Zephyr build reads its
  configuration into every file with `-imacros autoconf.h`). It holds `<nuttx/config.h>`, the NuttX
  headers needed before the layer (those that `<zephyr/kernel.h>` reads first: `<sys/types.h>`,
  `<nuttx/irq.h>`, `<nuttx/mutex.h>`, `<nuttx/semaphore.h>`, `<nuttx/wdog.h>`, `<stdio.h>`) and the
  values of the Zephyr Kconfig options that the shared sources use (`CONFIG_SENSOR_LOG_LEVEL` 3, the
  Zephyr build's value) and their names (`CONFIG_NINA_NTP_SERVER`, `_PORT` and `_INTERVAL_MS` of
  `time_sync.c`, set to the NuttX build's options `CONFIG_NANO_NINA_NTP_*`; `CONFIG_SENSOR_ASYNC_API`
  of `lsm6dso.c` and `lsm6dso.h`, set when `CONFIG_NANO_LSM6DSO_STREAM` is). So the shared sources
  have no lines that include NuttX headers. The socket headers are not in it (names such as
  `accept()` would collide with the sources' own functions; the layer's `<zephyr/net/socket.h>`
  reads them for the files that need them). The host tests use the same prelude (without
  `__NuttX__`, only the Kconfig values; the `time_sync` test and the tests of the LSM6DSO driver put
  a fake `<nuttx/config.h>` before it).
- **Headers of each OS** (of the shared sources): the few lines of a shared source that differ between the builds are in a
  small header of the same name, of which the Zephyr and the NuttX build each have their own
  (besides `lsm6dso_instance.h`, the LSM6DSO device, above):
  `sensor_shell_os.h` (included first by `sensor_shell.c` and by the layer's
  `sensor_shell_stream.c`; the NuttX one, `nuttx/apps/sensor/`, reads the Zephyr build's
  configuration values and the log and makes the two lines with which `sensor get` waits for its
  result to be printed, and the processing thread's callback, call functions; the Zephyr one,
  `zephyr/app/usb-console/drivers/sensor_shell/`, does nothing) and `wifi_profile_os.h` (the
  declaration of the reconnection thread's `wifi_reconnect_tid`: `const k_tid_t` in the Zephyr one,
  `zephyr/app/usb-console/src/`, as `K_THREAD_DEFINE()` makes it; without `const` in the NuttX one,
  `nuttx/apps/wifi/`, because the layer's `K_THREAD_DEFINE()` writes the value when the thread is
  started).
- **Semaphores and `k_poll()`**: `k_sem_give()` stops at the limit and wakes the threads in
  `k_poll()` on that semaphore (they are told, the count is not taken: Zephyr's
  `K_POLL_MODE_NOTIFY_ONLY`). `k_sem_take()` gives 0, `-EBUSY` (`K_NO_WAIT`) or `-EAGAIN` (timed
  out). `k_poll()` handles semaphores only.
- **RTIO**: a copy of Zephyr's in the build's configuration (`CONFIG_RTIO_SUBMIT_SEM`,
  `_CONSUME_SEM`, `_SYS_MEM_BLOCKS`; no callbacks, no user mode). The sizes come from the caller's
  `RTIO_DEFINE_WITH_MEMPOOL(name, submissions, completions, blocks, block size, alignment)`
  (`imu_stream.c`: 4, 16, 128, 64, 4). An iodev's submit runs in the context of the
  `rtio_submit()` caller. A successful multishot goes back to its iodev before its completion is
  posted (its pool buffer unbound); an error or a cancellation frees its buffer and the submission.
  A completion that finds no free entry is dropped and counted in `xcqcnt` (its buffer stays
  allocated, nobody can release it; as in Zephyr). `rtio_cqe_consume()` does not wait and takes
  `consume_sem`. A handle is the submission's pool index and generation; every free advances the
  generation, so an old handle neither resolves nor cancels. `rtio_sqe_cancel()` marks the
  submission and calls the iodev's cancel function (if any). Pool allocation takes the first run
  of free blocks (first fit), trying from `max` down to `min` one block at a time. Differences from
  Zephyr: the queues (`struct mpsc`) mask interrupts instead of being lock-free, and the pools'
  free lists are filled at their first allocation instead of at boot (`rtio_init()`), in the same
  order. Not ported: chains and transactions, the executor's own operations (callback, await,
  delay), writes and the other operations, `rtio_sqe_copy_in()`, `rtio_cqe_copy_out()`,
  `rtio_pool`, completion callbacks, user mode.
- **RTIO work queue**: as Zephyr's `rtio_workq.c`, a pool of 4 requests and one thread
  (`RTIO WQ 0`, priority 0 → 118, stack 1024), run in order. The single reads' fallback
  (`lsm6dso_compat.c`) uses it. NuttX turns spaces in task names into `_`, so `ps` shows
  `RTIO_WQ_0`.
- **GPIO**: one port, `gpio0` (GPIO0..29), inputs only (outputs are `-ENOTSUP`). Pin set-up,
  interrupts (edges and levels, logical or physical), levels and callbacks are copies of Zephyr's
  `gpio.h` and `gpio_rpi_pico.c`; the board's `src/nano_gpio.c` writes the registers as the
  pico-sdk's `gpio_set_pulls()`, `gpio_set_function()`, `gpio_set_dir()` and
  `gpio_set_irq_enabled()` do (the pin's events in `INTR` are cleared before they are enabled or
  disabled). The interrupt comes from NuttX's `rp2040_gpio` bank interrupt (which clears the
  enabled events, then calls the pin's handler).
- **Sensor clock**: the Zephyr version uses the system clock (SysTick, 125 MHz,
  `k_cycle_get_64()`), the NuttX port the board's TIMER (1 MHz from the crystal). Stream batch and
  single read times are whole microseconds.
- **Start-up**: the board's bring-up calls `zephyr_compat_initialize()` before the DMA, I2C0 and the
  LSM6DSOX; it starts the system work queue (`sysworkq`, priority 119, stack 1024), the RTIO work
  queue and `gpio0` (Zephyr starts them at `POST_KERNEL` before the sensor too). The LSM6DSOX's
  initialization (`lsm6dso_stream_init()`) creates `imu_fifo_worker`.
- **Static threads** (`K_THREAD_DEFINE()`): Zephyr's arguments define a thread with a static stack,
  the priority `K_PRIO_PREEMPT(prio)` and the token itself as its name (`imu_stream_tid`). Zephyr
  starts such threads by itself before `main()`; NuttX has no list of them, so init starts each
  one with `K_THREAD_DEFINE_START(name)` (`zephyr_compat.h`) at the beginning of the part that does
  what `main()` does. The thread is a NuttX kernel thread (`kthread_create_with_stack()`) and ends
  when its entry function returns. Only `options` 0 and `delay` 0 are accepted (anything else
  fails to build).
- **Time**: `k_uptime_get()` has the value of Zephyr's `k_ticks_to_ms_floor64(k_uptime_ticks())`
  (the tick count in ms, rounded down). The NuttX tick is the board's TIMER over 100 µs (the
  tickless timer), so it reads the TIMER once and divides by 1000 (the same value because 1000 is
  a multiple of the tick; checked by a host test). It is 64 bits wide, without the wrap after about
  5 days of the 32-bit `clock_systime_ticks()`. `k_sleep()` calls `sched_yield()` for `K_NO_WAIT`
  and otherwise sleeps with `nxsched_ticksleep()` until the current tick + ticks + 1 (the rounding
  of Zephyr's `z_add_timeout()`). It always returns 0, as there is no `k_wakeup()`. `k_msleep()`
  and `k_usleep()` are NuttX's `nxsched_msleep()` and `nxsched_usleep()`: the time rounded up to
  ticks, then the same wake-up as `k_sleep()` (Zephyr's rule); 0 or less yields, as Zephyr's
  does; no signal ends the sleep; they return 0. `k_busy_wait()` is `up_udelay()`, which polls the
  TIMER (the board's `nano_oneshot.c`). The LSM6DSO driver uses these three.
- **`hwinfo_get_device_id()`**: as the Zephyr version's RP2040 driver (`hwinfo_rpi_pico.c` without
  `CONFIG_HWINFO_RPI_PICO_CHIP_ID`), the external flash's 64-bit unique ID (command 0x4B) in the
  order the flash sends it (8 bytes at most). The board reads it once at boot (`nano_chipid.c`);
  this returns the copy, or `-EAGAIN` if the board has not read it yet (Zephyr's driver cannot
  fail).
- **Shell**: `shell_print()`, `shell_info()`, `shell_warn()` and `shell_error()` add a newline
  (Zephyr prints information in green, warnings in yellow and errors in red; no colors here). `sh`
  says where the text goes.
  - `sh` is NULL: the calling task's stdout (`shell_print()`, `shell_info()`) and stderr
    (`shell_warn()`, `shell_error()`). The commands whose tables are written with
    `common/shell/subcmd.h` (`kv`, `nina` and others) pass NULL.
  - `sh` is a backend: what a command registered with `SHELL_CMD_REGISTER()` (`sensor`, `imu`,
    `ntp`) gets. The NuttX build's backends (`nuttx/apps/subcmd/zshell_os_impl.h`; `struct shell` is
    in `zephyr_os_shell.h`) keep one per terminal that a command's stdout is (`/dev/ttyACM0`
    for USB, `/dev/telnet0` for Telnet; `F_GETPATH`, and since a Telnet session's device passes that
    on to its socket, a search of `/dev` for the inode number of `fstat()`). Like Zephyr's shell
    instances they stay for as long as the system runs. Text from the command itself (the running
    task) goes to its stdout and stderr; text from another task or thread (the sensor shell's
    processing thread, after the command has returned) opens the terminal each time, writes it with
    one `write()` and closes it again. While no Telnet client is connected the terminal cannot be
    opened, so the text is dropped, and it goes to the next client (as with Zephyr's Telnet
    backend). A Telnet session's device is open only while text is written, so the end of the
    session (when its last descriptor is closed) and the next connection are not held up.
  - The command table macros have the form of Zephyr's `shell.h`; `SHELL_CMD_REGISTER()` makes the
    builtin command's entry. By default the entry is `main()`, and the application's main source
    (`MAINSRC`) is the builtin command as it is (`sensor_shell.c`, `imu_stream.c`). A file that is
    not a main source is compiled with `Z_SHELL_CMD_NAMED_MAIN`, which its build definition adds for
    that file only, and the entry is `int z_shell_main_<name>(int argc, char *argv[])`, which the
    application's `main()` calls (`ntp` of `time_sync.c`: `apps/imu/Makefile` adds it for
    `time_sync.c`, and `main()` of `apps/ntp/ntp_main.c` calls `z_shell_main_ntp()`).
    `z_shell_cmd_main()` of `common/shell/zshell.c` follows Zephyr's `shell.c`
    `execute()`, `exec_cmd()` and `cmd_precheck()`, `shell_wildcard.c` and `shell_help.c`: it matches
    the words that nsh has split (calling the dynamic subcommand functions in Zephyr's order),
    expands wildcards, checks the argument count, prints the help (the `SHELL_HELP()` description
    and `Usage:` line, wrapped at 80 columns) and calls the handler, whose `argv` is NULL after the
    last word as in Zephyr. Differences from Zephyr: a subcommand set says whether it is dynamic in
    its `struct shell_cmd_entry` (Zephyr tells from the linker section its `union shell_cmd_entry`
    is in); no select mode, aliases, remote commands, raw arguments or Tab completion (nsh's);
    spaces where Zephyr moves the cursor with VT100 sequences.
  - `zshell.c` is OS-independent. The OS part (listed in `common/shell/zshell_os.h`) is
    `struct shell`, the output to a backend `z_shell_backend_vfprintf()`, a function that returns
    the backend of the calling task's session, and two marks around the handler's call. The help and
    the errors are printed with `printf()` and `fprintf(stderr)`, so stdout and stderr of the task
    that runs a command must be its session's terminal (on NuttX each command's task has them).
  - `shell_device_get_binding()` and `shell_device_filter()` are Zephyr's (`shell_utils.c`), on the
    device list above (`DEVICE_API_IS(sensor, dev)` is a mark in that list; Zephyr checks that the
    API table is in the class's linker section).
- **Network**: `zsock_inet_pton()` and `zsock_inet_ntop()` are the C library's functions. While
  networking is off, `imu` selects `CONFIG_LIBC_IPv4_ADDRCONV` for the IPv4 conversions. Sockets are
  the NINA's usrsock; `zsock_socket()`, `zsock_connect()`, `zsock_send()`, `zsock_sendmsg()`,
  `zsock_sendto()` and `zsock_close()` are NuttX's functions of the same names (a blocking socket
  waits with the NINA driver's deadlines).
- **`zsock_poll()`**: works as the Zephyr version's NINA `poll()` (`do_poll()` in
  `nina_socket.c`). In the calling thread it asks about each socket with
  `getsockopt(NINA_SOL, NINA_SO_POLL)` (one NINA poll command) and gives the `ZSOCK_POLLIN` and
  `ZSOCK_POLLOUT` asked for, `ZSOCK_POLLERR` (the NINA's error flag) and `ZSOCK_POLLHUP` (a TCP
  socket without a NINA socket yet). When nothing is ready it asks again after
  `k_sleep(K_MSEC(20))`, and returns 0 once the timeout (on `k_uptime_get()`; -1 for none) has gone
  by. A failed NINA poll returns -1 with its errno (the Zephyr version marks only that socket
  `ZSOCK_POLLERR` and does not fail the whole call). A descriptor that is not a NINA socket gets
  NuttX's `poll()` once without waiting (`ZSOCK_POLLNVAL` for a bad one), as in the Zephyr version.
  NuttX's `poll()` on the NINA's sockets and `nina_event` are not used. Used by `imu_raw_mqtt.c`
  for its input check (every 100 ms, timeout 0) and the CONNACK wait (5000 ms).
- **Cycle counter**: the Zephyr version's `k_cycle_get_*()` counts the 125 MHz SysTick (which falls
  behind under load); the NuttX version's is the board's TIMER (`nano_mono_us()`, 1 MHz from the
  crystal, as for the sensor clock). A cycle is a microsecond, so the conversions to microseconds
  (`k_cyc_to_us_floor64()`, `_ceil64()`) return the value as it is. `k_cycle_get_32()` is the low
  32 bits and wraps after about 71.6 minutes (the Zephyr version's after about 34 s), and
  `k_cyc_to_ns_floor64()` multiplies by 1000. `imu_raw_mqtt.c` times its queue lock waits and its
  publishes with it, the LSM6DSO driver its FIFO reads.
- **Interface** (`net_if.h`): the one interface is the NINA's `wlan0`; `net_if_get_default()` and
  `net_if_get_wifi_sta()` both return it (NULL before it is registered). It has one address (adding
  replaces it), always "preferred" (the Zephyr version's configuration has no IPv4 duplicate
  detection either), so `net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED)` returns it while
  there is one. Values are read and written under the network device's lock (`netdev_lock()`, as
  `ifconfig`'s ioctls do). `net_in_addr` is NuttX's `in_addr`. Interface events (`NET_EVENT_IF_UP`
  and so on, address added or removed) are not raised (nothing in the port uses them).
- **`net_mgmt`**: a request `net_mgmt(NET_REQUEST_X, iface, data, len)` calls
  `net_mgmt_NET_REQUEST_X()` in the calling thread, as Zephyr's macro does. Events follow the Zephyr
  version's configuration (`CONFIG_NET_MGMT_EVENT_THREAD`, `_QUEUE`, `_INFO`, a queue of 5 entries,
  10 ms of waiting for room): the information is copied into the queue, and the thread `net_mgmt`
  (priority −1 → 119, stack 2048) takes the events in order and calls each callback whose mask has
  the event (with `cb->info` and `info_length` set, under the callback list's lock). An event that
  finds no room within 10 ms is dropped (the Zephyr version's warning is compiled out at log level
  0). Event codes are bits of this port (not Zephyr's 64-bit layer and command values) compared by
  the same names. `nina_net_initialize()` starts the thread before the interface (Zephyr's
  `net_init()` order).
- **Wi-Fi requests** (`zephyr_wifi_mgmt.c`): as in Zephyr's `wifi_mgmt.c`, `-ENOTSUP` if the
  interface's API (named with `NET_DEVICE_DT_OFFLOAD_DEFINE()`) lacks the driver's operation,
  `-ENETDOWN` without `IFF_UP`, and the same checks of the connection parameters (channel, SSID length
  1 to 32, passphrase length 8 to 64 (5/10/13/26 for WEP), required fields per security type) before
  the driver is called (the Zephyr version's configuration has no wpa_supplicant). Scan results
  become one `NET_EVENT_WIFI_SCAN_RESULT` each, the end `NET_EVENT_WIFI_SCAN_DONE` (a
  `struct wifi_status`). `NET_REQUEST_WIFI_CONNECT_STORED` reads the stored networks in index order,
  makes connection parameters from each (channel 0 is "any", timeout 0 is 30 seconds, a type out of
  range is no encryption, band and MFP from the flags), issues `NET_REQUEST_WIFI_CONNECT` and stops
  at the first one accepted. It always returns 0 (as in Zephyr; the Zephyr version's configuration
  compiles this file's log messages out, so it logs nothing either).
- An I2C bus is a device holding a NuttX I2C master and a speed (the board binds them with
  `i2c_compat_bind()`). In Zephyr the bus driver keeps the speed and `i2c_configure()` changes it
  for every later transfer. A NuttX transfer carries a frequency in each message, so the layer's
  transfers (`i2c_burst_read_dt()` and so on) put the device's speed in their messages. Users of
  the NuttX master itself, such as the `i2c` tool, run at their own frequency.
- `i2c_configure()` takes only the classes the Zephyr version's RP2040 driver takes (100 kHz,
  400 kHz, 1 MHz); for anything else it changes nothing and returns `-EINVAL`. It ignores the mode
  bits (that driver forces controller mode too). While I2C0 is quarantined after a DMA failure,
  Zephyr's `i2c_configure()` returns `-EIO`; the layer records the speed and returns 0, and the
  next transfer gives `-EIO`.
- NuttX's `<nuttx/i2c/i2c_master.h>` has functions of its own named `i2c_write()` and `i2c_read()`
  and `I2C_SPEED_*` in Hz. In the layer, Zephyr's `i2c_write()` and `i2c_write_read()` on a bus
  device are named `i2c_compat_write()` and `i2c_compat_write_read()` (the `_dt` forms keep
  Zephyr's names). In a file that includes `<zephyr/drivers/i2c.h>`, `I2C_SPEED_*` are Zephyr's
  classes (include NuttX's header first if both are needed).

### Key-value store: copies of Zephyr's NVS and settings

To write the same bytes to the flash as the Zephyr build, the NuttX build (and the FreeRTOS build)
use unchanged copies of the parts of Zephyr v4.5.0's NVS and settings that the Zephyr build uses,
with the Zephyr build's configuration. When the Zephyr tree (`zephyr/zephyr`) is there, a host
test (`zephyr_copies` in `common/tests/run.sh`) checks with `cmp` that the copies are the
originals.

| Copy (`common/compat/`) | Original (Zephyr v4.5.0) | Changes |
|---|---|---|
| `subsys/kvss/nvs/nvs.c`, `nvs_priv.h` | `subsys/kvss/nvs/` | none |
| `subsys/settings/src/settings.c`, `settings_store.c`, `settings_init.c`, `settings_nvs.c`, `settings_priv.h` | `subsys/settings/src/` | none |
| `subsys/settings/include/settings/settings_nvs.h`, `settings_file.h` | `subsys/settings/include/settings/` | none |
| `subsys/crc/crc8_sw.c` | `subsys/crc/crc8_sw.c` | none |
| `lib/utils/hex.c` | `lib/utils/hex.c` | none |
| `zephyr/kvss/nvs.h`, `zephyr/settings/settings.h`, `zephyr/sys/slist.h`, `zephyr/sys/list_gen.h` | `include/zephyr/...` | none |
| `wifi_credentials_backend_settings.c` | `subsys/net/lib/wifi_credentials/` | the two in the table above |

- Configuration: the copied sources are built with `zephyr_kv_config.h` in front (the Zephyr
  build's `.config` values: `CONFIG_SETTINGS_NVS_SECTOR_SIZE_MULT` 1,
  `CONFIG_SETTINGS_NVS_SECTOR_COUNT` 8, `CONFIG_SETTINGS_DYNAMIC_HANDLERS`,
  `CONFIG_MULTITHREADING`, log level 3; `CONFIG_NVS_LOOKUP_CACHE`, `CONFIG_NVS_DATA_CRC`,
  `CONFIG_NVS_INIT_BAD_MEMORY_REGION` and `CONFIG_SETTINGS_NVS_NAME_CACHE` off, as in the Zephyr
  build). They test the options that are off with `#if`, so they are built without `-Wundef`;
  `nvs.c` prints `uint32_t` values with `%d` (Zephyr's `uint32_t` is `unsigned int`, newlib's and
  NuttX's `unsigned long`, both 32 bits), so it is built without `-Wformat`.
- The layout on the flash and the values that decide it (where the Zephyr build has them):

  | Value | Here | Zephyr build |
  |---|---|---|
  | Area | `0x2000` from `0xffe000` (`nano_flash_kv.h`, `<zephyr/storage/flash_map.h>`) | `storage_partition` of `app.overlay` |
  | Sectors | 4096 bytes × 2 | `erase-block-size` of `rp2040.dtsi` × `SECTOR_SIZE_MULT` 1, as many as fit in 8 KiB (`settings_nvs.c`) |
  | Write block, erase value | 1, 0xFF | `flash_rpi_parameters` of `flash_rpi_pico.c`, `write-block-size` of the DT |
  | Allocation table entry | 8 bytes, `crc8_ccitt(0xff, ...)` | `nvs_priv.h` (unchanged) |
  | Name and value IDs | 0x8000, 0x8001 on, +0x4000 | `settings_nvs.h` (unchanged) |
  | Values | calibration 56 bytes, text, `imu.session` 4 bytes, a credential 224 bytes | `imu_cal_codec.c`, `imu_stream.c` (shared), `struct wifi_credentials_personal` (its layout fixed to the Zephyr build's ELF by `BUILD_ASSERT`s; arm-none-eabi's 1-byte enums) |

- Assumptions kept: the settings' lock (`K_MUTEX_DEFINE(settings_lock)`, a mutex that starts all
  zero) and NVS's lock are the layer's `k_mutex` (recursive: `kv wipe` initializes the settings
  again while it holds the settings' lock). Initialization is, as in Zephyr, the first
  `settings_subsys_init()` (tried again by the next call after a failure).
- **The flash boundary** (`nano_flash_kv.h`; each OS implements it: the NuttX build in the board's
  `src/nano_flash_kv.c`, the FreeRTOS build with the Pico SDK's `flash_range_program()` and
  `flash_range_erase()` with interrupts off):

  ```c
  bool nano_flash_kv_os_ready(void);
  int nano_flash_kv_os_read(uint32_t offset, void *buf, uint32_t len);
  int nano_flash_kv_os_program(uint32_t page_offset, const uint8_t page[256]);
  int nano_flash_kv_os_erase(uint32_t sector_offset);
  ```

  `offset` is from the start of the flash (XIP 0x10000000). While `ready` is false, the flash map
  treats the device as missing (`-ENODEV`). `read` returns what the flash holds now (also right
  after a program or an erase). `program` writes the page at a multiple of 256 (0xFF where nothing
  is written; programming only clears bits), `erase` the sector at a multiple of 4096. Each
  returns 0 or a negative errno and is called from threads only, one at a time (the layer holds
  its lock). The caller (`zephyr_flash_kv.c`) passes only aligned requests inside the area, but
  the OS checks the range again and returns `-EINVAL` without touching the flash otherwise.
  Interrupts are off, and no code runs from the flash, while a page is programmed or a sector
  erased (the NuttX build: for one page or one sector at a time).
- `zephyr_flash_kv.c`: the flash device (`z_flash_kv_device`) and the flash map. It accepts
  requests inside the area only (the Zephyr build's driver accepts the whole flash), splits writes
  into 256-byte pages and erases into sectors for the boundary. `flash_area_open()` checks once
  after boot what the area holds; if it is neither Zephyr's NVS nor erased, it logs that and
  returns `-EILSEQ` (the settings are not initialized and nothing is written). After
  `kv wipe confirm` it checks again.
