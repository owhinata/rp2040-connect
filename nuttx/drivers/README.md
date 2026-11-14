# Drivers (NuttX port)

English | [日本語](README_ja.md)

`nuttx/drivers/` holds the port's code that NuttX builds on the kernel side (through the symlink
`nuttx/nuttx/external`, into `libexternal.a`). The board's part is in
`nuttx/boards/arduino-nano-rp2040-connect/src/`. The Zephyr version's driver notes are
`zephyr/app/usb-console/drivers/README.md`. The port is in progress; this document covers only
the parts ported so far.

## NINA-W102 (`nina`)

### SPI transfer layer

| File | Role |
|---|---|
| `nina/nina_spi.c` | Framing of commands and answers (`nina_request()`, `nina_ip_config()`). A copy of the functions of the same names in the Zephyr version's `drivers/nina/nina_wifi.c`, with the branches, order, constants and error values matching one to one |
| `nina/nina_proto.h` | The public declarations, an unchanged copy of the Zephyr version's file |
| `nina/nina_os.[ch]` | The lock (a recursive mutex with priority inheritance), sleeps and the millisecond clock: Zephyr's `K_MUTEX`, `k_sleep()` and `k_uptime_get()` |
| `include/nano_nina.h`, the board's `src/nano_nina.c` | SPI1 and the four lines (CS, READY, RESET, IRQ): what Zephyr's devicetree and its SPI and GPIO drivers set up |

- A request is handled synchronously in the calling thread. The mutex serializes requests, and CS
  stays asserted from the start of a phase until its last byte, including the wait for READY.
- The NINA is reset once, on the first request (10 ms, 750 ms). At boot the board only sets up
  SPI1 and its pins and drives CS inactive.
- The SPI is NuttX's `rp2040_spi.c`, polled (no DMA). Each transfer takes `SPI_LOCK`; the first
  one sets mode 0, 8 bits and `CONFIG_NANO_NINA_SPI_FREQUENCY`. `SPI_SELECT` is not used.
- The time limits and the wiring are in the "NINA-W102" section of
  `nuttx/README.md`.
- `nina/nina_spi.c` is checked by a host test (`nuttx/tests/nina_spi.c`) with a fake board and
  clock.

### Sockets (usrsock)

The Zephyr version uses a socket offload (`drivers/nina/nina_socket.c`); the NuttX version uses
usrsock, with `usrsock_request()` in the kernel (NuttX's `CONFIG_NET_USRSOCK_CUSTOM`). NuttX's own
TCP/IP stack is not used (`NET_TCP_NO_STACK`, `NET_UDP_NO_STACK`).

| File | Role |
|---|---|
| `nina/nina_sock.[ch]` | The socket table (10 sockets) and the NINA's socket commands: `nina_socket_create()`, `do_bind()`, `do_listen()`, `do_accept()`, `do_connect()`, `do_close()` and `do_setsockopt()` of the Zephyr version's `nina_socket.c`, and for sending and receiving `udp_send()`, `do_send()`, `do_sendmsg()`, `udp_recv()` and `do_recv()`, with the names, branches, order, constants and errno names kept (one pass of a send or receive wait loop per request). `FIONREAD` (`nina_sock_fionread()`) is the NuttX version's own. OS-independent |
| `nina/nina_usrsock.c` | Dispatches usrsock's requests to the functions above and answers them (ACK) |
| `nina/nina_poller.c` | The thread `nina_event`: turns the state of the NINA's sockets into usrsock events, and times the retries of a waiting send |
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
  a request wakes it.
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
  `ENOPROTOOPT` (NuttX handles `SO_RCVTIMEO` and `SO_SNDTIMEO` itself). `ioctl` other than `FIONREAD`
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
    receive to the driver only with `RECVFROM_AVAIL`, so the first query of a receive or a `poll()`
    can wait for `nina_event`'s poll (up to 20 ms; the Zephyr version asks there and then).
  - A poll that found data (`nina_event`'s, `FIONREAD`'s) stands for the next receive's first poll
    (nothing takes the data in between).
- A TCP receive of 0 means the peer closed or the connection failed (the NINA reports both as zero
  bytes, as in the Zephyr version). A UDP socket connected with `connect()` drops datagrams from
  other senders (as in the Zephyr version).
- `ioctl(FIONREAD)` is the NuttX version's own: one NINA poll there and then, in the calling thread;
  1 when data (for TCP also the peer's close) is there, 0 when not (the NINA does not give a byte
  count). With 1 the answer carries `RECVFROM_AVAIL`, so a following `recv(MSG_DONTWAIT)` reaches
  the NINA. It is for the time synchronization's receive polling every 1 ms (the Zephyr version's
  `time_sync.c` does the same with `recv(MSG_DONTWAIT)`).
- `nina/nina_sock.c` is checked by a host test (`nuttx/tests/nina_sock.c`) with a fake
  `nina_request()` and clock.

### Name resolution

`nina/nina_dns.c` is a line-by-line port of the Zephyr version's `drivers/nina/nina_dns.c`: it
resolves a name with the NINA's resolver commands (0x34 with the name, 0x35 to read the address) to
one IPv4 address. A numeric address is not sent to the NINA. The Zephyr version registers it as the
socket offload's `getaddrinfo()`, used by `zsock_getaddrinfo()`; the NuttX version has functions of
its own, `nina_getaddrinfo()` and `nina_freeaddrinfo()` (`include/nina_dns.h`), and the compatibility
layer's `<zephyr/net/socket.h>` maps `zsock_getaddrinfo()`, `zsock_freeaddrinfo()` and
`struct zsock_addrinfo` to them. NuttX's `getaddrinfo()` (`CONFIG_LIBC_NETDB`, which queries a DNS
server over UDP itself) is not used; `CONFIG_NANO_NINA_DNS` cannot be on together with
`CONFIG_LIBC_NETDB`. Errors are NuttX's `EAI_*` values under the Zephyr version's `DNS_EAI_*` names
(`EAI_NONAME` for a name that does not resolve, `EAI_AGAIN` for an SPI failure, `EAI_FAIL` for an
address of the wrong length, `EAI_BADFLAGS`/`EAI_FAMILY`/`EAI_SOCKTYPE` for bad hints, `EAI_SERVICE`
for a bad service, `EAI_MEMORY`). A mutex covers the two commands (the NINA keeps one result). The
result comes from `malloc()`, not the Zephyr version's 2 KB heap. The host test is
`nuttx/tests/nina_dns.c`. The only user now is the time synchronization when its NTP server is
given by name (the default is the gateway).

### Wi-Fi (network device `wlan0`)

| File | Role |
|---|---|
| `nina/nina_net.c` | A copy of the Zephyr version's `drivers/nina/nina_net.c` (only the leading comment and `<nuttx/config.h>` added): scan, connect, disconnect, status, and the three threads `nina_scan_tid`, `nina_connect_tid` and `nina_monitor_tid` |
| `nina/nina_netdev.c` | Registers the NINA as the NuttX network device `wlan0` and turns wapi's wireless ioctls into the Zephyr Wi-Fi requests (`net_mgmt()`). NuttX port only |
| `nina/wifi_profile_null.c` | Empty versions of the Wi-Fi profile functions `nina_net.c` calls (`wifi_profile_pause_reconnect()` and so on), used only in a build without the stored credentials and automatic reconnection (`nuttx/apps/wifi/wifi_profile.c`, the copy of the Zephyr version's `src/wifi_profile.c`; when `CONFIG_NANO_WIFI_PROFILE` is not set) |
| `include/nina_net.h`, `include/wifi_profile.h` | `nina_net_initialize()`. `wifi_profile.h` has the same declarations as the Zephyr version's `src/wifi_profile.h` |

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
  reconnection when it ends. A manual disconnect stops reconnection (`wifi_profile_pause_reconnect()`).
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
- wapi's ioctls (`d_ioctl`, handled in the calling thread):

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
- The stored credentials and automatic reconnection (`nuttx/apps/wifi/`: copies of the Zephyr
  version's `src/wifi_profile.c` and of Zephyr's `subsys/net/lib/wifi_credentials/`, `wifi cred`;
  "Stored Wi-Fi credentials and automatic reconnection" in `nuttx/README.md`) use the layer's
  `net_mgmt()` (`NET_REQUEST_WIFI_CONNECT_STORED`), `net_mgmt_add_event_callback()`
  (`NET_EVENT_WIFI_CONNECT_RESULT`), `net_if_get_wifi_sta()`, `net_if_ipv4_get_global_addr()` and
  `k_current_get()` as the Zephyr version does. `nina_net.c` calls
  `wifi_profile_connection_allowed()` for every connection request and refuses one from the
  reconnection thread with `-ECANCELED` after a disconnection asked for (as in the Zephyr version).

## I2C0

| File | Role |
|---|---|
| The board's `src/nano_i2c.c` | The I2C0 controller. A copy of the controller part of the Zephyr version's `drivers/i2c_dw/i2c_dw.c` (`i2c_dw_transfer()`, `i2c_dw_isr()`, `i2c_dw_data_ask()`, `_read()`, `_send()`, `i2c_dw_setup()`, `i2c_dw_configure()`, the initialization) and of the DMA reads (`i2c_dw_rp_dma_read()` and the rest) with the same functions, branches, order, register values, time limit and error values, offered as a NuttX `struct i2c_master_s` (`I2C_TRANSFER()`, `I2C_RESET()`) |
| The board's `src/nano_i2c.h` | `nano_i2c0_initialize()` (called by `nano_bringup()` at boot, which registers `/dev/i2c0`) |
| The board's `src/nano_i2c_dw.h` | The OS-independent part: the definitions of the Zephyr version's `i2c_dw.h`, the SCL counts, the speed class of a frequency, the NuttX flags (`I2C_M_*`) turned into Zephyr's (`I2C_MSG_*`) and the words written to `IC_DATA_CMD`. Host test `nuttx/tests/nano_i2c_dw.c` |
| The board's `src/nano_i2c_dw_registers.h` | A copy of the Zephyr version's `i2c_dw_registers.h` (only the accessors are defined for NuttX) |
| The board's `src/nano_dma.c`, `nano_dma.h` | The RP2040 DMA used by the DMA reads: Zephyr's `drivers/dma/dma_rpi_pico.c` (`dma_config()`, `dma_start()`, `dma_stop()`, the interrupt), the channel allocation of the DMA API (`dma_request_channel()`, `dma_release_channel()`) and the pico-sdk calls that `i2c_dw.c` makes itself (`dma_channel_is_busy()`, `dma_irqn_acknowledge_channel()`, `dma_hw->abort`), written on the registers |
| The board's `src/i2c_dw_rp2040_commands.[ch]` | Unchanged copies of the Zephyr version's files of that name (the command words sent by DMA). Host test `nuttx/tests/i2c_dma_commands.c` (also a copy) |
| `include/nano_i2c_dma.h` | The state that the `i2c_dma` command (`nuttx/apps/i2c_dma`) reads and sets: DMA on or off and the four counters |

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
| `lsm6dso/lsm6dso.c` | The driver. A copy of the Zephyr version's `drivers/lsm6dso/lsm6dso.c` with only the calls into the system replaced (the same functions, branches, order, register operations, conversions, error values and log lines). The devicetree instance macros at the end are one `DEVICE_DT_DEFINE()` with the board's values (INT1's `.fifo_gpio` is `<&gpio0 24 0>`) |
| `lsm6dso/lsm6dso.h` | A copy of the Zephyr version's file. Only the kernel object types (`k_mutex`, `k_spinlock`), the bus condition (`DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` → `LSM6DSO_BUS_I2C`), the stream condition (`CONFIG_SENSOR_ASYNC_API` → `CONFIG_NANO_LSM6DSO_STREAM`) and `<zephyr/types.h>` (→ `<stdint.h>` and others, and `<nuttx/config.h>`) are replaced |
| `lsm6dso/lsm6dso_os.[ch]` | The kernel services `lsm6dso.c` uses: mutexes and interrupts off (the compatibility layer's `k_mutex` and `k_spin_lock()`), `k_busy_wait()`, `k_msleep()`/`k_usleep()`, `k_cycle_get_32()` |
| `lsm6dso/stmemsc.h`, `stmemsc_i2c.c`, `stmemsc_mdelay.c` | A copy of the I2C part of Zephyr's `drivers/sensor/st/stmemsc/` (the glue between ST's `stmdev_ctx_t` and Zephyr's I2C API) |
| `lsm6dso/st_lsm6dso_reg.c` | A one-line wrapper that builds ST's `lsm6dso_reg.c` (`nuttx/stmemsc/`, not in Git) as it is |
| `lsm6dso/lsm6dso_fifo_hw.[ch]`, `lsm6dso_fifo_stats.h`, `lsm6dso_attributes.h` | Unchanged copies of the Zephyr version's files |
| `lsm6dso/lsm6dso_stream.c`, `lsm6dso_fifo_irq.[ch]` | The FIFO stream (below). Unchanged copies of the Zephyr version's files |
| `lsm6dso/lsm6dso_compat.c` | Single reads through RTIO (the fallback for a submission that is not a stream). A copy of the Zephyr version's file with only the log include and level name replaced |
| `lsm6dso/lsm6dso_fifo_decoder.[ch]` | The decoders of the stream's and the single reads' buffers (`get_decoder`). Unchanged copies |
| the board's `src/nano_lsm6dso.[ch]` | Makes I2C0 the compatibility layer's bus device (`i2c0`, 100 kHz) and initializes `lsm6dsox` (from `nano_bringup()`) |
| the board's `src/nano_gpio.c` | GPIO bank 0 for the compatibility layer's `gpio0` (input set-up, levels, edge and level interrupts); `<nano_gpio.h>` (`nuttx/drivers/include`) |

The replacements (the same table is at the top of `lsm6dso.c`):

| Zephyr | NuttX port |
|---|---|
| `<zephyr/logging/log.h>`, `CONFIG_SENSOR_LOG_LEVEL` (3) | `app_log.h` (the same lines in the RAM log), level 3 |
| `k_mutex_init/_lock/_unlock` | `lsm6dso_os_mutex_init/_lock/_unlock` (the layer's `k_mutex`, an `rmutex_t`; a `K_FOREVER` wait ended by a signal is repeated) |
| `k_spin_lock/_unlock` | `lsm6dso_os_spin_lock/_unlock` (the layer's `k_spin_lock()`) |
| `k_busy_wait()` | `up_udelay()` |
| `k_msleep()`, `k_usleep()` | `nxsched_msleep()`, `nxsched_usleep()` (rounded up to ticks plus one tick: Zephyr's rule, the same 100 µs tick) |
| `k_cycle_get_32()`, `k_cyc_to_ns_floor64()` | the TIMER's low 32 bits (1 MHz) and ×1000 |
| `sys_get_le32()` | the same arithmetic, written in the file |
| `CONFIG_SENSOR_ASYNC_API` | `CONFIG_NANO_LSM6DSO_STREAM` (always on with `NANO_LSM6DSO`; as in Zephyr, `lsm6dso.c` does not build without it) |
| the devicetree instance, `POST_KERNEL`, priority 90 | `DEVICE_DT_DEFINE(lsm6dsox, ...)`, `device_init()` in the board's bring-up |

- The board's `nano_bringup()` initializes the sensor right after I2C0 and before mounting the
  key-value store (NVS), the Zephyr version's order. A failure only leaves the device not ready
  (`device_is_ready()` is false, as with Zephyr's `do_device_init()`); the boot goes on.
- Not ported: SPI, the sensor hub (`lsm6dso_shub.c`), triggers (`lsm6dso_trigger.c`) and the die
  temperature. All are off in the Zephyr version's configuration (`CONFIG_LSM6DSO_SENSORHUB`,
  `_TRIGGER`, `_ENABLE_TEMP`) and not built there. Their `#ifdef` parts are kept in the copy.
- The FIFO read time (`transfer_time_ns` in the statistics) is measured with the TIMER (1 µs
  steps); the Zephyr version uses the CPU clock (SysTick, 8 ns steps).

### FIFO stream

`lsm6dso_stream.c` and `lsm6dso_fifo_irq.[ch]` are the Zephyr version's files as they are (`cmp`
identical); they use the kernel, RTIO, GPIO and sensor API under Zephyr's names from the
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

`nuttx/drivers/compat/` provides, under Zephyr's names and no more than needed, the part of
Zephyr's API that the copied driver and its callers (the acquisition service, the calibration, the
sensor command) use. The include path is `$(TOPDIR)/external/compat`. Zephyr's sensor subsystem,
RTIO and kernel as a whole are not ported.

| Header | Contents |
|---|---|
| `zephyr/device.h` | `struct device` (name, config, API, state, data, init function), `device_is_ready()`, `device_init()` (the meaning of Zephyr's `do_device_init()`), `DEVICE_DT_GET()`, `DEVICE_DT_DEFINE()`, `DEVICE_API()` |
| `zephyr/devicetree.h` | `DT_NODELABEL()`, `DT_BUS()`; the nodes are `gpio0`, `i2c0` and `lsm6dsox` only |
| `zephyr/kernel.h` | `k_timeout_t` (`K_NO_WAIT`, `K_FOREVER`, `K_MSEC()`, `K_SECONDS()`; ticks of 100 µs), `k_mutex`, `k_sem` (`K_SEM_DEFINE`), `k_poll()` on semaphores (`K_POLL_TYPE_SEM_AVAILABLE`, `K_POLL_MODE_NOTIFY_ONLY`), work (`k_work`, `k_work_delayable`, `k_work_q`, `k_work_submit()`, `k_work_cancel_sync()`, `k_work_schedule_for_queue()`, `k_work_reschedule_for_queue()`, `k_work_queue_start()`), `K_THREAD_STACK_DEFINE()`, `K_PRIO_PREEMPT()`, `k_thread_name_set()`, the system work queue `k_sys_work_q`, `K_MUTEX_DEFINE()`, `K_THREAD_DEFINE()` (below), `k_current_get()` (the calling thread's NuttX thread ID, comparable with the `k_tid_t` of `K_THREAD_DEFINE()`), `k_uptime_get()`, `k_sleep()` (below) |
| `zephyr/spinlock.h`, `zephyr/sys/atomic.h` | `k_spin_lock()` (below), `atomic_t` and `atomic_get/set/clear/cas/or/inc/add()` |
| `zephyr/rtio/rtio.h`, `zephyr/rtio/work.h` | RTIO (below) and the RTIO work queue (`rtio_work_req_alloc()`, `_submit()`) |
| `zephyr/drivers/gpio.h` | `struct gpio_dt_spec`, `gpio_is_ready_dt()`, `gpio_pin_configure_dt()` (inputs only), `gpio_pin_interrupt_configure_dt()`, `gpio_pin_get_dt()`, callbacks (`gpio_init_callback()`, `gpio_add_callback()`, `gpio_remove_callback()`) |
| `zephyr/drivers/sensor.h` | `struct sensor_value`, the channel, attribute and trigger enumerations (Zephyr's numbers), `struct sensor_driver_api`, `sensor_attr_set()`/`_get()`, `sensor_sample_fetch()`/`_chan()`, `sensor_channel_get()`, `sensor_trigger_set()`, the unit conversions. The async API: `struct sensor_chan_spec`, the decoders (`struct sensor_decoder_api`, `sensor_three_axis_data` and others), `SENSOR_DT_READ_IODEV()`, `SENSOR_DT_STREAM_IODEV()`, `sensor_get_decoder()`, `sensor_stream()`, `sensor_read()`, `sensor_read_async_mempool()`, `sensor_processing_with_callback()` |
| `zephyr/drivers/sensor_clock.h` | `sensor_clock_get_cycles()`, `sensor_clock_cycles_to_ns()` (below) |
| `zephyr/drivers/i2c.h` | `struct i2c_dt_spec`, `i2c_burst_read_dt()`, `i2c_write_dt()`, `i2c_write_read_dt()`, `i2c_configure()`, the speed classes (`I2C_SPEED_*`, `I2C_SPEED_SET()`) |
| `zephyr/sys/util.h`, `sys/__assert.h`, `sys/mpsc_lockfree.h`, `sys/mem_blocks.h`, `dsp/types.h` | `BIT()`, `ARRAY_SIZE()`, `MIN()`/`MAX()`, `IS_ENABLED()`, `CONTAINER_OF()`, `GENMASK()`/`FIELD_GET()`, `STRINGIFY()` and others, `__ASSERT()`, RTIO's queues and block pool, `q31_t` |
| `zephyr/sys/printk.h`, `zephyr/sys/byteorder.h`, `zephyr/sys/cpu_load.h` | `snprintk()` (`snprintf()`, as in the Zephyr version's configuration with `CONFIG_PICOLIBC=y`), `sys_put_be16/32()`/`sys_get_be16/32()`/`sys_get_le32()`, `cpu_load_get()` (the board's `cpu_load.h`) |
| `zephyr/drivers/hwinfo.h` | `hwinfo_get_device_id()` (below) |
| `zephyr/shell/shell.h` | `shell_print()` (stdout), `shell_warn()` and `shell_error()` (stderr), so that command functions in Zephyr's form can be called from a builtin command on `nuttx/apps/subcmd` (below) |
| `zephyr/net/socket.h` | `zsock_inet_pton()`, `zsock_inet_ntop()` (the C library's `inet_pton()`, `inet_ntop()`), `zsock_getaddrinfo()`, `zsock_freeaddrinfo()` (the NINA's resolver, `nina_getaddrinfo()`; errors are NuttX's `EAI_*`) |
| `zephyr/net/net_if.h` | `struct net_if` (one NuttX network device), `net_if_get_default()`, `net_if_get_wifi_sta()`, the IPv4 address, netmask and gateway (`net_if_ipv4_addr_add()`, `_rm()`, `net_if_ipv4_get_global_addr()`, `net_if_ipv4_set_netmask_by_addr()`, `net_if_ipv4_set_gw()`, `net_if_ipv4_get_gw()`), `net_if_dormant_on()`/`_off()`, `net_if_is_admin_up()`, `net_if_set_link_addr()` (below) |
| `zephyr/net/net_mgmt.h` | `net_mgmt()` (requests), `struct net_mgmt_event_callback`, `net_mgmt_init_event_callback()`, `net_mgmt_add_event_callback()`, `_del_event_callback()`, `net_mgmt_event_notify_with_info()` (below) |
| `zephyr/net/wifi_mgmt.h` | The Wi-Fi types (security, band, MFP, state, mode; Zephyr's values), `struct wifi_scan_params`, `wifi_scan_result`, `wifi_connect_req_params`, `wifi_iface_status`, `wifi_status`, `struct wifi_mgmt_ops`, the offloaded interface API and `NET_DEVICE_DT_OFFLOAD_DEFINE()`, the requests `NET_REQUEST_WIFI_SCAN`, `_CONNECT`, `_DISCONNECT`, `_IFACE_STATUS`, `_CONNECT_STORED` (with `CONFIG_NANO_WIFI_CREDENTIALS`), the events `NET_EVENT_WIFI_SCAN_RESULT`, `_SCAN_DONE`, `_CONNECT_RESULT`, `_DISCONNECT_RESULT`, `wifi_mgmt_raise_connect_result_event()`, `_disconnect_result_event()` (below), `wifi_security_txt()`, `wifi_band_txt()` |
| `zephyr/net/wifi_credentials.h` | A copy of Zephyr's header of that name: `struct wifi_credentials_header`, `_personal`, `WIFI_CREDENTIALS_FLAG_*`, `wifi_credentials_set_personal_struct()`, `_get_by_ssid_personal_struct()`, `_delete_by_ssid()`, `_for_each_ssid()`, `_is_empty()` and the rest. The configuration values (`CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES` and so on) come from `NANO_WIFI_CREDENTIALS_*` of `nuttx/apps/wifi/Kconfig`. The port's own `wifi_credentials_init()` (called by init; Zephyr's `SYS_INIT()`). The functions are in `nuttx/apps/wifi/` |
| `zephyr/net/wifi_utils.h` | `wifi_utils_validate_chan()` (`_2g`, `_5g`, `_6g`), `net_bytes_from_str()` (in Zephyr's `<zephyr/net/net_ip.h>`) |
| `zephyr/settings/settings.h` | `settings_subsys_init()`, `settings_save_one()`, `settings_delete()`, `settings_load_one()`: one name at a time, as the NuttX NVS (`/dev/config`) item of the same name (implemented in `nuttx/apps/wifi/settings_nvs.c`). No subtree loads and no handlers |
| `zephyr_compat.h` | `zephyr_compat_initialize()` (called by the board's bring-up, below), `K_THREAD_DEFINE_START()` (called by init) |

| Source | Contents |
|---|---|
| `zephyr_kernel.c` | The OS part on NuttX: `k_mutex` (`rmutex_t`), `k_sem` (`sem_t`, the limit checked in `k_sem_give()`), `k_poll()`, the work queue threads (kernel threads on static stacks), the timer of delayable work (a wdog), thread names, the start of `K_THREAD_DEFINE()` threads, `k_uptime_get()`, `k_sleep()`, `zephyr_compat_initialize()` |
| `zephyr_work.c` | Work items and queues: the used part of Zephyr's `kernel/work.c` (OS-independent) |
| `zephyr_rtio.c`, `zephyr_rtio_workq.c`, `zephyr_mem_blocks.c` | The RTIO executor (`subsys/rtio/rtio_executor.c`), the RTIO work queue (`rtio_workq.c`), the block pool (`lib/mem_blocks`) |
| `zephyr_gpio.c` | The `gpio0` device (Zephyr's `gpio.h` logic and `gpio_rpi_pico.c`) on the board's `<nano_gpio.h>` |
| `zephyr_sensor_clock.c` | The sensor clock on the board's TIMER (`nano_mono_us()`) |
| `sensor_default_rtio.c` | A copy of Zephyr's `drivers/sensor/default_rtio_sensor.c` (only the log replaced): the sensors' iodev API, the fallback for single reads, the default decoder |
| `zephyr_device.c`, `zephyr_i2c.c` | Device initialization and I2C |
| `zephyr_hwinfo.c` | `hwinfo_get_device_id()` on the board's chip ID (`nano_chip_id()`) |
| `zephyr_net_if.c`, `zephyr_net_mgmt.c`, `zephyr_wifi_mgmt.c` | The interface (values of the NuttX network device), the `net_mgmt` event queue and thread, the Wi-Fi requests (a copy of the checks of Zephyr's `subsys/net/l2/wifi/wifi_mgmt.c`) and events, `NET_REQUEST_WIFI_CONNECT_STORED` (a copy of `connect_stored_command()` of the same file; with `CONFIG_NANO_WIFI_CREDENTIALS`). Only with `CONFIG_NANO_NINA_NET` |
| `zephyr_wifi_utils.c` | Copies of `wifi_security_txt()` and `wifi_band_txt()` (Zephyr's `wifi_mgmt.c`), the channel checks (`wifi_utils.c`) and `net_bytes_from_str()` (`subsys/net/ip/utils.c`), for `wifi cred`. OS-independent. Only with `CONFIG_NANO_NINA_NET` |

- **Work**: the flags (RUNNING, CANCELING, QUEUED, DELAYED) and return values are those of
  `kernel/work.c`. `submit` does nothing if already queued (0) and queues a running item again on
  its queue (2). `schedule` does nothing while delayed or queued (a running item can be
  scheduled); `reschedule` stops the delay and schedules again (`K_NO_WAIT` queues at once).
  `k_work_cancel_sync()` removes a queued item and waits for a running one (one waiting thread
  only; Zephyr allows several). After an item, the queue's thread calls `sched_yield()` if more are
  pending (Zephyr's `k_yield()`). The delay timer is a wdog; a delay ends at the current tick +
  ticks + 1, as in Zephyr. Not ported: flush, drain, plug, stopping a queue, the work timeout,
  `k_work_cancel()`, `k_work_cancel_delayable()`.
- **Spin locks**: Zephyr's single-CPU `k_spin_lock()` masks interrupts, and no other thread runs
  until it is released (a thread made ready by `k_sem_give()` or the like runs at the release).
  NuttX switches to a thread that `nxsem_post()` makes ready at once, even with interrupts masked,
  so the layer's `k_spin_lock()` is `sched_lock()` plus `up_irq_save()` (`sched_lock()` does
  nothing in an interrupt handler). The `atomic_*()` functions use it too (the Cortex-M0+ has no
  exclusive loads and stores; Zephyr's ARMv6-M build masks interrupts as well).
- **Clashing NuttX names**: NuttX's `<nuttx/atomic.h>` (read by many kernel headers) has its own
  `atomic_t`, `atomic_set()` and others with other meanings. The layer's `<zephyr/sys/atomic.h>`
  reads it first, then takes the names over for the file (`atomic_t` is `zephyr_atomic_t`, NuttX's
  macros are undefined). A file that uses both includes the NuttX headers first. The board's
  bring-up does not include the Zephyr headers, only `zephyr_compat.h`.
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
  of Zephyr's `z_add_timeout()`). It always returns 0, as there is no `k_wakeup()`.
- **`hwinfo_get_device_id()`**: as the Zephyr version's RP2040 driver (`hwinfo_rpi_pico.c` without
  `CONFIG_HWINFO_RPI_PICO_CHIP_ID`), the external flash's 64-bit unique ID (command 0x4B) in the
  order the flash sends it (8 bytes at most). The board reads it once at boot (`nano_chipid.c`);
  this returns the copy, or `-EAGAIN` if the board has not read it yet (Zephyr's driver cannot
  fail).
- **Shell**: `shell_print()` writes to stdout, `shell_warn()` and `shell_error()` to stderr, each
  with a newline (where the other commands such as `kv` write; Zephyr prints warnings in yellow and
  errors in red). The `sh` argument is evaluated and not used. There are no
  registration macros (`SHELL_CMD_REGISTER()` and so on); command tables are written with
  `nuttx/apps/subcmd`.
- **Network**: `zsock_inet_pton()` and `zsock_inet_ntop()` are the C library's functions. While
  networking is off, `imu` selects `CONFIG_LIBC_IPv4_ADDRCONV` for the IPv4 conversions. Sockets are
  the NINA's usrsock (there is no `zsock_socket()` and so on; use NuttX's `socket()`).
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
