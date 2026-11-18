# Local drivers

English | [日本語](README_ja.md)

`zephyr/app/usb-console` builds with some Zephyr drivers replaced by the copies in this directory.
They carry the changes needed to read the IMU FIFO at a high rate, and a Wi-Fi driver for the
NINA-W102.

| Directory | Content | Origin |
|---|---|---|
| `lsm6dso/` | LSM6DSO sensor driver with FIFO streaming added | Copy from Zephyr v4.5.0 |
| `i2c_dw/` | DesignWare I2C driver with DMA reads on the RP2040 added | Copy from Zephyr v4.5.0 |
| `sensor_shell/` | Sensor shell with fixes and the private attributes | Copy from Zephyr v4.5.0 |
| `nina/` | The Zephyr side of the Wi-Fi and socket driver for the NINA-W102 over SPI (its core is in `common/drivers/nina/`) | Written here |

The copies come from Zephyr v4.5.0 (commit `41d10b59730406bbab4ca9f1fbbe9e96aafb4966`) and keep
each source's copyright and Apache-2.0 notices. The application's `CMakeLists.txt` swaps the
corresponding sources inside Zephyr's driver libraries for the copies. Kconfig, devicetree
bindings and the ST HAL are Zephyr's. If Zephyr's layout changes and a source to replace is
not found, the build fails.

The parts that are the same files as the NuttX and FreeRTOS builds' (the LSM6DSO driver itself, `lsm6dso.[ch]`, and its `lsm6dso_fifo.[ch]`,
`lsm6dso_fifo_packet.h`, `lsm6dso_cal.[ch]`, `lsm6dso_zero.[ch]`, `lsm6dso_fifo_decoder.[ch]`,
`lsm6dso_fifo_hw.[ch]`, `lsm6dso_fifo_irq.[ch]`, `lsm6dso_stream.c`, `lsm6dso_compat.c`,
`lsm6dso_fifo_stats.h` and `lsm6dso_attributes.h`, the sensor shell's `sensor_shell.c`, the NINA
driver's core, and the I2C driver's `i2c_dw_rp2040_commands.[ch]`) are kept in the repository's
`common/drivers/lsm6dso/`, `common/drivers/sensor_shell/`, `common/drivers/nina/` and
`common/drivers/i2c_dw/`. `CMakeLists.txt` adds them from there to the driver libraries (the NINA core
to the application), and the sensor shell reads `lsm6dso_attributes.h` from there.
`sensor_shell/sensor_shell_os.h`, which stays in this directory, is the Zephyr part that the shared
`sensor_shell.c` includes first (two macros that do nothing; the NuttX build's header of the same name
makes `sensor get` wait for its result to be printed). `lsm6dso/lsm6dso_instance.h` is the Zephyr part
that the shared `lsm6dso.c` includes at its end: Zephyr's devicetree instance macros of the driver,
unchanged (the NuttX build's header of the same name defines its one device with the board's
values). `lsm6dso/lsm6dso_shub.c` and `lsm6dso_trigger.c` (copies of Zephyr's, built only with
`CONFIG_LSM6DSO_SENSORHUB` and `CONFIG_LSM6DSO_TRIGGER`, which are off) stay here and read
`lsm6dso.h` from `common/drivers/lsm6dso/`. The NINA core (`nina_spi.c`, `nina_net.c`, `nina_sock.[ch]`,
`nina_dns.[ch]`, `nina_shell.c`, the declarations `nina_proto.h`, the boundary headers `nano_nina.h` and
`nina_os.h`, and others) is in `common/drivers/nina/`, shared by the three builds; `nina/` keeps only
this build's part (see the NINA section below). The file names in the tables below cover both places.

## LSM6DSO FIFO streaming

The FIFO can be read through Zephyr's Sensor Streaming API (`sensor_stream()` on RTIO) with the
trigger `SENSOR_TRIG_FIFO_WATERMARK` and the option `SENSOR_STREAM_DATA_INCLUDE`.

| File | Role |
|---|---|
| `lsm6dso_fifo.c` | Parsing FIFO words: accelerometer, gyroscope, timestamp, other; tag counter |
| `lsm6dso_fifo_hw.c` | FIFO setup, status, I2C burst read |
| `lsm6dso_fifo_irq.c` | INT1 (FIFO threshold) interrupt |
| `lsm6dso_stream.c` | Starting, reading and stopping the stream on its own work queue (priority 10) |
| `lsm6dso_fifo_decoder.c` | Zephyr decoder for FIFO buffers, used by `sensor stream` |
| `lsm6dso_compat.c` | Switching between this and the one-shot decoder |

### Behaviour

- At start the FIFO is put into bypass, the batch rates, timestamp interval and watermark are
  set, and it is switched to stream mode.
- On the threshold interrupt the FIFO status is read and all pending words are read in one
  burst and delivered as one RTIO buffer, headed by a header (word count, periods,
  sensitivities, conversion to physical units).
- The layout of the header (`lsm6dso_fifo_packet.h`) is private to the driver. Code that reads
  a buffer uses the decoder for physical values, or `lsm6dso_fifo_buffer_words()` for the raw
  words. `lsm6dso_fifo_next_entry()` interprets the words one at a time and yields the kind,
  xyz, the tag counter, and the counter value a timestamp word carries.
- On a **FIFO overrun**, an I2C error or a buffer that is too small, the FIFO is stopped and
  the stream ends with an error. The number of lost samples is unknown, so nothing that looks
  continuous is returned. Whether to restart is up to the caller (`common/imu/imu_stream.c`).
- **Stopping** is requested with `lsm6dso_fifo_request_stop()`. It pauses batching, delivers
  the words left in the FIFO and then posts a terminal completion. Canceling the RTIO request
  directly could free a buffer that was read but not yet consumed.
- While the FIFO is in use, one-shot reads and attribute changes return `-EBUSY`.

### Time

The FIFO contains the sensor's timestamps (nominally 25 us per tick) at an interval set by an
attribute. The timestamp counter is enabled at start and never disabled, so it keeps running
across stream restarts. Assigning ticks to samples and converting them to PC time is done by
the application (section 7 of the [design document](../../../../design/imu-raw-mqtt-design.md)).

`lsm6dso_timestamp_read()` reads the sensor's timestamp register between two readings of the
caller's clock, which relates the sensor clock to the board clock.

Sample times returned by the decoder are estimated from the host's read completion time and
the nominal data rate. They include I2C transfer and interrupt latency and are not accurate.

### Attributes

Private attributes on `SENSOR_CHAN_ALL` (`lsm6dso_attributes.h`). Both apply at the next
stream start and cannot be changed while a stream runs.

| Attribute | Name in the shell | Values | Initial |
|---|---|---|---|
| `SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK` | `fifo_watermark` | 1 to 256 words | 32 |
| `SENSOR_ATTR_LSM6DSO_FIFO_TIMESTAMP_DECIMATION` | `fifo_timestamp_decimation` | 0, 1, 8, 32 | 8 |

A larger watermark means fewer reads and less load, but data arrives later and less of the
512-word FIFO is left as margin.

### Conversion to physical units and calibration

The conversion from raw values to physical units lives in one function in `lsm6dso_cal.c`,
used by both the FIFO decoder (`sensor stream`) and the one-shot read (`sensor get`).

```text
out = M · (raw − bias)
```

- It is integer arithmetic throughout. `M` is a 3×3 matrix combining scale and axis
  misalignment; the output is Q31 with shift 9, in units of 2⁻²² m/s² or rad/s.
- Without a calibration the same function runs with the nominal sensitivity on the diagonal
  and a zero bias.
- A calibration comes from the function registered with `lsm6dso_cal_set_lookup()`. The driver
  calls it with the current sensitivity at registration, on `lsm6dso_cal_reload()`, and whenever
  a range is set. If it has nothing, the nominal conversion is used. Reading the KV store is
  left to the application (`common/imu/imu_cal.c`).
- Each FIFO buffer carries the conversion in effect when it was read. The decoder needs only
  the buffer, and a range or calibration change in between does not affect buffers already read.
- `lsm6dso_cal_status()` reports whether the accelerometer and the gyro use a calibration, and
  the gyro bias in use.

The gyro bias is taken again from data recorded at rest (`lsm6dso_zero.c`).

- Samples read from the FIFO are grouped into segments of an eighth of a second. When eight
  consecutive segments agree in mean angular rate (within 0.3 dps) and mean acceleration
  (within 2 mg), the sensor counts as resting and the mean rate of that window is blended in:
  a plain average over the first 20 windows, then exponential smoothing with 1/20. Single
  samples are not judged, because at 6667 Hz one sample has too much noise (about 1.7 dps).
- It runs whenever a FIFO stream runs (`imu start`, `sensor stream`). The driver never starts
  the sensor on its own.
- `lsm6dso_cal_zero()` takes one reading on request. If the sensor is off, it runs at 104 Hz
  for up to three seconds and is switched off again. While a stream owns the FIFO it returns
  `-EBUSY`.
- The value is kept in RAM and dropped whenever a conversion is selected again (range set,
  reload). A window more than 1 dps from the stored bias (5 dps from zero without a
  calibration) is not used.

### Statistics

`lsm6dso_fifo_get_stats()` returns the words read, the number and duration of transfers, I2C
errors, undersized buffers, overrun observations, the configured data rates, sensitivities
and the frequency trim. The statistics are cleared and `generation` advances each time the
FIFO is started.

### Limitations

- I2C only.
- The internal temperature is not read and not stored in the FIFO.
- Cannot be combined with `CONFIG_LSM6DSO_TRIGGER` or `CONFIG_LSM6DSO_SENSORHUB`.

## I2C (`i2c_dw`)

Changes for the RP2040; other controllers behave as in Zephyr.

- RX and TX FIFO depth 16 and an RX threshold of 4 bytes, lowered for short reads and at the
  end of a read. The TX_EMPTY interrupt is masked while no more requests can be queued.
- **DMA reads** (`CONFIG_I2C_DW_RP2040_DMA`, on by default) for I2C0 with a 7-bit address when
  the final read ending in STOP is 32 to 3584 bytes long. Writes, short reads and anything else
  use interrupts. If a DMA channel cannot be claimed or configured, the transfer falls back to
  interrupts before it starts.

```text
i2c_dma mode          show the current mode
i2c_dma mode on|off   switch, only while no stream or transfer is active
i2c_dma stats         DMA transfers, bytes, errors and fallbacks (cumulative)
```

In an earlier measurement of FIFO acquisition alone (6667 Hz, 1 MHz, single runs) with a
watermark of 128, `cpu_active` was 45.7% without DMA and 22.5% with it.

## Sensor shell (`sensor_shell`)

- Channels with no frames and completions without a buffer, such as after cancellation, are
  left out of the output.
- `fifo_watermark` and `fifo_timestamp_decimation` work with `sensor attr_set` and `attr_get`.

`sensor stream` and `imu start` cannot be used at the same time.

## NINA-W102 (`nina`)

A driver that lets Zephyr use the NINA-W102 (an ESP32) as Wi-Fi and sockets over SPI. The
module is expected to run Arduino NINA firmware 3.0.2 with two fixes applied (see
[firmware/nina-fw](../../../../firmware/nina-fw/README.md)).

### Division of work

Wi-Fi and TCP/IP are handled by the NINA firmware (lwIP). The RP2040 only issues SPI commands
to manage the Wi-Fi connection and sockets; it never sees packets. To Zephyr this is a Wi-Fi
management API plus socket offload.

The driver itself is the core shared with the NuttX and FreeRTOS builds (`common/drivers/nina/`);
`nina/` holds only the OS side of the core's boundaries and the glue to Zephyr's socket layer.

| File | Place | Role |
|---|---|---|
| `nina_spi.c` | `common/drivers/nina/` | Framing of SPI commands and replies, the module's reset, the waits for READY |
| `nina_net.c` | `common/drivers/nina/` | Network interface; Wi-Fi connect, disconnect, scan, status; applying the DHCP result (address, gateway); link monitoring |
| `nina_sock.[ch]` | `common/drivers/nina/` | The socket table and commands (one pass of each wait loop at a time) |
| `nina_dns.[ch]` | `common/drivers/nina/` | Name resolution (the module's resolver) |
| `nina_shell.c` | `common/drivers/nina/` | The `nina version` command |
| `nina_wait.[ch]` | `common/drivers/nina/` | The waits between the passes of the socket loops (an interval, or the IRQ line's edge) |
| `nano_nina.c` | `nina/` | Zephyr's side of the core's `nano_nina.h`: the devicetree's `spi1` and the four lines (CS, READY, RESET, IRQ), Zephyr's GPIO and SPI calls |
| `nina_os.c` | `nina/` | Zephyr's side of the core's `nina_os.h`: mutexes, sleeps, clock, the READY interrupt, the waits for the IRQ line's (GPIO2) edges |
| `nina_socket.c` | `nina/` | Socket offload (file descriptors, errno, `O_NONBLOCK`, `SO_RCVTIMEO`, the wait loops and their sleeps, poll), and the resolver registered as the `getaddrinfo` offload |
| `posix/` | `nina/` | Headers that map the POSIX socket names the core uses (`struct sockaddr_in`, `AF_INET`, `POLLIN`, ...) to Zephyr's (read by the core and `nina_socket.c` only) |

Zephyr's Telnet shell, MQTT client and zperf run on these sockets.

### SPI commands

- Commands are handled one at a time. The RP2040 serializes them with a mutex. SPI runs at 8 MHz
  (`CONFIG_NINA_SPI_FREQUENCY`).
- Changes of the module's READY line (GPIO10) are waited for on a GPIO edge interrupt
  (`nina/nina_os.c`). The interrupt is enabled before READY is read, so a change after the read is
  not missed; it is off during the transfers. Each wait lasts at most 10 ms, and READY is read
  again after it. Past the deadline (2 s before a command, 10 s for the reply (2 s for
  `nina_ip_config()`), 20 ms after CS is asserted) the result is `-ETIMEDOUT`.
- One TCP send carries at most 3584 bytes (`CONFIG_NINA_TCP_MAX_IO`), the most that fits the
  module's command buffer. A RAW batch of up to 2448 bytes goes out in one command.

### Sockets

Every socket uses the BSD-style socket commands of firmware 3.0.2 (0x70 to 0x7f: socket,
close, bind, listen, accept, connect, send, recv, sendto, recvfrom, ioctl, poll, getsockopt).
Each command is a single call of an lwIP socket function inside the module, and sockets are
independent of each other. The older per-type client commands are not used.

- **Ten sockets**, the limit of the module's lwIP (`CONFIG_LWIP_MAX_SOCKETS=10`), shared by
  the Telnet listener, MQTT, NTP and zperf. Zephyr's file descriptor table is sized with
  `CONFIG_ZVFS_OPEN_MAX=16`. By default its size follows `CONFIG_NET_MAX_CONTEXTS`, so
  lowering only that value leaves too few descriptors and accepting a Telnet client then
  fails over and over.
- Zephyr's own TCP/IP stack carries no traffic (`net conn` stays empty). Its packet buffers
  and connection tables are kept at a minimum in `prj.conf`.
- **Always non-blocking.** Commands run inside the module's SPI handler, so a blocking call
  stalls everything including Wi-Fi management. A socket is made non-blocking as soon as it is
  created, and the driver waits for connect completion, readable data and send space by
  repeating the poll command every 20 ms (with a firmware that reports socket events, on the
  edges of the IRQ line instead: see "Firmware additions" below).
- **A reply cannot tell a failure from zero bytes.** Receiving polls first (readable with zero
  bytes means the peer closed). When a TCP send returns zero, a poll command 1 ms later checks
  whether the socket is writable, and the data goes again only once it is (the full data is not
  sent over SPI at every try). The poll's error flag tells a broken connection. Connect results
  are read from SO_ERROR.
- **TCP connect** waits up to 10 s. A send fails when no buffer space appears for 10 s; this
  is why a vanished access point takes about 9 s to be recognized as a disconnection.
- **UDP** carries up to 512 bytes per datagram. What does not fit the receive buffer is
  dropped by lwIP, which keeps datagram boundaries. A connected UDP socket has datagrams from
  other senders discarded by the driver.
- `poll` does not fail as a whole when asking the module about one socket fails: that socket gets
  `POLLERR`. A socket that another thread closes while the poll runs gets `POLLNVAL`.
- `bind` accepts the any address (0.0.0.0) only, IPv4 only. Multicast is not
  supported.

### Wi-Fi

- Open and WPA2-PSK networks on 2.4 GHz. Open networks have not been tried on hardware.
- Power saving is switched off on every connection and reconnection, because with it sending
  can stall for hundreds of milliseconds. Confirming that the setting took effect needs the
  patched firmware.
- Credentials are stored with Zephyr's `wifi cred` (Settings on NVS). At boot and after an
  unexpected disconnection the driver keeps trying the stored settings. It does not reconnect
  after `wifi disconnect`. With several stored networks, which one is used is not guaranteed.
- Firmware 3.0.2 stops Wi-Fi while scanning, so the connection drops briefly (the driver treats
  it as a disconnection and connects again after the scan). A firmware that keeps the link does
  not drop it (next section). At most 10 results.

### Firmware additions (patches 0003 to 0006)

A firmware with the patches 0003 to 0006 of `firmware/nina-fw/` answers command 0x6A with the
bits of what it adds (the specification: "Host interface of 0003 to 0006" in
`firmware/nina-fw/README.md`). The driver asks once, when the interface is initialized (after
reading the MAC address), and switches its behaviour bit by bit. A firmware without the command
(3.0.2 and the like) answers `0xEF 0x00 0xEE`; after those three bytes the driver takes it as
"no additions" and works as before. The second line of `nina version` shows the module's bits
and those in use.

| Bit | Addition | What the driver does |
|---|---|---|
| 0 | A scan while connected keeps the link | A scan while connected is not treated as a disconnection (the address stays, no disconnection event, no reconnection). A loss of the link during the scan is found by the link monitor (every 2 s) as any other |
| 1 | Asynchronous scan | 0x36 starts it, 0x6B is asked every 100 ms, 0x27 reads the results when it is done. The NINA lock is held for one command at a time, so sending goes on during the scan. A scan still running after 10 s fails (`-ETIMEDOUT`) without 0x27. Could not start or failed: `-EIO`; stopped by a connection or disconnection: `-ECANCELED` |
| 2 | Socket events on the IRQ line | The socket waits (connect, receive, `poll`, the send retries) wake on GPIO2's rising edge interrupt instead of every 20 ms (1 ms for sends), asking again after 100 ms at most |

How bit 2 is used (`common/drivers/nina/nina_wait.h`):

- As the firmware's rule says: the edge count is noted before a command that may arm an event
  (poll, recv, accept, send, connect); after a "not yet" answer the driver waits until the count
  changes or 100 ms pass, and asks again. An edge from before the command costs one more ask; a
  lost edge costs at most 100 ms (the asking keeps it correct).
- One line serves every socket, so each edge wakes every waiting thread. A thread whose wake-ups
  keep finding nothing rests a little before it waits again (1, 2, 4, 8, 16 ms, then 20 ms).
- A send waits 1 ms after a pass that sent data and got zero bytes, as before, and for the edge
  after a pass that only polled (the socket was not writable). No wait goes past the call's
  deadline (`SO_RCVTIMEO` for receiving, the `poll` timeout, 10 s for connect and send).
- A `poll` that includes a descriptor other than a NINA socket looks again at least every 20 ms
  (for that descriptor, as before).
- Removing the `NINA_OS_EVENTS` line of `CMakeLists.txt` keeps the intervals whatever the
  firmware.

### Verified on hardware

Single runs each.

- NTP (UDP), Telnet (TCP listener) and MQTT (TCP client).
- 360 UDP datagrams echoed without loss or corruption while RAW streaming at 6667 Hz (MQTT)
  and Telnet were active, with no batch missing on the MQTT side.
- Automatic reconnection after the access point was stopped and restarted, with the address
  and gateway obtained again.

- zperf with iperf3 on the PC, 30 s each: 2.93 Mbit/s for TCP sending, 1.54 Mbit/s for TCP
  receiving, and no loss for UDP sending at the requested 500 kbit/s.

Not verified: several TCP clients at once, ten sockets at once, and the timeout path when a
server does not answer.

Two socket options (`setsockopt`) are accepted.

- The receive timeout (`SO_RCVTIMEO`) applies to the driver's own receive wait. It is not
  passed to the module.
- `TCP_NODELAY` is accepted but **has no effect**. The module's command cannot set TCP-level
  options, so Nagle's algorithm stays on. The iperf3 client asks for it on its control
  connection and gives up if it is refused, which is the only reason it is accepted.

Any other option returns `ENOPROTOOPT`.

### TLS

TLS is not supported. These points are unresolved.

- **Random source**: the build uses Zephyr's non-cryptographic test random generator, and the
  RP2040 has no cryptographic random source. The ATECC608A on the board has its configuration
  and data unlocked and returned the same answer to two Random commands, so it cannot be used
  as it is. Getting the ESP32's hardware random numbers over SPI needs a new firmware command.
- **Certificate verification**: TLS on the module cannot verify the host name for connections
  made by IP address, and certificate date checking is not enabled there.
- **Time**: checking certificate validity needs trustworthy time, and NTP replies are not
  authenticated.

Running Zephyr's TLS on a NINA TCP socket with the test random generator and a temporary
certificate was tried earlier: MQTT connected and sent, and a wrong host name, an expired
certificate and an untrusted CA were each refused. That was a functional check with an unsafe
random source, not usable TLS. An implementation must not fall back silently to a test random
generator when no proper source is available.

### Looking at the radio link

[Tracing TCP ACKs with a Wi-Fi monitor](../../../../design/wifi-monitor-debug.md) describes how to
examine sending delays and ACK behaviour at the level of radio frames.

## The bootrom's `memcpy` and `memset`

With `CONFIG_NANO_ROM_MEM_OPS` (the application's Kconfig, on by default), `memcpy`, `memset`,
`__aeabi_memcpy*` and `__aeabi_memset*` are wrapped (the linker's `--wrap`) to the RP2040 bootrom's
routines, split as the Pico SDK's `pico_mem_ops` does (`common/drivers/rp2040_rom/`, shared with the
NuttX build). Without it they are picolibc's byte loops. The routines are looked up on first use and
kept in `.bss`, so `arch_early_memset` and `arch_early_memcpy`, which clear `.bss` and copy `.data`,
use picolibc's (`src/rom_mem_early.c`). The host test is `rp2040_rom_mem` in `common/tests/` (a
fake function table).

## Tests

`zephyr/script/host_tests.sh` tests these parts of the drivers on the PC with fake Zephyr headers.
All but `nina_boundary_*` and `nina_socket*` are tests shared with the NuttX and FreeRTOS builds
(`common/tests/`, run by `common/tests/run.sh`).

| Test | Covers |
|---|---|
| `fifo_words` | FIFO word parsing |
| `fifo_hw` | FIFO setup sequence, rollback on failure, status |
| `fifo_decoder` | Decoder for FIFO buffers |
| `stream_stop` | Stopping, canceling and error termination of the stream |
| `i2c_dma_commands` | Argument handling of `i2c_dma` |
| `nina_spi`, `nina_spi_wait_ready` | NINA SPI framing, reply parsing (the firmware's error reply 0xEF included), READY deadlines, the capability probe (0x6A) (the shared core `nina_spi.c`, fake boundaries) |
| `nina_wait`, `nina_wait_no_events` | The socket waits (`nina_wait.c`): the intervals, the IRQ line's edges, the 100 ms timeout, the deadline, the back-off (fake clock and line) |
| `nina_sock`, `nina_dns`, `nina_dns_offload` | The NINA socket commands, name resolution (`nina_dns_offload` in this build's form, `NINA_OS_DNS_OFFLOAD`) |
| `nina_boundary_events`, `nina_boundary_irq`, `nina_boundary_poll` | The shared `nina_spi.c` on this build's `nina/nano_nina.c` and `nina_os.c`: framing, lines and flags, the reset sequence, the READY interrupt (callback, semaphore reset, edge direction, off before transfers, 10 ms bound) and sampling; `nina_boundary_events` the waits for the IRQ line's edges (GPIO2's callback and rising edge, the count, a wait ended by an edge, the timeout, an edge just before the wait) |
| `nina_socket`, `nina_socket_events` | The socket offload `nina/nina_socket.c` on the shared core: descriptors, the wait loops and sleeps, options, accept, sendmsg, datagrams, poll, the resolver's registration; `nina_socket_events` the waits for the IRQ line's edges (an edge during a pass, the timeout, the deadline, the send's waits, a `poll` with another kind of descriptor) |
