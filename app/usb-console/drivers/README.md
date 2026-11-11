# Local drivers

English | [日本語](README_ja.md)

`app/usb-console` builds with some Zephyr drivers replaced by the copies in this directory.
They carry the changes needed to read the IMU FIFO at a high rate, and a Wi-Fi driver for the
NINA-W102.

| Directory | Content | Origin |
|---|---|---|
| `lsm6dso/` | LSM6DSO sensor driver with FIFO streaming added | Copy from Zephyr v4.5.0 |
| `i2c_dw/` | DesignWare I2C driver with DMA reads on the RP2040 added | Copy from Zephyr v4.5.0 |
| `sensor_shell/` | Sensor shell with fixes and the private attributes | Copy from Zephyr v4.5.0 |
| `nina/` | Wi-Fi and socket driver for the NINA-W102 over SPI | Written here |

The copies come from Zephyr v4.5.0 (commit `41d10b59730406bbab4ca9f1fbbe9e96aafb4966`) and keep
each source's copyright and Apache-2.0 notices. The application's `CMakeLists.txt` swaps the
corresponding sources inside Zephyr's driver libraries for the copies. Kconfig, devicetree
bindings and the ST HAL are Zephyr's. If Zephyr's layout changes and a source to replace is
not found, the build fails.

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
  burst and delivered as one RTIO buffer, headed by `struct lsm6dso_fifo_packet` (word count,
  periods, sensitivities).
- On a **FIFO overrun**, an I2C error or a buffer that is too small, the FIFO is stopped and
  the stream ends with an error. The number of lost samples is unknown, so nothing that looks
  continuous is returned. Whether to restart is up to the caller (`src/imu_stream.c`).
- **Stopping** is requested with `lsm6dso_fifo_request_stop()`. It pauses batching, delivers
  the words left in the FIFO and then posts a terminal completion. Canceling the RTIO request
  directly could free a buffer that was read but not yet consumed.
- While the FIFO is in use, one-shot reads and attribute changes return `-EBUSY`.

### Time

The FIFO contains the sensor's timestamps (nominally 25 us per tick) at an interval set by an
attribute. The timestamp counter is enabled at start and never disabled, so it keeps running
across stream restarts. Assigning ticks to samples and converting them to PC time is done by
the application (section 7 of the [design document](../../../design/imu-raw-mqtt-design.md)).

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
[firmware/nina-fw](../../../firmware/nina-fw/README.md)).

### Division of work

Wi-Fi and TCP/IP are handled by the NINA firmware (lwIP). The RP2040 only issues SPI commands
to manage the Wi-Fi connection and sockets; it never sees packets. To Zephyr this is a Wi-Fi
management API plus socket offload.

| File | Role |
|---|---|
| `nina_wifi.c` | Framing of SPI commands and replies; Wi-Fi connect, disconnect, scan, status |
| `nina_net.c` | Network interface, applying the DHCP result (address, gateway), link monitoring |
| `nina_socket.c` | Socket offload: TCP clients and listeners, UDP, poll |
| `nina_dns.c` | `getaddrinfo` offload (name resolution) |

Zephyr's Telnet shell, MQTT client and zperf run on these sockets.

### SPI commands

- Commands are handled one at a time. The RP2040 serializes them with a mutex and samples the
  module's ready line every 100 us while waiting for a reply. SPI runs at 8 MHz
  (`CONFIG_NINA_SPI_FREQUENCY`).
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
  repeating the poll command every 20 ms.
- **A reply cannot tell a failure from zero bytes.** Receiving polls first (readable with zero
  bytes means the peer closed). When a TCP send returns zero, the poll error flag is checked
  every 100 ms to detect a broken connection. Connect results are read from SO_ERROR.
- **TCP connect** waits up to 10 s. A send fails when no buffer space appears for 10 s; this
  is why a vanished access point takes about 9 s to be recognized as a disconnection.
- **UDP** carries up to 512 bytes per datagram. What does not fit the receive buffer is
  dropped by lwIP, which keeps datagram boundaries. A connected UDP socket has datagrams from
  other senders discarded by the driver.
- `bind` accepts the any address (0.0.0.0) only, IPv4 only. Multicast and socket options are
  not supported.

### Wi-Fi

- Open and WPA2-PSK networks on 2.4 GHz. Open networks have not been tried on hardware.
- Power saving is switched off on every connection and reconnection, because with it sending
  can stall for hundreds of milliseconds. Confirming that the setting took effect needs the
  patched firmware.
- Credentials are stored with Zephyr's `wifi cred` (Settings on NVS). At boot and after an
  unexpected disconnection the driver keeps trying the stored settings. It does not reconnect
  after `wifi disconnect`. With several stored networks, which one is used is not guaranteed.
- The module stops Wi-Fi while scanning, so the connection drops briefly. At most 10 results.

### Verified on hardware

Single runs each.

- NTP (UDP), Telnet (TCP listener) and MQTT (TCP client).
- 360 UDP datagrams echoed without loss or corruption while RAW streaming at 6667 Hz (MQTT)
  and Telnet were active, with no batch missing on the MQTT side.
- Automatic reconnection after the access point was stopped and restarted, with the address
  and gateway obtained again.

Not verified: several TCP clients at once, ten sockets at once, the timeout path when a server
does not answer, and zperf throughput since the move to the BSD socket commands.

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

[Tracing TCP ACKs with a Wi-Fi monitor](../../../design/wifi-monitor-debug.md) describes how to
examine sending delays and ACK behaviour at the level of radio frames.

## Tests

`./script/host_tests.sh` tests these parts of the drivers on the PC with fake Zephyr headers.

| Test | Covers |
|---|---|
| `fifo_words` | FIFO word parsing |
| `fifo_hw` | FIFO setup sequence, rollback on failure, status |
| `fifo_decoder` | Decoder for FIFO buffers |
| `stream_stop` | Stopping, canceling and error termination of the stream |
| `i2c_dma_commands` | Argument handling of `i2c_dma` |
| `nina_spi` | NINA SPI framing and reply parsing |
