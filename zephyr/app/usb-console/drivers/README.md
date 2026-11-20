# Local drivers

English | [日本語](README_ja.md)

`zephyr/app/usb-console` builds with some Zephyr drivers replaced by the copies in this directory.
They carry the changes needed to read the IMU FIFO at a high rate, and a Wi-Fi driver for the
NINA-W102.

| Directory | Content | Origin |
|---|---|---|
| `lsm6dso/` | LSM6DSO sensor driver with FIFO streaming added | Copy from Zephyr v4.5.0 |
| `sensor_shell/` | Sensor shell with fixes and the private attributes | Copy from Zephyr v4.5.0 |
| `nina/` | The esptool passthrough to the NINA-W102, which is this board's (the Wi-Fi and socket driver's core is in `common/drivers/nina/`, its Zephyr binding in `common/drivers/nina/zephyr/`) | Written here |

The copies come from Zephyr v4.5.0 (commit `41d10b59730406bbab4ca9f1fbbe9e96aafb4966`) and keep
each source's copyright and Apache-2.0 notices. The application's `CMakeLists.txt` swaps the
corresponding sources inside Zephyr's driver libraries for the copies. Kconfig, devicetree
bindings and the ST HAL are Zephyr's. If Zephyr's layout changes and a source to replace is
not found, the build fails. The DesignWare I2C driver has no copy: it is Zephyr's with a local patch
(see I2C and Local patches of Zephyr below).

The parts that are the same files as the NuttX and FreeRTOS builds' (the LSM6DSO driver itself, `lsm6dso.[ch]`, and its `lsm6dso_fifo.[ch]`,
`lsm6dso_fifo_packet.h`, `lsm6dso_cal.[ch]`, `lsm6dso_zero.[ch]`, `lsm6dso_fifo_decoder.[ch]`,
`lsm6dso_fifo_hw.[ch]`, `lsm6dso_fifo_irq.[ch]`, `lsm6dso_stream.c`, `lsm6dso_compat.c`,
`lsm6dso_fifo_stats.h` and `lsm6dso_attributes.h`, the sensor shell's `sensor_shell.c`, the NINA
driver's core, and the `i2c_dma` command's `i2c_dma_shell.c`) are kept in the repository's
`common/drivers/lsm6dso/`, `common/drivers/sensor_shell/`, `common/drivers/nina/` and
`common/shell/`. `CMakeLists.txt` adds them from there to the driver libraries (the NINA core
and `i2c_dma_shell.c` to the application), and the sensor shell reads `lsm6dso_attributes.h` from there.
`sensor_shell/sensor_shell_os.h`, which stays in this directory, is the Zephyr part that the shared
`sensor_shell.c` includes first (two macros that do nothing; the NuttX build's header of the same name
makes `sensor get` wait for its result to be printed). `lsm6dso/lsm6dso_instance.h` is the Zephyr part
that the shared `lsm6dso.c` includes at its end: Zephyr's devicetree instance macros of the driver,
unchanged (the NuttX build's header of the same name defines its one device with the board's
values). `lsm6dso/lsm6dso_shub.c` and `lsm6dso_trigger.c` (copies of Zephyr's, built only with
`CONFIG_LSM6DSO_SENSORHUB` and `CONFIG_LSM6DSO_TRIGGER`, which are off) stay here and read
`lsm6dso.h` from `common/drivers/lsm6dso/`. The NINA core (`nina_spi.c`, `nina_wifi.[ch]`, `nina_sock.[ch]`,
`nina_dns.[ch]`, the declarations `nina_proto.h`, the protocol's command and reply bytes
`nina_cmd.h`, the boundary headers `nina_bus.h` and `nina_os.h`, and others) is in
`common/drivers/nina/` and the `nina` command `nina_shell.c` in `common/shell/`, shared by the three
builds; the core's binding to Zephyr's APIs is in `common/drivers/nina/zephyr/` (the FreeRTOS build
also builds its Wi-Fi and socket parts, `nina_net.c` and `nina_socket.c`, and has its own `nina_os.c` in
`common/drivers/nina/freertos/`),
and `nina/` keeps only this board's part (see the NINA section below).
The file names in the tables below cover all those places.

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

Zephyr's DesignWare I2C driver (`drivers/i2c/i2c_dw.c`) with the local patch `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch`
("Local patches of Zephyr" below). The patch holds everything the driver needs, in Zephyr's tree: the application adds no
source or include path to the driver's library. Other controllers behave as in Zephyr.

**Changes for the RP2040's controllers**: RX and TX FIFO depth 16 and an RX threshold of 4 bytes, lowered for short reads
and at the end of a read. The TX_EMPTY interrupt is masked while no more requests can be queued.

**DMA reads** (`CONFIG_I2C_DW_RP2040_DMA`; off by default in the patch, on in `prj.conf`):

- On I2C0 only, with a 7-bit address, when the final read ending in STOP is 32 to 3584 bytes long. Writes, short reads
  and anything else use interrupts.
- A read uses two channels of Zephyr's DMA driver (`dma_rpi_pico.c`): one writes the read commands to `IC_DATA_CMD`,
  the other takes the bytes from it. If a DMA channel cannot be claimed or configured, the transfer falls back to
  interrupts before it starts.
- The DMA code, with the function that builds the read commands, `i2c_dw_rp2040_read_commands()`, is
  `drivers/i2c/i2c_dw_rp2040_dma_priv.h` (included once by `i2c_dw.c`). The command words and buffer (10,752 bytes
  together), semaphore, events and channels are static variables of the driver, for I2C0 alone. The per-instance data
  only holds whether a DMA read is in progress and whether the controller is quarantined.

**The `i2c_dma` command** is `common/shell/i2c_dma_shell.c`, shared by the three builds, built into the application. It
calls the three functions (get and set the mode, get the counters) that the patch declares in
`include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`.

```text
i2c_dma mode          show the current mode
i2c_dma mode on|off   switch, only while no stream or transfer is active
i2c_dma stats         DMA transfers, bytes, errors and fallbacks (cumulative)
```

**The other builds**: the I2C core of the NuttX and FreeRTOS builds (`common/drivers/i2c_dw/`) is a port of this patched
driver. The host test `i2c_dw_sync` detects where the two drift apart (how, in section 12 of the
[design](../../../../design/imu-raw-mqtt-design.md); the steps when moving to a newer Zephyr, in "Local patches of
Zephyr" below).

In an earlier measurement of FIFO acquisition alone (6667 Hz, 1 MHz, single runs) with a
watermark of 128, `cpu_active` was 45.7% without DMA and 22.5% with it.

## Sensor shell (`sensor_shell`)

- Channels with no frames and completions without a buffer, such as after cancellation, are
  left out of the output.
- `fifo_watermark` and `fifo_timestamp_decimation` work with `sensor attr_set` and `attr_get`.

`sensor stream` and `imu start` cannot be used at the same time.

## NINA-W102 (`nina`)

A driver that lets Zephyr use the NINA-W102 (an ESP32) as Wi-Fi and sockets over SPI. The
module is expected to run Arduino NINA firmware 3.0.2 with patches 0001 to 0006 (see
[firmware/nina-fw](../../../../firmware/nina-fw/README.md)); without 0003 to 0006 it works without
those additions ("Firmware additions" below).

### Division of work

Wi-Fi and TCP/IP are handled by the NINA firmware (lwIP). The RP2040 only issues SPI commands
to manage the Wi-Fi connection and sockets; it never sees packets. To Zephyr this is a Wi-Fi
management API plus socket offload.

The driver itself is the core shared with the NuttX and FreeRTOS builds (`common/drivers/nina/`),
with no OS of its own; its binding to Zephyr's APIs is `common/drivers/nina/zephyr/` (the FreeRTOS
build compiles its Wi-Fi and socket parts, `nina_net.c` and `nina_socket.c`, on its compatibility layer,
which has Zephyr's socket layer down to the list of socket implementations; its kernel services are its own
binding, `common/drivers/nina/freertos/nina_os.c`, on FreeRTOS and the Pico SDK), and `nina/` holds what
is this board's, the esptool passthrough. The
application's `CMakeLists.txt` builds them from there; the binding's devicetree binding is found
through `DTS_ROOT` and its options through the application's `Kconfig` (`rsource`).

| File | Place | Role |
|---|---|---|
| `nina_spi.c` | `common/drivers/nina/` | Framing of SPI commands and replies, the module's reset, the waits for READY |
| `nina_wifi.[ch]` | `common/drivers/nina/` | The Wi-Fi operations: connect (and its wait for the connection and the DHCP result), disconnect, scan, status, the link monitor; the bodies of the threads `nina_scan_tid`, `nina_connect_tid` and `nina_monitor_tid`; what happens goes to the binding's callbacks |
| `nina_sock.[ch]` | `common/drivers/nina/` | The socket table and commands (one pass of each wait loop at a time) |
| `nina_dns.[ch]` | `common/drivers/nina/` | Name resolution (the module's resolver) |
| `nina_cmd.h` | `common/drivers/nina/` | The names of the protocol's bytes: framing, the commands the driver sends, the values of their replies |
| `nina_shell.c` | `common/shell/` | The `nina version` command (in this build `nina passthrough` too; "The esptool passthrough" below) |
| `nina_passthrough_logic.h` | `common/drivers/nina/` | The decisions of the esptool passthrough (the restart's request, DTR and RTS on the module's lines, the rate, the way out, the receive buffer), shared with the FreeRTOS build |
| `nina_wait.[ch]` | `common/drivers/nina/` | The waits between the passes of the socket loops (an interval, or the IRQ line's edge) |
| `nina_net.c` | `common/drivers/nina/zephyr/` | Zephyr's Wi-Fi management and network interface on the core: the device (`NET_DEVICE_DT_OFFLOAD_DEFINE()`), Zephyr's parameters checked and passed on, the results as Zephyr's structures and events, the address, netmask, gateway and dormant state on the interface, the Wi-Fi profile's reconnection as the core's hooks, the three threads (`K_THREAD_DEFINE()`) |
| `nina_os.c` | `common/drivers/nina/zephyr/` | Zephyr's side of the core's `nina_os.h`: mutexes, the Wi-Fi threads' requests, sleeps, clock, the READY interrupt, the waits for the IRQ line's (GPIO2) edges |
| `nina_socket.c` | `common/drivers/nina/zephyr/` | Socket offload (file descriptors, errno, `O_NONBLOCK`, `SO_RCVTIMEO`, the wait loops and their sleeps, poll), and the resolver registered as the `getaddrinfo` offload (the FreeRTOS build's sockets too) |
| `nina_passthrough.[ch]`, `nina_passthrough_hw.h` | `nina/` | The esptool passthrough (`nina passthrough confirm`); `_hw.h` is its part on the RP2040's registers (watchdog, UART1, GPIO) |
| `posix/` | `common/drivers/nina/zephyr/` | Headers that map the POSIX socket names the core uses (`struct sockaddr_in`, `AF_INET`, `POLLIN`, ...) to Zephyr's (read by the core and `nina_socket.c` only) |
| `Kconfig`, `dts/bindings/wifi/arduino,nina-spi.yaml` | `common/drivers/nina/zephyr/` | The driver's options (`CONFIG_WIFI_NINA_LOCAL`, `CONFIG_NINA_SPI_FREQUENCY`, `CONFIG_NINA_TCP_MAX_IO`, `CONFIG_NINA_SEND_RETRY_MS`) and the devicetree binding of the `nina_w102` node |
| `nina_bus.c` | `common/drivers/nina/zephyr/` | Zephyr's side of the core's `nina_bus.h`: the devicetree's `spi1` and the four lines (CS, READY, RESET, IRQ), Zephyr's GPIO and SPI calls; no bus while the application's `nina_passthrough_active()` is true (the esptool passthrough) |

Zephyr's Telnet shell, MQTT client and zperf run on these sockets.

### SPI commands

- Commands are handled one at a time. The RP2040 serializes them with a mutex. SPI runs at 8 MHz
  (`CONFIG_NINA_SPI_FREQUENCY`).
- Changes of the module's READY line (GPIO10) are waited for on a GPIO edge interrupt
  (`common/drivers/nina/zephyr/nina_os.c`). The interrupt is enabled before READY is read, so a change after the read is
  not missed; it is off during the transfers. Each wait lasts at most 10 ms, and READY is read
  again after it. Past the deadline (2 s before a command, 10 s for the reply (2 s for
  `nina_ip_config()`), 20 ms after CS is asserted) the result is `-ETIMEDOUT`. Whether the
  interrupt is used is decided at run time: after the first reset `nina_os_ready_start()` registers
  READY's callback, and if that fails READY is read every 100 µs instead (same deadlines and
  results).
- One TCP send carries at most 3584 bytes (`CONFIG_NINA_TCP_MAX_IO`), the most that fits the
  module's command buffer. A RAW batch of up to 2448 bytes goes out in one command.

### Sockets

Every socket uses the BSD-style socket commands of firmware 3.0.2 (0x70 to 0x7f: socket,
close, bind, listen, accept, connect, send, recv, sendto, recvfrom, ioctl, poll, getsockopt).
Each command is a single call of an lwIP socket function inside the module, and sockets are
independent of each other.

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
"no additions" and works without them. The second line of `nina version` shows the module's bits
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
- A send waits 1 ms after a pass that sent data and got zero bytes, as without bit 2, and for the edge
  after a pass that only polled (the socket was not writable). No wait goes past the call's
  deadline (`SO_RCVTIMEO` for receiving, the `poll` timeout, 10 s for connect and send).
- A `poll` that includes a descriptor other than a NINA socket looks again at least every 20 ms
  (for that descriptor, as without bit 2).
- When the edge interrupt cannot be used (`nina_os_event_start()` fails to register the IRQ
  line's callback or to enable its edge), the waits keep the intervals whatever the firmware, and
  bit 2 is dropped from the bits in use.

### The esptool passthrough (`nina passthrough confirm`)

`nina passthrough confirm` restarts the board into a mode that relays the USB serial port (CDC ACM)
to the NINA's UART (the ESP32's UART0, which its ROM bootloader uses), so that esptool on the PC reads
and writes the NINA's flash, instead of Arduino's SerialNINAPassthrough sketch (the RP2040's firmware
stays). It behaves as the FreeRTOS build's; the decisions are the shared
`common/drivers/nina/nina_passthrough_logic.h`. `CONFIG_NINA_PASSTHROUGH` (on by default).

- Wiring: GPIO8 = UART1 TX -> the ESP32's UART0 RX, GPIO9 = UART1 RX. They are the SPI link's MISO and
  CS, so the NINA's SPI cannot run meanwhile. GPIO2 = the ESP32's GPIO0 (boot mode), GPIO3 = RESETN.
  RTS set releases the reset, DTR set pulls GPIO0 low (the sketch's mbed branch). Opening the port
  starts the ROM's download mode, closing it holds the module in reset. When the reset is released
  with DTR clear (the NINA's own firmware starts), GPIO0 is released after 750 ms and GPIO8 is not
  driven. UART1's rate follows the host's line coding (8N1).
- The way in: the command leaves a request in the watchdog's scratch registers 0..3 and restarts with
  the watchdog. The start reads and clears it at once (`POST_KERNEL` 0), so RESET and any later
  restart are normal starts. Nothing is written to the flash (the key-value store).
- Zephyr starts as usual in this mode, but the NINA core sees no SPI bus (`nina_bus_ready()` is
  false): the Wi-Fi interface's initialization, the Telnet server's socket, `nina version` and the
  like fail with `-ENODEV` without touching SPI1 or the NINA's lines (the Telnet errors stay in the RAM
  log). `main()` runs the relay without settings, calibration, time synchronization or a Wi-Fi
  connection. The CDC ACM interrupt callback is replaced by the relay's (`POST_KERNEL` 99), so the
  shell gets no input and its output does not reach the port. The log goes to RAM only.
- No byte lost: module to PC, UART1's receive interrupt (priority 0, at 4 bytes in the FIFO) moves the
  bytes into a 16 KiB buffer, which holds 178 ms at 921600 bps while the PC does not read; what does
  not fit is counted. PC to module, one packet (64 bytes) at a time; what the UART's FIFO cannot take
  is written by its transmit interrupt, and the CDC ACM receive side stops meanwhile (the host waits),
  so nothing is lost. DTR, RTS and the rate are taken from the USB stack's messages (the system work
  queue) and checked every millisecond. The data move in the CDC ACM callback (on its own work
  queue, below).
- The way out: close the port at 300 bps (for example `stty -F /dev/ttyACM0 300`), or RESET. The NINA
  is started again into its own firmware (GPIO0 high, RESETN low for 10 ms, GPIO0 released after
  750 ms), then the watchdog restarts the board.
- The RAM log (`dmesg` afterwards) gets a line at the start, each time the port closes and at the way
  out (bytes each way, bytes dropped while USB was not configured, bytes lost to a full buffer and
  the most waiting, the UART's overruns, breaks and framing errors, the rate).
- esptool 5.3.0 does not connect with `--before no-reset`. Use `custom_reset_sequence = R0|D1|W0.2|R1|W0.6`
  in `ESPTOOL_CFGFILE` and `--before default-reset --after no-reset`, which keep RTS set (the procedure
  is in "Reading and writing the NINA firmware" in the root README).
- On the board (2026-11-18): `read-flash 0 0x200000` at 115200 bps (192 s) and at 921600 bps (26.5 s) gave
  the same data, with no bytes lost and no overruns. Closing the port at 300 bps went back to the normal
  start. A passthrough start leaves two `<err>` lines of the Telnet listener in `dmesg` (the NINA is
  not started).

### The CDC ACM work queue

`CONFIG_USBD_CDC_ACM_WORKQUEUE=y` in `prj.conf` moves the CDC ACM class (the USB shell's transmit
completions) from the system work queue to its own. Zephyr 4.5.0's zperf iperf3 server prints to the
shell while it holds `srv_lock` on `zperf tcp download stop`, and its `idle_work_handler()` on the
system work queue waits for `srv_lock`, so the shell (`shell_uart`) and the system work queue waited
for each other (seen on the board: both pending at priority -1). With the setting, 20 runs of
`zperf tcp download stop` during a connection left the USB and Telnet shells working (2026-11-18).

### Verified on hardware

The next four items were checked with the images from before the NINA driver moved to the shared core
(single runs each).

- NTP (UDP), Telnet (TCP listener) and MQTT (TCP client).
- 360 UDP datagrams echoed without loss or corruption while RAW streaming at 6667 Hz (MQTT)
  and Telnet were active, with no batch missing on the MQTT side.
- Automatic reconnection after the access point was stopped and restarted, with the address
  and gateway obtained again.

- zperf with iperf3 on the PC, 30 s each: 2.93 Mbit/s for TCP sending, 1.54 Mbit/s for TCP
  receiving, and no loss for UDP sending at the requested 500 kbit/s.

With the shared core and the new firmware (0001 to 0006, 2026-11-18): sending right after flashing,
Telnet and `wifi scan` while sending, a manual disconnection and the reconnection, and the USB and
Telnet shells after repeated `zperf tcp download stop` during a connection.

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

## Local patches of Zephyr

Some changes to Zephyr and its modules are local patches for Zephyr's `west patch`. They are listed in
`zephyr/manifest/zephyr/patches.yml`; the files are in `zephyr/manifest/zephyr/patches/<module>/`.

| Patch | Module | Change |
|---|---|---|
| `hal_rpi_pico/0001-boot2-at25sf128a-clkdiv-2.patch` | hal_rpi_pico | The board's boot2 (`boot2_at25sf128a.S`) runs the flash at clk_sys / 2 (62.5 MHz) instead of clk_sys / 4 (31.25 MHz). hal_rpi_pico has no way to pass the value, so the default changes |
| `zephyr/0001-arm-cortex-m-rp2040-divider-save.patch` | zephyr | `CONFIG_ARM_RP2040_DIVIDER_SAVE`: the thread switches (PendSV, `swap_helper.S`) save and restore the SIO divider |
| `zephyr/0002-soc-rp2040-rom-mem-ops.patch` | zephyr | `CONFIG_RP2040_ROM_MEM_OPS`: `memcpy` and `memset` through the boot ROM's routines |
| `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch` | zephyr | The DesignWare I2C driver (`drivers/i2c/i2c_dw.c`): the RP2040's FIFO depth and RX threshold, and I2C0's DMA reads with `CONFIG_I2C_DW_RP2040_DMA` (I2C). The constants are in a new file `drivers/i2c/i2c_dw_rp2040.h`, the DMA code with the read commands in `i2c_dw_rp2040_dma_priv.h`, and the declarations of the `i2c_dma` command's functions in `include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`; 13 places go into the upstream files (`i2c_dw.c` 11, `i2c_dw.h` 1, `Kconfig.dw` 1) |

- **Applying**: `source script/setup.sh` applies them to the modules' working trees with `west patch apply`
  (no commits; `git -C zephyr/zephyr status` shows them as changes). Patches already applied are skipped,
  so sourcing it again changes nothing. A copy of the set applied is kept in `zephyr/.west/nano-patches/`;
  when `patches.yml` or a patch changes, that set is taken back first and the current one applied (a patch
  removed from the list does not stay in a tree). When applying fails, the patches applied are taken back.
- **After changing a patch**: set its `sha256sum` in `patches.yml` to the value of `sha256sum <file>`, then
  source `script/setup.sh` again.
- **Taking them back**: run `west patch reverse` in `zephyr/` and remove `zephyr/.west/nano-patches/`. If
  that fails, restore each tree with `git -C <tree> checkout -- .` as `setup.sh` says, remove the copy and
  source it again. The application does not build on trees without the patches (`prj.conf` sets the
  patches' `CONFIG_RP2040_ROM_MEM_OPS` and `CONFIG_I2C_DW_RP2040_DMA`, and `CONFIG_NANO_HW_DIVIDER` stops in CMake without
  `CONFIG_ARM_RP2040_DIVIDER_SAVE`).
- **Moving to a newer Zephyr**: take the patches back first as above, change the versions in
  `manifest/west.yml` and run `west update`. Remake each patch on the new trees, put it in `patches/`,
  update its `sha256sum`, apply them with `source script/setup.sh` and build. After remaking 0003, run
  `script/host_tests.sh`: bring the upstream changes that `i2c_dw_sync` shows into the shared core
  (`common/drivers/i2c_dw/`), or add the reason not to to the list at the top of `i2c_dw.c`, then
  rewrite the expected differences with `common/tests/i2c_dw_sync.py ... --update` ([Tests](#tests)).

### boot2 at clock divider 2

The read boot2 uses (EBh) is rated for 108 MHz or more by the AT25SF128A's data sheet. With clk_sys at
125 MHz the divider is even, so 62.5 MHz is the most (the same divider as the FreeRTOS build's Pico SDK
boot2). On the board, `devmem 0x18000014` (the SSI's BAUDR) reads 2.

### The divider (`CONFIG_NANO_HW_DIVIDER`)

The application's `CONFIG_NANO_HW_DIVIDER` (on by default) builds the Pico SDK's `divider_hardware.S` from
hal_rpi_pico and wraps `__aeabi_idiv`, `__aeabi_idivmod`, `__aeabi_uidiv`, `__aeabi_uidivmod`,
`__aeabi_ldivmod` and `__aeabi_uldivmod` to it (the linker's `--wrap`, `CMakeLists.txt`). Without it they
are libgcc's shift-and-subtract loops.

- The routines do not lock interrupts (`PICO_DIVIDER_DISABLE_INTERRUPTS=0`). When an interrupt handler
  divides in the middle of a division, the routine sees DIRTY and saves the divider itself. When a thread
  is switched out in the middle of one, the patch's `CONFIG_ARM_RP2040_DIVIDER_SAVE` (selected by
  `CONFIG_NANO_HW_DIVIDER`) saves it in PendSV: only when DIRTY is set, the four registers go to 20 bytes
  of `struct _thread_arch`, and a thread that was not dividing comes back with DIRTY clear (the Pico SDK's
  FreeRTOS port of the FreeRTOS build saves the four words at every switch).
- A division by zero gives the Pico SDK routines' results (the same in the three builds): for 32 bits a
  saturated quotient (0 for 0 / 0) and a remainder of 0; for 64 bits the same as libgcc's.
- Test images (2026-11-18): without the saving, 30 mismatches in 81 seconds (in the lowest-priority
  threads only); with it, none in 361 seconds and about 40.7 million divisions.

### The boot ROM's `memcpy` and `memset` (`CONFIG_RP2040_ROM_MEM_OPS`)

With the patch's `CONFIG_RP2040_ROM_MEM_OPS` (set in `prj.conf`), the SoC's CMake wraps `memcpy`,
`memset`, `__aeabi_memcpy*` and `__aeabi_memset*` (the linker's `--wrap`) to the RP2040 boot ROM's
routines, split as the Pico SDK's `pico_mem_ops` does. This covers the calls from inside picolibc and
those GCC emits for struct copies and loops. Without it they are picolibc's byte loops. The routines are
looked up in the ROM's function table on first use and kept in `.bss`, so `arch_early_memset` and
`arch_early_memcpy`, which clear `.bss` and copy `.data`, call picolibc's. The NuttX build has the same
routines as a patch of NuttX (`nuttx/patches/nuttx/0010`).

## Tests

`zephyr/script/host_tests.sh` tests these parts of the drivers on the PC with fake Zephyr headers.
All but `nina_boundary_*`, `passthrough`, `nina_socket*` and `nina_net` are tests shared with the NuttX and FreeRTOS builds
(`common/tests/`, run by `common/tests/run.sh`).

| Test | Covers |
|---|---|
| `fifo_words` | FIFO word parsing |
| `fifo_hw` | FIFO setup sequence, rollback on failure, status |
| `fifo_decoder` | Decoder for FIFO buffers |
| `stream_stop` | Stopping, canceling and error termination of the stream |
| `i2c_dw_sync` | The shared I2C core (`common/drivers/i2c_dw/`) against the patched driver (made from the Zephyr tree's files at HEAD and patch 0003): the definitions of both headers (state, interrupts, registers, the IC_DATA_CMD bits, FIFO depth 16 and batch 4, the DMA limit 3584), the SCL counts at 125 MHz, the DMA path's lower limit 32, thresholds 8/0, interrupt mask, IC_CON bit 9, stop waits of 100 ms and timeout of 1000 ms, the spike lengths, and the pairs of functions (Zephyr's names, its DMA API's calls among them, turned into the core's, those of the DMA boundary `i2c_dw_dma_os.h`; compared statement by statement; the differences must be those of `common/tests/i2c_dw_sync.expected`); the read commands' function `i2c_dw_rp2040_read_commands()`, static in `i2c_dw_rp2040_dma_priv.h` of the patch and of the core, statement by statement, with its command bits; `zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`, which the patch adds, against the core's `common/drivers/i2c_dw/i2c_dw_rp2040_dma.h`, the same once the comments and include guards are taken out; and that the patched driver's includes are all in the tree. With the build, the values of its `.config` and `zephyr.dts` too. Only when the Zephyr tree is there |
| `nina_spi` | NINA SPI framing, reply parsing (the firmware's error reply 0xEF included), READY deadlines, the capability probe (0x6A) (the shared core `nina_spi.c`, fake boundaries); each both with the waits on READY's interrupt and, when they cannot start, with READY read every 100 µs |
| `nina_wait` | The socket waits (`nina_wait.c`): the intervals (before the events start, and after they could not), the IRQ line's edges, the 100 ms timeout, the deadline, the back-off (fake clock and line) |
| `nina_core_independent` | Each file of the NINA core (the top of `common/drivers/nina/`) compiles with the C library, the POSIX socket types and its own headers only |
| `nina_wifi` | The Wi-Fi operations (`nina_wifi.c`) on fake boundaries and binding: connect with its checks, its thread (the checks every 500 ms, power mode, the address, 60 checks, a disconnection during the attempt), disconnect, the monitor (lost after two checks), the scans of 3.0.2 and of the patched firmware with their ends, status |
| `nina_sock`, `nina_dns`, `nina_dns_offload` | The NINA socket commands, name resolution (`nina_dns_offload` in this build's form, `NINA_OS_DNS_OFFLOAD`) |
| `nina_boundary` | The shared `nina_spi.c` on `nina_bus.c` and `nina_os.c` of `common/drivers/nina/zephyr/` (also the Wi-Fi threads' lock and requests): framing, lines and flags, the reset sequence, the READY interrupt (callback, semaphore reset, edge direction, off before transfers, 10 ms bound) and the sampling when the callback cannot be registered; the waits for the IRQ line's edges (a start that fails, GPIO2's callback and rising edge, the count, a wait ended by an edge, the timeout, an edge just before the wait) |
| `nina_boundary_passthrough` | The same with `CONFIG_NINA_PASSTHROUGH`: no bus while the passthrough runs (`-ENODEV` without touching a line or SPI), as usual once it is gone |
| `passthrough` | The passthrough `nina/nina_passthrough.c` on fake Zephyr calls and fake hardware (`nina_passthrough_hw.h`): the request at the start, the lines and the rate from the USB messages and the tick, both data paths and the handovers between the interrupts, breaks and overruns, the bytes dropped without USB, the way out, the command. The decisions themselves: the FreeRTOS build's test `freertos/tests/passthrough` |
| `nina_socket` | The socket offload `common/drivers/nina/zephyr/nina_socket.c` on the shared core: descriptors, the wait loops and sleeps (after the events could not start), options, accept, sendmsg, datagrams, poll, the resolver's registration; the waits for the IRQ line's edges once the events are started (an edge during a pass, the timeout, the deadline, the send's waits, a `poll` with another kind of descriptor) |
| `nina_net` | The Zephyr binding `common/drivers/nina/zephyr/nina_net.c` on fake Zephyr declarations and a fake core: the device, the interface's init, the scan's parameters and results, connect, disconnect and status, the core's callbacks on the interface and as Zephyr's events, the profile's hooks, the threads' stacks and priorities |
