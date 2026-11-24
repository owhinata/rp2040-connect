# Local drivers

English | [日本語](README_ja.md)

The firmware of this repository is built on four OSes, Zephyr, NuttX, the Pico SDK with FreeRTOS and the Pico SDK
with ThreadX: the Zephyr, NuttX, FreeRTOS and ThreadX builds. This document describes the Zephyr build's drivers.

`zephyr/app/usb-console` uses some of Zephyr's drivers with local patches of Zephyr's tree, which
carry the changes needed to read the IMU FIFO at a high rate, and two drivers shared with the NuttX,
FreeRTOS and ThreadX builds, for the LSM6DSOX and the NINA-W102.

| Driver | Content | Origin |
|---|---|---|
| LSM6DSO (`lsm6dso`) | FIFO streaming, conversion with a calibration, gyro bias at rest | `common/drivers/lsm6dso/` (in place of Zephyr's `drivers/sensor/st/lsm6dso`) |
| Sensor shell (`drivers/sensor/sensor_shell.c`) | Fixes and the LSM6DSO's FIFO attributes | Zephyr v4.5.0 with `zephyr/0006-sensor-shell-fifo-attributes.patch` |
| DesignWare I2C (`drivers/i2c/i2c_dw.c`) | The RP2040's FIFO depth, I2C0's DMA reads | Zephyr v4.5.0 with `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch` (and `0004` for `i2c scan`) |
| NINA-W102 (`nina`) | Wi-Fi and sockets over SPI | `common/drivers/nina/` |
| PL022 SPI (`drivers/spi/spi_pl022.c`) | The DMA mode's threshold, for the NINA's SPI1 | Zephyr v4.5.0 with `zephyr/0007-spi-pl022-dma-min-len.patch` |

Zephyr v4.5.0 is commit `41d10b59730406bbab4ca9f1fbbe9e96aafb4966`. Each patch holds everything its driver
needs, in Zephyr's tree (see "Local patches of Zephyr" below): the application's `CMakeLists.txt` adds no
source or include path to the drivers' libraries. This directory holds only these documents.

The LSM6DSO driver is in `common/drivers/lsm6dso/`, shared by the four builds. Its core, with no OS, is
at its top: `lsm6dso.c` (how the chip is operated: the start and reset, rates and ranges, the single
reads and their conversion, the FIFO's register work, its counters and owner, the timestamp read, the
calibration in effect and the bias tracking), `lsm6dso_stream.c` (the FIFO stream's state machine,
below), `lsm6dso_cal.c` (the conversion's arithmetic and the tracker) and `lsm6dso_fifo.c` (the FIFO
words' parser); their headers are `include/lsm6dso_cal.h` and `include/lsm6dso_fifo.h` (for the
driver's users) and `internal/lsm6dso_core.h`, `internal/lsm6dso_stream.h` and
`internal/lsm6dso_fifo_packet.h` (the layout of the stream's buffers). The core reaches the chip through
ST's `lsm6dso_reg.c` (hal_st) and the rest through two boundaries in `port/`: `lsm6dso_os.h`, the OS (two
locks, waits, a cycle counter, the FIFO's long reads, the log; for the stream a work item on a thread of
the driver's own, the INT1 line and the clock of the batches' time stamps), and `lsm6dso_stream_req.h`,
the stream's user (the request it serves, a buffer for each batch, the batch or the error handed back).
Its Zephyr binding is `common/drivers/lsm6dso/port/zephyr/`: the device and the sensor API
(`lsm6dso_sensor.c`, `lsm6dso_sensor.h`), the devicetree's devices (`lsm6dso_instance.h`), the stream's
RTIO side (below), the OS part of the core's boundary (`lsm6dso_os_impl.h` with `lsm6dso_os.c`:
`k_mutex`, `k_spinlock`, `k_busy_wait()`, `k_msleep()`, `k_usleep()`, `k_cycle_get_32()`,
`i2c_burst_read_dt()`, a delayable work item on the work queue `imu_fifo_worker`, INT1's GPIO interrupt
through the system work queue, the sensor clock, Zephyr's logging) and the user part
(`lsm6dso_stream_req_impl.h`: RTIO's multishot request). The NuttX, FreeRTOS and ThreadX builds compile the
same binding on their Zephyr compatibility layers, with their own `lsm6dso_instance.h` (`port/nuttx/`;
`port/pico_sdk/` for the FreeRTOS and ThreadX builds on the Pico SDK). The binding is a Zephyr module of its own (`port/zephyr/zephyr/module.yml`, which the
application's `CMakeLists.txt` adds to `ZEPHYR_EXTRA_MODULES`): its `Kconfig` has the driver's option
`CONFIG_LSM6DSO_LOCAL` (on by default for an `st,lsm6dso` or `st,lsm6dso32` node; it selects
`HAS_STMEMSC` and `USE_STDC_LSM6DSO` for ST's register driver and Zephyr's stmemsc glue), `prj.conf`
turns Zephyr's driver of the same devices off (`CONFIG_LSM6DSO=n`), and its `CMakeLists.txt` builds the
library `lsm6dso_local` and exports the extended API `<zephyr/drivers/sensor/lsm6dso.h>`
(`port/zephyr/include/zephyr/drivers/sensor/lsm6dso.h`, which reads `include/lsm6dso.h`). The devicetree
binding is Zephyr's `st,lsm6dso`. The application reads only the extended API: the FIFO words and their
parser (`lsm6dso_fifo_next()`, `lsm6dso_fifo_next_entry()`, `lsm6dso_fifo_buffer_words()`), the
calibration's types and calls (`lsm6dso_cal_set_lookup()`, `_reload()`, `_status()`, `_zero()`), the
FIFO's attributes, settings, counters and stop, and the timestamp read.

`sensor_shell.c` with patch 0006 is the same file as the other builds' (`common/shell/`;
the host test `zephyr_sensor_shell_patched` checks it); `drivers/sensor/sensor_shell_os.h`, which it
includes first, is this build's (two macros that do nothing; in the other builds `sensor get` waits for
its result to be printed).

The NINA core is in `common/drivers/nina/` (at its top, with the headers in `include/`, what the driver offers, `nina_wifi.[ch]`,
`nina_sock.[ch]`, `nina_dns.[ch]` and `nina_passthrough.[ch]`; in `internal/` the protocol
`nina_proto.[ch]` and the waits `nina_wait.[ch]`; in `port/` the boundary headers `nina_bus.h`,
`nina_os.h` and the passthrough's two) and the `nina` command `nina_shell.c` in `common/shell/`. The core is shared by the four
builds, and `nina_shell.c` is used by the Zephyr, FreeRTOS and ThreadX builds (the NuttX build has `nuttx/apps/nina`,
with the same syntax and output); the core's binding to Zephyr's APIs is in `common/drivers/nina/port/zephyr/` (the FreeRTOS
and ThreadX builds also build its Wi-Fi and socket parts, `nina_net.c` and `nina_socket.c`, and each has its own `nina_os.c` in
`common/drivers/nina/port/freertos/` and `port/threadx/`). Nothing of the NINA driver is in this directory (see the NINA
section below).
The file names in the tables below cover all those places.

## LSM6DSO FIFO streaming

The FIFO can be read through Zephyr's Sensor Streaming API (`sensor_stream()` on RTIO) with the
trigger `SENSOR_TRIG_FIFO_WATERMARK` and the option `SENSOR_STREAM_DATA_INCLUDE`.

| File | Role |
|---|---|
| `lsm6dso.c` (core) | The FIFO's owner and lock: its register setup, pause and stop, the status and burst reads (static), INT1's routing of the threshold, its counters, the timestamp decimation, the timestamp read; the single reads, their conversion, the bias reading of `lsm6dso_cal_zero()` |
| `lsm6dso_stream.c` (core) | The stream's state machine: the start at its first run (the FIFO, INT1 enabled and routed), a batch read at the watermark on each INT1 edge and at the poll every 20 ms, the graceful stop, the end on an error, an overrun or the user's cancellation; the watermark |
| `lsm6dso_fifo.c` (core) | The FIFO words' parser (accelerometer, gyroscope, timestamp, other; tag counter) |
| `lsm6dso_cal.c` (core) | Conversion to physical units and the gyro bias at rest (below), used by `lsm6dso.c` and the decoder |
| `lsm6dso_sensor.c` (binding) | Upstream's driver on the core: the device, attributes, `sample_fetch` and the calibrated `channel_get`, the calls of the extended API that take a device |
| `lsm6dso_rtio_stream.c` (binding) | Each device's stream: its INT1 line from the devicetree, RTIO's request handed to the core's stream (another trigger `-ENOTSUP`, a second stream `-EBUSY`), the extended API's stop, watermark and timestamp decimation |
| `lsm6dso_os.c` (binding) | The stream's work queue (`imu_fifo_worker`, priority 10, 2048 bytes), INT1's GPIO interrupt, which wakes the stream through the system work queue |
| `lsm6dso_rtio.c` (binding) | The driver's RTIO submit: a stream goes to `lsm6dso_rtio_stream.c`, a single read (`sensor get`) to a copy of the sensor API's fallback on the RTIO work queue |
| `lsm6dso_decoder.c` (binding) | `lsm6dso_fifo_buffer_words()`; the decoder for FIFO buffers, used by `sensor stream`, and the decoder that hands a single read's buffer to Zephyr's default decoder |

The binding's file names are upstream's and those of the sibling driver with FIFO streaming,
`lsm6dsv16x`. The INT1 interrupt is the driver's own (in `lsm6dso_os.c`): the line is enabled
only while a stream runs and the system work queue only wakes the stream's work queue, which does all the
I2C reads. Upstream's trigger (`CONFIG_LSM6DSO_TRIGGER`, `lsm6dso_trigger.c`), sensor hub
(`CONFIG_LSM6DSO_SENSORHUB`, `lsm6dso_shub.c`) and temperature read (`CONFIG_LSM6DSO_ENABLE_TEMP`) are not
part of this driver.

### The core's boundaries and their cost

The core is called with its objects, `struct lsm6dso` and `struct lsm6dso_stream` (both in the binding's
`struct lsm6dso_data`); the binding takes the core's FIFO lock around a single read or a setting
(`lsm6dso_acquire()`, `_release()`, static inline). The core's functions are `lsm6dso_*()` and
`lsm6dso_stream_*()`. Where one would have the name of a call of the extended API, which takes a device
and keeps its name, or fall among `lsm6dso_cal.h`'s, its name puts the verb first:
`lsm6dso_set_cal_lookup()`, `lsm6dso_reload_cal()`, `lsm6dso_get_cal_status()`,
`lsm6dso_measure_gyro_bias()` (for `lsm6dso_cal_zero()`), `lsm6dso_track_gyro_bias()`,
`lsm6dso_copy_cal()`, `lsm6dso_get_fifo_stats()`, `lsm6dso_set_timestamp_decimation()`,
`lsm6dso_get_timestamp_decimation()`, `lsm6dso_read_timestamp()`.

The boundaries `port/lsm6dso_os.h` and `port/lsm6dso_stream_req.h` are made of static inline functions
that the OS's `lsm6dso_os_impl.h` and the user's `lsm6dso_stream_req_impl.h` define, so the locks, the
cycle counter, the bus reads, the work item and RTIO compile to the same calls as Zephyr's own; only
INT1's enable and disable (once a stream) are ordinary functions (`lsm6dso_os.c`). The OS calls the core
back with `lsm6dso_stream_run()` (the work item) and `lsm6dso_stream_int1()` (an edge). The bus is ST's
`stmdev_ctx_t`: its `read_reg` reads at most 255 bytes (stmemsc's length is a `uint8_t`), so the FIFO's
data (up to 3584 bytes), status and timestamp are read with `lsm6dso_os_bus_read()`
(`i2c_burst_read_dt()` on the I2C device behind the context); the stream starts on no other bus
(`lsm6dso_os_bus_read_supported()`). Each batch (once a watermark, at 6667 Hz with 128 words about 104
times a second) goes like this: the GPIO interrupt submits INT1's work
to the system work queue, which calls `lsm6dso_stream_int1()`, which reschedules the stream's work; the
work queue `imu_fifo_worker` calls `lsm6dso_stream_run()` through its handler (one call); the stream
looks the RTIO request up under its spin lock and calls the core's `lsm6dso_fifo_status()`,
`lsm6dso_fifo_read()`, `lsm6dso_track_gyro_bias()` and `lsm6dso_copy_cal()`; RTIO's buffer and
completion, the sensor clock and the next poll are inline. A new OS on the Zephyr compatibility layer
writes nothing for this driver; one without it would define the two `_impl.h` headers and INT1's two
functions.

### Behaviour

- At start the FIFO is put into bypass, the batch rates, timestamp interval and watermark are
  set, and it is switched to stream mode.
- On the threshold interrupt the FIFO status is read and all pending words are read in one
  burst and delivered as one RTIO buffer, headed by a header (word count, periods,
  sensitivities, conversion to physical units).
- The layout of the header (`struct lsm6dso_fifo_packet` in `internal/lsm6dso_fifo_packet.h`) is private to the driver. Code that reads
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

Private attributes on `SENSOR_CHAN_ALL` (`<zephyr/drivers/sensor/lsm6dso.h>`). Both apply at the next
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

The gyro bias is taken again from data recorded at rest (the `lsm6dso_zero_*()` functions of `lsm6dso_cal.c`).

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

## I2C (`i2c_dw`)

Zephyr's DesignWare I2C driver (`drivers/i2c/i2c_dw.c`) with the local patch `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch`
("Local patches of Zephyr" below). The patch holds everything the driver needs, in Zephyr's tree: the application adds no
source or include path to the driver's library. Other controllers behave as in Zephyr, but for the two changes below
that apply to every controller.

**Every controller**: a transfer with a message of no bytes is refused with `-ENOTSUP` before any of its messages is
sent (the controller cannot transfer no bytes; upstream sends one byte instead). The shell's `i2c scan` probes with such
writes; when the first one returns `-ENOTSUP`, it probes that address and the rest with a read of one byte and prints
`Probed with one-byte reads (no zero-length writes)` after the table (the local patch
`zephyr/0004-i2c-shell-scan-read-probe.patch`, for any controller that refuses those writes). An address that acks
is shown, any error (a NACK) is `--`. Probe one address with `i2c direct_read i2c@40044000 6a 1`.
`i2c_recover_bus()` (the shell's `i2c recover`) returns `-ENOTSUP`: the board has no bus recovery. Before a transfer,
the driver still clears a stuck-bus state and goes on to the transfer, as upstream.

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

**The `i2c_dma` command** is `common/shell/i2c_dma_shell.c`, shared by the four builds, built into the application. It
calls the three functions (get and set the mode, get the counters) that the patch declares in
`include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`.

```text
i2c_dma mode          show the current mode
i2c_dma mode on|off   switch, only while no stream or transfer is active
i2c_dma stats         DMA transfers, bytes, errors and fallbacks (cumulative)
```

**The other builds**: the I2C core of the NuttX, FreeRTOS and ThreadX builds (`common/drivers/i2c_dw/`) is a port of this patched
driver. The host test `i2c_dw_sync` detects where the two drift apart (how, in section 9.1 of the
[design](../../../../design/imu-raw-mqtt-design.md); the steps when moving to a newer Zephyr, in "Local patches of
Zephyr" below).

In an earlier measurement of FIFO acquisition alone (6667 Hz, 1 MHz, single runs) with a
watermark of 128, `cpu_active` was 45.7% without DMA and 22.5% with it.

## Sensor shell (`sensor_shell`)

Zephyr's `drivers/sensor/sensor_shell.c` with the local patch `zephyr/0006-sensor-shell-fifo-attributes.patch`:

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

The driver itself is the core shared with the NuttX, FreeRTOS and ThreadX builds (`common/drivers/nina/`),
with no OS of its own; its binding to Zephyr's APIs is `common/drivers/nina/port/zephyr/` (the FreeRTOS and ThreadX
builds compile its Wi-Fi and socket parts, `nina_net.c` and `nina_socket.c`, on their compatibility layer,
which has Zephyr's socket layer down to the list of socket implementations; their kernel services are their own
bindings, `common/drivers/nina/port/freertos/nina_os.c` and `port/threadx/nina_os.c`, on that OS and the Pico SDK). What is
the RP2040's, the esptool passthrough's module side `nina_passthrough_bus_rp2040.c`, is in the same
place as the binding. The
application's `CMakeLists.txt` builds them from there; the binding's devicetree binding is found
through `DTS_ROOT` and its options through the application's `Kconfig` (`rsource`).

| File | Place | Role |
|---|---|---|
| `nina_proto.c` | `common/drivers/nina/internal/` | Framing of SPI commands and replies, the module's reset, the waits for READY |
| `nina_wifi.[ch]` | `common/drivers/nina/` (the header in `include/`) | The Wi-Fi operations: connect (and its wait for the connection and the DHCP result), disconnect, scan, status, the link monitor; the body of the one thread `nina_wifi_tid`, which does them one at a time (the monitor's check every 2 s also during a scan or an attempt); what happens goes to the binding's callbacks |
| `nina_sock.[ch]` | `common/drivers/nina/` (the header, for the bindings, in `internal/`) | The socket table and commands (one pass of each wait loop at a time) |
| `nina_dns.[ch]` | `common/drivers/nina/` (the header in `include/`) | Name resolution (the module's resolver) |
| `nina_proto.h` | `common/drivers/nina/internal/` | The names of the protocol's bytes (framing, the commands the driver sends, the values of their replies) and the declarations of `nina_proto.c`'s requests and of the module's capabilities |
| `nina_shell.c` | `common/shell/` | The `nina version` command (in this build `nina passthrough` too; "The esptool passthrough" below) |
| `nina_passthrough.[ch]` | `common/drivers/nina/` (the header in `include/`) | The esptool passthrough (shared with the NuttX, FreeRTOS and ThreadX builds): the restart's request, DTR and RTS on the module's lines, the rate, the data both ways with the 16 KiB buffer, the way out, the counts, the LED; on its two boundaries `nina_passthrough_bus.h` (the module's side) and `nina_passthrough_os.h` (the host's side) |
| `nina_wait.[ch]` | `common/drivers/nina/internal/` | The waits between the passes of the socket loops (an interval, or the IRQ line's edge) |
| `nina_net.c` | `common/drivers/nina/port/zephyr/` | Zephyr's Wi-Fi management and network interface on the core: the device (`NET_DEVICE_DT_OFFLOAD_DEFINE()`), Zephyr's parameters checked and passed on, the results as Zephyr's structures and events, the address, netmask, gateway and dormant state on the interface, the Wi-Fi profile's reconnection as the core's hooks, the thread `nina_wifi_tid` (`K_THREAD_DEFINE()`, 2048 bytes, priority 8) |
| `nina_os.c` | `common/drivers/nina/port/zephyr/` | Zephyr's side of the core's `nina_os.h`: mutexes, the Wi-Fi thread's wake-up (a semaphore with a timed take), sleeps, clock, the READY interrupt, the waits for the IRQ line's (GPIO2) edges |
| `nina_socket.c` | `common/drivers/nina/port/zephyr/` | Socket offload (file descriptors, errno, `O_NONBLOCK`, `SO_RCVTIMEO`, the wait loops and their sleeps, poll), and the resolver registered as the `getaddrinfo` offload (the FreeRTOS and ThreadX builds' sockets too) |
| `nina_passthrough_os.c` | `common/drivers/nina/port/zephyr/` | Zephyr's side of the passthrough's `nina_passthrough_os.h`: the CDC ACM port and the USB stack's messages, the 1 ms tick, the start (`SYS_INIT`) and `main()`'s way out |
| `nina_passthrough_bus_rp2040.c` | `common/drivers/nina/port/zephyr/` | The passthrough's module side (`nina_passthrough_bus.h`) on the RP2040's registers: the watchdog's scratch registers and restart (`rp2040_watchdog_reboot()` of `common/chip/rp2040`), UART1 with its interrupt, GPIO2, GPIO3, GPIO8, the LED |
| `posix/` | `common/drivers/nina/port/zephyr/` | Headers that map the POSIX socket names the core uses (`struct sockaddr_in`, `AF_INET`, `POLLIN`, ...) to Zephyr's (read by the core and `nina_socket.c` only) |
| `Kconfig`, `dts/bindings/wifi/arduino,nina-spi.yaml` | `common/drivers/nina/port/zephyr/` | The driver's options (`CONFIG_WIFI_NINA_LOCAL`, `CONFIG_NINA_SPI_FREQUENCY`, `CONFIG_NINA_TCP_MAX_IO`, `CONFIG_NINA_SEND_RETRY_MS`) and the devicetree binding of the `nina_w102` node |
| `nina_bus.c` | `common/drivers/nina/port/zephyr/` | Zephyr's side of the core's `nina_bus.h`: the devicetree's `spi1` and the four lines (CS, READY, RESET, IRQ), Zephyr's GPIO and SPI calls; with `CONFIG_NINA_PASSTHROUGH`, no bus while `nina_passthrough_active()` is true (the esptool passthrough) |

Zephyr's Telnet shell, MQTT client and zperf run on these sockets.

### SPI commands

- Commands are handled one at a time. The RP2040 serializes them with a mutex. SPI runs at 8 MHz
  (`spi-max-frequency` of the NINA's devicetree node; the SPI bus and the chip select come from that
  node's parent and `reg` too).
- The PL022 driver runs in its DMA mode (`CONFIG_SPI_PL022_DMA`, with `spi1`'s `dmas` in `app.overlay`: channels
  10 and 11, requested at boot; the I2C0 reads take free channels from 0 up). A transfer of
  `CONFIG_SPI_PL022_DMA_MIN_LEN` bytes (64) or more (the long reads in 128-byte pieces, the socket writes'
  parameters) goes through the two channels while the thread waits for their interrupt; a shorter one, the
  core's single bytes among them, is polled as without the DMA (the local patch `zephyr/0007`; the NuttX,
  FreeRTOS and ThreadX builds draw the same line). The bus stays the module's while CS is low in both paths:
  `nina_bus_select()` takes the driver's lock with `SPI_LOCK_ON` through a transfer of no bytes, which the
  threshold keeps on the polled path (in the DMA mode without one, that transfer would start a DMA; `nina_bus.c`
  does not build that way). A DMA error (the driver logs `<err> spi_pl022: dma:... callback gets error`) or a
  transfer that does not end within its length's time plus 200 ms (`CONFIG_SPI_COMPLETION_TIMEOUT_TOLERANCE`;
  `Timeout waiting for transfer complete`; a byte lost to an RX overrun ends this way) fails the request with
  that error; the patch then stops the channels.
- Changes of the module's READY line (GPIO10) are waited for on a GPIO edge interrupt
  (`common/drivers/nina/port/zephyr/nina_os.c`). The interrupt is enabled before READY is read, so a change after the read is
  not missed; it is off during the transfers. Each wait lasts at most 10 ms, and READY is read
  again after it. Past the deadline (2 s before a command, 10 s for the reply (2 s for
  `nina_proto_ip_config()`), 20 ms after CS is asserted) the result is `-ETIMEDOUT`. Whether the
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

How bit 2 is used (`common/drivers/nina/internal/nina_wait.h`):

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
stays). The mode is the driver's core, shared with the NuttX, FreeRTOS and ThreadX builds
(`common/drivers/nina/nina_passthrough.c`); this build's parts are its host side on Zephyr's USB device
stack (`common/drivers/nina/port/zephyr/nina_passthrough_os.c`) and its module side on the RP2040's registers
(`common/drivers/nina/port/zephyr/nina_passthrough_bus_rp2040.c`). `CONFIG_NINA_PASSTHROUGH` (on by default).

- Wiring: GPIO8 = UART1 TX -> the ESP32's UART0 RX, GPIO9 = UART1 RX. They are the SPI link's MISO and
  CS, so the NINA's SPI cannot run meanwhile. GPIO2 = the ESP32's GPIO0 (boot mode), GPIO3 = RESETN.
  RTS set releases the reset, DTR set pulls GPIO0 low (the sketch's mbed branch). Opening the port
  starts the ROM's download mode, closing it holds the module in reset. When the reset is released
  with DTR clear (the NINA's own firmware starts), GPIO0 is released after 750 ms and GPIO8 is not
  driven. UART1's rate follows the host's line coding (8N1).
- The way in: the command leaves a request in the watchdog's scratch registers 0..3 and restarts with
  the watchdog. The start reads and clears it at once (`POST_KERNEL` 0), so RESET and any later
  restart are normal starts. Nothing is written to the flash (the key-value store).
- Zephyr starts as usual in this mode, but the NINA core sees no SPI bus (`nina_bus_initialize()`
  fails): the Wi-Fi interface's initialization, the Telnet server's socket, `nina version` and the
  like fail with `-ENODEV` without touching SPI1 or the NINA's lines (the Telnet errors stay in the RAM
  log). `main()` runs the relay without settings, calibration, time synchronization or a Wi-Fi
  connection. The CDC ACM interrupt callback is replaced by the relay's (`POST_KERNEL` 99), so the
  shell gets no input and its output does not reach the port. The log goes to RAM only.
- No byte lost: module to PC, UART1's receive interrupt (priority 0, at 4 bytes in the FIFO) moves the
  bytes into a 16 KiB buffer (from `malloc()` on entering the mode), which holds 178 ms at 921600 bps while the PC does not read; what does
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

## Code in RAM

The flash is executed through the RP2040's 16 KiB XIP cache, and the code on the 6667 Hz path of the
sensor and on the send path does not fit in it. The code (`.text.*`; the read-only data stays in the
flash) of the sources listed in `app/usb-console/ram_code.cmake` runs from RAM:

- the kernel objects that the drivers and the application wait on and signal (`kernel/sem.c`, `mutex.c`,
  `poll.c`, `work.c`, `timeout.c`, `queue.c`, `mem_slab.c`), the atomic operations (`lib/os/atomic_c.c`:
  functions on the Cortex-M0+), the memory blocks of the FIFO's buffers with their bit array, RTIO's
  executor and work queue, the sensor clock, and `gpio0`'s driver with hal_rpi_pico's `hardware_gpio`;
- I2C0 (`drivers/i2c/i2c_dw.c` with patch 0003's DMA part) and the DMA driver (`dma_rpi_pico.c`);
- the LSM6DSOX driver (the core's `lsm6dso.c`, `lsm6dso_stream.c`, `lsm6dso_fifo.c`, `lsm6dso_cal.c` and
  the binding's `lsm6dso_sensor.c`, `lsm6dso_rtio.c`, `lsm6dso_rtio_stream.c`, `lsm6dso_os.c`) and the
  stmemsc I2C glue; not ST's `lsm6dso_reg.c`, which only configures, nor the decoder;
- the IMU service's stream, batches, queue and time stamps (not the NTP exchange nor the calibration);
- the send: `imu_raw_mqtt.c`, `imu_raw_packet.c` and Zephyr's MQTT library, the sockets' dispatch
  (`sockets.c`, `socket_offload.c`, the `zvfs` descriptor table and `poll()`), the NINA's sockets,
  protocol, waits and Zephyr binding (`nina_socket.c`, `nina_os.c`, `nina_bus.c`), and the PL022 SPI
  driver.

The scheduler (`kernel/sched.c`), the thread switch (`swap_helper.S`), the interrupt entry and exit, the
SysTick driver and the idle loop stay in the flash. The other builds' lists have the same roles
(`common/imu/port/pico_sdk/ram_code.cmake`, which moves the compatibility layer's kernel part where this
list moves Zephyr's kernel objects). This placement took `cpu_active` while sending
from 35.8% to 21.4% and the cache's misses from about 390,000 to 5,200 a second (33.1% to 19.7% on the
FreeRTOS build). The scheduler, the thread switch and the interrupt entry in RAM too made 0.2 points
(21.2%, for about 4.3 kB), so they stay in the flash. To check a placement,
clear the cache's counters while sending (`devmem 0x1400000c 32 0`, `devmem 0x14000010 32 0`; they
saturate) and read them about 15 s later (the hits and the accesses).

- **How**: Zephyr's code relocation, `CONFIG_CODE_DATA_RELOCATION` (`prj.conf`) and
  `zephyr_code_relocate(FILES ... LOCATION RAM_TEXT NOKEEP)`. Its generator
  (`scripts/build/gen_relocate_app.py`) lists the objects' `.text.*` sections in an output section
  `.ram_text_reloc` that the Cortex-M linker script places before the flash's `.text`: at the start of
  RAM, loaded from the flash. `arch_data_copy()` copies it with `.data` in `z_prep_c()`, before the
  interrupts are set up and the kernel starts; what runs before (`z_arm_reset`, `relocate_vector_table()`,
  `arch_bss_zero()`, the copy with `arch_early_memcpy()`) is not in the list. With `NOKEEP` the functions
  that nothing calls are still left out. Patch 0008 lets the generator find the objects of the LSM6DSO
  module's library (`lsm6dso_local`, whose name is not its directory's).
- **Cost**: 57,320 bytes of RAM (`.ram_text_reloc`): 55,546 bytes of code and 106 long-branch stubs from
  RAM to the flash (16 bytes each). The flash's `.text` has 130 stubs to RAM (4 of them, for the flash's
  write routines, as before). The RAM comes from what is left after
  the image (`_image_ram_end`), which is picolibc's `malloc()` arena
  (`CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=-1`); its one user is the NINA passthrough mode's 16 KiB buffer,
  taken in the mode only (the stacks and the other buffers are static). 63,960 bytes are left.
- **Check**: `ram_code.cmake` stops the configuration when a listed source does not exist. After the link,
  `ram_code_check.py` (the target `ram_code_check`, on `compile_commands.json`, the objects and the map)
  stops the build when a listed object's code is not in RAM or part of it is in the flash (a source no
  longer built, or one the generator did not find), when the code of an object not listed is in RAM, or
  when less than 18,432 bytes of RAM are left (the passthrough mode's 16 KiB buffer and 2 KiB for
  `malloc()` itself); it prints what is in RAM.
- **What stays in the flash**: the vector table and the interrupt entry `_isr_wrapper`, which calls the
  handlers in RAM (`i2c_dw_isr()`, `dma_rpi_pico_isr()`, `gpio_rpi_isr()`) through the software ISR table,
  the start-up code, the key-value store's flash part (the flash's write and erase are hal_rpi_pico's
  `.time_critical` functions, in `.data` as before; while XIP is off only they run, with interrupts
  locked), the NINA passthrough's UART interrupt, and the fatal error path (`fault.c`, the kernel's
  `fatal.c`, the logging core). That path calls moved code: the atomic operations always, and through the
  logging core `k_sem_give()`, `k_sem_take()`, the timeouts and `k_work` when they are needed.

## Local patches of Zephyr

Some changes to Zephyr and its modules are local patches for Zephyr's `west patch`. They are listed in
`zephyr/manifest/zephyr/patches.yml`; the files are in `zephyr/manifest/zephyr/patches/<module>/`.

| Patch | Module | Change |
|---|---|---|
| `hal_rpi_pico/0001-boot2-at25sf128a-clkdiv-2.patch` | hal_rpi_pico | The board's boot2 (`boot2_at25sf128a.S`) runs the flash at clk_sys / 2 (62.5 MHz) instead of clk_sys / 4 (31.25 MHz). hal_rpi_pico has no way to pass the value, so the default changes |
| `zephyr/0001-arm-cortex-m-rp2040-divider-save.patch` | zephyr | `CONFIG_ARM_RP2040_DIVIDER_SAVE`: the thread switches (PendSV, `swap_helper.S`) save and restore the SIO divider |
| `zephyr/0002-soc-rp2040-rom-mem-ops.patch` | zephyr | `CONFIG_RP2040_ROM_MEM_OPS`: `memcpy` and `memset` through the boot ROM's routines |
| `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch` | zephyr | The DesignWare I2C driver (`drivers/i2c/i2c_dw.c`): the RP2040's FIFO depth and RX threshold, and I2C0's DMA reads with `CONFIG_I2C_DW_RP2040_DMA` (I2C). The constants are in a new file `drivers/i2c/i2c_dw_rp2040.h`, the DMA code with the read commands in `i2c_dw_rp2040_dma_priv.h`, and the declarations of the `i2c_dma` command's functions in `include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`; 16 places go into the upstream files (`i2c_dw.c` 14, `i2c_dw.h` 1, `Kconfig.dw` 1); on every controller, a message of no bytes is refused (`-ENOTSUP`) and `i2c_recover_bus()` returns `-ENOTSUP` |
| `zephyr/0004-i2c-shell-scan-read-probe.patch` | zephyr | The shell's `i2c scan` (`drivers/i2c/i2c_shell.c`): when a probe (a write of no bytes) returns `-ENOTSUP`, that address and the rest are probed with a read of one byte, and a line after the table says so, instead of showing `--` everywhere and "0 devices found". Any error of a probe shows `--` |
| `zephyr/0006-sensor-shell-fifo-attributes.patch` | zephyr | The sensor shell (`drivers/sensor/sensor_shell.c`): the LSM6DSO's FIFO attributes and the fixes (Sensor shell), with `drivers/sensor/sensor_shell_os.h`. Needs the LSM6DSO driver of `common/drivers/lsm6dso/` (its `<zephyr/drivers/sensor/lsm6dso.h>`) |
| `zephyr/0007-spi-pl022-dma-min-len.patch` | zephyr | The PL022 SPI driver (`drivers/spi/spi_pl022.c`, `Kconfig.pl022`): `CONFIG_SPI_PL022_DMA_MIN_LEN` (default 0, as upstream): in the DMA mode a transfer shorter than that many bytes takes the polled path (a transfer of no bytes takes the context lock and moves nothing); after a DMA transfer SSPDMACR's request enables are cleared, and after one that failed or timed out its channels are stopped (NINA-W102) |
| `zephyr/0008-code-relocation-module-objects.patch` | zephyr | The code relocation's generator (`scripts/build/gen_relocate_app.py`): when its two searches for a source's object find nothing, it takes the object of the source's name if no other object has that name. A library whose name is not its directory's (the LSM6DSO module's `lsm6dso_local`) names the object of a source in that directory `<library>.dir/<name>.obj`, which the searches did not find; the source then stayed in the flash without a message (Code in RAM) |

- **Applying**: `source script/setup.sh` applies them to the modules' working trees with `west patch apply`
  (no commits; `git -C zephyr/zephyr status` shows them as changes). Patches already applied are skipped,
  so sourcing it again changes nothing. A copy of the set applied is kept in `zephyr/.west/local-patches/`;
  when `patches.yml` or a patch changes, that set is taken back first and the current one applied (a patch
  removed from the list does not stay in a tree). When applying fails, the patches applied are taken back.
- **After changing a patch**: set its `sha256sum` in `patches.yml` to the value of `sha256sum <file>`, then
  source `script/setup.sh` again.
- **Taking them back**: run `west patch reverse` in `zephyr/` and remove `zephyr/.west/local-patches/`. If
  that fails, restore each tree with `git -C <tree> checkout -- .` as `setup.sh` says, remove the copy and
  source it again. The application does not build on trees without the patches (`prj.conf` sets the
  patches' `CONFIG_RP2040_ROM_MEM_OPS`, `CONFIG_I2C_DW_RP2040_DMA` and `CONFIG_SPI_PL022_DMA_MIN_LEN`, and `CONFIG_RP2040_PICO_DIVIDER` stops in CMake without
  `CONFIG_ARM_RP2040_DIVIDER_SAVE`; without 0008 the check of the code in RAM stops the build).
- **Moving to a newer Zephyr**: take the patches back first as above, change the versions in
  `manifest/west.yml` and run `west update`. Remake each patch on the new trees, put it in `patches/`,
  update its `sha256sum`, apply them with `source script/setup.sh` and build. After remaking 0003, run
  `script/host_tests.sh`: bring the upstream changes that `i2c_dw_sync` shows into the shared core
  (`common/drivers/i2c_dw/`), or add the reason not to to the list at the top of `i2c_dw.c`, then
  rewrite the expected differences with `common/tests/i2c_dw_sync.py ... --update` ([Tests](#tests)).
  0006 is made from `common/shell/sensor_shell.c` (with the new tree's upstream file
  merged into it first), so that `zephyr_sensor_shell_patched` passes.

### boot2 at clock divider 2

The read boot2 uses (EBh) is rated for 108 MHz or more by the AT25SF128A's data sheet. With clk_sys at
125 MHz the divider is even, so 62.5 MHz is the most (the same divider as the FreeRTOS and ThreadX builds' Pico SDK
boot2). On the board, `devmem 0x18000014` (the SSI's BAUDR) reads 2.

### The divider (`CONFIG_RP2040_PICO_DIVIDER`)

The application's `CONFIG_RP2040_PICO_DIVIDER` (on by default) builds the Pico SDK's `divider_hardware.S` from
hal_rpi_pico and wraps `__aeabi_idiv`, `__aeabi_idivmod`, `__aeabi_uidiv`, `__aeabi_uidivmod`,
`__aeabi_ldivmod` and `__aeabi_uldivmod` to it (the linker's `--wrap`, `CMakeLists.txt`). Without it they
are libgcc's shift-and-subtract loops.

- The routines do not lock interrupts (`PICO_DIVIDER_DISABLE_INTERRUPTS=0`). When an interrupt handler
  divides in the middle of a division, the routine sees DIRTY and saves the divider itself. When a thread
  is switched out in the middle of one, the patch's `CONFIG_ARM_RP2040_DIVIDER_SAVE` (selected by
  `CONFIG_RP2040_PICO_DIVIDER`) saves it in PendSV: only when DIRTY is set, the four registers go to 20 bytes
  of `struct _thread_arch`, and a thread that was not dividing comes back with DIRTY clear (the Pico SDK's
  FreeRTOS port of the FreeRTOS build saves the four words at every switch; ThreadX's Cortex-M0 port of the ThreadX
  build does not save them at a switch).
- A division by zero gives the Pico SDK routines' results (the same in the four builds): for 32 bits a
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
All but `nina_boundary_*`, `passthrough`, `nina_socket*` and `nina_net` are tests shared with the NuttX, FreeRTOS and ThreadX builds
(`common/tests/`, run by `common/tests/run.sh`).

| Test | Covers |
|---|---|
| `fifo_words` | FIFO word parsing |
| `fifo_decoder` | Decoder for FIFO buffers |
| `lsm6dso_stream` | The stream's state machine (`lsm6dso_stream.c`) on the host's part of its boundaries and fakes of the core's FIFO calls: the start (no INT1 line, the FIFO and INT1, a level already active, INT1 that cannot be enabled, read or routed), a batch on an edge and its header, nothing below the watermark, the end on an overrun, a cancellation or a request gone, the graceful stop with the last partial batch and its errors, a second request, the watermark's bounds |
| `zephyr_sensor_shell_patched` | `common/shell/sensor_shell.c` is byte for byte Zephyr's `drivers/sensor/sensor_shell.c` at HEAD with patch 0006. Only when the Zephyr tree is there |
| `i2c_dw_sync` | The shared I2C core (`common/drivers/i2c_dw/`) against the patched driver (made from the Zephyr tree's files at HEAD and patch 0003): the definitions of both headers (state, interrupts, registers, the IC_DATA_CMD bits, FIFO depth 16 and batch 4, the DMA limit 3584), the SCL counts at 125 MHz, the DMA path's lower limit 32, thresholds 8/0, interrupt mask, IC_CON bit 9, stop waits of 100 ms and timeout of 1000 ms, the spike lengths, and the pairs of functions (Zephyr's names, its DMA API's calls among them, turned into the core's, those of the DMA boundary `i2c_dw_dma_os.h`; compared statement by statement; the differences must be those of `common/tests/i2c_dw_sync.expected`); the read commands' function `i2c_dw_rp2040_read_commands()`, static in `i2c_dw_rp2040_dma_priv.h` of the patch and of the core, statement by statement, with its command bits; `zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`, which the patch adds, against the core's `common/drivers/i2c_dw/include/i2c_dw_rp2040_dma.h`, the same once the comments and include guards are taken out; and that the patched driver's includes are all in the tree. With the build, the values of its `.config` and `zephyr.dts` too. Only when the Zephyr tree is there |
| `i2c_dw_public_headers` | The header for the I2C core's users (`common/drivers/i2c_dw/include/`) compiles with that directory alone: it reads nothing of the core's `internal/` or `port/` |
| `nina_proto` | NINA SPI framing, reply parsing (the firmware's error reply 0xEF included), READY deadlines, the capability probe (0x6A) (the shared core `nina_proto.c`, fake boundaries); each both with the waits on READY's interrupt and, when they cannot start, with READY read every 100 µs |
| `nina_wait` | The socket waits (`nina_wait.c`): the intervals (before the events start, and after they could not), the IRQ line's edges, the 100 ms timeout, the deadline, the back-off (fake clock and line) |
| `nina_core_independent` | Each file of the NINA core (the top of `common/drivers/nina/`, `include/`, `internal/` and the boundary headers of `port/`) compiles with the C library, the POSIX socket types and its own headers only |
| `lsm6dso_core_independent` | Each file of the LSM6DSO core (the `.c` files at the top of `common/drivers/lsm6dso/`, `include/lsm6dso_cal.h` and `lsm6dso_fifo.h`, `internal/` and the boundaries of `port/`) compiles with the C library, ST's `lsm6dso_reg.h`, its own headers and the host's parts of the boundaries (`common/tests/lsm6dso_os_host/`) only |
| `lsm6dso_public_headers` | The headers of `common/drivers/lsm6dso/include/` compile with that directory alone (`lsm6dso.h` with the fakes of the sensor API and RTIO) |
| `nina_wifi` | The Wi-Fi operations (`nina_wifi.c`) on fake boundaries and binding: connect with its checks, its thread (the checks every 500 ms, power mode, the address, 60 checks, a disconnection during the attempt), disconnect, the monitor (lost after two checks), the scans of 3.0.2 and of the patched firmware with their ends, status; the one thread's states together (the monitor's checks at their deadlines within a scan and an attempt, a loss seen during a scan, requests refused while the results are fetched, a request at the deadline, the 10 s and 30 s limits with the checks between) |
| `nina_sock`, `nina_dns`, `nina_dns_offload` | The NINA socket commands, name resolution (`nina_dns_offload` in this build's form, `NINA_OS_DNS_OFFLOAD`) |
| `nina_boundary` | The shared `nina_proto.c` on `nina_bus.c` and `nina_os.c` of `common/drivers/nina/port/zephyr/` (also the Wi-Fi operations' lock and the Wi-Fi thread's wake-up): framing, lines and flags, the reset sequence, the READY interrupt (callback, semaphore reset, edge direction, off before transfers, 10 ms bound) and the sampling when the callback cannot be registered; the waits for the IRQ line's edges (a start that fails, GPIO2's callback and rising edge, the count, a wait ended by an edge, the timeout, an edge just before the wait) |
| `nina_boundary_passthrough` | The same with `CONFIG_NINA_PASSTHROUGH`: no bus while the passthrough runs (`-ENODEV` without touching a line or SPI), as usual once it is gone |
| `nina_boundary_spi_dma`, `nina_bus_spi_dma_threshold` | The same in the bus's configuration of `prj.conf` (the PL022's DMA mode with `CONFIG_SPI_PL022_DMA_MIN_LEN` 64); `nina_bus.c` does not build in the DMA mode without a threshold |
| `nina_passthrough` | The passthrough's core (`nina_passthrough.c`) on fake boundaries: the request against the fatal error record, DTR and RTS on the module's lines, the rate and the way out, the buffer (also on two threads), the start, both data paths with their counts (breaks, overruns, a full buffer, no host), the tick (GPIO0's release, the LED), the way out's sequence |
| `passthrough` | The passthrough's Zephyr side `common/drivers/nina/port/zephyr/nina_passthrough_os.c` and `nina passthrough` of `nina_shell.c` with the real core, on fake Zephyr calls and a fake module side: the request at the start, the lines and the rate from the USB messages and the tick, both data paths and the handovers between the interrupts, the bytes dropped without USB, the way out, the command |
| `nina_socket` | The socket offload `common/drivers/nina/port/zephyr/nina_socket.c` on the shared core: descriptors, the wait loops and sleeps (after the events could not start), options, accept, sendmsg, datagrams, poll, the resolver's registration; the waits for the IRQ line's edges once the events are started (an edge during a pass, the timeout, the deadline, the send's waits, a `poll` with another kind of descriptor) |
| `nina_net` | The Zephyr binding `common/drivers/nina/port/zephyr/nina_net.c` on fake Zephyr declarations and a fake core: the device, the interface's init, the scan's parameters and results, connect, disconnect and status, the core's callbacks on the interface and as Zephyr's events, the profile's hooks, the thread's stack and priority |
