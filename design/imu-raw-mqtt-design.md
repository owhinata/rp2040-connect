# IMU RAW acquisition and MQTT delivery: design

English | [日本語](imu-raw-mqtt-design-ja.md)

## 1. Purpose and status

Acceleration and angular rate are read from the LSM6DSO of an Arduino Nano RP2040 Connect and
sent to an application on a PC with MQTT. Each axis is sent as the sensor's signed 16-bit raw
integer.

This document describes the design of the implementation (`app/usb-console`) as of 2026-11-11
and what has and has not been verified on hardware. The whole is still experimental: the
payload format and topics are implemented but not final, and are expected to be revised
against the specification of a dedicated PC application. See the [README](../README.md) for
usage.

### Agreed requirements

- Acquire raw data of three axes each for the accelerometer and gyroscope at a configured
  output data rate (ODR) of 6667 Hz.
- Send it to the PC as binary over MQTT. The receiving application on the PC is still to be
  written.
- Keep measuring when Wi-Fi is lost. Data that cannot be sent meanwhile may be discarded.
- Synchronize with the PC's time, aiming at an error within a few milliseconds.

This does not promise freedom from loss in every other situation. FIFO overruns, network
congestion and a receiver that stops are to be detected and reported.

### Implementation status

| Item | Status |
|---|---|
| RAW acquisition, numbering, batching | Implemented |
| MQTT sending (QoS 0), discarding while disconnected, reconnecting | Implemented |
| info / status messages | Implemented (experimental JSON) |
| Recovery after FIFO errors and `epoch_id` | Implemented (verified on hardware with injected overruns only) |
| Sensor time (ticks) on batches | Implemented |
| Synchronization with PC time, PC time on batches | Implemented (NTP, payload version 2) |
| Starting and stopping a measurement | `imu start` / `imu stop` in the shell (no path from the PC yet) |
| Session ID and server address | Implemented (assigned automatically; default gateway by default, changed through the key-value store) |
| Receiving and recording application on the PC | Not started (verification script only) |
| Operation with a real MQTT broker | Not verified |

### Verified on hardware

Unless noted, the conditions were: accelerometer and gyroscope at 6667 Hz, FIFO watermark 128,
I2C at 1 MHz, timestamp decimation 8, a 2.4 GHz connection to a UGREEN USB adapter acting as
an access point on the PC, and `imu-host check` as the receiver (at the time, the script it was moved from). **Each test was
run once**; none of this is a long-term guarantee.

- 30 minutes continuously: 63128 batches with none missing, PC time on every batch, no time
  going backwards, no FIFO overrun.
- Receiver stopped (6 s), queue full because the receiver stopped reading, access point down
  (about 50 s and about 86 s): acquisition continued in every case, and the board's discard
  counts agreed with the batches missing on the PC.
- Injected FIFO overruns: the stream restarted by itself in a new epoch and the measurement
  continued.
- Time: checked over USB, the board's estimate stayed between -2.3 and +0.6 ms of PC time,
  steadily over 30 minutes. Time did not go backwards after chrony was stopped (about 90 s)
  or after the access point was lost.

The main things not verified are listed in sections 8 and 9.

### Build and host tests

```sh
west build -b arduino_nano_connect app/usb-console -d build/usb-console
./script/host_tests.sh
```

There is one configuration, `prj.conf`; RAW streaming and time stamping are always included.
The host tests `fifo_words`, `imu_raw_packet`, `imu_raw_batch`, `imu_raw_queue`,
`imu_raw_status`, `imu_raw_time`, `imu_tick_clock`, `time_offset` and others cover the
functions described here.

## 2. Overall structure

```mermaid
flowchart LR
    IMU[LSM6DSO FIFO] -->|I2C| STREAM[Sensor Streaming / RTIO]
    STREAM --> RAW[Parse, number, time stamp]
    RAW --> QUEUE[Bounded send queue]
    QUEUE --> MQTT[MQTT sender]
    MQTT -->|Wi-Fi / TCP| REC[MQTT receiver on the PC]
    NTP[NTP server on the PC: chrony] -->|Wi-Fi / UDP| SYNC[Time estimate on the board]
    SYNC --> RAW
```

| Role | Implementation |
|---|---|
| Parsing FIFO words | `drivers/lsm6dso/lsm6dso_fifo.*` |
| Batching, numbering, epochs | `src/imu_raw_batch.*` |
| Building the payload | `src/imu_raw_packet.*` |
| FIFO timestamps to sample ticks | `src/imu_raw_time.*` |
| Ticks to the board clock | `src/imu_tick_clock.*` |
| Board clock to PC time | `src/time_sync.*`, `src/time_offset.*` |
| Send queue | `src/imu_raw_queue.*` |
| MQTT sending, reconnecting, status / info | `src/imu_raw_mqtt.*`, `src/imu_raw_status.*` |
| Acquisition thread, recovery, shell (`imu`) | `src/imu_stream.c` |
| NINA sockets | `drivers/nina/nina_socket.c` (see the [driver README](../app/usb-console/drivers/README.md)) |

Reading the FIFO and sending are independent. A lost Wi-Fi or MQTT connection, or a send that
waits, never blocks FIFO reads. Thread priorities are 9 for acquisition (batching) and time
synchronization, 10 for the FIFO read worker and 11 for the MQTT sender. The sender makes the
acquisition side wait only for the queue mutex.

On the PC there is for now `imu-host check` (in `host/`), a verification tool that accepts one
MQTT client directly. It is neither a general broker nor a recorder.

## 3. RAW acquisition

### 3.1 Acquisition and parsing

1. Buffers are obtained per FIFO watermark through Zephyr's `sensor_stream()` and RTIO.
2. `lsm6dso_fifo_next_entry()` interprets a buffer in FIFO order and distinguishes
   accelerometer and gyroscope samples, timestamps and other tags. It also extracts each
   word's 2-bit tag counter.
3. Samples are numbered and packed into batches of up to 384 words.
4. A batch is handed to the send queue when it is full or 100 ms after its last sample.

Nothing assumes that accelerometer and gyroscope words alternate in the FIFO; the kind is
carried per word in a bitmap.

### 3.2 Sample numbers, batch numbers and epochs

- Each measurement gets a `session_id`, and the accelerometer numbers, gyroscope numbers and
  batch sequence each start at 0.
- A number is assigned when the sample is taken from the FIFO, whether or not it can be sent.
  Numbers advance through discards caused by a lost connection or a full queue, so the PC can
  tell what is missing from the gaps in what it receives.
- At a discontinuity where **the number of lost samples is unknown**, such as a FIFO overrun,
  `epoch_id` advances by one. Numbers keep counting so that they stay unique in the session,
  but across an epoch boundary consecutive numbers do not mean contiguous samples.

### 3.3 Recovery after FIFO errors

When the driver detects a FIFO overrun, an undersized buffer or an I2C error, it stops the
FIFO and ends the stream with an error. The acquisition thread then sends what it has
collected, advances the epoch and starts the stream again. It gives up and ends the
measurement with the error when

- the sensor's FIFO could not be stopped, or
- three restarts in a row delivered no data.

A packet containing an unexpected tag has its remainder dropped, the epoch advances and the
measurement continues. Recovery was verified by provoking overruns with a test command that
stalled FIFO handling on purpose. That command was removed in the cleanup, so overruns can no
longer be reproduced on the current firmware. I2C faults, the give-up path and unexpected tags
have not been produced on hardware.

## 4. Communication and behaviour when disconnected

### 4.1 Policy

- MQTT 3.1.1; RAW data with QoS 0 and no retain.
- One PUBLISH carries one self-contained batch.
- The send queue has a fixed length of 12 batches (about 0.35 s of data) and never makes FIFO handling wait. When it
  is full, the new batch is discarded and the number of discarded batches and accelerometer
  and gyroscope samples is counted.
- While not connected, the queue content and new batches are discarded. A batch whose send
  failed is not retried.
- After a disconnection the board retries every second. Once reconnected it sends info and
  status again and continues with new batches. The `session_id` does not change and numbers
  keep advancing.

Because of QoS 0, a batch that the board counted as sent may not arrive (data written to a
connection that was about to fail). The disconnection tests showed differences of 1 to 19
batches. The PC detects this from gaps in the sequence, but cannot determine a loss at the
very end unless a later batch arrives.

Right after the access point disappeared, sending blocked for up to about 9 s before the
disconnection was recognized. During that time batches are discarded as "queue full while
connected". Acquisition is not affected.

### 4.2 Server address and behaviour at start

The server is given by the key-value store entries `imu.host` and `imu.port`; the default is
port 1883 on the default gateway. Without `imu.host` the default gateway is looked up again
for every connection attempt, so a measurement can be started before Wi-Fi is connected, and
after moving to another access point the board connects to the PC behind that one.

`imu start` waits up to 3 s for the first connection before acquisition begins. If the
connection is not made in time, acquisition starts alone and batches are discarded and counted
until it is.

### 4.3 States

| State | IMU acquisition | Handling of RAW data |
|---|---|---|
| Not measuring | Stopped | Nothing sent |
| Measuring, MQTT connected | Running | Sent from the queue; new batches discarded and counted when it is full |
| Measuring, not connected | Running | Unsent and new data discarded and counted |
| Reconnected | Running | info / status sent again, new data sent |
| FIFO error | Restarted automatically | Epoch advances (loss of unknown size) |
| Measurement stopped | Stopped | The rest is sent, then a `stopped` status |

## 5. MQTT topics

`<device_id>` consists of letters, digits, `-` and `_` and comes from the key-value store
entry `imu.device` (default `rp2040`).

| Topic | Direction | Payload | QoS / retain | Status |
|---|---|---|---|---|
| `imu/<device_id>/raw` | Board to PC | Binary of section 6 | QoS 0, none | Implemented |
| `imu/<device_id>/info` | Board to PC | JSON with the measurement settings | QoS 1, retained | Implemented |
| `imu/<device_id>/status` | Board to PC | JSON with state and counters | QoS 0, none | Implemented |

### 5.1 info

Sent when a measurement starts, on reconnection and when the stream restarts after a FIFO
error. It uses QoS 1, but is not resent when no PUBACK arrives; it goes out again at the next
reconnection or change.

| Field | Meaning |
|---|---|
| `v` | Version of this JSON (1) |
| `session`, `epoch`, `device` | Session ID (32 hex digits), epoch, device ID |
| `sensor`, `payload`, `payload_version`, `raw`, `axes` | `LSM6DSO`, `IMU1`, `2`, `int16le`, `xyz` |
| `accel_odr_hz`, `gyro_odr_hz` | Configured ODR, not a measured value |
| `accel_ug_per_lsb`, `gyro_udps_per_lsb` | Sensitivity. `app.overlay` sets the ranges to ±8 g and ±2000 dps, which gives 244 and 70000 |
| `fifo_watermark`, `timestamp_decimation` | FIFO settings |
| `time`, `tick_hz_nominal`, `freq_fine` | Kind of tick (`sensor_tick`), nominal 40000 Hz, the sensor's frequency trim |

The tick rate implied by `freq_fine` is only a guide and does not match the measured rate
(section 7.2).

### 5.2 status

Sent on connection and reconnection, when the state changes, and every second. Nothing is kept
for a disconnected period; the cumulative values arrive after reconnection.

| Field | Meaning |
|---|---|
| `state` | `idle` / `measuring` / `stopped` |
| `accel`, `gyro`, `next_batch`, `timestamp_words` | Counts acquired and the next batch number, regardless of sending |
| `fifo_overruns`, `i2c_errors`, `raw_error`, `error` | Errors on the acquisition side |
| `dropped_batches`, `dropped_accel`, `dropped_gyro` | Discarded on the sending side |
| `published`, `connects`, `failures`, `queue_high` | Batches sent (not confirmed delivered), connections, failures, peak queue use |
| `tick`, `tick_mono_us`, `tick_window_us` | A reading of the sensor tick and the board clock taken together (section 7.2) |
| `unix_us`, `unix_estimate_us`, `sync_delay_us` | Estimated PC time for that instant (smoothed and raw) and the NTP round trip behind it |
| `sync` | `ntp` / `unsynchronized` |

## 6. RAW payload (version 2)

### 6.1 Common rules

- Multi-byte integers and raw values are little-endian. Only raw values are signed
  two's-complement `int16`.
- Raw values of each sensor are in the order they were taken from the FIFO.
- One word holds the three axes X, Y, Z in 6 bytes. The kind is in a separate bitmap.
- One PUBLISH contains the header, the bitmap and all raw values. Nothing spans PUBLISH
  boundaries.

### 6.2 Header (96 bytes)

Offsets 0 to 79 are laid out as in version 1; version 2 adds what follows offset 80.

| Offset | Type | Field | Meaning |
|---:|---|---|---|
| 0 | bytes[4] | magic | ASCII `IMU1` |
| 4 | u8 | version | `2` |
| 5 | u8 | flags | See below |
| 6 | u16 | header_len | `96` |
| 8 | bytes[16] | session_id | 128-bit value identifying one measurement |
| 24 | u64 | batch_seq | Batch sequence number, from 0 in a session |
| 32 | u64 | accel_first_index | Number of the first accelerometer sample in this batch |
| 40 | u64 | gyro_first_index | Number of the first gyroscope sample in this batch |
| 48 | u64 | accel_first_tick | Sensor time of the first accelerometer sample (tick extended to 64 bits) |
| 56 | u64 | gyro_first_tick | Sensor time of the first gyroscope sample |
| 64 | u16 | word_count | 1 to 384 |
| 66 | u16 | bitmap_len | `ceil(word_count / 8)` |
| 68 | u32 | epoch_id | Advances by one per loss of unknown size |
| 72 | u32 | time_map_id | Unused (always 0) |
| 76 | u32 | reserved | `0` |
| 80 | u64 | accel_first_unix_us | PC time of the first accelerometer sample (microseconds since 1970) |
| 88 | u64 | gyro_first_unix_us | PC time of the first gyroscope sample |

| flags | Meaning |
|---|---|
| bit 0 | `accel_first_tick` is valid |
| bit 1 | `gyro_first_tick` is valid |
| bit 2 | `accel_first_unix_us` is valid (requires bit 0) |
| bit 3 | `gyro_first_unix_us` is valid (requires bit 1) |
| others | 0 |

For a kind that is absent from the batch, `first_index` is `UINT64_MAX`. An invalid tick or PC
time is 0; validity is decided by the flag, not the value.

### 6.3 Kind bitmap and raw values

In the `bitmap_len` bytes after the header, bit `i % 8` gives the kind of word `i`: `0` for
gyroscope, `1` for accelerometer. Unused high bits of the last byte are 0. The following
`word_count × 6` bytes hold `x, y, z` of each word in order.

A receiver checks the magic, version, lengths, reserved field, flags, unused bitmap bits and
`payload_len == 96 + bitmap_len + 6 × word_count`, and rejects anything out of range or of an
unsupported version instead of misreading it. `host/imu_host/packet.py` does this.

A batch of 384 words has a 48-byte bitmap and 2,304 bytes of raw values, **2,448 bytes** with
the header. With both sensors at 6667 Hz that is about 28.7 ms of data, about 35 batches per
second and about 0.68 Mbit/s of payload.

### 6.4 Using the time on the receiving side

- The time of the first sample in a batch is `*_first_unix_us` as it is, when its flag is set.
- Later samples in a batch are placed by dividing the span to the next batch's first time
  equally by sample number.
- A batch without valid PC time still has ticks, so its time can be filled in from valid
  batches around it or from the readings in status.
- Do not treat the tick length as a constant. The sensor's oscillator was 0.55% off its
  nominal value and also varied by about 70 ppm within 30 minutes.

### 6.5 Session identification

A `session_id` is assigned at every `imu start`. The first 8 bytes are the chip's unique ID,
the next 4 a counter kept in the key-value store (big-endian), and the rest 0. The counter is
stored before the measurement starts, so an ID is never reused across reboots or power loss.
Erasing the key-value store resets the counter to 1.

## 7. Time

### 7.1 Three clocks

1. **Sensor tick**: the timestamp counter inside the LSM6DSO, nominally 25 us.
2. **Board clock**: the RP2040's 1 MHz hardware timer. It is never adjusted.
3. **PC time**: the PC's real time (Unix time), served by chrony.

Zephyr's kernel clock (`k_uptime`, `k_cycle`) is not used as the board clock. It is driven by
SysTick and kept time when idle, but lost about 480 ppm while streaming at 6667 Hz (-59 ms in
122 s). The cause was not investigated. `elapsed_ms` in `imu status` comes from the board
clock, but `kernel uptime` and `uptime_ms` in status are kernel time and can be off by that
much while streaming.

### 7.2 Sensor ticks and the board clock

**Assigning ticks to samples.** The sensor gives all words of one sampling instant the same
2-bit tag counter, which advances by one per instant. A timestamp word describes the samples
that follow it and share its counter. This is the layout observed on hardware with decimation
8; it is also checked at run time and deviations are counted. Samples right after a timestamp
take its value, and later ones advance by the number of ticks per instant measured between the
two most recent timestamps. If that interval is not a whole number of ticks, or the counter
skips, the tick is marked invalid. The 32-bit counter wraps after about 30 hours and is
extended to 64 bits.

A single timestamp cannot be told from a stale or corrupt one. A tick therefore counts as
valid only once the timestamp it rests on has been followed by another at a consistent
distance. This is decided when the batch is sent, so the first sample of a batch normally
still gets a valid tick. A timestamp that goes back without being a wrap is treated as a
restarted counter, and ticks obtained before it are not used.

At 6667 Hz with decimation 8, timestamps were always 48 ticks and 8 samples apart, 6 ticks per
sample.

**Relating ticks to the board clock.** While measuring, the sensor's timestamp register is
read over I2C once per second with the board clock read just before and after (the read takes
0.1 to 0.4 ms). The last 16 such readings are kept; the oldest and newest give the measured
tick rate, with which ticks are converted to the board clock.

- The measured rate was about 40219 ticks per second (24.864 us per tick), 0.55% faster than
  nominal and 0.05% from the value computed from `freq_fine`.
- Until a rate has been measured (about one second into a measurement), the rate of the
  previous measurement is used, or else the one derived from `freq_fine`.
- A conversion that would fall before the previous result is held just after it, so results
  never go backwards.
- A reading is checked against the FIFO's timestamps and not used if it lies before them or
  two seconds or more ahead. A reading that does not continue the earlier ones (a tick length
  outside 23 to 27 us) starts the series over. A tick far from the readings (5 s or more
  ahead, 20 s or more behind) is not converted. This keeps a single bad value from pinning
  all later times through the mechanism that prevents going backwards.

In the first measurement after writing a new image, the first read of the register has
returned a stale value (it did not after a plain reboot; the cause was not investigated). The
check above discards it.

How a tick relates to the instant the sensor actually sampled (the delay inside the sensor)
has not been measured.

### 7.3 The board clock and PC time

The board queries the PC in NTP format. The server is the default gateway (the PC acting as
access point) by default, or a fixed server set with `CONFIG_NINA_NTP_SERVER`. Synchronizing
with a public server was dropped. The PC runs chrony and allows queries from the access
point's subnet (`allow 10.42.0.0/24`, `local stratum 10`).

- A query is sent every 2 s (`CONFIG_NINA_NTP_INTERVAL_MS`) and the reply is polled for at
  1 ms intervals.
- Of the last eight exchanges, the one with the shortest round trip gives the offset. The
  error from an asymmetric path is bounded by half the round trip, so a short one is the more
  certain.
- The drift comes from the change between the best exchanges of successive windows of eight.
- With no reply for 60 s the estimate is invalid. It starts over when Wi-Fi reconnects.

**A clock that never goes back.** The estimate jumps by a few milliseconds, either way,
whenever it switches to another exchange. Times put on batches and used inside the board are
therefore taken from a clock that approaches the estimate at no more than ±500 ppm. It is
continuous and always increasing. Only a difference of half a second or more (the first
estimate, a stepped PC clock) is applied at once. The kernel's realtime clock (`date`) is only
ever moved forward.

**Accuracy verified.** One byte was sent to the board over USB, the board answered
immediately with its estimated time, and the PC's send and receive times bracketed it. In
30712 such probes over 30 minutes none fell outside; at closest the estimate was 0.15 to
0.57 ms after the send time and 1.4 to 2.3 ms before the receive time. If the error was
constant during the test, it lay between -2.3 and +0.6 ms. This is a path other than Wi-Fi,
so the bound includes the bias from an asymmetric Wi-Fi round trip. (The probe command was
removed in the cleanup.)

The NTP round trip while streaming had a median of about 7 ms, and the exchanges used for the
estimate 4 to 6 ms.

**An approach not adopted.** Synchronization requests and replies over MQTT were tried: the
round trip while streaming had a median of about 30 ms with a tail of several hundred, and
checking for input often enough raised the CPU load to 84%.

### 7.4 Without synchronization

Before synchronization, and when NTP has not answered for 60 s or more, the PC time of batches
is invalid (flag clear). Acquisition and ticks continue. When chrony was stopped for about
90 s, about 30 s of batches had invalid time; after it resumed, a correction of up to about
2 ms was approached gradually and time did not go backwards. When the access point was down
for about 86 s, times after reconnection did not fall before those from before the loss, and
the two sides agreed within 1.4 ms.

## 8. Verification

| Item | Status |
|---|---|
| RAW handling and payload format (host tests) | Done |
| Continuous sending (10 min, 30 min) | Done (once each) |
| Receiver stopped, queue full, access point lost | Done (once each) |
| Recovery from FIFO overruns | Done (by injection) |
| I2C faults, the give-up path, unexpected tags | Not done (never produced on hardware) |
| Time: board clock against the PC (USB probe) | Done (between -2.3 and +0.6 ms) |
| Time: delay inside the sensor | Not done (needs a physical reference) |
| Time: loss of synchronization, Wi-Fi reconnection | Done (once each) |
| Settings other than 6667 Hz and decimation 8; different ODRs for the two sensors | Not done |
| Repeated start and stop, resuming after a reboot | Not done |
| A real MQTT broker; operation for hours | Not done |

As a guide to load, `cpu_active` while streaming at 6667 Hz with time stamping was 62 to 66%
(about 72% with the diagnostic USB probe running alongside).

## 9. Open questions

- A path for starting, stopping and configuring a measurement from the PC. For now there are
  only `imu start` and `imu stop` in the shell.
- Whether the PC needs to choose the session ID (the board assigns it now).
- The fields and period of info and status, and the rules for updating the retained info.
  They are experimental JSON for now.
- Whether to use `time_map_id`. PC time is put directly on batches and the field is unused.
- How to treat the delay inside the sensor: take the datasheet value or measure it.
- The recording format of the PC application, and environments that need authentication and
  encryption.

## References

- [Zephyr Read and Decode](https://docs.zephyrproject.org/latest/hardware/peripherals/sensor/read_and_decode.html)
- [ST LSM6DSO datasheet](https://www.st.com/resource/en/datasheet/lsm6dso.pdf)
- [ST LSM6DSO application note AN5192](https://www.st.com/resource/en/application_note/an5192-lsm6dso-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf)
- [OASIS MQTT 3.1.1](https://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html)
- [chrony](https://chrony-project.org/documentation.html)
