# imu-host

[日本語](README_ja.md)

Tools that receive the RAW IMU data on the PC. They accept the RAW batches the board sends
over MQTT directly and do three things.

| Command | What it does |
|---|---|
| `imu-host check` | Checks reception (gaps and time consistency; stores nothing) |
| `imu-host record` | Starts a measurement, records the RAW data, and stops it |
| `imu-host calib` | Computes accelerometer and gyro calibration from a recording |
| `imu-host write` | Writes a calibration to the board |

See [`../README.md`](../README.md) for the board side. The desktop application that displays
the received data is in [`ImuStudio/`](ImuStudio/README.md).

## Requirements

- Python 3.10 or later
- A PC that runs the access point the board connects to (the one `script/wifi_ap.sh` creates)
- Docker, for `calib`. The calibration uses
  [imu_tk](https://bitbucket.org/alberto_pretto/imu_tk/), built into an image from
  [owhinata/ubuntu-imu_tk](https://github.com/owhinata/ubuntu-imu_tk).

## Installation

```bash
cd host
python3 -m venv .venv
.venv/bin/pip install -e .
```

This creates a dedicated environment in `host/.venv` and installs `.venv/bin/imu-host`. The
only dependency is numpy. It is separate from the `.venv` for Zephyr at the repository root.
The examples below are run from `host/`.

## Port

Every command listens on the port the board sends to (`imu.port` in the KV store, 1883 by
default). Pass `--port` if `imu.port` was changed. `record` stops the measurement and says so
when the two differ.

## Checking reception (check)

```bash
.venv/bin/imu-host check
```

It accepts one MQTT client and checks the format, sequence numbers, sample numbers and times
of the RAW batches. Start and stop the measurement from the board's shell (`imu start` /
`imu stop`). Ctrl-C ends it and prints the summary as one line of JSON after `RESULT`. The
main fields are:

| Field | Meaning |
|---|---|
| `packets`, `accel`, `gyro` | Batches and samples received |
| `missing_batches`, `missing_accel`, `missing_gyro`, `gaps` | Number and position of gaps |
| `invalid` | Payloads that do not follow the format |
| `unix_valid`, `unix_invalid`, `unix_regressions` | Batches with and without a valid PC time, and times that went backwards |
| `tick_regressions`, `ticks_per_sample` | Sensor times that went backwards, and ticks per sample |
| `epoch_changes` | Losses of unknown size (restarts after a FIFO error) |
| `batch_unix`, `time_points` | Spacing of the batch times, and comparison with the points in status |

Options reproduce a stopped or congested receiver.

| Option | Effect |
|---|---|
| `--drop-at S`, `--drop-for S` | Disconnect and stop listening S seconds after the first RAW batch, then resume |
| `--stall-at S`, `--stall-for S` | Stay connected but stop reading |
| `--rcvbuf BYTES` | Shrink the receive buffer so that a stall reaches the sender sooner |
| `--duration S`, `--idle-timeout S` | End S seconds after start, or after reception stops |

## Recording (record)

```bash
.venv/bin/imu-host record
```

1. Finds the board connected to the access point.
2. Sends `imu start` over Telnet.
3. Records the RAW batches and the info and status messages.
4. On Ctrl-C (or `--duration`), sends `imu stop`, receives the rest, and exits.

The board is driven over Telnet, not the USB serial port, so it can run from a power bank
while recording. Recordings go to `recordings/<date-time>/` by default (`--out` changes it).

A progress line is shown every second.

```text
  42 秒  静止  3.2 秒    姿勢  7  回転 X  310 Y  265 Z  180 度  欠落 0
```

It shows the elapsed time, the state (initial rest, at rest, moving), the pose count, the
rotation per axis in degrees, and the missing batches. A pose is counted when the board rests
for 3 seconds or more with gravity pointing at least 10 degrees away from the previously
counted pose. It is only a guide; imu_tk decides separately which static intervals the
calibration uses.

The rotation is the part of the motion between poses that changed the direction of gravity,
summed per axis. The gyro calibration needs it spread over all three axes; aim for 1000
degrees or more on each. Turning the board flat on the table does not count, because gravity
keeps its direction. A note is printed at the end for any axis below 500 degrees.

| Option | Meaning |
|---|---|
| `--board IP` | IP address of the board |
| `--no-board` | Do not drive the board (run `imu start` / `stop` by hand) |
| `--duration S` | End S seconds after the first RAW batch |
| `--out DIR` | Where to store the recording |

The board's IP address is looked up in the neighbour table of the interface that has
`10.42.0.1` (the default of NetworkManager's shared connection). Pass `--board` when several
devices are connected and none can be singled out, or when the access point uses another
address.

### Recording format

What is received is stored unmodified, so the calibration can be redone from it at any time.

| File | Content |
|---|---|
| `raw.bin` | The RAW payloads, each preceded by its length (u32, little-endian) |
| `messages.jsonl` | info / status, one per line, with the arrival time `t`, `topic` and `payload` |
| `meta.json` | Start and end time of the recording and the reception summary |

The payload format is described in §6 of
[`../design/imu-raw-mqtt-design.md`](../design/imu-raw-mqtt-design.md).

## Calibration (calib)

For the accelerometer and the gyro each, it computes the scale, the axis misalignment and
the bias. No reference equipment is needed. The board is left at rest in many orientations,
and the values follow from two conditions: the acceleration at rest has the magnitude of
gravity, and the rotation between two poses equals the integral of the angular rate (the
method of Tedaldi et al., implemented by imu_tk).

### Taking the data

1. Start the access point, then power the board. Leave it a few minutes to settle thermally.
2. Run `imu-host record`.
3. **Leave the board on the table, untouched, for the first 10 seconds or more.**
4. Put it down in a new orientation and wait **4 seconds or more**. Repeat **36 to 50 times**.
5. End with Ctrl-C.

- Put the board down rather than holding it still. A hand's tremor is not judged as rest.
- When changing the pose, **tip the board up, lay it down, and turn it over**, evenly about
  all three axes. Turning it flat on the table tells the gyro calibration nothing. Watch the
  rotation on the screen grow on all three axes (aim for 1000 degrees or more on each).
- Fixing the board and the power bank in a small box and rolling the box over its edges gives
  many stable orientations and brings in rotation about all three axes.
- A recording with gaps cannot be used, so stay near the access point.
- It takes about 5 minutes.

### Running it

```bash
.venv/bin/imu-host calib recordings/<date-time>
```

1. Checks the recording. It stops on gaps, an epoch change, saturation, or too short a length.
2. Builds the imu_tk Docker image if it is missing (more than 10 minutes the first time).
3. Runs imu_tk. A 5-minute recording takes 3 to 4 minutes, mostly detecting static intervals.
4. Validates and prints the result and saves it as `recordings/<date-time>/calib/calibration.json`.

| Option | Default | Meaning |
|---|---|---|
| `--gravity` | 9.7975 | Gravity [m/s²]. The default is the value for Tokyo |
| `--init-static` | 10 | Length of the initial rest [s] |
| `--interval` | 1.0 | Length taken from each static interval, and the window of the static detector [s] |
| `--force` | | Run despite gaps or saturation |

Changing `--gravity` and the like builds a separate image for those values, because the
parameters of imu_tk are fixed when it is built.

### Result

The values in `calibration.json` convert RAW values (LSB) to physical units.

```text
calibrated = misalignment @ scale @ (raw_lsb - bias_lsb)
```

| Field | Meaning |
|---|---|
| `accel`, `gyro` | `misalignment` (3×3), `scale` (3×3 diagonal), `bias_lsb`. Units are m/s² and rad/s |
| `chip_id`, `session` | The board's unique ID and the session ID of the recording |
| `accel_ug_per_lsb`, `gyro_udps_per_lsb`, `*_odr_hz` | Sensitivity and ODR when recorded. Recalibrate after changing a range |
| `imu_tk` | Commit, image and parameters used, and whether the gyro optimization converged |
| `recording` | The recording, its length, and the tick rate with its source |
| `validation` | The validation below |

The validation treats one-second windows with a small spread of the angular rate as rest.

- Difference between the acceleration magnitude and gravity (before and after calibration)
- Magnitude of the angular rate at rest (before and after)
- Deviation of the scale from the nominal sensitivity
- Rotation that changed the direction of gravity, summed per axis

When an axis has less than 500 degrees of such rotation, its gyro scale and misalignment are
poorly determined and can come out implausible. A note is printed and the exit code is 1
(`calibration.json` is still saved). The accelerometer values are not affected.

"Before" means converted with the nominal sensitivity only. The output of imu_tk (`.calib`
files, log) and its input (`acc.txt`, `gyro.txt`) stay in the same directory.

### Writing to the board (write)

```bash
.venv/bin/imu-host write recordings/<date-time>          # Telnet (through the access point)
.venv/bin/imu-host write recordings/<date-time> --usb    # USB console
```

This writes the calibration to the board's KV store and applies it to the output of
`sensor get` and `sensor stream`. **The RAW data sent over MQTT does not change.** The
argument can also be a `calibration.json` file.

1. Checks that no measurement is running (it does not write during one).
2. Writes the accelerometer and gyro keys with `kv sethex`, and reads them back with `kv get`
   to compare.
3. Runs `imu cal reload` and checks that the board now uses the calibration.

Telnet and the USB console differ only in the path; the commands sent are the same. Without a
device name, `--usb` looks in `/dev/serial/by-id/`.

| Option | Meaning |
|---|---|
| `--board IP` | IP address of the board (Telnet) |
| `--usb [DEVICE]` | Write over the USB console instead of Telnet |
| `--force` | Write even a calibration that did not converge or lacked rotation |

There is one key per sensor and sensitivity.

```text
imu.cal.accel.<µg/LSB>      e.g. imu.cal.accel.244   (±8 g)
imu.cal.gyro.<µdps/LSB>     e.g. imu.cal.gyro.70000  (±2000 dps)
```

The board uses a calibration only while a key for the current sensitivity exists. Changing a
range leaves no matching key, so the conversion falls back to the nominal sensitivity.
Calibrating in another range and writing it adds a key, and the board then switches with the
range. To remove one, run `kv delete <key>` and then `imu cal reload` in the board's shell.

The value is 56 bytes of integer coefficients (`app/usb-console/src/imu_cal.h`). The board
computes `M · (raw − bias)` without floating point. `M` is `misalignment @ scale` in output
units (2⁻²² m/s² or rad/s) with 14 fraction bits, and `bias` is in LSB with 8 fraction bits.
The rounding costs at most 0.2% of one LSB.

### Time base

Integrating the angular rate depends on the time between samples, so sample times are built
from the sensor tick. The tick runs about 0.5% off its nominal 25 µs, so its rate is measured
from the points in the status messages (sensor tick against the board clock). Calibration
works without NTP synchronization.

## What has been verified

- `check` and `record` were verified on hardware (6667 Hz, reception without gaps, detection
  of a disconnect and reconnect, start and stop over Telnet).
- `calib` was verified with synthetic data that embeds known calibration values (about
  4 minutes, 40 poses, with noise): it recovered the scale within 0.03% and the bias within
  0.3 LSB.
- On hardware, two recordings of real poses were calibrated. The calibration from the
  recording with rotation about all three axes reduced the error on the other recording too
  (acceleration error at rest 0.8 mg RMS, error predicting the direction of gravity between
  poses 0.20 degrees). The calibration from the recording with lopsided rotation did worse
  than the nominal sensitivity on the other recording. How well the gyro values repeat
  between recordings taken the same way has not been checked yet.
- `write` was verified over both Telnet and the USB console. With the calibration written,
  the board's output matched the PC applying the calibration to the board's uncalibrated
  output in the same pose, within 0.2 mg and 7×10⁻⁵ rad/s.
- The values hold for the temperature at calibration. The RAW data carries no temperature.
  About two and a half hours after calibrating, and after a power cycle, the acceleration at
  rest was 2 to 3 mg below gravity and a gyro bias of about 0.1 dps remained. The board takes
  the gyro bias again from data at rest (`imu cal`, [`../README.md`](../README.md)).

## Layout

| File | Content |
|---|---|
| `imu_host/packet.py` | Validation and decoding of the IMU1 payload |
| `imu_host/mqtt.py` | MQTT endpoint (one client) |
| `imu_host/board.py` | Finding the board's IP address, and shell commands over Telnet and the USB console |
| `imu_host/recording.py` | Reading and writing recordings, and tick-to-time conversion |
| `imu_host/check.py`, `record.py`, `calib.py` | The commands (`write` is in `calib.py`) |
