# imu-host

[日本語](README_ja.md)

Tools that receive the RAW IMU data on the PC. They accept the RAW batches the board sends
over MQTT directly and do three things.

| Command | What it does |
|---|---|
| `imu-host check` | Checks reception (gaps and time consistency; stores nothing) |
| `imu-host record` | Starts a measurement, records the RAW data, and stops it |
| `imu-host calib` | Computes accelerometer and gyro calibration from a recording |

See [`../README.md`](../README.md) for the board side.

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
  42 秒  静止  3.2 秒    姿勢  7  バッチ 1478  欠落 0  角速度の最大  412 dps
```

It shows the elapsed time, the state (initial rest, at rest, moving), the pose count, the
batches received, the missing batches, and the largest angular rate. A pose is counted when
the board rests for 3 seconds or more with gravity pointing at least 10 degrees away from the
previously counted pose. It is only a guide; imu_tk decides separately which static intervals
the calibration uses.

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
- Spread the orientations. Fixing the board and the power bank in a small box lets it rest
  stably in many of them.
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

"Before" means converted with the nominal sensitivity only. The output of imu_tk (`.calib`
files, log) and its input (`acc.txt`, `gyro.txt`) stay in the same directory.

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
- **Calibration from a recording of real poses on hardware has not been verified yet.**
- The values hold for the temperature at calibration. The RAW data carries no temperature.

## Layout

| File | Content |
|---|---|
| `imu_host/packet.py` | Validation and decoding of the IMU1 payload |
| `imu_host/mqtt.py` | MQTT endpoint (one client) |
| `imu_host/board.py` | Finding the board's IP address, and Telnet commands |
| `imu_host/recording.py` | Reading and writing recordings, and tick-to-time conversion |
| `imu_host/check.py`, `record.py`, `calib.py` | The commands |
