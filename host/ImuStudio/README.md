# ImuStudio

[日本語](README_ja.md)

A desktop application that receives the board's raw IMU data, converts it to physical units
and displays it. The board's shell (Telnet) can be used from inside the application.

- Framework: .NET 10 / Avalonia 11 / Silk.NET (OpenGL)
- Target OS: Linux (the `ip` command is used to find the board)

## What it does now

| Feature | State |
| --- | --- |
| Receiving RAW data (MQTT, 6667 Hz), checking for loss and format | Implemented |
| Using the board's shell (Telnet) | Implemented |
| Conversion to physical units with the calibration, gyro bias tracking at rest | Implemented |
| 3D attitude display with the Madgwick filter (60 Hz) | Implemented |
| Recording and replay | Implemented |
| Frequency content display (spectrogram, updated at 60 Hz) | Implemented |
| Position estimation | Not implemented |

## Build and run

```bash
cd host/ImuStudio
dotnet build ImuStudio.slnx
dotnet test tests/ImuStudio.Core.Tests
dotnet run --project src/ImuStudio.App
```

Arguments (all optional):

| Argument | Meaning |
| --- | --- |
| `--port <number>` | Port to listen on for MQTT. Default: the last value (1883 the first time) |
| `--board <address>` | Connect to the shell at this address on start. `address:port` is accepted |
| `--replay <directory>` | Replay a recording on start |
| `--view <attitude\|spectrum>` | The view to start with |

The port, the board address, the filter gain, the spectrum settings and the height of the
shell pane are saved to
`~/.config/ImuStudio/settings.json` on exit.

## Usage

By default the board's `imu start` sends to port 1883 of its default gateway (the PC running
the access point). If `imu.port` is set in the board's KV store, enter the same value in
**MQTT port** and press **Listen**. The port cannot be shared with `imu-host check` and the like.

1. **Find** looks up the board's address (in the access point's neighbour table; typing it
   works too).
2. **Shell** connects to the shell. Pressing it again disconnects.
3. **imu start** starts the measurement and **imu stop** ends it. Other commands can be sent
   from the shell at the bottom (Enter to send, Up/Down for history). Drag the bar above the
   shell pane to change its height; **Shell pane** at the bottom right hides and shows it.
4. When a measurement starts, the calibration (`imu.cal.accel.<sensitivity>`,
   `imu.cal.gyro.<sensitivity>`) is read from the board's KV store and used from then on.
   A sensitivity without a calibration is converted with the data sheet sensitivity. The
   status line shows which is in use after `calibration:` as `stored` or `nominal`.
   After writing a new calibration to the board, **Reload calibration** reads it again.

Without the shell, data started by other means (the USB console, for example) is still
received and shown. The calibration cannot be read then, so the data sheet sensitivity is used.

### Attitude

The Madgwick filter is updated on every gyro sample (6667 Hz) and drawn at 60 Hz.

- The first acceleration sets the starting tilt. Heading (yaw) cannot be known from an IMU
  alone and is relative to the start.
- **Madgwick β** is the strength of the correction from the acceleration. A larger value pulls
  the tilt to the acceleration faster and makes it more sensitive to acceleration while moving.
- **Reset attitude** sets the tilt again from the current acceleration.
- The view rotates by dragging, pans with Shift + drag and zooms with the wheel.

### Spectrum

Six spectrograms, of the acceleration and of the angular rate. Time runs to the right (about
ten seconds), frequency upwards, and the colour is the level. The curve at the right of each
is the current spectrum. Only the view whose tab is open is running (switching between
attitude and spectrum starts each one over).

- **Frame**: the mean acceleration of the first 0.25 s after the tab is opened is taken as the
  direction of gravity and becomes one axis (vertical). The other two are horizontal;
  horizontal 1 is the sensor's X axis laid flat. The frame is fixed to the sensor, so
  tilting the sensor takes "vertical" away from the true vertical. Hold it still and press
  **Level again** to take the direction of gravity again.
- **Window and step**: the last **FFT size** samples get a Hann window and have their mean
  removed before the FFT. The window moves by 1/60 s, so the display updates at 60 Hz
  however long the window is. A longer window resolves frequency more finely and blurs
  changes in time (4096 is about 0.61 s and about 1.6 Hz).
- **Levels**: amplitude in dB, re 1 m/s² for acceleration and re 1 dps for angular rate; a
  sine of amplitude A reads 20·log10(A) at its frequency. **Min dB** / **Max dB** are the
  levels drawn in the darkest and the brightest colour.
- **Frequency axis**: **Max freq** chooses the upper end (Nyquist is half the sample rate,
  about 3350 Hz). **Logarithmic frequency** makes the axis logarithmic (from 2 Hz). Changing
  the axis starts the pictures over.
- `peak` in each picture is the strongest frequency and its level (the two bins next to DC
  are left out).
- After a loss of data the pictures pause until the window has filled again, so that no
  window spans the loss.

### Gyro bias

The RAW data over MQTT does not carry the bias the board tracked, so the application tracks
it the same way (the sensor counts as resting once eight consecutive eighth-of-a-second
segment means agree). `Gyro bias` on the right shows the current value and the number of
updates. With a calibration, a value more than 1 dps from the stored bias is not taken.

### Recording and replay

- **Record** saves what is received until it is pressed again, to `host/recordings/<time>/`.
  The layout is that of `imu-host record` (`raw.bin`, `messages.jsonl`, `meta.json`) and
  `imu-host calib` can read it. `meta.json` also holds the calibration in use.
- **Replay…** takes a recording directory and replays it at its real pace. Data from the
  board is not shown during a replay. Recordings of `imu-host record` can be replayed too.
- During a replay, **Pause** pauses and resumes, the slider moves to another time and
  **Stop replay** ends it. After a move, the attitude and the bias tracking start over from
  the new place (the tilt is set again from the acceleration and the heading returns to 0).
  For a recording without a calibration, a calibration the application read from the board
  is used if there is one, otherwise the data sheet sensitivity.

## Status line

| Text | Meaning |
| --- | --- |
| `Live from <address>` / `Replay` | Where the data comes from |
| `measuring` etc. | The state the board reports in its status |
| `batches` / `missing` / `invalid` | Batches received, batches missing by sequence number, payloads that do not fit the format |
| `epoch changes` / `index jumps` | Losses of unknown size (such as a FIFO overrun on the board). Shown only when not zero |
| `<number> Hz` | Sample rate derived from the sensor's ticks. `(nominal tick)` means the tick rate has not been measured yet |
| `calibration:` | What converts the data: `stored` is the board's calibration, `nominal` the data sheet sensitivity |
| `PC time on <percent> of batches` | Share of batches that carry a PC timestamp |

The attitude continues across a loss. The rotation during the lost time (about 29 ms for one
batch) is not in the attitude; the tilt recovers through the correction from the
acceleration, an error in the heading stays.

## What has been checked

As of 2026-11-13.

- Unit tests (`dotnet test`, 44 tests): packet checks, the MQTT endpoint, Telnet output
  handling, decoding and applying a calibration, bias tracking, sample period, attitude,
  reading and writing recordings, pausing a replay and moving it to another time,
  the FFT and the spectra (frequency, level and axis of known sines).
- Replay of a recording taken on hardware (about 379 s, about 2.54 million samples): no loss,
  sample rate 6703.5 Hz, magnitude of the acceleration at rest 0.999 g with the calibration.
- Connection to a script standing in for the board (Telnet replies and a recording sent over
  MQTT): receiving, the shell pane, reading the calibration from KV, detection of one missing
  batch.
- The spectrum view: with a replay of a recording taken on hardware, the six pictures scroll.
  CPU use is about 40 % of one core (10–20 % for the attitude view). The drawing period was
  not measured.
- Connection to the real board: the user confirmed that the shell (Telnet) and the buttons
  work. No test of a set duration or count was made.

## Layout

| Project | Role |
| --- | --- |
| `src/ImuStudio.Core` | Packets, MQTT endpoint, shell, calibration, bias tracking, Madgwick, spectra, recording and replay |
| `src/ImuStudio.Rendering` | 3D drawing with Silk.NET OpenGL |
| `src/ImuStudio.App` | The Avalonia user interface |
| `tests/ImuStudio.Core.Tests` | xUnit unit tests |

The icon (`src/ImuStudio.App/Assets/icon.png`) is drawn by `tools/make_icon.py`. To have it
in the GNOME dock and the application list, run `tools/install_desktop.sh` once (it builds
the Release configuration and writes `~/.local/share/applications/ImuStudio.desktop`;
`--remove` takes it away).

`ImuPipeline` turns the received messages into samples in physical units and hands them to
the processing of the view in use (`ISampleSink`). A new view adds such a processing.
