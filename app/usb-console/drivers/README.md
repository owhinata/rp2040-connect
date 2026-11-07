# Local driver baseline

English | [日本語](README_ja.md)

`lsm6dso/` was copied from Zephyr v4.5.0, commit `41d10b59730406bbab4ca9f1fbbe9e96aafb4966`. The original copyright and Apache-2.0 license notices are retained. The copied sensor driver now includes internal FIFO support. `lsm6dso_fifo.c` parses FIFO words, and `lsm6dso_fifo_hw.c` implements FIFO configuration, status checks, and I²C burst reads. Internal start, stop, and read functions are connected to the Streaming API.

`lsm6dso_fifo_decoder.c` implements the Zephyr FIFO decoder and is connected to `get_decoder`, which selects it or the ordinary single-read decoder. Sample times are estimated from the host read-completion time and nominal ODR. They include I²C transfer and interrupt-processing latency and are not the exact sensor generation times.

The application replaces the relevant sensor sources in Zephyr's driver libraries with local copies. Zephyr's Kconfig, devicetree bindings, ST common bus layer, HAL, and unmodified `i2c_dw` I²C driver are used. The standard and local sensor versions are not compiled together. Configuration fails if a change in Zephyr's library structure leaves no sensor source to replace.

`sensor_shell/sensor_shell.c` is copied from the same Zephyr commit. It skips channels with no frames and completion notifications without a buffer, such as those that can occur during cancellation. FIFO processing runs on a dedicated workqueue, and the sensor shell processing stack is set to 3 KiB.

## TODO

- [ ] **Support sensor timestamps.** Sample times are currently estimated from host read-completion time and nominal ODR. Investigate reconstructing times from LSM6DSOX FIFO timestamps and tags, including mapping sensor time to RP2040 time, counter rollover and reset, and discontinuities after FIFO overrun. The nominal resolution is 25 µs; actual timing accuracy and ODR depend on the sensor's frequency error. See [ST AN5272](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf).
- [ ] **Support internal temperature measurements.** The temperature channel is currently disabled and temperature is not stored in the FIFO. Add ordinary Sensor Fetch/Get and FIFO streaming support, and validate temperature tag decoding, unit conversion, and timestamp association. Temperature is measured inside the sensor and has a maximum update rate of 52 Hz.
- [ ] **Re-evaluate with the additional data.** FIFO timestamps and temperature increase I²C traffic. Repeat the sample count, error, transfer time, and CPU usage measurements with both sensors at 6,667 Hz and I²C at 1 MHz. Choose the FIFO timestamp batching rate with both accuracy and bus bandwidth in mind.

The configuration has been built successfully, and generated build commands confirm that the sources are replaced. Hardware checks have covered I²C scans and `sensor get` at 100 kHz and 1 MHz, and accelerometer/gyroscope `sampling_frequency` changes at 100 kHz. NACK and invalid-argument checks remain. Ordinary reads after stopping and the absence of unrelated channel output at stop have also been checked. Recovery from errors and sharing the bus with other I²C devices remain unverified. The standard I²C driver has been checked on hardware at 6667 Hz for 10 seconds; FIFO accounting matched and `sensor get` worked after the stream stopped.

## Hardware results

### Continuous acquisition through the shell

| Sensors | Configured ODR | FIFO words | Transfers | Total transfer time | I²C errors / buffer shortages / overrun observations |
|---|---:|---:|---:|---:|---:|
| Accelerometer and gyroscope | 26 Hz each | 608 | 19 | Approx. 0.369 s | 0 / 0 / 0 |
| Accelerometer and gyroscope | 104 Hz each | 2,464 | 77 | Approx. 1.495 s | 0 / 0 / 0 |
| Accelerometer only | 104 Hz | 960 | 30 | Approx. 0.582 s | 0 / 0 / 0 |
| Gyroscope only | 104 Hz | 1,024 | 32 | Approx. 0.621 s | 0 / 0 / 0 |
| Accelerometer and gyroscope | 208 Hz each | 3,904 | 122 | Approx. 2.366 s | 0 / 0 / 0 |

After the 208 Hz run, `ps` reported a sensor shell stack high-water mark of 1,208 / 3,072 bytes. CPU usage measured after stopping does not indicate CPU load during acquisition.

### Measurements with `imu_measure`

The following results were obtained before final FIFO draining and accounting were added. Both sensors were set to the same ODR and samples were not printed individually. “Not recorded” means that the runtime I²C speed was not recorded.

| ODR (each) | I²C | Duration | Samples received (each) | Observed rate (each) | Batches / FIFO transfers | FIFO words | Transfer time total / max | CPU usage |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 208 Hz | Not recorded | 10 s | 2,080 | 208.0 Hz | 130 / 130 | 4,160 | 2.520 s / 19.4 ms | 1.04% |
| 417 Hz | Not recorded | 10 s | 4,176 | 417.6 Hz | 261 / 261 | 8,352 | 5.061 s / 19.4 ms | 1.71% |
| 833 Hz | 1 MHz | 10 s | 8,368 | 836.8 Hz | 523 / 523 | 16,736 | 1.042 s / 2.008 ms | 3.33% |
| 1,667 Hz | 1 MHz | 10 s | 16,752 | 1,675.2 Hz | 1,047 / 1,047 | 33,504 | 2.086 s / 2.004 ms | 6.60% |
| 3,333 Hz | 1 MHz | 10 s | 33,508 | 3,350.8 Hz | 2,094 / 2,095 | 67,048 | 4.186 s / 2.864 ms | 13.30% |
| 6,667 Hz | 1 MHz | 10 s | 67,015 | 6,701.5 Hz | 2,732 / 2,733 | 134,076 | 8.251 s / 4.660 ms | 15.13% |
| 6,667 Hz | 1 MHz | 60 s | 402,332 | 6,705.533 Hz | 16,393 / 16,394 | 804,710 | 49.528 s / 5.008 ms | 15.05% |

For each measurement, unknown tags, I²C errors, buffer shortages, overrun observations, dropped RTIO completions, stream errors, and cleanup timeouts were all zero. CPU usage covers the entire system during the measurement, including the measurement and other threads. The observed rate is an estimate from received samples over the measurement interval and includes ODR accuracy and partial-batch effects at both ends.

At 3,333 Hz, the FIFO word count exceeded the receive count by 32 words; the 10- and 60-second runs at 6,667 Hz each differed by 46 words. In-flight transfers and the statistics snapshot did not share the same boundary, so these differences alone do not establish sample loss. Zero overrun observations do not prove lossless capture.

Results after adding final FIFO draining and accounting:

| ODR (each) | I²C | Stop requested / finished | Samples received | Received / FIFO words | Batches / FIFO transfers | Transfer time total / max | CPU usage | Accounting |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 104 Hz | 100 kHz | 2.000 / 2.032 s | 210 each | 420 / 420 | 14 / 14 | 0.255 s / 19.457 ms | 1.22% | match |
| 6,667 Hz | 1 MHz | 10.001 / 10.013 s | 67,065 each | 134,130 / 134,130 | 2,021 / 2,021 | 8.272 s / 5.401 ms | 15.44% | match |
| 6,667 Hz | 1 MHz | 60.000 / 60.010 s | Accelerometer 402,348 / gyroscope 402,350 | 804,698 / 804,698 | 12,106 / 12,106 | 49.625 s / 5.399 ms | 15.34% | match |

For each of these measurements, unknown tags, I²C errors, buffer shortages, overrun observations, dropped RTIO completions, bufferless completions, invalid packets, stream and stop errors, and cleanup timeouts were all zero. `accounting=match` shows agreement between host-side reads and counted samples; it does not prove that every sample generated by the sensor was received.

At 104 Hz, an early stop (requested after 1.747 seconds) and an immediate restart were also tested on hardware. The early stop accounted for 364 / 364 words and the restart for 420 / 420 words; both reported `match` with no errors or cleanup timeout. Batching is stopped for the two sensors in sequence, so their final sample counts can differ.

The FIFO burst read uses the circular FIFO output register behavior described in [ST AN5272](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3axis-accelerometer-and-3axis-gyroscope-stmicroelectronics.pdf). The standard STMEMSC I²C read length is `uint8_t`, so the FIFO data is read directly through the Zephyr I²C API. The driver's internal start, stop, and read functions are mutually exclusive with ordinary fetch/get and attribute changes. FIFO batching uses the configured ODR, and after an error the driver does not return to ordinary acquisition until stopping succeeds. Existing triggers and the sensor hub do not yet use this lock, so FIFO start is rejected when either is enabled. INT1 watermark events are passed through GPIO to a workqueue that reads the FIFO.

Internal statistics record words read, transfer time, I²C errors, buffer shortages, and overrun flag observations. The number of observations is neither a lost-sample count nor a count of distinct overruns.

The current stream supports a 32-word watermark and `fifo_wm incl` only. Set a nonzero ODR for each sensor to use. FIFO overrun stops the stream with `-EOVERFLOW` to report a discontinuity of unknown size. I²C errors and insufficient receive buffers also stop the stream. Ordinary fetch/get and attribute changes return `-EBUSY` during streaming.

## Using `imu_measure`

`imu_measure` consumes FIFO batches through the Sensor Streaming API and collects statistics without printing individual samples. Configure the ODR of each sensor first. For example, measure both sensors at 208 Hz for 10 seconds:

```text
sensor attr_set lsm6dsox accel_xyz sampling_frequency 208
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 208
imu_measure start 10
```

The measurement stops automatically and prints its result after the specified duration. The duration must be 1–3,600 seconds. Use `imu_measure stop` to stop early and `imu_measure result` to show the last result again. To measure one sensor, set the other's `sampling_frequency` to `0`. `sensor stream` and `imu_measure` cannot run simultaneously. When stopping, the driver stops adding FIFO batches and receives the remaining words and in-flight completions before finalizing statistics. The two sensors are stopped in sequence, so their stop boundaries are not exactly simultaneous.

Example: measure both sensors at the maximum ODR:

```text
i2c speed i2c0 3
sensor attr_set lsm6dsox accel_xyz sampling_frequency 6667
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 6667
imu_measure start 60
```

After confirming `cleanup_timeout=0`, use `i2c speed i2c0 1` to return to 100 kHz. The device at `0x60` was not detected by scans at 1 MHz, so coexistence with that device at this speed remains unverified.

| Output | Meaning |
|---|---|
| `elapsed_ms` / `stop_requested_ms` | Elapsed time including the final drain and cleanup / time until stop was requested |
| `stopped_early` | 1 when an early stop was requested |
| `batches` | FIFO batches counted during the measurement |
| `accel_samples` / `gyro_samples` | Accelerometer and gyroscope samples received during the measurement |
| `other_words` | FIFO words with unsupported tags |
| `observed_hz` | Approximate rate: received samples divided by elapsed time |
| `configured_hz` | ODR configured when the stream started |
| `fifo_words` / `transfers` | FIFO words read and transfer count recorded by the driver |
| `transfer_ns` / `max_transfer_ns` | Total and maximum I²C burst transfer time |
| `i2c_errors` / `buffer_too_small` / `overrun_observations` | I²C errors, buffer shortages, and overrun flag observations |
| `cpu_busy` | Total system CPU usage during the measurement |
| `accounting` | `match` if counts agree without errors, `mismatch` if counts differ, or `incomplete` if errors or other conditions prevent reconciliation |
| `stream_error` / `stop_error` | Errors in streaming and in requesting or waiting for stop |
| `driver_stop_error` | First driver error while disabling interrupts or stopping the FIFO |
| `dropped_completions` / `bufferless_completions` / `invalid_packets` | Dropped completion notifications, successful notifications without a buffer, and invalid received packets |
| `terminal_seen` / `cleanup_timeout` | Whether the terminal notification was received / cleanup timeout |

CPU usage includes the measurement and other threads. The observed rate is an estimate that includes startup configuration and final drain/stop time. Overrun observations do not report lost samples or distinct overrun occurrences. `accounting=match` confirms host-side read/count agreement but does not prove that every sensor-generated sample was received. If `cleanup_timeout=1`, cleanup did not complete and the board must be rebooted before another measurement.

### Displaying values through the shell

The following commands print samples continuously, so use them at a low ODR:

```text
sensor attr_set lsm6dsox accel_xyz sampling_frequency 26
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 26
sensor stream lsm6dsox@6a on fifo_wm incl
sensor stream lsm6dsox@6a off
imu_fifo_stats
```

Zephyr 4.5.0's `sensor stream` requires the full device name `lsm6dsox@6a`; unlike `sensor get` and `sensor attr_set`, it does not accept the `lsm6dsox` node label.

The FIFO word parser can be tested on the host:

```sh
cc -std=c17 -Wall -Wextra -Werror -Iapp/usb-console/drivers/lsm6dso \
  app/usb-console/tests/fifo_words.c app/usb-console/drivers/lsm6dso/lsm6dso_fifo.c \
  -o /tmp/rp2040-fifo-words-test
/tmp/rp2040-fifo-words-test
```

The FIFO hardware control and transfer-length test uses fake I²C/ST APIs. It does not exercise a physical bus or IMU:

```sh
cc -std=c17 -Wall -Wextra -Werror -Iapp/usb-console/tests/fakes \
  -Iapp/usb-console/drivers/lsm6dso app/usb-console/tests/fifo_hw.c \
  app/usb-console/drivers/lsm6dso/lsm6dso_fifo_hw.c -o /tmp/rp2040-fifo-hw-test
/tmp/rp2040-fifo-hw-test
```

The FIFO decoder's unit conversion, frame counts, and partial reads can be tested with fake Sensor API definitions. This test does not access hardware:

```sh
cc -std=c17 -Wall -Wextra -Werror -Iapp/usb-console/tests/fakes \
  -Iapp/usb-console/drivers/lsm6dso app/usb-console/tests/fifo_decoder.c \
  app/usb-console/drivers/lsm6dso/lsm6dso_fifo_decoder.c \
  app/usb-console/drivers/lsm6dso/lsm6dso_fifo.c -o /tmp/rp2040-fifo-decoder-test
/tmp/rp2040-fifo-decoder-test
```

The measurement packet aggregation test covers mixed and unknown tags and invalid lengths and headers:

```sh
cc -std=c17 -Wall -Wextra -Werror -Iapp/usb-console/tests/fakes \
  -Iapp/usb-console/drivers/lsm6dso -Iapp/usb-console/src \
  app/usb-console/tests/imu_measure_packet.c app/usb-console/src/imu_measure_packet.c \
  app/usb-console/drivers/lsm6dso/lsm6dso_fifo.c -o /tmp/rp2040-imu-measure-test
/tmp/rp2040-imu-measure-test
```

The host stop test uses a fake FIFO and RTIO to exercise a stop request during a transfer, receipt of a final batch, and I²C, buffer-shortage, and overrun errors during stop:

```sh
cc -std=c17 -Wall -Wextra -Werror -Iapp/usb-console/tests/fakes \
  -Iapp/usb-console/drivers/lsm6dso app/usb-console/tests/stream_stop.c \
  app/usb-console/drivers/lsm6dso/lsm6dso_fifo.c -o /tmp/rp2040-stream-stop-test
/tmp/rp2040-stream-stop-test
```

## DMA comparison

An experimental local RP2040 DMA implementation was compared against its interrupt-driven path in the same firmware at 1 MHz I²C, with both sensors at 6667 Hz. Because the tested DMA implementation increased CPU usage, it was removed and the application returned to Zephyr’s standard `i2c_dw` driver. Each run lasted 10 seconds; the Zephyr standard driver was then tested separately for 10 seconds. All runs reported matching FIFO accounting, no errors, and no observed overrun.

| Mode | CPU busy | FIFO words | Batches | FIFO word count elapsed |
| --- | ---: | ---: | ---: | ---: |
| Interrupt | 13.05% | 134,186 | 1,494 | 129.366 ms |
| DMA | 34.87% | 134,202 | 3,059 | 144.568 ms |
| Zephyr standard interrupt driver | 15.62% | 134,138 | 2,089 | 143.352 ms |

Word count elapsed includes interrupt and scheduling delays; it is not CPU execution time. This compares the tested implementation, not every possible DMA design. `imu_measure` reports the same value as `decode_elapsed_ns`.
