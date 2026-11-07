# Local driver baseline

English | [日本語](README_ja.md)

`lsm6dso/` and `i2c_dw/` were copied from Zephyr v4.5.0, commit `41d10b59730406bbab4ca9f1fbbe9e96aafb4966`. The original copyright and Apache-2.0 license notices are retained. The copied sensor driver now includes internal FIFO support. `lsm6dso_fifo.c` parses FIFO words, and `lsm6dso_fifo_hw.c` implements FIFO configuration, status checks, and I²C burst reads. Internal start, stop, and read functions are connected to the Streaming API.

`lsm6dso_fifo_decoder.c` implements the Zephyr FIFO decoder and is connected to `get_decoder`, which selects it or the ordinary single-read decoder. Sample times are estimated from the host read-completion time and nominal ODR. They include I²C transfer and interrupt-processing latency and are not the exact sensor generation times.

The application replaces the relevant I²C and sensor sources in Zephyr's driver libraries with local copies. The local `i2c_dw` uses the RP2040's 16-entry RX and TX FIFOs and a four-byte RX threshold, adapting the threshold for short reads and read tails. TX-empty interrupts are masked while pending RX data prevents more requests. Other controllers retain Zephyr's original behavior. Zephyr's Kconfig, devicetree bindings, ST common bus layer and HAL are used; standard and local driver sources are not compiled together.

`sensor_shell/sensor_shell.c` is copied from the same Zephyr commit. It skips channels with no frames and completion notifications without a buffer, such as those that can occur during cancellation. FIFO processing runs on a dedicated workqueue, and the sensor shell processing stack is set to 3 KiB.

## TODO

- [ ] **Support sensor timestamps.** Sample times are currently estimated from host read-completion time and nominal ODR. Investigate reconstructing times from LSM6DSOX FIFO timestamps and tags, including mapping sensor time to RP2040 time, counter rollover and reset, and discontinuities after FIFO overrun. The nominal resolution is 25 µs; actual timing accuracy and ODR depend on the sensor's frequency error. See [ST AN5272](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf).
- [ ] **Support internal temperature measurements.** The temperature channel is currently disabled and temperature is not stored in the FIFO. Add ordinary Sensor Fetch/Get and FIFO streaming support, and validate temperature tag decoding, unit conversion, and timestamp association. Temperature is measured inside the sensor and has a maximum update rate of 52 Hz.
- [ ] **Re-evaluate with the additional data.** FIFO timestamps and temperature increase I²C traffic. Repeat the sample count, error, transfer time, and CPU usage measurements with both sensors at 6,667 Hz and I²C at 1 MHz. Choose the FIFO timestamp batching rate with both accuracy and bus bandwidth in mind.

The local driver has been built and tested on the Nano RP2040 Connect. At 100 kHz and 1 MHz, 1–65 byte reads, a split read without STOP, and repeated START across direction changes passed. At 1 MHz, an absent-address read returned NACK and the following WHO_AM_I read succeeded. A 208 Hz FIFO stream also completed while 70 separate I²C register reads were issued. The I²C scan found the IMU and ATECC608A, but concurrent communication with a second device and physical bus-stuck recovery remain unverified. A 2 ms timeout test is described below.

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
| `cpu_active` | Non-idle time from idle hooks, including interrupts; resolution 0.1% |
| `cpu_busy` | Scheduler non-idle time; interrupts taken while idle can be omitted |
| `accounting` | `match` if counts agree without errors, `mismatch` if counts differ, or `incomplete` if errors or other conditions prevent reconciliation |
| `stream_error` / `stop_error` | Errors in streaming and in requesting or waiting for stop |
| `driver_stop_error` | First driver error while disabling interrupts or stopping the FIFO |
| `dropped_completions` / `bufferless_completions` / `invalid_packets` | Dropped completion notifications, successful notifications without a buffer, and invalid received packets |
| `terminal_seen` / `cleanup_timeout` | Whether the terminal notification was received / cleanup timeout |

CPU usage is scheduler non-idle time and includes the measurement and other threads; interrupts taken while idle can be omitted. The observed rate is an estimate that includes startup configuration and final drain/stop time. Overrun observations do not report lost samples or distinct overrun occurrences. `accounting=match` confirms host-side read/count agreement but does not prove that every sensor-generated sample was received. If `cleanup_timeout=1`, cleanup did not complete and the board must be rebooted before another measurement.

### Displaying values through the shell

The following commands print samples continuously, so use them at a low ODR:

```text
sensor attr_set lsm6dsox accel_xyz sampling_frequency 26
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 26
sensor stream lsm6dsox@6a on fifo_wm incl
sensor stream lsm6dsox@6a off
imu_measure stats
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

## DMA and watermark measurements

DMA support is complete for this project’s IMU FIFO use case: the normal build enables it by default, and a 60-second run is the agreed duration for validation. DMA handles the final I²C0 read of 32–3584 bytes with a 7-bit address and STOP; writes, shorter reads and other transfer shapes use interrupts. If channels cannot be acquired or configured, the driver falls back to interrupts before starting DMA. Competition with other DMA users will be evaluated when such users are introduced.

The following comparison used the same experimental firmware for both modes at a 32-word watermark.

```sh
west build -p always -b arduino_nano_connect app/usb-console -d build/usb-console-dma-comparison -- -DEXTRA_CONF_FILE=prj_i2c_dma_comparison.conf -DEXTRA_DTC_OVERLAY_FILE=dma_comparison.overlay
west flash -d build/usb-console-dma-comparison -r uf2
```

Set I²C to 1 MHz and both sensor sampling frequencies to 6667 Hz, then use `i2c_dma mode off` or `i2c_dma mode on` before starting `imu_measure start 60`. Change the mode only when no stream or transfer is active. `i2c_dma stats` reports cumulative counters; subtract the values before the run from those afterward to check DMA transfers, errors and fallbacks.

| Mode | `cpu_active` | `cpu_busy` | FIFO words | Batches | Transfer time | DMA transfers |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Interrupt | 66.8% | 31.22% | 805,410 | 18,191 | 42.177 s | 0 |
| DMA | 45.1% | 37.28% | 805,118 | 17,499 | 40.641 s | 17,499 |

These 60-second measurements used the same comparison image. Both runs had matching host FIFO accounting and no reported I²C errors, overruns or dropped completions; DMA errors and fallbacks did not increase during the DMA run. Two alternating 10-second runs per mode reported `cpu_active` of 66.8–66.9% without DMA and 45.3% with DMA. The DMA run reduced measured active time by 21.7 percentage points; `cpu_busy` moved in the opposite direction because it is a different scheduler metric and can omit idle-time interrupts. Instrumentation and the comparison firmware itself contribute to the measured load. Matching host accounting and no observed overrun do not prove that every sensor-generated sample was retained.

Short and repeated-start reads and recovery after a NACK were checked at 100 kHz and 1 MHz. A forced DMA timeout followed by a short successful read passed three consecutive times at 100 kHz with a 2 ms timeout. Concurrent DMA users and physical bus-stuck recovery have not been verified.


### Watermark comparison

The normal build supports `fifo_watermark` values from 1 to 256; change the setting only when streaming is stopped. The default is 32 words. In the same comparison firmware, both sensors were set to 6667 Hz and I²C to 1 MHz. Each of the following runs lasted 10 seconds.

| Watermark (words) | DMA | `cpu_active` | `cpu_busy` | FIFO words | Batches |
| ---: | --- | ---: | ---: | ---: | ---: |
| 32 | Off | 67.0% | 34.12% | 134,194 | 3,062 |
| 32 | On | 45.9% | 38.93% | 134,194 | 2,914 |
| 64 | Off | 60.6% | 27.32% | 134,218 | 1,939 |
| 64 | On | 39.5% | 31.42% | 134,228 | 1,922 |
| 128 | Off | 45.7% | 15.50% | 134,118 | 1,007 |
| 128 | On | 22.5% | 18.25% | 134,186 | 1,003 |

All runs reported matching host accounting and no I²C errors, buffer shortages, observed overruns or dropped completions; DMA error and fallback counters did not increase. These are single 10-second runs per condition, with instrumentation overhead included. Larger thresholds also increase the waiting time before the first FIFO batch and reduce the FIFO's remaining headroom; no observed overrun does not prove that all generated samples were retained.

At 128 words, follow-up 60-second runs gave the following results. The first two used the same comparison firmware; the third used the normal DMA-enabled firmware.

| Firmware | DMA | `cpu_active` | FIFO words | Batches / DMA transfers |
| --- | --- | ---: | ---: | ---: |
| Comparison | Off | 45.6% | 804,974 | 6,040 / 0 |
| Comparison | On | 22.3% | 804,918 | 6,014 / 6,014 |
| Normal | On | 22.9% | 804,830 | 6,006 / 6,006 |

All three matched host FIFO accounting with no reported I²C errors or overruns. The DMA runs reported no new DMA errors or fallbacks. The normal run averaged about 10.0 ms between batches (60 seconds / 6,006); this is a batch delivery interval, not the sampling interval or a bound on latency.

## CPU measurement

`imu_measure` resets Zephyr's idle-hook CPU-load window at the start and reads it after the stream has stopped and drained. `cpu_active` includes interrupts, application and kernel work, and idle-hook overhead; it is rounded down to 0.1%. The measurement includes stream setup and cleanup. The idle-hook backend has a shared reset window, so other calls to `cpu_load_get(true)` must not run concurrently. `cpu_busy` remains the separate scheduler non-idle statistic and can omit interrupts taken while idle.

A test-only image built with `-DEXTRA_CONF_FILE=prj_cpu_load_validation.conf` provides `cpu_load_validate`. Run it when no IMU measurement is active. It measures an idle window, a busy thread and a timer ISR running for 500 µs every 2 ms; it briefly loads the CPU and resets the shared CPU-load window. On the Nano RP2040 Connect the observed results were 0.1%, 100.0% and 25.7% respectively, with 1,000 timer calls.

With the local I²C driver at 1 MHz and both sensors at 6667 Hz, a 60-second run reported `cpu_active=60.0%` and scheduler `cpu_busy=26.64%`. The run accounted for 804,954 FIFO words, with no reported I²C errors, overruns or dropped completions. FIFO transfer time totaled 39.197 seconds; an earlier run without CPU-load hooks took 38.909 seconds under similar settings, but these single runs do not establish a precise instrumentation overhead. DMA was not enabled for that earlier measurement; this result is not a DMA comparison.

The previously recorded 15.05% at 6667 Hz was scheduler non-idle time from an earlier 60-second run with a different driver and earlier FIFO stopping implementation; the 26.64% above is the same scheduler metric for the newer run. The runs processed 16,393 and 23,672 batches respectively. More frequent batch processing is a possible contributor to the difference, but neither result by itself determines the total physical CPU load because idle-time interrupts were omitted. The 60.0% `cpu_active` value cannot be compared to 15.05% as if they were the same metric.



## Earlier DMA comparison

An experimental local RP2040 DMA implementation was compared against its interrupt-driven path in the same firmware at 1 MHz I²C, with both sensors at 6667 Hz. After this comparison the DMA implementation was removed and the application returned to Zephyr’s standard `i2c_dw` driver. The later FIFO experiment led to the current local driver. Each run lasted 10 seconds; the Zephyr standard driver was then tested separately for 10 seconds. All runs reported matching FIFO accounting, no errors, and no observed overrun.

| Mode | CPU busy | FIFO words | Batches | FIFO word count elapsed |
| --- | ---: | ---: | ---: | ---: |
| Interrupt | 13.05% | 134,186 | 1,494 | 129.366 ms |
| DMA | 34.87% | 134,202 | 3,059 | 144.568 ms |
| Zephyr standard interrupt driver | 15.62% | 134,138 | 2,089 | 143.352 ms |

The CPU figures above are scheduler non-idle time; IRQs taken while idle may be omitted differently in each mode, so they do not establish the relative total physical CPU load. Word count elapsed includes interrupt and scheduling delays; it is not CPU execution time. This compares the tested implementation, not every possible DMA design. `imu_measure` reports the same value as `decode_elapsed_ns`.


## Earlier hardware results

### Continuous acquisition through the shell

| Sensors | Configured ODR | FIFO words | Transfers | Total transfer time | I²C errors / buffer shortages / overrun observations |
|---|---:|---:|---:|---:|---:|
| Accelerometer and gyroscope | 26 Hz each | 608 | 19 | Approx. 0.369 s | 0 / 0 / 0 |
| Accelerometer and gyroscope | 104 Hz each | 2,464 | 77 | Approx. 1.495 s | 0 / 0 / 0 |
| Accelerometer only | 104 Hz | 960 | 30 | Approx. 0.582 s | 0 / 0 / 0 |
| Gyroscope only | 104 Hz | 1,024 | 32 | Approx. 0.621 s | 0 / 0 / 0 |
| Accelerometer and gyroscope | 208 Hz each | 3,904 | 122 | Approx. 2.366 s | 0 / 0 / 0 |

After the 208 Hz run, `kernel ps` reported a sensor shell stack high-water mark of 1,208 / 3,072 bytes. CPU usage measured after stopping does not indicate CPU load during acquisition.

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

For each measurement, unknown tags, I²C errors, buffer shortages, overrun observations, dropped RTIO completions, stream errors, and cleanup timeouts were all zero. CPU usage is scheduler non-idle time, including measurement and other threads; interrupts taken while idle can be omitted. The observed rate is an estimate from received samples over the measurement interval and includes ODR accuracy and partial-batch effects at both ends.

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

The stream supports `fifo_wm incl`; `sensor attr_set lsm6dsox all fifo_watermark <1..256>` sets the watermark while stopped, and `sensor attr_get lsm6dsox all fifo_watermark` reads it (default 32, reset on reboot). Applications use `sensor_attr_set` / `sensor_attr_get` with `SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK` and `SENSOR_CHAN_ALL`. Set a nonzero ODR for each sensor to use. FIFO overrun stops the stream with `-EOVERFLOW` to report a discontinuity of unknown size. I²C errors and insufficient receive buffers also stop the stream. Ordinary fetch/get and attribute changes return `-EBUSY` during streaming.


## CPU distribution profiling

The optional thread profiler adds per-thread scheduler accounting to `imu_measure` without tracing interrupts:

```sh
west build -p always -b arduino_nano_connect app/usb-console -d build/usb-console-profile -- -DEXTRA_CONF_FILE=prj_cpu_profile.conf
west flash -d build/usb-console-profile -r uf2
```

Run `imu_measure start 10` after configuring the sensors. The report shows the fraction of the measurement interval charged to each thread, including the `imu_fifo_worker`. Zephyr's scheduler charges ISR time to the interrupted thread, including the idle thread. Therefore the listed thread times include ISR time, and `cpu_busy` can omit interrupts taken while idle; it is not a complete measure of physical CPU activity. Thread and idle times can also include profiling overhead. The profile is printed at the end of the run, not saved by `imu_measure result`.

For an approximate IRQ breakdown, build separately with `-DEXTRA_CONF_FILE=prj_cpu_profile_irq.conf`. This measures one in 61 invocations per exception and extrapolates; periodic sampling can be biased, sampled intervals include nested interrupts, and IRQ estimates overlap scheduler thread times. Hook overhead affects every traced IRQ. On this board the IRQ profile caused FIFO overruns at 6667 Hz, so its high-rate results do not represent successful normal streaming. Always check `accounting`, stream errors and overrun observations before interpreting a result. `profile_overflow` indicates missing profile data.

On Nano RP2040 Connect at 1 MHz I²C with both sensors at 6667 Hz, a 60-second thread-only profile completed with 804,728 FIFO / received words, matching accounting and no reported errors or overruns. Scheduler time was 10.59% for `imu_fifo_worker`, 4.43% for `imu_measure_tid`, and 84.96% for idle. `decode_elapsed_ns` was 755,065,688 ns including preemption. The IRQ portion at this rate is not separately measured and may be charged to idle.

In a separate IRQ-sampling build at 3333 Hz for 10 seconds, accounting matched with no errors or overruns. Scheduler time was 11.49% for the FIFO worker and 6.06% for the measurement thread; sampled I²C IRQ time was approximately 28.34% with 499,126 IRQ invocations. These figures overlap, depend on instrumentation, and cannot be added. The FIFO transfer-time sum was 5.182 seconds in this profiled run versus 4.186 seconds in the earlier unprofiled 3333 Hz run, illustrating the measurement's perturbation. The IRQ profile at 6667 Hz overran the FIFO and must not be used as a normal-streaming result.

The 3333 Hz IRQ-profile run read 67,078 FIFO words, or 469,546 FIFO payload bytes, and observed 499,126 I²C interrupts across all I²C transactions (about 1.06 per payload byte). Zephyr's DesignWare driver sets the RX FIFO threshold to zero, so each received byte can trigger RX_FULL; register-address writes, FIFO status reads, and STOP handling also generate traffic. The count is therefore consistent with roughly one interrupt per byte, rather than evidence by itself of a repeated TX_EMPTY interrupt. Interrupt-cause sampling is available in the IRQ profile as `i2c_status_samples`; readings are sampled, not exact counts, and the instrumentation changes timing.

A subsequent 3333 Hz / 10-second cause-sampling run completed with matching accounting (67,073 FIFO words), no I²C errors or FIFO overruns, and 499,091 I²C IRQs. Of 8,182 sampled masked interrupt-status snapshots, 7,904 had both RX_FULL and TX_EMPTY set, 278 had TX_EMPTY only, and none had RX_FULL only or neither. The RX FIFO was empty in all 278 TX-only samples. This is consistent with a receive interrupt for approximately every byte, while TX_EMPTY is also frequently asserted; a status snapshot does not establish which condition first asserted the IRQ or whether a TX-only invocation made progress. The samples are periodic rather than random, and the profiling overhead changes transfer timing.


## I²C RX FIFO experiment

This historical experiment compared standard-like depth handling with the actual FIFO depth and different RX thresholds. RP2040's `IC_COMP_PARAM_1` reads zero even though its RX and TX FIFOs have 16 entries; Zephyr's generic driver interprets the reported depth as one. The comparison used an instrumented experimental driver, which was subsequently replaced by the local driver described above. The four-byte threshold was selected for the local driver; the experimental runtime selection commands are no longer included.

A 10-second comparison at 1 MHz I²C with both sensors at 6667 Hz produced the following results. Each run had matching FIFO accounting, no I²C errors and no observed overruns. Mode 0 uses the experimental driver and counters, so it is a standard-like comparison, not an uninstrumented Zephyr build.

| Mode | I²C IRQs | Estimated I²C handler time | FIFO transfer time | FIFO words |
| --- | ---: | ---: | ---: | ---: |
| Standard-like depth handling | 947,909 | 4.006 s (39.96%) | 9.218 s | 134,160 |
| Depth 16, threshold 1 | 890,875 | 3.913 s (39.10%) | 6.435 s | 134,182 |
| Depth 16, threshold 4 | 271,894 | 2.855 s (28.53%) | 6.574 s | 134,172 |
| Depth 16, threshold 8 | 238,357 | 2.686 s (26.84%) | 6.789 s | 134,154 |

The estimated handler time uses one in 61 IRQs, scaled by the interrupt count; it includes possible nested interrupts but excludes interrupt entry, exit, wrapper and counter overhead. Periodic sampling can be biased. Handler time may overlap scheduler thread accounting, and the much smaller batches in the faster modes increase the number of application and driver callbacks. These results therefore do not establish a total CPU busy percentage. RX threshold 8 was also run for 60 seconds with the handler sampler: 805,030 FIFO words and 1,429,389 I²C IRQs were accounted for with no errors or observed overruns. The estimated handler time was 16.118 seconds (26.86%). An 8-byte read from an absent address returned a NACK, after which WHO_AM_I and a 208 Hz FIFO run succeeded. Short register reads of 1–5, 7–9, 15 and 16 bytes were also exercised under each nonstandard mode at 208 Hz; this is not comprehensive error-recovery or bus-sharing validation.

### Local-driver streaming check

At 1 MHz I²C and 6667 Hz on both sensors, the local driver completed a 60-second run with 804,916 received / FIFO words, 23,274 batches / transfers, matching accounting, and zero reported I²C errors, overruns or dropped completions. The FIFO transfer time was 38.909 seconds in total, maximum 3.739 ms. `sensor get` succeeded after stopping. Accounting does not prove every sensor-generated sample was received.


## I²C hardware validation

The optional validation commands are built separately from the ordinary image:

```sh
west build -p always -b arduino_nano_connect app/usb-console -d build/usb-console-i2c-validation -- -DEXTRA_CONF_FILE=prj_i2c_validation.conf
west flash -d build/usb-console-i2c-validation -r uf2
```

With the IMU idle, `i2c_validate` checks 1–65 byte register reads against a stable register prefix and checks guard bytes, a split read without STOP, and repeated START across multiple direction changes. It accesses the onboard LSM6DSOX registers and is not a general test for arbitrary I²C devices. NACK, concurrent use and FIFO streaming should be checked separately.

For timeout recovery, build with `-DEXTRA_CONF_FILE=prj_i2c_timeout.conf`, set `i2c speed i2c0 1` and run `i2c_validate_timeout`. It expects a 64-byte read to time out under the 2 ms limit, then checks that a short WHO_AM_I read succeeds. On hardware this sequence passed three times consecutively. Do not use this short timeout configuration as the ordinary firmware.
