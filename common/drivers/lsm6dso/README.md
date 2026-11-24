# LSM6DSO driver (`lsm6dso`)

English | [日本語](README_ja.md)

The firmware of this repository is built on four OSes, Zephyr, NuttX, the Pico SDK with FreeRTOS and the Pico SDK
with ThreadX: the Zephyr, NuttX, FreeRTOS and ThreadX builds. The compatibility layer is the layer that provides
Zephyr's APIs under Zephyr's names on the other OSes ([`common/compat/README.md`](../../compat/README.md)).

`common/drivers/lsm6dso/` is the driver of the board's LSM6DSOX (0x6a on I2C0), shared by the four builds. In place of
Zephyr's `drivers/sensor/st/lsm6dso`, it has FIFO streaming, conversion with a calibration and the tracking of the gyro
bias at rest. It is made of a core with no OS and a Zephyr binding (`port/zephyr/`); the binding is the same in all four
builds (on the compatibility layer in the NuttX, FreeRTOS and ThreadX builds). The `sensor` command
(`common/shell/sensor_shell.c`) is described here too. The threads and buffers of the three paths that read the sensor
(`sensor get`, `sensor stream`, `imu start`) are in
[section 3.4](../../../design/imu-raw-mqtt-design.md#34-the-three-paths-that-read-the-sensor) of the
[design](../../../design/imu-raw-mqtt-design.md), and where the driver sits among the shared sources in section 9.1.

## Directories

| File | Role |
|---|---|
| `lsm6dso.c` | Core: how the chip is operated (the start and reset, rates and ranges, the single reads and their conversion, the FIFO's register work, owner, lock and counters, the timestamp read, the calibration in effect and the bias tracking), on a `struct lsm6dso` |
| `lsm6dso_stream.c` | Core: the FIFO stream's state machine (below), on a `struct lsm6dso_stream` |
| `lsm6dso_cal.c` | Core: the conversion's arithmetic and the tracker of the gyro bias at rest (below) |
| `lsm6dso_fifo.c` | Core: the FIFO words' parser |
| `include/lsm6dso.h` | The extended API (the FIFO words and their parser, the calibration's types and calls, the FIFO's attributes, settings, counters and stop, the timestamp read). The application reads only this, by Zephyr's name `<zephyr/drivers/sensor/lsm6dso.h>` (each OS's section below) |
| `include/lsm6dso_cal.h`, `include/lsm6dso_fifo.h` | For the driver's users: the calibration's types and calls, the FIFO words with their parser and counters |
| `internal/lsm6dso_core.h` | The core's functions (called by the binding) |
| `internal/lsm6dso_stream.h`, `internal/lsm6dso_fifo_packet.h` | The stream's functions, and the layout of its buffers (`struct lsm6dso_fifo_packet`, private to the driver) |
| `port/lsm6dso_os.h` | Boundary: to the OS ("The boundaries the OS implements" below) |
| `port/lsm6dso_stream_req.h` | Boundary: to the stream's user |
| `port/zephyr/` | The Zephyr binding (in all four builds; "Zephyr" below), a Zephyr module of its own (`zephyr/module.yml`, `Kconfig`, `CMakeLists.txt`) |
| `port/zephyr/include/zephyr/drivers/sensor/lsm6dso.h` | The Zephyr build's header of the extended API, which reads `include/lsm6dso.h` |
| `port/nuttx/lsm6dso_instance.h` | The NuttX build's device |
| `port/pico_sdk/` | The device, its start and the build rules of the OSes on the Pico SDK (the FreeRTOS and ThreadX builds) |

Related files outside the driver:

| File | Role |
|---|---|
| `common/drivers/stmemsc/stmemsc.h`, `stmemsc_i2c.c`, `stmemsc_mdelay.c` | A copy of the I2C part of Zephyr's `drivers/sensor/st/stmemsc/` (the glue between ST's `stmdev_ctx_t` and Zephyr's I2C API; `stmemsc_mdelay()` is the layer's `k_msleep()`), used by the NuttX, FreeRTOS and ThreadX builds (compiled without the prelude). `lsm6dso_sensor.h` and `lsm6dso_os_impl.h` include `<stmemsc.h>`, which is this one through the include path. The Zephyr build uses Zephyr's stmemsc |
| ST's `lsm6dso_reg.c`, `lsm6dso_reg.h` | The chip's register work (hal_st): Zephyr's hal_st in the Zephyr build, `nuttx/stmemsc/`, `freertos/stmemsc/` and `threadx/stmemsc/` in the others (not in Git; fetched by each OS's `setup.sh`) |
| `common/compat/include/zephyr/drivers/sensor/lsm6dso.h` | The layer's `<zephyr/drivers/sensor/lsm6dso.h>`, which includes `include/lsm6dso.h` |
| `common/compat/port/gpio_os.h` with its implementations `common/compat/port/nuttx/rp2040_gpio_os.c` and `common/compat/port/pico_sdk/gpio_os_rp2040.c` | GPIO bank 0 under the layer's `gpio0` (the INT1 interrupt) |
| `common/shell/sensor_shell.c` | The `sensor` command ("The `sensor` command" below) |
| `common/compat/sensor_shell_stream.c`, `common/compat/include/sensor_shell.h` | Copies of Zephyr's `sensor_shell_stream.c` and `sensor_shell.h` (NuttX, FreeRTOS and ThreadX builds) |
| `common/shell/port/nuttx/sensor_shell_os.h`, `sensor_shell_nuttx.[ch]`, `common/shell/port/generic/sensor_shell_os.[ch]` | The OS part of the `sensor` command |
| `nuttx/drivers/lsm6dso/Kconfig`, `Make.defs`, `st_lsm6dso_reg.c` | The NuttX build's options and build rules |

## How the core works

### FIFO streaming

The FIFO can be read through Zephyr's Sensor Streaming API (`sensor_stream()` on RTIO) with the trigger
`SENSOR_TRIG_FIFO_WATERMARK` and the option `SENSOR_STREAM_DATA_INCLUDE`.

| File | Role |
|---|---|
| `lsm6dso.c` (core) | The FIFO's owner and lock: its register setup, pause and stop, the status and burst reads (static), INT1's routing of the threshold, its counters, the timestamp decimation, the timestamp read; the single reads, their conversion, the bias reading of `lsm6dso_cal_zero()` |
| `lsm6dso_stream.c` (core) | The stream's state machine: the start at its first run (the FIFO, INT1 enabled and routed), a batch read at the watermark on each INT1 edge and at the poll every 20 ms, the graceful stop, the end on an error, an overrun or the user's cancellation; the watermark |
| `lsm6dso_fifo.c` (core) | The FIFO words' parser (accelerometer, gyroscope, timestamp, other; tag counter) |
| `lsm6dso_cal.c` (core) | Conversion to physical units and the gyro bias at rest (below), used by `lsm6dso.c` and the decoder |
| `lsm6dso_sensor.c` (binding) | Upstream's driver on the core: the device, attributes, `sample_fetch` and the calibrated `channel_get`, the calls of the extended API that take a device |
| `lsm6dso_rtio_stream.c` (binding) | Each device's stream: its INT1 line from the devicetree, RTIO's request handed to the core's stream (another trigger `-ENOTSUP`, a second stream `-EBUSY`), the extended API's stop, watermark and timestamp decimation |
| `lsm6dso_os.c` (binding) | The stream's work queue (`imu_fifo_worker`, Zephyr priority 10, 2048 bytes), INT1's GPIO interrupt, which wakes the stream through the system work queue |
| `lsm6dso_rtio.c` (binding) | The driver's RTIO submit: a stream goes to `lsm6dso_rtio_stream.c`, a single read (`sensor get`) to a copy of the sensor API's fallback on the RTIO work queue |
| `lsm6dso_decoder.c` (binding) | `lsm6dso_fifo_buffer_words()`; the decoder for FIFO buffers, used by `sensor stream`, and the decoder that hands a single read's buffer to Zephyr's default decoder |

The INT1 interrupt is the driver's own (in `lsm6dso_os.c`): the line is enabled only while a stream runs and the system
work queue only wakes the stream's work queue, which does all the I2C reads.

- `sensor_stream()` (`SENSOR_DT_STREAM_IODEV`) submits one multishot read; `lsm6dso_sensor_submit()` hands it to the
  core's stream, which wakes the worker (`imu_fifo_worker`) in the caller.
- The start: on its first run the worker puts the FIFO into bypass, sets the accelerometer and gyroscope batch rates
  (BDR), the timestamp interval (one every 8 by default) and the watermark, switches it to stream mode, enables the
  rising-edge interrupt of INT1 and then routes the FIFO threshold to INT1. If INT1 is already high when enabled, it
  queues one run at once.
- A batch: an INT1 edge → the GPIO interrupt queues work on the system work queue (`sysworkq`, `k_work_submit()`) →
  there the core's `lsm6dso_stream_int1()` wakes the worker (`k_work_reschedule_for_queue(.., K_NO_WAIT)`) → the worker
  reads the FIFO status and, at the watermark or above, reads all pending words in one burst and delivers them as one
  RTIO buffer. The completed multishot goes straight back to the worker (`lsm6dso_sensor_submit()`), which looks at the
  status again at once; with nothing there it schedules another look in 20 ms
  (`k_work_schedule_for_queue(.., K_MSEC(20))`), which also catches a missed edge.
- Each buffer is headed by a header (word count, periods, sensitivities, conversion to physical units). The layout of
  the header (`struct lsm6dso_fifo_packet` in `internal/lsm6dso_fifo_packet.h`) is private to the driver. Code that
  reads a buffer uses the decoder for physical values, or `lsm6dso_fifo_buffer_words()` for the raw words.
  `lsm6dso_fifo_next_entry()` interprets the words one at a time and yields the kind, xyz, the tag counter, and the
  counter value a timestamp word carries.
- Buffers come from the context's memory pool (at least the packet header plus the watermark's words, at most the
  header plus 512 words).
- On a **FIFO overrun** (`-EOVERFLOW`), an I2C error (that error) or a buffer that is too small (`-ENOMEM`), the FIFO is
  stopped and the stream ends with an error. The number of lost samples is unknown, so nothing that looks continuous is
  returned. Whether to restart is up to the caller (`common/imu/imu_stream.c`). Ending puts the FIFO back in bypass and
  turns the INT1 routing and the interrupt off.
- **Stopping** is requested with `lsm6dso_fifo_request_stop()`. The worker stops batching (BDR 0), delivers the words
  left in the FIFO as a last partial batch, then puts the FIFO in bypass and posts a terminal completion
  (`-ECANCELED`). Canceling the RTIO request directly could free a buffer that was read but not yet consumed. A
  cancellation (`rtio_sqe_cancel()`) is noticed at the worker's next run (INT1 or the 20 ms look) and stops at once,
  without pausing, ending with `-ECANCELED` (or the stop's error); the words left in the FIFO are dropped.
- While the FIFO is in use, one-shot reads and attribute changes return `-EBUSY`.
- The stream does not change the ODRs (`imu start` sets them and they stay after the stop).

### Time

The FIFO contains the sensor's timestamps (nominally 25 us per tick) at an interval set by an attribute. The timestamp
counter is enabled at start and never disabled, so it keeps running across stream restarts. Assigning ticks to samples
and converting them to PC time is done by the application (section 7 of the
[design document](../../../design/imu-raw-mqtt-design.md)).

`lsm6dso_timestamp_read()` reads the sensor's timestamp register between two readings of the caller's clock, which
relates the sensor clock to the board clock.

Sample times returned by the decoder are estimated from the host's read completion time and the nominal data rate.
They include I2C transfer and interrupt latency and are not accurate.

### Attributes

Private attributes on `SENSOR_CHAN_ALL` (`<zephyr/drivers/sensor/lsm6dso.h>`). Both apply at the next stream start and
cannot be changed while a stream runs.

| Attribute | Name in the shell | Values | Initial |
|---|---|---|---|
| `SENSOR_ATTR_LSM6DSO_FIFO_WATERMARK` | `fifo_watermark` | 1 to 256 words | 32 |
| `SENSOR_ATTR_LSM6DSO_FIFO_TIMESTAMP_DECIMATION` | `fifo_timestamp_decimation` | 0, 1, 8, 32 | 8 |

A larger watermark means fewer reads and less load, but data arrives later and less of the 512-word FIFO is left as
margin.

### Conversion to physical units and calibration

The conversion from raw values to physical units lives in one function in `lsm6dso_cal.c`, used by both the FIFO
decoder (`sensor stream`) and the one-shot read (`sensor get`).

```text
out = M · (raw − bias)
```

- It is integer arithmetic throughout. `M` is a 3×3 matrix combining scale and axis misalignment; the output is Q31
  with shift 9, in units of 2⁻²² m/s² or rad/s.
- Without a calibration the same function runs with the nominal sensitivity on the diagonal and a zero bias.
- A calibration comes from the function registered with `lsm6dso_cal_set_lookup()`. The driver calls it with the
  current sensitivity at registration, on `lsm6dso_cal_reload()` (`imu cal reload`), and whenever a range is set. If it
  has nothing, the nominal conversion is used. The driver does not read the KV store: the lookup function is registered
  by the application's `common/imu/imu_cal.c` (on NuttX in init's `imu_cal_init()`).
- Each FIFO buffer carries the conversion in effect when it was read. The decoder needs only the buffer, and a range or
  calibration change in between does not affect buffers already read.
- The calibration applies to the output of the single reads (`sensor_channel_get()`) and of the FIFO decoder, not to
  the RAW the acquisition service sends over MQTT.
- `lsm6dso_cal_status()` reports whether the accelerometer and the gyro use a calibration, and the gyro bias in use.

The gyro bias is taken again from data recorded at rest (the `lsm6dso_zero_*()` functions of `lsm6dso_cal.c`, driven by
`lsm6dso.c`).

- Samples read from the FIFO are grouped into segments of an eighth of a second. When eight consecutive segments agree
  in mean angular rate (within 0.3 dps) and mean acceleration (within 2 mg), the sensor counts as resting and the mean
  rate of that window is blended in: a plain average over the first 20 windows, then exponential smoothing with 1/20.
  Single samples are not judged, because at 6667 Hz one sample has too much noise (about 1.7 dps).
- It runs whenever a FIFO stream runs (`imu start`, `sensor stream`). The driver never starts the sensor on its own.
- `lsm6dso_cal_zero()` takes one reading on request. If the sensor is off, it runs at 104 Hz for up to three seconds
  and is switched off again. While a stream owns the FIFO it returns `-EBUSY`.
- The value is kept in RAM and dropped whenever a conversion is selected again (range set, reload). A window more than
  1 dps from the stored bias (5 dps from zero without a calibration) is not used.

### Statistics

`lsm6dso_fifo_get_stats()` returns the words read, the number and duration of transfers, I2C errors, undersized
buffers, overrun observations, the configured data rates, sensitivities and the frequency trim. The statistics are
cleared and `generation` advances each time the FIFO is started.

### Limitations

- I2C only (the FIFO refuses a device on SPI).
- The internal temperature is not read and not stored in the FIFO. Upstream's trigger (`CONFIG_LSM6DSO_TRIGGER`,
  `lsm6dso_trigger.c`), sensor hub (`CONFIG_LSM6DSO_SENSORHUB`, `lsm6dso_shub.c`) and temperature read
  (`CONFIG_LSM6DSO_ENABLE_TEMP`) are not part of this driver.

## The boundaries the OS implements

The core is called with its objects, `struct lsm6dso` and `struct lsm6dso_stream` (both in the binding's
`struct lsm6dso_data`); the binding takes the core's FIFO lock around a single read or a setting (`lsm6dso_acquire()`,
`_release()`, static inline). The core's functions are `lsm6dso_*()` and `lsm6dso_stream_*()`. Where one would have the
name of a call of the extended API, which takes a device and keeps its name, or fall among `lsm6dso_cal.h`'s, its name
puts the verb first: `lsm6dso_set_cal_lookup()`, `lsm6dso_reload_cal()`, `lsm6dso_get_cal_status()`,
`lsm6dso_measure_gyro_bias()` (for `lsm6dso_cal_zero()`), `lsm6dso_track_gyro_bias()`, `lsm6dso_copy_cal()`,
`lsm6dso_get_fifo_stats()`, `lsm6dso_set_timestamp_decimation()`, `lsm6dso_get_timestamp_decimation()`,
`lsm6dso_read_timestamp()`.

The core reaches the chip through ST's `lsm6dso_reg.c` (hal_st) and the rest through two boundaries in `port/`:

- `port/lsm6dso_os.h`, the OS: two locks, waits, a cycle counter, the FIFO's long reads, the log; for the stream a work
  item on a thread of the driver's own, the INT1 line and the clock of the batches' time stamps.
- `port/lsm6dso_stream_req.h`, the stream's user: the request it serves, a buffer for each batch, the batch or the error
  handed back.

The boundaries are made of static inline functions that the OS's `lsm6dso_os_impl.h` and the user's
`lsm6dso_stream_req_impl.h` define, so the locks, the cycle counter, the bus reads, the work item and RTIO compile to the
same calls as Zephyr's own; only INT1's enable and disable (once a stream) are ordinary functions (`lsm6dso_os.c`). The
OS calls the core back with `lsm6dso_stream_run()` (the work item) and `lsm6dso_stream_int1()` (an edge).

The bus is ST's `stmdev_ctx_t`: its `read_reg` reads at most 255 bytes (stmemsc's length is a `uint8_t`), so the FIFO's
data (up to 3584 bytes), status and timestamp are read with `lsm6dso_os_bus_read()` (`i2c_burst_read_dt()` on the I2C
device behind the context); the stream starts on no other bus (`lsm6dso_os_bus_read_supported()`).

Each batch (once a watermark, at 6667 Hz with 128 words about 104 times a second) goes like this: the GPIO interrupt
submits INT1's work to the system work queue, which calls `lsm6dso_stream_int1()`, which reschedules the stream's work;
the work queue `imu_fifo_worker` calls `lsm6dso_stream_run()` through its handler (one call); the stream looks the RTIO
request up under its spin lock and calls the core's `lsm6dso_fifo_status()`, `lsm6dso_fifo_read()`,
`lsm6dso_track_gyro_bias()` and `lsm6dso_copy_cal()`; RTIO's buffer and completion, the sensor clock and the next poll
are inline.

In all four builds the implementation is the Zephyr binding's `port/zephyr/lsm6dso_os_impl.h` with `lsm6dso_os.c`, and
`port/zephyr/lsm6dso_stream_req_impl.h` (RTIO); the NuttX, FreeRTOS and ThreadX builds compile it on the compatibility
layer. Each OS has only the device at the end of the binding, `lsm6dso_instance.h`. A new OS on the Zephyr compatibility
layer writes nothing for this driver; one without it would define the two `_impl.h` headers and INT1's two functions.

## Each OS's binding

### Zephyr

The binding `port/zephyr/` has the device and the sensor API (`lsm6dso_sensor.c`, `lsm6dso_sensor.h`), the
devicetree's devices (`lsm6dso_instance.h`), the stream's RTIO side (`lsm6dso_rtio_stream.c`, `lsm6dso_rtio.c`,
`lsm6dso_decoder.[ch]`), the OS part of the core's boundary (`lsm6dso_os_impl.h` with `lsm6dso_os.c`: `k_mutex`,
`k_spinlock`, `k_busy_wait()`, `k_msleep()`, `k_usleep()`, `k_cycle_get_32()`, `i2c_burst_read_dt()`, a delayable work
item on the work queue `imu_fifo_worker`, INT1's GPIO interrupt through the system work queue, the sensor clock,
Zephyr's logging) and the user part (`lsm6dso_stream_req_impl.h`: RTIO's multishot request). The binding's file names
are upstream's and those of the sibling driver with FIFO streaming, `lsm6dsv16x`.

- The binding is a Zephyr module of its own (`port/zephyr/zephyr/module.yml`, which the application's `CMakeLists.txt`
  adds to `ZEPHYR_EXTRA_MODULES`).
- Its `Kconfig` has the driver's option `CONFIG_LSM6DSO_LOCAL` (on by default for an `st,lsm6dso` or `st,lsm6dso32`
  node; it selects `HAS_STMEMSC` and `USE_STDC_LSM6DSO` for ST's register driver and Zephyr's stmemsc glue), and
  `prj.conf` turns Zephyr's driver of the same devices off (`CONFIG_LSM6DSO=n`).
- Its `CMakeLists.txt` builds the library `lsm6dso_local` and exports the extended API
  `<zephyr/drivers/sensor/lsm6dso.h>` (`port/zephyr/include/zephyr/drivers/sensor/lsm6dso.h`, which reads
  `include/lsm6dso.h`). The devicetree binding is Zephyr's `st,lsm6dso`.
- The FIFO read time (`transfer_time_ns` in the statistics) is measured with the CPU clock (SysTick, 8 ns steps).

### NuttX

`nuttx/drivers/lsm6dso/Make.defs` finds the driver's sources (the core, `port/zephyr/`, `common/drivers/stmemsc/`)
through `VPATH`, and the objects are made in `nuttx/drivers`. The shared sources are compiled with the prelude
(`common/compat/port/nuttx/common_prelude.h`). The options are `CONFIG_LSM6DSO_LOCAL` (on by default; needs
`I2C_DW_I2C0`) and `CONFIG_LSM6DSO_STREAM` (always on with `LSM6DSO_LOCAL`) of `nuttx/drivers/lsm6dso/Kconfig`.

| File | Role |
|---|---|
| `port/nuttx/lsm6dso_instance.h` | The device, in place of the Zephyr version's devicetree instance macros (`port/zephyr/lsm6dso_instance.h`): one `DEVICE_DT_DEFINE()` with the board's values (INT1's `.fifo_gpio` is `<&gpio0 24 0>`). At its end `lsm6dso_sensor.c` includes `<lsm6dso_instance.h>`: `port/nuttx/` comes before `port/zephyr/` on the include path, so it is this version's, and `lsm6dso_sensor_init` and `lsm6dso_driver_api` stay `static` |
| `nuttx/drivers/lsm6dso/st_lsm6dso_reg.c` | A one-line wrapper that builds ST's `lsm6dso_reg.c` (`nuttx/stmemsc/`) as it is |
| the board's `src/rp2040_lsm6dso.[ch]` | Makes I2C0 the compatibility layer's bus device (`i2c0`, 1 MHz) and initializes `lsm6dsox` (from `rp2040_bringup()`) |
| `common/compat/port/nuttx/rp2040_gpio_os.c` | GPIO bank 0 for the compatibility layer's `gpio0` (input set-up, levels, edge and level interrupts). Built into the board's `libboard.a` (the board's `src/Make.defs`, through `VPATH`) |

What the driver's calls into the system are on NuttX (those of the core through `port/zephyr/lsm6dso_os_impl.h` and
`lsm6dso_stream_req_impl.h`, those of the binding directly):

| Zephyr | NuttX build |
|---|---|
| `<zephyr/logging/log.h>`, `CONFIG_SENSOR_LOG_LEVEL` (3) | the layer's `<zephyr/logging/log.h>` (`app_log.h`: the same lines in the RAM log), level 3 from the prelude |
| `k_mutex_init/_lock/_unlock` | the layer's `k_mutex` (an `rmutex_t`; a `K_FOREVER` wait ended by a signal is repeated) |
| `k_spin_lock/_unlock` | the layer's `k_spin_lock()` (`sched_lock()` and interrupts off) |
| `k_busy_wait()` | the layer's, `up_udelay()` |
| `k_msleep()`, `k_usleep()` | the layer's, `nxsched_msleep()`, `nxsched_usleep()` (rounded up to ticks plus one tick: Zephyr's rule, the same 100 µs tick) |
| `k_cycle_get_32()`, `k_cyc_to_ns_floor64()` | the layer's: the TIMER's low 32 bits (1 MHz) and ×1000 |
| `i2c_burst_read_dt()` (the core's FIFO reads) | the layer's I2C calls on I2C0 |
| `k_work_delayable` on `k_work_queue_start()`'s queue, `k_work` on `k_sys_work_q` (the stream's work and INT1's) | the layer's work queues (`imu_fifo_worker` priority 108, `sysworkq`) |
| `gpio_*_dt()`, `sensor_clock_get_cycles()`, RTIO (the stream) | the layer's, on the board's `gpio0` and the TIMER |
| `CONFIG_SENSOR_ASYNC_API` | defined by the prelude when `CONFIG_LSM6DSO_STREAM` is set (as in Zephyr, `lsm6dso_sensor.c` does not build without it) |
| `DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()` | the layer's `<zephyr/devicetree.h>`: 1 for `st_lsm6dso` on `i2c` only (the board has one LSM6DSOX, on I2C0) |
| the devicetree instance, `POST_KERNEL`, priority 90 | `DEVICE_DT_DEFINE(lsm6dsox, ...)` (`lsm6dso_instance.h`), `device_init()` in the board's bring-up |

- The board's `rp2040_bringup()` initializes the sensor right after I2C0 and before mounting the key-value store (NVS),
  the Zephyr version's order. A failure only leaves the device not ready (`device_is_ready()` is false, as with
  Zephyr's `do_device_init()`); the boot goes on.
- The FIFO read time (`transfer_time_ns` in the statistics) is measured with the TIMER (1 µs steps).

### Pico SDK (shared by FreeRTOS and ThreadX)

| File | Role |
|---|---|
| `port/pico_sdk/lsm6dso.cmake` | The build rules (`pico_sdk_lsm6dso()`): the same sources as the NuttX build's `CONFIG_LSM6DSO_LOCAL` and `CONFIG_LSM6DSO_STREAM` (the core, the `port/zephyr/` binding, `common/drivers/stmemsc/`, ST's `lsm6dso_reg.c`) and the layer's devices and sensor API. The configuration stops without `CONFIG_I2C_DW_I2C0` |
| `port/pico_sdk/lsm6dso_instance.h` | The device: one, with the values of the Zephyr build's devicetree node `lsm6dsox@6a` (the same values as the NuttX board's `BOARD_LSM6DSOX_*`); before `port/zephyr/` on the include path |
| `port/pico_sdk/lsm6dso_board.c`, `lsm6dso_board.h` | Its start, `lsm6dso_board_initialize()`, on the I2C core's device `i2c0` (after `i2c_dw_i2c0_initialize()`), called by the application (`common/imu/port/pico_sdk/usb_console.c`) where Zephyr does it before `main()`. Without a ready `i2c0` it returns `-ENODEV` without touching the sensor |
| `common/compat/port/pico_sdk/gpio_os_rp2040.c` | GPIO bank 0 under the layer's `gpio0` |

- ST's `lsm6dso_reg.c` is fetched by each OS's `setup.sh` into `freertos/stmemsc/` and `threadx/stmemsc/`
  (`PICO_SDK_STMEMSC_DIR`).
- GPIO (`gpio_os_rp2040.c`): GPIO bank 0 under the layer's `gpio0`, driven with the same Pico SDK calls as Zephyr's
  `gpio_rpi_pico.c`. The interrupt handler is added with `gpio_add_raw_irq_handler_masked()`; only the LSM6DSOX's INT1
  (GPIO24) can interrupt through `gpio0` (other pins get `-ENOTSUP`; the NINA's READY and IRQ line are on the NINA
  binding's handler).

### FreeRTOS

Nothing is the FreeRTOS build's alone (it runs on "Pico SDK" above and the layer's FreeRTOS part).

### ThreadX

Nothing is the ThreadX build's alone (it runs on "Pico SDK" above and the layer's ThreadX part).

## The `sensor` command

`common/shell/sensor_shell.c` is Zephyr's `drivers/sensor/sensor_shell.c` with the local patch
`zephyr/manifest/zephyr/patches/zephyr/0006-sensor-shell-fifo-attributes.patch`, the same file in all four builds
(`zephyr_sensor_shell_patched` checks it). The patch:

- Channels with no frames and completions without a buffer, such as after cancellation, are left out of the output.
- `fifo_watermark` and `fifo_timestamp_decimation` work with `sensor attr_set` and `attr_get`.
- The patch also adds `drivers/sensor/sensor_shell_os.h`, and needs the LSM6DSO driver of `common/drivers/lsm6dso/`
  (its `<zephyr/drivers/sensor/lsm6dso.h>`).

`sensor stream` and `imu start` cannot be used at the same time (the chip has one FIFO).

Only `sensor_shell_os.h`, which `sensor_shell.c` includes first, is each OS's own:

| OS | `sensor_shell_os.h` | Build |
|---|---|---|
| Zephyr | `drivers/sensor/sensor_shell_os.h` of the patched tree (two macros that do nothing) | The original file in the tree, and Zephyr's own `sensor_shell_stream.c` |
| NuttX | `common/shell/port/nuttx/sensor_shell_os.h` (`sensor_shell_nuttx.[ch]`) | `nuttx/apps/sensor` (`sensor_shell.c`, `common/compat/sensor_shell_stream.c`, `sensor_shell_nuttx.c`) |
| FreeRTOS, ThreadX | `common/shell/port/generic/sensor_shell_os.h` (`sensor_shell_os.c`) | `common/shell/port/pico_sdk/sensor_shell.cmake` (`sensor_shell.c`, `common/compat/sensor_shell_stream.c`, `sensor_shell_os.c`) |

In the NuttX, FreeRTOS and ThreadX builds `sensor get` waits for its result to be printed (at most 2 s). Zephyr's
`sensor get` returns at once and the processing thread's (`sensor_shell_tid`) lines come after the prompt, which
Zephyr's shell then draws again below them. The other OSes' line editors do not draw the prompt again after another
task's output, so the command waits until the processing thread has printed the result and returns to the prompt then:
the terminal shows what Zephyr's shows, the result and a prompt below it.

## Tests

The shared sources' tests (`common/tests/run.sh`, run first by the four builds' `host_tests.sh`):

| Test | Covers |
|---|---|
| `fifo_words` | FIFO word parsing |
| `fifo_decoder` | Decoder for FIFO buffers (`port/zephyr/lsm6dso_decoder.c`) |
| `lsm6dso_cal` | Conversion to physical units and the stored calibration format (`lsm6dso_cal.c`) |
| `lsm6dso_zero` | Gyro bias tracking at rest (the `lsm6dso_zero_*()` functions of `lsm6dso_cal.c`) |
| `lsm6dso_stream` | The stream's state machine (`lsm6dso_stream.c`) on the host's part of its boundaries (`common/tests/lsm6dso_os_host/`) and fakes of the core's FIFO calls: the start (no INT1 line, the FIFO and INT1, a level already active, INT1 that cannot be enabled, read or routed), a batch on an edge and its header, nothing below the watermark, the end on an overrun, a cancellation or a request gone, the graceful stop with the last partial batch and its errors, a second request, the watermark's bounds |
| `lsm6dso_core_independent` | Each file of the core (the `.c` files at the top, `include/lsm6dso_cal.h` and `lsm6dso_fifo.h`, `internal/` and the boundaries of `port/`) compiles with the C library, ST's `lsm6dso_reg.h`, its own headers and the host's parts of the boundaries (the two of `common/tests/lsm6dso_os_host/`) only |
| `lsm6dso_public_headers` | The headers of `include/` compile with that directory alone (`lsm6dso.h` with the fakes of the sensor API and RTIO) |
| `zephyr_sensor_shell_patched` | `common/shell/sensor_shell.c` is byte for byte Zephyr's `drivers/sensor/sensor_shell.c` at HEAD with patch 0006. Only when the Zephyr tree is there |

When moving to a newer Zephyr, 0006 is made from `common/shell/sensor_shell.c` (with the new tree's upstream file
merged into it first), so that `zephyr_sensor_shell_patched` passes.

Each OS's tests:

| Test | Covers |
|---|---|
| NuttX: `lsm6dso_driver` | The driver (the core's `lsm6dso.c`, and `port/zephyr/lsm6dso_sensor.c` with `port/nuttx/lsm6dso_instance.h`) and its FIFO stream (`lsm6dso_stream.c`, `lsm6dso_rtio_stream.c`, `lsm6dso_os.c`, `lsm6dso_rtio.c`, the decoder) on ST's real `lsm6dso_reg.c`, the compatibility layer (I2C, devices, work queues, RTIO, `gpio0`, the default sensor iodev and decoder) and the board's devices, against a fake sensor behind a fake I2C master and a fake INT1 (`nuttx/tests/lsm6dso_driver.c`) |
| NuttX: `sensor_shell` | The `sensor` command (`nuttx/tests/sensor_shell.c`) |
| FreeRTOS: `gpio` | GPIO bank 0's boundary, `gpio0`, the FIFO interrupt |
| FreeRTOS: `sensor_shell` | The `sensor` command, two sessions, calibration |

Without ST's `lsm6dso_reg.c` (fetched by `setup.sh`), the tests that use it (`lsm6dso_core_independent`, NuttX's
`lsm6dso_driver` and `sensor_shell`, FreeRTOS's `sensor_shell`) are skipped.
