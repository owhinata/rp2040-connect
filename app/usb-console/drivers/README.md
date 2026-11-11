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

The NINA driver is described in the [NINA driver notes](../../../design/nina-driver-ja.md)
(Japanese).

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
the application (section 7 of the [design document](../../../design/imu-raw-mqtt-design-ja.md)).

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
