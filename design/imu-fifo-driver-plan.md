# Implementation plan for LSM6DSOX FIFO burst acquisition and RP2040 I²C DMA

English | [日本語](imu-fifo-driver-plan-ja.md)

## Purpose and starting point

Implement burst acquisition of multiple samples using the FIFO of the onboard IMU (LSM6DSOX) on the Arduino Nano RP2040 Connect, then add I²C DMA support. Evaluate sample loss, transfer time, and CPU usage on hardware at output data rates (ODRs) up to 6,667 Hz.

- Zephyr: `v4.5.0`. Dependencies are managed in `manifest/west.yml`.
- Application: `app/usb-console`. Preserve the existing shell, logging, and Sensor API.
- IMU: I²C0, address `0x6a`. The board definition's node name is `lsm6dsox`.
- Sensor interrupt: GPIO24 in the board definition. The schematic connects it to the IMU's INT1.
- Standard driver: Zephyr's `drivers/sensor/st/lsm6dso`. The board's compatible string is also `st,lsm6dso`. The actual device is an LSM6DSOX, so account for differences.
- I²C driver: Zephyr's `drivers/i2c/i2c_dw.c`.
- Both drivers come from `zephyr/`, which is not tracked by Git in this repository. Keep custom versions in the tracked part of this repository.
- The current `lsm6dso` driver provides the Sensor Fetch/Get API and puts the FIFO in bypass mode.
- The current `CONFIG_I2C_DW_LPSS_DMA` targets Intel LPSS and must not be reused directly for RP2040 DMA. The RP2040 has TX/RX DMA requests for I²C0.

Previous hardware testing set 1 MHz with `i2c speed i2c0 3`, detected the IMU at `0x6a`, and obtained readings with `sensor get lsm6dsox`. However, the cryptographic chip at `0x60`, visible at 100 kHz, was not visible in a 1 MHz scan. The cause is unknown, and stable 1 MHz operation with other bus users has not been established.

## Requirements

**By step 2, the custom LSM6DSOX driver must satisfy Zephyr's Sensor Driver interface.** Do not implement the FIFO solely through application-specific direct register access. Preserve existing `sensor get` and `sensor attr_set` behavior, and consider implementing Zephyr's Read and Decode / Streaming API for the FIFO. Before implementation, check the required `submit`, decoder, and watermark trigger coverage against the Zephyr 4.5.0 API.

Do not let the standard and copied drivers initialize or operate the same IMU or I²C controller simultaneously. I²C0 shell commands, the IMU, and other devices must use the same I²C driver, with configuration changes, concurrent transfers, and mutual exclusion handled explicitly.

## Step 1: Copy existing drivers and verify equivalent behavior

1. Review Zephyr 4.5.0's `lsm6dso` and `i2c_dw` implementations and their dependencies, including related headers, Kconfig, CMake, and any required devicetree bindings. Copy them into this repository.
2. Preserve the original copyright and license notices and record the source version and commit.
3. Provide Kconfig, devicetree, and build configuration so that only the custom version binds to the target devices. Account for the scope of disabling the standard drivers and the effect on other I²C instances.
4. Do not add FIFO or DMA features yet; preserve existing behavior and output.

Completion criteria:

- The build succeeds, and the USB shell works on hardware.
- `i2c scan i2c0`, `i2c speed`, `sensor get lsm6dsox`, and `sensor attr_set ... sampling_frequency ...` behave as before.
- Exactly one driver is registered for each IMU and I²C controller in use.
- Check existing error behavior, including NACK and invalid arguments, as appropriate.

## Step 2: FIFO burst acquisition through the Sensor interface

1. Add handling for LSM6DSOX FIFO mode, sensor ODR, FIFO batch rate, watermark, interrupts, and FIFO tags. Refer to the configuration and FIFO implementation in Linux IIO's `st_lsm6dsx`, while adapting the implementation and interface to Zephyr.
2. Satisfy the Sensor Driver interface while the FIFO is enabled. Define how ordinary Fetch/Get and attribute operations coexist or are excluded. Design multi-sample acquisition to align with Zephyr's Read and Decode / Streaming interface.
3. On a FIFO watermark, read multiple seven-byte words in one combined I²C transfer and use tags to distinguish acceleration from gyroscope data. Delegate processing to a worker or similar mechanism rather than performing a long I²C transfer in the interrupt handler.
4. Distinguish and count FIFO overruns, host buffer shortages, and I²C errors. If the number of lost samples is unknown, report a discontinuity without guessing the count.
5. Consider 1 MHz for I²C. Evaluate it on hardware, including bus-speed and bus-sharing management during acquisition and effects on other devices.

Do not use I²C DMA at this stage. First validate normal I²C burst transfers. Consider accelerometer only, gyroscope only, and both together, starting at low ODRs and increasing to 6,667 Hz. Choose the sample destination, measurement duration, and acceptable loss for the purpose at implementation time. Printing measurement data as text for every sample affects performance measurements.

The uncompressed FIFO holds up to 512 words; each word consists of a one-byte tag and six data bytes. With both sensors at 6,667 Hz, FIFO data alone is approximately 93.3 kB/s, and the data portion including I²C ACK bits is approximately 840 kbit/s. Bus operations and status checks add overhead, so determine on hardware whether lossless continuous acquisition is possible at 1 MHz. Do not confuse a buffer's acquisition timestamp with the sensor's sample generation time.

Completion criteria:

- The existing Sensor interface works, and FIFO batch acquisition goes through the same driver.
- Accelerometer and gyroscope data are identified correctly, and ordinary acquisition is verified after adding FIFO support.
- Record ODR, actual sample counts, FIFO overruns, buffer losses, I²C errors, transfer time, and CPU usage. State explicitly if lossless acquisition cannot be achieved with both sensors at 6,667 Hz.
- Validate start, stop, resume, sensor configuration changes, and recovery from timeouts and bus failures.

## Step 3: RP2040 I²C DMA support

1. Add RP2040 DMA TX/RX support to the copied I²C driver.
2. Design the sequence and DMA synchronization for register-address transmission, repeated START, read requests, reception, and STOP. Preserve Zephyr I²C API semantics with and without DMA.
3. Validate normal and short transfers, concurrent requests, NACKs, timeouts, transfer cancellation and recovery, resource release, and interrupt races. Check RP2040 DMA abort errata as well.
4. Compare performance with DMA enabled and disabled without changing the step 2 IMU driver's Sensor interface.

Completion criteria:

- The I²C shell, IMU, and other I²C users can safely share the same controller.
- Data correctness and loss can be compared with DMA disabled.
- Record I²C transfer time, CPU usage, error counts, and FIFO overruns.
- Evaluate DMA on the understanding that it aims to reduce CPU load and does not itself increase I²C bus speed.

## References

- [Zephyr Sensor Read and Decode](https://docs.zephyrproject.org/latest/hardware/peripherals/sensor/read_and_decode.html)
- [Zephyr 6DoF FIFO streaming sample](https://docs.zephyrproject.org/latest/samples/sensor/6dof_fifo_stream/README.html)
- [Linux IIO st_lsm6dsx FIFO implementation](https://github.com/torvalds/linux/blob/master/drivers/iio/imu/st_lsm6dsx/st_lsm6dsx_buffer.c)
- [ST LSM6DSOX application note AN5272](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3axis-accelerometer-and-3axis-gyroscope-stmicroelectronics.pdf)
- [Arduino Nano RP2040 Connect schematic](https://content.arduino.cc/assets/ABX00053-schematics.pdf)
- [Raspberry Pi Pico SDK hardware API](https://www.raspberrypi.com/documentation/pico-sdk/hardware.html)
