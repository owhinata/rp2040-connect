# rp2040-connect

English | [日本語](README_ja.md)

A Zephyr workspace for the Arduino Nano RP2040 Connect. Multiple applications share the Zephyr version and modules specified in `manifest/west.yml`. Add modules as needed by your applications.

See the [IMU FIFO driver plan](design/imu-fifo-driver-plan.md) for the design and implementation goals.

## Setup

The following instructions are for Ubuntu 24.04. First, install the packages required for building:

```bash
sudo apt update
sudo apt install --no-install-recommends \
  git cmake ninja-build gperf ccache dfu-util device-tree-compiler \
  wget python3-dev python3-venv python3-tk xz-utils file make gcc \
  gcc-multilib g++-multilib libsdl2-dev libmagic1
```

On AArch64, omit `gcc-multilib` and `g++-multilib`. For other operating systems, see the [official Zephyr instructions](https://docs.zephyrproject.org/latest/develop/getting_started/).

From the root of this repository, source the setup script:

```bash
source script/setup.sh
```

The script prepares `.venv`, the west workspace, modules, Python packages, the Zephyr SDK, and the Arm toolchain as needed. It also sets `ZEPHYR_SDK_INSTALL_DIR` and a `west` alias in the current shell. You do not need to activate the virtual environment. Run `source script/setup.sh` again in each new terminal.

The script checks for west projects, key Python packages, the SDK, and the Arm compiler to determine what is already installed. If a new application requires additional modules, update `manifest/west.yml` and run:

```bash
west update
west packages pip --install
```

## Building applications

`app/blink` is a sample that toggles the board's `led0` every 500 ms. Build it with:

```bash
west build -b arduino_nano_connect app/blink -d build/blink
```

Connect the board over USB, enter bootloader mode, and flash it using the UF2 runner.

If double-pressing RESET does not enter bootloader mode, connect `REC` to `GND` with a jumper while pressing RESET. Remove the jumper when the USB storage device appears.

```bash
west flash -d build/blink -r uf2
```

## USB serial console

`app/usb-console` provides a Zephyr shell over USB CDC ACM. Run `west update` the first time or after updating the manifest, then build and flash:

```bash
west build -b arduino_nano_connect app/usb-console -d build/usb-console
west flash -d build/usb-console -r uf2
```

After flashing, connect with a USB serial terminal. The device name depends on your system; for example, if it is `/dev/ttyACM0`:

```bash
minicom -D /dev/ttyACM0
```

Press Enter to display the shell. Use `help` to list commands and `hello` to display a greeting. Logs are stored in a 4 KiB RAM ring buffer and can be displayed with `dmesg`; old entries are overwritten when the buffer fills. The buffer is stored in RAM that is not initialized at startup so logs can survive a normal reset. They are lost on power failure or a reset that initializes RAM, and unprocessed logs immediately before a reset may not be saved. Timestamps are microseconds since boot and restart from zero after a reset.

Install minicom with `sudo apt install minicom`; exit it with `Ctrl-A` followed by `X`.

The shell enables GPIO, I²C, sensor, reboot, and logging commands. Use `gpio --help`, `i2c --help`, `sensor --help`, `kernel reboot --help`, and `log --help` for details. Use `log` to inspect or change log levels and `dmesg` to display logs stored in RAM.

Run `kernel ps` to display thread CPU and stack usage.

## Measuring the IMU FIFO

`imu_measure` uses the Sensor Streaming API to collect FIFO batches and report counts and transfer times without printing each sample. Set the sampling frequencies of the sensors to use, then specify the measurement duration. For example, to measure both sensors at 208 Hz for 10 seconds:

```text
sensor attr_set lsm6dsox accel_xyz sampling_frequency 208
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 208
imu_measure start 10
```

The results are printed after the measurement stops. Use `imu_measure stop` to stop early and `imu_measure result` to show the last result again. The duration must be between 1 and 3,600 seconds. To measure one sensor, set the other sensor's `sampling_frequency` to `0`. `sensor stream` and `imu_measure` cannot run simultaneously. Use `imu_measure stats` to inspect the current FIFO counters for either kind of stream.

Use `sensor attr_get lsm6dsox all fifo_watermark` to display the FIFO watermark, or `sensor attr_set lsm6dsox all fifo_watermark 128` to set it to 128 words before starting a stream. Values 1–256 are supported; the default is 32 and a reset restores it. A larger watermark generally reduces batch overhead but increases delivery latency and reduces FIFO headroom. DMA support is complete for the IMU FIFO use case and enabled for eligible I²C0 reads in the normal build; `i2c_dma stats` displays cumulative DMA counters. The 60-second validation is sufficient for this use case; competition with other DMA users will be assessed when needed. See the [driver notes](app/usb-console/drivers/README.md) for transfer limitations and measurements.

See the [driver TODOs](app/usb-console/drivers/README.md#todo) for sensor timestamp and internal temperature support.

At the end of the requested duration or an early stop, the driver stops adding new FIFO batches and accounts for the remaining FIFO words and in-flight RTIO completions before finalizing the result. `stop_requested_ms` is the elapsed time when stopping was requested; `elapsed_ms` includes the time to retrieve remaining data and finish cleanup. Batching is stopped for each sensor in turn, so the stop boundary is not exactly simultaneous.

The result includes received accelerometer and gyroscope sample counts, unsupported-tag word counts, configured ODRs, FIFO burst counts and total/maximum transfer times, I²C errors, buffer shortages, FIFO overrun observations, dropped RTIO completions, and stream errors. `cpu_active` is the fraction of the measurement interval spent outside CPU idle, measured using the idle hooks and including interrupts. It includes measurement overhead and has 0.1% output resolution. `cpu_busy` is the scheduler’s non-idle time, including the measurement and other threads. Interrupts that occur while the idle thread is running may be charged to idle and omitted, so it is not the total physical CPU busy time. `observed_hz` is an estimate obtained by dividing received samples by `elapsed_ms`. Setup and final drain time affect it, especially for short measurements. It does not measure the FIFO hardware timestamp or the sensor's exact sample generation time.

`accounting=match` means that the driver's read word and transfer counts match the application's counted words and batches, with no reported errors preventing reconciliation. `mismatch` means the counts differ; `incomplete` means an error, dropped completion, or another condition prevented reconciliation. A match does not prove that every sample generated by the sensor was received.

An overrun observation signals a discontinuity, but does not give the number of lost samples or distinct overruns. The stream stops and reports `stream_error` if it encounters a buffer shortage, I²C error, or overrun. If `cleanup_timeout=1`, cleanup did not complete and a reboot is required before another measurement. Dropped completions or unknown tags also prevent a guarantee of data continuity.
