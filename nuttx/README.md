# NuttX port

English | [日本語](README_ja.md)

This directory holds the port of the IMU measurement firmware in `zephyr/app/usb-console` to
NuttX 13.1.0. The port is in progress; only the setup and the host tests of the OS-independent
common sources are available so far.

## Setup

Everything is prepared under `nuttx/` without sudo. The host needs these tools:

- `git`, `curl` or `wget`, `tar`, `xz`, `sha256sum`
- `python3` with the `venv` module (`python3-venv` on Ubuntu)
- `make` and `gcc` (NuttX builds its host tools)

Source the setup script from the repository root.

```bash
source nuttx/script/setup.sh
```

The script fetches only what is missing and leaves what already exists alone, so it can be
sourced any number of times. Source it again in every new terminal. It fetches the following;
the versions are pinned at the top of the script.

| Location | Contents | Version |
|---|---|---|
| `nuttx/nuttx` | NuttX | tag `nuttx-13.1.0` (shallow clone) |
| `nuttx/nuttx-apps` | NuttX applications | tag `nuttx-13.1.0` (shallow clone) |
| `nuttx/pico-sdk` | Raspberry Pi Pico SDK, used only to build boot2 (the flash second stage) | tag `2.2.0` (shallow clone, no submodules) |
| `nuttx/toolchain/` | Arm GNU Toolchain (arm-none-eabi, for x86_64 Linux) | 13.2.Rel1 (SHA-256 checked) |
| `nuttx/.venv` | Python venv with kconfiglib, used by NuttX's `make menuconfig` and `olddefconfig` | kconfiglib 14.1.0 |
| `nuttx/stmemsc/` | ST's LSM6DSO register driver (`lsm6dso_reg.c`, `lsm6dso_reg.h`) | lsm6dso-pid v3.2.2 (the same content as Zephyr's hal_st; SHA-256 checked) |

It also does the following:

- Creates the symlinks `nuttx/nuttx/external` → `nuttx/drivers` and `nuttx/nuttx-apps/external` →
  `nuttx/apps`. NuttX builds this repository's drivers and applications through them.
- Applies `nuttx/patches/nuttx/*.patch` and `nuttx/patches/apps/*.patch`, if any, to `nuttx/nuttx`
  and `nuttx/nuttx-apps`. Patches already applied are not applied again.
- Sets `PICO_SDK_PATH` in the current shell and adds the toolchain's `bin`, `nuttx/.venv/bin` and
  `nuttx/script` to `PATH`.

`nuttx/script/kconfig-tweak` stands in for the `kconfig-tweak` that NuttX's `tools/configure.sh`
(through `tools/sethost.sh`) uses to edit `.config`. kconfiglib does not include it, so only the
features NuttX uses are implemented.

The fetched files are not tracked by Git. To change a version, edit the value at the top of the
script, delete the corresponding directory and source the script again.

## Tests

```bash
nuttx/script/host_tests.sh
```

This builds and runs the tests of the OS-independent common sources on the PC. It needs neither
the board nor the setup script, only the host's `cc` (C17). The tests check FIFO word parsing,
batches and payloads, the send queue, the status and info JSON, time mapping, the mapping from the sensor
tick to the clock, the NTP estimate, FIFO packet decoding, conversion to physical units and the
stored calibration format, and gyro zero tracking at rest. Each test is built and run in turn,
and the script stops at the first failure. Build products go to a temporary directory that is
removed on exit.

The following files are unchanged copies of the files of the same name in the Zephyr version
(`zephyr/app/usb-console`). When the Zephyr version changes, replace the copies with the same
content.

| Location | Location in the Zephyr version | Contents |
|---|---|---|
| `nuttx/apps/imu/` | `src/` | `imu_raw_{batch,packet,queue,status,time}`, `imu_tick_clock`, `time_offset`, `imu_cal_codec.c`, `imu_cal.h` |
| `nuttx/drivers/lsm6dso/` | `drivers/lsm6dso/` | `lsm6dso_{fifo,cal,zero,fifo_decoder}`, `lsm6dso_fifo_packet.h` |
| `nuttx/tests/` | `tests/` | the tests of the files above |
| `nuttx/tests/fakes/` | `tests/fakes/` | stand-ins for the Zephyr headers the tests need (`zephyr/drivers/sensor.h`) |

Tests of the driver's I2C, FIFO control and stream handling and of NINA are added when those
parts are ported.
