# NuttX port

English | [日本語](README_ja.md)

This directory holds the port of the IMU measurement firmware in `zephyr/app/usb-console` to
NuttX 13.1.0. The port is in progress. So far there are the setup, the host tests of the
OS-independent common sources, the board definition with its build, and the USB console with the
flashing script: the firmware boots to an nsh prompt on the USB console (CDC ACM) and has none of
the measurement functions yet.

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
- Applies `nuttx/patches/nuttx/*.patch` and `nuttx/patches/apps/*.patch` to `nuttx/nuttx` and
  `nuttx/nuttx-apps`, in name order (see "Patches to NuttX" below). What it applied is recorded in
  the clone's `.git/nano-patches/`, and the same patches are not applied again. When a patch is
  added, removed or changed, the next run takes the recorded patches back in reverse order and
  applies the current ones. If a clone has local changes but no record, it stops without touching
  it; restore the clone with `git -C nuttx/nuttx checkout -- .` (likewise `nuttx-apps`) and source
  the script again.
- Sets `PICO_SDK_PATH` in the current shell and adds the toolchain's `bin`, `nuttx/.venv/bin` and
  `nuttx/script` to `PATH`.

### Patches to NuttX

Patches to NuttX 13.1.0 (`nuttx/patches/nuttx/`) and nuttx-apps (`nuttx/patches/apps/`). Some
make the shell behave like the Zephyr version; others leave out of the build in-tree files that
the board code replaces. The head of each file gives its purpose and the Zephyr code it matches.

| Patch | What it does |
|---|---|
| `nuttx/0001-cdcacm-no-at-protocol` | CDC ACM communication interface protocol 0 instead of 1 (AT commands), as in the Zephyr version; ModemManager does not take the port for a modem |
| `nuttx/0002-cdcacm-drop-output-without-dtr` | Drop the output while no program on the PC has the port open (DTR low) |
| `nuttx/0003-serial-canonical-ctrl-c-eof` | Ctrl-C and Ctrl-D end the read of a command that reads its standard input |
| `nuttx/0004-rp2040-make-defs-alarm-arch-idle-custom` | Leave `rp2040_timerisr.c` (periodic SysTick interrupt) out of the build with `CONFIG_ALARM_ARCH`, and `rp2040_idle.c` with `CONFIG_ARCH_IDLE_CUSTOM` (the board provides the system timer and the idle loop; `ARCH_IDLE_CUSTOM` is not used yet) |
| `apps/0001-readline-echo-in-readline` | readline, not the serial driver, echoes while a line is typed (only what goes into the line, and the line end) |
| `apps/0002-readline-cr-lf-line-end` | CR, LF, CR LF and LF CR are each one Enter |
| `apps/0003-readline-ctrl-c` | Ctrl-C drops the line being typed and shows a new prompt |
| `apps/0004-readline-full-line` | A full line drops further characters instead of being split |

`nuttx/script/kconfig-tweak` stands in for the `kconfig-tweak` that NuttX's `tools/configure.sh`
(through `tools/sethost.sh`) uses to edit `.config`. kconfiglib does not include it, so only the
features NuttX uses are implemented.

The fetched files are not tracked by Git. To change a version, edit the value at the top of the
script, delete the corresponding directory and source the script again.

## Build

```bash
nuttx/script/build.sh
```

This builds the firmware for the custom board `nuttx/boards/arduino-nano-rp2040-connect/` with its
only configuration, `configs/imu/defconfig`, and writes `nuttx/build/nuttx.uf2`. The script
sources `setup.sh` itself, so it also works in a shell where the setup script has not been
sourced. For now the firmware is a minimal one: nsh on the USB console (see "Shell").

The script does the following:

1. Configures NuttX (`tools/configure.sh -l -a ../nuttx-apps`) when it has not been configured
   yet, or when the defconfig, the board's `Kconfig`, a `Kconfig` under `nuttx/drivers/` or
   `nuttx/apps/`, the set of application directories, the patches or the NuttX/apps commit have
   changed since the last configure (recorded in `nuttx/build/.configure-stamp`). NuttX reads the
   board's `Kconfig` and generates `nuttx/apps/Kconfig` only at configure time, so it starts from
   `make distclean`. Otherwise the build is incremental.
2. Runs `make` in `nuttx/nuttx`.
3. Checks the image: `.boot2` is at 0x10000000 and is 256 bytes (without `PICO_SDK_PATH`, NuttX
   would silently leave boot2 out), `nuttx.bin` covers exactly what the ELF loads into flash,
   and the image ends before 0x10ffe000. The last 8 KB of the flash (0x10ffe000–0x10ffffff) is
   kept out of the image for the key-value store, as in the Zephyr version.
4. Converts `nuttx/nuttx/nuttx.bin` (raw image from 0x10000000, boot2 included) to
   `nuttx/build/nuttx.uf2` with `nuttx/script/bin2uf2.py` and prints the path and size. The
   converter checks the boot2 checksum first and writes 256-byte payloads with the RP2040 family
   ID (0xe48bff56). NuttX's own UF2 step (picotool) is turned off.

Any failed step stops the script with an error.

| Option | Effect |
|---|---|
| `--clean` | Start from `make distclean` and configure again |
| `--menuconfig` | Configure when needed, run `make menuconfig`, and stop without building |
| `--savedefconfig` | Run `make savedefconfig` and write the result, with `CONFIG_APPS_DIR`, back to `configs/imu/defconfig` (no build) |
| `-j N` | Parallel make jobs (default: number of CPUs) |
| `-v` | Verbose make output (`V=1`) |

Changes made with `--menuconfig` stay in `nuttx/nuttx/.config` until the next configure; save
them with `--savedefconfig` to keep them. Other outputs: the ELF is `nuttx/nuttx/nuttx` and the
raw image `nuttx/nuttx/nuttx.bin`. Build products, including those NuttX writes into
`nuttx/drivers/` and `nuttx/apps/`, are not tracked by Git.

## Flashing

```bash
nuttx/script/flash.sh            # write nuttx/build/nuttx.uf2
nuttx/script/flash.sh --build    # build first, then write
```

The UF2 goes in through the RP2040 USB bootloader (BOOTSEL mode, in which a USB drive named
`RPI-RP2` appears). The first time, while the board still runs the Zephyr version (or anything
without the `bootsel` command), put it into BOOTSEL by hand: connect it to USB and press the RESET
button twice. If that does not work, connect `REC` and `GND` with a jumper wire, press RESET, and
remove the wire when the drive appears. Then run `flash.sh`. From then on `flash.sh` puts the board
into BOOTSEL itself by sending `bootsel` to the NuttX USB console, and no button is needed.

The script does the following, without sudo:

1. Checks that the UF2 is a UF2 for the RP2040 (magic numbers, family ID 0xe48bff56).
2. Looks for the drive: a partition labelled `RPI-RP2` on a USB disk whose vendor and model are
   those of the RP2040 boot ROM (`RPI`, `RP2`), found with `lsblk`. If there is none, it looks for
   the NuttX USB console (`/dev/serial/by-id/usb-*NuttX_USB_Console*`), sends `bootsel` to it and
   waits for the drive. If there is neither, it explains how to enter BOOTSEL by hand (and says so
   when the Zephyr version's console is present) and waits for the drive. It stops when it finds
   more than one drive or more than one NuttX console, so that it does not write to the wrong
   board. `/dev/ttyACM*` numbers and mount points are never assumed.
3. If the drive is not mounted, waits a few seconds for the desktop to mount it, then tries
   `udisksctl mount` (no sudo needed); if that fails too, it tells you to mount it by hand and
   stops.
4. Checks that `INFO_UF2.TXT` on the drive is the RP2040 boot ROM's (`Model: Raspberry Pi RP2`,
   `Board-ID: RPI-RP2`), copies the UF2 with `cp`, and runs `sync`.
5. Waits for the drive to go away and the NuttX USB console to appear, and prints its
   `/dev/serial/by-id/` path.

| Argument / option | Effect |
|---|---|
| `UF2` | The UF2 to write (default: `nuttx/build/nuttx.uf2`) |
| `--build` | Run `nuttx/script/build.sh` first |
| `--wait SEC` | How long to wait for the drive, and then for the console (default: 60) |
| `--dry-run` | Look for the drive and the console (only `lsblk` and `/dev/serial/by-id/`) and print what would be done; do not build, mount, read the drive, send, copy or wait |

## Shell

The console and shell are nsh on the USB CDC ACM serial port, as in the Zephyr version (where they
are the Zephyr shell). The serial number of the USB device is the chip ID as 16 hexadecimal
digits, the same as the Zephyr version, so the port shows up as
`/dev/serial/by-id/usb-NuttX_NuttX_USB_Console_<chip ID>-if00`. Connect a terminal to it:

```bash
picocom /dev/serial/by-id/usb-NuttX_NuttX_USB_Console_*-if00
```

- The prompt is `nsh$ `. Press Enter to get a new one. The shell starts when the PC configures the
  USB device; unlike NuttX's own USB console, it does not wait for Enter to be pressed three times.
  Closing and reopening the port does not end the shell. Unplugging the cable does; a new shell
  starts when the board is connected again.
- Output is dropped while no program on the PC has the port open (DTR low). Opening the port
  after a reboot or a flash therefore gets no old prompt (it would come back as input: after the
  USB device is enumerated again, the PC's tty has ECHO on until the program turns it off, and
  the first command would fail). Nothing is shown right after opening; Enter (or Ctrl-C) brings
  the first prompt. A terminal that does not raise DTR gets output once it has sent something.
- Input is echoed. Enter may send CR (terminal programs), LF, CR LF (`host/imu_host`) or LF CR;
  each runs the line once (as in the Zephyr version).
- Lines can be up to 511 characters long (as in the Zephyr version); further characters are
  dropped, without echo, until Enter. Up and down recall the last 8 lines, Tab completes command
  names, and Ctrl-A, Ctrl-E, Ctrl-K, Ctrl-U and Ctrl-W edit the line.
- Ctrl-C drops the line being typed, moves to a new line and shows a new prompt (also on an empty
  line; as in the Zephyr version). Ctrl-C pressed while a command runs ends the read of a command
  that reads its standard input (`cat` with no file, for example; Ctrl-D does too); otherwise it
  clears the line at the next prompt. It never stops a running command (neither does the Zephyr
  version). Commands that never end (`while`/`until` loops, `watch`) are left out.
- `help` lists the commands. Most of them are nsh's own (`uname`, `ps`, `free`, `echo`, `sleep`,
  ...); the commands of this port are added as they are ported. An unknown command gives
  `nsh: <name>: command not found`.
- `reboot` restarts the board (the Zephyr version's `kernel reboot`). It also restarts when given
  an argument, `reboot --help` included.
- `bootsel` restarts the board into BOOTSEL (the `RPI-RP2` drive) after its message has been sent.
  With any argument (`bootsel --help`, for example) it only prints its usage. It is a development
  command for `flash.sh`; it can be turned off with `CONFIG_NANO_APP_BOOTSEL`.
- The system log is kept in RAM (`/dev/kmsg`, 4 KB) and is not written to the console, as in the
  Zephyr version. `dmesg` is not available yet.

UART0 (GPIO0 TX, GPIO1 RX, the Arduino D1/D0 pins) is not the console. Its pins are given to the
UART, as in the Zephyr version, but nothing is written to it.

## System clock

The kernel clock is tickless with a 100 µs tick (the same tick as the Zephyr version's
`CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000`, `CONFIG_TICKLESS_KERNEL=y`). There is no periodic
interrupt: ALARM0 of the RP2040 TIMER (64-bit, 1 MHz, from the crystal) is set to the time of the
next wait that ends (`nano_oneshot.c` and `nano_timerisr.c` in
`nuttx/boards/arduino-nano-rp2040-connect/src/`, and NuttX's `drivers/timers/arch_alarm.c`).
ALARM1 to ALARM3 are not used. The settings are `CONFIG_NANO_TICKLESS` (the board `Kconfig`, on by
default), `CONFIG_SCHED_TICKLESS=y` and `CONFIG_USEC_PER_TICK=100`.

- A wait is rounded up to whole ticks and one tick is added (`usleep(1000)` takes 1000 to 1100 µs
  plus the wake-up delay), the same rule as the Zephyr version's `k_sleep(K_MSEC(1))`.
- `clock_gettime()` with `CLOCK_MONOTONIC` or `CLOCK_REALTIME` returns the TIMER value directly, in
  µs (`clock_getres()` gives the tick, 100 µs). The Zephyr version's kernel clock is SysTick on the
  CPU clock, a different source. In both versions the application reads the TIMER directly as its
  time base.
- Threads of the same priority are time-sliced every 20 ms (`CONFIG_RR_INTERVAL=20`; the Zephyr
  version's `CONFIG_TIMESLICE_SIZE=20`).
- `up_udelay()` and `up_mdelay()` wait by reading the TIMER (they also work with interrupts
  disabled).

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
