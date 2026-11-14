# NuttX port

English | [日本語](README_ja.md)

This directory holds the port of the IMU measurement firmware in `zephyr/app/usb-console` to
NuttX 13.1.0. The port is in progress. So far there are the setup, the host tests of the
OS-independent common sources, the board definition with its build, the USB console with the
flashing script, the RAM log, the key-value store (`kv`), the SPI link to the NINA-W102
(`nina version`), the NINA's sockets (usrsock: the control side and sending and receiving) and
name resolution, I2C0 (with DMA), the LSM6DSOX driver with its FIFO stream, the IMU
acquisition service (`imu`), the Wi-Fi network device (`wapi`, `ifconfig`), stored Wi-Fi
credentials with automatic reconnection (`wifi cred`), the shell over Wi-Fi (Telnet),
throughput measurement (`iperf`), time synchronization (`ntp`), sending over MQTT, and the
calibration (`imu cal`).

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
| `nuttx/pico-sdk` | The part of the Raspberry Pi Pico SDK that the build of boot2 (the flash second stage) reads: the boot2 source with `pad_checksum` and its include directories, about 4 MB. A full `nuttx/pico-sdk` that is already there is used as it is; remove it and source `setup.sh` again to get the small one | tag `2.2.0` (shallow partial clone with a sparse checkout, no submodules) |
| `nuttx/toolchain/` | Arm GNU Toolchain (arm-none-eabi, for x86_64 Linux) | 13.2.Rel1 (SHA-256 checked) |
| `nuttx/.venv` | Python venv with kconfiglib, used by NuttX's `make menuconfig` and `olddefconfig` | kconfiglib 14.1.0 |
| `nuttx/stmemsc/` | ST's LSM6DSO register driver (`lsm6dso_reg.c`, `lsm6dso_reg.h`) | lsm6dso-pid v3.2.2 (the same content as Zephyr's hal_st; SHA-256 checked) |
| `nuttx/nuttx-apps/system/argtable3/argtable3` | argtable3 (the command line parser of `iperf`). The nuttx-apps Makefile uses a clone found here; without one it downloads the tarball from GitHub on every build and `make distclean` deletes it | tag `v3.2.0.7402e6e` (the default `CONFIG_SYSTEM_ARGTABLE3_VERSION` of nuttx-apps 13.1.0; shallow clone) |

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
| `nuttx/0004-rp2040-make-defs-alarm-arch-idle-custom` | Leave `rp2040_timerisr.c` (periodic SysTick interrupt) out of the build with `CONFIG_ALARM_ARCH`, and `rp2040_idle.c` with `CONFIG_ARCH_IDLE_CUSTOM` (the board provides the system timer and the idle loop) |
| `nuttx/0005-usrsock-dontwait-to-daemon-and-daemon-send-timeout` | usrsock's send and receive keep in the driver's request that the call does not wait (`MSG_DONTWAIT`, a non-blocking socket), and a send the driver answers with `-ETIME` ends with `-EAGAIN`. Stock usrsock repeats a waiting call's `-EAGAIN` for ever, so the Zephyr version's "a waiting TCP send gives `EAGAIN` after 10 s" and "a datagram the NINA did not take gives `EAGAIN` at once" could not be made, and `MSG_DONTWAIT` on a blocking socket waited on the second pass ("Sockets (usrsock)" in `nuttx/drivers/README.md`) |
| `nuttx/0006-telnet-crlf-line-end` | The Telnet driver sends a line feed of the output as CR LF (the Telnet end of line, as the Zephyr version's Telnet shell does) instead of LF CR |
| `apps/0001-readline-echo-in-readline` | readline, not the serial driver, echoes while a line is typed (only what goes into the line, and the line end) |
| `apps/0002-readline-cr-lf-line-end` | CR, LF, CR LF and LF CR are each one Enter |
| `apps/0003-readline-ctrl-c` | Ctrl-C drops the line being typed and shows a new prompt |
| `apps/0004-readline-full-line` | A full line drops further characters instead of being split |
| `apps/0005-iperf-free-argtable` | `iperf` freed only the first entry of its argument table (argtable3) and lost heap on every run; now it frees all of it |
| `apps/0006-iperf-len-option-udp-eagain` | Adds `-l`, the bytes per read or write (as in iperf 2), to `iperf`, and retries a UDP send that fails with `EAGAIN` after a pause, as for `ENOMEM`. The NINA refuses a datagram over 512 bytes with `EMSGSIZE` and one it cannot take with `EAGAIN` (as in the Zephyr version), so stock `iperf`'s UDP send (1472 bytes, giving up on anything but `ENOMEM`) ended at once |

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
- While Wi-Fi is connected, the same shell is also on Telnet (see "Shell over Wi-Fi (Telnet)").
- `help` lists the commands. Most of them are nsh's own (`uname`, `ps`, `free`, `echo`, `sleep`,
  ...); the commands of this port are added as they are ported. An unknown command gives
  `nsh: <name>: command not found`.
- `reboot` restarts the board (the Zephyr version's `kernel reboot`). It also restarts when given
  an argument, `reboot --help` included.
- `bootsel` restarts the board into BOOTSEL (the `RPI-RP2` drive) after its message has been sent.
  With any argument (`bootsel --help`, for example) it only prints its usage. It is a development
  command for `flash.sh`; it can be turned off with `CONFIG_NANO_APP_BOOTSEL`.
- `irqstack` prints the peak use of the interrupt stack since boot in one line (the format of the
  `IRQ 00` line of the Zephyr version's `kernel thread stacks`; see "Thread priorities and
  stacks").
- Wi-Fi is handled with NuttX's own `wapi` and `ifconfig` (see "Wi-Fi (`wapi`, `ifconfig`)"). The
  credentials are stored with the Zephyr version's `wifi cred` (see "Stored Wi-Fi credentials and
  automatic reconnection (`wifi cred`)").
- The log is kept in RAM and is not written to the console, as in the Zephyr version; `dmesg`
  shows it (see "Log").
- The shell (the nsh session) and the commands it starts run at priority 104, the place of the
  Zephyr version's shell thread (see "Thread priorities and stacks").

UART0 (GPIO0 TX, GPIO1 RX, the Arduino D1/D0 pins) is not the console. Its pins are given to the
UART, as in the Zephyr version, but nothing is written to it.

## Log (`dmesg`)

The RAM log of the Zephyr version's `src/log_ram.c` (`nuttx/drivers/log/`).

- An 8192-byte ring buffer in `.noinit` (the Zephyr version's has 4096 bytes; it was made larger
  so that a fatal error report fits from its first line). The startup code does not clear it, so
  its contents survive a reset such as `reboot`. At boot its header (the magic number and its
  inverse, the start and the length) is checked, and the ring is emptied if it is not intact. The
  magic number is the Zephyr version's `0x4c4f4752` XORed with the size (`0x4c4f6752`), so a ring
  of another size left in RAM (the 4096-byte one of earlier builds) is emptied rather than misread.
  Power-off clears it. When it is full, the oldest bytes are overwritten.
- `dmesg` prints what is stored as is, or `No logs.` when it is empty; arguments are ignored. As in
  the Zephyr version it takes 96 bytes at a time, skips what new messages push out while it
  prints, and stops after twice the ring's size (16384 bytes; 8192 in the Zephyr version). NuttX's own `dmesg` (`/dev/kmsg`) is turned off.
- Lines have the format of the Zephyr version's log, `[HH:MM:SS.mmm,uuu] <inf> module: message`,
  ending in LF only. The levels are `<err>`, `<wrn>`, `<inf>` and `<dbg>`; `<dbg>` lines have
  `module.function: `. Hours take at least two digits and more after 99 hours. The time is the
  board's monotonic clock (the TIMER, µs since boot) and starts again from 0 at every reboot (as
  seen in the Zephyr version).
- Output of `syslog()` (kernel and nsh messages, PANIC reports) goes to the same ring. Like the
  Zephyr version's `printk`, it is stored as is, without a prefix, with line ends as CR LF
  (syslog's `CONFIG_SYSLOG_CRLF`, the same as the Zephyr version's printk strings).
- On a fatal error (an exception such as a HardFault, `PANIC()`, a failed assertion, the stack
  sentinel) the NuttX version writes its report (the cause, registers, stack and task list) to the
  RAM log and then restarts by itself (`CONFIG_BOARD_RESET_ON_ASSERT=2`; a normal boot, as
  `reboot`, not BOOTSEL). The RAM log survives the restart, so `dmesg` after it shows the report.
  The report has the cause line (`Assertion failed` with the reason, file and line, and the task
  that stopped), the registers (PC, LR and SP among them), the base and size of the stopped task's
  stack (and of the interrupt stack when in an interrupt), `Stack pointer ... is not within the
  stack` when SP is outside it, and the task list (each stack's use; the `irq` line is the
  interrupt stack). Stack contents (hex dumps) are left out (`CONFIG_SCHED_DUMP_STACK` is off):
  they make the report long enough to push its first lines (the cause and the registers) out of the
  ring. NuttX's `CONFIG_ARCH_STACKDUMP_MAX_LENGTH` only limits the dump of a stack that
  SP is not in, so it does not help here. Checked on the board: after a `PANIC()`, an `__assert()`
  and a HardFault, each in a thread and in an interrupt, the report reads from its first line and
  is followed by the log of the next boot. The Zephyr version stays stopped after a fatal error (`CONFIG_RESET_ON_FATAL_ERROR` is
  off). This is a difference from the Zephyr version; the same behaviour is to be added to the
  Zephyr version after the port is finished.
- Sources use it the Zephyr way: `#include "app_log.h"` (`nuttx/drivers/include/`; usable from
  the kernel side, the board code and the applications, also in interrupt handlers), then per file
  `LOG_MODULE_REGISTER(name, LOG_LEVEL_INF)` (without a level: `CONFIG_NANO_LOG_DEFAULT_LEVEL`,
  default 3, or `LOG_LEVEL` if defined before the include), and `LOG_ERR()`, `LOG_WRN()`,
  `LOG_INF()`, `LOG_DBG()`, `LOG_ERR_RATELIMIT()` and so on in the code. Messages above the file's
  level are dropped at compile time; there is no run-time level control (Zephyr's `log` command).
  The header replaces `<syslog.h>`'s `LOG_ERR` (a syslog priority) with the macro of the same name,
  so a file that includes `app_log.h` uses `APP_LOG_SYSLOG_ERR` instead in `syslog(LOG_ERR, ...)`.
- A line is formatted in the calling thread (or interrupt), in a 192-byte buffer on its stack,
  and then written to the ring with interrupts disabled. A longer message is cut to fit 192
  bytes. In the Zephyr version's default (deferred) mode the caller only records the message, and
  a low-priority logging thread formats it later.

## Key-value store (`kv`)

Settings and calibration values are kept in the last 8 KiB of the flash (`0x10ffe000` to
`0x10ffffff`, two 4 KiB erase sectors), where the Zephyr version keeps its `storage_partition`. The
API and the command are those of the Zephyr version's `src/kv_store.[ch]`; the stored format is not
compatible with the Zephyr version's.

- Area: set by the board's `include/board.h` (`BOARD_KV_OFFSET`, `BOARD_KV_SIZE`) and the linker
  script (`__kv_start`, `__kv_end` in `scripts/nano-flash.ld`; the area is outside the image's
  flash region, and `build.sh` checks where the image ends). `src/nano_flash_kv.c` is an MTD that
  shows only this range. Before every erase and program it checks that the target lies inside the
  range, against both the constants and the linker script's values (after checking at boot that
  the two agree and that the image ends before the area), and only then touches the flash. A
  request outside the range does nothing and fails.
- Format: NuttX's NVS (`drivers/mtd/mtd_config_nvs.c`, `CONFIG_MTD_CONFIG_NVS`,
  `CONFIG_MTD_CONFIG_NAMED`, names up to 63 characters) as `/dev/config`, with the Zephyr version's
  names `app_kv/<key>`. The Wi-Fi credentials are in the same `/dev/config`, under the names of
  the Zephyr version's settings, `wifi_cred/<index>` (`kv` cannot see or delete them; see "Stored
  Wi-Fi credentials and automatic reconnection (`wifi cred`)"). The MTD reports a 1-byte program
  unit (so that entries are packed) and programs the flash in 256-byte pages, with 0xff outside
  the data.
- First boot after the Zephyr version: at boot, an area that is neither in this format nor fully
  erased (the Zephyr version's data is still there) is erased before it is used, and `dmesg` shows
  `<wrn> fs_nvs: Erasing the key-value area: not in this build's format`. A normal boot shows only
  `<inf> fs_nvs: 2 Sectors of 4096 bytes` (the first of the Zephyr version's `fs_nvs` lines at
  boot; the write positions of its second and third lines are not shown). Values are not carried
  over: back them up and write them back as below. What the Zephyr version does when it is flashed
  back after the NuttX port has not been checked.
- Command (priority 104, stack 3072; subcommands are handled as in "Subcommands of the commands"
  below):

  ```
  kv set <key> <text>        store the text as is
  kv sethex <key> <hex>      store 1 to 128 bytes given as hex pairs (upper case allowed)
  kv get <key>               print the value in lower-case hex
  kv delete <key>            delete (Deleted even for a missing key)
  ```

  Keys are 1 to 32 characters of letters, digits, `_`, `-` and `.`; values are 1 to 128 bytes.
  Keys starting with `sys.` give `Reserved key`. The output is the Zephyr version's (`Saved`,
  `Deleted`, `KV write failed: -22`, `KV read failed: -2`,
  `Expected 1-128 bytes as hexadecimal pairs`, `Invalid hexadecimal value` and so on). `kv` alone or
  with an unknown subcommand prints the help, and a wrong number of arguments prints
  `<subcommand>: wrong parameter count` and the subcommand's help (the Zephyr shell's text).
  Differences from the Zephyr version: nsh parses the arguments (quote `$`, `;`, `#`, `>` and the
  like), error lines go to standard error (the same console), and the exit status is 0 on success
  and 1 on failure.
- Capacity: all live entries must fit in one erase sector (4 KiB; the other one is kept for
  compaction). An entry takes "key length + 8 (`app_kv/` and the terminator) + value length + 16"
  bytes, and 4048 bytes are usable: with 32-character keys and 128-byte values, 184 bytes each, up
  to 22 entries (checked on the board: the write after the area is full gives
  `KV write failed: -28`). The Zephyr version (settings on NVS, the same 4 KiB × 2) stores a name and
  a value per entry (183 bytes for the same entry), about the same. Storing the value that is
  already stored writes nothing. A stored Wi-Fi network takes 12 + 224 + 16 = 252 bytes (the name
  `wifi_cred/<index>` with its terminator, `struct wifi_credentials_personal`, the index entry),
  in the same area, for up to 2 networks.
- Updates and compaction: an update appends the new entry and marks the old one; when an erase
  sector is full, the live entries are copied to the other sector and the full one is erased
  (NVS garbage collection). XIP stops while the flash is erased or programmed, so this runs from
  RAM with interrupts disabled, for each 4 KiB erase and each 256-byte page program. Updating one entry normally takes 4 page programs. The Zephyr version's flash driver also
  disables interrupts per erase call (4 KiB) and per write call (an entry's data, up to 2 pages).
- Power loss: NVS writes the data before its CRC-checked table entry, and marks the old entry only
  after the new one is written; at boot it repairs a half-written entry or an interrupted
  compaction. This has not been tested by cutting the power.
- API (`nuttx/apps/imu/kv_store.[ch]`; the header is the Zephyr version's): `kv_store_init()`,
  `set`, `get` and `delete` take the Zephyr version's arguments and return its values (`-EINVAL`
  for a bad key or value, `-ENOENT` for a missing key, `-ENOBUFS` for a buffer that is too small,
  `-ENOSPC` when full), and `-ENODEV` when there is no `/dev/config`. Every call opens and closes
  `/dev/config`, and the driver's mutex (with priority inheritance) serializes them, so any thread
  can call them. The board sets up `/dev/config` at boot, and init calls `kv_store_init()` where
  the Zephyr version's `main()` does (on failure: `Could not initialize settings: <value>`).

### Subcommands of the commands

The port's commands with subcommands (`kv`, `nina`, ...) give the Zephyr shell's output through a
shared part, `nuttx/apps/subcmd/` (`subcmd_main()`). Zephyr's `SHELL_CMD()` (no check of the
number of arguments) is `SUBCMD()`, and `SHELL_CMD_ARG()` (mandatory and optional counts) is
`SUBCMD_ARG()`.

- The command alone, or an unknown word: the help of the deepest matching command
  (`<name> - <help>`, `Subcommands:`, one line per subcommand).
- A wrong number of arguments: `<subcommand>: wrong parameter count` (on standard error) and the
  subcommand's help.
- A `*` or `?` in the word right after a subcommand that matches none of its subcommands ends the
  words there, as in the Zephyr shell (`kv get a*` is a wrong number of arguments, `nina version *`
  is `nina version`). When it does match subcommands (the Zephyr shell expands it to their names),
  this is not reproduced: the word is taken as an unknown word.
- `-h` and `--help` are ordinary words (as in the Zephyr version).
- Help texts are laid out as the Zephyr shell lays them out (`formatted_text_print()` of
  `shell_help.c`): a new line at each newline of the text and before a word that would pass column
  80, the next line starting at the column where the text began (the Zephyr shell moves the cursor
  there with a VT100 sequence; the port prints spaces). A newline at the end of a text leaves one
  empty line, as in Zephyr. There is no `resize` command to change the terminal width.
- The exit status is 0 on success and 1 on failure, help or a wrong number of arguments.

`sensor`, whose command table is a copy of Zephyr's, runs on `zshell.c` (`z_shell_cmd_main()`) of
the same part. The rules are the ones above; in addition, as in the Zephyr shell, it handles
dynamic subcommands (device names, channel names and so on), the `SHELL_HELP()` description with
its `Usage:` line, and the expansion of a wildcard that matches subcommands (see "Sensor command
(`sensor`)").

### Moving the values from the Zephyr version

```bash
# Back up while the Zephyr version runs (host/recordings/kv-backup.json, not in Git)
host/.venv/bin/python script/kv_transfer.py backup --usb
# Write them back after flashing the NuttX port, and register Wi-Fi (the password is typed at the
# terminal; the SSID defaults to rp2040-test, --ssid for another access point)
host/.venv/bin/python script/kv_transfer.py restore --usb
# Without registering Wi-Fi
host/.venv/bin/python script/kv_transfer.py restore --usb --no-wifi
```

`restore` writes with `kv sethex` and checks by reading back with `kv get` (`imu.session` is not
written back). For Wi-Fi it sends `wifi cred add -s <SSID> -k 1 -p <password>` (which prints
nothing on success, as in the Zephyr version) and `wifi cred auto_connect`. The credentials cannot
be carried over from the Zephyr version's flash (the stored formats differ), so the password is
typed then. With 2 networks already stored, `wifi cred add` fails with
`Failed to add network credentials, err: -105` (delete one with `wifi cred delete` first). After calibration values it sends `imu cal reload` and checks that the keys written back are
used (`calibrated`), as with the Zephyr version (see "Calibration (`imu cal`)").

## NINA-W102 (`nina`)

The Wi-Fi module NINA-W102 is reached over SPI1. Its firmware is the Zephyr version's (Arduino's NINA
firmware 3.0.2 with fixes, `firmware/nina-fw/README.md`) and is not changed. So far there is the
transfer layer that sends one command and receives its answer (`nina_request()`,
`nina_ip_config()`), `nina version`, the sockets (NuttX's usrsock: socket, close, bind, listen,
accept, connect, setsockopt, sending and receiving, `FIONREAD`), the NINA's name resolution
(`nina_getaddrinfo()`), and the Wi-Fi network device `wlan0` (scan, connect, disconnect, status,
link monitor) ("Sockets (usrsock)", "Name resolution" and "Wi-Fi" in `nuttx/drivers/README.md`).
Stored credentials and automatic reconnection (`wifi cred`) are in their own section below.

```
nsh$ nina version
NINA firmware: 3.0.2
```

- `nina version` prints what the Zephyr version prints. On failure it prints
  `NINA version request failed: <value>` on standard error (the same console) and exits with 1.
  `nina` alone or with an unknown word prints the help. As in the Zephyr version, `version` does
  not check the number of arguments, so extra words are ignored (priority 104, stack 3072).
- Wiring (`BOARD_NINA_*` in the board's `include/board.h`, as in Zephyr's board definition
  `arduino_nano_connect.dts`):

  | Signal | GPIO | Setting |
  |---|---|---|
  | SCK | 14 | SPI1. No pulls, 4 mA, slow slew, no Schmitt trigger |
  | MOSI (TX) | 11 | Same |
  | MISO (RX) | 8 | Same, input enabled |
  | CS | 9 | GPIO output, active low, no pulls. Inactive (high) from boot |
  | READY | 10 | GPIO input, active low, pull-up |
  | RESET | 3 | GPIO output, active low, no pulls |
  | IRQ (the NINA's GPIO0) | 2 | Pull-down. Driven high during the reset, then an input |

- SPI: mode 0 (CPOL 0, CPHA 0), 8 bits, MSB first, `CONFIG_NANO_NINA_SPI_FREQUENCY` (default
  8000000; the Zephyr version's `CONFIG_NINA_SPI_FREQUENCY`). The PL022 divides clk_peri (125 MHz)
  by an even number, so the clock is 7.8125 MHz (the same in the Zephyr version). NuttX's SPI1
  driver (`rp2040_spi.c`) is used by polling (the Zephyr version uses neither interrupts nor DMA).
  `CONFIG_RP2040_SPI_DMA` is off (with it, transfers over 4 bytes use DMA, whose completion is
  waited for without a time limit). The transfer layer, not the SPI driver, drives CS as a GPIO,
  and keeps it asserted while waiting for the answer. There is no `/dev/spi1`
  (`CONFIG_RP2040_SPI_DRIVER` is off).
- As in the Zephyr version, the NINA is set up on the first request: CS inactive, READY an input,
  IRQ high, RESET active; 10 ms later the reset is released, and after 750 ms IRQ is driven low and
  made an input again. The first request is the one that reads the MAC address when the start-up
  code sets up `wlan0` (at boot, as in the Zephyr version), so the USB console appears after these
  waits.
- The steps and values of a request are those of the Zephyr version's `drivers/nina/nina_wifi.c`
  (`nuttx/drivers/nina/nina_spi.c`). READY is sampled every 100 µs (rounded up to ticks plus one
  tick, so every 100 to 200 µs), with `-ETIMEDOUT` after 2000 ms before sending, 20 ms for the
  NINA to answer CS by releasing READY, and 10000 ms for the answer to be ready. Up to 1000 bytes
  are read looking for the start of the answer (0xE0). A recursive mutex (with priority
  inheritance) serializes the requests, and the SPI runs at the priority of the calling thread.

### Wi-Fi (`wapi`, `ifconfig`)

The NINA is the NuttX network device `wlan0`. Instead of the Zephyr version's `wifi` commands,
NuttX's own `wapi` (`CONFIG_WIRELESS_WAPI_CMDTOOL`, priority 104, stack 3072) and `ifconfig` are
used. The driver's work is the Zephyr version's `drivers/nina/nina_net.c` itself ("Wi-Fi" in
`nuttx/drivers/README.md`).

| Zephyr version | NuttX port |
|---|---|
| `wifi connect -s <SSID> -p <passphrase> -k 1` | `wapi psk wlan0 <passphrase> 3`, then `wapi essid wlan0 <SSID> 1` |
| `wifi connect -s <SSID> -k 0` (no encryption) | `wapi essid wlan0 <SSID> 1` (with no `wapi psk` just before) |
| `wifi disconnect` | `wapi disconnect wlan0` |
| `wifi scan` | `wapi scan wlan0` |
| `wifi status` | `wapi show wlan0` and `ifconfig wlan0` (below) |
| `net iface` | `ifconfig` |
| `wifi cred add/delete/list/auto_connect` | The same (`wifi cred`; see "Stored Wi-Fi credentials and automatic reconnection (`wifi cred`)" below) |

- In `wapi psk`, 3 is the CCMP cipher and the version defaults to WPA2: the Zephyr version's `-k 1`
  (WPA2-PSK). What `wapi psk` sets is used by the next `wapi essid` only and then forgotten, the
  passphrase wiped. As in the Zephyr version the driver takes only no encryption and WPA2-PSK (the
  NINA's passphrase command); anything else is `ENOTSUP`. Passphrases are 8 to 63 characters.
- Like the Zephyr version's `wifi connect`, `wapi essid` returns once the request is made. The
  connection steps and limit (every 500 ms, up to 60 times, so `ETIMEDOUT` after about 30
  seconds), turning power saving off and taking the IP configuration after association are the
  Zephyr version's. Check the result with `ifconfig wlan0` (`RUNNING` appears when connected, and
  `inet addr`, `DRaddr` and `Mask` show the NINA's DHCP values).
- Where the items of `wifi status` are: State is `RUNNING` in `ifconfig` (and the ESSID Flag in
  `wapi show`), SSID is ESSID in `wapi show`, BSSID is AP, RSSI is Sense (while connected only),
  Interface Mode is Mode (`WAPI_MODE_MANAGED`). As the Zephyr version's `wifi status`, `wapi show`
  asks the NINA for the SSID, BSSID and RSSI at that moment. Band, Channel, Security, MFP, Link
  Mode, Beacon Interval, DTIM, TWT and the PHY rate are not shown (the Zephyr version does not get
  them from the NINA either; they are fixed values or 0 there).
- `wapi scan` lists the NINA's scan (up to 10 networks) in the NINA's order, with the columns BSSID,
  frequency, signal level (dBm), encryption (`8000` none, `0800` encrypted, `ffff` unknown) and
  SSID. Encryption comes from the Zephyr version's mapping of the NINA's types (7 none, 5 WEP, 2
  WPA, 4 WPA2, others unknown); `wapi` does not tell WEP, WPA and WPA2 apart. The frequency column
  is in MHz (`iperf` selects `CONFIG_LIBC_FLOATINGPOINT`, so `printf` handles floating point;
  without it the column shows `*float*`). Scans for an ESSID, passive scans and scans of given channels
  are refused, as in the Zephyr version. A scan while connected is treated as a disconnection (the
  IP configuration is cleared) and asks for a reconnection when it ends, as in the Zephyr version.
- A connection during a scan and a scan during a connection attempt fail with `EBUSY`, as in the
  Zephyr version.
- `wapi` prints nothing when it fails (its error messages go through NuttX's debug output
  `wlerr()`, which this configuration leaves out). Its exit status is 1, so `echo $?` tells. The
  Zephyr version's `wifi` commands print the error value.
- `ifconfig wlan0 down` and `up` do nothing (the Zephyr version's interface is always up as well).

### Stored Wi-Fi credentials and automatic reconnection (`wifi cred`)

The credentials of the networks to join are stored, and the board joins again by itself at boot
and when the connection is lost. The credentials are a copy of Zephyr's library
(`subsys/net/lib/wifi_credentials/` with its settings backend), the command a copy of
`wifi_credentials_shell.c`, and the reconnection a copy of the Zephyr version's
`src/wifi_profile.c` with only the calls into the system replaced (`nuttx/apps/wifi/`). Syntax,
output, error messages and return values are the Zephyr version's.

```
wifi cred add -s <SSID> [-k <type>] [-p <passphrase>] [-c <channel>] [-b <band>] [-w <MFP>] [-m <BSSID>]
                                     store (replaces the same SSID; prints nothing on success)
wifi cred delete <SSID>              delete (prints Deleting network ssid: ...)
wifi cred list                       list by stored index (with the passphrase, as in the Zephyr version)
wifi cred auto_connect               ask for a connection with the stored networks (does not wait)
```

```text
nsh$ wifi cred add -s example-ap -k 1 -p example-pass
nsh$ wifi cred list
  network ssid: "example-ap", ssid_len: 10, type: WPA2-PSK, password: "example-pass", password_len: 12, MFP_OPTIONAL
```

- Number and lengths: up to 2 networks (`CONFIG_NANO_WIFI_CREDENTIALS_MAX_ENTRIES`, the Zephyr
  version's `CONFIG_WIFI_CREDENTIALS_MAX_ENTRIES=2`). A third gives
  `Failed to add network credentials, err: -105` (`ENOBUFS`), and `dmesg` shows
  `<err> wifi_credentials: Cannot store WiFi credentials, no space left` (as in the Zephyr
  version). SSIDs are 1 to 32 characters (`SSID too long (max 32 characters)`,
  `SSID not provided`), passphrases 8 to 64 (`Passphrase should be minimum 8 characters`,
  `Passphrase too long (max 64 characters)`); the NINA takes passphrases up to 63 characters only
  (as in the Zephyr version).
- Type (`-k`): Zephyr's number (0 none, 1 WPA2-PSK, 2 WPA2-PSK-SHA256, 3 WPA3-SAE and so on; see the
  help of `-h`) is stored as it is. As in the Zephyr version only 0 and 1 can connect; stored
  networks of other types are skipped by the automatic connection. A passphrase without `-k` is
  stored with the warning `Passphrase provided without security configuration`. `-t` gives
  `Invalid option`, as in the Zephyr version (it is in the option string but not handled).
- `wifi cred`, `wifi` alone, or an unknown word print the help; `add` with other than 1 to 13
  words prints `add: wrong parameter count` and the help (see "Subcommands of the commands"). The
  only subcommand of `wifi` is `cred` (the others are `wapi` and `ifconfig`).
- Options are parsed by NuttX's `getopt_long()`, which, unlike Zephyr's `sys_getopt_long()`, does
  not move the words that are not options to the end (options after such a word are not read),
  and does not log the line the Zephyr version logs for an unknown option (`<wrn> sys_getopt`).
- Storage: each network is written at once, on every `add` and `delete`, to `wifi_cred/<index>` in
  `/dev/config` (the lowest free index), as Zephyr's `struct wifi_credentials_personal`. At boot,
  init reads the stored SSIDs (where the Zephyr library's `SYS_INIT()` runs, before the threads
  start). The keys of `kv` start with `app_kv/`, so `kv` can neither read nor delete the
  credentials (`wifi_cred/0` contains `/`: `KV read failed: -22`). As with the KV, the store is not
  compatible with the Zephyr version's.
- Automatic reconnection (thread `wifi_reconnect_tid`, Zephyr priority 8 → 110, 2048-byte stack):
  when networks are stored, at boot (`wifi_profile_init()`) and when the connection is lost (the
  link monitor finds it gone, or after a scan while connected), it waits 4 seconds, asks for a
  connection with the stored networks (`NET_REQUEST_WIFI_CONNECT_STORED`: in stored order, until
  one request is accepted), then checks every second and asks again after 32 seconds without an
  address. After `wapi disconnect wlan0` (a disconnection asked for) it does not reconnect until
  the next successful connection (as in the Zephyr version). With nothing stored it does nothing.
  A refused request (other than `EBUSY` and `EALREADY`) logs
  `<wrn> wifi_profile: Stored Wi-Fi request failed: <value>`; a failed connection itself
  (`ETIMEDOUT` after about 30 seconds) is not logged (nor in the Zephyr version).
- `wifi cred auto_connect` only asks for a connection from the shell and does not start the
  automatic reconnection (a successful connection does). When the request is refused it prints
  `An error occurred when trying to auto-connect to a network. err: <value>` (the exit status is
  always 0, as in the Zephyr version).
- The migration of the Zephyr version's old profile formats (the KV key `sys.wifi_profile`, the
  settings item `wifi_last/last`) runs at boot as in the Zephyr version, and finds nothing: they
  are never in the port's store.
- The passphrase is shown by `wifi cred list` (as in the Zephyr version) and nowhere else: not in
  the log, not in error messages.

## Shell over Wi-Fi (Telnet)

As with the Zephyr version's Telnet shell (`CONFIG_SHELL_BACKEND_TELNET`), the shell of the USB
console is also on TCP port 23 while Wi-Fi is connected (`nuttx/apps/init/nano_telnet.c`,
`CONFIG_NANO_INIT_TELNET`; the port is `CONFIG_NANO_INIT_TELNET_PORT`, the Zephyr version's
`CONFIG_SHELL_TELNET_PORT`). There is no authentication or encryption, and anyone who can reach
the port can do everything the shell can, so use it on trusted networks only.

```bash
telnet <the board's IPv4 address>
```

- Listening: the start-up (init) opens the listening socket before the `USB console started` log
  line, the place where the Zephyr version's Telnet backend opens it (a `SYS_INIT()` before
  `main()`); the log says `<inf> shell_telnet: Telnet shell backend initialized`. As in the Zephyr
  version it listens on all addresses (`INADDR_ANY`) with a backlog of 1 and no `SO_REUSEADDR`.
  The NINA holds the socket, so a socket opened before Wi-Fi is connected starts accepting once it
  is, and keeps listening across Wi-Fi disconnections and reconnections (as in the Zephyr
  version). If it cannot be opened, the Zephyr version's messages are logged (`Unable to start
  telnet on NET_AF_INET (<value>)` and others) and there is no Telnet.
- One client at a time. A second client that connects while one is connected is accepted and
  closed at once, and `<err> shell_telnet: Telnet client already connected.` is logged; the first
  one keeps working (as in the Zephyr version). The `telnetd` task accepts the clients, at
  priority 104 with a 2048-byte stack (those of the Zephyr version's `net_socket_service` thread,
  which does this there). When an accept fails, it logs `Telnet accept error (<value>)` and waits
  `CONFIG_NANO_INIT_RETRY_MS` (100 ms) before accepting again, so that a lasting failure does not
  become a busy loop (the Zephyr version does not wait).
- A session is the same nsh as on USB (the same commands and output, the prompt `nsh$ `, no
  greeting). It is a task of its own and runs at the same time as, and independently of, the USB
  shell (like the Zephyr version's two backends), at priority 104 with a 3072-byte stack (as the
  USB session; the Zephyr version's `shell_telnet` thread priority 14 and
  `CONFIG_SHELL_STACK_SIZE`). `ps` shows it as `nsh_telnet`. NuttX's Telnet driver
  (`CONFIG_NETDEV_TELNET`, `CONFIG_TELNET_MAXLCLIENTS=1`) turns the accepted connection into the
  terminal `/dev/telnet0`, which the session uses as its standard input and output. NuttX's own
  Telnet daemon (`netutils/telnetd`, `NSH_TELNET`) is not used: it ends, listening socket and all,
  on a second client (a session over `TELNET_MAXLCLIENTS`) and when `SO_REUSEADDR` fails (the
  NINA's sockets give `ENOPROTOOPT`), and its sessions print a greeting.
- The prompt arrives right after connecting. The Zephyr version sends nothing on a connection (the
  output before it is dropped, and the prompt comes after an Enter). `host/imu_host` reads and
  drops whatever arrives after connecting before it sends a command, so it works the same with
  both.
- Echo and line editing: as on USB, readline echoes what is typed, and line editing, history, Tab
  and Ctrl-C (drop the line being typed and show a new prompt) work. NuttX's Telnet driver sends
  `IAC WILL ECHO` while a line is read and `IAC WONT ECHO` while a command runs, switching the
  Telnet client's local echo. The Zephyr version's Telnet shell turns echo off on a connection and
  negotiates no options; with echo off, the Zephyr shell also ignores the line editing, history,
  Tab and Ctrl-C keys, which leaves echo and editing to the client's local echo. `host/imu_host`
  drops the echo and the Telnet commands, so the results are the same.
- Line ends: the output's line end is CR LF (as in the Zephyr version; patch `nuttx/0006`). An
  Enter may be CR, LF, CR LF or CR NUL and runs the line once, as on USB.
- Disconnection: a session ends when the client closes the connection or the connection fails,
  and the next client is accepted. When Wi-Fi drops, the NINA closes the connection and the
  session ends. The PC side gets an RST on its first send after Wi-Fi is back. A `reboot` of the
  board resets the NINA too, so no disconnection reaches the PC.
- A command that reads its standard input (such as `cat` without arguments) does not end with
  Ctrl-C or Ctrl-D on Telnet (patch `nuttx/0003` is for the serial driver, not for the Telnet
  driver); closing the connection ends it. A command started in the background with `&` inherits
  the session's terminal, so until it ends, the next client after the connection is closed is
  refused like a second one.
- In a Telnet session a command's output goes on the command task's stack down to the SPI
  transfer to the NINA (on USB it only goes into the CDC ACM buffer). The port's commands
  therefore have the stack of the Zephyr version's shell, 3072 bytes (`dmesg` and `irqstack` were
  raised from 1024).

## Throughput measurement (`iperf`)

Instead of the Zephyr version's `zperf` (iperf 3 protocol), the port uses NuttX's `iperf`
(`netutils/iperf` in nuttx-apps, iperf 2 protocol). On the PC, use `iperf` (iperf 2), not
`iperf3`. The default port is 5001 (the Zephyr version uses 5201).

| Zephyr version | NuttX port | PC (with the NuttX port) |
|---|---|---|
| `zperf tcp upload <PC> 5201 30 1024` | `iperf -c <PC> -t 30 -l 1024` | `iperf -s` |
| `zperf udp upload <PC> 5201 30 500 500K` | `iperf -c <PC> -u -t 30 -l 500` (no rate setting; it sends as fast as it can) | `iperf -s -u` |
| `zperf tcp download 5201` | `iperf -s -B 0.0.0.0` | `iperf -c <board> -t 30` |
| `zperf tcp download stop` | The TCP server ends by itself after one connection | – |
| – | `iperf -s -u -B 0.0.0.0 -l 512` (UDP server; stop it with `iperf -a` from another shell) | `iperf -c <board> -u -l 500 -b 500K` |

- Main options: `-c <host>` (client), `-s` (server), `-u` (UDP), `-p <port>`, `-t <s>` (client
  time, default 30), `-i <s>` (report interval, default 3), `-l <bytes>` (per read or write;
  default 1472 for UDP, 16384 for TCP), `-B <address>` (server bind address), `-a` (tell a running
  `iperf` to stop).
- Give a server `-B 0.0.0.0`. Without it, it binds to the address of `wlan0`, and the NINA does not
  accept a bind to a given address (`EADDRNOTAVAIL`, `error code: 99`, as in the Zephyr version).
- UDP datagrams are at most 512 bytes (the NINA; as in the Zephyr version). With the default 1472
  bytes the send ends with `EMSGSIZE` (`err=90`) and the receive drops what is past 512 bytes, so
  give `-l 512` or less. `-l`, and waiting and sending again when the NINA cannot take a datagram
  (`EAGAIN`), come from patch `apps/0006`.
- The TCP server ends after one connection (NuttX's `iperf` does so). It does not return while no
  client comes (it waits in `accept()`, which `iperf -a` does not interrupt). The UDP server does
  not end; stop it with `iperf -a` from another shell (Telnet or USB, whichever is free). It prints
  `udp server recv error` at every 10 s receive timeout and ends at the first one after `-a`.
- Threads: the command `iperf` (priority 104, 3072-byte stack) and its pthreads `iperf_traffic`
  (priority 100, 4096) and `iperf_report` (101, 4096). The two pthreads' priorities and stacks are
  fixed in `iperf`'s source (the Zephyr version's `zperf_work_q` has priority 15, which Zephyr
  lowers to the lowest application priority 14, here 104). The read and write buffer comes from
  the heap (16 KiB for TCP).
- `iperf` selects floating point in `printf` (`CONFIG_LIBC_FLOATINGPOINT`) and argtable3
  (`CONFIG_SYSTEM_ARGTABLE3`).

## I2C0 (`i2c`)

I2C0 carries the LSM6DSOX (0x6A) and the ATECC608A (0x60). The driver is the board's
`src/nano_i2c.c`, a copy of the controller part of the DesignWare I2C driver that the Zephyr
version uses (`zephyr/app/usb-console/drivers/i2c_dw/i2c_dw.c`). NuttX's `rp2040_i2c.c`
(`CONFIG_RP2040_I2C`) is not used. It is registered as `/dev/i2c0` for NuttX's `i2c` tool. Reads
that meet the conditions run on DMA (see "DMA reads (`i2c_dma`)" below).

```
nsh$ i2c get -a 6a -r 0f
READ Bus: 0 Addr: 6a Subaddr: 0f Value: 6c
```

- Wiring (`BOARD_I2C0_*` in the board's `include/board.h`): SDA on GPIO12, SCL on GPIO13. As the
  Zephyr version's pinctrl sets them: the I2C function, no internal pulls, 4 mA, slow slew, Schmitt
  trigger on, input enabled. The I2C0 block is reset at boot before it is set up.
- The speed comes with each transfer, from the messages' frequency, and only 100000, 400000 and
  1000000 are taken (Zephyr's speed classes; other values give `-EINVAL`). The registers get the
  Zephyr version's values: at 100 kHz `IC_SS_SCL_HCNT`/`LCNT` 500/625, at 400 kHz
  `IC_FS_SCL_HCNT`/`LCNT` 93/109, at 1 MHz 31/31 (both with the FAST speed in `IC_CON`),
  `IC_FS_SPKLEN` 1, `IC_SDA_HOLD` at its reset value 1. `IC_CON` is 0x363 at 100 kHz and 0x365
  otherwise (with the bit that holds the bus while the RX FIFO is full; as in the Zephyr version it
  is set only while DMA is on, so after `i2c_dma mode off` transfers use 0x163 and 0x165).
- Transfers that do not use DMA are interrupt driven. A read queues as many requests as the RX FIFO (16 entries) has
  room for and takes the data 4 bytes at a time. A transfer has 1 s; after that it is aborted and
  returns `-ETIMEDOUT`. A target that does not answer (NACK) and other failures give `-EIO` and
  leave the Zephyr version's `<err> i2c_dw: NACK on i2c@40044000` and the like in `dmesg` (rate
  limited). The next transfer works as usual. A bus lock serializes transfers.
- A transfer has one address and one frequency for all its messages. 10-bit addresses are not
  supported. The last message ends with a STOP even with `I2C_M_NOSTOP` (as in the Zephyr version).

### DMA reads (`i2c_dma`)

As the Zephyr version's `CONFIG_I2C_DW_RP2040_DMA` (here `CONFIG_NANO_I2C0_DMA`, on by default), a
read that meets all of the following runs on DMA; other reads and all writes are interrupt driven.

- It is the last message of the transfer (so it ends with STOP), 32 to 3584 bytes long, to a 7-bit
  address, with DMA on (`i2c_dma mode on`, the state at boot).
- It takes two DMA channels (TX sends the command words, RX takes the data). When a channel cannot be
  had or configured, the read goes to the interrupt path before anything starts, and
  `dma_fallbacks` counts it.
- Time limit and error values are those of the interrupt path (`-ETIMEDOUT` after 1 s, `-EIO` for a
  NACK and the like). After a failure the DMA and the I2C are stopped before the call returns. If
  the DMA channels do not stop within 100 ms, I2C0 is quarantined
  (`<err> i2c_dw: DMA channels did not stop; I2C controller quarantined`) and every later transfer
  gives `-EIO` until a reboot. The steps are in the "I2C0" section of `nuttx/drivers/README.md`.
- NuttX's DMA layer (`CONFIG_RP2040_DMAC`) is off; only the board's `src/nano_dma.c` (which behaves as
  Zephyr's RP2040 DMA driver) uses the DMA.

```text
i2c_dma mode          show the mode (i2c_dma=on or i2c_dma=off)
i2c_dma mode on|off   switch (only while no stream or transfer runs)
i2c_dma stats         DMA transfers, bytes, errors, fallbacks to interrupts (since boot)
```

The output is the Zephyr version's (`dma_transfers=0 dma_bytes=0 dma_errors=0 dma_fallbacks=0`;
anything but `on` or `off` gives `Expected on or off`). As in the Zephyr version, switching only
sets the value: it neither waits for nor stops a transfer in progress. "Only while no stream or
transfer runs" is a rule for the user; the command does not check it. Priority 104, stack 3072.

Checked on the board (1 MHz): 1000 DMA reads of 256 bytes in a row give no error, and neither do
500 each from two tasks at once (the bus lock runs them one at a time); DMA and interrupt reads of
the same range give the same bytes except in registers that change by themselves (outputs, status,
time stamp).

### The `i2c` tool and the Zephyr version's `i2c` command

The Zephyr version's `i2c` command (Zephyr's shell) is not ported; NuttX's `i2c` tool
(`CONFIG_SYSTEM_I2CTOOL`) takes its place. Its options (`-a` address, `-r` register, `-f`
frequency and so on) stay until changed. The default frequency is 100000 (the Zephyr version's
speed at boot). Addresses, registers and values to write are hexadecimal; the byte count of `dump`
is decimal.

| Zephyr version | NuttX port | Differences |
|---|---|---|
| `i2c scan i2c@40044000` | `i2c dev 03 77` | Probes differently (the Zephyr version writes 1 byte, the NuttX port reads 1 byte). Both see 0x60 and 0x6a. Do not add `-z` (a 0-byte write cannot be sent; each address times out after 1 s) |
| `i2c read_byte i2c@40044000 6a 0f` | `i2c get -a 6a -r 0f` | |
| `i2c read i2c@40044000 6a 10 10` | `i2c dump -a 6a -r 10 16` | Up to 256 bytes (the Zephyr version: 64). The Zephyr version's byte count is hexadecimal |
| `i2c direct_read i2c@40044000 6a 1` | `i2c get -a 6a` | Without `-r` no register is sent |
| `i2c write_byte i2c@40044000 6a 07 5a` | `i2c set -a 6a -r 07 5a` | |
| `i2c write i2c@40044000 6a 07 5a 00` | none | The NuttX tool writes one value at a time (two bytes with `-w 16`) |
| `i2c speed i2c@40044000 3` | `-f 1000000` (kept for later commands) | The Zephyr version changes the bus setting, and `imu start` sets 1 MHz there; the NuttX tool only changes its own setting |
| `i2c recover i2c@40044000` | `i2c reset` | Both only take and release the bus (the board has no recovery routine) |

`i2c verf` writes and reads back, so it changes the sensor's settings. The `i2c` tool runs at
priority 100 (the Zephyr version's shell commands are at the place of 104).

## LSM6DSOX (IMU)

The driver is `nuttx/drivers/lsm6dso/lsm6dso.c`, a copy of the Zephyr version's
`drivers/lsm6dso/lsm6dso.c` (Zephyr's LSM6DSO driver with the FIFO, calibration and zero-rate
changes) with only the calls into the system replaced. The registers are handled by ST's
`lsm6dso_reg.c` (lsm6dso-pid v3.2.2, fetched into `nuttx/stmemsc/` by `script/setup.sh`; with LF
line ends identical to the one in Zephyr's hal_st). The FIFO stream (the INT1 interrupt and RTIO)
runs the Zephyr version's files as they are, too (below). The acquisition service that uses it
(`imu start` and so on) is the next section.

- **Initialization at boot**: the board's bring-up (`nano_bringup()`) initializes the sensor right
  after I2C0 and before mounting the key-value store (NVS). That is the Zephyr version's order: it
  initializes its devices before `main()` (`POST_KERNEL`, priority 90) and mounts NVS in `main()`.
  The first two `dmesg` lines are the Zephyr version's:

  ```text
  [00:00:00.007,154] <inf> LSM6DSO: Initialize device lsm6dsox@6a
  [00:00:00.008,475] <inf> LSM6DSO: chip id 0x6c
  [00:00:00.028,920] <inf> fs_nvs: 2 Sectors of 4096 bytes
  ```

  On failure (a wrong chip ID, an I2C error) the same `<err>` lines as in the Zephyr version are
  logged, the device stays "not ready" and the boot goes on.
- **The initialization steps and values** are the Zephyr version's: select the user register bank
  → `WHO_AM_I` → disable I3C → if the sensor hub master runs, stop it and wait 300 µs → software
  reset and 100 µs → accelerometer power mode, full scale, ODR → gyro power mode, full scale, ODR →
  FIFO bypass → BDU. The settings are those of the Zephyr version's devicetree (with
  `app.overlay`), kept in the board's `include/board.h` as `BOARD_LSM6DSOX_*` (accelerometer ±8 g,
  gyro ±2000 dps, both ODRs off, high-performance mode). After it, `CTRL1_XL`=0x0c,
  `CTRL2_G`=0x0c, `CTRL3_C`=0x44, `CTRL9_XL`=0xe2, and `FIFO_CTRL1`..`4`, `COUNTER_BDR_REG1`/`2`,
  `INT1_CTRL`, `INT2_CTRL`, `CTRL4_C`..`CTRL8_XL` and `CTRL10_C` are 0 (see
  `i2c dump -a 6a -r 00 32`).
- **I2C**: the speed at boot is the devicetree's 100 kHz; `i2c_configure()` (a copy of Zephyr's
  API) changes it to 400 kHz or 1 MHz (the Zephyr version's `imu start` sets 1 MHz). A NuttX I2C
  transfer carries its frequency in each message, so the I2C0 device (`nuttx/drivers/compat`)
  keeps this speed and the driver's transfers use it. The `i2c` tool runs at its own `-f`
  frequency and does not change the driver's speed (the Zephyr version's `i2c speed` changes the
  bus). A register read is one transfer (write the register number → RESTART → read), a write is
  one message with STOP: the shape of Zephyr's `i2c_burst_read_dt()` and `i2c_write_dt()`. FIFO
  reads are 32 bytes or more and use DMA.
- **Single readings** (`sensor_sample_fetch()`, `sensor_channel_get()`), **attributes** (ODR and
  full scale through `sensor_attr_set()`), the functions that start, read and stop the FIFO,
  reading the sensor's timestamp, the stored calibration and the on-demand gyro zero
  (`lsm6dso_cal_zero()`) behave as in the Zephyr version. The ODR becomes the lowest step at or
  above the request (12, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz); the accelerometer full
  scale is m/s² rounded to g and only 2, 4, 8 or 16 g; the gyro full scale is rad/s rounded to
  dps and only 125, 250, 500, 1000 or 2000 dps (anything else is `-EINVAL`). While the FIFO is in
  use, single readings and attribute changes are `-EBUSY`.
- **FIFO stream**: `lsm6dso_stream.c` and `lsm6dso_fifo_irq.[ch]` are unchanged copies of the
  Zephyr version's files. As there, a stream starts with `sensor_stream()`; a rising edge of INT1
  (GPIO24) → the system work queue (`sysworkq`) → the FIFO worker (`imu_fifo_worker`) reads the
  FIFO and completes an RTIO request. Below the watermark it looks again every 20 ms;
  `lsm6dso_fifo_request_stop()` delivers the rest as a last partial batch and ends with
  `-ECANCELED`. The Zephyr kernel parts it needs (work queues, semaphores, `k_poll()`), RTIO, GPIO
  and the sensors' async API are in the compatibility layer (`nuttx/drivers/compat/`); the board's
  bring-up starts `sysworkq` and `RTIO WQ 0` with `zephyr_compat_initialize()`. Checked on the
  board (1 MHz, 6667 Hz, watermark 128, the RTIO sizes of `imu_stream.c`; with a temporary test
  command at T11 whose consumer only takes and releases the completions) in 10 s and 60 s runs: no
  errors, overruns, `i2c_dma` errors or fallbacks.
- The copies of Zephyr's API for the callers (`struct device`, `struct sensor_value`, the channel
  and attribute enumerations, `sensor_*()`, part of the I2C API, the kernel, RTIO, GPIO) are in
  `nuttx/drivers/compat/`. See the "LSM6DSOX" and "Zephyr API compatibility layer" sections of
  `nuttx/drivers/README.md`.

## Sensor command (`sensor`)

`nuttx/apps/sensor/` is a copy of the Zephyr version's sensor shell
(`zephyr/app/usb-console/drivers/sensor_shell/sensor_shell.c`: Zephyr v4.5.0's
`drivers/sensor/sensor_shell.c` with its fixes and the LSM6DSO's FIFO attributes) and of Zephyr's
`drivers/sensor/sensor_shell_stream.c` and `sensor_shell.h`. Side by side in `diff`, the only
differences are:

- both `.c` files: a leading comment and the include of `sensor_shell_nuttx.h` (the values of the
  Zephyr build's `CONFIG_SENSOR_SHELL_*`, and the log through `app_log.h`);
- `sensor_shell.c`: two lines with which `sensor get` waits for its result to be printed (see
  "Where the output goes" below), and the format of the data ready trigger's log line (`PRIu32`,
  `PRId32`);
- `sensor_shell_stream.c`: one line, the callback of the processing thread, which is the original
  callback followed by telling a waiting `sensor get` (`sensor_shell_nuttx.c`);
- `sensor_shell.h` is unchanged.

The command table (`SHELL_STATIC_SUBCMD_SET_CREATE()`, `SHELL_DYNAMIC_CMD_CREATE()`,
`SHELL_CMD_REGISTER()`) is copied too. The compatibility layer's `<zephyr/shell/shell.h>` and
`nuttx/apps/subcmd/zshell.c` make it the builtin command `sensor` with the Zephyr shell's rules
(matching the words, dynamic subcommands, wildcard expansion, the argument count check, and the
`SHELL_HELP()` description with its `Usage:` line and their wrapping).

```text
nsh$ sensor
sensor - Sensor commands
Subcommands:
  get       : Get sensor data.
              Channel names are optional. All channels are read when no channels
              are provided.
              Usage: get <device_name> [<channel name 0> .. <channel name N>]
  attr_set  : Set the sensor's channel attribute.
              Usage: attr_set <device_name> <channel_name> <attribute_name> <value>
  ...
nsh$ sensor get lsm6dsox accel_xyz
channel type=0(accel_x) index=0 shift=4 num_samples=1 value=<time>ns (<m/s²>)
...
channel type=3(accel_xyz) index=0 shift=4 num_samples=1 value=<time>ns, (<x>, <y>, <z>)
```

- The subcommands are `get`, `attr_set`, `attr_get`, `stream` and `trig`. The Zephyr build has no
  `CONFIG_SENSOR_INFO`, so there is no `info`: `sensor info` prints the help of `sensor` (as in the
  Zephyr version). The syntax, the output formats (`channel type=... index=... shift=...
  num_samples=... value=...ns (...)`, `%.6f` in `attr_get`) and the error messages are the Zephyr
  version's. Error numbers are NuttX's (for example `ENOTSUP` is 138 here, 134 in the Zephyr
  version).
- Device names: `get`, `attr_set`, `attr_get` and `trig` take the node name `lsm6dsox@6a` or the
  node label `lsm6dsox`. `stream` takes the node name `lsm6dsox@6a` only (`device_get_binding()`,
  as in the Zephyr version). The device listed in the help is `lsm6dsox@6a` too. Wildcards (`*`,
  `?`) expand to channel names only after the node name (for example
  `sensor get lsm6dsox@6a gyro_?`; as in the Zephyr version).
- The attributes are `fifo_watermark` (1 to 256, default 32), `fifo_timestamp_decimation` (0, 1, 8
  or 32, default 8), and `sampling_frequency` and `full_scale` (see "LSM6DSOX (IMU)"). They are
  refused with `-16` (`-EBUSY`) while a stream runs (`sensor stream`, `imu start`). `imu start`
  overwrites the watermark with the KV value (default 128). After boot the ODR is off, so
  `sensor stream` ends with `Read failed: -22` until `sampling_frequency` is set (as in the Zephyr
  version).
- It cannot run together with `imu start`: `imu start` during a stream says
  `The IMU FIFO is in use (sensor stream?)`, and `sensor stream` and `sensor get` during a
  measurement say `Read failed: -16` (the driver and `imu_stream.c` are copies of the Zephyr
  version's).
- The calibration applies to the physical values of `sensor get` and `sensor stream`. During
  `sensor stream` the gyro bias is updated at rest by itself (`tracked at rest` in `imu cal`).
- `trig` gives `Error while setting trigger 1 on device lsm6dsox (-38)` (`-ENOSYS`): as in the Zephyr
  build, the driver has no trigger (`CONFIG_LSM6DSO_TRIGGER_NONE`).
- Tab completes command names only (nsh). The Zephyr version also completes subcommands, device,
  channel and attribute names.

**Where the output goes**: in the Zephyr version, the results of `sensor get` and `sensor stream`
are written by the processing thread (`sensor_shell_processing_tid`, priority 0, 3072-byte stack)
to the shell that ran the command (USB or Telnet). Here the same thread (priority 118, 3072-byte
stack, started by init at boot) writes them.

- It writes to a shell backend of the compatibility layer (`zshell.c`): one per terminal that a
  command's stdout is (`/dev/ttyACM0` for USB, `/dev/telnet0` for Telnet). For each line the
  processing thread opens that terminal, writes the line with one `write()` and closes it again.
  The command's own output goes to its stdout and stderr.
- `sensor get` returns to the prompt once the processing thread has written the result (it waits at
  most 2 s). The Zephyr version prints the result after the prompt and draws the prompt again;
  nsh cannot draw its prompt again, hence the wait.
- The output of `sensor stream` goes on after the prompt has come back. nsh does not draw the
  prompt or the line being typed again, so the output comes in the middle of what is typed (the
  Zephyr version draws the prompt and the line again after each line). `sensor stream
  lsm6dsox@6a off`, from any terminal, stops it (so does `reboot`).
- When the Telnet client goes, the output is dropped until the next client connects (there is no
  terminal device to open) and then goes to that client (as with the Zephyr version's Telnet
  backend). The processing thread has the terminal open only while it writes, so it does not hold
  up the next connection (no `Telnet client already connected.`).
- While the PC has the USB port closed, the output is dropped like all other output (see "Shell").

## IMU acquisition (`imu`)

The acquisition service is `nuttx/apps/imu/imu_stream.c`, a copy of the Zephyr version's
`src/imu_stream.c` with only the calls into the system replaced, by the compatibility layer
(`nuttx/drivers/compat/`, which keeps Zephyr's names). Constants, branches, order, waits, counters
and output text are the Zephyr version's; side by side with `diff`, the only differences are the
includes at the top (NuttX's headers first) and the command registration at the end (the Zephyr
shell's `SHELL_CMD_REGISTER()` made the builtin command `imu` on `nuttx/apps/subcmd`).

```text
nsh$ imu
imu - IMU RAW streaming
Subcommands:
  start   : Start streaming to the MQTT server (settings: kv imu.*)
  stop    : Stop streaming
  cal     : Show the conversion used by sensor get/stream: [reload|zero]
  status  : Show counters and errors of the current or last run
```

- `imu start`, `imu stop` and `imu status` have the Zephyr version's syntax and output (`imu` and
  `imu status` while idle are character for character the output recorded from the Zephyr
  version; the items of `imu status` have the same order and columns). The settings come from the
  same KV keys (`imu.host`, `imu.port`, `imu.device`, `imu.odr`, `imu.watermark`).
- Acquisition runs in the thread `imu_stream_tid` (Zephyr priority 9 → 109, a static 2048-byte
  stack). Zephyr starts `K_THREAD_DEFINE()` threads before `main()`; here init starts it at the
  beginning of the part that does what `main()` does (`K_THREAD_DEFINE_START()`). The `imu`
  command itself is a builtin at the shell's priority (104).
- The session ID is the chip ID in its first 8 bytes (the USB serial number; the bytes of Zephyr's
  `hwinfo_get_device_id()`), then the KV value `imu.session` (4 bytes, big-endian, incremented and
  stored by every `imu start`), then zeros. The Zephyr version's counter is not carried over; the
  NuttX port starts at 1.
- init calls, in the order of the Zephyr version's `main()`, `kv_store_init()` →
  `imu_cal_init()` (on failure `Could not set up the IMU calibration: <value>`) →
  `time_sync_init()` → `wifi_profile_init()`.

Checked on the board (transport stub, no network, 6667 Hz, watermark 128, 1 MHz) with `imu start`
→ 10 s → `imu stop` four times (one right after flashing) and 60 s once: every item of `errors` and
`fifo`, every anomaly count of `time`, and the `i2c_dma` errors and fallbacks are 0 (before the
MQTT transport was added).

### Calibration (`imu cal`)

`nuttx/apps/imu/imu_cal.c` is a copy of the Zephyr version's `src/imu_cal.c`; side by side in
`diff`, the only differences are the leading comment and the include of `<nuttx/config.h>` (the
value format, `imu_cal_codec.c` and `imu_cal.h`, are unchanged copies). The KV keys
(`imu.cal.accel.<ug/LSB>`, `imu.cal.gyro.<udps/LSB>`) and values (56 bytes), using only the key of
the current sensitivity, and the syntax, output and error messages of `imu cal`,
`imu cal reload` and `imu cal zero` are the Zephyr version's (the Zephyr version's README,
"Applying a calibration", and the design document, section 10).

```text
nsh$ imu cal
conversion:
  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)
  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)
  gyro bias: 7.65 -4.25 -0.81 LSB (stored, 0 rejected)
```

- init calls `imu_cal_init()` after `kv_store_init()`, which gives the driver the function that
  looks up the calibration for a sensitivity (where the Zephyr version's `main()` does). The
  driver reads the KV store when it is given the function, on `imu cal reload`, and when a range
  is set.
- The calibration applies to the driver's physical values only (single reads,
  `sensor_channel_get()`, and the FIFO decoder), **not to the RAW sent over MQTT** (as in the
  Zephyr version). `sensor get` and `sensor stream` show it (see "Sensor command (`sensor`)").
- `imu cal zero` is refused during a measurement with
  `A measurement is running; it tracks the gyro bias itself`. `imu cal` and `imu cal reload` work
  during a measurement. A gyro bias taken again is kept in RAM only.
- Errors go to stderr and the exit status is 1 (as with `kv`; 0 on success). With two or more
  arguments it prints `cal: wrong parameter count` and the help (as the Zephyr shell does).

Checked on the board: the loading at boot, every subcommand of `imu cal` and the refusal during a
measurement (USB and Telnet), the switch to `nominal` on a range change and back (with a
temporary test command), the check `script/kv_transfer.py restore` makes on the reply to
`imu cal reload` (the same code run without writing to the KV store), and
`host/.venv/bin/imu-host write` (writing the same values; Telnet, and USB with the device given:
the automatic USB detection of `imu-host` does not know the NuttX port yet). The output of
`imu cal` right after boot is character for character the output recorded from the Zephyr
version.

### MQTT transport

`nuttx/apps/imu/imu_raw_mqtt.c` is a copy of the Zephyr version's `src/imu_raw_mqtt.c`; side by
side in `diff`, the only differences are the leading comment and the includes of NuttX's headers
(read before the compatibility layer's). It is built with `CONFIG_NANO_IMU_RAW_MQTT` (on by
default); without it the stub `imu_raw_mqtt_null.c` stands in (it drops and counts every batch, as
the Zephyr version does without a network). The topics (`imu/<device>/raw`, `status`, `info`)
and QoS (0 for raw and status, 1 with retain for info), the client ID (`imu-<device>`), the
keep-alive (60 s), no clean session, the order of sending (on every connection the info, the
status, then the queued batches; the status every second and when the state changes, input
checked every 100 ms, a 5 ms sleep while the queue is empty), the reconnection interval (1 s),
the CONNACK wait (5 s), the wait for the first connection in `imu start` (up to 3 s), dropping and
counting the batches while disconnected, not resending a batch whose publish failed, the up to
0.5 s that `imu stop` gives the last status, and the items of `mqtt` in `imu status` with the
events that advance them are the same as in the Zephyr version.

- Thread `imu_raw_mqtt_tid` (Zephyr priority 11 → 107, a static 2048-byte stack). init starts it
  at the start of its start-up work; it waits for `imu start`.
- The MQTT client is a copy of the part of Zephyr's MQTT library (`subsys/net/lib/mqtt/`) that is
  used (`nuttx/apps/imu/mqtt_min.c`, with the API of the compatibility layer's
  `<zephyr/net/mqtt.h>`): MQTT 3.1.1 over TCP only, with the bytes it sends, the defaults and the
  way it reads in pieces of Zephyr's library (the host test `mqtt_min` runs the same test on
  Zephyr's own library as well). A PUBLISH goes as its fixed header and its body in two vectors
  of one `sendmsg()`, which the NINA driver joins into one send command (as in the Zephyr
  version).
- The socket stays blocking (as in the Zephyr version). A connection waits up to 10 s, a send
  that finds no room up to 10 s, before failing (the NINA driver's deadlines; "Sockets (usrsock)"
  in `nuttx/drivers/README.md`).
- The socket is made with the compatibility layer's `zsock_socket()`, which marks it as asked
  about by its callers only (`NINA_SO_CALLER_POLLS`). `nina_event` does not query it except to check
  the connection; only `imu_raw_mqtt_tid` asks the NINA about it, as in the Zephyr version ("Sockets
  (usrsock)" in `nuttx/drivers/README.md`).
- The input check (`zsock_poll()`, every 100 ms) and the CONNACK wait (`zsock_poll()`, 5000 ms) ask
  the NINA in the calling thread (one `CMD_POLL`) and, with nothing there, again every 20 ms, as the
  Zephyr version's NINA `poll()` does. A failed connection that the NINA reports (POLL_ERROR) is
  `ZSOCK_POLLERR`, as in the Zephyr version. A read that does not wait (`zsock_recv()` with
  `ZSOCK_MSG_DONTWAIT`) asks first with `ioctl(FIONREAD)`. The commands the NINA receives (a poll,
  and when something is there a poll and a read) are those of the Zephyr version.
- Difference from the Zephyr version: the `lock_wait_*` and `publish_*` times are the 125 MHz
  SysTick cycles (`k_cycle_get_*()`, which fall behind while streaming) in microseconds in the
  Zephyr version, and the TIMER (1 µs) here (the compatibility layer's `k_cycle_get_*()`).

Checked on the board (6667 Hz, watermark 128, 1 MHz, `host/.venv/bin/imu-host check --port 18830`
on the PC): in the first measurement after flashing, in 60 s runs and in a 10-minute run the
receiver's missing batches (`missing_batches`) and time regressions (`unix_regressions`) and the
`errors` and `fifo` of `imu status` are 0; with Telnet connecting and disconnecting while
sending, with the receiver dropping the connection (`--drop-at`) and stopping reading
(`--stall-at`), and with the board's Wi-Fi disconnected (`wapi disconnect wlan0` → `wifi cred
auto_connect`), the board's `published` and `dropped_batches` match what the receiver got and
missed (except a few batches written into a failing connection just before it broke, as in the
Zephyr version), acquisition goes on, and the board reconnects.

## Time synchronization (`ntp`)

`nuttx/apps/imu/time_sync.c` is a copy of the Zephyr version's `src/time_sync.c`. Only the calls
into the system (sockets, `net_mgmt` events, the realtime clock; the compatibility layer provides
them under Zephyr's names), the body of `time_sync_mono_us()` (the board's `nano_mono_us()`, the
same read), the Kconfig names and the command registration at the end are replaced. The NTP
packet and the checks of the reply, where t1 (right before the send) and t4 (right after the
reply is received) are read, the estimate (`time_offset.c`, an unchanged copy: the reply with the
shortest round trip of the last 8 is used), the estimate's expiry after 60 s without a reply, the
clock that follows it at up to ±500 ppm without going back, moving the realtime clock only
forward, the query interval, wait and retry, the counters and the output of `ntp status` are the
Zephyr version's. It is built with `CONFIG_NANO_TIME_SYNC` (on by default); without it the stub
`time_sync_null.c` stands in (only `time_sync_mono_us()` is real; no synchronization).

```text
nsh$ ntp
ntp - Network time synchronization
Subcommands:
  status  : Show time synchronization status
```

- Thread `time_sync_tid` (Zephyr priority 9 → 109, 2048-byte stack), started by init at the start
  of its work. A successful Wi-Fi connection (`NET_EVENT_WIFI_CONNECT_RESULT`) starts the queries
  and a disconnection (`NET_EVENT_WIFI_DISCONNECT_RESULT`) stops them. Every connection starts a
  new estimate (it may be another network). Nothing happens while Wi-Fi is not connected.
- The server is `CONFIG_NANO_NINA_NTP_SERVER` (the Zephyr version's `CONFIG_NINA_NTP_SERVER`;
  empty by default: the default gateway), the port `CONFIG_NANO_NINA_NTP_PORT` (123), the interval
  `CONFIG_NANO_NINA_NTP_INTERVAL_MS` (2000 ms). A reply is awaited for 250 ms; none is counted in
  `timeouts`, one that fails the checks in `rejected`, and the next query follows at the next
  interval. When there is no address, no socket or the send fails, the error goes to `last_error`
  and the thread tries again after 5 seconds.
- Waiting for the reply: the Zephyr version calls `recv(MSG_DONTWAIT)` about every millisecond,
  and its NINA driver asks the module for the socket's state each time (`CMD_POLL`) and reads what
  has arrived. NuttX's usrsock answers a receive that does not wait from the readable mark it holds
  (`RECVFROM_AVAIL`), so the compatibility layer's `zsock_recv()` with `ZSOCK_MSG_DONTWAIT` first
  calls `ioctl(FIONREAD)` (one `CMD_POLL` in the calling thread) and reads with `recv(MSG_DONTWAIT)`
  only when something is there (the driver uses that poll's answer and does not poll again). The
  module sees the Zephyr version's sequence: a poll, a read when something has arrived, about 1 ms
  of wait, again.
- The socket is made with the compatibility layer's `zsock_socket()`, so it is marked as asked
  about by its callers only (`NINA_SO_CALLER_POLLS`) and `nina_event` does not query it. As in the
  Zephyr version, the NINA is asked only when a receive is called (see "Sockets (usrsock)" in
  `nuttx/drivers/README.md`).
- Difference from the Zephyr version: `k_uptime_get()` (the 250 ms wait, the 60 s check,
  `last_reply_ms_ago`) comes from the TIMER in the NuttX version. The Zephyr version's is the
  SysTick kernel clock, which falls behind while streaming.
- The realtime clock (`CLOCK_REALTIME`) is set to the estimate only when it is 20 ms or more behind
  (counted in `realtime_clock_steps`); when it is ahead, it is left alone. It is good for dates and
  times of day; precise times come from `time_sync_unix_us()`. nsh's `date` shows and sets the
  realtime clock (the Zephyr version's `date`; `CONFIG_NSH_DISABLE_DATE` is cleared, for the Zephyr
  version's `CONFIG_DATE_SHELL=y`). It shows UTC, in nsh's format:

  | Zephyr version | NuttX version |
  |---|---|
  | `date get` | `date` (e.g. `Sat, Nov 14 03:23:03 2026`) |
  | `date set 2026-11-14 03:23:03` | `date -s "Nov 14 03:23:03 2026"` |
- Checked on the board: after flashing and after `reboot`, synchronization comes with the first
  reply once Wi-Fi is up; `ntp status` prints the items of the Zephyr version's recorded output in
  the same order and widths; after `wapi disconnect wlan0` the queries stop and the state becomes
  `not synchronized` 60 s after the last reply; `wifi cred auto_connect` reconnects, starts a new
  estimate and synchronizes again; the realtime clock only moves forward; with a measurement
  (`imu start`) and a Telnet session running at the same time, the `time` items of `imu status`
  show synchronized values.

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
- The board's monotonic clock is `nano_mono_us()` (`<arch/board/nano_timer.h>`). It reads the
  TIMER's TIMERAWH, TIMERAWL and TIMERAWH again until the two high words agree (the read of the
  Zephyr version's `time_sync_mono_us()`). It has no side effect and takes no lock, so the kernel,
  the applications and interrupt handlers may all call it. The value
  is µs since boot; the TIMER starts again from 0 at every boot (`reboot` included). It is the
  counter behind `CLOCK_MONOTONIC`, and the two agree to within the time between the reads (a few
  µs).
- Threads of the same priority are time-sliced every 20 ms (`CONFIG_RR_INTERVAL=20`; the Zephyr
  version's `CONFIG_TIMESLICE_SIZE=20`). The watchdog that times the rest of a slice is not
  cancelled when the thread starts to wait, so a sleep longer than the rest of the slice takes one
  extra interrupt while the thread sleeps (NuttX 13.1.0 behaviour).
- `up_udelay()`, `up_mdelay()` and `up_ndelay()` wait by reading the TIMER directly, to the
  microsecond (they also work with interrupts disabled).
- The conversions between the counter and ticks or time are done by the board, which replaces the
  weak functions of `arch_alarm.c` (`up_timer_gettick()`, `up_timer_gettime()`,
  `up_alarm_tick_start()`, `up_alarm_cancel()`, `up_udelay()`, `up_ndelay()`) with arithmetic for
  1 MHz and 100 µs (`src/nano_timeconv.h`: division by 100 one byte at a time, with a 32-bit
  multiply and a shift). The Cortex-M0+ has no divide instruction, and NuttX's generic conversions
  (`ONESHOT_COUNT`) do 64-bit divisions, which are slow. The results equal
  the generic ones over the whole 64-bit range (host test `nano_timeconv`). The conversions assume a
  100 µs tick, so the build stops if `CONFIG_USEC_PER_TICK` is not 100.

## CPU load

`cpu_load_get(bool reset)` (`#include "cpu_load.h"`; the board's `src/nano_idle.c`) returns what
the Zephyr version's `CONFIG_CPU_LOAD` (`CONFIG_CPU_LOAD_BACKEND_IDLE_HOOK`) returns.

- The idle loop (`up_idle()`, the board's own with `CONFIG_ARCH_IDLE_CUSTOM=y`) disables interrupts,
  reads the TIMER, waits for an interrupt with `WFI`, reads the TIMER again and adds the time to
  the idle time, and then enables interrupts. The handler of the interrupt that woke the CPU runs
  after that, so the time spent in interrupts that arrive while idle counts as load (as in the
  Zephyr version). `WFI` on the Cortex-M0+ returns on a pending interrupt even while interrupts are
  disabled, so an interrupt that arrives between disabling them and `WFI` does not leave the CPU
  asleep.
- The value is `1000 × (total − idle) / total` since the last call with `reset=true` (since boot
  before the first), rounded down, in tenths of a percent; 0 if no time has passed. A call with
  `reset=true` also returns the value up to that moment and then starts the next window.
- The time base is the TIMER (1 µs); the Zephyr version converts CPU clock cycles to µs. Each idle
  window is off by up to ±1 µs, without bias; even with many windows the effect on the result is
  estimated to be well below 0.1‰.
- The values were checked on the board while doing nothing, with a low-priority busy loop, with
  10 ms of work and 10 ms of waiting in turn, and with interrupts alone (a wdog at short
  intervals). The entry and exit of each interrupt (the timer interrupt, the wdog, setting the next
  alarm) also count as load.

## Thread priorities and stacks

Priorities map as `NuttX = 118 − Zephyr`; `nuttx/drivers/include/priorities.h` holds the constants
(`NANO_PRIO_*`) and the stack sizes (`NANO_STACK_*`). The Zephyr version's application threads
(priorities 8 to 11) become 110 to 107, its shell thread (14) 104.

| Thread | Zephyr | NuttX | Stack |
|---|---|---|---|
| `wifi_reconnect_tid`, `nina_scan_tid`, `nina_connect_tid` | 8 | 110 | 2048, 1536, 1024 |
| `time_sync_tid`, `imu_stream_tid`, `nina_monitor_tid` | 9 | 109 | 2048, 2048, 1536 |
| `imu_fifo_worker` | 10 | 108 | 2048 |
| `imu_raw_mqtt_tid` | 11 | 107 | 2048 |
| `nina_event` (turns the NINA's socket state into usrsock events; NuttX only; started by the first socket) | – | 106 | 1536 |
| `sysworkq` (system work queue; compatibility layer; INT1's work) | −1 (cooperative) | 119 | 1024 |
| `net_mgmt` (network events; compatibility layer; the Wi-Fi event callbacks) | −1 (cooperative) | 119 | 2048 |
| `RTIO WQ 0` (RTIO work queue; compatibility layer; single reads; `RTIO_WQ_0` in `ps`) | 0 | 118 | 1024 |
| `sensor_shell_processing_tid` (prints the results of `sensor get` and `sensor stream`) | 0 | 118 | 3072 |
| `main` (NuttX: init, which drops to the shell's priority after the start-up work) | 0 | 118 | 2048 |
| Shell (nsh session, commands; `nsh` on USB, `nsh_telnet` on Telnet) | 14 | 104 | 3072 |
| `telnetd` (accepts Telnet clients; the Zephyr version's `net_socket_service`) | 15 → 14 | 104 | 2048 |
| `iperf_traffic`, `iperf_report` (`iperf`'s pthreads; the Zephyr version's `zperf_work_q`) | 15 → 14 | 100, 101 | 4096, 4096 |

- Zephyr's cooperative threads (negative priorities: `sysworkq`, `net_mgmt` and others) map above
  118, but NuttX has no cooperative threads that other threads cannot preempt; a thread of higher
  priority preempts them. In the port so far these are `sysworkq` and `net_mgmt` (both 119), and
  no thread has a higher priority. The two at 119 take turns by time slicing (Zephyr's cooperative
  threads only switch when one yields).
- The `K_THREAD_DEFINE()` threads (`nina_scan_tid`, `nina_connect_tid`, `nina_monitor_tid`,
  `imu_stream_tid`, `imu_raw_mqtt_tid`, `time_sync_tid`, `wifi_reconnect_tid`,
  `sensor_shell_processing_tid`) are started by init at the start of its start-up work, as the Zephyr version
  starts them before `main()`.
- The stacks of `imu_stream_tid`, `imu_raw_mqtt_tid`, `time_sync_tid`, `imu_fifo_worker`, `sysworkq`, `RTIO WQ 0`, `net_mgmt`, the
  `nina_*_tid` threads, `wifi_reconnect_tid` and `sensor_shell_processing_tid` are static arrays
  as in Zephyr (`K_THREAD_DEFINE()`, `K_THREAD_STACK_DEFINE()`, NuttX's
  `kthread_create_with_stack()`). `ps` shows them smaller by the area NuttX puts at their start
  (2048 → 2008, 2000 for `imu_stream_tid`; 1024 → 976).
- `CONFIG_PRIORITY_INHERITANCE=y`. Mutexes inherit priority; semaphores do not by default (NuttX
  13.1.0's `sem_init()` sets their flags to 0), so semaphores used for signalling stay without
  inheritance.
- Interrupt stack (`CONFIG_ARCH_INTERRUPTSTACK=2048`, the size of the Zephyr version's
  `CONFIG_ISR_STACK_SIZE=2048`): threads run on the process stack (PSP), interrupt handlers on the
  main stack (MSP), so handlers do not use the stack of the thread they interrupt. Only the 32 bytes
  that the hardware saves (and the save area of a context switch) go on that thread's stack, as in
  the Zephyr version. Checked on the board: a log line written in an interrupt while nothing runs
  does not change the idle thread's stack use. `ps` does not show the interrupt stack; `irqstack`
  does (in the format of the last line of the Zephyr version's `kernel thread stacks`; the value
  is the peak since boot found from the stack coloration (`CONFIG_STACK_COLORATION`), the same as
  the `irq` line of a PANIC report):

  ```
  nsh$ irqstack
  <address> IRQ 00                           (real size 2048):	unused <unused>	usage <used> / 2048 (<percent> %)
  ```
- Stack sentinel (`CONFIG_NANO_STACK_SENTINEL`, board `Kconfig`, on by default): like the Zephyr
  version's `CONFIG_STACK_SENTINEL`, the lowest word of each stack (the value written by
  `CONFIG_STACK_COLORATION`) is checked at every thread switch (for the thread switched out) and at
  the end of every interrupt handler (for the interrupted thread, whose stack holds what the
  hardware saved), and a changed word raises PANIC (the report is made on the interrupt stack and
  goes to the RAM log; the board then restarts by itself, see "Log (`dmesg`)"; the Zephyr version stays
  stopped). The interrupt stack itself has no sentinel, as in the
  Zephyr version. The checks run
  in two scheduler instrumentation hooks (`sched_note_suspend()` at a switch, and
  `sched_note_irqhandler()`, which `irq_dispatch()` calls around each handler;
  `src/nano_stackcheck.c`). On the Cortex-M0+ peripheral interrupts do not go through
  `arm_doirq()` (they use `exception_direct()`), so the switch hook alone does not see the end of
  an interrupt. All interrupts run at one priority and never nest, so every handler exit is the
  end of a non-nested interrupt in Zephyr's terms. NuttX's `CONFIG_STACKCHECK_MARGIN` is not used:
  it checks at switches only and needs `CONFIG_DEBUG_ASSERTIONS` (every DEBUGASSERT).
- `ps` shows stack use in its STACK, USED and FILLED columns.

## Tests

```bash
nuttx/script/host_tests.sh
```

This builds and runs the tests of the OS-independent common sources on the PC. It needs no board,
only the host's `cc` (C17). Nor does it need the setup script, except that `lsm6dso_driver` uses
ST's `lsm6dso_reg.c` that `script/setup.sh` fetches (`nuttx/stmemsc/`); without it, that test
(and `imu_cal`, `sensor_shell`) prints `skip` and is left out. The tests check FIFO word parsing,
batches and payloads, the send queue, the status and info JSON, time mapping, the mapping from the sensor
tick to the clock, the NTP estimate, FIFO packet decoding, conversion to physical units and the
stored calibration format, gyro zero tracking at rest, the board's time conversions
(`nano_timeconv`), the CPU load arithmetic (`nano_cpuload`), the RAM log's ring, `dmesg`
pieces, line prefix and rate limit (`log_ram`), the key-value area's range checks, page writes and
boot-time contents check (`nano_flash_kv`), the key rules of the key-value store (`kv_name`) and
the hex conversions of the `kv` command (`kv_hex`), the handling of the commands' subcommands
(`subcmd`), the stored Wi-Fi credentials and the `wifi cred` command (`wifi_cred`), the NINA's SPI transfer layer (`nina_spi`), the NINA's socket table and commands
(`nina_sock`), the NINA's name resolution (`nina_dns`), the time synchronization's query, reply checks, wait
for the reply, estimate expiry, realtime clock and `ntp status` output (`time_sync`, with a fake
NINA socket and clock), the MQTT transport's start checks, topics and thread, the order of its
socket calls, CONNECT and the CONNACK wait, the order of info, status and batch each in one
`sendmsg()`, the status and input intervals, the dropped batches and the error paths
(`imu_raw_mqtt`, with a fake socket, clock and network interface), the OS-independent part of the I2C0
driver (`nano_i2c_dw`), the command words of the I2C0 DMA reads (`i2c_dma_commands`), the
output of the `i2c_dma` command (`i2c_dma_app`, with the driver side replaced by fakes), the
LSM6DSO FIFO control (`fifo_hw`), the LSM6DSO driver and its FIFO stream (`lsm6dso_driver`), the
loading of the calibration and the output of `imu cal` (`imu_cal`), the `sensor` command's help,
argument checks, wildcards, device names, the output of `get`, `attr_get`, `attr_set`, `stream`
and `trig`, written by the processing thread's callback to the command's terminal (`sensor_shell`,
on the driver and fake sensor of `lsm6dso_driver`), the stream's state machine (`stream_stop`), the compatibility layer's RTIO (`zephyr_rtio`) and
work items (`zephyr_work`), and the parts of the layer added for the acquisition service
(`zephyr_compat_misc`). Each test is built and run in turn,
and the script stops at the first failure. Build products go to a temporary directory that is
removed on exit.

The following files are unchanged copies of the files of the same name in the Zephyr version
(`zephyr/app/usb-console`). When the Zephyr version changes, replace the copies with the same
content.

| Location | Location in the Zephyr version | Contents |
|---|---|---|
| `nuttx/apps/imu/` | `src/` | `imu_raw_{batch,packet,queue,status,time}`, `imu_tick_clock`, `time_offset`, `imu_cal_codec.c`, `imu_cal.h`, `kv_store.h`, `time_sync.h` |
| `nuttx/drivers/lsm6dso/` | `drivers/lsm6dso/` | `lsm6dso_{fifo,cal,zero,fifo_decoder,fifo_hw,fifo_irq}`, `lsm6dso_stream.c`, `lsm6dso_fifo_packet.h`, `lsm6dso_fifo_stats.h`, `lsm6dso_attributes.h` |
| `boards/arduino-nano-rp2040-connect/src/` | `drivers/i2c_dw/` | `i2c_dw_rp2040_commands.[ch]` |
| `nuttx/tests/` | `tests/` | the tests of the files above (`i2c_dma_commands.c`, `fifo_hw.c` and `stream_stop.c` included; `stream_stop.c` includes `../drivers/lsm6dso/lsm6dso_stream.c`) |
| `nuttx/tests/fakes/` | `tests/fakes/` | stand-ins for the Zephyr and ST headers the tests need (`zephyr/drivers/sensor.h`, `zephyr/drivers/sensor_clock.h`, `zephyr/drivers/i2c.h`, `zephyr/rtio/rtio.h`, `lsm6dso_reg.h`) |

`nuttx/drivers/nina/nina_net.c` is a copy with only the leading comment and `<nuttx/config.h>`
added; `nuttx/apps/wifi/wifi_profile.c` is a copy with only the includes (the log is `app_log.h`)
and the declaration of `wifi_reconnect_tid` (without `const`) changed; `nuttx/apps/imu/imu_stream.c` is a copy with only the calls into the system and the command
registration at the end replaced; `nuttx/apps/imu/time_sync.c` is a copy with only the calls into
the system, the body of `time_sync_mono_us()`, the Kconfig names and the command registration at
the end replaced; `nuttx/apps/imu/imu_raw_mqtt.c` is a copy with only a leading comment and the
includes of NuttX's headers added; `nuttx/apps/imu/imu_cal.c` is a copy with only a leading
comment and the include of `<nuttx/config.h>` added; `nuttx/apps/sensor/sensor_shell.c`,
`sensor_shell_stream.c` and `sensor_shell.h` are copies of the Zephyr version's sensor shell and of
Zephyr's, with only the differences listed in "Sensor command (`sensor`)"; `imu_raw_mqtt.h` is a copy with `<zephyr/net/net_ip.h>` replaced
by `<netinet/in.h>`.

`nuttx/tests/fakes/nuttx/config.h` (empty) is the NuttX port's own: it lets a host test build a
source of `nuttx/apps` that includes `<nuttx/config.h>` (`i2c_dma_app`).

`nuttx/tests/nano_timeconv.c` is a test of the NuttX port only. It compares the conversions of
`src/nano_timeconv.h` with C's `/` and `%` and with the generic conversions of NuttX 13.1.0 (the
arithmetic of `oneshot.h` and `clkcnt.h` written out for 1 MHz and 100 µs): every number in the
range where the one-byte division step must be exact, 300 values on each side of edge values (0,
99, 100, around 2^32, around 2^32 × 100, 2^63, 2^64 − 1 and others), every count of the first
10^7 µs, 2000 values on each side of 256 multiples of 2^32, and 4 million random values.

`nuttx/tests/nano_cpuload.c` and `nuttx/tests/log_ram.c` are also tests of the NuttX port only.
The first compares the arithmetic of `src/nano_cpuload.h` with the steps of Zephyr's
`cpu_load_get_cpu()` (reset, an open window, zero time, rounding down). The second compares
`nuttx/drivers/log/log_ring.c` with the byte-by-byte store of the Zephyr version's `log_ram.c`
over 20,000 random writes, compares the line prefix with the time arithmetic of Zephyr's
`log_output.c` for 200,000 random values, and checks the `dmesg` pieces (skipping what is
overwritten, stopping at 8192 bytes) and the rate limit.

`nuttx/tests/nano_flash_kv.c`, `kv_name.c` and `kv_hex.c` are also tests of the NuttX port only.
`nano_flash_kv` checks the range checks of `src/nano_flash_kv.h` around the edges and for every
page and erase sector of the first and last MiB of the flash, and checks the page writes, at many
offsets and lengths around page boundaries, by seeing that exactly the written bytes change. For
the boot-time check it confirms that the table entries NuttX's NVS writes are accepted and that
the Zephyr version's NVS entries, laid out as they are in the flash, are not (including ones that
happen to pass NuttX's CRC). `kv_name` checks the key checks split out of the Zephyr version's
`kv_store.c` (`apps/imu/kv_name.c`), and `kv_hex` the copy of Zephyr's `lib/utils/hex.c`
(`apps/kv/kv_hex.c`).

`nuttx/tests/subcmd.c` checks `apps/subcmd/subcmd.c` with the tables of `kv` and `nina` (the help
is the one recorded from the Zephyr version), a nested table, optional arguments, and the layout of
help texts (newlines, the line break before column 80, a newline at the end).

`nuttx/tests/wifi_cred.c` checks the copies of Zephyr's library in `apps/wifi/`
(`wifi_credentials.c`, the settings backend) and the `wifi cred` command (the copy of
`wifi_credentials_shell.c`, `subcmd`, the layer's `zephyr_wifi_utils.c`) on a fake settings store
of named items: loading at boot, the stored names (`wifi_cred/<index>`, apart from `app_kv/`) and
contents, the length limits (SSID 32/33, passphrase 7/8/64/65), the third network's `-105` and its
log line, replacing the same SSID and the list in index order, each option and error message, a
stored value of the wrong length, a store failure, `auto_connect`, and the help of `wifi` and
`wifi cred`. The expected output is what Zephyr's sources print.
`nuttx/tests/nina_spi.c` checks `drivers/nina/nina_spi.c` with a fake SPI, GPIO, clock and lock: the
cases of the Zephyr version's `tests/nina_spi.c` (wide parameters and padding, bulk transfers, the
three answer parameters of 0x75 and 0x7a, the length checks, a failing transfer), and also narrow
parameters, the version request, the search for the start of the answer (1000 bytes), each
`-EPROTO`, `-EMSGSIZE` and `-EINVAL`, 0x27, `nina_ip_config()` (0x21), the READY time limits (2000,
20 and 10000 ms, counted as samples every 100 µs), the reset on the first request, the order of
the CS edges and transfers, and the lock.

`nuttx/tests/nano_i2c_dw.c` checks `src/nano_i2c_dw.h` against the relevant parts of the Zephyr
version's `i2c_dw.c` and `i2c_dw.h` and of Zephyr's `i2c.h`, written out in the test: the SCL
counts (500/625, 93/109 and 31/31 at 125 MHz, and every combination of a 1 to 400 MHz clock and a
speed class), the frequencies taken, the NuttX message flags turned into Zephyr's, the words
written to `IC_DATA_CMD` (the `i2c` tool's `get`, `set`, `set -s`, `dump` and `dev`, a sensor
register read and write, three messages and so on, run as consecutive transfers so that the
direction of one transfer sets the RESTART at the start of the next), the `IC_CON` value and the
register layout.

`nuttx/tests/lsm6dso_driver.c` is also a test of the NuttX port only. It builds
`drivers/lsm6dso/lsm6dso.c` with ST's real `lsm6dso_reg.c`, the stmemsc glue, the compatibility
layer (`drivers/compat`) and the board's `src/nano_lsm6dso.c`, and runs it against a fake LSM6DSOX
(register banks, address auto-increment, software reset, FIFO, timestamp) behind a fake I2C
master. Its stand-in headers are in `nuttx/tests/lsm6dso_fakes/` (the board's `board.h` is the
real one). It checks the initialization's log lines and their order, the register writes and
values in order (with the 100 µs after the reset), the registers after it, the sensitivities, an
I2C failure at each step and its log lines, `device_is_ready()`, the address, speed (100 kHz and
what `i2c_configure()` sets) and message shape of every transfer, the registers a single reading
reads and its conversion (against `lsm6dso_cal.c` and the physical value, the signs of `val1` and
`val2`, one axis and three), the attributes (the ODR steps, the rounding of m/s² and rad/s, the
refused values and channels, `-EBUSY`), starting, reading, pausing and stopping the FIFO and its
statistics, the timestamp reading, and `lsm6dso_cal_zero()`. It also runs the FIFO stream:
`lsm6dso_stream.c`, `lsm6dso_fifo_irq.c`, `lsm6dso_compat.c` and the decoder with the
compatibility layer (work, RTIO, `gpio0`, the default iodev and decoder), on a fake kernel
(`nuttx/tests/zephyr_fakes/`: one thread; the test runs the work queues and fires the timers) and a
fake INT1 that follows the fake sensor's watermark. It checks the stream's start (registers, the
INT1 interrupt), batches on an INT1 edge and on the 20 ms look, a second stream (`-EBUSY`) and
another trigger (`-ENOTSUP`), the graceful stop (pause, the last partial batch, `-ECANCELED`, the
registers back), a cancellation, an overrun (`-EOVERFLOW`), and a single read through the RTIO
work queue with the decoders.

`nuttx/tests/imu_cal.c` is a test of the NuttX port only too. It includes `lsm6dso_driver.c` for
the same fake sensor (running the checks of `lsm6dso_driver` first) and builds
`apps/imu/imu_cal.c` with a fake KV store. It checks the loading at boot (two keys), that a stored
value is applied to the single reads (`M·(raw−bias)`), that a value of the wrong length, version,
kind, sensitivity or reserved bytes, or one too long, leaves the nominal conversion, that a range
change reads the key of the other sensitivity and back, `reload` (a deleted key is nominal again,
a tracked bias is dropped), `zero` (at rest, far from the stored bias, during a stream, on an I2C
error), the usage error, an IMU that is not ready, and every line of `imu cal` (including the
rounding of the bias to two decimals) with where it goes (stdout or stderr).

`nuttx/tests/sensor_shell.c` is a test of the NuttX port only too. It includes `lsm6dso_driver.c`
for the same fake sensor, builds the copies of `apps/sensor/` with `apps/subcmd/zshell.c`, and
points the command's stdout and stderr at one file (in place of the terminal). It checks the help
of `sensor` (no `info`, the `SHELL_HELP()` description and `Usage:` line wrapped at 80 columns),
the wrong argument count with the help and the dynamic device list, wildcards (expanded to
subcommands or channel names, the words no longer read after one that matches nothing, the error
when a second handler would run), the device names (node name and label, the node name only for
`stream`), `get` (all channels, named ones, a number, unknown names, the line format), `attr_get`
and `attr_set` (the FIFO attributes, the value parser, unknown names, the driver's errors, `%.6f`,
every channel and attribute), `stream` (the argument errors, the start, the trigger line and the
batch averages printed by the processing thread's callback, `attr_set` refused during the stream,
`off`) and `trig`. The results are written after the command has returned, by the processing
thread's callback, to the command's terminal (the backend's path).

`nuttx/tests/zephyr_rtio.c` and `zephyr_work.c` are tests of the NuttX port only too: they check
the compatibility layer's RTIO and work items on the fake kernel. The first uses a context of
`imu_stream.c`'s sizes and a fake iodev and checks the memory pool (block size, allocation from
`max` down to `min`, binding to the submission, first fit, exhaustion and release), the
submission pool (4, reuse in order of release, handles and generations), completions (order, 16
of them, a dropped one counted in `xcqcnt` with its buffer left allocated, `consume_sem`),
multishot (handed back before the completion, the binding cleared, the handle kept, freed on an
error or a cancellation), cancellation (the mark, the iodev's function, stale and invalid
handles, no completion for one canceled before submission) and the RTIO work queue. The second
checks the return values and flags of `submit`, `schedule`, `reschedule` and
`k_work_cancel_sync()`, and `K_MSEC()` in ticks.

`nuttx/tests/zephyr_compat_misc.c` is a test of the NuttX port only too. It checks the parts added
to the compatibility layer for the acquisition service: `K_MUTEX_DEFINE()`, `K_THREAD_DEFINE()`
(name, priority 9 → 109, stack, entry and arguments) with `K_THREAD_DEFINE_START()`, the rounding
of `k_uptime_get()` (against Zephyr's `k_ticks_to_ms_floor64()` of 100 µs ticks, around every carry
and across the wrap of the 32-bit tick count), the ticks of `k_sleep(K_MSEC(1))`,
`sys_put_be32()`/`sys_get_be32()`, `snprintk()`, `hwinfo_get_device_id()` (fake chip ID), where
`shell_print()` and `shell_error()` write, and `zsock_inet_pton()`/`zsock_inet_ntop()`.

`nuttx/tests/nina_sock.c` checks `drivers/nina/nina_sock.c` with a fake `nina_request()` that checks
each command and its parameters and gives scripted answers, and a fake clock: the supported socket
types, the table of 10 and `EMFILE`, creating the NINA's socket on first use (type and protocol,
FIONBIO), bind, listen (backlog clamped to 1..255), accept (the peer's address and the byte order of
its port, `EAGAIN`, `EMFILE`), connect (UDP; TCP in progress, the errno of each SO_ERROR value, the
error flag, the 10000 ms limit, a failed poll, closing the NINA's socket after a failure), close,
setsockopt, what `nina_event` queries and turns into events (a socket marked as polled by its
callers only: only while it connects), and `NINA_SO_POLL` (the Zephyr version's `do_poll()` for one
socket). For sending and receiving: TCP
send (at most 3584 bytes a call, the answer checks, `MSG_DONTWAIT`, the passes of a waiting send
with its state check every 100 ms and its 10000 ms limit, a continued call told from a new one),
`sendmsg` (vectors joined into one command, what it refuses), UDP send (the address checks, the
connected peer, the 512-byte limit, a datagram the NINA did not take), TCP and UDP receive (the
poll first, the size parameter, the byte order of the sender's port, dropping datagrams from other
than the connected peer, an empty datagram), a poll that found data standing for the next
receive's first poll, and `FIONREAD`.

`nuttx/tests/nina_dns.c` checks `drivers/nina/nina_dns.c` with a fake `nina_request()` and lock:
the hints' defaults and the refused flags, families and socket types, the service (a port number),
numeric addresses (the NINA is not asked), a name (commands 0x34 and 0x35 with the lock held), and
the `EAI_*` of each failure.

There are no host tests yet of the NINA's Wi-Fi (`nina_net.c`, `nina_netdev.c`, the network parts
of the compatibility layer).
