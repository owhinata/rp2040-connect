# Design and behaviour of the NuttX build

English | [日本語](nuttx-porting-ja.md)

How each part of the NuttX build works in detail. Usage is in
`nuttx/README.md`, the drivers and the compatibility layer in `nuttx/drivers/README.md`, and the
history and the reasons for the decisions in `nuttx/PORTING_STATUS.md`, `nuttx/PORTING_NOTES.md`
and `nuttx/PORTING_PLAN.md` (in Japanese). A section named in quotes is a section of this document
("Commands of the Zephyr build and of the NuttX build" and "Error numbers (errno)" are in
`nuttx/README.md`).

## What `setup.sh` does

The table of what is fetched is in "Setup" in `nuttx/README.md`. What the table does not say:

- `nuttx/pico-sdk` is a sparse checkout of what the boot2 build reads and of the divider routines the
  board builds (`divider_hardware.S` and the `divider_helper.S` it includes; `CONFIG_NANO_HW_DIVIDER`).
  Paths an earlier partial clone lacks are added to its sparse checkout, and their files are fetched then
  (the clone fetches blobs later, so this needs the network). A full checkout already in `nuttx/pico-sdk`
  is used as it is. To get the small partial one, delete it and source `setup.sh` again.
- The argtable3 version is the default `CONFIG_SYSTEM_ARGTABLE3_VERSION` of nuttx-apps 13.1.0.
  nuttx-apps' Makefile uses a clone in `nuttx/nuttx-apps/system/argtable3/argtable3` if there is
  one; otherwise it fetches it from GitHub on every build and removes it at `make distclean`.

It also does the following:

- Creates the symlinks `nuttx/nuttx/external` → `nuttx/drivers` and `nuttx/nuttx-apps/external` →
  `nuttx/apps`. NuttX builds this repository's drivers and applications through them.
- Applies `nuttx/patches/nuttx/*.patch` and `nuttx/patches/apps/*.patch` to `nuttx/nuttx` and
  `nuttx/nuttx-apps`, in name order (see "Patches to NuttX" in `nuttx/README.md`). What it applied is recorded in
  the clone's `.git/nano-patches/`, and the same patches are not applied again. When a patch is
  added, removed or changed, the next run takes the recorded patches back in reverse order and
  applies the current ones. If a clone has local changes but no record, it stops without touching
  it; restore the clone with `git -C nuttx/nuttx checkout -- .` (likewise `nuttx-apps`) and source
  the script again.
- Sets `PICO_SDK_PATH` in the current shell and adds the toolchain's `bin`, `nuttx/.venv/bin` and
  `nuttx/script` to `PATH`.

`nuttx/script/kconfig-tweak` stands in for the `kconfig-tweak` that NuttX's `tools/configure.sh`
(through `tools/sethost.sh`) uses to edit `.config`. kconfiglib does not include it, so only the
features NuttX uses are implemented.

The fetched files are not tracked by Git. To change a version, edit the value at the top of the
script, delete the corresponding directory and source the script again.

## What `build.sh` does

The script does the following:

1. Configures NuttX (`tools/configure.sh -l -a ../nuttx-apps`) when it has not been configured
   yet, or when the defconfig, the board's `Kconfig` or `scripts/Make.defs`, a `Kconfig` under
   `nuttx/drivers/` or `nuttx/apps/`, the set of application directories, the patches or the
   NuttX/apps commit have changed since the last configure (recorded in
   `nuttx/build/.configure-stamp`). NuttX reads the board's `Kconfig` and generates
   `nuttx/apps/Kconfig` only at configure time, and make does not rebuild boot2 when its flags in
   `scripts/Make.defs` change, so it starts from `make distclean`. Otherwise the build is
   incremental.
2. Runs `make` in `nuttx/nuttx`.
3. Checks the image: `.boot2` is at 0x10000000 and is 256 bytes (without `PICO_SDK_PATH`, NuttX
   would silently leave boot2 out), `nuttx.bin` covers exactly what the ELF loads into flash,
   and the image ends before 0x10ffe000. The last 8 KB of the flash (0x10ffe000–0x10ffffff) is
   kept out of the image for the key-value store, as in the Zephyr build. With
   `CONFIG_RP2040_ROM_MEM_OPS` it also checks that `__start()` calls neither `memcpy` nor `memset`
   (section "The C library's `memcpy` and `memset`"). With `CONFIG_NANO_HW_DIVIDER` it checks that the six
   `__wrap___aeabi_*` are there and that no libgcc division (`__aeabi_idiv`, `__divsi3`, `__udivmoddi4` and
   so on) is left in the image (section "Integer division").
4. Converts `nuttx/nuttx/nuttx.bin` (raw image from 0x10000000, boot2 included) to
   `nuttx/build/nuttx.uf2` with `nuttx/script/bin2uf2.py` and prints the path and size. The
   converter checks the boot2 checksum first and writes 256-byte payloads with the RP2040 family
   ID (0xe48bff56). NuttX's own UF2 step (picotool) is turned off.

Any failed step stops the script with an error.

Changes made with `--menuconfig` stay in `nuttx/nuttx/.config` until the next configure; save
them with `--savedefconfig` to keep them. Other outputs: the ELF is `nuttx/nuttx/nuttx` and the
raw image `nuttx/nuttx/nuttx.bin`. Build products, including those NuttX writes into
`nuttx/drivers/` and `nuttx/apps/`, are not tracked by Git.

## Shell

The console and shell are nsh on the USB CDC ACM serial port, as in the Zephyr build (where they
are the Zephyr shell). The serial number of the USB device is the chip ID as 16 hexadecimal
digits, the same as the Zephyr build, so the port shows up as
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
  each runs the line once (as in the Zephyr build).
- Lines can be up to 511 characters long (as in the Zephyr build); further characters are
  dropped, without echo, until Enter. Up and down recall the last 8 lines, Tab completes command
  names, and Ctrl-A, Ctrl-E, Ctrl-K, Ctrl-U and Ctrl-W edit the line.
- Ctrl-C drops the line being typed, moves to a new line and shows a new prompt (also on an empty
  line; as in the Zephyr build). Ctrl-C pressed while a command runs ends the read of a command
  that reads its standard input (`cat` with no file, for example; Ctrl-D does too); otherwise it
  clears the line at the next prompt. It never stops a running command (neither does the Zephyr
  version). Commands that never end (`while`/`until` loops, `watch`) are left out.
- While Wi-Fi is connected, the same shell is also on Telnet (see "Shell over Wi-Fi (Telnet)").
- `help` lists the commands: first nsh's own (`uname`, `ps`, `free`, `echo`, `sleep`, ...), then,
  after `Builtin Apps:`, the commands of the NuttX build and the NuttX applications (`wapi`, `iperf`,
  `i2c`, ...). "Commands of the Zephyr build and of the NuttX build" maps them to the Zephyr build's
  commands. An unknown command gives `nsh: <name>: command not found` (the Zephyr build prints
  `<name>: command not found`).
- `reboot` restarts the board (the Zephyr build's `kernel reboot`). It also restarts when given
  an argument, `reboot --help` included.
- `bootsel` restarts into BOOTSEL (the USB mass storage `RPI-RP2`), as in the FreeRTOS build (the
  boot ROM's `reset_usb_boot(0, 0)`). No confirmation word; arguments are ignored. It saves pressing
  RESET twice before writing an image.
- `irqstack` prints the peak use of the interrupt stack since boot in one line (the format of the
  `IRQ 00` line of the Zephyr build's `kernel thread stacks`; see "Thread priorities and
  stacks").
- Wi-Fi is handled with NuttX's own `wapi` and `ifconfig` (see "Wi-Fi (`wapi`, `ifconfig`)"). The
  credentials are stored with the Zephyr build's `wifi cred` (see "Stored Wi-Fi credentials and
  automatic reconnection (`wifi cred`)").
- The log is kept in RAM and is not written to the console, as in the Zephyr build; `dmesg`
  shows it (see "Log").
- The shell (the nsh session) and the commands it starts run at priority 104, the place of the
  Zephyr build's shell thread (see "Thread priorities and stacks").

UART0 (GPIO0 TX, GPIO1 RX, the Arduino D1/D0 pins) is not the console. Its pins are given to the
UART, as in the Zephyr build, but nothing is written to it.

## Log (`dmesg`)

The RAM log of the Zephyr build's `src/log_ram.c` (`nuttx/drivers/log/`; the OS-independent ring,
line prefix and rate limit are `common/log/log_ring.c`, the `LOG_*` macros `common/log/app_log.h`
with its NuttX part `nuttx/drivers/include/app_log_os.h`).

- An 8192-byte ring buffer in `.noinit` (the Zephyr build's has 4096 bytes; it was made larger
  so that a fatal error report fits from its first line). The startup code does not clear it, so
  its contents survive a reset such as `reboot`. At boot its header (the magic number and its
  inverse, the start and the length) is checked, and the ring is emptied if it is not intact. The
  magic number is the Zephyr build's `0x4c4f4752` XORed with the size (`0x4c4f6752`), so a ring
  of another size left in RAM (the 4096-byte one of earlier builds) is emptied rather than misread.
  Power-off clears it. When it is full, the oldest bytes are overwritten.
- `dmesg` prints what is stored as is, or `No logs.` when it is empty; arguments are ignored. As in
  the Zephyr build it takes 96 bytes at a time, skips what new messages push out while it
  prints, and stops after twice the ring's size (16384 bytes; 8192 in the Zephyr build). NuttX's own `dmesg` (`/dev/kmsg`) is turned off.
- Lines have the format of the Zephyr build's log, `[HH:MM:SS.mmm,uuu] <inf> module: message`,
  ending in LF only. The levels are `<err>`, `<wrn>`, `<inf>` and `<dbg>`; `<dbg>` lines have
  `module.function: `. Hours take at least two digits and more after 99 hours. The time is the
  board's monotonic clock (the TIMER, µs since boot) and starts again from 0 at every reboot (as
  seen in the Zephyr build).
- Output of `syslog()` (kernel and nsh messages, PANIC reports) goes to the same ring. Like the
  Zephyr build's `printk`, it is stored as is, without a prefix, with line ends as CR LF
  (syslog's `CONFIG_SYSLOG_CRLF`, the same as the Zephyr build's printk strings).
- On a fatal error (an exception such as a HardFault, `PANIC()`, a failed assertion, the stack
  sentinel) the NuttX build writes its report (the cause, registers, stack and task list) to the
  RAM log and then restarts by itself (`CONFIG_BOARD_RESET_ON_ASSERT=2`; a normal boot, as
  `reboot`, not BOOTSEL). The RAM log survives the restart, so `dmesg` after it shows the report.
  The report has the cause line (`Assertion failed` with the reason, file and line, and the task
  that stopped), the registers (PC, LR and SP among them), the base and size of the stopped task's
  stack (and of the interrupt stack when in an interrupt), `Stack pointer ... is not within the
  stack` when SP is outside it, and the task list (each stack's use; the `irq` line is the
  interrupt stack). Stack contents (hex dumps) are left out (`CONFIG_SCHED_DUMP_STACK` is off):
  they make the report long enough to push its first lines (the cause and the registers) out of the
  ring. NuttX's `CONFIG_ARCH_STACKDUMP_MAX_LENGTH` only limits the dump of a stack that
  SP is not in, so it does not help here. The Zephyr build also restarts after a fatal error
  (`CONFIG_RESET_ON_FATAL_ERROR=y`; its RAM log is 4 KiB).
- Sources use it the Zephyr way: `#include "app_log.h"` (`common/log/`; usable from
  the kernel side, the board code and the applications, also in interrupt handlers), then per file
  `LOG_MODULE_REGISTER(name, LOG_LEVEL_INF)` (without a level: `CONFIG_NANO_LOG_DEFAULT_LEVEL`,
  default 3, or `LOG_LEVEL` if defined before the include), and `LOG_ERR()`, `LOG_WRN()`,
  `LOG_INF()`, `LOG_DBG()`, `LOG_ERR_RATELIMIT()` and so on in the code. Messages above the file's
  level are dropped at compile time; there is no run-time level control (Zephyr's `log` command).
  The header replaces `<syslog.h>`'s `LOG_ERR` (a syslog priority) with the macro of the same name,
  so a file that includes `app_log.h` uses `APP_LOG_SYSLOG_ERR` instead in `syslog(LOG_ERR, ...)`.
- A line is formatted in the calling thread (or interrupt), in a 192-byte buffer on its stack,
  and then written to the ring with interrupts disabled. A longer message is cut to fit 192
  bytes. In the Zephyr build's default (deferred) mode the caller only records the message, and
  a low-priority logging thread formats it later.

## Key-value store (`kv`)

The settings, the calibration and the Wi-Fi credentials are stored in the last 8 KiB of the flash
(`0x10ffe000`-`0x10ffffff`, two 4 KiB erase sectors), the Zephyr build's `storage_partition`, in
the Zephyr build's format (Zephyr's NVS and settings): the NuttX build reads and writes what the
Zephyr build stored, and the other way round. The API and the command are the Zephyr build's
`kv_store.c` itself (`common/imu/kv_store.c`, used by all three builds).

- Layers: `kv_store.c` → Zephyr's settings (`settings_save_one()`, `settings_load_one()`,
  `settings_delete()`) → Zephyr's NVS → the compatibility layer's flash device and flash map
  (`common/compat/zephyr_flash_kv.c`) → the OS boundary (the four functions of
  `common/compat/nano_flash_kv.h`; on NuttX the board's `src/nano_flash_kv.c`). The settings and NVS
  are unchanged copies of Zephyr v4.5.0's sources (`common/compat/subsys/` and others; the list is
  in the compatibility layer's section of the driver notes), built with the Zephyr build's Kconfig
  values (`common/compat/zephyr_kv_config.h`).
- Format (the values of the Zephyr build's `.config` and devicetree): NVS sectors of 4096 bytes × 2
  (`CONFIG_SETTINGS_NVS_SECTOR_SIZE_MULT=1`; of `CONFIG_SETTINGS_NVS_SECTOR_COUNT=8`, the 2 that fit
  in 8 KiB), a write block of 1 byte, erase value 0xFF. NVS's allocation table entries (ATEs) are 8
  bytes (id, offset, len, part, CRC-8), written from the end of a sector downwards, the data from
  the start upwards. The settings keep the largest name ID in use at ID 0x8000, the names from
  0x8001 (without a terminator) and each value at its name's ID + 0x4000. The names are
  `app_kv/<key>` (the `kv` keys) and `wifi_cred/<index>` (the credentials). The values are the
  Zephyr build's bytes too (the calibration 56 bytes, `imu.port` and the like text, `imu.session`
  4 bytes big-endian, a credential the 224 bytes of `struct wifi_credentials_personal`, whose layout
  `BUILD_ASSERT`s in the layer's `<zephyr/net/wifi_credentials.h>` check against the Zephyr build's
  ELF).
- Area: the board's `include/board.h` (`BOARD_KV_OFFSET`, `BOARD_KV_SIZE`), the constants of
  `nano_flash_kv.h` and the linker script (`__kv_start`, `__kv_end` in `scripts/nano-flash.ld`;
  the area is left out of the image's flash region, and `build.sh` checks where the image ends).
  Every read, program and erase is checked to lie inside the area in three places before the flash
  is touched: the compatibility layer (the constants), the board (the `board.h` constants and the
  linker script's values, found equal at boot) and the RAM function (right before the boot ROM is
  called). Anything outside is refused without touching the flash (`<err> flash_kv: Refused flash
  ...` in `dmesg`). The Zephyr build's flash driver accepts the whole flash; its NVS only touches
  this area.
- Boot: the board's start-up (`nano_bringup()`, after the LSM6DSOX and the NINA) only prepares the
  flash operations (the linker script checked, the boot ROM's functions found, boot2 copied). NVS
  is mounted, as in the Zephyr build, by the first call of the settings: the credentials'
  initialization, which init runs where Zephyr runs its `SYS_INIT()`. `dmesg` then shows the
  Zephyr build's three lines (`<inf> fs_nvs: 2 Sectors of 4096 bytes`, `alloc wra: <sector>,
  <offset>`, `data wra: <sector>, <offset>`). init then calls `kv_store_init()`
  (`settings_subsys_init()`) where the Zephyr build's `main()` does (`Could not initialize settings:
  <value>` on failure).
- Other data in the area: before mounting (at the first settings call after boot, once), the area is
  checked to hold Zephyr's NVS (a valid NVS ATE in one of the last two ATE slots of a sector) or to
  be erased throughout. If neither (the NuttX build's earlier format, what another firmware such as
  the OpenMV build left), the area is not erased; `dmesg` shows two lines and the settings are not
  initialized:

  ```text
  <err> flash_kv: Key-value area 0xffe000-0xffffff holds the NuttX build's earlier format
  <err> flash_kv: Not used; "kv wipe confirm" erases it (all keys and Wi-Fi credentials)
  ```

  (for data other than the earlier format the first line ends `holds data that is not Zephyr's
  NVS`). The `kv` operations return `-EILSEQ` (84 on NuttX: `KV read failed: -84`), storing a
  credential `-ENOENT`, and nothing is written. `kv wipe confirm` erases the area and mounts an
  empty NVS on it at once (`<wrn> flash_kv: Key-value area erased` and NVS's three lines). The
  calibration and the credentials' SSID list read before stay in RAM until the next start. The
  Zephyr build, in the same situation, does what Zephyr's NVS does: if the last 8 bytes of both
  sectors are written, the mount fails with `-EDEADLK` (`CONFIG_NVS_INIT_BAD_MEMORY_REGION` is off)
  and the store cannot be used; if either is erased, it takes the area for an interrupted garbage
  collection (`<inf> fs_nvs: No GC Done marker found: restarting gc`), erases both sectors and
  starts with an empty store. The NuttX build's earlier format, whose block being written ends
  erased, is usually the latter (checked on a fake flash, not on the board).
- Command (priority 104, 3072-byte stack; the Zephyr build's `SHELL_CMD_REGISTER(kv, ...)` run by
  `common/shell/zshell.c`, whose `main()` in `nuttx/apps/kv` only calls `z_shell_main_kv()`):

  ```
  kv set <key> <string>       store the string as it is
  kv sethex <key> <hex>       store 1-128 bytes given as hexadecimal pairs (upper case allowed)
  kv get <key>                print the value as lower-case hexadecimal
  kv delete <key>             delete it (Deleted also for a missing key)
  kv wipe confirm             erase the whole key-value area (NuttX build only; nothing
                              without confirm)
  ```

  Keys are 1 to 32 characters of letters, digits, `_`, `-` and `.`; values 1 to 128 bytes. Keys
  starting with `sys.` give `Reserved key`. The output is the Zephyr build's (`Saved`, `Deleted`,
  `KV write failed: -22`, `KV read failed: -2`, `Expected 1-128 bytes as hexadecimal pairs`,
  `Invalid hexadecimal value` and so on). `kv` alone or an unknown subcommand prints the help, a
  wrong number of arguments `<subcommand>: wrong parameter count` and the subcommand's help (the
  Zephyr shell's text); the help has a `wipe` line more. `kv wipe` with a word other than `confirm`
  prints `Type "kv wipe confirm" to erase all keys and Wi-Fi credentials` and does nothing;
  success prints `Erased`. Differences from the Zephyr build: nsh parses the arguments (quote `$`,
  `;`, `#`, `>` and the like), error lines go to standard error (the same console), the exit
  status is 0 on success and 1 on failure, and `kv wipe`.
- Capacity: all live entries must fit in one sector (4 KiB; the other is kept for garbage
  collection). An item is two entries, its name (`app_kv/` + key, an 8-byte ATE) and its value (an
  8-byte ATE): 183 bytes for a 32-character key and a 128-byte value. The write after the store is
  full gives `KV write failed: -28`. Storing the value already stored writes nothing (NVS compares).
  A Wi-Fi credential takes 11 + 8 + 224 + 8 = 251 bytes; up to two share the same area.
- Rewriting and garbage collection: a rewrite only adds a new entry (the value's ATE) and leaves
  the old one alone. When the sector being written is full, NVS closes it, copies the live entries
  of the other sector over, then erases that sector (garbage collection, done synchronously inside
  the write). Erasing and programming stop XIP, so they run from RAM with interrupts disabled, for
  one 4 KiB erase (34-40 ms measured) or one 256-byte page program (0.4-0.5 ms) at a time. Rewriting
  an existing key is usually two page programs (the data and the ATE; three across a page
  boundary). The Zephyr build's flash driver also disables interrupts per call: one erase (4 KiB),
  one write (one entry's data or one ATE).
- Writes during a measurement: the KV is written during a measurement only by shell commands
  (`kv set`, `kv wipe`, `wifi cred add` and the like; `imu start` stores `imu.session` before the
  acquisition starts). If a garbage collection runs then, one 4 KiB erase (interrupts off 34-40 ms)
  happens during the measurement. There is one erase each time the sector being written fills:
  about once in 300 rewrites of `imu.session` (a 4-byte value + an 8-byte ATE; NuttX's NVS before
  used about 38 bytes, about once in 100). On the board, one such erase during a measurement (6667 Hz)
  overflowed the sensor's FIFO and the recovery from an overrun ran once (2026-11-17, once in 260
  `kv set`; no batch missing at the receiver, the epoch advances by one). Reads (`kv_store_get()`, loading the credentials) write
  nothing, except that the settings delete a half-written item they find (only a name or only a
  value left by a power loss), as in the Zephyr build.
- Power loss: NVS writes the data before the ATE with its CRC and repairs, at boot, a half-written
  entry or an interrupted garbage collection. A host test (`common/tests/kv_settings.c`) cuts the
  power at each of the 652 programs and erases of a sequence of rewrites that goes through garbage
  collection several times (half a page programmed, half a sector erased) and checks that after
  the next boot every key holds its value from before or after the rewrite. No power was cut on the
  board.
- API (`common/imu/kv_store.h`, the Zephyr build's): `kv_store_init()`, `set`, `get` and `delete`
  take the Zephyr build's arguments and return its values (`-EINVAL` for a bad key or value,
  `-ENOENT` for a missing key, `-ENOBUFS` for a short buffer, `-ENOSPC` when full, `-ENODEV` while
  the flash is not ready, `-EILSEQ` for other data). `kv_store_*()` check, every time, that the
  settings are initialized (and initialize them again if not), as in the Zephyr build. The settings'
  and NVS's mutexes (Zephyr's `k_mutex`; in the layer NuttX's recursive mutex with priority
  inheritance) serialize the calls, so any thread can call them.

### Subcommands of the commands

The port's commands with subcommands (`kv`, `nina`, ...) give the Zephyr shell's output through a
shared part, `common/shell/subcmd.c` (`subcmd_main()`, built by `nuttx/apps/subcmd/`). Zephyr's `SHELL_CMD()` (no check of the
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
- `-h` and `--help` are ordinary words (as in the Zephyr build).
- Help texts are laid out as the Zephyr shell lays them out (`formatted_text_print()` of
  `shell_help.c`): a new line at each newline of the text and before a word that would pass column
  80, the next line starting at the column where the text began (the Zephyr shell moves the cursor
  there with a VT100 sequence; the port prints spaces). A newline at the end of a text leaves one
  empty line, as in Zephyr. There is no `resize` command to change the terminal width.
- The exit status is 0 on success and 1 on failure, help or a wrong number of arguments.

`sensor`, `imu`, `ntp` and `i2c_dma`, whose command tables are in the sources shared with the Zephyr build
(`common/`), run on `zshell.c` (`z_shell_cmd_main()`) of the same part. The rules are the ones
above; in addition, as in the Zephyr shell, it handles dynamic subcommands (device names, channel
names and so on), the `SHELL_HELP()` description with its `Usage:` line, the expansion of a
wildcard that matches subcommands (followed by a subcommand with a handler:
`Error: requested multiple function executions`), and lines of more than 30 words
(`CONFIG_SHELL_ARGC_MAX`: `Too many arguments in the command.`) (see "Sensor command (`sensor`)").
The handler gets the backend of the command's terminal (a `struct shell`) as `sh`, and
`shell_print()` and the others write to the command's standard output and standard error.

The compatibility layer's `<zephyr/shell/shell.h>` makes a command's registration
(`SHELL_CMD_REGISTER()`) the builtin command's entry. In the application's main source
(`MAINSRC`) the entry is `main()` (`sensor_shell.c`, `imu_stream.c`). A file that is not a main
source (`time_sync.c`: `ntp` is an application of its own, apart from `imu`) is compiled with
`Z_SHELL_CMD_NAMED_MAIN` by its build, and the entry is `z_shell_main_<name>()`
(`z_shell_main_ntp()`), which the application's `main()` (`nuttx/apps/ntp/ntp_main.c`) calls.

## NINA-W102 (`nina`)

The Wi-Fi module NINA-W102 is reached over SPI1. Its firmware is the Zephyr build's (Arduino's NINA
firmware 3.0.2 with fixes, `firmware/nina-fw/README.md`) and is not changed. The driver consists of the
transfer layer that sends one command and receives its answer (`nina_request()`,
`nina_ip_config()`), `nina version`, the sockets (NuttX's usrsock: socket, close, bind, listen,
accept, connect, setsockopt, sending and receiving), the NINA's name resolution
(`nina_dns_getaddrinfo()`), and the Wi-Fi network device `wlan0` (scan, connect, disconnect, status,
link monitor) ("Sockets (usrsock)", "Name resolution" and "Wi-Fi" in `nuttx/drivers/README.md`).
Stored credentials and automatic reconnection (`wifi cred`) are in their own section below.

```
nsh$ nina version
NINA firmware: 3.0.2
```

- `nina version` prints what the Zephyr build prints. On failure it prints
  `NINA version request failed: <value>` on standard error (the same console) and exits with 1.
  `nina` alone or with an unknown word prints the help. As in the Zephyr build, `version` does
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

- SPI: mode 0 (CPOL 0, CPHA 0), 8 bits, MSB first, `CONFIG_NINA_SPI_FREQUENCY` (default
  8000000; the same option as the Zephyr build's). The PL022 divides clk_peri (125 MHz)
  by an even number, so the clock is 7.8125 MHz (the same in the Zephyr build). NuttX's SPI1
  driver (`rp2040_spi.c`) is used by polling (the Zephyr build uses neither interrupts nor DMA).
  `CONFIG_RP2040_SPI_DMA` is off (with it, transfers over 4 bytes use DMA, whose completion is
  waited for without a time limit). The transfer layer, not the SPI driver, drives CS as a GPIO,
  and keeps it asserted while waiting for the answer. There is no `/dev/spi1`
  (`CONFIG_RP2040_SPI_DRIVER` is off).
- As in the Zephyr build, the NINA is set up on the first request: CS inactive, READY an input,
  IRQ high, RESET active; 10 ms later the reset is released, and after 750 ms IRQ is driven low and
  made an input again. The first request is the one that reads the MAC address when the start-up
  code sets up `wlan0` (at boot, as in the Zephyr build), so the USB console appears after these
  waits.
- The steps and values of a request are those of the Zephyr build (`common/drivers/nina/internal/nina_proto.c`,
  shared by the three builds). READY is not sampled every 100 µs but, as in the Zephyr and FreeRTOS
  builds, waited for on its GPIO interrupt (see "Waiting for READY" in `nuttx/drivers/README.md`),
  with the same limits: `-ETIMEDOUT` after 2000 ms before sending, 20 ms for the
  NINA to answer CS by releasing READY, and 10000 ms for the answer to be ready. Up to 1000 bytes
  are read looking for the start of the answer (0xE0). A recursive mutex (with priority
  inheritance) serializes the requests, and the SPI runs at the priority of the calling thread.

### Passthrough mode (`nina passthrough confirm`)

A mode for reading and writing the NINA's firmware with esptool on the PC (in place of Arduino's
SerialNINAPassthrough sketch; the same mode in the three builds, the NINA driver's
`common/drivers/nina/nina_passthrough.c`). `nina passthrough confirm` restarts the
board with its USB serial port connected to the NINA's UART (without the confirmation word the
command refuses). The UART is on the pins of the SPI's MISO and CS (GPIO8, GPIO9), so in this mode the
NINA's SPI, Wi-Fi, Telnet, MQTT, time synchronization, nsh and the key-value store do not run.

- The PC's DTR and RTS: RTS set releases the NINA's reset, DTR set pulls the NINA's GPIO0 low (boot
  mode). Opening the port (both set) starts the ESP32's ROM download mode; closing it holds the NINA in
  reset. The UART follows the rate the PC sets (8N1).
- Way out: close the port at 300 bps (`stty -F /dev/ttyACM0 300`), or press RESET. The NINA is started
  normally again, then the board restarts into the normal firmware. Nothing is written to the flash.
- At the start, at each close of the port and at the end, the bytes each way and the bytes lost go to the
  RAM log (`dmesg` after the way out).
- The esptool steps on the PC (`custom_reset_sequence = R0|D1|W0.2|R1|W0.6` in `ESPTOOL_CFGFILE` and
  `--before default-reset --after no-reset`; `--before no-reset` does not connect) are in "Reading and
  writing the NINA firmware" in the top-level `README.md`. On the board, the whole 2 MB was read at
  115200 bps and at 921600 bps with the same result (2026-11-18).
- How it works: "Passthrough mode" in `nuttx/drivers/README.md`.

### Wi-Fi (`wapi`, `ifconfig`)

The NINA is the NuttX network device `wlan0`. Instead of the Zephyr build's `wifi` commands,
NuttX's own `wapi` (`CONFIG_WIRELESS_WAPI_CMDTOOL`, priority 104, stack 3072) and `ifconfig` are
used. The ioctls call the NINA core's Wi-Fi operations (`common/drivers/nina/nina_wifi.c`, shared
with the other builds) through the core's NuttX binding (`common/drivers/nina/port/nuttx/nina_netdev.c`;
"Wi-Fi" in `nuttx/drivers/README.md`).

| Zephyr build | NuttX build |
|---|---|
| `wifi connect -s <SSID> -p <passphrase> -k 1` | `wapi psk wlan0 <passphrase> 3`, then `wapi essid wlan0 <SSID> 1` |
| `wifi connect -s <SSID> -k 0` (no encryption) | `wapi essid wlan0 <SSID> 1` (with no `wapi psk` just before) |
| `wifi disconnect` | `wapi disconnect wlan0` |
| `wifi scan` | `wapi scan wlan0` |
| `wifi status` | `wapi show wlan0` and `ifconfig wlan0` (below) |
| `net iface` | `ifconfig` |
| `wifi cred add/delete/list/auto_connect` | The same (`wifi cred`; see "Stored Wi-Fi credentials and automatic reconnection (`wifi cred`)" below) |

- In `wapi psk`, 3 is the CCMP cipher and the version defaults to WPA2: the Zephyr build's `-k 1`
  (WPA2-PSK). What `wapi psk` sets is used by the next `wapi essid` only and then forgotten, the
  passphrase wiped. As in the Zephyr build the driver takes only no encryption and WPA2-PSK (the
  NINA's passphrase command); anything else is `ENOTSUP`. Passphrases are 8 to 63 characters.
- Like the Zephyr build's `wifi connect`, `wapi essid` returns once the request is made. The
  connection steps and limit (every 500 ms, up to 60 times, so `ETIMEDOUT` after about 30
  seconds), turning power saving off and taking the IP configuration after association are the
  Zephyr build's. Check the result with `ifconfig wlan0` (`RUNNING` appears when connected, and
  `inet addr`, `DRaddr` and `Mask` show the NINA's DHCP values).
- Where the items of `wifi status` are: State is `RUNNING` in `ifconfig` (and the ESSID Flag in
  `wapi show`), SSID is ESSID in `wapi show`, BSSID is AP, RSSI is Sense (while connected only),
  Interface Mode is Mode (`WAPI_MODE_MANAGED`). As the Zephyr build's `wifi status`, `wapi show`
  asks the NINA for the SSID, BSSID and RSSI at that moment. Band, Channel, Security, MFP, Link
  Mode, Beacon Interval, DTIM, TWT and the PHY rate are not shown (the Zephyr build does not get
  them from the NINA either; they are fixed values or 0 there).
- `wapi scan` lists the NINA's scan (up to 10 networks) in the NINA's order, with the columns BSSID,
  frequency, signal level (dBm), encryption (`8000` none, `0800` encrypted, `ffff` unknown) and
  SSID. Encryption comes from the Zephyr build's mapping of the NINA's types (7 none, 5 WEP, 2
  WPA, 4 WPA2, others unknown); `wapi` does not tell WEP, WPA and WPA2 apart. The frequency column
  is in MHz (`iperf` selects `CONFIG_LIBC_FLOATINGPOINT`, so `printf` handles floating point;
  without it the column shows `*float*`). Scans for an ESSID, passive scans and scans of given channels
  are refused, as in the Zephyr build. A scan while connected is treated as a disconnection (the
  IP configuration is cleared) and asks for a reconnection when it ends, as in the Zephyr build.
- A connection during a scan and a scan during a connection attempt fail with `EBUSY`, as in the
  Zephyr build.
- `wapi` prints nothing when it fails (its error messages go through NuttX's debug output
  `wlerr()`, which this configuration leaves out). Its exit status is 1, so `echo $?` tells. The
  Zephyr build's `wifi` commands print the error value.
- `ifconfig wlan0 down` and `up` do nothing (the Zephyr build's interface is always up as well).

### Stored Wi-Fi credentials and automatic reconnection (`wifi cred`)

The credentials of the networks to join are stored, and the board joins again by itself at boot
and when the connection is lost. The credentials are a copy of Zephyr's library
(`subsys/net/lib/wifi_credentials/` with its settings backend), the command a copy of
`wifi_credentials_shell.c` (`nuttx/apps/wifi/`), and the reconnection is the Zephyr build's own
file, `common/imu/wifi_profile.c`. Syntax,
output, error messages and return values are the Zephyr build's.

```
wifi cred add -s <SSID> [-k <type>] [-p <passphrase>] [-c <channel>] [-b <band>] [-w <MFP>] [-m <BSSID>]
                                     store (replaces the same SSID; prints nothing on success)
wifi cred delete <SSID>              delete (prints Deleting network ssid: ...)
wifi cred list                       list by stored index (with the passphrase, as in the Zephyr build)
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
  (as in the Zephyr build).
- Type (`-k`): Zephyr's number (0 none, 1 WPA2-PSK, 2 WPA2-PSK-SHA256, 3 WPA3-SAE and so on; see the
  help of `-h`) is stored as it is. As in the Zephyr build only 0 and 1 can connect; stored
  networks of other types are skipped by the automatic connection. A passphrase without `-k` is
  stored with the warning `Passphrase provided without security configuration`. `-t` gives
  `Invalid option`, as in the Zephyr build (it is in the option string but not handled).
- `wifi cred`, `wifi` alone, or an unknown word print the help; `add` with other than 1 to 13
  words prints `add: wrong parameter count` and the help (see "Subcommands of the commands"). The
  only subcommand of `wifi` is `cred` (the others are `wapi` and `ifconfig`).
- Options are parsed by NuttX's `getopt_long()`, which, unlike Zephyr's `sys_getopt_long()`, does
  not move the words that are not options to the end (options after such a word are not read),
  and does not log the line the Zephyr build logs for an unknown option (`<wrn> sys_getopt`).
- Storage: each network is written at once, on every `add` and `delete`, to the settings name
  `wifi_cred/<index>` (the lowest free index), as Zephyr's `struct wifi_credentials_personal` (the
  copy of the Zephyr library's settings backend; it reads with `settings_load_subtree_direct()` as
  Zephyr does). At boot, init reads the stored SSIDs (where the Zephyr library's `SYS_INIT()` runs,
  before the threads start). The keys of `kv` start with `app_kv/`, so `kv` can neither read nor
  delete the credentials (`wifi_cred/0` contains `/`: `KV read failed: -22`). The format and place
  are the Zephyr build's, so credentials stored by the Zephyr build are used as they are (and the
  other way round). A stored entry of the wrong length reads as `-ENOENT` (`err: -2`), as in
  Zephyr.
- Automatic reconnection (thread `wifi_reconnect_tid`, Zephyr priority 8 → 110, 2048-byte stack):
  when networks are stored, at boot (`wifi_profile_init()`) and when the connection is lost (the
  link monitor finds it gone, or after a scan while connected), it waits 4 seconds, asks for a
  connection with the stored networks (`NET_REQUEST_WIFI_CONNECT_STORED`: in stored order, until
  one request is accepted), then checks every second and asks again after 32 seconds without an
  address. After `wapi disconnect wlan0` (a disconnection asked for) it does not reconnect until
  the next successful connection (as in the Zephyr build). With nothing stored it does nothing.
  A refused request (other than `EBUSY` and `EALREADY`) logs
  `<wrn> wifi_profile: Stored Wi-Fi request failed: <value>`; a failed connection itself
  (`ETIMEDOUT` after about 30 seconds) is not logged (nor in the Zephyr build).
- `wifi cred auto_connect` only asks for a connection from the shell and does not start the
  automatic reconnection (a successful connection does). When the request is refused it prints
  `An error occurred when trying to auto-connect to a network. err: <value>` (the exit status is
  always 0, as in the Zephyr build).
- The migration of the Zephyr build's old profile formats (the KV key `sys.wifi_profile`, the
  settings item `wifi_last/last`) runs at boot as in the Zephyr build (the store being the same, it
  moves them when they are there, as the Zephyr build does).
- The passphrase is shown by `wifi cred list` (as in the Zephyr build) and nowhere else: not in
  the log, not in error messages.

## Shell over Wi-Fi (Telnet)

As with the Zephyr build's Telnet shell (`CONFIG_SHELL_BACKEND_TELNET`), the shell of the USB
console is also on TCP port 23 while Wi-Fi is connected (`nuttx/apps/init/nano_telnet.c`,
`CONFIG_NANO_INIT_TELNET`; the port is `CONFIG_NANO_INIT_TELNET_PORT`, the Zephyr build's
`CONFIG_SHELL_TELNET_PORT`). There is no authentication or encryption, and anyone who can reach
the port can do everything the shell can, so use it on trusted networks only.

```bash
telnet <the board's IPv4 address>
```

- Listening: the start-up (init) opens the listening socket before the `USB console started` log
  line, the place where the Zephyr build's Telnet backend opens it (a `SYS_INIT()` before
  `main()`); the log says `<inf> shell_telnet: Telnet shell backend initialized`. As in the Zephyr
  version it listens on all addresses (`INADDR_ANY`) with a backlog of 1 and no `SO_REUSEADDR`.
  The NINA holds the socket, so a socket opened before Wi-Fi is connected starts accepting once it
  is, and keeps listening across Wi-Fi disconnections and reconnections (as in the Zephyr
  version). If it cannot be opened, the Zephyr build's messages are logged (`Unable to start
  telnet on NET_AF_INET (<value>)` and others) and there is no Telnet.
- One client at a time. A second client that connects while one is connected is accepted and
  closed at once, and `<err> shell_telnet: Telnet client already connected.` is logged; the first
  one keeps working (as in the Zephyr build). The `telnetd` task accepts the clients, at
  priority 104 with a 2048-byte stack (those of the Zephyr build's `net_socket_service` thread,
  which does this there). When an accept fails, it logs `Telnet accept error (<value>)` and waits
  `CONFIG_NANO_INIT_RETRY_MS` (100 ms) before accepting again, so that a lasting failure does not
  become a busy loop (the Zephyr build does not wait).
- A session is the same nsh as on USB (the same commands and output, the prompt `nsh$ `, no
  greeting). It is a task of its own and runs at the same time as, and independently of, the USB
  shell (like the Zephyr build's two backends), at priority 104 with a 3072-byte stack (as the
  USB session; the Zephyr build's `shell_telnet` thread priority 14 and
  `CONFIG_SHELL_STACK_SIZE`). `ps` shows it as `nsh_telnet`. NuttX's Telnet driver
  (`CONFIG_NETDEV_TELNET`, `CONFIG_TELNET_MAXLCLIENTS=1`) turns the accepted connection into the
  terminal `/dev/telnet0`, which the session uses as its standard input and output. NuttX's own
  Telnet daemon (`netutils/telnetd`, `NSH_TELNET`) is not used: it ends, listening socket and all,
  on a second client (a session over `TELNET_MAXLCLIENTS`) and when `SO_REUSEADDR` fails (the
  NINA's sockets give `ENOPROTOOPT`), and its sessions print a greeting.
- The prompt arrives right after connecting. The Zephyr build sends nothing on a connection (the
  output before it is dropped, and the prompt comes after an Enter). `host/imu_host` reads and
  drops whatever arrives after connecting before it sends a command, so it works the same with
  both.
- Echo and line editing: as on USB, readline echoes what is typed, and line editing, history, Tab
  and Ctrl-C (drop the line being typed and show a new prompt) work. NuttX's Telnet driver sends
  `IAC WILL ECHO` while a line is read and `IAC WONT ECHO` while a command runs, switching the
  Telnet client's local echo. The Zephyr build's Telnet shell turns echo off on a connection and
  negotiates no options; with echo off, the Zephyr shell also ignores the line editing, history,
  Tab and Ctrl-C keys, which leaves echo and editing to the client's local echo. `host/imu_host`
  drops the echo and the Telnet commands, so the results are the same.
- Line ends: the output's line end is CR LF (as in the Zephyr build; patch `nuttx/0006`). An
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
  therefore have the stack of the Zephyr build's shell, 3072 bytes (`dmesg` and `irqstack` were
  raised from 1024).

## Throughput measurement (`iperf`)

Instead of the Zephyr build's `zperf` (iperf 3 protocol), the port uses NuttX's `iperf`
(`netutils/iperf` in nuttx-apps, iperf 2 protocol). On the PC, use `iperf` (iperf 2), not
`iperf3`. The default port is 5001 (the Zephyr build uses 5201).

| Zephyr build | NuttX build | PC (with the NuttX build) |
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
  accept a bind to a given address (`EADDRNOTAVAIL`, `error code: 99`, as in the Zephyr build).
- UDP datagrams are at most 512 bytes (the NINA; as in the Zephyr build). With the default 1472
  bytes the send ends with `EMSGSIZE` (`err=90`) and the receive drops what is past 512 bytes, so
  give `-l 512` or less. `-l`, and waiting and sending again when the NINA cannot take a datagram
  (`EAGAIN`), come from patch `apps/0006`.
- The TCP server ends after one connection (NuttX's `iperf` does so). It does not return while no
  client comes (it waits in `accept()`, which `iperf -a` does not interrupt). The UDP server does
  not end; stop it with `iperf -a` from another shell (Telnet or USB, whichever is free). It prints
  `udp server recv error` at every 10 s receive timeout and ends at the first one after `-a`.
- Threads: the command `iperf` (priority 104, 3072-byte stack) and its pthreads `iperf_traffic`
  (priority 100, 4096) and `iperf_report` (101, 4096). The two pthreads' priorities and stacks are
  fixed in `iperf`'s source (the Zephyr build's `zperf_work_q` has priority 15, which Zephyr
  lowers to the lowest application priority 14, here 104). The read and write buffer comes from
  the heap (16 KiB for TCP).
- `iperf` selects floating point in `printf` (`CONFIG_LIBC_FLOATINGPOINT`) and argtable3
  (`CONFIG_SYSTEM_ARGTABLE3`).

## I2C0 (`i2c`)

I2C0 carries the LSM6DSOX (0x6A) and the ATECC608A (0x60). It is registered as `/dev/i2c0` for NuttX's `i2c` tool.
Reads that meet the conditions run on DMA (see "DMA reads (`i2c_dma`)" below).

The driver is the I2C core shared with the FreeRTOS build (`common/drivers/i2c_dw/`). It is a copy of the controller part
of the DesignWare I2C driver that the Zephyr version uses (Zephyr's `drivers/i2c/i2c_dw.c` with the local patch
`zephyr/manifest/zephyr/patches/zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch`); NuttX's `rp2040_i2c.c`
(`CONFIG_RP2040_I2C`) is not used. Section 12 of the [design](imu-raw-mqtt-design.md) describes the core; the NuttX part
(the files in the board's `src/` and the tables of the boundaries) is in the "I2C0" section of `nuttx/drivers/README.md`.

```
nsh$ i2c get -a 6a -r 0f
READ Bus: 0 Addr: 6a Subaddr: 0f Value: 6c
```

- Wiring (`BOARD_I2C0_*` in the board's `include/board.h`): SDA on GPIO12, SCL on GPIO13. As the
  Zephyr build's pinctrl sets them: the I2C function, no internal pulls, 4 mA, slow slew, Schmitt
  trigger on, input enabled. The I2C0 block is reset at boot before it is set up.
- The speed comes with each transfer, from the messages' frequency, and only 100000, 400000 and
  1000000 are taken (Zephyr's speed classes; other values give `-EINVAL`). The registers get the
  Zephyr build's values: at 100 kHz `IC_SS_SCL_HCNT`/`LCNT` 500/625, at 400 kHz
  `IC_FS_SCL_HCNT`/`LCNT` 93/109, at 1 MHz 31/31 (both with the FAST speed in `IC_CON`),
  `IC_FS_SPKLEN` 1, `IC_SDA_HOLD` at its reset value 1. `IC_CON` is 0x363 at 100 kHz and 0x365
  otherwise (with the bit that holds the bus while the RX FIFO is full; as in the Zephyr build it
  is set only while DMA is on, so after `i2c_dma mode off` transfers use 0x163 and 0x165).
- Transfers that do not use DMA are interrupt driven. A read queues as many requests as the RX FIFO (16 entries) has
  room for and takes the data 4 bytes at a time. A transfer has 1 s; after that it is aborted and
  returns `-ETIMEDOUT`. A target that does not answer (NACK) and other failures give `-EIO` and
  leave the Zephyr build's `<err> i2c_dw: NACK on i2c@40044000` and the like in `dmesg` (rate
  limited). The next transfer works as usual. A bus lock serializes transfers.
- A transfer has one address and one frequency for all its messages. 10-bit addresses are not
  supported. The last message ends with a STOP even with `I2C_M_NOSTOP` (as in the Zephyr build).

### DMA reads (`i2c_dma`)

As with the Zephyr build's `CONFIG_I2C_DW_RP2040_DMA` (the same symbol here, on by default), a
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
  gives `-EIO` until a reboot.
- The DMA is NuttX's own driver (`CONFIG_RP2040_DMAC`, `arch/arm/src/rp2040/rp2040_dmac.c`). The board takes two
  channels from it at boot and keeps them for the reads, so `dma_fallbacks` stays 0 in this build unless a channel's
  setup is refused (still busy). Other code may use the other channels through `rp2040_dmac.c` as usual (what it must
  not do, and the details of the implementation on NuttX's DMA, are in the "I2C0" section of
  `nuttx/drivers/README.md`). `CONFIG_RP2040_SPI_DMA` stays off: the NINA's SPI is polled, as in the Zephyr version.

```text
i2c_dma mode          show the mode (i2c_dma=on or i2c_dma=off)
i2c_dma mode on|off   switch (only while no stream or transfer runs)
i2c_dma stats         DMA transfers, bytes, errors, fallbacks to interrupts (since boot)
```

The command is the one of the Zephyr and FreeRTOS builds, `common/shell/i2c_dma_shell.c`,
run on `zshell.c` (`nuttx/apps/i2c_dma` only builds it as the builtin command's main source), so the
output, help and errors are the same (`dma_transfers=0 dma_bytes=0 dma_errors=0 dma_fallbacks=0`;
anything but `on` or `off` gives `Expected on or off`). As in the Zephyr build, switching only
sets the value: it neither waits for nor stops a transfer in progress. "Only while no stream or
transfer runs" is a rule for the user; the command does not check it. Priority 104, stack 3072.

### The `i2c` tool and the Zephyr build's `i2c` command

The Zephyr build's `i2c` command (Zephyr's shell) is not ported; NuttX's `i2c` tool
(`CONFIG_SYSTEM_I2CTOOL`) takes its place. Its options (`-a` address, `-r` register, `-f`
frequency and so on) stay until changed. The default frequency is 100000 (the Zephyr build's
speed at boot). Addresses, registers and values to write are hexadecimal; the byte count of `dump`
is decimal.

| Zephyr build | NuttX build | Differences |
|---|---|---|
| `i2c scan i2c@40044000` | `i2c dev 03 77` | Probes differently (the Zephyr build writes 1 byte, the NuttX build reads 1 byte). Both see 0x60 and 0x6a. Do not add `-z` (a 0-byte write cannot be sent; each address times out after 1 s) |
| `i2c read_byte i2c@40044000 6a 0f` | `i2c get -a 6a -r 0f` | |
| `i2c read i2c@40044000 6a 10 10` | `i2c dump -a 6a -r 10 16` | Up to 256 bytes (the Zephyr build: 64). The Zephyr build's byte count is hexadecimal |
| `i2c direct_read i2c@40044000 6a 1` | `i2c get -a 6a` | Without `-r` no register is sent |
| `i2c write_byte i2c@40044000 6a 07 5a` | `i2c set -a 6a -r 07 5a` | |
| `i2c write i2c@40044000 6a 07 5a 00` | none | The NuttX tool writes one value at a time (two bytes with `-w 16`) |
| `i2c speed i2c@40044000 3` | `-f 1000000` (kept for later commands) | The Zephyr build changes the bus setting, and `imu start` sets 1 MHz there; the NuttX tool only changes its own setting |
| `i2c recover i2c@40044000` | `i2c reset` | Both only take and release the bus (the board has no recovery routine) |

`i2c verf` writes and reads back, so it changes the sensor's settings. The `i2c` tool runs at
priority 100 (the Zephyr build's shell commands are at the place of 104).

## LSM6DSOX (IMU)

The driver is the Zephyr build's own file, `common/drivers/lsm6dso/lsm6dso.c` and `lsm6dso.h`
(Zephyr's LSM6DSO driver with the FIFO, calibration and zero-rate changes). Its calls into the
system keep Zephyr's names and come from the compatibility layer; only the device at its end is
the NuttX build's own, `nuttx/drivers/lsm6dso/lsm6dso_instance.h` (one `DEVICE_DT_DEFINE()` with the
board's values, in place of the Zephyr build's devicetree instance macros in
`zephyr/app/usb-console/drivers/lsm6dso/lsm6dso_instance.h`). The registers are handled by ST's
`lsm6dso_reg.c` (lsm6dso-pid v3.2.2, fetched into `nuttx/stmemsc/` by `script/setup.sh`; with LF
line ends identical to the one in Zephyr's hal_st). The FIFO stream (the INT1 interrupt and RTIO)
runs on the Zephyr build's own files (`common/drivers/lsm6dso/`) (below). The acquisition service that uses it
(`imu start` and so on) is the next section.

- **Initialization at boot**: the board's bring-up (`nano_bringup()`) initializes the sensor right
  after I2C0 and before the key-value store (NVS) is mounted (in init). That is the Zephyr build's
  order: it initializes its devices before `main()` (`POST_KERNEL`, priority 90) and mounts NVS
  afterwards.
  The first two `dmesg` lines are the Zephyr build's:

  ```text
  [00:00:00.007,154] <inf> LSM6DSO: Initialize device lsm6dsox@6a
  [00:00:00.008,475] <inf> LSM6DSO: chip id 0x6c
  [00:00:00.028,920] <inf> fs_nvs: 2 Sectors of 4096 bytes
  ```

  On failure (a wrong chip ID, an I2C error) the same `<err>` lines as in the Zephyr build are
  logged, the device stays "not ready" and the boot goes on.
- **The initialization steps and values** are the Zephyr build's: select the user register bank
  → `WHO_AM_I` → disable I3C → if the sensor hub master runs, stop it and wait 300 µs → software
  reset and 100 µs → accelerometer power mode, full scale, ODR → gyro power mode, full scale, ODR →
  FIFO bypass → BDU. The settings are those of the Zephyr build's devicetree (with
  `app.overlay`), kept in the board's `include/board.h` as `BOARD_LSM6DSOX_*` (accelerometer ±8 g,
  gyro ±2000 dps, both ODRs off, high-performance mode). After it, `CTRL1_XL`=0x0c,
  `CTRL2_G`=0x0c, `CTRL3_C`=0x44, `CTRL9_XL`=0xe2, and `FIFO_CTRL1`..`4`, `COUNTER_BDR_REG1`/`2`,
  `INT1_CTRL`, `INT2_CTRL`, `CTRL4_C`..`CTRL8_XL` and `CTRL10_C` are 0 (see
  `i2c dump -a 6a -r 00 32`).
- **I2C**: the speed at boot is the devicetree's 100 kHz; `i2c_configure()` (a copy of Zephyr's
  API) changes it to 400 kHz or 1 MHz (the Zephyr build's `imu start` sets 1 MHz). A NuttX I2C
  transfer carries its frequency in each message, so the I2C0 device (the layer's NuttX implementation, `nuttx/drivers/compat`)
  keeps this speed and the driver's transfers use it. The `i2c` tool runs at its own `-f`
  frequency and does not change the driver's speed (the Zephyr build's `i2c speed` changes the
  bus). A register read is one transfer (write the register number → RESTART → read), a write is
  one message with STOP: the shape of Zephyr's `i2c_burst_read_dt()` and `i2c_write_dt()`. FIFO
  reads are 32 bytes or more and use DMA.
- **Single readings** (`sensor_sample_fetch()`, `sensor_channel_get()`), **attributes** (ODR and
  full scale through `sensor_attr_set()`), the functions that start, read and stop the FIFO,
  reading the sensor's timestamp, the stored calibration and the on-demand gyro zero
  (`lsm6dso_cal_zero()`) behave as in the Zephyr build. The ODR becomes the lowest step at or
  above the request (12, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz); the accelerometer full
  scale is m/s² rounded to g and only 2, 4, 8 or 16 g; the gyro full scale is rad/s rounded to
  dps and only 125, 250, 500, 1000 or 2000 dps (anything else is `-EINVAL`). While the FIFO is in
  use, single readings and attribute changes are `-EBUSY`.
- **FIFO stream**: `lsm6dso_stream.c` and `lsm6dso_fifo_irq.[ch]` are the same files as the
  Zephyr build's (`common/drivers/lsm6dso/`). As there, a stream starts with `sensor_stream()`; a rising edge of INT1
  (GPIO24) → the system work queue (`sysworkq`) → the FIFO worker (`imu_fifo_worker`) reads the
  FIFO and completes an RTIO request. Below the watermark it looks again every 20 ms;
  `lsm6dso_fifo_request_stop()` delivers the rest as a last partial batch and ends with
  `-ECANCELED`. The Zephyr kernel parts it needs (work queues, semaphores, `k_poll()`), RTIO, GPIO
  and the sensors' async API are in the compatibility layer (`common/compat/` and `nuttx/drivers/compat/`); the board's
  bring-up starts `sysworkq` and `RTIO WQ 0` with `zephyr_compat_initialize()`.
- The copies of Zephyr's API for the callers (`struct device`, `struct sensor_value`, the channel
  and attribute enumerations, `sensor_*()`, part of the I2C API, the kernel, RTIO, GPIO) are in the
  compatibility layer (its OS-independent part in `common/compat/`, its NuttX implementation in
  `nuttx/drivers/compat/`). See the "LSM6DSOX" and "Zephyr API compatibility layer" sections of
  `nuttx/drivers/README.md`.

## Sensor command (`sensor`)

The command itself is the Zephyr build's sensor shell, the same file
`common/drivers/sensor_shell/sensor_shell.c` (Zephyr v4.5.0's `drivers/sensor/sensor_shell.c` with
its fixes and the LSM6DSO's FIFO attributes). The copies of Zephyr's
`drivers/sensor/sensor_shell_stream.c` and `sensor_shell.h` are in the shared part of the
compatibility layer (`common/compat/`; the Zephyr build uses Zephyr's), the NuttX part in
`nuttx/apps/sensor/`.

- `sensor_shell.c` first includes the header of each OS, `sensor_shell_os.h`. The NuttX one
  (`nuttx/apps/sensor/`) includes `sensor_shell_nuttx.h` (the values of the Zephyr build's
  `CONFIG_SENSOR_SHELL_*`, and the log through `app_log.h`) and turns the two lines with which
  `sensor get` waits for its result to be printed (`sensor_shell_os_get_arm()`,
  `sensor_shell_os_get_wait()`; see "Where the output goes" below) into the functions of
  `sensor_shell_nuttx.c`. In the Zephyr build's header of the same name
  (`zephyr/app/usb-console/drivers/sensor_shell/`) the two lines do nothing. The log level
  `CONFIG_SENSOR_LOG_LEVEL` is in the prelude ("Zephyr API compatibility layer" in
  `nuttx/drivers/README.md`).
- `sensor_shell_stream.c` (a copy): a leading comment and the include of the header of each OS,
  `sensor_shell_os.h`, and one line, the callback of the processing thread, which is the OS's
  (`sensor_shell_os_processing_callback()`; in the NuttX build the original callback followed by
  telling a waiting `sensor get`, in `sensor_shell_nuttx.c`).
- `sensor_shell.h` is an unchanged copy.

The command table (`SHELL_STATIC_SUBCMD_SET_CREATE()`, `SHELL_DYNAMIC_CMD_CREATE()`,
`SHELL_CMD_REGISTER()`) is the Zephyr build's text too. The compatibility layer's `<zephyr/shell/shell.h>` and
`common/shell/zshell.c` (with the NuttX part of the terminal backends,
`nuttx/apps/subcmd/zshell_os_impl.h`) make it the builtin command `sensor` with the Zephyr shell's rules
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
  Zephyr build). The syntax, the output formats (`channel type=... index=... shift=...
  num_samples=... value=...ns (...)`, `%.6f` in `attr_get`) and the error messages are the Zephyr
  version's. Error numbers are NuttX's (for example `ENOTSUP` is 138 here, 134 in the Zephyr
  version).
- Device names: `get`, `attr_set`, `attr_get` and `trig` take the node name `lsm6dsox@6a` or the
  node label `lsm6dsox`. `stream` takes the node name `lsm6dsox@6a` only (`device_get_binding()`,
  as in the Zephyr build). The device listed in the help is `lsm6dsox@6a` too. Wildcards (`*`,
  `?`) expand to channel names only after the node name (for example
  `sensor get lsm6dsox@6a gyro_?`; as in the Zephyr build).
- The attributes are `fifo_watermark` (1 to 256, default 32), `fifo_timestamp_decimation` (0, 1, 8
  or 32, default 8), and `sampling_frequency` and `full_scale` (see "LSM6DSOX (IMU)"). They are
  refused with `-16` (`-EBUSY`) while a stream runs (`sensor stream`, `imu start`). `imu start`
  overwrites the watermark with the KV value (default 128). After boot the ODR is off, so
  `sensor stream` ends with `Read failed: -22` until `sampling_frequency` is set (as in the Zephyr
  version).
- It cannot run together with `imu start`: `imu start` during a stream says
  `The IMU FIFO is in use (sensor stream?)`, and `sensor stream` and `sensor get` during a
  measurement say `Read failed: -16` (the driver is a copy of the Zephyr version's, and `imu_stream.c`
  is the Zephyr build's file).
- The calibration applies to the physical values of `sensor get` and `sensor stream`. During
  `sensor stream` the gyro bias is updated at rest by itself (`tracked at rest` in `imu cal`).
- `trig` gives `Error while setting trigger 1 on device lsm6dsox (-38)` (`-ENOSYS`): as in the Zephyr
  build, the driver has no trigger (`CONFIG_LSM6DSO_TRIGGER_NONE`).
- Tab completes command names only (nsh). The Zephyr build also completes subcommands, device,
  channel and attribute names.

**Where the output goes**: in the Zephyr build, the results of `sensor get` and `sensor stream`
are written by the processing thread (`sensor_shell_processing_tid`, priority 0, 3072-byte stack)
to the shell that ran the command (USB or Telnet). Here the same thread (priority 118, 3072-byte
stack, started by init at boot) writes them.

- It writes to a shell backend of the compatibility layer (`zshell.c`): one per terminal that a
  command's stdout is (`/dev/ttyACM0` for USB, `/dev/telnet0` for Telnet). For each line the
  processing thread opens that terminal, writes the line with one `write()` and closes it again.
  The command's own output goes to its stdout and stderr.
- `sensor get` returns to the prompt once the processing thread has written the result (it waits at
  most 2 s). The Zephyr build prints the result after the prompt and draws the prompt again;
  nsh cannot draw its prompt again, hence the wait.
- The output of `sensor stream` goes on after the prompt has come back. nsh does not draw the
  prompt or the line being typed again, so the output comes in the middle of what is typed (the
  Zephyr build draws the prompt and the line again after each line). `sensor stream
  lsm6dsox@6a off`, from any terminal, stops it (so does `reboot`).
- When the Telnet client goes, the output is dropped until the next client connects (there is no
  terminal device to open) and then goes to that client (as with the Zephyr build's Telnet
  backend). The processing thread has the terminal open only while it writes, so it does not hold
  up the next connection (no `Telnet client already connected.`).
- While the PC has the USB port closed, the output is dropped like all other output (see "Shell").

## IMU acquisition (`imu`)

The acquisition service is `common/imu/imu_stream.c`, the Zephyr build's file (`nuttx/apps/imu/`
builds it as the main source of the builtin command `imu`). The compatibility layer
(`common/compat/` and `nuttx/drivers/compat/`, which keep Zephyr's names) provides the calls into the system, and the
constants, branches, order, waits, counters and output text are the Zephyr build's. The command
registration at the end (`SHELL_CMD_REGISTER()`) is the Zephyr build's text too; the
compatibility layer makes it the `main()` of the builtin command `imu` (see "Subcommands of the
commands").

```text
nsh$ imu
imu - IMU RAW streaming
Subcommands:
  start   : Start streaming to the MQTT server (settings: kv imu.*)
  stop    : Stop streaming
  cal     : Show the conversion used by sensor get/stream: [reload|zero]
  status  : Show counters and errors of the current or last run
```

- `imu start`, `imu stop` and `imu status` have the Zephyr build's syntax and output (`imu` and
  `imu status` while idle are character for character the output recorded from the Zephyr
  version; the items of `imu status` have the same order and columns). The settings come from the
  same KV keys (`imu.host`, `imu.port`, `imu.device`, `imu.odr`, `imu.watermark`).
- Acquisition runs in the thread `imu_stream_tid` (Zephyr priority 9 → 109, a static 2048-byte
  stack). Zephyr starts `K_THREAD_DEFINE()` threads before `main()`; here init starts it at the
  beginning of the part that does what `main()` does (`K_THREAD_DEFINE_START()`). The `imu`
  command itself is a builtin at the shell's priority (104).
- The session ID is the chip ID in its first 8 bytes (the USB serial number; the bytes of Zephyr's
  `hwinfo_get_device_id()`), then the KV value `imu.session` (4 bytes, big-endian, incremented and
  stored by every `imu start`), then zeros. The Zephyr build's counter is not carried over; the
  NuttX build starts at 1.
- init calls, in the order of the Zephyr build's `main()`, `kv_store_init()` →
  `imu_cal_init()` (on failure `Could not set up the IMU calibration: <value>`) →
  `time_sync_init()` → `wifi_profile_init()`.

### Calibration (`imu cal`)

`common/imu/imu_cal.c` is the Zephyr build's own file (as are the value format, `imu_cal_codec.c` and
`imu_cal.h`, in `common/imu/`). The KV keys
(`imu.cal.accel.<ug/LSB>`, `imu.cal.gyro.<udps/LSB>`) and values (56 bytes), using only the key of
the current sensitivity, and the syntax, output and error messages of `imu cal`,
`imu cal reload` and `imu cal zero` are the Zephyr build's (the Zephyr build's README,
"Applying a calibration", and the design document, section 10).

```text
nsh$ imu cal
conversion:
  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)
  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)
  gyro bias: 7.65 -4.25 -0.81 LSB (stored, 0 rejected)
```

- init calls `imu_cal_init()` after `kv_store_init()`, which gives the driver the function that
  looks up the calibration for a sensitivity (where the Zephyr build's `main()` does). The
  driver reads the KV store when it is given the function, on `imu cal reload`, and when a range
  is set.
- The calibration applies to the driver's physical values only (single reads,
  `sensor_channel_get()`, and the FIFO decoder), **not to the RAW sent over MQTT** (as in the
  Zephyr build). `sensor get` and `sensor stream` show it (see "Sensor command (`sensor`)").
- `imu cal zero` is refused during a measurement with
  `A measurement is running; it tracks the gyro bias itself`. `imu cal` and `imu cal reload` work
  during a measurement. A gyro bias taken again is kept in RAM only.
- Errors go to stderr and the exit status is 1 (as with `kv`; 0 on success). With two or more
  arguments it prints `cal: wrong parameter count` and the help (as the Zephyr shell does).

### MQTT transport

`common/imu/imu_raw_mqtt.c` and `imu_raw_mqtt.h` are the Zephyr build's own files (the
`<zephyr/net/net_ip.h>` that `imu_raw_mqtt.h` includes is the compatibility layer's, which includes
`<netinet/in.h>`). It is built with `CONFIG_NANO_IMU_RAW_MQTT` (on by
default); without it the stub `imu_raw_mqtt_null.c` stands in (it drops and counts every batch, as
the Zephyr build does without a network). The topics (`imu/<device>/raw`, `status`, `info`)
and QoS (0 for raw and status, 1 with retain for info), the client ID (`imu-<device>`), the
keep-alive (60 s), no clean session, the order of sending (on every connection the info, the
status, then the queued batches; the status every second and when the state changes, input
checked every 100 ms, a 5 ms sleep while the queue is empty), the reconnection interval (1 s),
the CONNACK wait (5 s), the wait for the first connection in `imu start` (up to 3 s), dropping and
counting the batches while disconnected, not resending a batch whose publish failed, the up to
0.5 s that `imu stop` gives the last status, and the items of `mqtt` in `imu status` with the
events that advance them are the same as in the Zephyr build.

- Thread `imu_raw_mqtt_tid` (Zephyr priority 11 → 107, a static 2048-byte stack). init starts it
  at the start of its start-up work; it waits for `imu start`.
- The MQTT client is a copy of the part of Zephyr's MQTT library (`subsys/net/lib/mqtt/`) that is
  used (`common/imu/mqtt_min.c`, with the API of the compatibility layer's
  `<zephyr/net/mqtt.h>`; its socket layer is `common/imu/mqtt_min_sock.c`): MQTT 3.1.1 over TCP only, with the bytes it sends, the defaults and the
  way it reads in pieces of Zephyr's library (the host test `mqtt_min` runs the same test on
  Zephyr's own library as well). A PUBLISH goes as its fixed header and its body in two vectors
  of one `sendmsg()`, which the NINA driver joins into one send command (as in the Zephyr
  version).
- The socket stays blocking (as in the Zephyr build). A connection waits up to 10 s, a send
  that finds no room up to 10 s, before failing (the NINA driver's deadlines; "Sockets (usrsock)"
  in `nuttx/drivers/README.md`).
- The compatibility layer's `zsock_socket()`, `zsock_poll()` and `zsock_recv()` are NuttX's
  `socket()`, `poll()` and `recv()`, so the socket is an ordinary usrsock socket: `nina_event` asks
  the NINA about it (every 20 ms, or at the IRQ line's edges with the socket events) and reports
  what it finds as events ("Sockets (usrsock)" in `nuttx/drivers/README.md`). The Zephyr build asks
  the NINA only from `imu_raw_mqtt_tid` (one `CMD_POLL` per input check, every 100 ms).
- The input check (`zsock_poll()`, every 100 ms, timeout 0) and the CONNACK wait (`zsock_poll()`,
  5000 ms) answer from the events usrsock holds; the CONNACK wait ends at the event. A failed
  connection that the NINA reports (POLL_ERROR, or a failed poll) is `ZSOCK_POLLHUP` (usrsock's
  `REMOTE_CLOSED`; the Zephyr build gives `ZSOCK_POLLERR`); `imu_raw_mqtt.c` ends the connection on
  either. The reads (`zsock_recv()` with `ZSOCK_MSG_DONTWAIT`) reach the driver only while usrsock
  holds `RECVFROM_AVAIL`; the CONNACK, read as 2 bytes and then 2, gets its second piece because
  the driver keeps `RECVFROM_AVAIL` after a receive that filled its request (the next read then
  asks the NINA, as each read of the Zephyr build does).
- Difference from the Zephyr build: the `lock_wait_*` and `publish_*` times are the 125 MHz
  SysTick cycles (`k_cycle_get_*()`, which fall behind while streaming) in microseconds in the
  Zephyr build, and the TIMER (1 µs) here (the compatibility layer's `k_cycle_get_*()`).

## Time synchronization (`ntp`)

`common/imu/time_sync.c` is the Zephyr build's file (`nuttx/apps/imu/` builds it). The compatibility
layer provides the calls into the system (sockets, `net_mgmt` events, the realtime clock) under
Zephyr's names. `time_sync_mono_us()` reads the TIMER's TIMERAWH and TIMERAWL with `sys_read32()`,
as written for the Zephyr build (the layer's `<zephyr/sys/sys_io.h>` and the `timer` node of its
`<zephyr/devicetree.h>`; the same read as the board's `nano_mono_us()`). The Kconfig options
that the file uses (`CONFIG_NINA_NTP_*`) are the NuttX build's options of the same names. The
layer makes the command registration at the end
`z_shell_main_ntp()`, which the `main()` of the builtin command `ntp` (`nuttx/apps/ntp/`) calls
(see "Subcommands of the commands"). The NTP
packet and the checks of the reply, where t1 (right before the send) and t4 (right after the
reply is received) are read, the estimate (`time_offset.c`, an unchanged copy: the reply with the
shortest round trip of the last 8 is used), the estimate's expiry after 60 s without a reply, the
clock that follows it at up to ±500 ppm without going back, moving the realtime clock only
forward, the query interval, wait and retry, the counters and the output of `ntp status` are the
Zephyr build's. It is built with `CONFIG_NANO_TIME_SYNC` (on by default); without it the stub
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
- The server is `CONFIG_NINA_NTP_SERVER` (as in the Zephyr build; empty by default: the default
  gateway), the port `CONFIG_NINA_NTP_PORT` (123), the interval
  `CONFIG_NINA_NTP_INTERVAL_MS` (2000 ms). A reply is awaited for 250 ms; none is counted in
  `timeouts`, one that fails the checks in `rejected`, and the next query follows at the next
  interval. When there is no address, no socket or the send fails, the error goes to `last_error`
  and the thread tries again after 5 seconds.
- Waiting for the reply: the file calls `zsock_recv()` with `ZSOCK_MSG_DONTWAIT` about every
  millisecond. In the Zephyr build its NINA driver asks the module for the socket's state each time
  (`CMD_POLL`) and reads what has arrived. Here it is NuttX's `recv(MSG_DONTWAIT)`, which usrsock
  answers `EAGAIN` itself until it holds the readable mark (`RECVFROM_AVAIL`); `nina_event` raises
  the mark from its polls of the socket (the first right after the first send, which creates the
  NINA's socket; then every 20 ms, or at the IRQ line's edge with the socket events), and the
  receive then reads the datagram without another poll (the driver uses that poll's answer). So t4
  is read when `nina_event` has seen the reply: with the socket events, soon after the reply's edge;
  without them, up to 20 ms late, which enters the round trip and half of it the offset (the
  estimate uses the shortest round trip of the last 8). See "Sockets (usrsock)" in
  `nuttx/drivers/README.md`.
- Difference from the Zephyr build: `k_uptime_get()` (the 250 ms wait, the 60 s check,
  `last_reply_ms_ago`) comes from the TIMER in the NuttX build. The Zephyr build's is the
  SysTick kernel clock, which falls behind while streaming.
- The realtime clock (`CLOCK_REALTIME`) is set to the estimate only when it is 20 ms or more behind
  (counted in `realtime_clock_steps`); when it is ahead, it is left alone. It is good for dates and
  times of day; precise times come from `time_sync_unix_us()`. nsh's `date` shows and sets the
  realtime clock (the Zephyr build's `date`; `CONFIG_NSH_DISABLE_DATE` is cleared, for the Zephyr
  version's `CONFIG_DATE_SHELL=y`). It shows UTC, in nsh's format:

| Zephyr build | NuttX build |
|---|---|
| `date get` | `date` (e.g. `Sat, Nov 14 03:23:03 2026`) |
| `date set 2026-11-14 03:23:03` | `date -s "Nov 14 03:23:03 2026"` |

## System clock

The kernel clock is tickless with a 100 µs tick (the same tick as the Zephyr build's
`CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000`, `CONFIG_TICKLESS_KERNEL=y`). There is no periodic
interrupt: ALARM0 of the RP2040 TIMER (64-bit, 1 MHz, from the crystal) is set to the time of the
next wait that ends (`nano_oneshot.c` and `nano_timerisr.c` in
`nuttx/boards/arduino-nano-rp2040-connect/src/`, and NuttX's `drivers/timers/arch_alarm.c`).
ALARM1 to ALARM3 are not used. The settings are `CONFIG_NANO_TICKLESS` (the board `Kconfig`, on by
default), `CONFIG_SCHED_TICKLESS=y` and `CONFIG_USEC_PER_TICK=100`.

- A wait is rounded up to whole ticks and one tick is added (`usleep(1000)` takes 1000 to 1100 µs
  plus the wake-up delay), the same rule as the Zephyr build's `k_sleep(K_MSEC(1))`.
- `clock_gettime()` with `CLOCK_MONOTONIC` or `CLOCK_REALTIME` returns the TIMER value directly, in
  µs (`clock_getres()` gives the tick, 100 µs). The Zephyr build's kernel clock is SysTick on the
  CPU clock, a different source. In both versions the application reads the TIMER directly as its
  time base.
- The board's monotonic clock is `nano_mono_us()` (`<arch/board/nano_timer.h>`). It reads the
  TIMER's TIMERAWH, TIMERAWL and TIMERAWH again until the two high words agree (the read of the
  Zephyr build's `time_sync_mono_us()`). It has no side effect and takes no lock, so the kernel,
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
the Zephyr build's `CONFIG_CPU_LOAD` (`CONFIG_CPU_LOAD_BACKEND_IDLE_HOOK`) returns.

- The idle loop (`up_idle()`, the board's own with `CONFIG_ARCH_IDLE_CUSTOM=y`) disables interrupts,
  reads the TIMER, waits for an interrupt with `WFI`, reads the TIMER again and adds the time to
  the idle time, and then enables interrupts. The handler of the interrupt that woke the CPU runs
  after that, so the time spent in interrupts that arrive while idle counts as load (as in the
  Zephyr build). `WFI` on the Cortex-M0+ returns on a pending interrupt even while interrupts are
  disabled, so an interrupt that arrives between disabling them and `WFI` does not leave the CPU
  asleep.
- The value is `1000 × (total − idle) / total` since the last call with `reset=true` (since boot
  before the first), rounded down, in tenths of a percent; 0 if no time has passed. A call with
  `reset=true` also returns the value up to that moment and then starts the next window.
- The time base is the TIMER (1 µs); the Zephyr build converts CPU clock cycles to µs. Each idle
  window is off by up to ±1 µs, without bias; even with many windows the effect on the result is
  estimated to be well below 0.1‰.

### CPU time per thread (the CPU column of `ps`)

This uses the per-thread CPU time that NuttX's own critical section monitor
(`CONFIG_SCHED_CRITMONITOR`) keeps (`CONFIG_SCHED_CPULOAD_CRITMONITOR`; patches `nuttx/0007` and
`nuttx/0008`). Its maximum-time monitors (`CONFIG_SCHED_CRITMONITOR_MAXTIME_*`) stay at their
default of −1 and are not used. `cpu_load_get()` above (`cpu_active` in `imu status`) is a separate
mechanism; turning this on does not change how it measures.

- At every thread switch, the time since the previous switch is added to the thread being switched
  out. The time base is the board's perf counter (`src/nano_perf.c`, `CONFIG_NANO_PERF`: the TIMER,
  1 µs; the Cortex-M0+ has no cycle counter). The Zephyr build's `CONFIG_SCHED_THREAD_USAGE` also
  adds CPU cycles only at a switch (PendSV).
- In both versions the time spent in interrupt handlers goes to the thread that was interrupted
  (interrupts while idle go to the idle thread). So the idle thread's share does not match what
  `cpu_active`, which counts interrupt handling as load, leaves (100 % − `cpu_active`).
- What can be read:
  - The CPU column of `ps` and `/proc/<tid>/loadavg`: the thread's share, in 0.1 % steps. It is a
    moving average: whenever the total exceeds `CONFIG_SCHED_CPULOAD_TIMECONSTANT` (2 s), every
    thread's count is halved (with the CPU always running, about once a second, so the last 1–2
    seconds weigh most). `ps` reads one row at a time, so the rows may not add up to 100 %, and the
    shell running `ps` counts too.
  - `/proc/cpuload`: 100 % − the idle thread's share (the same moving average).
  - `/proc/<tid>/critmon`: `longest continuous run,total run time since boot` (seconds, to the
    nanosecond digit). The total only grows, so the difference of two readings gives the share over
    an interval (what `Total execution cycles` of the Zephyr build's `kernel thread list` gives).
    The longest run goes back to 0 at each read.
- The CPU of the Zephyr build's `kernel ps` and the FreeRTOS build's `ps` is the share since the previous
  run of the command (in 0.1 % steps, since boot for the first; type it at the start and at the end of
  the interval of interest), and `Total execution cycles` of the Zephyr build's `kernel thread list` a
  cycle count since boot. The CPU column of
  NuttX's `ps` is the moving average above.
- nsh's `top` (which shows the same values over and over) is turned off, as the Zephyr build has
  no such command.
- The task list of a PANIC report shows the CPU column too.

## Thread priorities and stacks

Priorities map as `NuttX = 118 − Zephyr`; `common/compat/priorities.h` holds the constants
(`NANO_PRIO_*`) and the stack sizes (`NANO_STACK_*`) (the mapping `NANO_PRIO_FROM_ZEPHYR()` and the
NuttX-only threads are in `nuttx/drivers/include/priorities_os.h`). The Zephyr build's application threads
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
| init, `nina_pt_rx`, `nina_pt_tx` (in the NINA passthrough mode only; NuttX's init and two kernel threads) | – | 120 | 2048 |
| Shell (nsh session, commands; `nsh` on USB, `nsh_telnet` on Telnet) | 14 | 104 | 3072 |
| `telnetd` (accepts Telnet clients; the Zephyr build's `net_socket_service`) | 15 → 14 | 104 | 2048 |
| `iperf_traffic`, `iperf_report` (`iperf`'s pthreads; the Zephyr build's `zperf_work_q`) | 15 → 14 | 100, 101 | 4096, 4096 |

- Zephyr's cooperative threads (negative priorities: `sysworkq`, `net_mgmt` and others) map above
  118, but NuttX has no cooperative threads that other threads cannot preempt; a thread of higher
  priority preempts them. In the port so far these are `sysworkq` and `net_mgmt` (both 119), and
  no thread has a higher priority. The two at 119 take turns by time slicing (Zephyr's cooperative
  threads only switch when one yields).
- The `K_THREAD_DEFINE()` threads (`nina_scan_tid`, `nina_connect_tid`, `nina_monitor_tid`,
  `imu_stream_tid`, `imu_raw_mqtt_tid`, `time_sync_tid`, `wifi_reconnect_tid`,
  `sensor_shell_processing_tid`) are started by init at the start of its start-up work, as the Zephyr build
  starts them before `main()`.
- The stacks of `imu_stream_tid`, `imu_raw_mqtt_tid`, `time_sync_tid`, `imu_fifo_worker`, `sysworkq`, `RTIO WQ 0`, `net_mgmt`, the
  `nina_*_tid` threads, `wifi_reconnect_tid` and `sensor_shell_processing_tid` are static arrays
  as in Zephyr (`K_THREAD_DEFINE()`, `K_THREAD_STACK_DEFINE()`, NuttX's
  `kthread_create_with_stack()`). `ps` shows them smaller by the area NuttX puts at their start
  (2048 → 2008, 2000 for `imu_stream_tid`; 1024 → 976).
- `CONFIG_PRIORITY_INHERITANCE=y`. Mutexes inherit priority; semaphores do not by default (NuttX
  13.1.0's `sem_init()` sets their flags to 0), so semaphores used for signalling stay without
  inheritance.
- Interrupt stack (`CONFIG_ARCH_INTERRUPTSTACK=2048`, the size of the Zephyr build's
  `CONFIG_ISR_STACK_SIZE=2048`): threads run on the process stack (PSP), interrupt handlers on the
  main stack (MSP), so handlers do not use the stack of the thread they interrupt. Only the 32 bytes
  that the hardware saves (and the save area of a context switch) go on that thread's stack, as in
  the Zephyr build. `ps` does not show the interrupt stack; `irqstack`
  does (in the format of the last line of the Zephyr build's `kernel thread stacks`; the value
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
  goes to the RAM log; the board then restarts by itself, see "Log (`dmesg`)"). The interrupt stack itself has no sentinel, as in the
  Zephyr build. The checks run
  in two scheduler instrumentation hooks (`sched_note_suspend()` at a switch, and
  `sched_note_irqhandler()`, which `irq_dispatch()` calls around each handler;
  `src/nano_stackcheck.c`). On the Cortex-M0+ peripheral interrupts do not go through
  `arm_doirq()` (they use `exception_direct()`), so the switch hook alone does not see the end of
  an interrupt. All interrupts run at one priority and never nest, so every handler exit is the
  end of a non-nested interrupt in Zephyr's terms. NuttX's `CONFIG_STACKCHECK_MARGIN` is not used:
  it checks at switches only and needs `CONFIG_DEBUG_ASSERTIONS` (every DEBUGASSERT).
- `ps` shows stack use in its STACK, USED and FILLED columns, and each thread's share of CPU time in its CPU column ("CPU load").

## The C library's `memcpy` and `memset`

`CONFIG_NANO_ROM_MEM_OPS` (board `Kconfig`, on by default) selects `CONFIG_RP2040_ROM_MEM_OPS` of the NuttX
patch `nuttx/0010`, which makes `memcpy`, `memset`, `__aeabi_memcpy*` and `__aeabi_memset*` the RP2040 boot
ROM's routines, split as the Pico SDK's `pico_mem_ops` does (the Zephyr build does the same with a local patch
of Zephyr's SoC). It selects `LIBC_ARCH_MEMCPY` and `LIBC_ARCH_MEMSET`, which leave out the byte loops of
`libs/libc/string`. The code is `rp2040_rom_mem_ops.S` and `rp2040_rom_mem_lookup.c` in the chip's directory.
Each routine is looked up in the ROM's function table on its first call and kept in `.bss` (a value left in
RAM from before a reset is not used), so neither can be called before `.bss` is cleared. GCC turns the loops
of `rp2040_start.c` that clear `.bss` and copy `.data` into calls of `memset` and `memcpy`, so the patch puts
`START_LOOP_BARRIER()` (an empty asm with a memory clobber) in them to keep them loops, and `build.sh` checks
the result.

## Integer division

With `CONFIG_NANO_HW_DIVIDER` (board `Kconfig`, on by default), the integer divisions the compiler emits
(`__aeabi_idiv`, `__aeabi_idivmod`, `__aeabi_uidiv`, `__aeabi_uidivmod`, `__aeabi_ldivmod`, `__aeabi_uldivmod`)
run on the SIO hardware divider instead of libgcc's shift-and-subtract loops. The Pico SDK's `pico_divider`
(`divider_hardware.S`, from `nuttx/pico-sdk`, built with the minimal headers of the board's `src/pico_sdk`) is
linked in with `ld --wrap` in the board's `scripts/Make.defs`.

- The routines do not mask interrupts (`PICO_DIVIDER_DISABLE_INTERRUPTS` 0). When an interrupt handler
  divides in the middle of a division, the routine sees DIRTY and saves and restores the divider.
- When a task is switched out in the middle of a division, `CONFIG_RP2040_HW_DIVIDER` of the NuttX patch
  `nuttx/0009` (selected by `CONFIG_NANO_HW_DIVIDER`) saves it. Every switch on the ARMv6-M goes through
  `arm_doirq()`, which, when the task changes, keeps the old task's divider, only when DIRTY is set, in 20
  bytes of `struct xcptcontext` and restores the new one's (a task that was not dividing comes back with DIRTY
  clear). It is also saved before the last switch of `arm_sigdeliver()`. The FreeRTOS build (the Pico SDK's
  port) saves four words at every switch.
- A division by zero gives the Pico SDK routines' results (as in the Zephyr and FreeRTOS builds): for 32 bits
  a saturated quotient (0xffffffff, 0x7fffffff or 0x80000000; 0 for 0 / 0) and a remainder of 0 (libgcc gave a
  quotient of 0 and the dividend); for 64 bits the same as libgcc's.
- The routines use only the SIO and the stack, so they work from the start of `__start()`, in interrupt
  handlers and in the fatal error handling.
- Test images (2026-11-18, on the board): with the saving, no mismatch in 250 seconds and about 11.28 million
  divisions; without it, 10 in 40 seconds.

## Signal handlers

The NuttX build's code registers no signal handler (no `sigaction()` or `signal()`, no SIGEV_SIGNAL timer, no
`kill()` or `pthread_kill()` to run a handler). Waits are written with semaphores, sleeps and blocking I/O.

A load where task switches (wake-ups every 1 ms and priority changes) overlap a running signal handler froze
the board within seconds, with nothing in the log (seen on the board, 2026-11-18; the same with an image using
libgcc's division and none of the divider patch's code). The cause has not been found and no fix is applied.
The present code registers no handler, so normal operation does not take this path.

## Host test details

The following files are used by both the Zephyr and the NuttX build (and the FreeRTOS build) and are kept once each in the
repository's `common/` (the NuttX and FreeRTOS builds use the I2C core;
see the table; the layout of `common/` as a whole is in §12 of the design document). The Zephyr
build's `CMakeLists.txt` and the NuttX build's Makefiles
(`apps/imu/Makefile`, `apps/nina/Makefile`, `apps/wifi/Makefile`, `apps/sensor/Makefile`, `drivers/compat/Make.defs`,
`drivers/lsm6dso/Make.defs`, `drivers/nina/Make.defs`, the board's `src/Make.defs`) refer to them
there. The files that use
Zephyr's API (in `common/drivers/`, the FIFO setup and stream, the RTIO single reads, the NINA's
Wi-Fi and the sensor shell; in `common/imu/`, the acquisition service, the time synchronization,
the calibration, MQTT and the Wi-Fi profile) run on
the compatibility layer in the NuttX build. The NuttX build compiles each file of `common/` (but the NINA core and the I2C core) with
the prelude (`nuttx/drivers/compat/common_prelude.h`), and the few lines that differ between the
builds are in headers of each OS (`sensor_shell_os.h`, `wifi_profile_os.h`, `lsm6dso_instance.h`;
the Zephyr build's under `zephyr/app/usb-console/`, the NuttX build's under `nuttx/apps/` and
`nuttx/drivers/lsm6dso/`) ("Zephyr API compatibility layer" in `nuttx/drivers/README.md`). The
OS-independent part of the compatibility layer is in `common/compat/` too; the NuttX build uses it
with the layer's NuttX implementation (`nuttx/drivers/compat/`; the Zephyr build does not use it).

| Location | Contents |
|---|---|
| `common/imu/` | `imu_stream.c`, `imu_raw_{batch,packet,queue,status,time}`, `imu_tick_clock`, `time_sync.[ch]`, `time_offset`, `imu_cal_codec.c`, `imu_cal.[ch]`, `imu_raw_mqtt.[ch]`, `wifi_profile.[ch]`, `kv_store.[ch]`, `mqtt_min.[ch]` and `mqtt_min_sock.c` (the MQTT client of the builds on the compatibility layer and its socket layer; the Zephyr build does not use them) |
| `common/drivers/lsm6dso/` | `lsm6dso.[ch]` (the driver; the device at its end is `lsm6dso_instance.h` of each build), `lsm6dso_{fifo,cal,zero,fifo_decoder,fifo_hw,fifo_irq}`, `lsm6dso_stream.c`, `lsm6dso_compat.c`, `lsm6dso_fifo_packet.h`, `lsm6dso_fifo_stats.h`, `lsm6dso_attributes.h` |
| `common/drivers/nina/` | The NINA core: at the top `nina_wifi.[ch]`, `nina_sock.[ch]`, `nina_dns.[ch]`, the esptool passthrough `nina_passthrough.[ch]`; in `internal/` `nina_proto.[ch]` and `nina_wait.[ch]`; in `port/` the headers of its boundaries, `nina_os.h` and `nina_bus.h` (the passthrough's `nina_passthrough_bus.h` and `nina_passthrough_os.h`) (the three builds use them; they do not use Zephyr's API and the NuttX build compiles them without the prelude; the boundary is described under "NINA-W102" in `nuttx/drivers/README.md`); the core's bindings in `port/zephyr/` (the Zephyr and FreeRTOS builds) and `port/nuttx/` (this build: `wlan0`, the applications' Wi-Fi management, `nina_os.c`, usrsock, the passthrough's host side `nina_passthrough_os.c`, the RP2040's SPI and lines `nina_bus_rp2040.c`, the passthrough's module side `nina_passthrough_bus_rp2040.c`) |
| `common/drivers/sensor_shell/` | `sensor_shell.c` |
| `common/drivers/stmemsc/` | `stmemsc.h`, `stmemsc_i2c.c`, `stmemsc_mdelay.c` (a copy of the I2C part of Zephyr's stmemsc, used by the NuttX and FreeRTOS builds and compiled without the prelude; the Zephyr build uses Zephyr's) |
| `common/drivers/i2c_dw/` | The I2C core `i2c_dw.c` (I2C0, and I2C1 with `CONFIG_I2C_DW_I2C1`, off by default) with its definitions `i2c_dw.h`, `i2c_dw_registers.h`, `i2c_dw_rp2040.h` and its DMA reads `i2c_dw_rp2040_dma_priv.h`, with the DMA command words (the Zephyr build has the same function in patch 0003) (the file names of Zephyr's patched driver), the declarations of the `i2c_dma` functions `i2c_dw_rp2040_dma.h`, the headers of its boundaries, `i2c_dw_os.h` and, for the DMA reads, `i2c_dw_dma_os.h` (the NuttX and FreeRTOS builds use them; they are compiled without the prelude, and the OS's part is the board's `src/i2c_dw_os_impl.h`, `i2c_dw_instance.h` and `i2c_dw_dma_os.c`; the boundaries are described under "I2C0" in `nuttx/drivers/README.md`) |
| `common/compat/` | the OS-independent part of the compatibility layer (the headers of Zephyr's API `zephyr/...`, among them the `i2c_dma` state `zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`, which includes the I2C core's `common/drivers/i2c_dw/i2c_dw_rp2040_dma.h`, `zephyr_compat.h`, work items, RTIO, the block pool, the device list, `gpio0`, `net_mgmt`, the Wi-Fi requests, the sensors' default iodev) and the copies of Zephyr's libraries that the builds on the layer use (`sensor_shell_stream.c`, `sensor_shell.h`, `wifi_credentials.c`, `wifi_credentials_backend_settings.c`, `wifi_credentials_internal.h`), and boundary headers (`priorities.h`, `cpu_load.h`, `nano_cpuload.h`, `nano_gpio.h`); the Zephyr build does not use it |
| `common/shell/` | `subcmd.[ch]` (the subcommand tables), `zshell.c` (the handling of commands written with `<zephyr/shell/shell.h>`: word matching, wildcards, argument count checks, help) and its boundary to the OS, `zshell_os.h` (the NuttX build's part is `nuttx/apps/subcmd/zshell_os_impl.h`); compiled without the prelude; the Zephyr build does not use them. Also `i2c_dma_shell.c` (the `i2c_dma` command of the three builds, written on Zephyr's shell API; the NuttX build's `nuttx/apps/i2c_dma` builds it with the prelude), and `nina_shell.c` (the `nina` command of the Zephyr and FreeRTOS builds; the NuttX build has its own, `nuttx/apps/nina`) |
| `common/log/` | `log_ring.[ch]` (the RAM log's ring, line prefix and rate limit), `app_log.h` (the `LOG_*` macros; the NuttX part is `nuttx/drivers/include/app_log_os.h`), `log_ram.h` (the RAM log's functions; in the NuttX build `nuttx/drivers/log/log_ram.c`), `fault_record.[ch]` (the record of a fatal error: its format, CRC-32, the copy in the watchdog's scratch registers and its lines; only the FreeRTOS build uses it); the Zephyr build does not use it |
| `common/tests/` | the 37 tests of the files above and the check of the copies, `zephyr_copies` (run by `common/tests/run.sh`, which the `host_tests.sh` of the three builds call first; the layer's `zephyr_rtio`, `zephyr_work` and `mqtt_min` on the fake kernel, the NINA core's `nina_proto`, `nina_sock`, `nina_wifi`, `nina_dns` and `nina_passthrough` on fakes of its boundaries defined by the tests, and `nina_core_independent`, each core file compiled with the C library and its own headers only); `stream_stop.c` includes `../drivers/lsm6dso/lsm6dso_stream.c` (with the include guard of `lsm6dso.h` defined first, so nothing of that header is used; `stream_stop.c` provides the definitions it needs) |
| `common/tests/fakes/` | stand-ins for the Zephyr and ST headers the tests need (`zephyr/drivers/sensor.h`, `zephyr/drivers/sensor_clock.h`, `zephyr/drivers/i2c.h`, `zephyr/rtio/rtio.h`, `lsm6dso_reg.h`) |
| `common/tests/zephyr_fakes/` | the compatibility layer's fake kernel: the OS headers (`zephyr_os.h`, `zephyr_os_spinlock.h`, `zephyr_os_atomic.h`, `zephyr_rtio_workq_os.h`, `zephyr/sys/__assert.h`) and their functions (`zephyr_kernel_host.[ch]`: one thread; the test runs the work queues and fires the timers); the NuttX build's tests use it too |

The tests of `common/` sources that run on the NuttX build's implementation of the compatibility
layer (sockets, I2C, the shell and so on) or the board's code with the fake kernel
(`common/tests/zephyr_fakes/`), `lsm6dso_driver`, `imu_cal`, `imu_raw_mqtt`, `sensor_shell`,
`time_sync`, `wifi_cred` and `zephyr_compat_misc`, are in `nuttx/tests/`;
`nuttx/script/host_tests.sh` builds them with the prelude. It also builds `common/tests/mqtt_min.c`
on Zephyr's MQTT library (which needs the Zephyr tree and the NuttX build's
`<zephyr/net/socket.h>`).

The following files are copies of Zephyr's. When the Zephyr build changes, put the same change
into the copies. `common/compat/sensor_shell_stream.c` and `sensor_shell.h` are copies of Zephyr's, with only
the differences listed in "Sensor command (`sensor`)".

`nuttx/tests/fakes/nuttx/config.h` (empty) is the NuttX build's own: it lets a host test build a
source of `nuttx/apps` that includes `<nuttx/config.h>` (`nina_passthrough`: `apps/nina/nina_main.c`).

`nuttx/tests/nano_timeconv.c` is a test of the NuttX build only. It compares the conversions of
`src/nano_timeconv.h` with C's `/` and `%` and with the generic conversions of NuttX 13.1.0 (the
arithmetic of `oneshot.h` and `clkcnt.h` written out for 1 MHz and 100 µs): every number in the
range where the one-byte division step must be exact, 300 values on each side of edge values (0,
99, 100, around 2^32, around 2^32 × 100, 2^63, 2^64 − 1 and others), every count of the first
10^7 µs, 2000 values on each side of 256 multiples of 2^32, and 4 million random values.

`nuttx/tests/nina_os.c` checks the waits for NINA's READY on its interrupt
(`common/drivers/nina/port/nuttx/nina_os.c` under the core's `wait_ready()`, through `common/compat/zephyr_gpio.c`) on fakes of NuttX's
semaphores and clock and of GPIO bank 0: READY changing during the wait (both directions), the
timeout at the deadline after a single wait, a change right after the core's read (the wait
returns at once), a wake-up and a latched edge left from before (dropped), the edge enabled only
during a wait, and at most one count in the semaphore; and the Wi-Fi operations' lock and the
requests of their threads.

`nuttx/tests/nina_passthrough.c` checks `nina passthrough` of `apps/nina` (the order of the help,
the confirmation word, the argument counts, the messages, and the driver's request made only with
`confirm`). The mode itself (`common/drivers/nina/nina_passthrough.c`: the lines, the way out, the
rate, UART1's words, the ring) is tested by `common/tests/nina_passthrough.c`; its NuttX side
(`common/drivers/nina/port/nuttx/nina_passthrough_os.c`, `nina_passthrough_bus_rp2040.c`) has no
host test.

`common/tests/nano_cpuload.c` and `common/tests/log_ring.c` test shared parts that the Zephyr
build does not use. The first compares the arithmetic of `common/compat/nano_cpuload.h` with the
steps of Zephyr's `cpu_load_get_cpu()` (reset, an open window, zero time, rounding down). The second compares
`common/log/log_ring.c` with the byte-by-byte store of the Zephyr build's `log_ram.c`
over 20,000 random writes, compares the line prefix with the time arithmetic of Zephyr's
`log_output.c` for 200,000 random values, and checks the `dmesg` pieces (skipping what is
overwritten, stopping at 8192 bytes) and the rate limit.

The key-value store's tests are in `common/tests/` (`common/tests/run.sh`). `kv_settings` runs
`common/imu/kv_store.c` with the copied Zephyr settings and NVS (built as they are, without
warnings), the layer's flash device and `kv wipe` on a fake flash (`common/tests/kv_flash_fake.c`:
8 KiB of RAM, programs only clear bits, 4 KiB erases, a request outside the area or not aligned
stops the test). Each boot runs in a child process and only the flash is kept. It checks that the
NVS has the Zephyr build's size and place; the bytes on the flash (the first ATE Zephyr's NVS writes
into an empty area, and the names, values and largest-name-ID entries of `app_kv/imu.port` and
`app_kv/imu.session`, with the CRC computed bit by bit in the test and also compared with the
copied `crc8_ccitt()`); reads, writes and deletes, the key and value limits, after a reboot,
garbage collection (10 erases or more), a full store (`-ENOSPC`, deleting still works); other data
(random bytes, an area with one stray byte, the earlier NuttX format: not used, not touched, logged
once, usable after `kv wipe`); that the flash device refuses requests outside the area; that
Zephyr's NVS itself gives `-EDEADLK` on an area whose two sector ends are written; and the power
loss (see "Key-value store"). `nano_flash_kv` checks the range checks of
`common/compat/nano_flash_kv.h` around the edges and for every page and erase sector of the first
and last MiB of the flash, and the page writes, at many offsets and lengths around page boundaries,
by seeing that exactly the written bytes change. For the boot-time check it confirms that Zephyr's
and NuttX's NVS entries are each accepted and that the other format's entries (200,000 layouts each,
including ones that happen to pass the CRC) are not. `hex` checks the copy of Zephyr's
`lib/utils/hex.c`. `zephyr_copies` checks with `cmp`, when the Zephyr tree (`zephyr/zephyr`) is
there, that the 23 copied files (the key-value store's NVS and settings, fnmatch, getopt, the `wifi cred` shell and others) are the v4.5.0 originals. `nuttx/tests/kv_cmd.c` runs the `kv`
command through `z_shell_main_kv()` on that store and checks the output of every subcommand (the
earlier NuttX build's text), the `sys.` refusal, the limits, `kv wipe`, and `-EILSEQ` with other
data. `nuttx/tests/wifi_cred.c` checks the credentials on the same store.

`common/tests/subcmd.c` checks `common/shell/subcmd.c` with the tables of `kv` and `nina` (the help
is the one recorded from the Zephyr build), a nested table, optional arguments, and the layout of
help texts (newlines, the line break before column 80, a newline at the end).

`nuttx/tests/shell_register.c` checks the registration of `imu`, `ntp` and `kv` (`kv` against the
earlier NuttX build's `subcmd` table with `wipe` added). `host_tests.sh` cuts
the text from `SHELL_STATIC_SUBCMD_SET_CREATE()` to `SHELL_CMD_REGISTER()` out of
`common/imu/imu_stream.c` and `time_sync.c`; the test builds it with the compatibility layer's
`<zephyr/shell/shell.h>` and `Z_SHELL_CMD_NAMED_MAIN`, runs it on `common/shell/zshell.c` (with the
NuttX backends, `apps/subcmd/zshell_os_impl.h`), and
compares it with the same tables written as `subcmd` tables (`subcmd_main()`, whose handlers get
NULL as `sh`). For the help (the command alone, `-h` and `--help`, unknown words, wildcards),
wrong numbers of arguments and the arguments of each handler, it checks that the exit status, the
handler calls with their `argc` and `argv`, standard output, standard error, and the order of the
two on one file are the same (the handlers write lines with `shell_print()`, `shell_warn()` and
`shell_error()`; standard output has the NuttX build's 64-byte line buffer), and the Zephyr
shell's output where the two differ (a subcommand after a wildcard that matches, a line of more
than 30 words). `nuttx/tests/i2c_dma_app.c` does the same for the `i2c_dma` command
(`common/shell/i2c_dma_shell.c` on `zshell.c`, against the NuttX build's command until
2026-11-18, the same table as a `subcmd` table), with the text itself and where the two differ
(fakes for the I2C0 core's side).

`nuttx/tests/wifi_cred.c` checks the copies of Zephyr's library in `common/compat/`
(`wifi_credentials.c`, the settings backend) and the `wifi cred` command (the copy of
`wifi_credentials_shell.c`, `subcmd`, the layer's `zephyr_wifi_utils.c`) on a fake settings store
of named items: loading at boot, the stored names (`wifi_cred/<index>`, apart from `app_kv/`) and
contents, the length limits (SSID 32/33, passphrase 7/8/64/65), the third network's `-105` and its
log line, replacing the same SSID and the list in index order, each option and error message, a
stored value of the wrong length, a store failure, `auto_connect`, and the help of `wifi` and
`wifi cred`. The expected output is what Zephyr's sources print.
`common/tests/nina_proto.c` checks `common/drivers/nina/internal/nina_proto.c` with a fake SPI, GPIO, clock and lock: wide
parameters and padding, bulk transfers, the
three answer parameters of 0x75 and 0x7a, the length checks, a failing transfer, narrow
parameters, the version request, the search for the start of the answer (1000 bytes), each
`-EPROTO`, `-EMSGSIZE` and `-EINVAL`, 0x27, `nina_ip_config()` (0x21), the READY time limits (2000,
20 and 10000 ms, counted as samples every 100 µs), the reset on the first request, the order of
the CS edges and transfers, and the lock.

`common/tests/i2c_dw.c` checks `common/drivers/i2c_dw/i2c_dw.h` against the relevant parts of the Zephyr
version's `i2c_dw.c` and `i2c_dw.h` and of Zephyr's `i2c.h`, written out in the test: the SCL
counts (500/625, 93/109 and 31/31 at 125 MHz, and every combination of a 1 to 400 MHz clock and a
speed class), the frequencies taken, the NuttX message flags turned into Zephyr's, the words
written to `IC_DATA_CMD` (the `i2c` tool's `get`, `set`, `set -s`, `dump` and `dev`, a sensor
register read and write, three messages and so on, run as consecutive transfers so that the
direction of one transfer sets the RESTART at the start of the next), the `IC_CON` value and the
register layout.

`nuttx/tests/lsm6dso_driver.c` is also a test of the NuttX build only. It builds
`common/drivers/lsm6dso/lsm6dso.c` (with the NuttX build's `lsm6dso_instance.h`) with ST's real
`lsm6dso_reg.c`, the stmemsc glue, the compatibility layer (`drivers/compat`) and the board's
`src/nano_lsm6dso.c`, and runs it against a fake LSM6DSOX
(register banks, address auto-increment, software reset, FIFO, timestamp) behind a fake I2C
master. Its stand-in headers are in `nuttx/tests/lsm6dso_fakes/` (the board's `board.h` is the
real one). It checks the initialization's log lines and their order, the register writes and
values in order (with the 100 µs after the reset), the registers after it, the sensitivities, an
I2C failure at each step and its log lines, `device_is_ready()`, the address, speed (100 kHz and
what `i2c_configure()` sets) and message shape of every transfer, the registers a single reading
reads and its conversion (against `lsm6dso_cal.c` and the physical value, the signs of `val1` and
`val2`, one axis and three), the attributes (the ODR steps, the rounding of m/s² and rad/s, the
refused values and channels, `-EBUSY`), starting, reading, pausing and stopping the FIFO and its
statistics, the timestamp reading, and `lsm6dso_cal_zero()`. The driver's waits
(`k_busy_wait()`, `k_msleep()`, `k_usleep()`) and its cycle counter (`k_cycle_get_32()`) are the
test's own fakes, which put the waits in the order of the register writes and add up the time
slept; its mutexes and spin lock are the fake kernel's. It also runs the FIFO stream:
`lsm6dso_stream.c`, `lsm6dso_fifo_irq.c`, `lsm6dso_compat.c` and the decoder with the
compatibility layer (work, RTIO, `gpio0`, the default iodev and decoder), on a fake kernel
(`common/tests/zephyr_fakes/`: one thread; the test runs the work queues and fires the timers) and a
fake INT1 that follows the fake sensor's watermark. It checks the stream's start (registers, the
INT1 interrupt), batches on an INT1 edge and on the 20 ms look, a second stream (`-EBUSY`) and
another trigger (`-ENOTSUP`), the graceful stop (pause, the last partial batch, `-ECANCELED`, the
registers back), a cancellation, an overrun (`-EOVERFLOW`), and a single read through the RTIO
work queue with the decoders.

`nuttx/tests/imu_cal.c` is a test of the NuttX build only too. It includes `lsm6dso_driver.c` for
the same fake sensor (running the checks of `lsm6dso_driver` first) and builds
`common/imu/imu_cal.c` with a fake KV store. It checks the loading at boot (two keys), that a stored
value is applied to the single reads (`M·(raw−bias)`), that a value of the wrong length, version,
kind, sensitivity or reserved bytes, or one too long, leaves the nominal conversion, that a range
change reads the key of the other sensitivity and back, `reload` (a deleted key is nominal again,
a tracked bias is dropped), `zero` (at rest, far from the stored bias, during a stream, on an I2C
error), the usage error, an IMU that is not ready, and every line of `imu cal` (including the
rounding of the bias to two decimals) with where it goes (stdout or stderr).

`nuttx/tests/sensor_shell.c` is a test of the NuttX build only too. It includes `lsm6dso_driver.c`
for the same fake sensor, builds `common/drivers/sensor_shell/sensor_shell.c` and `apps/sensor/` with
`common/shell/zshell.c`, and
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

`common/tests/zephyr_rtio.c` and `zephyr_work.c` check the RTIO and work items of the compatibility
layer's shared part on the fake kernel (`common/tests/zephyr_fakes/`; run by `common/tests/run.sh`). The first uses a context of
`imu_stream.c`'s sizes and a fake iodev and checks the memory pool (block size, allocation from
`max` down to `min`, binding to the submission, first fit, exhaustion and release), the
submission pool (4, reuse in order of release, handles and generations), completions (order, 16
of them, a dropped one counted in `xcqcnt` with its buffer left allocated, `consume_sem`),
multishot (handed back before the completion, the binding cleared, the handle kept, freed on an
error or a cancellation), cancellation (the mark, the iodev's function, stale and invalid
handles, no completion for one canceled before submission) and the RTIO work queue. The second
checks the return values and flags of `submit`, `schedule`, `reschedule` and
`k_work_cancel_sync()`, and `K_MSEC()` in ticks.

`nuttx/tests/zephyr_compat_misc.c` is a test of the NuttX build only too. It checks the parts added
to the compatibility layer for the acquisition service: `K_MUTEX_DEFINE()`, `K_THREAD_DEFINE()`
(name, priority 9 → 109, stack, entry and arguments) with `K_THREAD_DEFINE_START()`, the rounding
of `k_uptime_get()` (against Zephyr's `k_ticks_to_ms_floor64()` of 100 µs ticks, around every carry
and across the wrap of the 32-bit tick count), the ticks of `k_sleep(K_MSEC(1))`,
`sys_put_be32()`/`sys_get_be32()`, `snprintk()`, `hwinfo_get_device_id()` (fake chip ID), where
`shell_print()` and `shell_error()` write, and `zsock_inet_pton()`/`zsock_inet_ntop()`.

`nuttx/tests/zephyr_mutex.c` checks the layer's `k_mutex` (`drivers/compat/zephyr_mutex.c`), built
with the NuttX branch (`__NuttX__`) on a model of NuttX 13.1.0's recursive mutex (an all-zero one
is held by thread 0): `K_MUTEX_DEFINE()`, `k_mutex_init()`, a `k_mutex` left all zero and never
initialized (the MQTT client's before `mqtt_client_init()`) that the first lock takes at once, as
in Zephyr, recursion, and `-EBUSY`/`-EAGAIN` while another thread holds it.

`common/tests/nina_sock.c` checks `common/drivers/nina/nina_sock.c` with a fake `nina_request()` that checks
each command and its parameters and gives scripted answers, and a fake clock: the supported socket
types, the table of 10 and `EMFILE`, creating the NINA's socket on first use (type and protocol,
FIONBIO), bind, listen (backlog clamped to 1..255), accept (the peer's address and the byte order of
its port, `EAGAIN`, `EMFILE`), connect (UDP; TCP in progress, the errno of each SO_ERROR value, the
error flag, the 10000 ms limit, a failed poll, closing the NINA's socket after a failure), close,
setsockopt, what `nina_event` queries and turns into events, and `nina_sock_poll_revents()` (the
Zephyr and FreeRTOS builds' `do_poll()` for one socket). For sending and receiving: TCP
send (at most 3584 bytes a call, the answer checks, `MSG_DONTWAIT`, the passes of a waiting send
with its state check every 100 ms and its 10000 ms limit, a continued call told from a new one),
`sendmsg` (vectors joined into one command, what it refuses), UDP send (the address checks, the
connected peer, the 512-byte limit, a datagram the NINA did not take), TCP and UDP receive (the
poll first, the size parameter, the byte order of the sender's port, dropping datagrams from other
than the connected peer, an empty datagram), and a poll that found data standing for the next
receive's first poll.

`common/tests/nina_dns.c` checks `common/drivers/nina/nina_dns.c` with a fake `nina_request()` and lock:
the hints' defaults and the refused flags, families and socket types, the service (a port number),
numeric addresses (the NINA is not asked), a name (commands 0x34 and 0x35 with the lock held), and
the `EAI_*` of each failure.

`common/tests/nina_wifi.c` checks the core's Wi-Fi operations (`common/drivers/nina/nina_wifi.c`) on
fakes of the module, of `nina_os.h` and of a binding: connect with its checks and its thread,
disconnect, the monitor, the scans, status. `nuttx/tests/nina_netdev.c` checks `wlan0`
(`common/drivers/nina/port/nuttx/nina_netdev.c`) on NuttX's `<nuttx/wireless/wireless.h>` and fakes of the
device, the core and the management: the start, the core's results on the device, wapi's ioctls
and the threads. `nuttx/tests/nina_wifi_mgmt.c` checks the applications' management
(`nina_wifi_mgmt.c`) under the layer's Wi-Fi requests and interface.
