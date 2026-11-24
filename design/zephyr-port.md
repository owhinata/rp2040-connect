# Inside the Zephyr port

English | [日本語](zephyr-port-ja.md)

The firmware of this repository is the same function built on four OSes, Zephyr, NuttX, the Pico SDK with FreeRTOS
and the Pico SDK with ThreadX: the Zephyr, NuttX, FreeRTOS and ThreadX builds. This document describes the inside of
the Zephyr build's port (`zephyr/app/usb-console`), that is, what only the Zephyr build has outside the sources shared
by the four OSes (`common/`). The shared drivers are described in their own documents and not here. Usage is in the
top-level [README](../README.md); the sources the four builds share and the OS boundary are in §9 of the
[design](imu-raw-mqtt-design.md) and in [The OS boundary](os-boundary.md).

## Drivers

`zephyr/app/usb-console` uses some of Zephyr's drivers with local patches of Zephyr's tree, which
carry the changes needed to read the IMU FIFO at a high rate, and two drivers shared with the NuttX,
FreeRTOS and ThreadX builds, for the LSM6DSOX and the NINA-W102.

| Driver | Content | Origin | Described in |
|---|---|---|---|
| LSM6DSO (`lsm6dso`) | FIFO streaming, conversion with a calibration, gyro bias at rest | `common/drivers/lsm6dso/` (in place of Zephyr's `drivers/sensor/st/lsm6dso`) | [The LSM6DSO driver](../common/drivers/lsm6dso/README.md) |
| Sensor shell (`drivers/sensor/sensor_shell.c`) | Fixes and the LSM6DSO's FIFO attributes | Zephyr v4.5.0 with `zephyr/0006-sensor-shell-fifo-attributes.patch` | [The LSM6DSO driver](../common/drivers/lsm6dso/README.md) |
| DesignWare I2C (`drivers/i2c/i2c_dw.c`) | The RP2040's FIFO depth, I2C0's DMA reads | Zephyr v4.5.0 with `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch` (and `0004` for `i2c scan`) | [The DesignWare I2C core](../common/drivers/i2c_dw/README.md) |
| NINA-W102 (`nina`) | Wi-Fi and sockets over SPI | `common/drivers/nina/` | [The NINA-W102 driver](../common/drivers/nina/README.md) |
| PL022 SPI (`drivers/spi/spi_pl022.c`) | The DMA mode's threshold, for the NINA's SPI1 | Zephyr v4.5.0 with `zephyr/0007-spi-pl022-dma-min-len.patch` | "The PL022 SPI's DMA mode" below |

Zephyr v4.5.0 is commit `41d10b59730406bbab4ca9f1fbbe9e96aafb4966`. Each patch holds everything its driver
needs, in Zephyr's tree (see "Local patches of Zephyr" below): the application's `CMakeLists.txt` adds no
source or include path to the patched drivers' libraries.

What is written on Zephyr's APIs (the shell, the sockets, the kernel) uses Zephyr's own, so the Zephyr build does not
use the compatibility layer (the layer that provides Zephyr's APIs on the other OSes, `common/compat/`).

## The application's directory

| File | Content |
|---|---|
| `CMakeLists.txt` | Builds the application's sources of `common/imu/`, the NINA core with its Zephyr binding (`common/drivers/nina/port/zephyr/`), and `nina_shell.c` and `i2c_dma_shell.c` of `common/shell/`. Adds the NINA binding's devicetree binding to `DTS_ROOT` and the LSM6DSO binding (`common/drivers/lsm6dso/port/zephyr/`, a Zephyr module) to `ZEPHYR_EXTRA_MODULES`. Has the divider (see "The divider" below) and includes `ram_code.cmake` |
| `Kconfig` | `CONFIG_RP2040_PICO_DIVIDER`, the NINA binding's `Kconfig` (`rsource`), `CONFIG_NINA_PASSTHROUGH`, the NTP settings, the application modules' symbols (`CONFIG_KV_STORE`, `CONFIG_APP_IMU`, `CONFIG_WIFI_PROFILE`) |
| `prj.conf` | The configuration (only one). The patches' symbols (`CONFIG_RP2040_ROM_MEM_OPS`, `CONFIG_I2C_DW_RP2040_DMA`, `CONFIG_SPI_PL022_DMA_MIN_LEN`), `CONFIG_CODE_DATA_RELOCATION`, `CONFIG_LSM6DSO=n` with `CONFIG_LSM6DSO_LOCAL=y`, `CONFIG_USBD_CDC_ACM_WORKQUEUE` (see "The CDC ACM work queue" below) |
| `app.overlay` | Additions to the devicetree (see "Devicetree" below) |
| `ram_code.cmake`, `ram_code_check.py` | The list of the code in RAM and the check after the link (see "Code in RAM" below) |
| `src/main.c` | The entry `main()` (calls `app_main()` of the shared start, `common/imu/app_main.c`), `kernel ps`, `bootsel` |
| `src/log_ram.c` | The RAM log (4096 bytes, `dmesg`) |
| `src/kv_store_os.h`, `src/wifi_profile_os.h` | The Zephyr build's headers read by shared sources (`common/imu/kv_store.c`, `wifi_profile.c`) |
| `tests/` | The fakes and tests of the host tests of the Zephyr build only (see "Tests" below) |

### Devicetree

`app.overlay` adds the following.

- The console and the shell are on the USB CDC ACM (`cdc_acm_uart0`, a 2048-byte receive FIFO).
- `dma` is enabled.
- I2C0 runs at 1 MHz from the start (the board's file has 100 kHz).
- `spi1`'s `dmas` are DMA channels 10 (TX) and 11 (RX) (see "The PL022 SPI's DMA mode" below).
- The LSM6DSOX's ranges (8 g for the accelerometer, 2000 dps for the gyroscope).
- The NINA-W102's node is `arduino,nina-spi` (the NINA binding's binding).
- The last two erase sectors of the flash (8 KiB from 0xffe000) are the key-value area `storage_partition`, kept out of
  the image's partition.

## The PL022 SPI's DMA mode

The PL022 driver of the NINA's SPI1 runs in its DMA mode (`CONFIG_SPI_PL022_DMA` in `prj.conf`, with `spi1`'s `dmas`
in `app.overlay`: channels 10 and 11). A transfer shorter than `CONFIG_SPI_PL022_DMA_MIN_LEN` of the local patch
`zephyr/0007` (64 in `prj.conf`) takes the polled path (the NuttX, FreeRTOS and ThreadX builds draw the same line).
The transfers' flow, the bus lock, and how DMA errors and timeouts are handled are in "Zephyr" of
[The NINA-W102 driver](../common/drivers/nina/README.md).

## The CDC ACM work queue

`CONFIG_USBD_CDC_ACM_WORKQUEUE=y` in `prj.conf` moves the CDC ACM class (the USB shell's transmit
completions) from the system work queue to its own. Zephyr 4.5.0's zperf iperf3 server prints to the
shell while it holds `srv_lock` on `zperf tcp download stop`, and its `idle_work_handler()` on the
system work queue waits for `srv_lock`, so without the setting the shell (`shell_uart`) and the system work
queue wait for each other (both pending at priority -1).

## Code in RAM

The flash is executed through the RP2040's 16 KiB XIP cache, and the code on the 6667 Hz path of the
sensor and on the send path does not fit in it. The code (`.text.*`; the read-only data stays in the
flash) of the sources listed in `app/usb-console/ram_code.cmake` runs from RAM:

- the kernel objects that the drivers and the application wait on and signal (`kernel/sem.c`, `mutex.c`,
  `poll.c`, `work.c`, `timeout.c`, `queue.c`, `mem_slab.c`), the atomic operations (`lib/os/atomic_c.c`:
  functions on the Cortex-M0+), the memory blocks of the FIFO's buffers with their bit array, RTIO's
  executor and work queue, the sensor clock, and `gpio0`'s driver with hal_rpi_pico's `hardware_gpio`;
- I2C0 (`drivers/i2c/i2c_dw.c` with patch 0003's DMA part) and the DMA driver (`dma_rpi_pico.c`);
- the LSM6DSOX driver (the core's `lsm6dso.c`, `lsm6dso_stream.c`, `lsm6dso_fifo.c`, `lsm6dso_cal.c` and
  the binding's `lsm6dso_sensor.c`, `lsm6dso_rtio.c`, `lsm6dso_rtio_stream.c`, `lsm6dso_os.c`) and the
  stmemsc I2C glue; not ST's `lsm6dso_reg.c`, which only configures, nor the decoder;
- the IMU service's stream, batches, queue and time stamps (not the NTP exchange nor the calibration);
- the send: `imu_raw_mqtt.c`, `imu_raw_packet.c` and Zephyr's MQTT library, the sockets' dispatch
  (`sockets.c`, `socket_offload.c`, the `zvfs` descriptor table and `poll()`), the NINA's sockets,
  protocol, waits and Zephyr binding (`nina_socket.c`, `nina_os.c`, `nina_bus.c`), and the PL022 SPI
  driver.

The scheduler (`kernel/sched.c`), the thread switch (`swap_helper.S`), the interrupt entry and exit, the
SysTick driver and the idle loop stay in the flash. The other builds' lists have the same roles
(`common/imu/port/pico_sdk/ram_code.cmake`, which moves the compatibility layer's kernel part where this
list moves Zephyr's kernel objects; "Code in RAM" in [The FreeRTOS port](freertos-port.md)). To check a placement,
clear the cache's counters while sending (`devmem 0x1400000c 32 0`, `devmem 0x14000010 32 0`; they
saturate) and read them about 15 s later (the hits and the accesses).

- **How**: Zephyr's code relocation, `CONFIG_CODE_DATA_RELOCATION` (`prj.conf`) and
  `zephyr_code_relocate(FILES ... LOCATION RAM_TEXT NOKEEP)`. Its generator
  (`scripts/build/gen_relocate_app.py`) lists the objects' `.text.*` sections in an output section
  `.ram_text_reloc` that the Cortex-M linker script places before the flash's `.text`: at the start of
  RAM, loaded from the flash. `arch_data_copy()` copies it with `.data` in `z_prep_c()`, before the
  interrupts are set up and the kernel starts; what runs before (`z_arm_reset`, `relocate_vector_table()`,
  `arch_bss_zero()`, the copy with `arch_early_memcpy()`) is not in the list. With `NOKEEP` the functions
  that nothing calls are still left out. Patch 0008 lets the generator find the objects of the LSM6DSO
  module's library (`lsm6dso_local`, whose name is not its directory's).
- **Cost**: 57,320 bytes of RAM (`.ram_text_reloc`): 55,546 bytes of code and 106 long-branch stubs from
  RAM to the flash (16 bytes each). The flash's `.text` has 130 stubs to RAM (4 of them for the flash's
  write routines). The RAM comes from what is left after
  the image (`_image_ram_end`), which is picolibc's `malloc()` arena
  (`CONFIG_COMMON_LIBC_MALLOC_ARENA_SIZE=-1`); its one user is the NINA passthrough mode's 16 KiB buffer,
  taken in the mode only (the stacks and the other buffers are static). 63,960 bytes are left.
- **Check**: `ram_code.cmake` stops the configuration when a listed source does not exist. After the link,
  `ram_code_check.py` (the target `ram_code_check`, on `compile_commands.json`, the objects and the map)
  stops the build when a listed object's code is not in RAM or part of it is in the flash (a source no
  longer built, or one the generator did not find), when the code of an object not listed is in RAM, or
  when less than 18,432 bytes of RAM are left (the passthrough mode's 16 KiB buffer and 2 KiB for
  `malloc()` itself); it prints what is in RAM.
- **What stays in the flash**: the vector table and the interrupt entry `_isr_wrapper`, which calls the
  handlers in RAM (`i2c_dw_isr()`, `dma_rpi_pico_isr()`, `gpio_rpi_isr()`) through the software ISR table,
  the start-up code, the key-value store's flash part (the flash's write and erase are hal_rpi_pico's
  `.time_critical` functions, in `.data`; while XIP is off only they run, with interrupts
  locked), the NINA passthrough's UART interrupt, and the fatal error path (`fault.c`, the kernel's
  `fatal.c`, the logging core). That path calls moved code: the atomic operations always, and through the
  logging core `k_sem_give()`, `k_sem_take()`, the timeouts and `k_work` when they are needed.

## Local patches of Zephyr

Some changes to Zephyr and its modules are local patches for Zephyr's `west patch`. They are listed in
`zephyr/manifest/zephyr/patches.yml`; the files are in `zephyr/manifest/zephyr/patches/<module>/`.

| Patch | Module | Change |
|---|---|---|
| `hal_rpi_pico/0001-boot2-at25sf128a-clkdiv-2.patch` | hal_rpi_pico | The board's boot2 (`boot2_at25sf128a.S`) runs the flash at clk_sys / 2 (62.5 MHz) instead of clk_sys / 4 (31.25 MHz). hal_rpi_pico has no way to pass the value, so the default changes |
| `zephyr/0001-arm-cortex-m-rp2040-divider-save.patch` | zephyr | `CONFIG_ARM_RP2040_DIVIDER_SAVE`: the thread switches (PendSV, `swap_helper.S`) save and restore the SIO divider |
| `zephyr/0002-soc-rp2040-rom-mem-ops.patch` | zephyr | `CONFIG_RP2040_ROM_MEM_OPS`: `memcpy` and `memset` through the boot ROM's routines |
| `zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch` | zephyr | The DesignWare I2C driver (`drivers/i2c/i2c_dw.c`): the RP2040's FIFO depth and RX threshold, and I2C0's DMA reads with `CONFIG_I2C_DW_RP2040_DMA` ([The DesignWare I2C core](../common/drivers/i2c_dw/README.md)). The constants are in a new file `drivers/i2c/i2c_dw_rp2040.h`, the DMA code with the read commands in `i2c_dw_rp2040_dma_priv.h`, and the declarations of the `i2c_dma` command's functions in `include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`; 16 places go into the upstream files (`i2c_dw.c` 14, `i2c_dw.h` 1, `Kconfig.dw` 1); on every controller, a message of no bytes is refused (`-ENOTSUP`) and `i2c_recover_bus()` returns `-ENOTSUP` |
| `zephyr/0004-i2c-shell-scan-read-probe.patch` | zephyr | The shell's `i2c scan` (`drivers/i2c/i2c_shell.c`): when a probe (a write of no bytes) returns `-ENOTSUP`, that address and the rest are probed with a read of one byte, and a line after the table says so, instead of showing `--` everywhere and "0 devices found". Any error of a probe shows `--` |
| `zephyr/0006-sensor-shell-fifo-attributes.patch` | zephyr | The sensor shell (`drivers/sensor/sensor_shell.c`): the LSM6DSO's FIFO attributes and the fixes ([The LSM6DSO driver](../common/drivers/lsm6dso/README.md)), with `drivers/sensor/sensor_shell_os.h`. Needs the LSM6DSO driver of `common/drivers/lsm6dso/` (its `<zephyr/drivers/sensor/lsm6dso.h>`) |
| `zephyr/0007-spi-pl022-dma-min-len.patch` | zephyr | The PL022 SPI driver (`drivers/spi/spi_pl022.c`, `Kconfig.pl022`): `CONFIG_SPI_PL022_DMA_MIN_LEN` (default 0, as upstream): in the DMA mode a transfer shorter than that many bytes takes the polled path (a transfer of no bytes takes the context lock and moves nothing); after a DMA transfer SSPDMACR's request enables are cleared, and after one that failed or timed out its channels are stopped ("The PL022 SPI's DMA mode") |
| `zephyr/0008-code-relocation-module-objects.patch` | zephyr | The code relocation's generator (`scripts/build/gen_relocate_app.py`): when its two searches for a source's object find nothing, it takes the object of the source's name if no other object has that name. A library whose name is not its directory's (the LSM6DSO module's `lsm6dso_local`) names the object of a source in that directory `<library>.dir/<name>.obj`, which the searches do not find without the patch; the source then stays in the flash without a message ("Code in RAM") |

- **Applying**: `source script/setup.sh` applies them to the modules' working trees with `west patch apply`
  (no commits; `git -C zephyr/zephyr status` shows them as changes). Patches already applied are skipped,
  so sourcing it again changes nothing. A copy of the set applied is kept in `zephyr/.west/local-patches/`;
  when `patches.yml` or a patch changes, that set is taken back first and the current one applied (a patch
  removed from the list does not stay in a tree). When applying fails, the patches applied are taken back.
- **After changing a patch**: set its `sha256sum` in `patches.yml` to the value of `sha256sum <file>`, then
  source `script/setup.sh` again.
- **Taking them back**: run `west patch reverse` in `zephyr/` and remove `zephyr/.west/local-patches/`. If
  that fails, restore each tree with `git -C <tree> checkout -- .` as `setup.sh` says, remove the copy and
  source it again. The application does not build on trees without the patches (`prj.conf` sets the
  patches' `CONFIG_RP2040_ROM_MEM_OPS`, `CONFIG_I2C_DW_RP2040_DMA` and `CONFIG_SPI_PL022_DMA_MIN_LEN`, and `CONFIG_RP2040_PICO_DIVIDER` stops in CMake without
  `CONFIG_ARM_RP2040_DIVIDER_SAVE`; without 0008 the check of the code in RAM stops the build).
- **Moving to a newer Zephyr**: take the patches back first as above, change the versions in
  `manifest/west.yml` and run `west update`. Remake each patch on the new trees, put it in `patches/`,
  update its `sha256sum`, apply them with `source script/setup.sh` and build. After remaking 0003, run
  `script/host_tests.sh`: bring the upstream changes that `i2c_dw_sync` shows into the shared core
  (`common/drivers/i2c_dw/`), or add the reason not to to the list at the top of `i2c_dw.c`, then
  rewrite the expected differences with `common/tests/i2c_dw_sync.py ... --update` (the tests of
  [The DesignWare I2C core](../common/drivers/i2c_dw/README.md)).
  0006 is made from `common/shell/sensor_shell.c` (with the new tree's upstream file
  merged into it first), so that `zephyr_sensor_shell_patched` passes.

### boot2 at clock divider 2

The read boot2 uses (EBh) is rated for 108 MHz or more by the AT25SF128A's data sheet. With clk_sys at
125 MHz the divider is even, so 62.5 MHz is the most (the same divider as the FreeRTOS and ThreadX builds' Pico SDK
boot2). On the board, `devmem 0x18000014` (the SSI's BAUDR) reads 2.

### The divider (`CONFIG_RP2040_PICO_DIVIDER`)

The application's `CONFIG_RP2040_PICO_DIVIDER` (on by default) builds the Pico SDK's `divider_hardware.S` from
hal_rpi_pico and wraps `__aeabi_idiv`, `__aeabi_idivmod`, `__aeabi_uidiv`, `__aeabi_uidivmod`,
`__aeabi_ldivmod` and `__aeabi_uldivmod` to it (the linker's `--wrap`, `CMakeLists.txt`). Without it they
are libgcc's shift-and-subtract loops.

- The routines do not lock interrupts (`PICO_DIVIDER_DISABLE_INTERRUPTS=0`). When an interrupt handler
  divides in the middle of a division, the routine sees DIRTY and saves the divider itself. When a thread
  is switched out in the middle of one, the patch's `CONFIG_ARM_RP2040_DIVIDER_SAVE` (selected by
  `CONFIG_RP2040_PICO_DIVIDER`) saves it in PendSV: only when DIRTY is set, the four registers go to 20 bytes
  of `struct _thread_arch`, and a thread that was not dividing comes back with DIRTY clear (the Pico SDK's
  FreeRTOS port of the FreeRTOS build saves the four words at every switch; ThreadX's Cortex-M0 port of the ThreadX
  build does not save them at a switch).
- A division by zero gives the Pico SDK routines' results (the same in the four builds): for 32 bits a
  saturated quotient (0 for 0 / 0) and a remainder of 0; for 64 bits the same as libgcc's.

### The boot ROM's `memcpy` and `memset` (`CONFIG_RP2040_ROM_MEM_OPS`)

With the patch's `CONFIG_RP2040_ROM_MEM_OPS` (set in `prj.conf`), the SoC's CMake wraps `memcpy`,
`memset`, `__aeabi_memcpy*` and `__aeabi_memset*` (the linker's `--wrap`) to the RP2040 boot ROM's
routines, split as the Pico SDK's `pico_mem_ops` does. This covers the calls from inside picolibc and
those GCC emits for struct copies and loops. Without it they are picolibc's byte loops. The routines are
looked up in the ROM's function table on first use and kept in `.bss`, so `arch_early_memset` and
`arch_early_memcpy`, which clear `.bss` and copy `.data`, call picolibc's. The NuttX build has the same
routines as a patch of NuttX (`nuttx/patches/nuttx/0010`; "The C library's `memcpy` and `memset`" in
[the NuttX build's design](nuttx-porting.md)).

## Tests

`zephyr/script/host_tests.sh` first runs the tests shared with the NuttX, FreeRTOS and ThreadX builds
(`common/tests/`, run by `common/tests/run.sh`). They are described in the tests sections of the drivers' documents
([NINA-W102](../common/drivers/nina/README.md), [DesignWare I2C](../common/drivers/i2c_dw/README.md),
[LSM6DSO](../common/drivers/lsm6dso/README.md), [the compatibility layer](../common/compat/README.md)). It then
tests the NINA's Zephyr binding on the PC with fake Zephyr headers (`nina_boundary`, `nina_boundary_passthrough`,
`nina_boundary_spi_dma`, `nina_bus_spi_dma_threshold`, `passthrough`, `nina_socket`, `nina_net`; the fakes are in
`zephyr/app/usb-console/tests/`), described in the tests section of [The NINA-W102 driver](../common/drivers/nina/README.md).
Last it checks the following.

| Test | Covers |
|---|---|
| local patches | `patches.yml` lists each patch file exactly once, and the `sha256sum` of each entry is that of its file (`west patch` refuses the patch otherwise) |
| `host/imu_host` (syntax) | The PC side's `host/imu_host/*.py` parse as Python |
