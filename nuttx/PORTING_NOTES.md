# NuttX 13.1.0 で確かめた事実（T01b）

NuttX 13.1.0 のソースを 2026-11-13 に読んで確かめた事実の記録。`N/` = `nuttx/nuttx`、`A/` = `nuttx/nuttx-apps`。
【事実】はコードで確かめたこと、【推測】【提案】【計算】は未ビルド・未確認。調査した subagent の報告を
メインが書き起こしたもの。使う前に該当箇所を 13.1.0 のソースで確かめ直すこと。未追跡のまま置く。

## 1. out-of-tree の仕組み

【事実】
- **configure**: `./tools/configure.sh -l -a ../nuttx-apps ../boards/arduino-nano-rp2040-connect/configs/imu`
  （`N/tools/configure.sh:157-171`。`TOPDIR` からの相対パスか絶対パス）。
  - `-a` が無く defconfig にも `CONFIG_APPS_DIR` が無いと、`${TOPDIR}/../apps`、`../nuttx-apps` の順に探す
    （`:300-305`）。**`TOPDIR/../apps` は自前の `nuttx/apps/` なので、必ず `-a ../nuttx-apps` を渡し、
    defconfig にも `CONFIG_APPS_DIR="../nuttx-apps"` を書く。**
  - Make.defs の探索順（`:174-194`）: `configpath/Make.defs` → `configpath/../../scripts/Make.defs` →
    `../../../common/scripts/Make.defs`。
  - `sethost.sh` を呼ぶので `kconfig-tweak` が要る（T01 で対応済み）。
- **カスタムボード**（`N/boards/Kconfig:3792-3856`）: `ARCH_BOARD_CUSTOM`、`_CUSTOM_NAME`（uname に出るだけ）、
  `_CUSTOM_DIR`、`_CUSTOM_DIR_RELPATH`（既定 y、`TOPDIR` からの相対）。`BOARD_CUSTOM_LEDS`/`BUTTONS` などで
  `ARCH_HAVE_*` を select できる。
- **BOARD_DIR**（`N/tools/Config.mk:165-188`）: `BOARD_DIR = $(TOPDIR)/$(CUSTOM_DIR)`。ボードの Kconfig は
  `$(BOARD_DIR)/Kconfig` で、`boards/dummy/Kconfig` に**1 回だけ**写される（`Unix.mk:382-384`）。
  `if ARCH_BOARD_CUSTOM` の中で source される（`boards/Kconfig:5160-5162`）。
  → ボードの Kconfig を書き換えたら `rm N/boards/dummy/Kconfig` か distclean。
- **common**: `ARCH_CHIP_RP2040` が `select ARCH_BOARD_COMMON`（`N/arch/arm/Kconfig:382`）。
  `$(BOARD_DIR)/../common`（= `nuttx/boards/common`）が**無ければ** `boards/arm/rp2040/common` を使う
  （`Config.mk:182-188`）。`BOARD_DRIVERS_DIR` も同様に `../drivers` を見る（`:189`）。
  **→ `nuttx/boards/` の直下に `common/`・`drivers/` を作らない。**
  - リンク: `arch/arm/src/board` → common、`arch/arm/src/board/board` → `BOARD_DIR/src`（`Unix.mk:321-338`）。
    common の Makefile が `include board/Make.defs` でボードの `src/Make.defs` を読む。
    `include/arch/board` → `BOARD_DIR/include`。
  - common の Kconfig（`RP2040_FLASH_BOOT`、`_FLASH_CHIP`、`_UF2_BINARY`、ピン番号など）は
    `if ARCH_BOARD_COMMON` で読まれる（`boards/Kconfig:5472, 5504`）。
- **nuttx/external**（カーネル側）: `external/Kconfig` が読めれば `EXTERNALDIR=external`、無ければ `dummy`
  （`Unix.mk:84`）。Kconfig はルートの末尾で source（`N/Kconfig:3070-3074`、どの menu の中でもない）。
  ビルドは `$(MAKE) -C external libexternal$(LIBEXT) EXTRAFLAGS="$(KDEFINE) …"`（`-D__KERNEL__` 付き、
  `LibTargets.mk:112-116`、`Config.mk:126`）。`KERNDEPDIRS` に入るので depend/clean/distclean が呼ばれる
  （`Directories.mk:60-62, 199`）。リンクは `staging/libexternal.a`（`FlatLibs.mk:39-40`）。
  symlink 越しの `-C` なのでオブジェクトは `nuttx/drivers/` の中にできる（.gitignore 済み）。
- **nuttx-apps/external**（アプリ側）: apps の `Make.defs` が `$(APPDIR)/*/Make.defs` を読む。Kconfig の無い
  ディレクトリは `preconfig` で `Directory.mk` の mkkconfig が `external/Kconfig` を生成する（`A/Make.defs`、
  `A/Directory.mk`、`A/Makefile:191-195`）。**生成は初回だけ。** 消えるのは apps の distclean のとき。
  → アプリを足したら `nuttx/apps/Kconfig`（自動生成、.gitignore 済み）を消してから configure。

【提案】雛形（未ビルド。T02 で実際に通る形に直す）

`nuttx/drivers/Kconfig`:
```kconfig
menu "Nano RP2040 Connect drivers"
rsource "nina/Kconfig"
rsource "lsm6dso/Kconfig"
endmenu
```
（`rsource` が使えなければ `source "external/nina/Kconfig"`。srctree = `N/`）

`nuttx/drivers/Makefile`（`N/wireless/Makefile` が手本）:
```make
include $(TOPDIR)/Make.defs
CSRCS :=
include nina/Make.defs
include lsm6dso/Make.defs
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)/external/include
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)/arch/arm/src/chip
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)/arch/arm/src/common
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)/arch/arm/src/armv6-m
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)/sched
AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))
SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)
BIN = libexternal$(LIBEXT)
all: $(BIN)
.PHONY: depend clean distclean
$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)
$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)
$(BIN): $(OBJS)
	$(call ARCHIVE_REBUILD, $@, $(OBJS))
makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)
.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile
	$(Q) touch $@
depend: .depend
clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)
distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
-include Make.dep
```
各 `nina/Make.defs` は `CSRCS += nina_spi.c …`、`VPATH += :nina`、`DEPPATH += --dep-path nina`。

`nuttx/apps/Make.defs`: `include $(wildcard $(APPDIR)/external/*/Make.defs)`

`nuttx/apps/Makefile`:
```make
MENUDESC = "Nano RP2040 Connect apps"
include $(APPDIR)/Directory.mk
```

`nuttx/apps/imu/Make.defs`:
```make
ifneq ($(CONFIG_NANO_APP_IMU),)
CONFIGURED_APPS += $(APPDIR)/external/imu
endif
```

`nuttx/apps/imu/Makefile`（`A/examples/hello` の形）:
```make
include $(APPDIR)/Make.defs
PROGNAME  = imu
PRIORITY  = 100
STACKSIZE = 2048
MODULE    = $(CONFIG_NANO_APP_IMU)
MAINSRC   = imu_main.c
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)/external/include
include $(APPDIR)/Application.mk
```
各アプリの Kconfig は `config NANO_APP_IMU tristate …`。

## 2. RP2040 のビルド

【事実】
- **boot2**: `PICO_SDK_PATH` があるときだけ `include chip/boot2/Make.defs`（`N/arch/arm/src/rp2040/Make.defs:102-106`）。
  **未設定でもエラーにならず、`.boot2` の無いイメージができる。** → build.sh で `.boot2` が 256 B あることを検査。
  - 作り方: `$(PICO_SDK_PATH)/src/rp2040/boot_stage2/boot2_$(CONFIG_RP2040_FLASH_CHIP).S` を
    `$(CC) -nostdlib -T$(ARCHSCRIPT)` でリンク（**ボードの ld をそのまま使う**）→ objcopy →
    `python3 pad_checksum -s 0xffffffff`。ビルド中に pico-sdk 内の `version.h`・`config_autogen.h` を作って消す。
  - `RP2040_FLASH_CHIP` は自由入力の文字列（既定 `"w25q080"`、`common/Kconfig:17-24`）。`boot2_at25sf128a.S` は
    pico-sdk 2.2.0 にある。
- **リンカースクリプト**（`raspberrypi-pico-flash.ld`）: flash 0x10000000 から 2048K、sram 264K。
  並び: `.flash_begin` → `.boot2`（`KEEP(*(.boot2))`）→ `.text`（`_stext`、`.vectors` から）→ `.init_section` →
  `.ARM.extab`/`.exidx` → `_eronly` → `.ram_vectors (COPY)` > sram → `.data`（`*(.ram_code.*)` を含む、
  sram AT> flash）→ `.flash_section`（4 KB 境界の `*(.flash.*)`）→ `.bss`。`.noinit` は無い。
  - 起動処理は `_sbss`〜`_ebss` を 0 で埋め、`_eronly` から `_sdata`〜`_edata` に写す（`rp2040_start.c:108,137`）。
    idle スタックは `_ebss + IDLETHREAD_STACKSIZE`、ヒープは `g_idle_topstack` から `RAM_END`
    （`rp2040_start.c:46`、`arm_allocateheap.c`）。
  - RAM 関数は `RAM_CODE_ATTR`/`locate_code(".ram_code.xxx")`（`rp2040_rom.h:58`、`rp2040_uniqueid.c:171,196`）。
- **UF2**（`N/tools/rp2040/Config.mk`）: `CONFIG_RP2040_UF2_BINARY=y`（既定）なら POSTBUILD で
  `picotool uf2 convert`。PATH に無ければ pico-sdk の中で cmake して picotool を作る（ネットワークが要り、
  pico-sdk を汚す）。`CONFIG_RAW_BINARY=y` で `nuttx.bin`（`Unix.mk:588-591`、0x10000000 から、boot2 を含む）。
- 【提案】自前 ld: `flash LENGTH = 16376K`（16 MiB − 8 KiB）。`.noinit (NOLOAD)` を `.data` と `.bss` の間の
  sram に置けば起動処理のどちらの範囲にも入らない。boot2 のリンクにも同じ ld が使われるので `ASSERT` などを
  入れない。

## 3. USB CDC ACM コンソールと nsh

【事実】
- 「Enter 3 回」は 13.1.0 でも同じ（`A/nshlib/nsh_usbconsole.c:160-194` の `nlc < 3`）。`HAVE_USB_CONSOLE` は
  `CDCACM && CDCACM_CONSOLE`、または `NSH_USBCONSOLE` で定義（`A/nshlib/nsh.h:132-151`）。
  **両方切れば汎用の `nsh_consolemain.c` が使われる**（`NSH_CONSOLE=y` が必要）。
- 汎用版は `nsh_newconsole(true)` で fd 0/1/2 を使い、セッションが EOF になると **`exit()` する**
  （`A/nshlib/nsh_consolemain.c`、`nsh_console.c:389-395`）。init 自身で呼ぶと init が消える。
  → telnetd と同じ形（`A/netutils/telnetd/telnetd_daemon.c:262-299`）: init が `/dev/ttyACM0` を開き、`dup2` で
  0/1/2 にし、`task_create("nsh", …, nsh_consolemain)` で起動し、`waitpid` で待ち、開き直す。
- **13.1.0 では `NSH_ARCHINIT`・`BOARDIOC_INIT`・`board_app_initialize()` が無い。** ボードの初期化は
  `BOARD_LATE_INITIALIZE`（既定 y）の `board_late_initialize()` で、init タスクの起動直前に呼ばれる
  （`N/sched/init/nx_bringup.c:316-323`）。
- `nsh_initialize()` は公開 API（`A/include/nshlib/nshlib.h:124`）。プロンプトを決め、`NSH_NETINIT` なら
  `netinit_bringup()`、`NSH_TELNET && !NSH_DISABLE_TELNETSTART && !NETINIT_NETLOCAL` なら
  `nsh_telnetstart(AF_UNSPEC)`（`nsh_init.c:106-185`）。
  【提案】自前の init は最初に `nsh_initialize()` を呼ぶ。Telnet は `NETINIT_NETLOCAL=y`（または
  `NSH_DISABLE_TELNETSTART=y`）にして、NINA の準備後に `nsh_telnetstart(AF_INET)` を呼ぶ。
- `NSH_ALTCONDEV` は `depends on !NSH_TELNET` なので使えない（`Kconfig:1017-1020`）。
- **CDC ACM の登録**: `boardctl(BOARDIOC_USBDEV_CONTROL)` は `BOARDCTL_USBDEVCTRL=y` が必要
  （`N/boards/boardctl.c:130-146`）。flat build なので `board_late_initialize()` から `cdcacm_initialize(0, NULL)` を
  直接呼ぶ方が簡単。構成されるまで `open("/dev/ttyACM0")` は `-ENOTCONN`（`cdcacm.c:2160-2164`）。
  DTR は接続状態に使われない。
- **console でない tty**: `CDCACM_CONSOLE=n` だと `tc_oflag` は 0 で `\n`→`\r\n` 変換をしない
  （`N/drivers/serial/serial.c:2320-2332` は isconsole のときだけ設定）。init で `tcgetattr`/`tcsetattr` を呼び
  `OPOST|ONLCR` を立てる（`SERIAL_TERMIOS` が無くても iflag/oflag/lflag は扱える）。エコーは
  `READLINE_ECHO=y`（既定）。入力の CR（ICRNL）は要判断: ホストは `\r\n` を送る（`host/imu_host/board.py:100`）ので
  立てると空行が 1 つ増え、立てないと端末ソフトの CR だけの Enter で確定しない。readline は `\n` だけで確定。
- **USB の文字列**: `CDCACM_VENDORID`（0x0525）、`PRODUCTID`（0xa4a7）、`VENDORSTR`（"NuttX"）、`PRODUCTSTR` は
  Kconfig（`N/drivers/usbdev/Kconfig:734-757`）。シリアル文字列は "0" 固定
  （`N/include/nuttx/usb/cdcacm.h:227-228`）。変えるなら `CONFIG_BOARD_USBDEV_SERIALSTR=y` と
  `board_usbdev_serialstr()`。by-id 名は【推測】`usb-NuttX_NuttX_USB_Console_0-if00` のようになる。
- **プロンプト**: `CONFIG_NSH_PROMPT_STRING`（既定 `"nsh> "`）。環境変数 PS1 があればそちらが優先
  （`nsh_prompt.c`）。`"nsh$ "` の `$` が config.h にそのまま入るかはビルドで確かめる。
- **builtin と nsh コマンド**: 順序は「ファイル → builtin → nsh コマンド」で builtin が優先
  （`nsh_parse.c:516-552`、`NSH_BUILTIN_AS_COMMAND=n` のとき）。nsh 標準に `dmesg` がある。`wifi`・`kv`・`ntp`・
  `imu`・`nina`・`bootsel`・`i2c_dma` は同名なし。`NSH_DISABLE_DMESG=y` を推奨。

## 4. tickless

【事実】
- RP2040 に tickless/oneshot/alarm の実装は無く、`ARCH_CHIP_RP2040` は `ARCH_HAVE_TICKLESS` を select しない
  （`N/arch/arm/Kconfig:375-388`）。RP23XX は select（`:400`）。`rp2040_timerisr.c` は無条件にビルドされ、
  SysTick を周期で回して `up_timer_initialize()` を定義する。
- `RP2040_TIMER_IRQ_0..3`（`N/arch/arm/include/rp2040/irq.h:65-68`）と `RP2040_TIMER_BASE` 0x40054000
  （`memorymap.h:81`）は定義済み。**`hardware/rp2040_timer.h` は無い。** オフセット: ALARM0 0x10、ARMED 0x20、
  TIMERAWH 0x24、TIMERAWL 0x28、INTR 0x34、INTE 0x38、INTF 0x3c、INTS 0x40。
- TIMER はブート時にリセット解除され（`rp2040_clock.c:411-433`）、watchdog の tick は 1 MHz（`:207`）。
- **枠組み**: `ALARM_ARCH`（プロンプトあり、`if ONESHOT` の中）が `select ARCH_HAVE_TICKLESS`、
  `SCHED_TICKLESS_ALARM`、`SCHED_TICKLESS_LIMIT_MAX_SLEEP`（`N/drivers/timers/Kconfig:172-179`）。
  → **`ARCH_HAVE_TICKLESS` の select のためのパッチは要らない。** `ONESHOT_COUNT` はプロンプトの無い bool で
  select でしか立たない（`:154-159`）。`SCHED_TICKLESS_TICK_ARGUMENT` は 13.1.0 に無い。
  - `arch_alarm.c` が `up_alarm_set_lowerhalf`、`up_timer_gettick`、`up_alarm_tick_start`、weak の
    `up_udelay`/`up_mdelay`、`up_perf_*` を提供。`ALARM_ARCH` のとき `sched/clock/clock_delay.c` が外れる
    （`sched/clock/Make.defs:32-34`）。
  - 13.1.0 の tickless の中心は wdog（`sched/sched/sched_processtickless.c`）。alarm の割り込みが
    `nxsched_process_timer()` を呼ぶ。RR も wdog で動く。
- **手本**: `N/arch/arm/src/rp23xx/rp23xx_oneshot.c`（331 行、TIMER の ALARM0、`ONESHOT_COUNT`、clkcnt_t）。
  `rp23xx_timerisr.c` は `up_alarm_set_lowerhalf(rp23xx_oneshot_initialize())` を呼ぶ。
- `clock_t` は常に int64_t（`N/include/sys/types.h:248`）。`USEC_PER_TICK=100` で 64 ビット時刻に問題なし。

【提案】
1. ボードの Kconfig:
   ```kconfig
   config NANO_TICKLESS
   	bool "Tickless OS on RP2040 TIMER"
   	default y
   	select ONESHOT
   	select ALARM_ARCH
   	select ONESHOT_COUNT
   	select ONESHOT_FAST_DIVISION
   ```
   （M0 に除算器が無いため FAST_DIVISION。ボードの Kconfig からの select が効くかはビルドで確かめる。）
2. defconfig: `CONFIG_SCHED_TICKLESS=y`、`CONFIG_USEC_PER_TICK=100`。
3. `boards/…/src/nano_oneshot.c`: `rp23xx_oneshot.c` を写し、RP2040 の TIMER・`RP2040_TIMER_IRQ_0`・ALARM0 に。
4. `nano_timerisr.c`: `void up_timer_initialize(void) { up_alarm_set_lowerhalf(nano_oneshot_initialize()); }`
5. パッチ（`nuttx/patches/nuttx/` に 1 本。T05 の idle の分も同じファイル）:
   ```diff
   --- a/arch/arm/src/rp2040/Make.defs
   +++ b/arch/arm/src/rp2040/Make.defs
   @@
   -CHIP_CSRCS += rp2040_idle.c
   +ifneq ($(CONFIG_ARCH_IDLE_CUSTOM),y)
   +CHIP_CSRCS += rp2040_idle.c
   +endif
    CHIP_CSRCS += rp2040_irq.c
    CHIP_CSRCS += rp2040_uart.c
    CHIP_CSRCS += rp2040_serial.c
    CHIP_CSRCS += rp2040_start.c
   -CHIP_CSRCS += rp2040_timerisr.c
   +ifneq ($(CONFIG_ALARM_ARCH),y)
   +CHIP_CSRCS += rp2040_timerisr.c
   +endif
   ```
ALARM1〜3 は他の用途（I2C DMA の監視など）に残せる。SMP=n が前提。

## 5. アイドルと CPU 負荷

【事実】
- `N/arch/arm/src/rp2040/Make.defs:25` の `rp2040_idle.c` は無条件。`ARCH_IDLE_CUSTOM` で外れるのは
  `arm/src/common` の `arm_idle.c` だけ（`common/Make.defs:37-39`）。リンクは `--start-group` で libboard は最後
  なので、ボード側にも `up_idle` があると重複エラーか、黙って arch 側が使われる。→ 上のパッチで外す。
- 呼び出し元は `nx_start()` 末尾の `for(;;) up_idle();`、割り込み許可の状態（`N/sched/init/nx_start.c:765-770`）。
  13.1.0 の tickless はアイドル中に `nxsched_process_timer` を呼ぶ必要が無い。
- `SCHED_CPULOAD_SYSCLK`/`EXTCLK`/`CRITMONITOR` はどれも Zephyr と意味が違う（CRITMONITOR はスレッド切り替えで
  数えるのでアイドル中の割り込みがアイドル側に入る）。
- **Zephyr の意味**（`zephyr/zephyr/lib/os/cpu_load/cpu_load_backend_idle.c`、`tracing_none.c:10-36`）:
  アイドルに入るとき時刻を記録し、割り込みの入口（`sys_trace_isr_enter`）かアイドルの出口で窓を閉じる（冪等）。
  `cpu_load_get(reset)` は**前回 reset=true で呼んでから**の `1000*(total−idle)/total`（µs、切り捨て）。読んだ時点で
  窓が開いていればそこまでを加える。アプリは `imu start` で `cpu_load_get(true)`（`imu_stream.c:475,557,759`）。

【提案】`nano_idle.c`
```c
void up_idle(void)
{
  irqstate_t f = up_irq_save();          /* PRIMASK=1 (armv6-m) */
  uint32_t t0 = getreg32(TIMER+0x28);    /* TIMERAWL */
  g_idle_open = true; g_idle_t0 = t0;
  __asm__ volatile("wfi");               /* PRIMASK 中でも pending で起床 */
  g_idle_us += getreg32(TIMER+0x28) - t0;
  g_idle_open = false;
  up_irq_restore(f);                     /* ここで ISR が走り、負荷に数わる */
}
```
`nano_cpu_load_get(bool reset)` は割り込み禁止の中で total = TIMERAWH/L の 64 ビット差、idle = `g_idle_us`
（窓が開いていれば加算）を取り、Zephyr と同じ式・同じ reset の意味にする。PRIMASK 中の WFI の起床は実機で確認。

## 6. usrsock

【事実】
- `NET_USRSOCK_CUSTOM` は choice の中（`N/drivers/usrsock/Kconfig:10-29`）。宣言
  （`N/include/nuttx/net/usrsock.h`）: `usrsock_request(struct iovec*, unsigned)`（`:419`）、
  `usrsock_response(buf, len, bool *req_done)`（`:412`）、`usrsock_register(void)`（`:429`）。
  実例: `N/arch/arm/src/nrf91/nrf91_modem_sock.c`、`N/arch/sim/src/sim/sim_usrsock.c`。
- `usrsock_register()` は `net_initialize` から非常に早く呼ばれる。**ここで NINA を初期化しない。** 準備前の
  要求には `-ENETDOWN`。
- **要求の流れ**: `usrsock_request()` は `g_usrsock_lock`（rmutex）と `req->lock` を持ったまま呼ばれ、その後
  `acksem` を時間制限なし・割り込み不可で待つ（`usrsock_devif.c:640-686`）。中から `usrsock_response()` を
  呼んでよい。ACK を渡した後に負の値を返すと `acksem` の数がずれる。
- **ロックの順序**: 「`usrsock_lock()` → NINA のミューテックス」に固定。kthread もポーリングの前に
  `usrsock_lock()` を取る。逆にするとデッドロック。
- **(a) connect**: 非ブロッキングでは ACK の `flags=REQ_IN_PROGRESS`、`result=0` で `-EINPROGRESS`。完了は
  SENDTO_READY イベント（失敗は REMOTE_CLOSED）。同期で成功したら ACK の events に SENDTO_READY
  （`usrsock_event.c:72-86`）。2 回目の ACK で知らせる方式は xid が上書きされると捨てられる。接続の時間切れは
  ドライバー側で扱う。
- **(b) accept**: DATA_ACK で「アドレス（valuelen）→ int16 の新しい usockid」、`result=2`。ACK の後、新しい
  usockid に SENDTO_READY を送る。待ち受けソケットの RECVFROM_AVAIL が立っていないと、非ブロッキングでは
  ハンドラーへ届かず `-EAGAIN`（`usrsock_accept.c:307-332, 385-413`）。
- **(c) send**: SENDTO_READY が立っていないと、非ブロッキングはハンドラーへ送らず `-EAGAIN`、ブロッキングは
  `SO_SNDTIMEO` まで待つ。応答に SENDTO_READY が無ければフラグを下ろす。`-EAGAIN` なら待ち直し、0 なら 0 を返す
  （`usrsock_sendmsg.c:76-84, 286-320, 382`）。→ NINA が受け付けなかったら `-EAGAIN`。受け付けたら、まだ送れる
  見込みがある限り events に SENDTO_READY。
- **(d) recv**: RECVFROM_AVAIL が立っていないと、非ブロッキングはハンドラーへ送らず `-EAGAIN`。0 は EOF
  （`usrsock_recvmsg.c:88-97, 297-334`）。→ NTP の 1 ms ポーリングは recv ではできない。FIONREAD を使う。
- **(e) ioctl**: `usrsock_ioctl` は FIONBIO 以外の、長さの分かるコマンドをハンドラーへ送る。FIONREAD は必ず届く。
  **FIONREAD が 0 より大きいときは ACK の events に RECVFROM_AVAIL を入れる**（入れないと続く非ブロッキング recv が
  `-EAGAIN`）。無線系は `NETDEV_WIRELESS_IOCTL` のときポインターの先も付けて送る。`-ENOTTY` が返ったときだけ
  netdev の処理（`d_ipaddr`、`d_ioctl`）に落ちる（`netdev_ioctl.c:1675-1683, 623-656`）。`netdev_wifr_ioctl` は
  `d_ioctl` を NULL 検査なしで呼ぶ（`:652`）。
  - wapi と netlib は `SOCK_CTRL` を使う。`psock_socket` はまず usrsock を試し、`-ENOSYS`・`-ENOTSUP`・`-ENETDOWN`
    のときだけ通常の経路に戻る（`N/net/socket/socket.c:105-116`）。→ usrsock 側は `SOCK_CTRL` を `-ENOTSUP` で
    断り、netdev の `d_ioctl`/`d_ipaddr` で処理する。
- **(f) netdev**: `netdev_register(dev, NET_LL_IEEE80211)` には `DRIVERS_IEEE80211=y`。ifconfig は
  `/proc/net/<if>`（`d_ipaddr`、`d_draddr`、`d_netmask`、`IFF_UP`、`IFF_RUNNING`）を表示。gs2200m は `d_*` を自分で
  書き `IFF_UP` も自分で立てる（`drivers/wireless/gs2200m.c:1620, 1725-1727, 3425, 3529`）。`NET_TCP=n` には
  できない（telnet と DNS が依存）。`NET_USRSOCK_TCP`/`UDP=y` で「スタックなし」の TCP/UDP になる
  （`net/tcp/Kconfig:15-19`）。
- **(g) telnet**: telnetd は socket、SO_REUSEADDR（`NET_SOCKOPTS` のとき）、bind、listen(5)、accept4。SO_LINGER は
  スタックなしでは入らない。ドライバーは `psock_dup2`、recv、send、poll。setsockopt のうちカーネルが処理するのは
  SO_TYPE、SO_RCVTIMEO、SO_SNDTIMEO だけで、他はハンドラーへ届く（SO_REUSEADDR、SO_KEEPALIVE、TCP_NODELAY は
  0 で受けて無視。SO_ERROR は保持しているエラーを返す）。`NET_USRSOCK_NPOLLWAITERS` は既定 1。
  - **`TELNET_MAXLCLIENTS=1` だと 2 本目で telnetd が終了する**: 上限を超えると `telnet_session` が `-ENFILE`
    （`N/drivers/net/telnet.c:1013-1017`）、`telnetd_daemon` は待ち受けソケットも閉じて return
    （`A/netutils/telnetd/telnetd_daemon.c:249-255, 327-334`）。候補: (i) `MAXLCLIENTS=2`、(ii) usrsock の accept 側で
    セッション中は受け付けない、(iii) telnetd を直す。
- DNS は usrsock の UDP（connect、send、recv）を通る。`SOCK_NONBLOCK` は usrsock では効かない
  （`fcntl(O_NONBLOCK)` か `MSG_DONTWAIT`）。

## 7. RP2040 のドライバー（12.13 からの差は実質 `<nuttx/debug.h>` と usbdev の修正だけ）

- **`rp2040_i2c.c`**: 受信は 16 B の塊。STOP はメッセージの最後にだけ付き、RESTART ビットは使わない。1 MHz は
  t_low 500 ns / t_high 260 ns、FS のレジスタ、SPKLEN は clk_peri から。【計算】125 MHz で lcnt=62、hcnt=51。
  SDA_HOLD は 1 サイクル固定（pico-sdk は 1 MHz で 16）。待ちは 20 ms の wdog でタイムアウトしても abort しない。
  `static wostop` は I2C0 と I2C1 で共有。`CONFIG_RP2040_I2C=n` で丸ごと外れる（`Make.defs:64-66`、宣言は
  `rp2040_i2c.h:64,81`）。**外すとピン設定（`common_initialize.c:135-143`）とピン番号の Kconfig
  （`common/Kconfig:221`）も消えるのでボード側で持つ。**
- **`rp2040_dmac.h`**: `rp2040_dmachannel`、`dmafree`、`rx/txdmasetup(handle, paddr, maddr, n,
  dma_config_t{dreq,size,noincr})`、`dmastart(handle, cb, arg)`、`dmastop`、`dma_register`。DREQ: I2C0 TX=32、
  RX=33。割り込みは DMA_IRQ_0 だけで、コールバックは割り込み文脈。`rp2040_dmastop` は ABORT のビットが落ちるまで
  無期限に待つ（`rp2040_dmac.c:579-597`）。RP2040-E13（abort 後の遅れた完了割り込み）への備えも無い。
- **`hardware/rp2040_i2c.h`**: `IC_DMA_CR`、`TDLR`、`RDLR`（0x88/0x8c/0x90）とビット定義あり。
- **`rp2040_spi.c`**: 4 ワードを超えると DMA（`RP2040_SPI_DMA`）。DMA の待ちにタイムアウト無し。DMA チャネルを
  2 本常時確保。周波数は SSPCPSR だけで決まり、8 MHz 要求で 7.8125 MHz。CS はボード側の weak 関数
  `rp2040_spi0select`（SPI1 は `rp2040_spi1select`）。
- **`rp2040_gpio.h`**: `rp2040_gpio_irq_attach(gpio, mode, isr, arg)`、`enable_irq`、`disable_irq`。モードは
  LEVEL_LOW/HIGH = 1/2、EDGE_LOW/HIGH = 4/8。**`rp2040_gpio_clear_interrupt` はシフト量が誤っていて CLR エイリアスも
  使っているので使わない**（`:387-390`）。
- **`rp2040_flash_mtd.c`**: ROM_LOOKUP、`RAM_CODE`、boot2 を RAM に写して XIP を再開する方式、
  `enter_critical_section`、+0x03000000 の別名からの読み出しは流用できる。ioctl は GEOMETRY と BULKERASE だけ。
  `mtd_dev_s` に `FTL_BBM` のときだけ isbad/markbad が増えた（初期化子は指定子付きで書く）。
  `RP2040_FLASH_FILE_SYSTEM` は初回に「イメージの後ろから flash の末尾まで」を消去するので使わない。
- **`rp2040_rom.h`**: `ROM_RESET_USB_BOOT` は定義だけで呼び出しなし。
  `((void(*)(uint32_t,uint32_t))ROM_LOOKUP(ROM_RESET_USB_BOOT))(0,0)`。
- **common**: `rp2040_reset.c` の `board_reset` は status を無視して `up_systemreset()`（weak ではない）。
  `common_bringup` の初期化はすべて CONFIG で外せる（procfs のマウントもここ）。
  `board_uniqueid()` は flash の RUID を種にした xorshift の出力（`rp2040_uniqueid.c:262-285`）で、Zephyr 版の
  生の 8 B と違う。公開関数 `rp2040_flash_cmd()`（同ファイル `:196`、static でない）で 0x4B を読めば生の値が取れる。

## 8. KV

【事実】
- `mtd_config_nvs` は `MTDIOC_ERASESTATE` が必須で、無ければ登録に失敗（`N/drivers/mtd/mtd_config_nvs.c:1347-1353`）。
  in-tree の `rp2040_flash_mtd` は未対応。`progsize = geo.blocksize`。256 B のままだと ATE が 512 B になり、
  1 項目に最低 768 B【計算】。
- Kconfig は choice（`MTD_CONFIG`/`MTD_CONFIG_NVS`/`NONE`）。項目名の長さは `MTD_CONFIG_NAME_LEN`
  （既定 `NAME_MAX` = 32 でキーは 31 文字まで）。32 文字のキーには 33 以上。
- ioctl は `CFGDIOC_GET`、`SET`、`DEL`、`FIRST`、`NEXT` と BULKERASE（`FINDCONFIG` は扱わない）。
  `struct config_data_s{name[NAME_LEN], configdata, len}`（`N/include/nuttx/mtd/configdata.h`）。長さ 0 の SET は削除。
  1 項目の上限は `blocksize − 3*ate_size`。ブロックは 2 つ以上必要。登録は `mtdconfig_register(mtd)` で
  `/dev/config`（権限 0600）。
- 【提案】末尾 8 KB の自前 MTD で `blocksize` を 1（`MTD_BYTE_WRITE`）か 16 に見せ、書き込みは 0xFF 埋めの 256 B
  ページ単位、ERASESTATE（0xff）と BULKERASE を実装。【計算】`blocksize=1` ならキー 32 文字＋値 128 B の項目が
  177 B で、1 ブロックに約 22 項目。【推測】GC 先に 1 ブロック空く設計なので、生きているデータは 1 ブロックに
  収まる必要がある。

## 9. その他

- **wapi**: essid は `SIOCSIWESSID`、psk は `SIOCSIWAUTH`×2 と `SIOCSIWENCODEEXT`、disconnect は `SIOCSIWAP` に 0 を
  入れ乱数の ESSID を設定、scan は `SIOCSIWSCAN`/`SIOCGIWSCAN`。`d_ioctl` で実装。
- **iperf**: iperf2 系、ポート 5001、`-c`・`-s`・`-u`・`-t`・`-i`。バッファ 16 KB、スタック 4 KB×2。
- **uORB**: `UORB` は `SENSORS && USENSOR` に依存。`uorb_listener` と `sensortest`（`ENABLE_ALL_SIGNALS` が必要）。
- **ps のスタック**: `STACK_COLORATION=y` と `!NSH_DISABLE_PSSTACKUSAGE` で STACK・USED・FILLED の列。procfs が必要。
- **優先度継承**: `PRIORITY_INHERITANCE=y` でもセマフォは既定で継承なし（flags 0）。ミューテックスは継承付き。
  `nxsem_set_protocol()` もある。
- **ラウンドロビン**: `RR_INTERVAL > 0` で全タスクが SCHED_RR で始まる。13.1 で tickless のときの数え方が追加
  （`sched_roundrobin.c:232-289`）。
- **RAMLOG と syslog**: `RAMLOG_SYSLOG` で `/dev/kmsg`、`RAMLOG_BUFFER_SECTION` で置き場所のセクション指定。
  `syslog_channel_register(syslog_channel_t*)` は `SYSLOG_REGISTER` が必要。`MAX_CHANNELS` が 1 だと 0 番を置き換える。
  ops は `{sc_putc, sc_force, sc_flush, sc_write, sc_write_force, sc_close}`。nsh の dmesg は `SYSLOG_DEVPATH` を cat。
- **board_uniqueid**: `CONFIG_BOARDCTL_UNIQUEID`、`BOARDCTL_UNIQUEID_SIZE`（既定 16）。
- 違いが無かったもの: boot2（at25sf128a が選べる）、`ROM_RESET_USB_BOOT` の定義、`PRIORITY_INHERITANCE` の振る舞い。

## T02・T03 の実装者向け

**流用元**（`N/boards/arm/rp2040/raspberrypi-pico/` から写して改名）
- `scripts/Make.defs`（`LDSCRIPT` を自前の ld に。`ARCHSCRIPT += $(BOARD_DIR)/scripts/$(LDSCRIPT)` の形は残す）
- `scripts/raspberrypi-pico-flash.ld`
- `src/Make.defs`（`VPATH`、`DEPPATH`、`-I…/board/board` の行は残す）
- `src/rp2040_boardinitialize.c`（early と initialize で `rp2040_common_*`、`board_late_initialize` から bringup）
- `src/rp2040_bringup.c`
- `include/board.h`（クロック 125 MHz、`BOARD_TICK_CLOCK`）。`rp2040_i2cdev.h`・`spidev.h`・`i2sdev.h`・`spisd.h` も
  写す（board.h が include し、common のソースも使う）
- 手本: `N/arch/arm/src/rp23xx/rp23xx_oneshot.c`・`rp23xx_timerisr.c`（T03b）、
  `A/netutils/telnetd/telnetd_daemon.c:235-299`（init の dup2 と task_create）、`N/drivers/wireless/gs2200m.c`（netdev）

**defconfig の主な項目（未ビルド）**
```
CONFIG_ARCH="arm"
CONFIG_ARCH_CHIP="rp2040"
CONFIG_ARCH_CHIP_RP2040=y
CONFIG_ARCH_BOARD_CUSTOM=y
CONFIG_ARCH_BOARD_CUSTOM_DIR="../boards/arduino-nano-rp2040-connect"
CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y
CONFIG_ARCH_BOARD_CUSTOM_NAME="arduino-nano-rp2040-connect"
CONFIG_ARCH_BOARD_COMMON=y
CONFIG_APPS_DIR="../nuttx-apps"
CONFIG_ARCH_RAMVECTORS=y
CONFIG_ARCH_STACKDUMP=y
CONFIG_RAM_START=0x20000000
CONFIG_RAM_SIZE=270336
CONFIG_RP2040_FLASH_BOOT=y
CONFIG_RP2040_FLASH_CHIP="at25sf128a"
# CONFIG_RP2040_UF2_BINARY is not set
CONFIG_RAW_BINARY=y
CONFIG_BOARD_LOOPSPERMSEC=10450
# CONFIG_RP2040_UART0 is not set
# CONFIG_DEV_CONSOLE is not set
CONFIG_USBDEV=y
CONFIG_USBDEV_BUSPOWERED=y
CONFIG_CDCACM=y
# CONFIG_CDCACM_CONSOLE is not set
CONFIG_CDCACM_PRODUCTSTR="NuttX USB Console"
CONFIG_BOARD_USBDEV_SERIALSTR=y        # シリアル文字列を変えるときだけ
CONFIG_BOARD_LATE_INITIALIZE=y          # 既定 y。ここで cdcacm_initialize(0,NULL)
CONFIG_BOARDCTL=y
CONFIG_BOARDCTL_RESET=y                 # nsh の reboot
CONFIG_INIT_ENTRYPOINT="nanoinit_main"  # apps/init の PROGNAME=nanoinit
CONFIG_SCHED_WAITPID=y
CONFIG_BUILTIN=y
CONFIG_NSH_LIBRARY=y
CONFIG_NSH_CONSOLE=y
# CONFIG_NSH_USBCONSOLE is not set
CONFIG_NSH_BUILTIN_APPS=y
CONFIG_NSH_READLINE=y
CONFIG_READLINE_ECHO=y
CONFIG_NSH_PROMPT_STRING="nsh$ "
CONFIG_NSH_DISABLE_DMESG=y
CONFIG_FS_PROCFS=y
CONFIG_STACK_COLORATION=y
# T03b
CONFIG_ONESHOT=y
CONFIG_ALARM_ARCH=y
CONFIG_SCHED_TICKLESS=y
CONFIG_USEC_PER_TICK=100
# T05
CONFIG_ARCH_IDLE_CUSTOM=y
```
（`RAM_SIZE`、`LOOPSPERMSEC`、`RAMVECTORS` は pico の usbnsh の値。）

**T03 の init の流れ**【提案】
1. `nsh_initialize()`。
2. 繰り返し: `open("/dev/ttyACM0", O_RDWR)` を開けるまで（`-ENOTCONN` の間）短い間隔で再試行 → termios に
   `OPOST|ONLCR` → `dup2` で 0/1/2 にして元の fd を閉じる → `task_create("nsh", 100, stack, nsh_consolemain, NULL)`
   → `waitpid` → fd 0/1/2 を閉じて最初に戻る。

**`bootsel`**: 出力を出し切ってから（`fsync` や短い待ち）ROM の `reset_usb_boot(0,0)`。in-tree の `board_reset` には
手を入れない。

**build.sh**: `PICO_SDK_PATH` を確かめる。configure に `-a ../nuttx-apps`。ボードの Kconfig や apps のディレクトリを
変えたら `N/boards/dummy/Kconfig` と `nuttx/apps/Kconfig` を消す（または distclean）。`.boot2` が 0x10000000 から
256 B あることを確かめる。
