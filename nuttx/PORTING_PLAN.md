# NuttX 移植計画（Arduino Nano RP2040 Connect IMU 測定ファームウェア）

## Context

`zephyr/app/usb-console` の IMU 測定ファームウェア（LSM6DSOX の RAW を 6667 Hz で取得し、PC 時刻を付けて
MQTT で送る）を、Zephyr 版に忠実に NuttX へ移植する。置き場所は `nuttx/`（2026-11-13 に `zephyr/` を
分けたのはこのため）。Zephyr 版は変更しない。完成条件は、Zephyr 版と同じ機能が動き、`imu-host` と
ImuStudio が（USB の検出を除いて）無変更で使えること。

**NuttX は 13.1.0 を使う**（nuttx と nuttx-apps の両方、タグ `nuttx-13.1.0`）。この計画の NuttX 側の事実は、
手元にあった別の NuttX のツリー（12.13.0 系、読むだけ）で確かめたもので、13.1.0 では
未確認。T01 の最後に 13.1.0 のソースで下の「版の再確認」を行い、違いがあれば計画を直してから先へ進む。

版の再確認（T01b）で見る点: カスタムボードと `nuttx/external`・`nuttx-apps/external` の仕組み、
`tools/configure.sh`・`sethost.sh` の要求ツール、RP2040 の boot2 と UF2 生成、nshlib の USB コンソール
（Enter 3 回）、`NET_USRSOCK_CUSTOM` と `usrsock_request()` の形、`mtd_config_nvs` の Kconfig、
`rp2040_i2c.c`・`rp2040_dmac.h`・`rp2040_flash_mtd.c`・`rp2040_idle.c`、RP2040 の tickless 対応の有無
（13.1.0 で入っていれば T03b は設定だけで済む）、`ROM_RESET_USB_BOOT` の呼び出し実装の有無、
`TELNET_MAXLCLIENTS`、wapi、iperf。

## 決定事項

| 項目 | 決定 |
|---|---|
| NuttX 本体 | upstream の **13.1.0**（タグ `nuttx-13.1.0`）を `nuttx/nuttx`、`nuttx/nuttx-apps` に clone（Git 管理外）。本体の変更は (1) Kconfig で in-tree 側を無効にして out-of-tree の写しをビルド、(2) 無理なら `nuttx/patches/*.patch` を setup で適用 |
| 共通ソース | OS 非依存のソースは `nuttx/` にコピー（無変更）。Zephyr 側は触らない |
| 範囲 | 全機能同等。独自コマンド `imu`・`kv`・`ntp`・`nina`・`dmesg`・`i2c_dma` は構文・出力とも同じ。`wifi cred add/delete/list/auto_connect` は Zephyr と同じ構文で独自実装。それ以外の Zephyr 標準コマンドは NuttX 標準（`wapi`、`ifconfig`、`iperf`、`ps`、`reboot`、`i2c`、uORB のツール）に任せ、対応表を文書に書く |
| ホスト互換 | USB 製品文字列は `NuttX USB Console`、プロンプトは `nsh$ `。`host/firmware_profiles.toml` に差分を書き、`board.py` が by-id 名とプロンプトでファームウェアを判定する |
| I2C | 最初から DMA 読み出しを移植 |
| ツール | sudo なしで `nuttx/` 配下に導入（Arm GNU Toolchain の tarball、`.venv` に kconfiglib、UF2 は自前の `bin2uf2.py`）。`zephyr/` に依存しない |
| 外部ソース | `setup.sh` が取得して Git 管理外に置く: pico-sdk（**boot2 のビルドにだけ必要**。AT25SF128A 用 second stage のソースと `pad_checksum`。実行時の HAL は使わない）、ST の `lsm6dso_reg.[ch]`（Zephyr の hal_st と同じ版に固定） |
| KV | flash 末尾 8 KB（`0xffe000`、4 KB×2）を見せる小さな MTD に NuttX の NVS（`MTD_CONFIG_NVS` + `MTD_CONFIG_NAMED`）。Zephyr 版との互換なし。退避・再投入の手順を文書化 |
| 速度測定 | NuttX 標準の `iperf`（iperf2 系）。PC 側の `iperf` は導入済み |
| BOOTSEL | 開発用に `bootsel` コマンド（ROM の `reset_usb_boot`）を用意し、`nuttx/script/flash.sh` で書き込みを自動化。手動 BOOTSEL は最初の 1 回だけ。完成時に残すかはユーザーに確認する |

## 技術方針（確認済みの根拠つき）

- **ビルド**: Make。カスタムボード（`CONFIG_ARCH_BOARD_CUSTOM*`）＋ `nuttx/nuttx/external`（カーネル側
  ライブラリ、symlink）＋ `nuttx/nuttx-apps/external`（アプリ、symlink）。`boards/arm/rp2040/common` は
  in-tree のものを流用。`raspberrypi-pico` のボードファイルを写して改名。リンカースクリプトは自前
  （16 MB、末尾 8 KB を除外、`.noinit` を追加）。
- **コンソール**: NuttX の USB コンソール nsh は Enter 3 回を待つので使わない。`CDCACM_CONSOLE` と
  `NSH_USBCONSOLE` を off にし、自前の init が `/dev/ttyACM0` を開いて `nsh_consolemain()` を呼ぶ。
  Telnet は `NSH_TELNET` + `TELNET_MAXLCLIENTS=1`。
- **NINA**: `NET_USRSOCK_CUSTOM` でカーネル内に `usrsock_request()` を実装し、要求を呼び出し元スレッドの
  文脈で同期処理する（実例: `arch/arm/src/nrf91/nrf91_modem_sock.c`）。Zephyr 版の「呼び出し元の優先度で
  SPI、`nina_lock` で直列化」と同じ形。ポーリング（20 ms、1 ms 再送）は小さな kthread がイベントに変換。
  NTP の 1 ms ポーリングは `ioctl(FIONREAD)` でその場の CMD_POLL にする。`wlan0` を netdev 登録して
  `ifconfig`・`wapi` に見せる。READY 待ちは Zephyr 版と同じ 100 µs 間隔のポーリング（tickless 化で成り立つ。
  T03b が周期 tick に落ちた場合も 100 µs tick なので同じ）。
- **LSM6DSO**: ドライバーは `nuttx/drivers/lsm6dso/`。RTIO の使っている部分だけの互換層
  （`imu_rtio.[ch]`: 128×64 B のプール、完了 16 個）を置き、`lsm6dso_stream.c` の状態機械をそのまま移す。
  ワーカーは専用 kthread。`k_poll` は 1 本の起床セマフォ＋フラグに置き換える。
- **I2C DMA**: `CONFIG_RP2040_I2C=n` にして `rp2040_i2c.c` をボード側に写し、32〜3584 B・STOP 終わりの
  読み出しに DMA 分岐を足す。`rp2040_dmastop()` は無期限待ちなので使わず、期限つきで止める。
- **時計**: `time_sync_mono_us()` は TIMER の TIMERAWH/TIMERAWL（`0x40054000+0x24/0x28`）を直読み。
- **tick**: NuttX の RP2040 移植には tickless の実装が無い（SysTick の周期割り込みだけで、
  `ARCH_CHIP_RP2040` が `ARCH_HAVE_TICKLESS` を select していない。ハードウェアの制約ではない）。Zephyr 版は
  100 µs 刻みの tickless なので、RP2040 の TIMER（64 ビット 1 MHz、アラーム）を使った oneshot タイマーを
  out-of-tree で実装し、`SCHED_TICKLESS` + `USEC_PER_TICK=100` で動かす（T03b）。`ARCH_HAVE_TICKLESS` の
  select と `rp2040_timerisr.c` の除外は `.patch` になる見込み。うまくいかなければ 100 µs の周期 tick に
  落とし、ユーザーに報告する。
- **CPU 負荷（`cpu_active`）**: Zephyr 版は `CPU_LOAD_BACKEND_IDLE_HOOK` で、アイドル窓を**割り込みの入口で
  閉じる**（`zephyr/zephyr/subsys/tracing/tracing_none.c` の `sys_trace_isr_enter` →
  `cpu_load_on_exit_idle`、`lib/os/cpu_load/cpu_load_backend_idle.c`）。アイドル中に入った割り込みの処理時間は
  負荷に数えられる。NuttX の `SCHED_CPULOAD_SYSCLK`（tick でのサンプリング）は別物なので使わない。
  自前で同じ計測を実装する: アイドルループで割り込みを禁止 → TIMER を読む → `WFI`（PRIMASK 中でも
  起床し、ISR はまだ走らない）→ TIMER を読んでアイドル時間に加算 → 割り込み許可（ここで ISR が走り、
  負荷側に入る）。`rp2040_idle.c` は無条件にビルドされるので、`ARCH_IDLE_CUSTOM` で外す `.patch` を当てて
  ボード側の `up_idle()` に置き換える。`cpu_load_get(reset)` と同じ意味（リセットからの千分率）の関数を出す。
- **優先度**: `NuttX = 118 − Zephyr`（Wi-Fi 再接続 110、取得・時刻同期・NINA 監視 109、FIFO ワーカー 108、
  MQTT 107、NINA イベント 106、nsh 100）。`PRIORITY_INHERITANCE=y`。通知用セマフォは `SEM_PRIO_NONE`。
  同じ優先度のスレッドの時分割（Zephyr 版の `TIMESLICING` の設定）も合わせる。
- **MQTT**: 自前の最小実装（CONNECT/CONNACK、PUBLISH QoS0/1、PUBACK、PING、DISCONNECT）。Zephyr の
  mqtt_lib のバイト列・既定値に合わせ、ホストテストを付ける。
- **ログ**: `log_ram.c` を移植（`.noinit` の 4 KiB、syslog チャネル登録、`<inf>`/`<err>` の体裁）。

## 13.1.0 での再確認による修正（T01b、2026-11-13）

詳細（ファイルと行、雛形、defconfig の項目）は `nuttx/PORTING_NOTES.md` にある。**この節は上の「技術方針」と
タスク表より優先する。** 実装の指示を書く前に、該当する項目を `PORTING_NOTES.md` で読む。

1. **apps ディレクトリの取り違え**: `tools/configure.sh` は `-a` が無いと `TOPDIR/../apps`（= 自前の
   `nuttx/apps/`）を apps として使う。`build.sh` は必ず `-a ../nuttx-apps` を渡し、defconfig にも
   `CONFIG_APPS_DIR="../nuttx-apps"` を書く。
2. **tickless（T03b）**: 13.1.0 には RP2350 用の手本 `arch/arm/src/rp23xx/rp23xx_oneshot.c`（TIMER の ALARM0、
   `ONESHOT_COUNT`）と `rp23xx_timerisr.c` がある。これを写して `boards/…/src/nano_oneshot.c`・
   `nano_timerisr.c` にする。`CONFIG_ONESHOT`・`ALARM_ARCH`（`ARCH_HAVE_TICKLESS` を select する）・
   `SCHED_TICKLESS`・`USEC_PER_TICK=100`。`hardware/rp2040_timer.h` は無いのでレジスタは自分で定義する。
   パッチは `arch/arm/src/rp2040/Make.defs` の 1 本だけ（`rp2040_timerisr.c` を `ALARM_ARCH` で、
   `rp2040_idle.c` を `ARCH_IDLE_CUSTOM` で外す）。
3. **init（T03）**: 13.1.0 では `NSH_ARCHINIT`・`BOARDIOC_INIT`・`board_app_initialize()` が無い。ボードの初期化は
   `board_late_initialize()`（ここで `cdcacm_initialize(0, NULL)`）。`nano_appinit.c` は作らない。
   `nsh_consolemain()` は終わると `exit()` するので、init は telnetd と同じ形にする: `nsh_initialize()` →
   繰り返し（`/dev/ttyACM0` を開けるまで再試行 → termios で `OPOST|ONLCR` → `dup2` で 0/1/2 →
   `task_create` で `nsh_consolemain` → `waitpid` → 閉じる）。Telnet は NINA の準備後に `nsh_telnetstart()`。
   USB のシリアル文字列は `board_usbdev_serialstr()` でしか変えられない。入力の CR の扱いは、ホストが
   `\r\n` を送ること（`board.py`）と端末ソフトの CR だけの Enter の両方で確かめて決める。
4. **アイドル（T05）**: `ARCH_IDLE_CUSTOM` だけでは `rp2040_idle.c` は外れない（上のパッチで外す）。
   `cpu_load_get(reset)` の意味は「前回 reset=true で呼んでから」の `1000*(total−idle)/total`。
5. **Telnet（T18）**: `TELNET_MAXLCLIENTS=1` だと、2 本目の接続で telnetd が待ち受けごと終了する。Zephyr 版は
   2 本目を断って 1 本目を保つ（`Telnet client already connected`）。同じ振る舞いにするため、usrsock の
   accept 側でセッション中は受け付けない、または telnetd を直す（パッチ）。T18 で 13.1.0 のコードを読んで決め、
   CP4 で確かめる。
6. **KV（T06）**: `mtd_config_nvs` は `MTDIOC_ERASESTATE` が必須で、`progsize = blocksize`。自前 MTD は
   `blocksize` を小さく（1 か 16）見せ、書き込みは 0xFF 埋めの 256 B ページ単位、ERASESTATE と BULKERASE を
   実装する。`MTD_CONFIG_NAME_LEN` を 33 以上にする（キー 32 文字）。生きているデータは 1 ブロック（4 KB）に
   収まる必要がある見込み。
7. **チップ ID（T05/T12）**: `board_uniqueid()` は flash の RUID を xorshift でかき混ぜた値で、Zephyr 版（生の
   8 B）と違う。`rp2040_flash_cmd()` で 0x4B を読む自前の関数をボード側に持つ。
8. **usrsock（T14〜T16）**: ロックの順序は「`usrsock_lock()` → NINA のミューテックス」に固定（kthread も先に
   `usrsock_lock()`）。非ブロッキング connect の完了は SENDTO_READY イベント（失敗は REMOTE_CLOSED）。
   accept は待ち受けソケットの RECVFROM_AVAIL が前提。FIONREAD が 0 より大きいときは ACK の events に
   RECVFROM_AVAIL を入れる。`SOCK_CTRL`（wapi、ifconfig）は usrsock 側で `-ENOTSUP` にして netdev の `d_ioctl` で
   処理する。`usrsock_register()` は起動の早い時期に呼ばれるので、そこで NINA を初期化しない。
   `SOCK_NONBLOCK` は効かない（`fcntl(O_NONBLOCK)` か `MSG_DONTWAIT`）。
9. **I2C（T08）**: `CONFIG_RP2040_I2C=n` にするとピン設定とピン番号の Kconfig も消えるので、ボード側で持つ。
   in-tree の 1 MHz 設定は SDA_HOLD が 1 サイクル固定（pico-sdk は 16）。Zephyr 版の値と比べて合わせる。
   `rp2040_gpio_clear_interrupt` は不具合があるので使わない。
10. **その他**: ボードの Kconfig と `nuttx/apps/Kconfig`（自動生成）は初回しか作られないので、変えたら
    `build.sh` が消してから configure する。`nuttx/boards/` の直下に `common/`・`drivers/` を作らない。
    `PICO_SDK_PATH` が未設定でもエラーにならず boot2 の無いイメージができるので、`build.sh` で `.boot2` が
    256 B あることを検査する。セマフォは既定で優先度継承なし、ミューテックスは継承あり。

## ディレクトリ構成

```
nuttx/
  script/     setup.sh build.sh flash.sh host_tests.sh bin2uf2.py kconfig-tweak
  boards/arduino-nano-rp2040-connect/   Kconfig configs/imu/defconfig include/ scripts/ src/
  drivers/    Kconfig Makefile nina/ lsm6dso/ include/        （nuttx/nuttx/external へ symlink）
  apps/       Makefile Make.defs init/ imu/ kv/ ntp/ nina/ dmesg/ i2c_dma/ wifi/ bootsel/
  tests/      ホストテストと偽ヘッダー
  patches/    必要になったときだけ
  README.md README_ja.md  drivers/README.md drivers/README_ja.md
  nuttx/ nuttx-apps/ pico-sdk/ stmemsc/ toolchain/ .venv/     … Git 管理外
```

## 進め方（1 タスクの流れ）

1. **実装**: subagent（`model: opus`）に依頼。指示には、対応する Zephyr 版のファイルと行、守るべき定数・
   順序・出力文字列、作るファイル、この計画の該当節、`CLAUDE.md` の注意を入れる。写しを作るタスクでは
   元ファイルとの diff を報告させる。あわせて **Zephyr 版との機能差分表**（振る舞いごとに「同じ／違う／
   未実装」と理由）を報告させる。
2. **レビュー**: メインが diff を読み、**Zephyr 版の対応ソースを自分で開いて並べ**、機能差分を確認する
   （subagent の差分表は手がかりで、鵜呑みにしない）。見る点は、定数・タイムアウト・順序・エラー値・
   出力文字列に加えて、「何を数えているか・いつ測るか」のような意味の一致（例: CPU 負荷がアイドル中の
   割り込みを負荷に数えるか、時刻をどの時点で読むか、カウンターがどの事象で増えるか）と、Zephyr の
   サブシステムが暗黙にやっていた動作（既定値、再試行、優先度継承、バッファ寸法）。
3. **修正**: 問題があれば、指摘を具体的に書いて subagent（opus）に直させ、再レビューする。機能差分は
   無くすのが原則。NuttX の制約で残る差分は、理由を付けてユーザーに相談してから進める。
4. **動作確認**: 通過したら、確認内容を subagent（opus）に渡して実行させる（ビルド、ホストテスト、節目では
   実機）。結果は測定条件・回数・未確認の範囲を区別して報告させる。
5. **commit**: タスクごとに `main` へ commit（push はしない）。`CLAUDE.md` は未追跡のまま。

### compact や clear が入っても続けられるようにする

メインの会話の内容に頼らず、次の 3 つだけで再開できる状態を保つ。

- **`nuttx/PORTING_PLAN.md`**: この計画の写し（未追跡）。最初に作り、計画を変えたら直す。
- **`nuttx/PORTING_STATUS.md`**: 進捗の記録（未追跡）。タスクごとに、状態（未着手／実装中／レビュー中／
  修正中／確認中／完了）、commit、レビューで出た指摘と結果、確認の結果（条件・回数）、**Zephyr 版との
  機能差分で残っているもの**、ユーザーに確認待ちの事項、実機の現在の状態（載っているファームウェア、
  KV の内容、AP の状態）を書く。**状態が変わるたびに、次の作業へ進む前に更新する。**
- **`CLAUDE.md`**: 先頭近くに「NuttX 移植中。作業を始める前に `nuttx/PORTING_PLAN.md` と
  `nuttx/PORTING_STATUS.md` を読む」と書く（未追跡のまま）。

再開時は、この 2 ファイルと `git log`・`git status` を読み、実機の状態を確認してから続ける。subagent への
指示は毎回それだけで完結させ、会話の経緯を前提にしない。移植が終わったら、残す内容を README・設計書に
移し、`PORTING_*.md` を消してよいかユーザーに確認する。

タスクは原則として順に進める。ホストだけで完結するもの（T04、T20）は他と並行させてよい。実機確認で問題が
出たら、原因のタスクへ修正を戻す。実機を使う前には毎回 `lsusb`・`/dev/serial/by-id/`・`lsblk` で状態を
確認し、ユーザーが起動した AP は止めない。sudo の要る操作はユーザーに依頼する。

## タスク

| ID | 内容 | 依存 | 主なファイル（`nuttx/` 以下） | 対応する Zephyr 版 | 確認 |
|---|---|---|---|---|---|
| T01 | 環境構築 | – | `script/setup.sh`、`script/kconfig-tweak`、ルート `.gitignore`、README 骨組み | `zephyr/script/setup.sh` | 新しいシェルで source し、gcc・menuconfig・kconfig-tweak が動く。再実行して壊れない |
| T01b | 13.1.0 での版の再確認（Context の一覧）。結果を `PORTING_STATUS.md` に書き、計画との違いを報告 | T01 | – | – | 違いがあれば計画を直し、大きな変更はユーザーに相談 |
| T02 | ボード定義とビルド | T01b | `boards/…/{Kconfig,configs/imu/defconfig,include/board.h,scripts/*,src/nano_{boardinitialize,bringup,appinit}.c}`、`script/{build.sh,bin2uf2.py}` | ボード DT、`app.overlay` | ビルドが通る。`.boot2` が 0x10000000 に 256 B。UF2 のアドレス範囲 |
| T03 | USB コンソール、nsh、`bootsel`、`flash.sh` | T02 | `apps/{Makefile,Make.defs,init/,bootsel/}`、`boards/…/src/nano_{reset,usbserial}.c` | console/shell の設定 | **CP1** |
| T04 | 共通ソースのコピーとホストテスト | T01 | `apps/imu/imu_raw_*`、`imu_tick_clock`、`time_offset`、`imu_cal_codec.c`、`drivers/lsm6dso/lsm6dso_{fifo,cal,zero,fifo_decoder}`、`tests/`、`script/host_tests.sh` | `src/`、`drivers/lsm6dso/`、`tests/` | `diff` で一致。ホストテスト全通過 |
| T03b | tickless（TIMER のアラームによる oneshot、100 µs 刻み） | T03 | `boards/…/src/nano_oneshot.c`（または `nano_tickless.c`）、`patches/`、defconfig | Zephyr の tickless（100 µs tick） | ビルド。CP1 に続けて実機で: `sleep 10` の実時間、1 ms の sleep を 1000 回繰り返した所要時間（TIMER で測る）、アイドル時に周期割り込みが無いこと、USB コンソールと `bootsel` が動くこと |
| T25 | KV の移行スクリプト（2026-11-13 にユーザーが追加で依頼）。`backup`: Zephyr 版から既知のキーを `kv get` で読んでファイルに保存。`restore`: NuttX 版へ `kv sethex` で書き戻し、Wi-Fi を登録（SSID は既定 `rp2040-test`、`--ssid` で指定可、パスワードは標準入力） | –（restore の実機確認は T06・T17 の後） | `script/kv_transfer.py` | `kv_store.c` のコマンド、`wifi cred add` | backup は CP1 の前に Zephyr 版で実行。restore の KV は CP2、Wi-Fi は CP4 で確認 |
| T05 | 時計、ログ、`dmesg`、CPU 負荷（アイドル窓の実測）、優先度 | T03b | `apps/imu/{platform,app_log,priorities}.[ch]`、`log_ram.c`、`apps/dmesg/`、`boards/…/src/nano_idle.c`、`patches/` | `time_sync.c:45-63`、`log_ram.c`、Zephyr `lib/os/cpu_load/cpu_load_backend_idle.c` と `tracing_none.c` | CP2。CPU 負荷は、アイドル時にほぼ 0、ビジーループで約 1000、割り込みだけが多い状況（USB 転送中など）で 0 にならないことを見る |
| T06 | KV | T05 | `boards/…/src/nano_flash_kv.c`、`apps/imu/kv_store.[ch]`、`apps/kv/` | `kv_store.c` | CP2 |
| T07 | NINA の SPI 転送と `nina version` | T03 | `drivers/{Kconfig,Makefile}`、`drivers/nina/{nina_spi.c,nina_proto.h}`、`boards/…/src/nano_nina.c`、`apps/nina/`、`tests/nina_spi.c` | `nina_wifi.c`、`main.c:13-34` | ホストテスト、CP2 |
| T08 | I2C ドライバー（DMA なし） | T03 | `boards/…/src/nano_i2c.[ch]` | `rp2040_i2c.c` の写し | CP2 |
| T09 | I2C の DMA 読み出しと `i2c_dma` | T08 | `nano_i2c.c`、`nano_i2c_commands.[ch]`、`apps/i2c_dma/`、`tests/i2c_dma_commands.c` | `i2c_dw.c:578-801` | ホストテスト、CP3 |
| T10 | LSM6DSO ドライバー本体 | T04, T08 | `drivers/lsm6dso/{lsm6dso.[ch],lsm6dso_bus.c,lsm6dso_fifo_hw.c}`、`boards/…/src/nano_lsm6dso.c` | `lsm6dso.c`（shub・trigger 除く） | ホストテスト、CP3 |
| T11 | RTIO 互換層と FIFO ストリーム | T10 | `drivers/lsm6dso/{imu_rtio.[ch],lsm6dso_stream.c,lsm6dso_fifo_irq.c}`、`tests/stream_stop.c` | `lsm6dso_stream.c`、`lsm6dso_fifo_irq.c` | ホストテスト `stream_stop` |
| T12 | 取得サービス（送信はスタブ） | T05, T06, T11 | `apps/imu/{imu_stream.c,imu_main.c,imu_raw_mqtt_null.c}` | `imu_stream.c` | ビルド |
| T13 | CP3 の実機確認 | T09, T12 | – | – | **CP3** |
| T14 | usrsock 制御系とイベント kthread | T07 | `drivers/nina/{nina_usrsock.c,nina_sock.[ch],nina_poller.c}` | `nina_socket.c:1-372, 668-688, 835-882` | ビルド |
| T15 | usrsock データ系 | T14 | 同上 | `nina_socket.c:375-666, 762-833`、`nina_dns.c` | ビルド |
| T16 | Wi-Fi 管理（netdev、wapi、監視、スキャン） | T14 | `drivers/nina/nina_net.c` | `nina_net.c` | CP4 |
| T17 | 資格情報の保存、自動再接続、`wifi cred` | T06, T16 | `apps/imu/wifi_profile.[ch]`、`apps/wifi/` | `wifi_profile.c`、`wifi cred` の構文 | CP4 |
| T18 | Telnet と iperf | T15, T16 | defconfig、init | `prj.conf` のネットワーク設定 | **CP4** |
| T19 | 時刻同期と `ntp status` | T05, T15, T16 | `apps/imu/time_sync.[ch]`、`apps/ntp/` | `time_sync.c` | CP5 |
| T20 | MQTT 最小クライアント | T04 | `apps/imu/mqtt_min.[ch]`、`tests/mqtt_min.c` | Zephyr `subsys/net/lib/mqtt/` の使用範囲 | ホストテスト |
| T21 | `imu_raw_mqtt` の移植 | T12, T15, T20 | `apps/imu/imu_raw_mqtt.c` | `imu_raw_mqtt.c` | **CP5** |
| T22 | 校正と `imu cal` | T06, T10, T12 | `apps/imu/imu_cal.c` | `imu_cal.c` | CP6 |
| T23 | sensor 相当（uORB 登録、FIFO の属性） | T10, T11 | `drivers/lsm6dso/lsm6dso_uorb.c` | `sensor_shell.c` の使用範囲 | CP6 |
| T24 | ホストの差分ファイルと文書 | T21 | `host/firmware_profiles.toml`、`host/imu_host/board.py`、README・設計書・ドライバー説明・`host/README`（各日英）、`CLAUDE.md` | – | **CP6** |

各タスクで守る忠実性の要点（実装指示とレビューで Zephyr 版と並べて確認する）:

- **T03**: `flash.sh` は `/dev/ttyACM*` やマウントパスを固定しない。`bootsel` は出力を出し切ってから ROM を呼ぶ。
- **T06**: 構文と応答（`Saved`、`Reserved key`、16 進表示）、キー 32 文字・値 128 B、`sys.` の拒否。消去範囲が
  `0xffe000`〜`0xffffff` だけであること（定数と ld の両方）。
- **T07**: リセット手順（10 ms、750 ms）、READY の 100 µs ポーリングとタイムアウト（2000/20/10000 ms）、フレーム形式、
  CS を READY 待ちの間も保持、エラー値。
- **T09**: 条件（32〜3584 B、STOP、I2C0）、失敗時の abort と 100 ms、統計 4 項目とコマンド出力。
- **T10/T11**: レジスタ設定の順序、`STREAM_CANCEL_POLL_MS 20`、オーバーラン判定、停止の順序（pause → 読み切り →
  stop → 終端）、終端のエラー値。`stream_work()` の分岐が 1 対 1。
- **T12**: `imu_stream.c:42-62` の定数、回復（200 ms、3 回）、`restart_blocked`、セッション ID、`imu start`/`status` の
  出力文字列が完全一致（ホストが `Streaming`、`port N`、`state:` を照合する）。
- **T14/T15**: ソケット 10 本、`NINA_MAX_IO 512`、`NINA_TCP_MAX_IO 3584`、connect 10 s/20 ms、再送 1 ms・状態確認
  100 ms・送信期限 10 s、UDP の peer フィルタ、`TCP_NODELAY` は受け付けるだけ。ロックの順序。
- **T16/T17**: 監視 2 s、未接続 2 回で切断扱い、スキャン後の復帰、再接続スレッドの待ち（4 s、32 s）、手動切断後は
  再接続しない。パスフレーズをログに出さない。
- **T19**: NTP パケットと応答検査、t1/t4 を取る位置、直近 8 回の最短往復、60 秒で無効、slew ±500 ppm、
  実時間時計は前にだけ進める、`ntp status` の出力。
- **T21**: トピックと QoS、ヘッダーと本体を 1 回の送信で出す、`serve_connection()` の順序、再接続 1000 ms、
  開始時の 3 秒待ち、切断中は破棄して数える。

## 実機確認の節目

| 節目 | 確認内容 |
|---|---|
| **CP1**（T03 後） | ユーザーが手動 BOOTSEL（この 1 回だけ）。by-id に `NuttX USB Console`。開いてすぐコマンドを送るとプロンプトが返る（Enter の空打ちなし）。開き直しても同じ。`bootsel` → `RPI-RP2` → `flash.sh` で再書き込み。`reboot` |
| **CP2**（T05〜T08 後） | `dmesg`（再起動後も残り、電源断で消える）。`kv set/sethex/get/delete`、再起動後の保持、`sys.` の拒否、128 B 値。`nina version` を 100 回。`i2c` で 0x6A の WHO_AM_I が 0x6C |
| **CP3**（T13） | 送信スタブで `imu start` → 10 秒 → `imu stop` → `imu status`。取得数が 6667 Hz×秒数に近い、`errors`・`fifo` が 0。`i2c_dma stats` の transfers 増、errors 0、`mode off` で切り替わる。**書き込み直後の最初の計測**。5 分連続を 1 回。`dmesg` に `<err>` なし。スタック使用量 |
| **CP4**（T18） | `wifi cred add` → 接続、`ifconfig` に 10.42.0.x、再起動後の自動接続、`wapi scan` 後の復帰、手動切断後は再接続しない。Telnet で USB と同じ出力、2 本目の挙動、5 秒間隔で 10 回の接続・切断。ImuStudio を無変更で接続。`iperf` 送受信 |
| **CP5**（T21） | `host/.venv/bin/imu-host check --port 18830` を動かし、`imu start`・`stop`・`status`。`missing_batches`、`unix_valid`、`unix_regressions`、`batch_unix` を `imu status` と照合。書き込み直後の最初の計測。送信中の Telnet 接続・切断。`--drop-at`・`--stall-at` で切断復帰と輻輳。`ntp status` を Zephyr 版の値（設計書 §8）と比較。`cpu_active` の比較。10 分連続を 1 回。スタック使用量 |
| **CP6**（T22〜T24 後） | `imu-host` の `write`・`record`・`calib` が NuttX 版で動く（USB と Telnet）。`imu cal`・`reload`・`zero`。uORB のツールで値が出て、計測中は拒否。`zephyr/script/host_tests.sh` と `nuttx/script/host_tests.sh`。可能なら Zephyr 版の UF2 に戻して `imu-host` の判定を 1 回確認 |

## リスクと、実機で判断すること

- usrsock の細部（`REQ_IN_PROGRESS` 後の最終応答、accept の応答形式、poll のイベント）は T14 の最初に
  `net/usrsock/usrsock_{connect,accept,poll,event}.c` を読んでから実装する。usrsock だけの構成での netdev 登録が
  RP2040 でビルドできるかは未確認。
- NuttX 13.1.0 は未読。12.13.0 系で確かめた事実が変わっている可能性があり、T01b で確かめる。
- tickless の自作（T03b）: NuttX の oneshot/alarm の枠組みへの載せ方と、必要な `.patch` の範囲は 13.1.0 の
  ソースを読んでから決める。動かなければ 100 µs の周期 tick に落とす（その場合は周期割り込みの分だけ
  `cpu_active` が Zephyr 版より高く出る。差分として記録し、ユーザーに報告する）。
- アイドル窓の計測は「`WFI` が割り込み禁止中でも起床する」Cortex-M0+ の動作に依存する。実機で値を確かめる。
- I2C 1 MHz の波形、DMA の 16 ビット書き込み、KV 消去中の割り込み禁止時間。
- チップ ID（セッション ID の前半）が Zephyr 版と一致するか。合わなければ flash の unique ID を生で読む。
- Telnet の 2 本目の拒否の見え方、同じ優先度のスレッドの時分割は Zephyr と違う可能性がある。
- 違いが残った点は、設計書に「確認した範囲」と分けて書く。原因が未確定のものは文書に入れない。

## 完了の確認

1. `nuttx/script/host_tests.sh` と `zephyr/script/host_tests.sh` が通る。
2. `nuttx/script/build.sh` で UF2 ができ、`flash.sh` で書き込める。
3. CP5 と CP6 の内容が通り、結果を測定条件・回数・未確認の範囲つきで報告する。
4. README（日英）、設計書（日英）、ドライバー説明（日英）、`host/README`（日英）が NuttX 版を説明している。
5. Zephyr 版のソースに差分がない（`git diff --stat -- zephyr/` が空）。
