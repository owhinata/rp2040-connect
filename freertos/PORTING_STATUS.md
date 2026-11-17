# 共通化と FreeRTOS 版の進捗

計画は `freertos/PORTING_PLAN.md`。このファイルと計画は未追跡のまま置く（commit しない）。状態が変わるたびに、
次の作業へ進む前にここを更新する。再開時は、この 2 ファイルと `git log`・`git status` を読み、実機の状態を
確認してから続ける。

## 現在の状態（2026-11-17）

- 計画は 2026-11-17 にユーザーが承認した（フェーズの順序、LSM6DSOX を C 層に残すこと、`common/` の構成、
  FreeRTOS の tick 100 µs、KV を Zephyr 版と同じ形式にする第 1 候補、新しいファイルの MIT 表記）。
- 計画・進捗・調査メモ（`PORTING_*.md`）は `freertos/` に置く（ユーザーの指示、2026-11-17）。
- P1（T10〜T13）は完了（2026-11-17）。Zephyr の像は最初の基準（0f14ce0）とバイト単位で同じまま。NuttX の像は
  T12b と T13 で変わった（基準は `snapshot-t13`）。P2 の T20 を進行中。CP-A は省略（ユーザーの決定）。

## ユーザーが決めたこと

- FreeRTOS 版は全機能同等（2026-11-17）。
- タスクごとに `main` へ commit、push しない（2026-11-17）。
- subagent は `model: opus`（2026-11-17）。
- コーディングスタイルは Zephyr に合わせる。README にコーディングスタイルの節は置かない（2026-11-16）。
- ツールチェーンは最新の GCC。必要なツールは `setup.sh` で入れる（2026-11-16）。
- T01b の結果への回答（2026-11-17）: tick は 32 ビット・10 kHz で `k_uptime_*` は TIMER から。速度測定は iperf3 互換を
  最小限に自作。優先度継承が 1 段、協調優先度の近似は受け入れる。KV 領域に別のデータがあれば自動では消さず、
  エラー表示と消去用のコマンド。
- CP-A（P1 の後の Zephyr 版の実機確認）は省く（2026-11-17）。像が作業前と同じため。
- **NuttX 版の KV も Zephyr の NVS の形式にそろえる**（2026-11-17 の指示。計画の T24）。
- `freertos/app/usb-console` の SPDX 表記は MIT（2026-11-16）。既存の Apache-2.0 の表記（385 か所）は指示が
  あるまで触らない。

## タスクの状態

| ID | 状態 | commit | メモ |
|---|---|---|---|
| T00 `freertos/` の環境を commit | 完了 | 0f14ce0 | `freertos/PORTING_*.md` と `CLAUDE.md` は含めていない |
| T01 事前調査 | 完了（2026-11-17） | – | 結果は `freertos/PORTING_NOTES_T01a.md`・`_T01b.md`。計画への反映は `PORTING_PLAN.md` の「T01 の結果による修正」。記録は下 |
| T10 A 層を `common/` へ | 完了（2026-11-17） | f4ec105 | 両方の像が基準と一致。記録は下 |
| T11 同一の C 層を `common/` へ | 完了（2026-11-17） | c2a07de | 両方の像が基準と一致。記録は下 |
| T12a 少差分の一本化（シェルの登録を除く） | 完了（2026-11-17） | acbd898 | 両方の像が基準と一致。記録は下 |
| T12b `imu_stream.c`・`time_sync.c` とシェルの登録 | 完了（2026-11-17。**NuttX の像が変わった。実機は CP-B で確認**） | 7f78570 | 記録は下 |
| T13 `lsm6dso.c`・`lsm6dso.h` の一本化 | 完了（2026-11-17。**NuttX の像が変わった。実機は CP-B で確認**） | 61a4cf2 | 記録は下 |
| CP-A | **省略**（ユーザーの決定、2026-11-17）。Zephyr の像が作業前の基準とバイト単位で同じため。Zephyr 版の実機確認は CP-Z で行う | – | |
| T20 互換層を共通部と NuttX 部に分ける | 完了（2026-11-17） | ce2bee5 | 両方の像が基準と一致。記録は下 |
| T21 NINA のコアを `common/` へ | 完了（2026-11-17） | eaebb16 | 両方の像が基準と一致。記録は下 |
| T22 I2C0 と DMA のコア | 完了（2026-11-17） | 82d6c4e | 両方の像が基準と一致。記録は下 |
| T23 シェル・ログ・境界のヘッダーを `common/` へ | 完了（2026-11-17） | 6b541ca | 両方の像が基準と一致。記録は下 |
| T24 KV を Zephyr の NVS の形式に | 完了（2026-11-17） | `daae9ec` | Zephyr 一致、NuttX は変化（基準は `snapshot-t24`、279316 B） |
| CP-B | 未着手 | – | |
| F01〜F15 | 未着手 | – | |
| Z01、Z02、CP-Z | 未着手 | – | |

## 機能差分（残っているもの）

NuttX 版が共通化で変わった点（どれも Zephyr 版の動作に近づく方向。**実機では未確認、CP-B で見る**）:

- T12b: `imu`・`ntp` コマンドが `subcmd` の表ではなく `zshell.c`（`sensor` と同じ経路）で動く。
  - ハンドラーに渡る `sh` が NULL ではなく端末のバックエンドになり、出力は `fflush(stdout)` の後に fd へ直接 `write()` する
    （行き先は同じ）。
  - 一致するワイルドカードの後にハンドラーのあるサブコマンドが続く行（例 `imu s* status`）は、旧版は `imu` のヘルプ、新版は
    `Error: requested multiple function executions`。30 語を越える行は `wrong parameter count` → `Too many arguments in the
    command.`。512 バイトを越える行は何も出さずに失敗する（未テスト）。ヘルプ、引数の数の誤り、存在しないサブコマンドの
    出力は同じ（`nuttx/tests/shell_register.c` が 49 通りを旧い表と比べる）。
  - コマンドのタスク（スタック 3072 B）の最大の使用量が約 0.4 KB 増えうる（`-fstack-usage` からの見積もり）。コマンドの
    たびにヒープを約 1.2 KB 確保して解放する。

- T13: LSM6DSOX のドライバーが `lsm6dso_os_*` の包みではなく互換層の `k_*` を直接呼ぶ。
  - `k_cycle_get_32()`（FIFO の読み出し時間の計測）は、TIMERAWL を 1 回読む形から、H → L → H を読んで上位が一致するまで
    読み直す形（`nano_mono_us()` の下位 32 ビット）になった。数えるものは同じ（1 MHz）。
  - 0 以下の sleep は何もしない → yield（Zephyr と同じ）。ドライバーが渡す値は正の定数なので、動作は変わらない。

CP-B で見ること（上に対応）: USB と Telnet での `imu`・`imu --help`・`imu start`/`stop`/`status`/`cal`・`ntp status` の出力、
`ps` のスタックの使用量（`imu`・`ntp` のタスク）、`ntp status` の時刻の値。

## ユーザーに確認待ち

T01b の結果から（2026-11-17。詳細は `freertos/PORTING_NOTES_T01b.md` の最後の節）:

- 周期 tick の負荷が大きすぎたときに自作の tickless へ進むか（F01 で実測してから相談）。
- `PICO_USE_STACK_GUARDS` で割り込み用スタックのあふれを検出するか（F01 で調べてから相談）。

## 実機の状態

- 2026-11-16 の夜の時点: FreeRTOS 版（`freertos/app/usb-console`、CLI と LED だけ）が入っている。Zephyr 版・NuttX 版の
  KV は消えたものとして扱う（その前に OpenMV 版を書いている）。
- ユーザーが picocom でコンソールを開いていることがある。開いている間は読み書きしない。

## タスクごとの記録

### T00（完了、0f14ce0）

`.gitignore`、ルートの README（日英）、`freertos/`（`PORTING_*.md` を除く）を commit した。

### T01 事前調査（完了、2026-11-17）

- subagent 2 つ（opus）。T01a は 121 回のツール呼び出し・約 35 分、T01b は 72 回・約 15 分。
- メインの確認: T01b の要点 5 つ（tick の幅 `portmacro.h:66-78`、トピックの作り方 `imu_raw_mqtt.c:375-379`、
  `hwinfo_rpi_pico.c` の `flash_get_unique_id`、`pico_flash/flash.c:177-182` の `NOT_PERMITTED`、NVS が `subsys/kvss/nvs`）を
  ソースで確かめた。T01a は、基準（`freertos/build/baseline/`）と `compare.py` があること、NuttX のワークスペースの状態
  （`git -C nuttx/nuttx status` がパッチ 8 本の `M` だけ、`boards/arm/rp2040/common/board` の symlink がある）を確かめ、
  ノートの 1〜4 節を読んだ。移動の実験と再現性の結果は subagent の報告のままで、メインは再実行していない。
- **事故**: T01a が NuttX のワークスペースを tar で写して `distclean` し、絶対パスの symlink をたどって元の
  ワークスペースの symlink とオブジェクトを消した。subagent が作り直してビルドし直した。以後の指示に注意を入れる。
- 状態の変化: `zephyr/build/usb-console` は今のソースのビルド。`nuttx/build/nuttx.uf2` は `-j 1`・日時固定のビルドに
  上書きされている。`freertos/build/t01b-*` は T01b の実験のビルドで、消してよい。
- ユーザーの回答（2026-11-17）: tick 32 ビット、速度測定は最小限の自作、近似 2 点の受け入れ、KV 領域の別データの
  扱い、NuttX 版の KV の統一（T24）。

### T10 A 層を `common/` へ（完了、f4ec105）

- 移したもの: `common/imu`（`imu_raw_*` 5 組、`imu_tick_clock`、`time_offset`、`imu_cal_codec.c`、`imu_cal.h`）、
  `common/drivers/lsm6dso`（`lsm6dso_fifo`・`_cal`・`_zero`・`_fifo_decoder`、`lsm6dso_fifo_packet.h`）、
  `common/drivers/i2c_dw`（コマンド語）、`common/tests`（テスト 12 本、`fakes` 5 本、`run.sh`）。
- **T11 に回したもの**: テスト `fifo_hw`・`stream_stop`（対象の `lsm6dso_fifo_hw.c`・`lsm6dso_stream.c` が T11 で移るため）。
  今は各版の `tests/` に残り、各 `host_tests.sh` が走らせる。同一のテストは 15 本ではなく 14 本だった（ノートの数え違い）。
- **T12 で直すもの**: `imu_stream.c` の `"../drivers/lsm6dso/lsm6dso_fifo.h"` は、`-I common/imu` の後ろにパスが付く形で
  見つかっている。`"lsm6dso_fifo.h"` に直す。
- メインの確認（subagent とは別に実行）: Zephyr 版を `west build -p always` して `zephyr.bin` が基準と一致、NuttX 版を
  `SOURCE_DATE_EPOCH=1763337600 ./script/build.sh --clean -j 1` して `nuttx.bin` が基準と一致、`zephyr/script/host_tests.sh`
  16 個と `nuttx/script/host_tests.sh` 38 個がすべて `ok`、`common/` にオブジェクトなし。ビルド定義の差分と、ルートの
  README・設計書の差分を読んだ。ほかの文書（`nuttx/`・`host/`・ドライバー説明）の差分は行数を見ただけ。
- 実機では未確認（像が同じなので CP-A・CP-B まで確認しない）。
- `git` は移動を `nuttx/... -> common/...` の rename と表示する（同一内容の削除が 2 つあるため）。

### T11 同一の C 層を `common/` へ（完了、c2a07de）

- 移したもの: `common/drivers/lsm6dso` に `lsm6dso_fifo_hw.[ch]`、`lsm6dso_stream.c`、`lsm6dso_fifo_irq.[ch]`、
  `lsm6dso_fifo_stats.h`、`lsm6dso_attributes.h`。`common/tests` に `fifo_hw.c`、`stream_stop.c`（共通のテストは 14 本）。
- ソースの変更は、両方の `sensor_shell.c` の `#include "lsm6dso_attributes.h"` の 1 行だけ。
- 計画に無かったこと: `common/tests/fakes/lsm6dso.h`（ガードだけの偽ヘッダー。`stream_stop` が `lsm6dso_stream.c` の
  `#include "lsm6dso.h"` を開けるようにするため）、`nuttx/apps/imu/Makefile` の使われなくなった `-I $(APPDIR)/external` の削除。
- `imu_stream.c` は変えていない。`"../drivers/lsm6dso/lsm6dso_fifo_stats.h"` は `-I common/imu` の後ろに付く形で見つかる
  （T12b で `"lsm6dso_fifo.h"`・`"lsm6dso_fifo_stats.h"` に直す）。
- メインの確認（別に実行）: Zephyr・NuttX の像が基準と一致、ホストテスト 16 個と 38 個が `ok`、`common/` に
  オブジェクトなし。ビルド定義・ソース・偽ヘッダーの差分を読んだ。文書の差分は読んでいない（subagent の報告のまま）。
- `zephyr/app/usb-console/tests/fakes/` に残っていた空のディレクトリー（未追跡）はメインが消した。

### T12a 少差分の一本化（完了、acbd898）

- 1 本にしたもの（`common/`）: `imu/imu_cal.c`、`imu_raw_mqtt.[ch]`、`wifi_profile.[ch]`、`kv_store.h`、`time_sync.h`、
  `drivers/nina/nina_net.c`、`drivers/lsm6dso/lsm6dso_compat.c`、`drivers/sensor_shell/sensor_shell.c`。
- Zephyr の文面に入れた変更（メインが commit 済みの Zephyr 版と `diff` して確認）: `lsm6dso_compat.c` の `<inttypes.h>` と
  `PRIu32`・`PRId32` 2 か所、`sensor_shell.c` の `"sensor_shell_os.h"`・`sensor_shell_os_get_arm()`・`_get_wait()`・`PRI*` 1 か所、
  `wifi_profile.c` の `extern const k_tid_t ...` → `#include "wifi_profile_os.h"`。ほかの 7 本は Zephyr 版のまま。
- NuttX 側: 前置きヘッダー `nuttx/drivers/compat/common_prelude.h`（`common/` のソースにだけ `-include`）、互換層に
  `<zephyr/logging/log.h>`・`<zephyr/net/net_ip.h>`。前置きにソケットのヘッダーは入れない（`lsm6dso_zero.c` の `static` 関数
  `accept` と衝突する）。
- **OS ごとに残った差**: `sensor_shell_os.h`（`sensor get` の待ち。Zephyr 版は何もしない）、`wifi_profile_os.h`
  （`wifi_reconnect_tid` の `const`。互換層の `K_THREAD_DEFINE()` が起動時に書くため。T20 で `k_tid_t` の作りを変えれば消せる）。
- NuttX だけのテスト（`imu_cal`、`imu_raw_mqtt`、`sensor_shell`）は `nuttx/tests` に残した（偽物が `common/` に無い）。
- 計画 10 項の `<zephyr/types.h>`・`<zephyr/sys/sys_io.h>` は T12b・T13 で足す。
- メインの確認（別に実行）: Zephyr・NuttX の像が基準と一致、ホストテスト 16 個と 38 個が `ok`。新しいヘッダー 7 本と
  CMake の差分を読んだ。新しいファイルの SPDX 表記はメインが MIT に直した（T11 の `common/tests/fakes/lsm6dso.h` も）。
  NuttX の Makefile 類と文書の差分は読んでいない。

### T12b `imu_stream.c`・`time_sync.c` とシェルの登録（完了、7f78570）

- `common/imu/imu_stream.c`: Zephyr の文面への変更は `#include "lsm6dso_fifo.h"`・`"lsm6dso_fifo_stats.h"` と `PRId32` 1 か所
  だけ。`common/imu/time_sync.c`: Zephyr 版と同一。OS ごとに残った差は無い。
- 互換層: `SHELL_CMD_REGISTER()` の入口は既定で `main`、`-DZ_SHELL_CMD_NAMED_MAIN` を付けたファイルでは
  `z_shell_main_<syntax>`（`time_sync.c` → `ntp_main.c` が呼ぶ）。`<zephyr/sys/sys_io.h>`（`sys_read32()`）、`devicetree.h` の
  `DT_REG_ADDR(DT_NODELABEL(timer))`、前置きヘッダーに `CONFIG_NINA_NTP_*` の対応。
- `time_sync_mono_us()`: Zephyr 版、旧 NuttX 版（`nano_mono_us()`）、新 NuttX 版とも TIMERAWH → TIMERAWL → TIMERAWH を
  読み、上位が一致するまで読み直す（subagent の比較。新しい関数の命令列は旧 `nano_mono_us` と同じとのこと）。
- 像: Zephyr は基準と一致。NuttX は 278956 → 278996 B。変わったオブジェクトは `imu_stream.o`・`time_sync.o`・`ntp_main.o`。
  **NuttX の新しい基準は `freertos/build/baseline/nuttx/snapshot-t12b/`**（T13 以降はこれと比べる）。
- メインの確認（別に実行）: Zephyr の像が基準と一致、NuttX の像が `snapshot-t12b` と一致（再現する）、`compare.py` で
  変わった関数が `imu_main`・`ntp_main`・`sync_thread`・`time_sync_mono_us`・`z_shell_fprintf_` と登録のデータだけである
  こと、ホストテスト 16 個と 39 個が `ok`。1 本にした 2 ファイルを commit 済みの Zephyr 版と `diff`、互換層の差分を読んだ。
  「ハンドラーの命令列が再配置まで一致」と「スタックの見積もり」は subagent の報告のままで、再実行していない。
  文書の差分は読んでいない。
- `compare.py` は `i2c_dw_rp2040_commands.c.txt: only in A` を出す（T10 で `common/` へ移ったファイルをスクリプトが
  探さないため。スクリプトの限界で、差ではない）。

### T13 `lsm6dso.c`・`lsm6dso.h` の一本化（完了、61a4cf2）

- `common/drivers/lsm6dso/lsm6dso.h`: Zephyr 版と同一。`lsm6dso.c`: Zephyr の文面への変更は `#if` → `#ifdef
  CONFIG_LSM6DSO_TRIGGER` の 1 行と、末尾（デバイスの生成、元の 1346〜1448 行）を `#include "lsm6dso_instance.h"` に置き換えた
  ことだけ（メインが commit 済みの Zephyr 版と `diff` して確認。Zephyr 版の断片は元の 103 行と同一）。
- OS ごとに残ったもの: `lsm6dso_instance.h`（Zephyr 版は `zephyr/app/usb-console/drivers/lsm6dso/`、NuttX 版は
  `nuttx/drivers/lsm6dso/`）。`lsm6dso_os.[ch]` は消した。
- 互換層: `k_busy_wait()`（`up_udelay()`）、`k_msleep()`・`k_usleep()`（`nxsched_msleep()`・`nxsched_usleep()`）、
  `k_cyc_to_ns_floor64()`（×1000）、`DT_HAS_COMPAT_ON_BUS_STATUS_OKAY()`、`<zephyr/types.h>`、`<zephyr/init.h>`、前置きヘッダーに
  `CONFIG_SENSOR_ASYNC_API`。
- 計画に無かったこと: `common/tests/fakes/lsm6dso.h`（T11 で足した偽ヘッダー）を消した（本物が同じディレクトリーに来て
  読まれなくなった）。Zephyr の CMake で、shub・trigger の差し替えを該当の Kconfig があるときだけにした。
- 像: Zephyr は基準と一致。NuttX は 278996 → 279044 B。**NuttX の新しい基準は `freertos/build/baseline/nuttx/snapshot-t13/`**。
- メインの確認（別に実行）: Zephyr の像が基準と一致、NuttX の像が `snapshot-t13` と一致、`compare.py` で変わった関数が
  `lsm6dso_*` の 17 個と `lsm6dso_init`・`stmemsc_mdelay`、増減が `k_busy_wait`・`k_msleep`・`k_usleep` と `lsm6dso_os_*` だけで
  あること、ホストテスト 16 個と 39 個が `ok`。互換層の追加と旧 `lsm6dso_os.c` を並べて読んだ。「対照実験（旧ソースを
  マクロで互換層に向けたものと一致）」と「テストのイベント 1835 行の一致」は subagent の報告のままで、再実行していない。
  文書の差分は読んでいない。
- **ユーザーが `common/drivers/nina/nina_net.c` を vim で開いていた**（スワップファイルがあった。commit の時点では消えていた）。
  `git add` はファイルを指定して行う。

### T20 互換層を共通部と NuttX 部に分ける（完了、ce2bee5）

- `common/compat/`: Zephyr の API のヘッダー 31 本（`zephyr/...`）と `zephyr_compat.h`、OS に依存しない `.c` 10 本
  （`zephyr_work`、`zephyr_rtio`、`zephyr_rtio_workq`、`zephyr_mem_blocks`、`sensor_default_rtio`、`zephyr_wifi_utils`、
  `zephyr_wifi_mgmt`、`zephyr_device`、`zephyr_gpio`、`zephyr_net_mgmt`）、Zephyr 本体の写し（`sensor_shell_stream.c`、
  `sensor_shell.h`、`wifi_credentials.c`、`wifi_credentials_backend_settings.c`、`wifi_credentials_internal.h`）。`mqtt_min.[ch]` は
  `common/imu`。
- `nuttx/drivers/compat/` に残ったもの: `zephyr_kernel.c`、`zephyr_mutex.c`、`zephyr_i2c.c`、`zephyr_net_if.c`、`zephyr_hwinfo.c`、
  `zephyr_sensor_clock.c`、`common_prelude.h`、ヘッダー `zephyr/net/socket.h`・`net_if.h`・`sys/clock.h`・`sys/__assert.h`・
  `drivers/i2c.h`（T22）・`shell/shell.h`（T23）、OS ごとのヘッダー `zephyr_os.h`・`zephyr_os_spinlock.h`・`zephyr_os_atomic.h`・
  `zephyr_rtio_workq_os.h`・`zephyr_net_mgmt_os.h`。
- **FreeRTOS 版（F02 ほか）が用意するもの**は `common/compat/zephyr/kernel.h` の先頭のコメントに一覧がある。そのほか
  `socket.h`・`net_if.h`・`sys/clock.h`・`__assert.h`・（当面）`i2c.h`・`shell.h` を OS ごとに書く。境界のヘッダー
  `priorities.h`・`nano_gpio.h`・`app_log.h`・`cpu_load.h`・`sensor_shell_os.h`、構成 `CONFIG_NANO_LSM6DSO_STREAM`・
  `CONFIG_NANO_WIFI_CREDENTIALS*` も要る。
- NuttX に残したアプリ側: `settings_nvs.c`（T24 で置き換わる）、`mqtt_min_sock.c`（F09/F13 で決める）、
  `wifi_credentials_shell.c`（getopt と `subcmd` に依存。T23）。
- テスト: `common/tests` に `zephyr_fakes/` と `zephyr_rtio`・`zephyr_work`・`mqtt_min`。共通のテストは 17 本
  （Zephyr 側 19 個、NuttX 側 39 個）。
- **やっていないこと**: `k_tid_t` の作り直し（`wifi_profile_os.h` の `const` の差が残る）。案: `K_THREAD_DEFINE` が
  `struct k_thread` と `const k_tid_t` を作り、OS のハンドルを構造体に持たせる。NuttX の像が変わる。FreeRTOS の互換層を
  書く F02 のときに、やるかどうかを決める。
- メインの確認（別に実行）: Zephyr の像が基準と一致、NuttX の像が `snapshot-t13` と一致、ホストテスト 19 個と 39 個が `ok`、
  `common/` にオブジェクトなし。`kernel.h` の先頭の一覧と、ディレクトリーの構成を見た。個々のヘッダーの分け方と文書の
  差分は読んでいない（像が一致しているので、展開結果は変わっていない）。共通の互換層が FreeRTOS でコンパイルできるかは
  未確認（F02 で分かる）。

### T21 NINA のコアを `common/` へ（完了、eaebb16）

- `common/drivers/nina/`: `nina_spi.c`、`nina_sock.[ch]`、`nina_dns.[ch]`、`nina_proto.h`（Zephyr 版の同一の写しと 1 本化）、
  境界の `nina_os.h`・`nano_nina.h`、`nina_sockopt.h`（T12a で移した `nina_net.c` もここ）。`common/tests/` に `nina_spi`・
  `nina_sock`・`nina_dns`（共通のテストは 20 本。Zephyr 側 22 個、NuttX 側 39 個）。Zephyr 版だけのテストは `nina_wifi_spi` に改名。
- NuttX に残したもの: `nina_os.c`、`nina_usrsock.[ch]`、`nina_poller.c`、`nina_netdev.c`、`wifi_profile_null.c`、`include/nina_net.h`、
  ボードの `nano_nina.c`。
- **FreeRTOS 版（F05・F09）が実装する境界**: `nina_os.h` の 7 関数（再帰・優先度継承のロック、DNS のロック、
  「切り上げ + 1 tick」の sleep、ms の時計）、`nano_nina.h` の 6 関数（SPI1 のポーリング転送、ピン 4 本。極性は
  `nuttx/drivers/README_ja.md` の NINA の節の表）。構成 `CONFIG_NANO_NINA_TCP_MAX_IO`（3584）・`_SEND_RETRY_MS`（1）は
  無ければ既定値。コアが POSIX のヘッダーから使うもの（F09 で用意）は subagent の報告のとおり: `ssize_t`、`struct sockaddr`・
  `sockaddr_in`・`iovec`・`addrinfo`、`AF_INET`・`SOCK_*`・`IPPROTO_*`・`MSG_DONTWAIT`・`TCP_NODELAY`・`POLL*`・`AI_*`・`EAI_*`、
  `inet_pton()`、`htons()`。
- Zephyr 側の変更: `src/main.c` の `#include "nina_proto.h"`、`CMakeLists.txt` の `-I` 1 か所（メインが差分を確認）。
- 残っている古い記述: 移したファイルの中のコメントのパス（`nuttx/drivers/nina/...`）とインクルードガード
  （`__NUTTX_DRIVERS_*`）は、内容を変えない方針のためそのまま。Z01 でコアに手を入れるときにまとめて直す。
- メインの確認（別に実行）: Zephyr の像が基準と一致、NuttX の像が `snapshot-t13` と一致、ホストテスト 22 個と 39 個が `ok`。
  NuttX の Makefile 類と文書の差分は読んでいない。
- **ユーザーが `zephyr/app/usb-console/CMakeLists.txt` を vim で開いている**（スワップファイルあり、未追跡。commit に
  入れていない）。subagent がこのファイルを直した後に開かれたとのこと。ユーザーが保存すると、以後の変更と食い違う
  おそれがあるので、次にこのファイルを触るタスク（T22）の前後で `git diff` を見る。

### T22 I2C0 と DMA のコア（完了、82d6c4e）

- `common/drivers/i2c_dw/`: コア `nano_i2c.c`（転送の状態機械 `i2c_dw_transfer_msgs()`、割り込みの本体 `i2c_dw_isr_handler()`、
  DMA 読み出し、`i2c_dma` のカウンター）、`nano_dma.c`（チャネル、`dma_rpi_pico_isr_handler()`）、`nano_dma.h`、`nano_i2c_dw.h`、
  `nano_i2c_dw_registers.h`、`nano_i2c_dma.h`、境界の一覧 `i2c_dw_os.h`（新規）。`common/tests/nano_i2c_dw.c`（共通のテスト 21 本。
  Zephyr 側 23 個、NuttX 側 39 個）。
- NuttX への接続（ボードの `src/`）: `i2c_dw_os_impl.h`（inline の包み）、`nano_i2c_instance.h`（`i2c_master_s`、割り込みの包み、
  ピン、初期化。`nano_i2c.c` の末尾で読む）、`nano_dma_instance.h`。
- **FreeRTOS 版（F06）が用意するもの**は `common/drivers/i2c_dw/i2c_dw_os.h` のコメントと `nuttx/drivers/README_ja.md` の
  I2C0 の節の表にある。要点: セマフォ（割り込みから give）、期限つきの take、割り込み禁止、ms の時計と sleep、
  I2C0・DMA_IRQ_0/1 の割り込みの登録、I2C0 と DMA のリセット、GPIO12・13 の設定、Zephyr の `struct i2c_msg` を読むマクロ 3 つ。
  subagent が偽の OS 実装で GCC 15.2 のコンパイルを確かめた（`-Wextra` では `MIN` に `-Wsign-compare` が 2 件。F06 では
  このファイルにだけ `-Wno-sign-compare`）。
- 動かさなかったもの: `nuttx/apps/i2c_dma`（T23・F06）、互換層の `zephyr_i2c.c`・`zephyr/drivers/i2c.h`（F07 で FreeRTOS 用を書く）、
  `nano_i2c.h`（NuttX の API）。
- 形の選び方: 境界を static inline とマクロにし、OS の断片を先頭と末尾で `#include` する。`always_inline` や一部の inline 化では
  命令列や関数の並びが変わったので、元と同じになる形を選んだ（subagent の報告）。
- 像: Zephyr も NuttX も基準と一致（NuttX は `snapshot-t13` のまま）。
- メインの確認（別に実行）: Zephyr の像が基準と一致、NuttX の像が `snapshot-t13` と一致、ホストテスト 23 個と 39 個が `ok`。
  `i2c_dw_os.h` の先頭を読んだ。コアと断片の中身、文書の差分は読んでいない（像が一致しているので、コンパイル結果は
  変わっていない）。
- `CONFIG_NANO_I2C0_DMA=n` の構成（出荷の構成ではない）では、転送の関数のスタックの割り当てと関数の並びが変わる
  （subagent の報告。動作は同じとのこと）。
- ユーザーのエディターのスワップファイルが `common/compat/` に出たり消えたりしている（`.wifi_credentials.c.swp`、
  `.zephyr_net_mgmt.c.swp`）。commit に入れていない。

### T23 シェル・ログ・境界のヘッダーを `common/` へ（完了、6b541ca）

- `common/shell/`: `subcmd.[ch]`（中身は変えていない。メインが `cmp` で確認）、`zshell.c`（共通の部分。バックエンドは
  `nuttx/apps/subcmd/zshell_os_impl.h`、境界の一覧は `zshell_os.h`）。`common/compat/zephyr/shell/shell.h`（`struct shell` は OS の
  `zephyr_os_shell.h`）。
- `common/log/`: `log_ring.[ch]`（`.c` は変えていない。メインが `cmp` で確認）、`log_ram.h`、`app_log.h`（OS の部分は
  `app_log_os.h`）。`log_ram.c` は NuttX に残した。
- `common/compat/`: `priorities.h`（OS の番号は `priorities_os.h`）、`cpu_load.h`、`nano_cpuload.h`、`nano_gpio.h`。
- NuttX に残したもの: `nano_timer.h`、`nano_timeconv.h`、`nina_net.h`。`kv_hex`・`kv_name` は T24。
- テスト: `common/tests/` に `subcmd`・`log_ring`（旧 `log_ram`）・`nano_cpuload`。共通のテストは 24 本（Zephyr 側 26 個、
  NuttX 側 39 個）。
- **FreeRTOS 版が実装するもの（F01・F03）**: シェルは `struct shell`、`zshell_os_backend_get()`、`zshell_os_cmd_enter()`・
  `_exit()`、`z_shell_backend_vfprintf()`、`fnmatch()`。ログは `app_log_printf()`・`app_log_vprintf()`・`app_log_ratelimit_pass()`・
  `log_ram_initialize()`・`log_ram_piece()`。ほかに `NANO_PRIO_FROM_ZEPHYR(p)`、`cpu_load_get()`、`nano_gpio.h` の 3 関数。
- **F03 への持ち越し（設計の前提）**: `zshell.c` と `subcmd.c` はヘルプとエラーを `printf`・`fprintf(stderr)` で出すので、
  コマンドを実行するタスクの stdout・stderr がそのセッションの端末である必要がある。USB と Telnet のシェルを別の
  タスクにして、タスクごとに行き先を選ぶ `_write` を用意する案。Pico SDK の `_write` を置き換えられるかは未確認。
  NuttX のコマンド（`kv`・`nina`・`dmesg`・`i2c_dma`・`wifi`）を共有する案: `-Dmain=nano_cmd_<名前>` と静的な表、
  `getopt` は呼び出しごとの状態（newlib の `__getopt_long_r`）、シェルのタスクのスタックは 3072 以上。
- メインの確認（別に実行）: Zephyr の像が基準と一致、NuttX の像が `snapshot-t13` と一致、ホストテスト 26 個と 39 個が `ok`、
  `subcmd.c`・`log_ring.c` が元と同じ。分けたファイル（`zshell.c`、`app_log.h`、`priorities.h`）の中身と文書の差分は
  読んでいない（像が一致しているので、展開結果は変わっていない）。

## T24 の記録（2026-11-17、commit daae9ec）

- Zephyr v4.5.0 の NVS・settings・crc8・hex を無改変で `common/compat/{subsys,lib,zephyr}` に写した
  （`common/tests/run.sh` の `zephyr_copies` が 15 本を上流と照合）。改変した写しは
  `wifi_credentials_backend_settings.c` だけ（`buf` の整列、`%zu`）。
- flash の境界は `common/compat/nano_flash_kv.h` の `nano_flash_kv_os_{ready,read,program,erase}`。
  FreeRTOS 版（F04）はこの 4 つを実装すればよい。flash デバイスと flash map は `zephyr_flash_kv.c`。
- `kv_store.c` は `common/imu` の 1 本。OS ごとの差は `kv_store_os.h`（Zephyr は空、ほかは `kv wipe`）。
- 別のデータ: 自動で消さず `-EILSEQ`（NuttX では 84）、`dmesg` に `flash_kv` の 2 行。消去は `kv wipe confirm`。
- NuttX の比較の基準は以後 `freertos/build/baseline/nuttx/snapshot-t24/`。
- メインが確かめたこと: 両方の再ビルド（Zephyr は基準と一致、NuttX は snapshot-t24 と一致）、ホストテスト
  （Zephyr 30、NuttX 41）、写しが上流と同一、`kv_store.c`・Zephyr 側の差分、`zephyr_flash_kv.c`・
  `kv_store_wipe.c` の全文、wipe 後の再初期化（`settings_store_init()` が読み込み元の一覧を作り直す）。
- メインが確かめていないこと: `nano_flash_kv.h` の判定関数とテストの中身、文書と `kv_transfer.py` の差分、
  スタックの見積もり、像の差の関数ごとの内訳（subagent の報告のまま）。
- CP-B で見ること（追加）: Zephyr 版が書いた KV を NuttX 版が読むこと（今の実機は FreeRTOS 版の前が OpenMV
  なので、まず領域の判定結果と `kv wipe confirm` → `kv_transfer.py restore`）、起動時の `fs_nvs` 3 行、
  `ps` のスタック（init、kv、wifi、imu、`wifi_reconnect`）、計測中の `kv set` で GC が走ったときの欠番、
  `/dev/config` が無いこと、`wifi cred list` の長さ違いのエラーが -2。

## CP-B の途中経過（2026-11-17）

- 実機（FreeRTOS 版が動いていた）に `bootsel` を送り、T24 の NuttX 版（daae9ec）を書き込んだ。
- 起動時の `dmesg`: `flash_kv: Key-value area 0xffe000-0xffffff holds the NuttX build's earlier format`、
  `Not used; "kv wipe confirm" erases it`、資格情報と settings の初期化が -84。`kv get imu.port` は
  `KV read failed: -84`。`/dev/config` は無い。`imu status` は `state: idle`。つまり KV 領域は OpenMV 版と
  FreeRTOS 版を書いたあとも以前の NuttX の形式のまま残っていた。
- `ps`（起動直後、KV なし）: init 752/2008、nsh 1760/3032、telnetd 848/2008。
- まだ: `kv wipe confirm`（ユーザーの承認待ち）、`kv_transfer.py restore`、Wi-Fi、計測、Telnet、Zephyr 版との読み合い。
- 2026-11-17（ユーザー承認）: `kv wipe confirm` → `Erased`、`fs_nvs` の 3 行（`alloc wra: 0, fe8`、`data wra: 0, 0`）。
  `kv_transfer.py restore --usb --no-wifi` で校正値 2 個と `imu.port` を書き戻し、読み返し OK、`imu cal reload` で反映。
  Wi-Fi の資格情報はユーザーの入力待ち（AP は起動済み）。
- 2026-11-17 CP-B の計測（NuttX 版 daae9ec、AP 経由、受信器 `imu-host check --port 18830`）:
  - 資格情報の登録（ユーザー）後、接続（10.42.0.97）。再起動後も `fs_nvs` 3 行（`alloc wra: 0, ea8`）、エラーなし、
    自動接続、`imu.port`・校正値が残る。
  - 書き込み後の最初の計測 71 秒: 欠番 0、`unix_regressions` 0、エラー・オーバーラン 0、`cpu_active` 70.0%。
    送信中に Telnet の接続・`imu status`・切断を 2 回（5 秒間隔）、`kv set` 20 回。
  - 計測中の `kv set` 260 回（95 秒）: GC の消去 1 回（書き込み 140〜160 回目）で FIFO オーバーラン 1 回、回復 1 回
    （-75）、受信側の欠番 0、エポック 0→1。`max_transfer_us` 47437。設計書（日英）に 1 文を足した（未 commit）。
  - 450 回の回は受信器が先に終わったので参考（オーバーラン 2 回、KV の失敗 0）。
  - スタック（計測後）: init 976/2008、nsh 1760/3032、imu_raw_mqtt 1272/1992、time_sync 1224/2000、telnetd 872/2008。
  - 見ていないもの: Wi-Fi スキャン、切断からの復帰、`imu`・`ntp` の端のケース、Zephyr 版との KV の読み合い（CP-Z）、長時間。
  - 追加（同日）: `imu`・`imu bogus`・`imu st*` はヘルプを出して終了状態 1、`ntp status` は synchronized、
    `kv` のヘルプに `wipe`、`kv wipe` は引数の数のエラー、`kv wipe no` は何も消さない。NuttX 版の `wifi` には `scan` が無い
    （`cred` だけ。以前から）。**CP-B は完了とする**（切断からの復帰と長時間は見ていない）。

## F01 の記録（2026-11-17）

- 実装: tick 10 kHz・32 ビット、時分割 20 ms（`configUSE_TIME_SLICING` 0 と tick フック `tick.c`、優先度 1〜20 だけ）、
  `priorities_os.h`（FreeRTOS = 20 − Zephyr、`Tmr Svc` 31）、`os_clock.h`（`nano_mono_us()`、`os_sleep_us()` は n+1 tick）、
  `idle.c`（`cpu_load_get()`、`ps` の最後に `cpu_active`）、`irq_stack.c`（`irqstack`）、`fault.[ch]` と
  `common/log/fault_record.[ch]`（RAM と watchdog scratch[0..3]、`fault` コマンドとバナー）、`sleep <ms>`。
- メインが確かめたこと: クリーンビルド警告 0、`format.sh --check`、`common/tests/run.sh` 29 本、`tick.c`・`idle.c`・`fault.c`・
  `os_clock.h` の全文。Zephyr・NuttX の像の一致は subagent の報告のまま（`common/` の変更はテストと新規ファイルだけ）。
- 実機（FreeRTOS 版を書き込み）: tick 231543 / 23.154258 s = 10000.0/s。アイドル 21 秒の `cpu_active` 2.0%。
  sleep の往復時間（USB 込み、9 回の中央値）: コマンドだけ 0.87 ms、`sleep 0` 1.00、`sleep 1` 1.99、`sleep 10` 11.10、
  `sleep 100` 101.16、`sleep 1000` 1001.21 ms（コマンドの往復を引くと N + 0.1〜0.3 ms）。`irqstack` 468/2048、
  `console` の空き 2264 B（3072 中）。`fault` は `No fatal error before this start`。
- 実機で未確認: 異常停止の記録（起こす手段が無い。`devmem` を足すかはユーザーに相談中）、時分割（同じ優先度で回る
  タスクがまだ無い。F08 以降）。
- 決めたこと・残ること: 再起動は watchdog（T01b の案の AIRCR ではない）。`PICO_USE_STACK_GUARDS` は使わない
  （MSP があふれるとロックアップして再起動しない、というのは subagent がソースから推測したもの）。フラッシュ消去中の
  異常停止は記録できないかもしれない（F04）。文書のテストの本数（`nuttx/README*`、`design/nuttx-porting*` の 39/24）が
  古い → F15 で直す。

## ユーザーの指示（2026-11-17、F01 のあと）

- `devmem` を入れる。書き込みも入れ、Zephyr 版と同じ書式（`devmem <address> [<width> [<value>]]`）。F02 が戻ってからメインが実装。
- `sleep <ms>` は残す。
- `selftest`（F02 の自己診断）は、確認が終わったら外す。メインの解釈: 既定のビルドには入れない（オプションで入れられる形で
  ソースは残す）。この形でユーザーが承認（同日）。

## F02 の記録（2026-11-17）

- 実装: `freertos/drivers/compat/`（`zephyr_os*.h`、`zephyr_kernel.c`、`zephyr_mutex.c`、`zephyr_os_sched.h`、
  `zephyr_selftest.[ch]`、`rp2040/`）、`freertos/drivers/log/app_log_min.c`（F03 で置き換える）、`freertos/tests/compat/`
  （POSIX ポート）、`freertos/script/host_tests.sh`、`build.sh` の `-DNAME=VALUE`。共有ソースが使う API だけを作った
  （`k_thread_create`・`k_timer`・`k_msgq`・`k_fifo`・`k_condvar`・`k_event`・`k_poll_signal`・`sys_timepoint_*` は無い。
  F09 以降で要るものが出たら足す）。
- `k_tid_t`: 作り直さない（`TaskHandle_t` の変数、NuttX 版と同じ）。F10 で FreeRTOS 用の `wifi_profile_os.h` が要る。
- **実機で分かった違いと対応**: FreeRTOS は横取りのたびに同じ優先度の次のタスクを選ぶ（Zephyr は横取りされたスレッドが
  続きを走る）。USB の割り込み → `Tmr Svc`（31）の一瞬の横取りで実際に交代した。`zephyr_os_sched.h` のトレースマクロ
  （`traceTASK_SWITCHED_OUT/IN`、カーネル V11.3.3 の内部の名前を使う。**カーネルの版を上げるときは見直す**）で Zephyr の
  規則にした。残る近似は README と同ヘッダーのコメント。
- メインが追加: `devmem <addr> [<bits> [<val>]]`（Zephyr と同じ書式と出力、読み書き）。
- 実機: 修正前は sleep（1 回目）と timeslice（3 回中 2 回）が失敗。修正後は `selftest` 4 回とも `0 of 22 tests failed`
  （timeslice 200×6、timeslice_preempt `1 turns, 35 preemptions`、ISR からの起床 26〜29 µs）。`devmem 30000000` →
  HardFault → 自動再起動 → バナーと `fault`（task console、pc = `cmd_devmem` の `ldr r1,[r4]`）→ `reboot` 後は記録なし。
  既定のビルド（selftest なし）に書き直し済み。アイドルの `cpu_active` は 3.0%（F01 は 2.0%。tick フックに互換層の
  タイムアウト処理が入った分とみられるが、内訳は測っていない）。
- メインが確かめたこと: ビルド（両方、警告 0）、整形、`host_tests.sh`、`zephyr_mutex.c`・セマフォと poll の部分・
  `zephyr_os_sched.h` の全文、実機の上記。読んでいない: `zephyr_selftest.c`、work とスレッドの起動の部分、README の互換層の節。
- 残り: F07 で `sysworkq`（1024 B）のスタックを実測。gpio0 デバイスの初期化は F07。`selftest` は既定 OFF でソースを残す
  （ユーザー承認済み）。

## 文書の分担（2026-11-17、ユーザー承認）

- `freertos/README`（日英）はメインだけが編集する。使い方だけを書く（必要なもの、環境構築、ビルド、書き込み、コマンドの表、
  短い注意）。subagent には触らせず、README に入れる内容は報告に書かせる。仕組みの説明はソースのコメントに。
- `PORTING_PLAN.md`・`PORTING_STATUS.md` はメインが書く（subagent の報告は要約して STATUS に）。subagent 用のノートは作らない。
- 設計書・既存のドライバー説明（日英）は、Zephyr 版・NuttX 版の動作や構成が変わったときだけ subagent が最小限に直し、
  メインがレビューする。FreeRTOS 版の内部の説明は F15 でまとめて書く（材料はソースのコメントと STATUS）。
- `freertos/PORTING_*.md` は Git 管理にする（以前の「未追跡のまま」を変更）。タスクの commit に STATUS の更新を含める。
  `CLAUDE.md` は未追跡のまま。
- 残り: `freertos/README`（360 行）を使い方だけに整理する。F03 が戻ってからメインがやる。

## あとで見直すこと: 時分割のフック（2026-11-17、ユーザーの意向）

- `zephyr_os_sched.h` のトレースマクロ（横取りされたタスクが続きを走る、切り替えのたびにスライスを数え直す）は、
  FreeRTOS 標準の動きで問題がなければ戻すことを検討する。カーネルの内部の名前に依存するのが弱点。
- 判断の材料: 全機能が動いた時点（CP-F4 か CP-F5）で、フックあり・なしの両方で実機を測って比べる（欠番、FIFO のオーバーラン、
  `cpu_active`、時刻同期の残差、Telnet・シェルの応答）。そのために、フックをビルドのオプションで切れるようにする
  （F03 のあとでメインが入れる。既定はあり）。なしのときは自己診断の `timeslice`・`timeslice_preempt` の期待が変わる。

## F03 の記録（2026-11-17）

- 実装: コンソールを共有の `common/shell/zshell.c`（Zephyr のシェル API）に載せた。`freertos/drivers/shell/`
  （`nano_console`: タスクごとの出力先、`nano_stdio`: SDK の stdio ドライバーの差し替え、`nano_shell`、`cli`（移動）、
  `shell_argv`（Zephyr の写し）、`getopt_state`）、`drivers/libc/stdio_wrap.c`（newlib の FILE 関数の `--wrap`）、
  `drivers/log/log_ram.c`（共有の `log_ring`、8 KB、再起動をまたいで残る）、`dmesg`。Zephyr の fnmatch・getopt を
  無改変で `common/compat/` に写した（`zephyr_copies` は 22 本）。`shell.h` に `Z_SHELL_CMD_ROOT_GLOBAL`。
- 計画との違い: 出力先の切り替えは「`_write` の差し替え」ではなく、SDK の stdio ドライバーの差し替えと `--wrap`
  （`printf` は `_write` を通らないため）。`PICO_STDOUT_MUTEX=0`。2 本目（F11 の Telnet）は `nano_console_init()` と
  `nano_console_bind()` で足す。
- 変わった動作（Zephyr に合わせた）: `help` の書式、`x: command not found`、引数の数の誤りの表示、`error N` を出さない、
  履歴（上下の矢印、512 B）、1 行 511 文字・30 語、引用符と `\`、`free` に `malloc:` の行。
- メインが追加: ビルドのオプション `-DNANO_SCHED_ZEPHYR=OFF`（時分割のフックを切る。既定 ON。`selftest` の
  `timeslice`・`timeslice_preempt` は skipped になる）。README（日英）を使い方だけに書き直した（360 → 135 行）。
- メインが確かめたこと: Zephyr の像が基準と一致、NuttX の像が snapshot-t24 と一致、FreeRTOS のビルド（既定、selftest、
  selftest+フックなし）が警告 0、整形、`host_tests.sh`。ソースは読んでいない（報告と実機の動作で判断）。
- 実機: `flash.sh` の `bootsel`（F03 の像からも）、`help`（13 行）、引数の誤り、`command not found`、`dmesg`、
  閉じていない引用符、履歴（上矢印）と左右の矢印が行に入らないこと、`devmem 30000000` → 再起動 → バナー・`fault`・
  `dmesg` の `<err> fault:` の 6 行、`reboot` 後も RAM ログが残ること、端末を閉じたまま `ps` を 5 回送っても止まらないこと。
  `ps`: `console` の空き 1788〜1900 B（3072 中）、アイドルの `cpu_active` 2.0〜2.1%（F02 の 3.0% から下がった。機能は
  同じなので、配置で動く範囲とみられる）。フックなしのビルドで `selftest` が `0 of 22 tests failed`（2 項目 skipped）。
- 相談中: コマンド名を Zephyr にそろえるか（案: `kernel version|uptime|sleep|reboot|ps|thread stacks` を足し、今の
  短い名前は別名で残す。`kv_transfer.py` が送る `kernel version` も通る）。
- 残り: 文書の本数（共有のテスト 33、NuttX 側 44）は F15 で。異常停止の行の時刻は次の起動の時刻、モジュール名は
  `fault`（Zephyr 版は `os`）。USB の送信バッファー 64 B は CP-F2 で計測中に確かめる。

## F04 の記録（2026-11-17）

- 実装: `freertos/drivers/kv/`（`nano_flash_kv.c`: `flash_safe_execute()` + `PICO_FLASH_ASSUME_CORE1_SAFE=1`、範囲の検査 3 段、
  読み出しはキャッシュなしの別名から。`ld/`: FLASH の長さ 0xffe000、`__kv_start`/`__kv_end`、ASSERT 3 つ。`kv.cmake`）、
  `main` タスク（優先度 20、2048 B。`kv_store_init()` のあと自分を消す）、`kv` コマンド、ホストテスト `freertos/tests/kv`。
  `common/`・`nuttx/`・`zephyr/` は変えていない。
- 別のデータのときのエラーは newlib では `-138`（EILSEQ。NuttX 版は -84）。
- メインが確かめたこと: `nano_flash_kv.c` とリンカーの差し替えの全文、ビルド（既定・selftest、警告 0）、整形、
  `host_tests.sh`、UF2 の全ブロックが 0x10000000〜0x10016000（0x10ffe000 より下）。
- 実機: 起動時に `fs_nvs` の 3 行、エラーなし。**NuttX 版が書いた KV がそのまま読めた**（`imu.port` = 18830、校正値 2 個が
  `kv-backup.json` と一致）。`kv set`/`get` → `reboot` 後も残る → `delete` → `-2`。`kv wipe`（引数なし）と `kv wipe no` は
  消さない。100 文字の値を 90 回書き込み（GC 3 回）: 全部 `Saved`、最後の値と `imu.port` が読め、`reboot` 後も同じ。
  tick の数え落とし: GC のあった書き込みで 457・497・458 tick（46〜50 ms。その 1 回の `kv set` 全体の分で、消去だけでは
  ない）、90 回の合計 2256 tick。`console` のスタックの空き 1580 B（3072 中）。`kv wipe confirm` は実行していない。
- 未確認: Wi-Fi の資格情報が GC のあとも読めること（F10 か、NuttX 版・Zephyr 版に戻したとき）、`main` タスクのスタックの実測
  （終わると消えるので見えない。見積もり約 1.0 KB / 2048）。
- 残る点: 消去のたびに tick が約 40〜50 ms 遅れる（`vTaskDelay`・遅延 work・時分割が遅れる。`k_uptime` は TIMER なので進む）。
  補正するかは未決（`xTaskCatchUpTicks()` は互換層のタイムアウトを補正しない）。XIP 停止中の例外は記録できず、ロックアップ
  して手動の RESET が要る（subagent の推論、未確認）。計測中の KV 書き込みの扱いは CP-F2 で NuttX 版と同じか確かめる。

## F05 の記録と CP-F1（2026-11-17）

- 実装: `freertos/drivers/nina/`（`nano_nina.c`: SPI1・GPIO の初期化を Zephyr のドライバーと同じ順と値で、`nina_os.c`:
  `nina_os.h` の境界）、共有の `nina_spi.c` をリンク、`common/drivers/nina/nina_shell.c`（Zephyr 版 `main.c` の
  `nina version` の写し。Zephyr 版を載せ替えると像が変わるので Z01 まで写しが 2 か所。SPDX はメインが MIT に直した）、
  ホストテスト `freertos/tests/nina`。転送はポーリング（3 版とも同じ）。GPIO の割り込みは F05 では不要（F07 で
  `gpio_add_raw_irq_handler_masked()` の案）。
- メインが確かめたこと: ビルド（既定・selftest、警告 0）、整形、`host_tests.sh`、`common/tests/run.sh` 31 本。Zephyr・NuttX の
  像の一致は subagent の報告のまま（`common/` の変更は、どちらもビルドしない新しいファイル 1 つだけ）。ソースは読んでいない。
- 実機: `nina version` → `NINA firmware: 3.0.2`。1 回目 763 ms（モジュールのリセット）、2〜12 回目 2.1〜2.5 ms（USB の往復込み）。
  `nina`・`nina bogus` はヘルプ。SPI1 のレジスター: CR0 0x707、CPSR 0x2、CR1 0x2、DMACR 0x0（7.8125 MHz、Zephyr と同じ）。
  `console` のスタックの空き 1680 B、アイドルの `cpu_active` 2.0%。`dmesg` に NINA のエラーなし。
- **CP-F1 完了**（シェル、RAM ログ、KV、NINA との通信）。未確認: NINA が応答しないときの -116（起こす手段が無い）。
- 次: F06（I2C0 コアの接着と `i2c_dma`）、F07（LSM6DSOX）、F08（取得サービスと `imu`）→ CP-F2。

## F06 の記録（2026-11-17）

- 実装: `freertos/drivers/i2c_dw/`（境界 `i2c_dw_os_impl.h`、デバイス `i2c0` と Zephyr の I2C API `nano_i2c_instance.h`、DMA の
  部分、`nano_i2c0_initialize()`）、`freertos/drivers/compat/zephyr/drivers/i2c.h`、`common/drivers/i2c_dw/i2c_dma_shell.c`
  （Zephyr 版 `i2c_dw.c` の `i2c_dma` コマンドの写し、Apache-2.0。FreeRTOS 版だけがビルドする）、`selftest` の I2C の 6 項目、
  ホストテスト `freertos/tests/i2c_dw`。`common/`・`nuttx/`・`zephyr/` の既存ファイルは変えていない。
- 割り込みの優先度は SDK の既定 0x80（NuttX 版と同じ。Zephyr 版は 0xC0）。「デバイスの割り込みは全部同じ高さ」は保つ。
  I2C の RTIO は不要（Zephyr 版も無効で、FIFO は work から `i2c_burst_read_dt` で読む）。
- メインが確かめたこと: ビルド（既定・selftest、警告 0）、整形、`host_tests.sh`。ソースは読んでいない。Zephyr・NuttX の像の
  一致は subagent の報告のまま。
- 実機（selftest のビルド）: 初期化直後のレジスター（IC_COMP_TYPE 0x44570140、SPKLEN 1、SDA_HOLD 1、pad 0x52、CTRL 0x3、
  NVIC の bit 11・12・23、優先度 0x80）が期待どおり。`i2c_dma` の `mode`・`stats`・誤った引数。`selftest` 3 回とも
  `0 of 22` と `i2c: 0 of 6 tests failed`（WHO_AM_I、標準と fast plus、拒否、NACK、DMA 読み出し）。`i2c_dma stats` は
  1 回ごとに transfers +1、bytes +35。転送後のレジスター（IC_CON 0x165、SS 0x1f4/0x271、FS 0x1f/0x1f、TAR 0x6a、DMA の
  CTRL 0x100015/0x108821）も期待どおり。`dmesg` に NACK の項目の `<err> i2c_dw:` 3 行（`NACK`、`ABR and STOP`、
  `IO Fail`。報告の予想は 2 行）。`irqstack` 564/2048。既定のビルドに書き直し、`selftest` が無いこと、`nina version`、
  `kv get` を確認。
- 次: F07（LSM6DSOX。`zephyr_device.c` のリンク、stmemsc、GPIO の割り込みは `gpio_add_raw_irq_handler_masked()` の案、
  gpio0 デバイス、`sysworkq` のスタックの実測）。

## ユーザーの決定と相談（2026-11-17、F06 のあと）

- コマンド名は短いまま（`kernel ...` は足さない）。`script/kv_transfer.py` のファームウェアの判定は F15 で FreeRTOS 版の
  `version` に対応させる。
- 消去中の tick の遅れ: tickless では解決しない（下の理由）と回答。対応の案は「フラッシュ操作のあとで TIMER から落ちた tick を
  数えて `xTaskCatchUpTicks()` で補い、互換層のタイムアウトを tick の数から判定する形に直す」。入れるかは CP-F2 の実測を見て決める。
  - FreeRTOS の tickless idle は、アイドルの間だけ tick を止めてあとで補う機能で、割り込み禁止中に落ちた tick は補わない。
    RP2040 ポートは tickless 未対応（T01b）。Zephyr・NuttX の「tickless」は時刻をハードウェアのカウンターから作る方式で別物。

## F07 の記録（2026-11-17）

- 実装: 共有の LSM6DSO ドライバー 9 本（ストリーム込み）を FreeRTOS 版にリンク。`freertos/drivers/lsm6dso/`
  （`lsm6dso_instance.h`、`nano_lsm6dso.[ch]`、`lsm6dso.cmake`、selftest の 3 項目）、`drivers/compat/rp2040/nano_gpio.c`
  （`gpio_add_raw_irq_handler_masked()`、割り込みは GPIO24 だけ。ほかは -ENOTSUP）、`zephyr_sensor_clock.c`、gpio0 の初期化、
  ホストテスト `freertos/tests/gpio`。`setup.sh` が ST の `lsm6dso_reg.[ch]` を `freertos/stmemsc/` に取得（Git 管理外）。
- **計画外の共通化**: NuttX 版の `stmemsc.h`・`stmemsc_i2c.c`・`stmemsc_mdelay.c` を `common/drivers/stmemsc/` へ移動
  （`DEBUGASSERT` → `__ASSERT_NO_MSG`、コメント）。メインが NuttX を再ビルドし、像が snapshot-t24 と一致、ホストテスト全通過。
- 起動の順: `main()` で 互換層（gpio0 含む）→ NINA → DMA → I2C0 → flash_kv の準備。タスク `main`（優先度 20）の先頭で
  LSM6DSOX の初期化 → `USB console started` → `kv_store_init()`。
- メインが確かめたこと: NuttX の像と NuttX 側のホストテスト、FreeRTOS のビルド（既定・selftest、警告 0）、整形、
  `host_tests.sh`、移動したファイルの差分。Zephyr の像の一致は subagent の報告のまま（Zephyr がビルドするファイルは変えていない）。
- 実機（selftest のビルド）: 起動時に `LSM6DSO: Initialize device lsm6dsox@6a`、`chip id 0x6c`、エラーなし。`selftest` 3 回とも
  `0 of 22`、`i2c: 0 of 6`、`lsm6dso: 0 of 3`。fetch: 加速度 (0.139, -1.412, 9.779) m/s²、ジャイロ 0.01 rad/s 以下、
  温度 31.8 °C。stream（6667 Hz、500 ms）: 54 バッチ、7136 語、accel 3357・gyro 3359、最大転送 5984 µs。`i2c_dma stats` は
  1 回ごとに transfers +55、約 +50 KB、errors・fallbacks 0。スタックの空き: `imu_fifo_worker` 1248/2048、`sysworkq` 816/1024、
  `console` 1616/3072。`irqstack` 564/2048。既定のビルドに書き直し済み（アイドルの `cpu_active` 2.0%）。
- 予想と違った点（未調査）: INT1 のエッジが 137 回（予想は約 55。バッチは 54）。終了コードは -140（newlib の ECANCELED。
  報告の -110 は誤記とみられる）。F08 で NuttX 版・Zephyr 版の INT1 の回数と比べる。
- 残り: `sensor` コマンドは F14（`sensor_shell_os.h`、処理スレッドの出力先）。`hwinfo` は F08。

## あとで判断すること: tick を 1 ms に戻すか（2026-11-17、ユーザーの意向）

- ユーザーの見立て: FreeRTOS に Zephyr・NuttX のような tickless が無いなら、1 tick = 1 ms にして、100 µs 単位の sleep は
  タイマーで作るほうがよさそう。**全体を組み終えたあと（CP-F4 か CP-F5 のころ）に判断する**。それまでは 100 µs のまま進める。
- 判断の材料として集めておくもの: アイドル時と計測中の `cpu_active`（今はアイドル約 2%）、計測中の tick の遅れ、フラッシュの
  消去での遅れ（F04: 1 回 約 46〜50 ms 分）、100 µs 単位の待ちを使う箇所の一覧（NINA の READY のポーリング 100 µs、
  ほか共有コードの `K_USEC`・`k_usleep`・`k_busy_wait`）と、その待ちが通信の速度に効く度合い。
- 1 ms にする場合に要ること: 互換層の時間の単位（`K_TICKS`・`K_MSEC` の換算、`Z_USEC_PER_TICK`）を 100 µs のまま保ち、
  1 ms 未満の待ちを TIMER のアラームで起こす仕組み、時分割 20 ms、`selftest` の期待値。時分割のフックの比較
  （`-DNANO_SCHED_ZEPHYR=OFF`）と同じ時期にまとめて測る。

## F08 の記録と CP-F2（2026-11-17）

- 実装: 共有の取得サービス 8 本を FreeRTOS 版にリンク（`freertos/drivers/imu/imu.cmake`）。送信と時刻同期は NuttX の T12 の
  スタブの写し（`imu_raw_mqtt_null.c`、`time_sync_null.c`、Apache-2.0）。`zephyr_hwinfo.c`、IPv4 の `inet_pton`/`inet_ntop` と
  ソケットのヘッダーの最小限（`drivers/libc/`、`AF_INET` = 2。F09 で確定し `compat.cmake` へ移す）、`imu` コマンド
  （`cal` は本物の `imu_cal.c`。F14 に残るのは `sensor` と `imu cal` の実機確認）、ホストテスト `freertos/tests/imu`。
  `common/`・`nuttx/`・`zephyr/` は変えていない。
- F12・F13 で外す手順は subagent の報告のとおり（`imu.cmake` のスタブを本物に置き換え、スレッドの起動を足す）。
- メインが確かめたこと: ビルド（既定・selftest、警告 0）、整形、`host_tests.sh`。ソースは読んでいない。
- **CP-F2（実機、既定のビルド、送信なし）**:
  - 書き込み直後の最初の計測 52.1 秒: accel 349248・gyro 349250（約 6703 Hz）、batches 1820、エラー・オーバーラン 0、
    `max_transfer_us` 6420、`cpu_active` 27.5%（NuttX の T12 の送信なしは 38.4〜39.2%）、`us_per_1e6_ticks` 24858301、
    `tick_read_failures` 0。`dmesg` にスタブの警告 1 行。
  - 計測中の `ps`: `imu_fifo_worker` 10.2%、`imu_stream_tid` 6.2%、`sysworkq` 1.1%。スタックの空き: `imu_stream_tid` 1104/2048、
    `imu_fifo_worker` 1256/2048、`sysworkq` 816/1024、`console` 1528/3072。`irqstack` 468/2048。`i2c_dma` errors・fallbacks 0。
  - **tick の遅れ**: 計測中 55.1 秒で 3 tick。`imu start`（`imu.session` の保存）で 6 tick。アイドル 10 秒で 0。
  - 開始と停止を 10 回（各 7 秒）: 毎回 `stopped`、連番 +1、約 26800 サンプル（4.0 秒分）、エラー 0。重ねての `imu start` は
    `Already running`、止まっているときの `imu stop` は `Not running`。
  - `imu start` から取得の開始まで約 3 秒（共有コードの `CONNECT_WAIT_MS` 3000。スタブは接続しないので毎回待つ）。
  - 計測中の `kv set`（100 文字 × 100 回、消去 3 回）: 消去のたびに tick が 466・470・504 落ち、FIFO のオーバーランと回復が
    1 回ずつ（`last_recovery_error` -139 = newlib の EOVERFLOW）、エポック +1。`errors.stream` 0、`max_transfer_us` 42758。
    NuttX 版の CP-B と同じ振る舞い。計測中に `dmesg`・`ps` の長い出力を 3 回ずつ出してもエラーなし。
  - 未確認: 計測中に端末を閉じる、長時間、受信側での欠番（Wi-Fi がまだ無い）。
- INT1 の 137 回（F07）の説明（subagent、ソースからの推定で実測なし）: しきい値の信号はラッチせず、FIFO を読んで水位が
  下がる途中に新しい語が届くと、しきい値を越え直して余分な立ち上がりが出る。余分な実行は FIFO_STATUS の 2 バイトを読むだけ。
  Zephyr 版・NuttX 版でも同じになるはず（数える手段が無く未実測）。
- 未調査の観察（subagent）: 924 バイトの読み出しが最大 5984 µs なら実効は約 1.4 Mbit/s で、1 MHz の SCL の計算と合わない。
  3 版共通のコア。オシロで SCL を見る価値がある。
- tick の補正の案は subagent の報告のとおり（フラッシュ操作のあとで `xTaskCatchUpTicks()`、互換層のタイムアウトを tick の
  数で判定）。入れるか・tick を 1 ms にするかは、全体を組み終えてから判断（ユーザーの意向）。
- 次: F09（ソケットの接着、名前解決）、F10（Wi-Fi）、F11（Telnet、速度測定）→ CP-F3。
