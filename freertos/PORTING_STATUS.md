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
| CP-B | 完了（2026-11-17） | – | NuttX 版の実機確認。記録は下 |
| F01〜F15 | 完了（2026-11-17） | `9cc3797` まで | FreeRTOS 版の全機能。CP-F1〜F5 で実機確認。記録は下 |
| Z01、Z02、CP-Z | 完了（2026-11-18） | `e838842` まで | Z01 は NINA を共通コアへ。Z02 は共通コアへ載せ替えず、上流のドライバー + ローカルのパッチ（ユーザーの決定）。実機確認は各節 |

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

## F09 の記録（2026-11-17）

- 実装: `freertos/drivers/nina/nina_socket.c`（`zsock_*` を fd の表 10 本を介して共有の `nina_sock.c` に直接つなぐ。Zephyr の
  オフロードに近い形: 呼んだスレッドで待つ、イベントスレッドなし）、名前解決（`nina_dns.c`）、newlib に無いヘッダー
  （`drivers/libc/include/`: `sys/socket.h`・`netinet/in.h`・`netinet/tcp.h`・`poll.h`・`netdb.h`・`sys/uio.h`。型と定数だけ）、
  ホストテスト `freertos/tests/socket`。インクルードパスと `inet.c` は `compat.cmake` へ移動。まだ呼ぶコードが無いので、
  既定の像には入らない（`--gc-sections`）。実機では未確認（F10 以降）。
- 定数は Linux の値（`AF_INET` 2 以外は Zephyr と同じ。`EAI_*` は Zephyr の負の値）。
- Zephyr 版との違い（subagent の報告）: ロックを 1 本にまとめた、connect の期限の比べ方で最悪 20 ms 長い、閉じた fd は EBADF。
- 計画との違い: `getsockname` は作らない（Zephyr 版も NuttX 版も EOPNOTSUPP）。`clock_gettime` は F12 で `sys_clock_gettime` として。
- メインが確かめたこと: ビルド（既定・selftest、警告 0）、整形、`host_tests.sh`。ソースは読んでいない。
- **100 µs 単位の待ちの一覧（tick を 1 ms にするかの判断材料）**: 1 ms 未満は NINA の READY のポーリング 100 µs
  （`nina_spi.c:47,62`。今は 2 tick = 200 µs。1 ms の tick では 2 ms になり、全ソケット命令の待ちに効く）だけ。1 ms の待ちは
  送信のやり直し（`NINA_SEND_RETRY_MS` 1）と NTP の応答のポーリング（`time_sync.c:188`。時刻の精度に出る）。
- F12 への注意: `time_sync.c` の `TIMER_BASE` が Pico SDK の定義と重なる。F13 への注意: `mqtt_min.c` は `-Wno-sign-compare`、
  `mqtt_min_sock.c` は NuttX 版の写しで足りる。
- `cpu_active` の比較（ユーザーの決定、同日）: Zephyr 版の 22.5% は「FIFO の取得だけ」の値で、FreeRTOS 版の 27.5%（取得サービス
  込み）とは範囲が違う。3 版の比較は**送信が入ってから**（CP-F4）同じ条件で測る。それまで追加の計測は入れない。

## あとで判断すること: NINA の READY の割り込み化（2026-11-17、ユーザーの決定）

- **全体を組み終えたあとで判断する**（tick を 1 ms にするか、時分割のフックを外すか、と同じ時期）。それまでは変えない。
- 案: `nina_spi.c` の `wait_ready()`（READY = GPIO10 を 100 µs ごとに読む。実際は 2 tick = 200 µs ごと）を、GPIO のエッジ
  割り込みで起こす形にする。RP2040 側だけの変更で、ファームウェアの版に依らない（READY は `SPIS.cpp` の `transfer()` が
  上げ下げするハードウェアの信号）。3 版共通のコアの動作が変わるので、入れるなら 3 版に入れて実機で確かめる。
  FreeRTOS 版の GPIO の割り込みは今 GPIO24 だけ（`NANO_GPIO_IRQ_PINS`）なので、GPIO10 を足す。
- 効果の見込み: 1 ms 未満の待ちがこの 1 か所だけなので、tick を 1 ms にしても通信が遅くならない。応答待ちの空回りが減る。
- 調べた事実（ファームウェア 3.0.2 のソース、`firmware/nina-fw/source/main/CommandHandler.cpp`）: データの到着を知らせる
  IRQ の線（ESP32 の GPIO0 = RP2040 の GPIO2）はあるが、`updateGpio0Pin()` が見るのは古い方式のソケット（`tcpClients`・
  `udps`・`tlsClients`）だけ。ドライバーが使う BSD 風の命令（0x70〜0x7f）のソケットでは上がらない。データの到着待ち
  （20 ms ごとの poll）をイベントにするには、ファームウェアへの 3 件目の修正が要る。実機で線を観測してはいない。

## F10 の記録（2026-11-17）

- 実装: `freertos/drivers/wifi/wifi.cmake`（共有の `nina_net.c`、`net_mgmt`・`wifi_mgmt`・`wifi_utils`、資格情報、
  `wifi_profile.c`）、`drivers/compat/zephyr_net_if.c`・`zephyr_net_mgmt_os.h`、`drivers/nina/nina_netif.c`、FreeRTOS 用の
  `wifi_profile_os.h`、ホストテスト `freertos/tests/wifi`（約 25 秒）。`wifi cred` は Zephyr 本体の `wifi_credentials_shell.c` の
  無改変の写し（`zephyr_copies` 23 本）。`wifi connect/disconnect/scan/status` は Zephyr の `wifi_shell.c`（6289 行）から 4 つだけ
  取り出した `common/compat/zephyr_wifi_shell.c`（Apache-2.0、約 1200 行。FreeRTOS 版だけがビルド）。
- 共有ファイルの変更: `shell.h`（`SHELL_SUBCMD_ADD()`、`shell_help()`）、`zshell.c`・`zshell_os.h`（`ZSHELL_OS_ACTIVE_CMD`）。
  メインが両方を再ビルドし、Zephyr の像が基準と、NuttX の像が snapshot-t24 と一致。NuttX 側のホストテスト全通過。
- スレッド: `nina_scan_tid` 12・1536、`nina_connect_tid` 12・1024、`nina_monitor_tid` 11・1536、`wifi_reconnect_tid` 12・2048、
  `net_mgmt` 21・2048。起動の順は NuttX 版の bringup と init と同じ。起動時に KV を消す・書くコードは無い。
- メインが確かめたこと: Zephyr・NuttX の像、FreeRTOS のビルド（既定・selftest、警告 0）、整形、`host_tests.sh`。ソースは読んでいない。
- 実機（AP `rp2040-test`、PC 10.42.0.1）:
  - 書き込み後、**保存済みの資格情報で自動接続**（F04 の GC を何度も経た KV。`wifi status` が `State: COMPLETED`、SSID 一致、
    RSSI -40）。アドレスは PC の `ip neigh` で 10.42.0.97、ping 4/4。`wifi cred list` は実行していない。
  - `wifi scan`: `Disconnected` → 表（12 件ほど、`rp2040-test` を含む）→ `Scan request done` → `Connected`、ping 復帰。
  - `wifi disconnect`: ping 0、`State: DISCONNECTED`、10 秒後も再接続しない。`wifi cred auto_connect`: `Connected`、ping 復帰。
  - 計測中（`imu start`）に切断 → 5 秒 → 再接続: `errors.stream` 0、オーバーラン 0、`cpu_active` 27.2%、`max_transfer_us` 6650。
  - `reboot` 後、約 12 秒で ping が戻る。`dmesg` にエラーなし。
  - スタックの空き: `nina_connect_tid` 496/1024、`nina_scan_tid` 696/1536、`nina_monitor_tid` 1128/1536、`wifi_reconnect_tid`
    1044/2048、`net_mgmt` 1288/2048、`console` 1532/3072。アイドル（接続中）の `cpu_active` 2.6%。ヒープの空き 31536/32768。
  - 未確認: AP 側を落としての切断の検出と再接続（AP はユーザーが起動したものなので止めない）、接続の期限 30 秒、長時間。
- 相談事項: IP アドレスを表示するコマンドが無い（`net iface` を作るか、`wifi status` に足すか）。`wifi` のヘルプは 5 個だけで、
  `wifi connect` のヘルプが 50 行ある（Zephyr の文面のまま）。互換層の `net_mgmt` の add が重複を除かない（NuttX の像が変わるので
  直していない）。`net_mgmt`（優先度 21）がイベントの文字を直接コンソールに書く。
- 次: F11（Telnet、速度測定）→ CP-F3。

## ユーザーの指示: `wifi` の使えないオプションを削る（2026-11-17）

- `common/compat/zephyr_wifi_shell.c`（FreeRTOS 版だけがビルド）から、このボード（NINA のドライバー）で使えないオプションを、
  ヘルプと引数の解析の両方から削る（企業向けの認証、AP・TWT 用、ドライバーが見ないスキャンの指定など）。残すのは
  `nina_net.c` が実際に使うものだけ。F11 の subagent が同じビルドディレクトリーを使っているので、F11 が戻ってからメインがやる。

## F11 の記録と CP-F3（2026-11-17）

- 実装: `freertos/drivers/telnet/nano_telnet.c`（タスク `telnet_service` と `shell_telnet`、どちらも優先度 6。ポート 23、同時 1 接続、
  IAC は取り除いて応答しない、エコーなし = Zephyr の `shell_telnet.c` と同じ）、`common/compat/zperf_iperf3.[ch]`（iperf3 の
  制御プロトコルの最小限の自作、MIT）と `zephyr_zperf_shell.c`（Zephyr の `zperf_shell.c` からの抜き出し、Apache-2.0）、
  `zperf_server_tid`（優先度 6、2048 B）、`nano_console` にロックの待ち時間の設定、ホストテスト `freertos/tests/telnet`
  （python3 と iperf3 を使う。約 60 秒）。共有の既存ファイルは変えていない。
- Zephyr 版と違えた点（subagent の報告）: 2 本目を断る前に 1 本目が閉じていないか確かめ直す（無いと、閉じてすぐ繋ぎ直した
  クライアントが毎回断られる）。送信の再試行は 10 ms ごと、閉じるのはサービスのタスクだけ。プロンプトをすぐ送る。新しい
  クライアントは空の行から。poll の失敗は 100 ms ごとに再試行（Zephyr 版はサービスのスレッドが終わる、とのこと）。
- メインが確かめたこと: ビルド（既定・selftest、警告 0）、整形、`host_tests.sh`。Zephyr・NuttX の像の一致は subagent の報告の
  まま（既存の共有ファイルは変えていない）。ソースは読んでいない。
- **CP-F3（実機、ボード 10.42.0.97、PC 10.42.0.1）**:
  - 起動時に `shell_telnet: Telnet shell backend initialized`。Telnet で接続（Python のソケット）: Enter でプロンプト、`help`
    （23 行）、`ps`、`dmesg`（124 行、切れない）。2 本目はすぐ閉じられ（受信 0 バイト）、`Telnet client already connected.`、
    1 本目は使える。USB の `ps` と Telnet の `dmesg` を同時に出しても混ざらない。
  - 繋ぎ直し: 3 秒おき 20 回 20/20、0.5 秒おき 20 回 20/20、そのあとも繋がる。
  - `zperf tcp upload 10.42.0.1 5201 20 1024`: ボード側 2.94 Mbps、`iperf3 -s`（3.16）側 2.93 Mbit/s（Zephyr 版の文書は
    2.93〜3.00）。`zperf tcp download 5201` + `iperf3 -c -t 20`: 1.38 Mbit/s（Zephyr 版 1.54〜1.57、NuttX 版 0.95〜1.03）。
  - 計測中（`imu start`）の download 10 秒: 1.10 Mbit/s、取得のエラー・オーバーラン 0、`cpu_active` 51.9%。
  - スタックの空き: `shell_telnet` 1540/3072、`telnet_service` 1276/2048、`zperf_server_tid` 1104/2048。アイドル（接続中、
    Telnet の待ち受けあり）の `cpu_active` 4.2%（F10 は 2.6%）。ヒープ 31392/32768。
  - 1 回だけ見たもの: 書き込み後の最初の起動の 25.0 秒に `<err> shell_telnet: Telnet poll failed (-116)`（そのあと Telnet は
    使えた。再起動後は出ない。原因未調査）。
  - 未確認: `wifi disconnect` での Telnet の切断と再接続後の接続、`telnet` コマンド・`nc` からの操作、読まないクライアントの
    10 秒での切断、Telnet からの `zperf`。
- **説明のつかない記録**: F11 の書き込みの前、F10 の像が動いていた間（メインの `reboot` から 1119 秒後）に HardFault の記録が
  1 件（task console、レジスターの形は `devmem` で存在しない番地を読んだときと同じ）。メインはその時刻に実機を操作していない。
  **解決**: ユーザーが自分で `devmem 30000000` を試したもの（同日に確認）。subagent は実機に触れていない。
- IP アドレスの表示の案（subagent）: Zephyr の `net iface` を、互換層が持てる行だけで作る（100〜200 行。ルートの README の
  手順が `net iface` を使っている）。実装はユーザーと相談のうえ。
- 次: `wifi` の使えないオプションを削る（ユーザーの指示）→ F12（時刻同期と `ntp`）、F13（MQTT）→ CP-F4。

## `wifi` の使えないオプションを削った（2026-11-17、ユーザーの指示）

- `common/compat/zephyr_wifi_shell.c`: `wifi connect` は `-s`・`-p`・`-k`（0: なし、1: WPA2-PSK だけ）と `-h`/`--help`。
  `scan`・`disconnect`・`status` は引数なし。削ったオプションは解析からも外した。使わなくなった補助関数も削除
  （text −8.7 KB）。`wifi` のヘルプは約 100 行 → 14 行。`wifi status` の出力と `wifi cred` は変えていない。
- メインの追加の修正（Zephyr の癖を直した）: `-h` が引数を取らない、`--help` でヘルプが 2 回出ない、知らない長いオプションの
  エラーにその文字列を出す（Zephyr は NUL を 1 個出す）。
- 確認: ビルド（既定・selftest、警告 0）、整形、`host_tests.sh`（41 個）。実機: `wifi`、`wifi connect --help`・`-h`（各 1 回）、
  `--channel=6`・`-c 6`・`-k 3`・`scan -t active`・`status -i 1` はエラーとヘルプだけで、接続は保たれる（`State: COMPLETED`、
  ping 2/2）。`dmesg` には getopt の `<wrn>` が残る（Zephyr の getopt の動作）。`-k 1 -p` での手動の接続は実機では未確認
  （パスフレーズの入力が要る。ホストテストでは通っている）。
- IP アドレスの表示: NuttX 版と同じ `ifconfig`（2 行）にする方向（`net iface` だと `net` の下が `iface` だけになる）。F12 と一緒に入れる。

## F12 の記録（2026-11-17）

- 実装: スタブ `time_sync_null.c` を共有の `time_sync.c`・`time_offset.c` に置き換え。壁時計 `zephyr_sys_clock.c`（TIMER + 基準値、
  NuttX 版と同じ作り。newlib の `gettimeofday` も同じ時計に）、`time_sync_prelude.h`（Pico SDK の `TIMER_BASE` との重なりを
  `time_sync.c` だけで処理）、`CONFIG_NINA_NTP_*`（3 版と同じ値）、`time_sync_tid`（優先度 11、2048 B、NuttX 版と同じ起動位置）、
  `ntp` コマンド、ホストテスト `freertos/tests/time_sync`。共有・NuttX・Zephyr のファイルは変えていない。
- `ifconfig`（`freertos/drivers/wifi/nano_ifconfig.c`）: NuttX 版と同じ 2 行と空行。切断中は `at UP`、アドレスとマスクは 0.0.0.0
  （NuttX 版は切断後もマスクを残す。違いは 1 点だけ）。`net iface` は作らない（ユーザーの決定）。
- **`irqstack` を廃止し `ps` にまとめた**（ユーザーの指示）: `ps` の表に `(interrupts)` の行（割り込み用スタックの空き）。
  `ps` のタスク数の上限を 16 → 24（タスクが 17 個になり `More than 16 tasks` で表示できなくなったのを実機で見つけた）。
- メインが確かめたこと: ビルド（既定・selftest、警告 0）、整形、`host_tests.sh`（42 個）。ソースは読んでいない。
- 実機（PC の chrony が相手）:
  - 起動から 26.6 秒の時点で `state: synchronized`、queries 9・replies 9、`realtime_clock_steps` 1、timeouts 0。往復 `min_us` 6012。
    66 秒後: `rate_valid` 1、`rate_ppb` -1028、`last_residual_us` 3366、`in_use_us` 6294。
  - 同期後の `imu start`（36.6 秒）: `synchronized: 1`、`unix_unavailable: 0`、エラー・オーバーラン 0、`cpu_active` 30.3〜30.4%
    （時刻同期なしの CP-F2 は 27.5%）、落ちた tick 7。
  - `wifi disconnect` 中: `ifconfig` が `at UP`・0.0.0.0、`ntp status` の queries が増えない。再接続後: `at RUNNING`・10.42.0.97、
    replies が数え直しで同期に戻る。`ifconfig eth0` はエラー。Telnet からも `ifconfig`・`ntp status` が出る。
  - `ps`: 17 タスクと `(interrupts)` 1580〜1620。`time_sync_tid` の空き 952/2048。アイドル（接続中）の `cpu_active` 2.7%。
  - 未確認: 切断 60 秒後の `not synchronized`、サーバーを名前で指定する経路、長時間（壁時計の step の増え方）。
- 相談事項: Zephyr 版・NuttX 版にある `date` が FreeRTOS 版に無い（足すか）。`ifconfig` の切断中のマスクを NuttX に合わせるか。
- 次: F13（MQTT と `imu_raw_mqtt.c`）→ CP-F4。そこで受信器での確認と、3 版の `cpu_active` の比較。

## ユーザーの決定（2026-11-17、F12 のあと）

- `date` コマンドを足す。`ifconfig` の切断中のマスクは 0.0.0.0 のままでよい。
- `date` は F13 が戻ってからメインが入れる（`commands.c` を F13 の subagent が使っているため）。
- `date` は NuttX 版の表示（引数なしで `Thu, Jan 01 00:00:00 1970` の形）で小さく自作する（ユーザーの決定。Zephyr の
  `date get`/`date set` の写しは使わない）。表示だけで、設定は付けない。

## F13 の記録と CP-F4（2026-11-17）

- 実装: スタブ `imu_raw_mqtt_null.c` を共有の `imu_raw_mqtt.c`・`imu_raw_status.c`・`mqtt_min.c` に置き換え。
  **`mqtt_min_sock.c` を `nuttx/apps/imu/` から `common/imu/` へ移して NuttX 版と共有**（`<nuttx/config.h>` の行とコメントだけ変更）。
  `imu_raw_mqtt_tid`（優先度 9、2048 B、NuttX 版と同じ起動位置）、ホストテスト `freertos/tests/mqtt`（偽のブローカー）と
  `live`（ホスト上で本物の `imu-host check` を相手に通す。`host/.venv` が無ければ skip）。
- メインの追加: `date`（NuttX 版の書式、UTC、表示だけ。ユーザーの決定）。
- メインが確かめたこと: NuttX の像が snapshot-t24 と一致・NuttX 側のホストテスト全通過、Zephyr の像が基準と一致、FreeRTOS の
  ビルド（既定・selftest、警告 0）、整形、`host_tests.sh`（44 個）。ソースは読んでいない。
- **CP-F4（実機、AP 経由、受信器 `imu-host check --port 18830`）**:
  - `date` が PC の UTC と秒まで一致。
  - **書き込み直後の最初の計測 56.1 秒**: 受信器 1959 パケット、欠番 0、`unix_invalid` 0、`unix_regressions` 0、`us_per_sample` 中央値
    149.177、バッチの時刻と対応点の直線との差 -0.319〜+0.313 ms。ボード: エラー・オーバーラン 0、`cpu_active` **45.5%**、
    `publish_max_us` 37030、`queue_high` 1、`lock_wait_max_us` 20、`max_transfer_us` 8000、`dropped_batches` 0。`imu start` の
    2 秒後にはもう接続・取得中（3 秒の待ちは無くなった）。送信中に Telnet の接続・`imu status`・切断を 2 回。落ちた tick は
    46 秒で 2。NTP: `min_us` 4614、`last_residual_us` -147。
  - 送信中の `ps`（起動からの割合）: `imu_fifo_worker` 6.9%、`imu_raw_mqtt_tid` 6.3%、`imu_stream_tid` 4.5%、`sysworkq` 0.6%。
    スタックの空き: `imu_raw_mqtt_tid` 920〜952/2048、`imu_stream_tid` 1056、`imu_fifo_worker` 1224、`time_sync_tid` 952。
  - `--drop-at 10 --drop-for 5`: 再接続（ボード connects 2・failures 5、受信器 connects 2）、ボードの破棄 182・受信器の欠番 183。
  - `--stall-at 10 --stall-for 3 --rcvbuf 4096`: `queue_high` 12、ボードの破棄 90 = 受信器の欠番 90、接続は 1 回のまま、
    `publish_max_us` 1226857。
  - 計測中に `wifi disconnect` → 5 秒 → `wifi cred auto_connect`: MQTT が自動で繋ぎ直る（connects 2、failures 9、破棄 318）、
    取得のエラー 0（受信器側の集計は取れていない: メインのスクリプトの都合）。
  - 計測中の `zperf tcp download` 10 秒: 817 kbit/s、`cpu_active` 52.0%、`queue_high` 12、破棄 1 = 受信器の欠番 1。
  - 開始と停止を 10 回（各 6 秒）: 毎回 `stopped`、連番 +1、publish 208〜209、破棄 0、受信器は接続 10 回・欠番 0。
    重ねての `imu start` は `Already running`。
  - 未確認: 長時間、10 秒を超える輻輳、時分割のフックなしとの比較（あとで判断する項目）。
- **`cpu_active` の比較（送信あり、同じ AP と受信器）**: FreeRTOS 版 45.5%（今回）、NuttX 版 70.0%（今日の CP-B、71 秒、T24 の像）、
  Zephyr 版 63〜64%（文書の値。今日は測っていない）。Zephyr 版を同じ条件で測るには書き込みが要る（CP-Z）。
- 次: F14（`sensor` コマンド、`imu cal` の確認）、F15（ホストの差分ファイルと文書）→ CP-F5。

## F14 の記録（2026-11-17）

- 実装: 共有の `sensor_shell.c`・`sensor_shell_stream.c` を FreeRTOS 版にリンク。`freertos/drivers/sensor_shell/`
  （`sensor_shell_os.[ch]`: `sensor get` は結果が出るまで最長 2 秒待つ = NuttX 版と同じ。Zephyr 版はすぐ戻ってプロンプトを
  描き直す）、`sensor_shell_processing_tid`（優先度 20、3072 B）、ホストテスト `freertos/tests/sensor_shell`。出力はコマンドを
  打ったセッション（USB か Telnet）へ。`common/`・`nuttx/`・`zephyr/` は変えていない。
- メインが確かめたこと: ビルド（既定・selftest、警告 0）、整形、`host_tests.sh`（45 個）。ソースは読んでいない。
- 実機（ボード静止）: `sensor` のヘルプ。`imu cal` が accel・gyro とも `calibrated`（KV の校正値）。`sampling_frequency 104` を設定して
  `sensor get lsm6dsox@6a` → 8 行（Zephyr の書式）。`full_scale` を 4 g / 1000 dps にすると `imu cal` が `nominal` になり値が変わる、
  8 g / 2000 dps に戻すと `calibrated`。`imu cal zero` → `tracked at rest, 1 updates`、そのあとのジャイロは 0.0005 rad/s 以下、
  `imu cal reload` で `stored` に戻る。`sensor stream lsm6dsox@6a on fifo_wm incl` → 約 0.14 秒ごとに 3 行、`off` で止まる。
  ストリーム中の `imu start` は `The IMU FIFO is in use (sensor stream?)`。計測中の `sensor get` は `Read failed: -16`、
  `imu cal zero` は `A measurement is running; ...`。Telnet からの `sensor get` は Telnet にだけ出る。
  スタックの空き: `sensor_shell_processing_tid` 1720/3072、**`RTIO WQ 0` 376/1024**（`sensor get` で初めて使われた。余裕が小さい）。
  ヒープ 30992/32768。
  - 見たもの: ODR を設定した直後の最初の `sensor get` は値がずれていた（加速度 z 8.56、ジャイロ z -0.33）。2 回目以降は正常。
    センサーの立ち上がりとみられるが未調査（ほかの版と比べていない）。
- このボードで使えないもの（subagent の調査）: `trig` 全部（-88）、`stream` は `on fifo_wm incl`/`off` 以外、`attr_get`・`attr_set` の
  大半の組、`get` の加速度・角速度以外の 70 チャネル。FreeRTOS 版だけで隠す案 A（`trig` を外す）、B（A + `stream` を絞る）、
  C（B + チャネル名・属性名を絞る）。共有ソースは変えず、FreeRTOS 側で `#include` して表だけ作り直す。ユーザーに相談中。
- 次: F15（ホストの差分ファイル、`kv_transfer.py`、文書）→ CP-F5。

## F15 の記録と CP-F5（2026-11-17）

- PC 側: `host/firmware_profiles.toml` に FreeRTOS 版、`script/kv_transfer.py`（`version` で判定、`--firmware freertos`）、
  `nuttx/script/flash.sh`（FreeRTOS 版が動いていれば `bootsel` を案内）、`host/tests/test_kv_transfer.py`（新規）と
  `test_board.py`。`imu-host` の calib・record・check は版に依らない。
- 文書（日英）: 新規 `freertos/drivers/README`（内部の説明、246/275 行）。設計書に §12「共有ソース」と §13「FreeRTOS 版」。
  ルートの README に「3 つの版と `common/`」「FreeRTOS 版」「FreeRTOS 版の性能」。古い数字（テストの本数、`zephyr_copies`）と
  共有化で変わったパスを修正。未調査・推定の事柄は文書に入れていない（subagent の報告に一覧）。
- メインの追加: `freertos/README`（日英）から `drivers/README` と `kv_transfer.py` への案内、ソースのコメント 2 か所の参照先、
  `CLAUDE.md` の FreeRTOS 版の節の書き直し（未追跡）。
- メインが確かめたこと: ホスト側のテスト 27 個、実機で `kv_transfer.py backup --usb`（FreeRTOS 版と判定、校正値 2 個と
  `imu.port` が以前の退避と一致）と `--board 10.42.0.97`（Telnet、USB と同じ結果）、`restore --dry-run`。ルートの README（日）の
  差分を読んだ。ほかの文書は読んでいない（subagent の報告のまま）。`restore` の実行、`imu-host write`・`record`、ImuStudio は未確認。
- **CP-F5（F01〜F15）完了**。残り: `sensor` の使えないサブコマンドの扱い（相談中）、あとで判断する 3 項目、P4（Zephyr 版の
  載せ替え: Z01 NINA、Z02 I2C は着手前に相談）、CP-Z。

## `sensor` の決定と `cpu_active` の数え漏れの確認（2026-11-17）

- ユーザーの決定: `sensor` は削らない（案 A・B・C とも不採用。共有ソースのまま）。
- `cpu_active` に数え漏れがないかの確認（FreeRTOS 版、F15 の既定ビルド、再起動後）。
  - 仕組み（`idle.c` を読んだ）: アイドルと数えるのは、アイドルフックが割り込みを止めて WFI で待つ区間だけ。起こした割り込みの
    ハンドラーは区間を閉じたあとで走るので負荷になる。タスクを横取りした割り込み、割り込み禁止の区間（フラッシュ消去など）、
    カーネルの切り替え、アイドルタスクのループも負荷。WFI を使う場所はほかに無い（`fault.c` の停止後だけ）。
  - 実測（`ps` のタスクごとの時間と比べた。こちらは割り込みの時間を横取りされたタスクに入れるので、アイドル中の割り込みは
    `IDLE` に入る）: 静止（Wi-Fi 接続、計測なし）60 秒で `cpu_active` 4.1%、`100 − IDLE` 1.0%、差 3.1 ポイント。計測と送信中 60 秒で
    44.3%、40.7%（`imu_raw_mqtt_tid` 14.1、`imu_fifo_worker` 13.8、`imu_stream_tid` 9.7、`telnet_service` 1.3、`sysworkq` 1.3）、
    差 3.6 ポイント。差はアイドル中に入った割り込みとアイドルのループで、`cpu_active` に入っている。受信器は欠番 0。
  - 数え漏れ: 区間の両端（時刻を読んでから WFI まで、起きてから時刻を読むまで）の約 45 命令サイクルがアイドルに入る。
    逆アセンブルからの見積もりで 1 回 約 0.4 µs、tick ごとに 1 回なので約 0.4 ポイント（実測ではない）。
  - 静止時の約 3 ポイントは、ほぼ 100 µs の tick 割り込み（1 回あたり約 3 µs）。tick を 1 ms にする判断の材料。

## tick の頻度を変えた静止時の `cpu_active`（2026-11-18）

- 目的: アイドル中の割り込みが `cpu_active` に入っていることの確認。`configTICK_RATE_HZ` だけを変えた試験用の像（1000・5000・
  20000。ソースは戻した）と既定の像（10000）を順に書き、起動 15 秒後から静止 60 秒を測った。
- 結果（`cpu_active` / `100 − IDLE` / 差）: 1 kHz 1.0 / 0.7 / 0.3、5 kHz 2.6 / 1.0 / 1.6、10 kHz 3.1〜3.2 / 1.0 / 2.1〜2.2（7 回。
  Wi-Fi 未接続 4 回、接続 3 回、`reboot` のあと 2 回を含む）、20 kHz 7.2 / 1.1 / 6.1。1・5・20 kHz は Wi-Fi 未接続（AP が落ちていた）で各 1 回。
- 言えること: タスクの時間は変わらず、`cpu_active` だけが tick の頻度とともに増える。アイドル中の tick 割り込みは数えられている。
  tick を 1 ms にすると静止時の差は 0.3 ポイントになる。
- 合わない点（未解明）: 1 回あたりは 1・5・20 kHz で約 3.1 µs、10 kHz で約 2.2 µs と比例しない。前日の 10 kHz は 4.1 / 1.0 / 3.1（1 回）
  だった。メインの推定: 時刻は 1 µs 刻みで、tick の周期はどれも µs の整数倍なので、アイドルの区間の端が µs の境目に対して毎回同じ
  位置に来て、切り捨ての誤差が平均されない（1 区間あたり最大 1 µs、10 kHz で最大 1 ポイント）。確かめていない。
- 実機: 既定の像（F15 のビルド）に戻した。AP に接続、計測は停止。

## 送信中の `cpu_active` が Zephyr 版より低い原因の切り分け（2026-11-18）

- 発端: 送信中の `cpu_active` が Zephyr 版 62.6〜63.3%（2026-11-14）、FreeRTOS 版 44〜45%。ユーザーの指示でこの時点で原因を調べる。
- ビルドの違い（確認済み）: 最適化（FreeRTOS `-O3`、Zephyr・NuttX `-Os`）、boot2（FreeRTOS は Pico SDK のボード定義で W25Q080 用・
  分周 2 = 62.5 MHz、Zephyr・NuttX は `boot2_at25sf128a.S` の既定の分周 4 = 31.25 MHz。逆アセンブルで BAUDR の値を確認。Zephyr と NuttX の
  boot2 は同一）、割り算・`memcpy`・64 ビット演算（FreeRTOS は Pico SDK のハードウェア除算器と ROM、ほかは libgcc）。
- 実測（FreeRTOS 版を 1 項目ずつ変えた試験用の像。書き込み後に `imu start`、10 秒後から 60 秒、各 1 回。全回 欠番 0、overruns 0）:

  | 像 | `cpu_active` | mqtt / fifo / stream |
  |---|---|---|
  | 既定 | 44.0% | （前日 14.1 / 13.8 / 9.7） |
  | boot2 だけ Zephyr 版のもの | 54.9% | 15.5 / 18.6 / 12.5 |
  | `-Os`（MinSizeRel）だけ | 43.6% | 14.2 / 11.5 / 12.0 |
  | `-O3` + libgcc の割り算・mem・int64・bit | 47.3% | 14.3 / 16.3 / 11.3 |
  | `-Os` + libgcc | 45.4% | 14.4 / 11.8 / 12.8 |
  | `-Os` + libgcc + Zephyr の boot2 | 54.5% | 15.8 / 15.4 / 15.0 |

- 言えること: 差の約 11 ポイントは boot2（フラッシュの読み出し）。最適化と libgcc は合わせても 1〜3 ポイント。3 項目をそろえても
  54.5% で、Zephyr 版の値（別の日）まで約 8 ポイント残る。残りは未解明（候補: Zephyr 版は NINA・I2C が共通コアでない自前のドライバー、
  SPI ドライバー、カーネル。Zephyr 版を同じ日・同じ手順で測り直していない）。
- 試験用の像はスクラッチにあり、ソースは変えていない。実機は既定の像に戻した。

## Zephyr 版に速い boot2・除算器・ROM の `memcpy` を入れた実験（2026-11-18）

- ユーザーの指示: 今の Zephyr 版の測り直しは飛ばし、Zephyr 版の boot2 を差し替え、割り算と `memcpy` をハードウェア除算器と ROM の
  ルーチンにして測る。
- 実験用の像（commit していない。パッチと `exp_fastlib.c` はスクラッチ、ビルドは `zephyr/build/exp-fastlib`）:
  - boot2: UF2 の先頭 256 バイトを FreeRTOS 版のもの（W25Q080 用、分周 2）に差し替え。実機で `devmem 0x18000014` が 2。
  - 割り算: Pico SDK の `divider_hardware.S`（Zephyr の hal_rpi_pico にある版）を `--wrap` で 32・64 ビットの 6 関数に。Zephyr は
    コンテキスト切り替えで除算器を退避しないので `PICO_DIVIDER_DISABLE_INTERRUPTS=1`（割り算の間だけ割り込み禁止。FreeRTOS 版は
    切り替え時に退避する方式）。
  - `memcpy`・`memset`: ブート ROM のルーチンを初回に引いて呼ぶ自作の包み（SDK の `mem_ops_aeabi.S` は起動時の初期化の順序が
    合わないので使っていない）。`.data`・`.bss` の準備前は `arch_early_memcpy`・`arch_early_memset` をバイト単位の実装に置き換え。
  - 最適化は `-Os` のまま。
- 実測（FreeRTOS 版と同じ手順: `imu start`、10 秒後と 70 秒後に `kernel thread list`、`imu stop`。`imu status` の `cpu_active`、約 71 秒）:
  48.7%、48.8%（2 回。欠番 0、overruns 0、batches = published）。元の Zephyr 版は 62.6〜63.3%（2026-11-14、今日は測っていない）、
  FreeRTOS 版の既定は 44.0〜44.3%。スレッドごとの値は集計スクリプトの不備で取れていない。
- 言えること: この 2 つの変更で Zephyr 版が約 14 ポイント下がり、FreeRTOS 版との差は約 4.5 ポイントになる。boot2 と除算・`memcpy` の
  内訳は Zephyr 版では分けて測っていない。
- 併せて確かめたこと: NuttX 版・FreeRTOS 版が書いた KV を Zephyr 版が読み、保存済みの資格情報で自動接続した（`kv get imu.port` も一致）。
- 実機: 実験用の Zephyr 版のまま（FreeRTOS 版へ戻すには RESET 2 回の操作が要る）。

## FreeRTOS 版を `-Os` に、時分割のフックの比較（2026-11-18）

- ユーザーの決定: FreeRTOS 版の最適化を `-Os` にする（Zephyr 版・NuttX 版と同じ）。boot2 の分周 2 を Zephyr 版・NuttX 版に正式に入れる。
  除算器と ROM の `memcpy`・`memset` を Zephyr 版・NuttX 版に入れるかは変更量を見てから（まず検討）。保留の 3 項目に着手する。
- `-Os`: `build.sh` の `CMAKE_BUILD_TYPE` を `MinSizeRel` に。像は 297860 → 227328 バイト（約 24% 減）。ビルド（既定・selftest）警告 0。
  書き込み直後の最初の計測（送信、60 秒）: `cpu_active` 43.5%、欠番 0、overruns 0。`selftest` は `-Os` では未実行。
- 時分割のフック（`-DNANO_SCHED_ZEPHYR=OFF` との比較、どちらも `-Os`）:

  | 試験 | フックあり（既定） | フックなし |
  |---|---|---|
  | 送信 60 秒の `cpu_active` | 43.5%（1 回） | 43.0%、42.9%（2 回） |
  | 同、欠番・overruns | 0・0 | 0・0 |
  | 送信 + `zperf tcp upload` 20 秒 + Telnet で `ps` を繰り返す | drop 586/1439、zperf 2.67 Mbit/s、overruns 0 | drop 636/1438、zperf 2.78 Mbit/s、overruns 0 |

  併用の試験の drop は、NINA の回線を zperf と分け合うためのもの（CP-F4 と同じ振る舞い）。フックの有無で差は見えない。
  未確認: 長時間、`selftest`（フックなしでは `timeslice` の 2 項目が skipped）。
- 1 ms 未満の待ちの一覧（tick を 1 ms にする材料）: 共有コードでは NINA の READY のポーリング（`nina_spi.c` の
  `nina_os_sleep_us(NINA_READY_POLL_US)`）だけ。READY を割り込みにすれば、1 ms 未満の待ちは無くなる。
- 実機: フックなしの像（`-Os`）。

## Zephyr 版・NuttX 版の boot2 を分周 2 に（2026-11-18）

- 実装（subagent）: Zephyr は `app/usb-console/CMakeLists.txt` で上流の bootloader プロジェクトを `-DPICO_FLASH_SPI_CLKDIV=2` 付きで
  もう一度ビルドし、`use_local_driver_sources` で hal_rpi_pico の `boot_stage2.S` と差し替える（上流に指定の口が無い）。NuttX は
  ボードの `scripts/Make.defs` に `BOOT2CFLAGS += -DPICO_FLASH_SPI_CLKDIV=2`、`build.sh` の再構成の入力に同ファイルを追加。
  設計書（`nuttx-porting`、日英）に build.sh の説明を 1 か所。フラッシュの種類（AT25SF128A 用）は変えていない。
- メインが確かめたこと: 差分を読んだ。subagent がビルドした像が、基準と 5 バイト（オフセット 0x1c の 4 → 2 と CRC 4 バイト）だけ違い、
  2 版の新しい boot2 が同一。メイン自身は再ビルドしていない。ホストテストは subagent の報告（両方 rc=0）。
- 実機: 新しい boot2 を FreeRTOS 版（`-Os`）の UF2 の先頭に入れて書き込み、起動・`devmem 18000014` = 2・送信 60 秒
  （`cpu_active` 43.1%、欠番 0）を確認。Zephyr 版・NuttX 版そのものでは未確認（CP-Z と NuttX の再確認で測る）。
- **新しい基準**: `freertos/build/baseline/zephyr/snapshot-clkdiv2/image.bin`、`.../nuttx/snapshot-clkdiv2/image.bin`
  （以後 `common/` を触ったらこれと比べる。NuttX は従来どおり `SOURCE_DATE_EPOCH=1763337600 ... --clean -j 1`）。
- README に入れること（未反映）: ルートの README の性能の表は boot2 の変更前の値。Zephyr 版・NuttX 版を測り直したら直す。
- 除算器と ROM の `memcpy`・`memset` の検討（subagent、実装なし）: Zephyr は `--wrap` で新規 1 ファイル約 70 行 + CMake 約 15 行。
  NuttX は `memcpy`・`memset` が `LIBC_ARCH_MEMCPY/MEMSET` で約 50 行、除算器は pico-sdk の取得範囲の追加・空ヘッダー 2 つ・
  `--wrap` で約 20 行。除算器はどちらもコンテキスト切り替えで退避できないので割り込み禁止方式（64 ビットで推定 100 サイクル前後）。
  現状は両版とも `memcpy`・`memset` がバイト単位のループ、割り算は libgcc。subagent の推奨は「boot2 だけで測ってから、`memcpy`・
  `memset` の ROM 版を先に、除算器は見送り寄り」。ユーザーの判断待ち。

## READY の割り込み待ちと tick 1 ms（オプション）の実装と比較（2026-11-18）

- ユーザーの決定（同日）: 時分割のフックは外し、コードも削除する。Zephyr 版・NuttX 版に ROM の `memcpy`・`memset` を入れる
  （subagent が実装中）。除算器は未定（状態の退避ができるかの質問あり）。
- 実装（subagent）: `-DNANO_NINA_READY_IRQ=ON`（既定 OFF。共有コアに任意の境界 `nina_os_ready_arm/wait/disarm`、`NINA_OS_WAIT_READY` の
  ときだけ使う。FreeRTOS 版は GPIO10 のエッジ割り込み + セマフォ、10 ms ごとに読み直す）と `-DNANO_TICK_HZ=1000`（既定 10000）。
  既定の像は変更前と一致、Zephyr・NuttX の像は `snapshot-clkdiv2` と一致（subagent の確認）。ホストテスト `nina_irq`、
  `common/tests` の `nina_spi_wait_ready` を追加。1 ms 未満の待ちをタイマーで作る仕組みは作っていない。
- メインが確かめたこと: `nina_spi.c` の差分を読んだ（割り込みを有効にしてから READY を読む順）。`format.sh --check`。ほかは読んでいない。
  ビルド・ホストテスト・像の比較は subagent の報告。
- 実機（`-Os`、フックあり、新しい boot2。各 1 回。送信 60 秒は書き込み直後の最初の計測、全回 欠番 0・overruns 0）:

  | 像 | 送信中 `cpu_active` | 静止 30 秒 | zperf 送信 / 受信（10 秒、Mbit/s） | `ntp status` の `last_us` |
  |---|---|---|---|---|
  | 既定（10 kHz、ポーリング） | 43.1% | 2.9% | 3.04 / 1.36 | 6236 |
  | READY 割り込み | 44.0% | 4.0% | 3.71 / 2.41 | 5227 |
  | tick 1 ms（ポーリングのまま） | 40.5% | 1.0% | 0.90 / 0.21 | 46982 |
  | 両方 | 41.7% | 1.3% | 3.46 / 2.31 | 5478 |

  - 両方 + `selftest`: 0 of 22、i2c 0 of 6、lsm6dso 0 of 3。
  - tick 1 ms だけの像は、1 回目の書き込みのあと約 40 秒たっても Wi-Fi に繋がらなかった（原因未調査）。書き直した 2 回目は 10 秒で接続。
    この組み合わせは通信が大幅に遅く、使わない想定。
  - READY 割り込みだけの像の静止 4.0% が既定より高い理由は未調査（静止時の値には刻みの誤差の疑いがある。前の節）。
  - `dmesg` の `<err>`: `shell_telnet` の 2 行は書き込みのための `bootsel`（ネットワーク停止）のとき、`i2c_dw` の 3 行は `selftest` の
    NACK の試験のもの（RAM ログは再起動をまたいで残る）。Telnet はそのあとも使える。
- 実機: 両方を入れた像（`irq-tick1k`）。既定をどうするかはユーザーの判断待ち。

## フラッシュのクロックの規格（2026-11-18、データシートで確認）

- AT25SF128A（Renesas、DS-AT25SF/QF128A-168 Rev. Q、表 24、85 °C）: 03h は 70 MHz、6Bh は 133 MHz（3.0〜3.6 V）、それ以外
  （EBh を含む）は 120 MHz（3.0〜3.6 V）・108 MHz（2.7〜3.6 V）。105 °C 品は最大 120 MHz。
- W25Q80DV（Winbond、9.6 AC Electrical Characteristics）: 03h は 50 MHz、それ以外は 104 MHz（2.7〜3.6 V、工業用温度）。
- boot2 が使う読み出しは EBh で、分周 2 の 62.5 MHz はどちらの規格にも収まる。clk_sys 125 MHz では分周は偶数なので 62.5 MHz が上限。
- ユーザーの決定（同日）: FreeRTOS 版は READY の割り込み待ちと tick 1 ms を既定にし、古いコード（オプション、フック）は残さない
  （subagent が作業中）。

## Zephyr 版・NuttX 版に ROM の `memcpy`・`memset`（2026-11-18）

- 実装（subagent）: 共有の `common/drivers/rp2040_rom/`（Thumb-1 の入口 `rp2040_rom_mem_ops.S`、ROM を引く `rp2040_rom_mem_lookup.c`。
  ポインターは `.bss`、NULL なら初回に引く。FreeRTOS 版はリンクしない）。Zephyr は `CONFIG_NANO_ROM_MEM_OPS`（既定 y）で 8 記号を
  `--wrap`、`arch_early_memcpy/memset` は picolibc を呼ぶ形で上書き（`src/rom_mem_early.c`）。NuttX は同名の Kconfig（既定 y）が
  `LIBC_ARCH_MEMCPY/MEMSET` を select、`rp2040_start.c` を `-fno-tree-loop-distribute-patterns` でコンパイル（GCC が `.bss`・`.data` の
  ループを `memset`・`memcpy` の呼び出しに変えていたため）、`build.sh` が `__start` に呼び出しが無いことを検査。ホストテスト
  `rp2040_rom_mem`。文書（日英）: Zephyr のドライバー説明、`nuttx-porting`、設計書 §12。
- subagent の確認: OFF の像は 2 版とも `snapshot-clkdiv2` と一致。ON の像は呼び出しが新しい実装へ行く（起動時の 1〜2 か所を除く）。
  像は Zephyr +144 B、NuttX +88 B。ホストテスト 3 種とも成功。メインはソースを読んでいない。
- 実機（Zephyr 版、boot2 分周 2 + ROM の `memcpy`・`memset`）: 起動、`devmem 0x18000014` = 2、KV の読み出し、送信 約 71 秒 × 2 回で
  `cpu_active` 49.0%・49.1%、欠番 0、overruns 0、`dmesg` に `<err>` なし。除算器も入れた実験の像は 48.7〜48.8% だったので、
  Zephyr 版での除算器の上積みは約 0.3 ポイント。NuttX 版は未確認（RESET の操作待ち）。
- **新しい基準**: `freertos/build/baseline/zephyr/snapshot-rommem/image.bin`、`.../nuttx/snapshot-rommem/image.bin`。
- README に入れること（未反映）: `zephyr/README`・`nuttx/README` にオプションの 1 行、ルートの README の性能の節の書き直し（測り直してから）。
- 実機（NuttX 版、boot2 分周 2 + ROM の `memcpy`・`memset`、書き込み直後から 2 回、各 約 70 秒）: 起動、KV の読み出し、自動接続、
  `cpu_active` 55.7%・55.9%、欠番 0、overruns 0、`dmesg` にエラーなし。変更前は 67.7〜67.9%（2026-11-14）、70.0%（2026-11-17、T24 の像）。
- 3 版の送信中の `cpu_active`（2026-11-18、同じ AP と受信器、同じ手順）: Zephyr 49.0〜49.1%、NuttX 55.7〜55.9%、
  FreeRTOS 41.7%（READY 割り込み + tick 1 ms、フックあり、`-Os`）。
- 実機: NuttX 版（FreeRTOS 版へ戻すには RESET 2 回の操作が要る）。

## FreeRTOS 版: READY 割り込み・tick 1 ms を固定、フックを削除（2026-11-18）

- 実装（subagent）: オプション `NANO_NINA_READY_IRQ`・`NANO_TICK_HZ`・`NANO_SCHED_ZEPHYR` を削除し、常に READY 割り込み + tick 1 ms +
  FreeRTOS 標準の同優先度の選び方。`zephyr_os_sched.h`、`z_sched_yield_pending`・`z_sched_switched_out`、`tick_slice_restart` を削除。
  20 ms の時分割（`tick.c`）は決まった刻みで残る（切り替えのたびの数え直しは無い）。共有コアのポーリングの経路は NuttX 版用に残す。
  `selftest` は `timeslice_preempt` を削除して 21 項目、`timeslice` は優先度を Zephyr 0 に上げて残した（ボード上でほかのタスクが
  起きると fail になり得る、未確認）。ホストテストは `nina` を 1 本にまとめ、`telnet` の失敗 2 件（テスト環境の `z_host_isr_depth` を
  スレッドごとに、`client.py` の待ち方）を直した。文書: `freertos/drivers/README`（日英）、設計書 §13.1・§13.2、ルートの README の 1 語。
- 像: 比較用（3 つのオプションで同じ構成にした像）より 64 バイト小さい。差は読まれないフラグの書き込みと yield の周りだけ（subagent の説明）。
- メインがしたこと: `freertos/README`（日英）のオプションの表から `-DNANO_SCHED_ZEPHYR=OFF` の行を削除。`common/` はコメントだけの
  変更で、NuttX を再ビルドして像が `snapshot-rommem` と一致。`CLAUDE.md` の該当箇所を更新。ソースは読んでいない。ビルド・ホストテストは
  subagent の報告。
- 実機: **未確認**（実機は NuttX 版。BOOTSEL の操作待ち）。見ること: 書き込み直後の最初の計測、静止時、zperf、送信中の Telnet、
  `selftest`（0 of 21 か、`timeslice`）、`ntp status`、`dmesg`。
- **実機（最終の既定の像、書き込み直後から）**: 送信 60 秒 `cpu_active` 41.5%、欠番 0、overruns 0、`time` の各カウンター 0。静止 30 秒 1.4%。
  zperf 送信 3.60・受信 2.31 Mbit/s。送信 + zperf + Telnet の `ps` 繰り返し: drop 533/1438（回線の取り合い、従来どおり）、overruns 0、
  zperf 3.27 Mbit/s。`dmesg` に `<err>`・`<wrn>` なし、異常停止の記録なし。静止時の `ntp status` の `last_us` は 5.5 ms 前後（8 点、1 点 7.5 ms。
  zperf の直後に読んだ 1 点は 49.5 ms）。`ps` のタスクは 19 本で以前と同じ（READY のポーリングは独立したタスクではなく、NINA の命令を
  出したタスクの中の待ちだった）。
- `selftest`（最終の selftest の像、21 項目）: Wi-Fi 接続中の 1 回目に 1 件 fail（`k_poll(K_TICKS(3))`: 4134 µs、期待 3001..4080。
  許容幅が 80 µs で、1 ms の tick に対して狭い）。2 回目と Wi-Fi 切断中は 0 of 21。`timeslice` は 3 回とも「8 turns in 7 slices」、
  間隔 20 tick。i2c 0 of 6、lsm6dso 0 of 3。許容幅の見直しは未着手。
- 実機: 最終の既定の像（FreeRTOS 版、`-Os`、新しい boot2 ではなく Pico SDK の boot2）。`selftest` の中で `wifi disconnect` を実行したあと
  書き直したので、起動時の自動接続に戻っている。

## `selftest` の許容幅（2026-11-18）

- ユーザーの決定: `selftest` の許容幅を直す。除算器は保留。README の更新は READY の割り込み化（Zephyr 版・NuttX 版）が終わってから。
  Zephyr 版・NuttX 版の READY の割り込み化を進める（NuttX は境界の実装、Zephyr は Z01 = NINA の共通コアへの載せ替えと一緒。subagent 2 つが作業中）。
- 起床の遅れの許容を `WAKE_SLACK_US` 300 µs に分けた（空回りの待ちの検査は `SLACK_US` 80 µs のまま）。1 ms の tick では同じ tick で
  優先度の高いタスクも起きるので遅れが増える（実測 134 µs）。メインが修正。実機で Wi-Fi 接続中に 5 回、すべて 0 of 21・i2c 0 of 6・lsm6dso 0 of 3。
- 実機: 最終の既定の像。

## NuttX 版の READY の割り込み待ち（2026-11-18）

- 実装（subagent）: `nuttx/drivers/nina/nina_os.c` に `nina_os_ready_arm/wait/disarm`（互換層の `gpio0` のコールバック + `sem_t`、GPIO10 の
  エッジ、GPIO24 と同じ IRQ を共有）。`NINA_OS_WAIT_READY` を常に定義。待ちは 10 ms に区切らず 1 回（NuttX は tickless で、期限と
  待ちが同じ TIMER の上にあるため）。`NANO_NINA` が `NANO_LSM6DSO_STREAM` に依存（`gpio0` のため。既定の構成は変わらない）。
  ホストテスト `nina_os`（壊した版 6 通りが失敗することも確認、subagent）。文書: `nuttx/drivers/README`、`nuttx-porting`（日英）。
  `common/` は変更なし。像 +160 バイト、変わったオブジェクトは `nina_os.o` と `nina_spi.o` だけ（subagent）。
- メインはソースを読んでいない。ビルドとホストテストは subagent の報告。
- 実機（NuttX 版、書き込み直後から）: 起動、自動接続。送信 約 70 秒 × 2 回 `cpu_active` 56.2%・55.8%（ポーリングの像は 55.7%・55.9%）、
  欠番 0、overruns 0。`iperf` 送信 3.19 Mbit/s（12 秒）、受信 1.14 Mbit/s（11 秒）、各 1 回。`dmesg` にエラーなし。
  ポーリングの像の `iperf` を同じ日に測っていないので、速度の変化はまだ言えない（2026-11-14 の値は送信 3.6〜3.9、受信 0.95〜1.03）。
  未確認: 送信中の Telnet、スキャン、切断復帰、KV の書き込みのあと。
- 残っている古いコメント: `common/compat/nano_gpio.h` の冒頭（gpio0 を使う線に GPIO10 が無い）。Z01 の作業が終わってから直す。
- 実機: READY 割り込みの NuttX 版。
- 同じ日のポーリングの NuttX 版（`snapshot-rommem` の像）の `iperf`、2 回: 送信 3.01・2.97 Mbit/s、受信 1.16・1.16 Mbit/s。
  READY 割り込みの像（上、各 1 回）は送信 3.19、受信 1.14。NuttX 版では送信が約 6% 増、受信は変わらない（回数が少ない）。
  FreeRTOS 版の受信 2.31 Mbit/s との差の原因は未調査。
- 実機: ポーリングの NuttX 版（比較のために書いた。Z01 の確認のあと FreeRTOS 版へ戻す）。

## Z01（Zephyr 版の NINA を共通コアへ）と READY の割り込み待ち: 実機の途中経過（2026-11-18）

- ユーザーの決定: NuttX 版の READY 割り込みは残す。Zephyr 版ができたら書く。
- 実装（subagent、**未 commit**）: `nina_wifi.c`・`nina_dns.c` を削除し共通の `nina_spi.c`・`nina_dns.c` に、`nina_socket.c` を共通の `nina_sock.c` への
  オフロードの接着に書き換え。新規 `nano_nina.c`・`nina_os.c`（READY の割り込み待ちは `NINA_OS_WAIT_READY`、CMake の 2 行で戻せる）、
  POSIX の名前の対応ヘッダー 7 本、ホストテスト `nina_boundary_irq/poll`・`nina_socket`。`common/` は `NINA_OS_DNS_OFFLOAD` の任意の境界と
  コメント・ガード名。`i2c_dma_shell.c` の写しは Z02 まで残る。像 +1448 バイト、bss +613。動作の違いの一覧は subagent の報告（connect の期限の
  起点、SO_RCVTIMEO の起点、ロックの粒度、iovec の写し直し、DNS の結果の大きさ）。
- メインが確かめたこと: `common/` の変更後も NuttX の像は READY 割り込みの像と、FreeRTOS の像は変更前と一致（再ビルド）。ソースは読んでいない。
- 実機（Z01 の像、書き込み直後から）: 起動、`nina version`、KV、自動接続。送信 約 71 秒 × 3 回 `cpu_active` 51.2%・51.4%・51.5%（載せ替え前 49.0〜49.1%）、
  欠番 0、`time` のカウンター 0。zperf 送信 3.02・3.30、受信 1.78・1.89 Mbit/s（載せ替え前の同日の値は無い。2026-11-14 は 3.00・1.57）。
  `ntp status` の `last_us` 4.8〜6.6 ms（1 点 13.9 ms）。`wifi disconnect` → `auto_connect` で復帰。`dmesg` に `<err>`・`<wrn>` なし。
  スタック: `imu_raw_mqtt_tid` 1284/2048、`shell_uart` 1972/3072、`nina_scan_tid` 912/1536、ほか 60% 未満。
  送信中の Telnet 3 回 OK（drop 0）。送信中の `wifi scan`: drop 395/905、overruns 1、`publish_max_us` 3.3 秒、failures 8（載せ替え前の
  Zephyr 版で同じ試験をしていないので、変化かどうかは不明）。
- **見つかった不具合**: 書き込み直後の起動で、zperf を 2 往復したあと Telnet に繋がらなくなった（connect がタイムアウト、ping は通る、
  ログなし、`shell_telnet` のスタック未使用）。再起動で復帰。subagent の分析: `zperf tcp download stop` の close と、Zephyr の
  `net_socket_service` スレッドの poll が重なると `do_poll()` が -EBADF で全体を失敗させ、スレッドが `poll failed` で終わる
  （`CONFIG_NET_SOCKETS_LOG_LEVEL=0` でログも出ない）。メインが実機で再現: 接続中に `download stop` を繰り返すと 6 回目で
  `kernel thread list` から `net_socket_service` が消え、Telnet がタイムアウト。旧ドライバーにも同じ窓がある（-EIO）というのは subagent の
  読みで、実機では未確認。2026-11-12 の「Telnet を短い間隔で繋ぎ直すと繋がらなくなる」も同じ経路の可能性（未確認）。
  修正（subagent が実装中）: 消えたソケットは `POLLNVAL`、1 つのソケットの poll の失敗は全体を失敗させず `POLLERR`。
- 未解明: `cpu_active` の +2 ポイント。subagent の仮説は、READY 割り込みで往復が短くなり、送信バッファ満杯のときの全量の送り直しが
  増える。切り分けは同じソースのポーリングの像（`ready/zephyr-poll.uf2`）との比較（BOOTSEL の操作が要る）。
- 実機: Z01 の像（修正前）。

## Z01 の修正と確認、commit（2026-11-18）

- ユーザーの決定: 負荷の +2 ポイントの切り分けは省く（原因未解明のまま残す）。
- 修正（subagent）: Zephyr の `nina_socket.c` の `do_poll()`。表から消えたソケット（-EBADF）は `POLLNVAL`、1 つのソケットの poll の失敗は
  そのソケットだけ `POLLERR`、呼び出し全体は失敗させない。ホストテスト 2 件追加（修正前のコードで失敗することを確認、subagent）。
  ドライバー説明（日英）に 1 項目。利用者（Telnet、zperf、MQTT）はエラーの fd を閉じるか作り直すので空回りしない（subagent の読み）。
  FreeRTOS 版・NuttX 版には、待ち受けが永久に止まる同じ経路は無い（subagent が読んだだけ）。
- 実機（修正入りの Z01 の像、書き込み直後から）: 送信 約 71 秒 `cpu_active` 51.3%、欠番 0。接続中に `zperf tcp download stop` を 20 回
  繰り返しても `net_socket_service` が残り、Telnet に繋がる（修正前は 6 回目で消えた）。送信中の Telnet 3 回 OK、drop 0、overruns 0。
  `dmesg` に `<err>`・`<wrn>` なし。
- **新しい基準**: `freertos/build/baseline/zephyr/snapshot-z01/image.bin`、`.../nuttx/snapshot-ready/image.bin`。
- 未確認・未解明: `cpu_active` の +2 ポイント（49.0〜49.1% → 51.2〜51.5%）、送信中の `wifi scan` での drop とオーバーラン 1 回が載せ替え前から
  あるか、AP を落とす切断復帰、輻輳（`--drop-at`・`--stall-at`）、長時間。
- README に入れること（未反映、READY が 3 版に入ったのでまとめて行う）: ルートの性能の節、`zephyr/README`・`nuttx/README` のオプションの 1 行、
  NINA が共通コアになったこと。`common/compat/nano_gpio.h` の古いコメントも未修正。
- 実機: 修正入りの Zephyr 版（FreeRTOS 版へ戻すには RESET 2 回）。

## ユーザーの決定と残件（2026-11-18）

- 文書をいったん整理する（subagent が README・設計書・ドライバー説明を作業中。`freertos/README` はメイン）。
- Zephyr 版の負荷の +2 ポイント: 深追いしない。
- 送信中の `wifi scan` でバッチが落ちる件: 共有の `nina_net.c` の `wifi_scan()` は、接続中ならスキャンの前に「切断」の扱いにする
  （`connected = false`、IP 設定を消す、切断のイベントを上げる。スキャン後に `scan_reconnect` で繋ぎ直す）。3 版共通の作りで、
  スキャン中に送信が止まってバッチが落ちるのはこのため（メインがソースで確認）。
- AP を落とす切断からの復帰・受信側の輻輳・長時間: 最終版で 3 版まとめて確かめる（最後に回す）。
- 追加の作業: (1) FreeRTOS 版で割り込み停止中に tick が進まない件の修正（subagent が実装中）。(2) Zephyr 版・NuttX 版の除算器対応
  （方式の確認待ち: 割り算の間だけ割り込み禁止か、上流へのローカルのパッチで切り替え時に退避か）。(3) NINA のファームウェアを直して
  受信データの到着を割り込みで知らせる（subagent が調査と計画づくり中。ファームウェアの書き換えは計画をユーザーに相談してから）。
- 実機: 最終の FreeRTOS 版（既定のビルド）に戻した。

## 文書の整理（2026-11-18）

- subagent が実施（コードは `common/compat/nano_gpio.h` のコメントだけ）: ルートの README（日英、約 70 行減）は性能の 2 節を
  「3 つの版の性能」の 1 節に統合し今日の値に、`CONFIG_NANO_ROM_MEM_OPS` の 1 文、構成の表（NINA のコアは 3 版）。2026-11-14 の
  「スレッドごとの CPU 時間」「メモリの使い方」「NTP の往復の比較」は今の像の値でないので削除、`sensor stream` の上限は日付を付けて残した。
  `nuttx/README`（オプションの 1 文、テストの本数）。設計書 §8・§11.7・§12・§13.2・§13.6（今日確かめたことと、最終の像では未確認のもの）。
  `nuttx-porting`、`nuttx/drivers/README`、`freertos/drivers/README` の READY・NINA の記述。`zephyr/` 直下に README は無い。
- メインが読んだのはルートの README（日）の性能の節だけ。`freertos/README` は直す点なし（subagent の確認）。
- subagent が見つけて直さなかった点: boot2 の分周 2 を書いた文書が `nuttx-porting` 以外に無い、`nuttx-porting` のホストテストの表に
  `rp2040_rom` の行が無い。boot2 と `memcpy`・`memset` の入れ方をパッチに変える話が進行中なので、それが決まってから直す。
- ユーザーの決定（同日）: 除算器は方式 B（切り替え時に退避、上流へのローカルのパッチ）。Zephyr 版の boot2 もパッチにする。
  `memcpy`・`memset` もパッチにできないかの質問あり（3 案を提示、返事待ち）。スキャン中の接続維持を NINA のファームウェアの計画に追加。

## NINA のファームウェアの計画: 受信データの割り込みと、スキャン中の接続維持（2026-11-18、subagent の調査。実装は未着手）

- 事実（ソース）: IRQ の線（ESP32 GPIO0 = RP2040 GPIO2）は BSD 風の命令（0x70〜0x7f）のソケットを反映しない（`updateGpio0Pin()`、
  `CommandHandler.cpp:3024-3064`）。`scanNetworks()`（`WiFi.cpp:531-570`）は最初に `esp_wifi_stop()`／`start()` を呼ぶので接続が切れ、
  接続済みのソケットは `handleWiFiDisconnect()` で閉じられる。0x27 はスキャンが終わるまでブロックし、0x36 は何もしない。
  ドライバーの 20 ms の poll: `NINA_POLL_INTERVAL_MS`（`nina_sockopt.h:44`）、利用箇所は Zephyr・FreeRTOS の `nina_socket.c`、NuttX の
  `nina_poller.c` ほか。NTP と MQTT の受信は 20 ms の poll に依らない。送信の再試行は 1 ms ごとに全量（最大 3584 バイト）を送り直す。
- 実機（メイン、FreeRTOS 版、静止時）: `devmem d0000004` の bit 2（GPIO2）は 20 回とも 0。
- 案（subagent の推奨）: 新しい命令 0x6A（capabilities）で機能を検出し、古いファームウェアでは今の動作に戻る。パッチは 4 つに分ける:
  0003 capabilities、0004 スキャン中の接続維持（接続中は stop/start を省く、`home_chan_dwell_time` 30、`_status` を保つ。約 20 行）、
  0005 非同期のスキャン（0x36 を開始だけにし、状態の命令を足す。任意。0x27 の間 NINA のロックで送信が数秒止まるのを避ける）、
  0006 受信の割り込み（案 A: 待っているソケットだけを one-shot で登録し、見張りのタスクが `select()` で待って線を上げる。次の命令の
  先頭で下げる。約 150〜200 行）。ドライバーは `nina_os_event_seq/wait`（複数の待ち手へのブロードキャスト）を 3 版に。
- 効果の見込み（推測）: TCP の受信は今「受信窓 5744 バイト ÷ 20 ms = 2.30 Mbit/s」で頭打ち（FreeRTOS の 2.31 と一致）に見える。割り込みで
  この上限が外れる。NTP の往復はほぼ変わらない。CPU は分からない。
- ファームウェアを変えずに試せること: (a) poll 間隔を一時的に 5 ms にして受信速度が伸びるか（上の仮説の確認）、(b) 送信が 0 だったあと
  全量を送り直す前に poll で書き込み可能を確かめる（空振りの転送を減らす）。
- 書き込み: この PC に docker の `espressif/idf:v4.4.8`、esptool、arduino-cli、SerialNINAPassthrough がある（確認だけ）。手順は KV の退避 →
  パススルーのスケッチ → NINA の flash の退避（認証情報を含みうる、非公開）→ 書き込み → RP2040 を戻す → KV の復元（Wi-Fi のパスワードは
  ユーザーが入力）。ユーザーの BOOTSEL 2 回以上。未確認: select 中の close の動作、接続中のスキャンで TCP が保たれるか、パススルーが KV を消すか。
- ユーザーへの相談待ち（ファームウェアの書き換えは承認のあと）。

## FreeRTOS 版: 割り込み禁止のあと tick が TIMER に追いつく（2026-11-18）

- 実装（subagent）: tick フック（`tick.c`、判定は新規 `tick_catchup.h`）が tick ごとに TIMER と比べ、1 tick 以上遅れていれば SysTick を
  保留にし直す。失われた tick はポートの通常のハンドラーが 1 回ずつ数え、tick フック（互換層のタイムアウト、時分割）も各 tick で呼ばれる。
  `xTaskCatchUpTicks()` は使わない（タスクからしか呼べず、補った tick で tick フックが呼ばれない）。tick は TIMER より先に進まない
  （1 µs の余裕。まれに 1 tick 遅れたままの 1 ms 未満の窓で始めた待ちが最大 1 tick 短くなり得る、subagent の説明）。`main.c` に
  clk_sys が tick の整数倍であることの検査。`selftest` に `tick_catchup`（割り込みを 50 ms 止める）で 22 項目、ホストテスト `tick`。
  カーネル・Pico SDK は無変更。F04・CP-F2 の「消去のたびに tick が遅れる、補正は未決」はこれで決着。
- メインはソースを読んでいない。ビルド・ホストテストは subagent の報告。
- 実機: `selftest` 2 回とも 0 of 22（`masked 50000 us: 50 ticks in 50316 us`、解除後のタイムアウト 27・40 µs、sleep の起床 126・134 µs）。
  既定の像で `uptime` の「TIMER の ms − tick」は 2.5〜2.9、100 文字の `kv set` 90 回のあと 2.8〜3.1（修正前は F04 で 90 回に 2256 tick の遅れ）。
  書き込み直後の送信 60 秒: `cpu_active` 41.4%、欠番 0、overruns 0。異常停止の記録なし。試験のキー `test.tick` は削除した。
  未確認: 計測中の `kv set`。
- 文書（未反映）: `freertos/drivers/README` の tick・KV・自己診断・ホストテストの表・比較表、設計書 §13 の tick の項（subagent の報告に文案）。
- ユーザーの決定（同日）: `memcpy`・`memset` は全部パッチにする（案 1。`common/drivers/rp2040_rom/` とホストテストは削除）。NINA の
  ファームウェアは 4 つのパッチの案で進める。問い合わせ間隔 5 ms の測定はしない。送信のやり直しの改善は、ファームウェアと別に改善の
  余地があれば正式版に向けて入れる。KV は退避済みなので必要なら復元すればよい。NINA のファームウェアの転送を、専用の Arduino の
  スケッチではなくコマンドでできるようにしたい（ユーザーの意向）。

## 除算器（方式 B）、パッチの仕組み、NINA のファームウェアのパッチ（2026-11-18）

- ユーザーの決定: 0 除算の結果は Pico SDK のルーチンのまま（32 ビットは飽和した商・余り 0。3 版でそろえる）。NINA のファームウェアは
  4 つのパッチで進める。`memcpy`・`memset` は全部パッチ。
- **Zephyr 版**（subagent、commit 済み）: `west patch`（`zephyr/manifest/zephyr/patches.yml`・`patches/`）+ `setup.sh` の `_zephyr_patch`
  （当てた集合の写しを `.west/nano-patches` に持ち、集合が変わったら戻して当て直す。commit は作らない）。パッチ 3 つ: hal_rpi_pico の boot2 の
  分周 2（アプリの `nano_boot2` を削除。この時点の像は `snapshot-z01` と一致）、`swap_helper.S`（PendSV）での除算器の退避
  （`CONFIG_ARM_RP2040_DIVIDER_SAVE`、`struct _thread_arch` に 20 B、切り替え 1 回で約 20〜26 命令、DIRTY のときだけ 4 レジスターを退避）、
  SoC の ROM の `memcpy`・`memset`（`CONFIG_RP2040_ROM_MEM_OPS`、`--wrap` は SoC の CMake から。アプリの `NANO_ROM_MEM_OPS` と
  `src/rom_mem_early.c` を削除）。アプリの `CONFIG_NANO_HW_DIVIDER`（既定 y）が SDK の `divider_hardware.S` を 6 記号に `--wrap`。
  既定の像 364396 B、libgcc の除算は像に残らない。
- **NuttX 版**（subagent、commit 済み）: パッチ `nuttx/0009`（`RP2040_HW_DIVIDER`、`struct xcptcontext` に 20 B、`arm_doirq()` の切り替えで
  退避・復元、`arm_sigdeliver` の扱い）、ボードの `NANO_HW_DIVIDER`（既定 y）、pico-sdk の sparse checkout に除算器の 2 ファイルを追加
  （blob の遅延取得でネットワークを使う）、`build.sh` の検査。無効の像は `snapshot-ready` と一致。有効の像 279092 B（−472）。
  subagent が足した 0 除算を libgcc にそろえる pico-sdk のパッチは、ユーザーの決定により外す（続きの作業で、ROM の `memcpy` のパッチ化と
  `common/drivers/rp2040_rom/` の削除と一緒に依頼中）。
- **試験の像**（commit しない、`scratchpad/div/`）: Zephyr「退避なし」81 秒で食い違い 30（最低優先度の 3 スレッドだけ、割り込みと高優先度は 0）、
  「退避あり」361 秒・約 4,070 万回で食い違い 0。NuttX の試験の像は未実行。
- **NINA のファームウェア**（subagent、commit 済み、**書き込みはしていない**）: パッチ 0003（0x6A capabilities）、0004（スキャン中の接続維持）、
  0005（非同期のスキャン: 0x36 開始、0x6B 状態、0x27 結果）、0006（ソケットの事象を IRQ の線で。見張りのタスク、one-shot の登録、次の命令の
  先頭で線を下げる、最短 5 µs）。`build.sh` が足りないパッチだけ当てる。docker でビルド済み（像 `source/NINA_W102.bin`、sha256 b04f0796…、
  アプリ +5,040 バイト）。元の 2 パッチの像は `firmware/nina-fw/images/`（Git 管理外）に退避、sha256 ee0f213b…。ドライバー向けの仕様は
  `firmware/nina-fw/README.md` の「Host interface of 0003 to 0006」。実機では何も確かめていない。
- メインはどのソースも読んでいない。ビルド・テストは各 subagent の報告。
- 実機: Zephyr の「退避あり」の試験の像。

## NuttX の除算器の試験の途中経過、中継コマンド（2026-11-18）

- NuttX「退避なし」の試験の像（実機、起動から 40 秒）: `a 4/686160 b 6/694544 h 0/289512 sig 0/0 isr 0/144756`。食い違いは a・b だけ。
- **試験コードの問題**: どちらの試験の像でも空きメモリー（`/proc/meminfo` の Umem の free）が約 2.3 KB/秒で減り、55〜60 秒で使い切って
  コマンドが実行できなくなる（`command not found`・`out of memory`）。`sig` は 0/0 のまま（シグナルの経路が通っていない）。退避ありの像でも
  同じ速さで減った。退避ありの像の食い違いの数は読めていない（新しい起動の行を確認できなかった）。subagent に試験の修正を依頼中。
  正式の変更に由来しないことの説明も依頼。**NuttX の退避ありの確認は未了**。
- 反省: これまで NuttX 版で `dmesg` のエラー行を「数えて 0」としていた確認は、コマンドが実行できなかった場合と区別できない。今日の
  正常な像（boot2・ROM・READY の各確認）ではコマンドは動いていたはずだが、次に確かめるときはログの中身を見る。
- FreeRTOS 版の `nina passthrough confirm`（subagent、commit 済み、**実機未確認**）: watchdog の scratch に印を付けて再起動し、スケジューラーも
  SPI も起こさずに USB CDC ⇄ UART1（GPIO8 = TX、GPIO9 = RX。SPI の MISO・CS と同じピン）を中継する。RTS → RESETN、DTR → GPIO0、
  line coding のレートに追従、16 KiB の受信リング。出るのは RESET か「300 bps にして閉じる」。フラッシュには書かない。ホストテスト
  `passthrough`。配線は mbed のボード定義から（回路図は未確認）。README への反映は未了。
- 手順の案（ユーザーに提示）: FreeRTOS 版 → `nina passthrough confirm` → `read_flash` で NINA を退避（まず 115200、次に高速で読み直して
  一致を見る）→ 了承を得てから `write_flash`。
- 実機: NuttX の「退避あり」の試験の像（メモリーを使い切った状態）。

## NuttX の続き、中継コマンドの実機確認、NINA の退避（2026-11-18）

- NuttX 版（subagent、commit 済み）: pico-sdk の 0 除算のパッチと、pico-sdk にパッチを当てる仕組みを削除（0 除算は SDK の結果、3 版で同じ）。
  ROM の `memcpy`・`memset` はパッチ `nuttx/0010`（`RP2040_ROM_MEM_OPS`、チップのディレクトリーに 3 ファイル、`rp2040_start.c` のループに
  `START_LOOP_BARRIER()`＝メモリーを clobber する空の asm）。ボードのフラグの行と VPATH の差し込みを削除、`build.sh` の `__start` の検査は残す。
  `common/drivers/rp2040_rom/` と `common/tests/rp2040_rom_mem.c` を削除（Zephyr 版も自分のパッチへ移行済み）。既定の像 279108 B。
- 除算器の試験コードの不具合の原因（subagent）: `kill()` が NuttX ではカーネルスレッドのグループ全体へのシグナルになり、走れないスレッドに
  保留が溜まっていた（配送先が idle というのは推定）。`pthread_kill()` に直し、割り算の途中のタスクへ配送される場合を作った。出力に起動の行、
  方式、free を追加。正式の変更は確保・解放をしない（subagent の説明）。直した試験の像は**実機未実行**。
- **`nina passthrough confirm` の実機確認（FreeRTOS 版）**: 通常動作は変わらず（書き込み直後の送信 60 秒 41.6%、欠番 0）。確認の語なしは拒否。
  モードへ入り、300 bps で閉じると通常の起動に戻り、`nina version`・自動接続・KV とも正常。
  - esptool 5.3.0 の `--before no-reset` では繋がらなかった（応答なし）。pyserial で直接開く（DTR・RTS とも有効）と、ROM の
    `waiting for download` の表示と同期の応答が返った。esptool には設定ファイル（`ESPTOOL_CFGFILE`）で
    `custom_reset_sequence = R0|D1|W0.2|R1|W0.6` を与え、`--before default-reset --after no-reset` で繋がった（RTS を有効にしたままに
    する必要がある。esptool 5.3 が no-reset で線をどう扱うかは未確認）。README に書くときはこの手順にする。
  - **NINA の flash の退避**: `read-flash 0 0x200000` を 115200 bps（191 秒）と 921600 bps（26 秒）で行い、2 つは一致。中継の記録は
    約 423 万バイトを受けて取りこぼし 0、オーバーラン 0。置き場所 `firmware/nina-fw/images/backup-20261118-115200.bin`・`-921600.bin`
    （Git 管理外、認証情報を含みうる。sha256 の先頭 1da388c18bbf7090）。
  - 退避した像のアプリの領域は、`images/NINA_W102_ALL-3.0.2-p0002.bin`（11-10 にビルドした 2 パッチの像）と大半のブロックが違う。
    実機に入っているのはそのビルドそのものではない（何が入っているかは未調査）。復元には今日の退避を使う。
- 実機: FreeRTOS 版（中継コマンド入り）、AP 接続中。NINA は未変更。

## NINA のファームウェアを書き換えた（2026-11-18、ユーザーの了承のうえ）

- 書いた像: `firmware/nina-fw/source/NINA_W102.bin`（sha256 b04f0796…、パッチ 0001〜0006。ソースで 6 つとも当たっていることを確認: 0003〜0006 は
  写しの上で後ろから順に戻せた、0001・0002 は逆適用の検査が通る）。FreeRTOS 版の `nina passthrough confirm` と esptool 5.3.0
  （`ESPTOOL_CFGFILE` の `custom_reset_sequence = R0|D1|W0.2|R1|W0.6`、`--before default-reset --after no-reset --baud 921600
  write-flash 0`）で 19.9 秒、esptool のハッシュ検証 OK。
- 読み戻し（`images/after-write-20261118.bin`、Git 管理外）: 先頭 0x134c00 バイトが像と一致。像の終わり〜0x1B0000、storage（0x1B0000〜）、
  nvs（0x1F0000〜）は書き込み前の退避と一致（保存領域は変わっていない）。
- 元へ戻すとき: `images/backup-20261118-115200.bin`（2 MB 全体）を `write-flash 0`。
- **今のドライバーのまま（FreeRTOS 版、未対応）での確認**: 起動、`nina version` 3.0.2、自動接続。送信 60 秒 `cpu_active` 41.4%、欠番 0、overruns 0。
  静止 1.2%。zperf 送信 3.76・受信 2.52 Mbit/s。`ntp status` の `last_us` 5842。送信 + zperf + Telnet の同時実行: drop 521/1440（従来どおり）、
  overruns 0、zperf 3.28 Mbit/s。Telnet 3 回 OK。`wifi disconnect` → `auto_connect` は 3 秒で復帰。`dmesg` に `<err>`・`<wrn>` なし、異常停止なし。
  GPIO2 は静止時 0。
- **気づいた点**: `wifi scan` のあと Wi-Fi が RUNNING に戻るまで約 44 秒（2 回とも）。新しいファームウェアはスキャン中も接続を保つが、今の
  ドライバーはスキャンを切断の扱いにして繋ぎ直すので、その組み合わせの結果とみられる（書き換え前の所要時間は測っていない）。ドライバーの
  対応（機能のビットを見て切断の扱いをやめる）で解消する見込み。それまでスキャンは使えるが復帰が遅い。
- 実機: FreeRTOS 版（中継コマンド入り）、NINA は新しいファームウェア。

## NuttX の除算器の試験: シグナルで固まる件（2026-11-18）

- 直した試験の像（`pthread_kill` でシグナルが届く版、退避あり）は、実機で試験の開始（起動 5.8 秒）の直後に毎回固まった。シリアルは列挙された
  ままで入力を受けず、PANIC もリセットも無し。RESET 1 回のあと RAM ログを読むと `divtest: start (switch save)` の行のあと何も無かった。
- subagent の分析（コードの上、実機未確認）: 上流の NuttX の armv6-m の不具合。`arm_doirq.c` の PendSV の分岐
  `if (tcb->sigdeliver) up_schedule_sigaction(tcb)` が、シグナルハンドラーの実行中に PendSV が来ると配送のフレームをもう一度作り、
  `xcp.saved_regs` を上書きする。除算器のパッチ 0009 とは無関係（0009 はフレームにもスタックポインターにも触れない）。正式の像は
  ハンドラーを登録するコードが無い（`sigaction`・`kill`・`pthread_kill` なし、SIGCHLD・SIGINT も生成・配送されない）ので、通常の動作では
  この経路を通らない、という判断。修正の候補（`saved_regs` を NULL に戻し、NULL のときだけフレームを作る）は scratch の
  `nuttx-0011-candidate-armv6m-sigdeliver-pendsv.patch`。リポジトリには入れていない。
- 切り分けの像（`scratchpad/div/`、watchdog 3 秒で固まりを検出し、次の起動で前回の記録を出す）: `nuttx-divtest-nosig(-nosave)`、
  `nuttx-divtest(-nosave)`（0011 の候補あり）、`nuttx-divtest-sig-nofix`、`nuttx-divtest-libgcc-sig-nofix`。
- **実機（`nuttx-divtest-nosig.uf2`: 除算器 + 退避あり、シグナルなし）**: 250 秒・約 1,128 万回で mismatches 0、free 114016 で一定
  （1 行だけ 109872）。退避なしの像は、前の版の試験で 40 秒に a・b 合わせて 10 件の食い違い。**切り替えでの退避は NuttX でも正しく動く**。
- 未確認: シグナルの経路（0011 の候補ありの像で固まらず食い違い 0 か）、libgcc + シグナル + 修正なしの像でも固まるか（上流の不具合で
  あることの確認）。0011 を正式に入れるかは未決。
- 実機: NuttX の `nuttx-divtest-nosig` の試験の像。

## NINA のドライバーを新しいファームウェアに対応（2026-11-18、commit 済み、**3 版とも実機未確認**）

- 実装（subagent）: (1) 機能の検出 `nina_probe_capabilities()`（0x6A、未定義の命令の `0xEF 00 EE` は `-ENOTSUP`。`iface_init()` の MAC の
  読み出しの直後に 1 回）、`nina version` に `NINA capabilities: 0x%08x (in use 0x%08x)` の 1 行。(2) スキャン: ビット 0 で接続中の
  スキャンを切断の扱いにしない、ビット 1 で 0x36 → 0x6B を 100 ms ごと → 0x27（期限 10 秒）。(3) ソケットの事象: OS 境界
  `nina_os_event_start/seq/wait`（`NINA_OS_EVENTS`、GPIO2 の立ち上がりで世代番号を進め全員を起こす）、共通の待ち `nina_wait.[ch]`
  （命令の前に印、最長 100 ms、空振りが続けば 0〜20 ms の休み）。3 版の connect・send・recv・poll の待ちと NuttX の poller を置き換え。
  `time_sync.c` の 1 ms の試みと MQTT の `poll(0)` は替えていない。(4) 送信の再試行: 続きのパスは先に 0x7C で書き込み可能を確かめる
  （古いファームウェアでも有効）。ホストテスト多数、わざと壊した版 14 通りで失敗を確認（NuttX の poller のエッジはテストなし）。
- 像の大きさ: Zephyr 365756 B、NuttX 281164 B。試験用の像は `scratchpad/drv/`（`zephyr.uf2`・`nuttx.uf2`・`freertos.uf2`・
  `freertos-noevent.uf2` = 事象を使わない比較用）。
- メインはソースを読んでいない。ビルド・テストは subagent の報告。実機で見ること: `nina version` の 2 行目（0x7）、接続中の `wifi scan`
  （RUNNING のまま、数秒）、スキャン中の送信、zperf の受信、送信中の `cpu_active`（noevent と比較）、Telnet、切断復帰、GPIO2。
- README（未反映）: 接続中のスキャン、`nina version` の 2 行目、事象を使わないビルドの方法、`nina passthrough confirm` と esptool の手順、
  除算器・パッチの仕組み・tick の追いつき。実機確認のあとにまとめて直す。

## 新しいドライバーの実機確認（FreeRTOS 版）と、spike-nx の Issue #26（2026-11-18）

- ユーザーの指示: A（FreeRTOS 版の新しいドライバー）→ B（NuttX の固まる件）の順。spike-nx（owhinata/spike-nx）の過去の Issue に同じ
  問題が無かったか調べる（close 済みも含めた調査を subagent に依頼中）。
- **FreeRTOS 版（新しいドライバー、新しいファームウェア）**: `nina version` の 2 行目 `NINA capabilities: 0x00000007 (in use 0x00000007)`。
  書き込み直後の送信 60 秒 `cpu_active` 40.4%、欠番 0。

  | 項目 | 事象あり（既定） | 事象なし（比較用の像） |
  |---|---|---|
  | 送信 60 秒の `cpu_active` | 40.4%、40.3% | 42.3% |
  | 静止 30 秒 | 0.7%（3 回） | 1.3%、1.2% |
  | zperf 送信（Mbit/s） | 3.47、3.84、3.70 | 3.62、3.60 |
  | zperf 受信（Mbit/s） | 2.41、2.52、2.52 | 2.41、2.52 |

  - 受信速度は事象の有無で変わらない（「20 ms の poll が上限」という見立ては外れ）。負荷は送信中 約 2 ポイント、静止時 約 0.5 ポイント下がった。
  - 接続中の `wifi scan`（静止時）: 3.5 秒で結果、RUNNING のまま（前は復帰まで 約 44 秒）。
  - 送信中に 2 回スキャン: 接続は保たれ（connects 1、failures 0）、drop 109/1117、overruns 0、受信側の最長の間隔 1.48 秒
    （載せ替え前の Zephyr は 1 回のスキャンで drop 395/905・再接続）。スキャン中は無線がチャネルを離れるので送信が滞り、キューがあふれる分が落ちる。
  - 送信 + zperf + Telnet: drop 468/1443（従来と同程度）、overruns 0。Telnet 3 回 OK。GPIO2 は静止時 0。`dmesg` に `<err>`・`<wrn>` なし。
- **spike-nx の Issue #26**（open、STM32F413・ARMv7-M・BUILD_PROTECTED、`ostest` の `signest_test` で固まる）: シグナルハンドラーの実行中に
  `saved_regs` の領域が上書きされ、復帰で PC=0 に飛ぶ、という原因まで確定し、修正の試みは 15 回とも失敗して未解決（テストを skip）。
  今回の NuttX 版（ARMv6-M、FLAT）の固まりと同じ種類（ハンドラーの実行中の割り込み・切り替えで `saved_regs` が壊れる）。経路が同じかは未確認。
- 実機: FreeRTOS 版（新しいドライバー、事象あり）。

## NuttX のシグナルの固まり: 除算器と無関係であることを実機で確認、spike-nx の調査（2026-11-18）

- **実機（`nuttx-divtest-libgcc-sig-nofix.uf2`: libgcc の割り算、除算器のパッチの処理なし、シグナルあり、修正の候補なし）**: 試験の開始から
  1.77 秒（シグナル 221 回を配送したあと）で止まり、watchdog（3 秒）がリセットした。記録: `previous boot 2: test running, reset reason
  0x1 (watchdog), ms 1765, last feed at ms 1746, signals 221/221`。**固まりは除算器（パッチ 0009）と無関係**で、NuttX 本体の側にある。
  （1 回目は、前の試験の `.noinit` の記録と書き込み時の watchdog リセットを「前回固まった」と取り違えて試験を始めなかった。`reboot` で実行。）
- 修正の候補（scratch の `nuttx-0011-candidate-…`）ありの像は実行していない。
- **spike-nx（owhinata/spike-nx）の Issue の調査**（subagent、全 160 件を検索し約 30 件を読んだ）: シグナルの仕組みそのものの不具合は #26（open、
  #38 は重複）だけ。`ostest` の `signest_test`（`pthread_kill`、優先度 101・102 の 2 スレッド）で固まる。PROTECTED ビルドでは「ハンドラーの中の
  syscall の SVC のフレームが `saved_regs` に重なって壊す」まで確定、修正の試み 15 回は全敗、テストを skip して運用。FLAT ビルドの時期
  （2026-04-07〜04-18）にも同じハングがあり、原因は未確定（「timing-sensitive な race」、計装を足し引きすると止まる位置が動く）。
  ほかは使い方の記録: #108（SIGKILL では pipe の待ちを起こせない）、#152・#166・#122（SIGINT・SIGTERM のハンドラーはフラグを立てるだけ）、
  #185（`TTY_SIGINT` 無効で Ctrl-C が効かない、open）。#100・#99・#96 などはシグナルと無関係（PendSV が走れない別の原因）。
  spike-nx の NuttX は 12.13 系のフォーク。上流での修正の有無は記録なし。
- 方針の案（ユーザーに提示）: 修正の候補は正式には入れず、「NuttX 版ではシグナルハンドラーを登録しない」ことを文書に明記する。
- 実機: NuttX の試験の像（試験は止まった状態、シェルは使える）。

## NuttX 版の正式の像の実機確認（2026-11-18）

- ユーザーの決定: NuttX のシグナルの問題は修正しない（修正の候補は入れない。「NuttX 版ではシグナルハンドラーを登録しない」を文書に明記する）。
  修正の候補ありの試験は行わない。
- 像: コミット済みのツリーをメインが再ビルド（`SOURCE_DATE_EPOCH=1763337600 ... --clean -j 1`、警告は ld の RWX の 1 件）。281164 B。
  subagent が残した `drv/nuttx.uf2` と一致。中身: boot2 分周 2、ROM の `memcpy`・`memset`（パッチ 0010）、READY 割り込み、除算器（パッチ 0009）、
  新しい NINA のドライバー。Zephyr も再ビルドして `drv/zephyr.uf2` と一致（365756 B）。
- **新しい基準**: `freertos/build/baseline/nuttx/snapshot-1118/image.bin`、`.../zephyr/snapshot-1118/image.bin`。
- 実機（NuttX、書き込み直後から）: 起動、`nina version`（capabilities 0x7、in use 0x7）、KV、自動接続。送信 約 70 秒 × 2 回 `cpu_active`
  49.4%・49.4%（ドライバーの対応前は 55.8〜56.2%）、欠番 0、overruns 0。`iperf` 送信 3.46・3.36、受信 1.18・1.18 Mbit/s。Telnet 3 回 OK。
  `dmesg` は 9 行で `<err>`・`<wrn>`・fail・assert なし（今回は中身を見た）。空きメモリー 122264。NuttX 版の `wifi` には `scan` が無い
  （スキャンの確認はしていない）。
- 実機: NuttX の正式の像。

## Zephyr 版の正式の像の実機確認: 退行が 1 件（2026-11-18）

- ユーザーの要望: `bootsel` と NINA の中継コマンド（`nina passthrough confirm`）を、正式の機能として NuttX 版と Zephyr 版にも入れる
  （subagent 2 つが作業中。`nina_passthrough_logic.h` は Zephyr の担当が `common/drivers/nina/` へ移す）。
- 像: コミット済みのツリーから再ビルド、365756 B（`snapshot-1118`）。boot2 分周 2・ROM の `memcpy`・除算器（いずれもパッチ）、NINA は共通コア、
  新しいドライバー。
- 実機（書き込み直後から）: 起動、`nina version`（capabilities 0x7、in use 0x7）、KV、自動接続。送信 約 71 秒 × 2 回 `cpu_active` 48.6%・48.7%
  （Z01 の時点 51.2〜51.5%）、欠番 0、`time` のカウンター 0。zperf 送信 3.36・3.47、受信 1.99・1.89 Mbit/s。`ntp status` の `last_us` 5.4〜12.8 ms。
  送信中に Telnet 4 回と `wifi scan` 1 回: Telnet 4/4、drop 57/1886（Z01 の時点は 395/905）、overruns 1。`wifi disconnect` → `auto_connect` で復帰。
  `dmesg` に `<err>`・`<wrn>` なし。スタック: `sysworkq` 792/1024（77%、以前 480）、`imu_raw_mqtt_tid` 1332/2048、`shell_telnet` 1584/3072。
- **退行**: `zperf tcp download` → PC の `iperf3 -c … -t 6` の接続中に `zperf tcp download stop` を打つと、最初の 1 回で USB のシェルが応答しなく
  なった（エコーも返らない。USB の書き込みは受け付ける）。そのあと Telnet も connect がタイムアウト。ping は通る。リセットは起きない。
  Z01 の `do_poll()` の修正の時点では同じ手順が 20 周通っていたので、ドライバーの対応（事象の待ち・送信前の poll）で入った退行とみられる。
  ドライバーの担当の subagent に原因の特定と修正を依頼中。**Zephyr 版の新しいドライバーは、この件が直るまで未完了**。
  FreeRTOS 版は zperf の実装が別（自前）で、同じ手順は試していない。
- 実機: Zephyr の正式の像（USB のシェルが止まった状態）。

## Zephyr の固まりの原因と対処、`bootsel` と中継コマンドを Zephyr・NuttX に（2026-11-18、commit 済み）

- **Zephyr の固まり（`zperf tcp download stop` を接続中に打つ）**: 原因は Zephyr 4.5.0 本体（subagent の分析）。`srv_disable()` が `srv_lock` を
  持ったまま `tcp_download_cb` でシェルに `TCP session error.` を印字 → USB のシェルの送信完了は CDC ACM のコールバック（既定では system
  work queue）待ち → 同じキューで zperf の 1 秒ごとの `idle_work_handler()` が `srv_lock` を待つ。実機のスレッドの状態（`shell_uart` と
  `sysworkq` がともに優先度 -1 で pending、`net_socket_service` は動く、idle 93%）と一致。NINA のドライバーのロックは輪に入らない。
  以前 20 周通ったのに今回 1 周目で出る理由は未特定。
- **対処**: `prj.conf` に `CONFIG_USBD_CDC_ACM_WORKQUEUE=y`（CDC ACM を専用のワークキューで）。この設定だけを足した像で同じ手順を 20 周、
  毎回 `TCP session error.` が出て USB・Telnet とも生きたまま、`dmesg` に `<err>`・`<wrn>` なし。スタック: `cdc_acm_work_q` 340/1024、
  `sysworkq` 792/1024。
  - **気になる点**: この像の送信中の `cpu_active` は 51.0%・51.1%・51.1%（設定なしの正式の像は 48.6%・48.7%）。約 2.4 ポイント高い。
    設定によるものか、像の配置によるものかは未確認。
- **`bootsel` と `nina passthrough confirm`（Zephyr・NuttX、subagent、実機未確認）**: 共通部分 `nina_passthrough_logic.h` を
  `common/drivers/nina/` へ移動（FreeRTOS の像は移動の前後で一致、subagent）。どちらも「scratch の印 + watchdog で再起動 → OS は起動するが
  NINA・Wi-Fi・ソケット・シェルを起こさず、USB CDC と UART1（レジスター直接）を中継」。出口は 300 bps で閉じるか RESET。`bootsel` は ROM の
  `reset_usb_boot(0, 0)`。Zephyr: `drivers/nina/nina_passthrough.[ch]`・`_hw.h`、`CONFIG_NANO_NINA_PASSTHROUGH`、像 370144 B（+4388、bss +17.8 KB）。
  NuttX: ボードの `nano_nina_passthrough.c`・`nano_reset.c`、`apps/bootsel`、像 284900 B（+3736）。シグナルハンドラーは使っていない。
  ホストテストを両方に追加。メインはソースを読んでいない。
- 文書（未反映）: `freertos` の `nina_passthrough.c` の表示とコメントに残る `--before no_reset`（実機で誤りと分かった）を直す。
  README・CLAUDE.md の「Zephyr 版・NuttX 版にはシェルから BOOTSEL に入るコマンドが無い」を直す。
- 実機: Zephyr の `zephyr-fix`（設定だけ足した像。`bootsel` はまだ無い）。

## 3 版の最終の像の実機確認: `bootsel`・`nina passthrough` と通常動作（2026-11-18）

- **Zephyr（`snapshot-1118b`、370144 B: `bootsel`・`nina passthrough`・`CONFIG_USBD_CDC_ACM_WORKQUEUE=y` 入り）**: 起動、capabilities 0x7。
  `nina passthrough` は確認の語なしを拒否。送信 約 71 秒 × 2 回 `cpu_active` 49.4%・49.5%、欠番 0。接続中の `zperf tcp download stop` × 6 回で
  USB・Telnet とも生きたまま、`dmesg` に `<err>`・`<wrn>` なし。
  - 設定だけを足した像の 51.0〜51.1% は像の配置の影響とみられる（設定なし 48.6〜48.7%、最終 49.4〜49.5%。同じソースの系統で ±1.3 ポイント動く）。
  - **中継**: `nina passthrough confirm` → esptool 5.3.0 で `read-flash 0 0x200000` を 115200 bps（192 秒）と 921600 bps（26.5 秒）。2 つは一致、
    書き込み後の退避（`after-write-20261118.bin`）とも一致。取りこぼし 0、オーバーラン 0（リングの peak 42/16384）。300 bps で閉じて通常の起動に
    戻り、`nina version`・自動接続とも正常。中継の起動の間に `shell_telnet` の `<err>` 2 行（Cannot bind -19。NINA を起こさないため、想定どおり）。
  - **`bootsel`**: 実行すると `RPI-RP2` が現れた（ドライブのマウントまで数秒かかる）。
- **NuttX（`snapshot-1118b`、284900 B: `bootsel`・`nina passthrough` 入り）**: Zephyr の `bootsel` から書き込み。起動、capabilities 0x7、KV。
  送信 約 70 秒 × 2 回 `cpu_active` 50.8%・50.8%（1 つ前の像は 49.4%。配置の影響とみられる）、欠番 0。`iperf` 送信 3.34、受信 1.22 Mbit/s。
  - **中継**: 同じ手順で 115200 bps（192 秒）と 921600 bps（26.1 秒）。2 つは一致、書き込み後の退避とも一致。取りこぼし 0、オーバーラン 0
    （peak 21/16384）。300 bps で閉じて通常の起動に戻り、自動接続、`dmesg` に `<err>`・`<wrn>` なし（中身を見た）。
  - **`bootsel`**: `RPI-RP2` が現れた。そこから FreeRTOS 版を書いた。
- **FreeRTOS**: NuttX の `bootsel` から現在のビルド（`nina_passthrough_logic.h` の移動後）を書き込み。起動、capabilities 0x7、自動接続、異常停止なし。
- これで 3 版とも、RESET ボタンなしで互いに書き換えられる（`bootsel`）。NINA のファームウェアの読み書きも 3 版のどれからでもできる
  （`nina passthrough confirm` + esptool。`ESPTOOL_CFGFILE` の `custom_reset_sequence = R0|D1|W0.2|R1|W0.6`、`--before default-reset --after no-reset`）。
- **新しい基準**: `freertos/build/baseline/zephyr/snapshot-1118b/image.bin`、`.../nuttx/snapshot-1118b/image.bin`。
- 残り: 文書の更新（README 3 版・ルート、ドライバー説明、設計書、`CLAUDE.md` は更新済み）、FreeRTOS 版の `nina_passthrough.c` の表示に残る
  `--before no_reset` の修正、最終版での確認（AP を落とす切断復帰、受信側の輻輳、長時間。3 版とも）、Z02（I2C、着手前に相談）。
- 実機: FreeRTOS 版、AP 接続中。NINA は新しいファームウェア（パッチ 0001〜0006）。

## 文書の更新（2026-11-18 の後半の分）

- ユーザーの決定: 最終版での確認（AP を落とす切断復帰、受信側の輻輳、長時間）は今回は行わない（最終の像で性能は測ったので）。
  設計書の「未確認の範囲」に残す。
- subagent: ルートの README（性能の節を最終の像の値に、`bootsel`・`nina passthrough confirm`・esptool の手順の小節、オプション名、
  `setup.sh` がパッチを当てること。日英とも純増 31 行）、`nuttx/README`、3 版のドライバー説明（Zephyr はローカルのパッチの節に書き直し、
  NuttX は除算器・ROM と「シグナルハンドラーを使わない」＝事実だけで原因は特定していないと記載、FreeRTOS は tick の追いつき・中継・比較表）、
  設計書 §1・§8・§11・§12・§13、`nuttx-porting`、`firmware/nina-fw/README.md`。`freertos/drivers/nina/nina_passthrough.c` の表示を
  `(for esptool)` に（ビルド警告 0、`.bin` 231,872 B、ホストテスト 51 項目 ok）。
- メイン: `freertos/README`（日英）にコマンドの行（`nina version` の 2 行目、`nina passthrough confirm`）、BOOTSEL の入り方、スキャン、
  esptool の手順（日 170 → 178 行）。subagent の文書は読んでいない。
- 直していないもの: `common/drivers/nina/nina_passthrough_logic.h` と `freertos/tests/passthrough/main.c` のコメントに残る
  `--before no_reset`、`freertos/drivers/nina/nina_os.c` の古いコメント（Zephyr・NuttX は 100 µs ごと）、`openmv/README` の BOOTSEL の入り方、
  `nuttx/script/flash.sh`・`freertos/script/flash.sh` が Zephyr 版・NuttX 版に `bootsel` を送らないこと、Zephyr 版に書き込みのスクリプトが無いこと。
  Zephyr を上げる手順と `west patch reverse` は文書に書いたが実際には試していない。
- 実機: FreeRTOS 版（表示の修正の前のビルド）、AP 接続中。

## 片づけ: 古いコメント、書き込みのスクリプト（2026-11-18）

- subagent: コメントだけの修正 5 か所（`nina_passthrough_logic.h`・`freertos/tests/passthrough/main.c` の `--before no_reset`、
  `freertos/drivers/nina/nina_os.c`・`nano_nina.c` と `nuttx/drivers/nina/nina_os.c` の「100 µs ごと」）。FreeRTOS の像は前後で一致（subagent）。
  **Zephyr・NuttX はメインが再ビルドし、`snapshot-1118b` と一致**。
- 書き込みのスクリプト: 共有の `script/flash_uf2.sh`（新規）と、それを呼ぶだけの `zephyr/script/flash.sh`（新規）・`nuttx/script/flash.sh`・
  `freertos/script/flash.sh`。`RPI-RP2` があれば書く、無ければ 3 版のどれかの USB コンソールに Ctrl-C → プロンプトが返れば `bootsel`、
  応答が無い・ほかのプログラムが開いている・コンソールが 2 つ以上なら手動の入り方を表示して待つ。`--dry-run` を追加。
  文書: ルートの README・`nuttx/README`・`openmv/README`（日英）。`freertos/README`（日英）はメインが直した。
- 実機（メイン）: `--dry-run`、BOOTSEL のドライブへの直接の書き込み、FreeRTOS → NuttX、NuttX → Zephyr、Zephyr → NuttX、NuttX → FreeRTOS が
  スクリプトだけで通った（毎回 `its shell answered; sent "bootsel"`、新しいコンソールのパスを表示）。
  - 1 回だけ、Zephyr → NuttX でコピーのあと 30 秒たってもドライブが残り、書き込まれなかった（次の FreeRTOS の書き込みは成功、
    同じ組み合わせの再試行も成功）。原因は未調査。そのときは `found the RPI-RP2 drive: /dev/sda1`（マウント先の表示なし＝自動マウント前に
    見つけた経路）だった。
  - 未確認: 中継モード中・picocom で開いているときの動作（偽の環境では確認済み、subagent）、`zephyr/script/flash.sh --build`。
- ユーザーの意向: I2C（Z02）は「パッチ対応のほうがよいのでは」。調査（subagent）: 今の Zephyr 版の `i2c_dw.c` は上流 v4.5.0 に +364/−6 行、
  `i2c_dw.h` +14 行、`i2c_dw_registers.h` は上流と同じ。上流のファイルにその差分を当てた像は `snapshot-1118b` とバイト単位で一致（scratch で確認）。
  共有のコアとの間に、写しのあと片方だけに入った修正は無い。推奨は P（上流 + ローカルのパッチ、写し約 2,640 行を削除、コアとパッチのずれを
  検出するホストテストを追加）。A（共有のコアへ載せ替え）は見送り。**ユーザーの返事待ち**。
- 実機: FreeRTOS 版、AP 接続中。

## Z02: Zephyr 版の I2C0 を「上流のドライバー + ローカルのパッチ」に（2026-11-18、commit 済み）

- ユーザーの決定: 案 P で進める。`i2c_dma` コマンドは共通化する。共有のコアへの載せ替え（案 A）は行わない。
- 実装（subagent）: パッチ `zephyr/manifest/zephyr/patches/zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch`（620 行。上流の `i2c_dw.c` に 14 か所
  +130/−6、`i2c_dw.h` +14、`Kconfig.dw` +14、新規 `i2c_dw_rp2040.h`・`i2c_dw_rp2040_dma.h`）。アプリの `drivers/i2c_dw/` の 3 ファイル（約 2,640 行）と
  CMake の差し替えを削除。`CONFIG_I2C_DW_RP2040_DMA` はパッチの Kconfig へ（`prj.conf` で有効）。
  - 段階 1（パッチ化だけ）の像は `snapshot-1118b` とバイト単位で一致（メインも `zephyr-stage1.bin` の一致を確認）。
  - ずれを検出するテスト `common/tests/i2c_dw_sync.py` + `.expected`（値 135 個とアクセサー 71 個、125 MHz の SCL、DMA の定数、関数 19 対の正規化した
    差分。壊した版 15 通りで失敗を確認、subagent）。Zephyr のツリーが無ければ skip。意図した変更のあとは `--update`。
  - 段階 3: `i2c_dma` は共有の `common/drivers/i2c_dw/i2c_dma_shell.c`（パッチ側は `nano_i2c_dma_*` の 3 関数）。像 370144 → 370200 B（+56）。
    NuttX 版の `i2c_dma`（`nuttx/apps/i2c_dma`）は別の実装のまま。
  - 文書（日英）: Zephyr のドライバー説明、設計書 §11・§12、ルートの README の構成の表、ほか写しのパスの参照。
    直していない古い参照: `nuttx/apps/i2c_dma/Kconfig:12`・`i2c_dma_main.c:3`、`nuttx/boards/.../Kconfig:137`。
- メインが確かめたこと: `common/` のコメントの変更後も NuttX の像は `snapshot-1118b` と一致（再ビルド）。FreeRTOS は subagent の報告（前後で一致）。
  ソースは読んでいない。
- **実機（Zephyr の最終の像、`zephyr/script/flash.sh` で書き込み、書き込み直後から）**: 送信 約 71 秒 `cpu_active` 48.2%、欠番 0、overruns 0、
  `i2c_errors` 0、`max_transfer_us` 7812。`i2c_dma mode` → `i2c_dma=on`、`i2c_dma stats` → `dma_transfers=7717 dma_bytes=7115444 dma_errors=0
  dma_fallbacks=0`、`i2c_dma` だけでヘルプ、`i2c_dma mode x` → `Expected on or off`。`i2c_dma mode off` で送信 約 72 秒: 69.8%、欠番 0、
  DMA の転送数は増えない。`on` に戻した。`sensor get` は加速度・角速度とも妥当な値。`dmesg` に `<err>`・`<wrn>` なし。
- **新しい基準**: Zephyr は `freertos/build/baseline/zephyr/snapshot-1118c/image.bin`（370200 B）。NuttX は `snapshot-1118b` のまま。
- これで計画の P4（Z01・Z02）は完了。残り: `zephyr/script/flash.sh` などで 1 回出た書き込みの失敗の原因（未調査）、最終版での確認
  （AP を落とす切断復帰・輻輳・長時間。ユーザーの決定で今回は行わない）。
- 実機: FreeRTOS 版、AP 接続中。

## 残りの片づけ（2026-11-18、commit 済み）

- NuttX 版の `i2c_dma` を共有の `common/drivers/i2c_dw/i2c_dma_shell.c` に一本化（subagent。`sensor` と同じく `zshell.c` で動かす。
  `nuttx/apps/i2c_dma/i2c_dma_main.c` は削除）。3 版の `i2c_dma` が同じ実装になった。出力の違いは 2 つだけ（複数のサブコマンドに一致する
  ワイルドカードと 31 語以上の入力で、Zephyr のシェルのエラー文になる）。ホストテスト `i2c_dma_app` を書き直し。像 284900 → 284988 B（+88）。
- 古い参照の修正（NuttX の Kconfig 2 か所ほか）と文書（`nuttx-porting`、設計書、`nuttx/drivers/README`、日英）。`common/` の古いコメント 2 か所は
  メインが直した（FreeRTOS・Zephyr・NuttX を再ビルドし、像はそれぞれ変更前・`snapshot-1118c`・subagent の像と一致）。
- 書き込みのスクリプトで 1 回出た失敗: ユーザーが同じボードへ同時に書き込んでいた可能性が高い（ユーザーの説明。試験の開始時にボードは
  最初から BOOTSEL だった）。スクリプトの不具合としては扱わない。書き直しの処理は入れない（ユーザーの決定）。ドライブが残ったときの表示に
  「ほかから同じドライブへ書いていないか確かめて、もう一度実行する」を足しただけ。
  - 以後、こちらが `bootsel` を送っていないのに `RPI-RP2` が見えているときは、書き込む前にユーザーに確認する。
- 実機（NuttX、`nuttx/script/flash.sh` で書き込み）: 送信 約 70 秒 `cpu_active` 50.7%、欠番 0、overruns 0。`i2c_dma`（ヘルプ）、`mode`（`i2c_dma=on`）、
  `mode x`（`Expected on or off`）、`mode on off`（`mode: wrong parameter count` とヘルプ）、`stats`（`dma_errors=0 dma_fallbacks=0`）、`mode off`/`on`。
  Telnet からも同じ出力。`dmesg` は 9 行で `<err>`・`<wrn>` なし。
- **新しい基準**: NuttX は `freertos/build/baseline/nuttx/snapshot-1118c/image.bin`（284988 B）。Zephyr は `snapshot-1118c`（370200 B）。
- 実機: FreeRTOS 版、AP 接続中。

## 完了（2026-11-18、ユーザーの決定）

- **共通化と FreeRTOS 版の移植は完了とする**。P1・P2（共通部分の切り出し、KV の形式の統一）、P3（FreeRTOS 版 F01〜F15）、
  P4（Zephyr 版: NINA は共通コア、I2C は上流 + パッチ）、追加の作業（性能の差の原因と対策、NINA のファームウェア 0003〜0006 と
  ドライバー、`bootsel`・`nina passthrough`、共通の書き込みスクリプト、tick の追いつき）まで、すべて commit 済み・実機で確認済み。
- 最終の像の送信中の `cpu_active`（2026-11-18）: Zephyr 48.2〜49.5%、NuttX 50.7〜50.8%、FreeRTOS 40.3〜40.4%。
- **最終の像では確かめていないもの**（ユーザーの決定: 問題があれば使っていくなかで出てくるので、確認は行わずに完了とする）:
  受信側の停止・輻輳からの回復、計測中の Wi-Fi の切断からの回復（どちらも 2026-11-17 の FreeRTOS 版では確認済み）、AP を落とす切断からの
  再接続、長時間の連続動作、計測中の `kv set`、`imu-host write`・`record` と ImuStudio の FreeRTOS 版での動作、`kv_transfer.py restore` の実行。
  使っていて不具合が出たら、まずこの一覧と照らす。
- 未調査のまま残したもの: I2C の SCL の実周波数（HCNT/LCNT 31/31）、INT1 の回数の実測、TCP の受信が約 2.5 Mbit/s（NuttX は約 1.2）で
  頭打ちになる理由、NuttX のシグナルハンドラー実行中の固まりの原因（NuttX 版ではハンドラーを登録しない）、Zephyr を上げる手順の実地の確認。
- 実機: FreeRTOS 版（最終のビルド）、NINA はパッチ 0001〜0006 のファームウェア。push はしていない。

## `ps` のタスクごとの CPU の割合を「前回の `ps` からの区間」に（2026-11-18、完了のあとの修正）

- ユーザーの指摘: FreeRTOS 版の `ps` と Zephyr 版の `kernel ps` で、タスクごとの CPU 使用率が小さすぎる。原因は、どちらも OS の統計の
  「起動からの累計」をそのまま割っていたこと（FreeRTOS: `ulRunTimeCounter / total`、Zephyr: thread analyzer の
  `execution_cycles * 100 / all.execution_cycles`、整数の %）。NuttX 版は NuttX 標準の `ps` で、スケジューラーの積算を全体が 2 秒を超えるたびに
  半分にする方式（直近 1〜2 秒の重みが大きい）なので問題なし。
- ユーザーの決定: いったん NuttX と同じ挙動（周期の標本化で積算を半分にする案 B）を選んだが、「`ps` のために常に動く処理で負荷を増やしたくない」
  ので、**A 案（前回の `ps` からの区間。常に動く処理なし）に決定**。NuttX 版とは意味が違うままになる。
- 実装（subagent）: 共有の `common/shell/ps_interval.h`（ヘッダーだけ。ハンドルで識別、静的な表、ホストテスト `ps_interval`）。FreeRTOS は
  `cmd_ps` に `ps_lock` を足し、`cpu_active` の窓を読む位置をタスクの読み出しの直後へ。Zephyr は `thread_analyzer_run()` をやめて
  `k_thread_foreach_unlocked()` と `k_thread_runtime_stats_get()`・`_all_get()`（分母は全体の `execution_cycles`＝アイドルを含む）、小数 1 桁、
  ISR0 の CPU 欄は `-`、最後に区間の長さの 1 行。上流は変えていない。文書（日英）: `freertos/drivers/README`、設計書、`nuttx-porting`、
  `nuttx/README` の 1 か所。`freertos/README` の `ps` の行はメインが直した。
- 実機: FreeRTOS は静止 10 秒で `IDLE` 99.6%（合計 100.0%）、送信 20 秒で `IDLE` 61.6・`imu_raw_mqtt_tid` 13.6・`imu_fifo_worker` 11.9・
  `imu_stream_tid` 11.4・`sysworkq` 0.8・`telnet_service` 0.5（合計 100.0%）、同じ区間の `cpu_active` 40.4%、欠番 0。Zephyr は静止 10 秒で
  `idle` 97.6・`net_socket_service` 1.4、送信 20 秒で `idle` 56.2・`imu_raw_mqtt_tid` 15.8・`imu_fifo_worker` 12.7・`imu_stream_tid` 11.8・
  `net_socket_service` 2.3・`sysworkq` 0.6（合計 100.0%）、`imu status` の `cpu_active` 48.3%、欠番 0、`dmesg` に `<err>`・`<wrn>` なし。
- **新しい基準**: Zephyr は `freertos/build/baseline/zephyr/snapshot-1118d/image.bin`（370652 B）。NuttX は `snapshot-1118c` のまま（NuttX は
  新しいヘッダーを使わない。再ビルドでの確認はしていない）。
- **メインの誤り**: 1 回目の確認のとき、`fuser` の確認と書き込み・確認のスクリプトを 1 つのコマンドにまとめていて、ユーザーの picocom が開いて
  いるのに `ifconfig`・`ps`・`imu start`/`stop`・`imu status` を送った（書き込みはスクリプトが止めた）。以後、確認は単独のコマンドで行い、結果を
  見てから操作する。
- 調査中（subagent、読むだけ）: `watch` コマンド、実行中の Ctrl+C、バックグラウンド実行を 3 版でどこまで作れるか（ユーザーの依頼）。
- 実機: FreeRTOS 版（新しい `ps`）、AP 接続中。

## `watch`・実行中の Ctrl+C・バックグラウンド実行: 調査だけ、見送り（2026-11-18、ユーザーの決定）

- ユーザーの決定: **この段階では 3 つとも見送る**。実装していない。
- 調査の結果（subagent、読むだけ。あとで再開するときの材料）:
  - NuttX: nsh の標準の `watch` はあるが `CONFIG_NSH_DISABLE_WATCH=y` で意図して外してある（「終わらないコマンドは外す」。止める手段が `-c` だけ、
    コマンドは 1 語しか取らない、`-n 0` で回り続ける）。Ctrl+C を標準の仕組み（`TTY_SIGINT` + `SIG_DEFAULT`）で作ると、既定の動作も sigaction として
    登録されて `arm_sigdeliver` を通るので、シグナルの固まりの経路を避けられない。シグナルを使わない形（`watch` の待ちを標準入力の `poll()` に
    して ^C か Enter で止める、nshlib へのパッチ 60〜80 行）なら作れる見込み。`&` は builtin アプリだけで、止める手段が無い。
  - Zephyr: `watch` に当たるものは無い。上流の `net ping` の形（専用スレッド + `shell_set_bypass()`、スレッドから `shell_execute_cmd()`）で作れる
    （約 150 行、RAM 約 +3 KB、上流へのパッチ不要）。シェルのスレッドの中で `shell_execute_cmd()` を回す形はシェルの内部の状態を壊す。
    バックグラウンド実行は `sh->ctx` を並行して使うので危険（dummy バックエンドが要り RAM 約 +4 KB）。
  - FreeRTOS: `nano_shell_execute()` を中から呼べるので `watch` は易しい（約 100 行）。実行中の入力を見る口（`nano_console` に追加、約 50 行）が要る。
  - Telnet のクライアントは ^C を IAC IP で送ることが多く、3 版とも捨てているので、止める操作に Enter を含めるのが確実。
  - 書式の案: `watch [-n 秒] [-c 回数] <コマンド…>`（nsh に合わせる）。`ps_interval` の前回の値は全セッションで 1 組なので、片方で `watch ps` を
    動かすと、もう片方の `ps` の区間も短くなる。

## Zephyr 版の tickless と刻みが負荷に占める分: 測定だけ、設定は変えない（2026-11-18、ユーザーの決定）

- きっかけ: 送信中の `cpu_active` が FreeRTOS 版（約 40%）より Zephyr 版（48〜50%）・NuttX 版（約 51%）で高い。tickless が上げていないか（ユーザーの問い）。
- 設定: FreeRTOS は周期 tick 1 ms。Zephyr は `CONFIG_TICKLESS_KERNEL=y`・`CONFIG_SYS_CLOCK_TICKS_PER_SEC=10000`、NuttX は
  `CONFIG_SCHED_TICKLESS=y`・`CONFIG_USEC_PER_TICK=100`。
- 測定（Zephyr、同じソースに追加の `.conf` を足した試験用のイメージ、送信 6667 Hz 約 70 秒、各 2 回、どれも欠番 0・`unix_regressions` 0）:
  - tickless・100 µs（現行、`snapshot-1118d` と同じイメージ）: 48.4%・48.4%
  - tickless・1 ms（`SYS_CLOCK_TICKS_PER_SEC=1000`）: 47.6%・47.5%
  - 周期 tick・1 ms（`TICKLESS_KERNEL=n` も）: 46.5%・46.7%
- 読み方: 刻みで約 0.9 ポイント、tickless で約 0.9 ポイント、合わせて約 1.8 ポイント。FreeRTOS 版との差の大半（6〜7 ポイント）は別の要因。
  1 ポイント前後の差はイメージの配置による変動（±1.3 ポイント）と同じ大きさで、内訳は不確か。
- 確かめていない: 周期 tick のイメージでの Telnet・スキャン・切断復帰・スタック、静止時の負荷、NuttX 版。
- **ユーザーの決定: tickless は使えるなら使っておく。設定は変えない**（リポジトリの変更なし）。
- 実機: 測定のあと FreeRTOS 版に戻した。

## NINA: `NINA_OS_WAIT_READY`・`NINA_OS_EVENTS` を無くし、実行時に選ぶ（2026-11-18、ユーザーの提案）

- 変更前: 共通コアの 2 つの分岐（READY を割り込みで待つか 100 µs おきに読むか、ソケットの事象を IRQ の線で待つか）はビルド時のマクロで選び、
  3 版とも定義していた（定義なしの分岐を通るビルドが無かった）。
- 変更後: マクロは無い。OS の境界（`common/drivers/nina/nina_os.h`）の 7 関数は必須。
  - READY: 新しい `nina_os_ready_start()` を `init_nina()` の最後で 1 回呼ぶ（起動ごとに 1 回）。0 なら `nina_os_ready_arm/wait/disarm()` で待ち、
    負なら共通コアが `NINA_READY_POLL_US`（100 µs）おきに読む。3 版の実装は、コールバックの登録を最初の `arm()` からここへ移しただけ
    （3 版とも、いまの GPIO の層では登録は失敗しない）。
  - 事象: 実行時の判断（capabilities のビットと `nina_os_event_start()` の結果）は変えていない。`#ifdef` だけ消した。
  - 割り込みを使えない OS は 2 つの `_start()` から負の値（`-ENOTSUP`）を返せばよく、残りの関数は呼ばれない。
- ホストテスト: マクロあり・なしの 2 回ビルドをやめ、1 つのバイナリーで `_start()` の成功・失敗の両方を通す（`common/tests/nina_spi.c`・`nina_wait.c`、
  Zephyr の `nina_boundary.c`・`nina_socket.c`、NuttX の `nina_os.c`、FreeRTOS の `tests/socket`）。Zephyr 41、NuttX 50、FreeRTOS 51 の `ok`、失敗 0。
  `format.sh --check` も通る。
- ビルド（メインが再ビルド、警告の増加なし）: FreeRTOS 232320 → 232416 B、Zephyr 370908 → 371012 B、NuttX 284988 → 285060 B。
  実行時の分岐が増えたので、イメージは前の基準と一致しない。**新しい基準**: `freertos/build/baseline/zephyr/snapshot-1118e/image.bin`（371012 B）、
  `.../nuttx/snapshot-1118e/image.bin`（285060 B）。
- 実機（書き込み直後の最初の計測、送信 6667 Hz 約 70 秒、各 1 回）: FreeRTOS `cpu_active` 40.2%、Zephyr 48.8%、NuttX 49.8%。どれも欠番 0・
  `unix_regressions` 0・`overruns` 0、`nina version` は `in use 0x00000007`、`dmesg` にエラー行なし。
- 確かめていない: 実機で `_start()` が失敗する経路（ホストテストだけ）、Telnet・スキャン・切断復帰。
- 文書（日英）: 3 版の `drivers/README`。
- 実機: FreeRTOS 版（この変更のビルド）。

## Zephyr 版の I2C のパッチ（0003）を作り直し: ツリーだけで完結、DMA の配列は 1 組、名前は `i2c_dw_rp2040_...`（2026-11-19、ユーザーの指摘）

- 指摘 1: パッチ後の `i2c_dw.c` が読む `nano_i2c_dma.h` と `i2c_dw_rp2040_commands.h` が Zephyr のツリーに無く、アプリが include パスとソースを
  足していた（別のアプリで `CONFIG_I2C_DW_RP2040_DMA=y` にするとビルドできない）。
- 指摘 2: `rp_dma_commands`・`rp_dma_buffer`（合わせて 10,752 B）が `struct i2c_dw_dev_config` のメンバーで、I2C のインスタンスごとに付く（DMA が動くのは
  i2c0 だけ。このボードは i2c0 だけ有効なので実際の無駄は無かった）。
- 直し方（ユーザーの決定: 新しいパッチを足さず 0003 を作り直す。名前は `nano_i2c_dma` ではなく `i2c_dw_rp2040_...`）:
  - パッチが足すファイル: `drivers/i2c/i2c_dw_rp2040.h`（変更なし）、`i2c_dw_rp2040_dma_priv.h`（旧 `i2c_dw_rp2040_dma.h`）、
    `i2c_dw_rp2040_commands.[ch]`、公開ヘッダー `include/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`。上流のファイルに食い込む箇所は 17 → 14
    （`i2c_dw.c` 11、`i2c_dw.h` 1、`Kconfig.dw` 1、`drivers/i2c/CMakeLists.txt` 1）。
  - 関数は `i2c_dw_rp2040_dma_get_stats()`・`_enabled()`・`_set_enabled()`、構造体は `struct i2c_dw_rp2040_dma_stats`。3 版で同じ名前。
    共有側の宣言は `common/compat/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h`（`nano_i2c_dma.h` は削除）で、`i2c_dma_shell.c` は 3 版とも
    `<zephyr/drivers/i2c/i2c_dw_rp2040_dma.h>` を読む。
  - 構造体から出して静的にしたもの（i2c0 用の 1 組）: 2 つの配列、`rp_dma_sem`、`rp_dma_events`、`rp_dma_tx_channel`・`rp_dma_rx_channel`。
    `rp_dma_active`・`rp_dma_quarantined` は全インスタンスの経路が読むので残した。共有コア（FreeRTOS・NuttX）は構造体のまま（イメージを変えない）。
  - アプリの `CMakeLists.txt` は `drivers__i2c` に何も足さない。`i2c_dma_shell.c` は `app` に入れる。
  - 同じ内容が 2 か所（パッチと `common/`）にあるファイルは `common/tests/i2c_dw_sync.py` がバイト単位で比べる（31 checks）。
- 確認（メインが再ビルド）: FreeRTOS は `SOURCE_DATE_EPOCH=1795003200` で基準（`freertos/build/baseline/freertos/snapshot-1118e/image.bin`、232416 B）と
  一致（日付を固定しないと `__DATE__` の 1 バイトだけ違う）。NuttX は `snapshot-1118e` と一致。Zephyr は 371012 → 370788 B、`i2c_0_runtime` は
  10904 → 116 B、RAM の合計はほぼ同じ（177419 → 177415 B）。ホストテスト: Zephyr 41、NuttX 50、FreeRTOS 51 の `ok`。`format.sh --check` も通る。
  subagent の確認: 無関係の `samples/hello_world` が `CONFIG_I2C_DW_RP2040_DMA=y` でビルドできる。`setup.sh` で旧 0003 から新 0003 へ当て直せる。
- 実機（Zephyr、書き込み直後の最初の計測、送信 約 70 秒、1 回）: `cpu_active` 47.0%、欠番 0、`unix_regressions` 0、`overruns` 0、
  `i2c_dma stats` は `dma_transfers=7700 dma_errors=0 dma_fallbacks=0`、`dmesg` に `<err>`・`<wrn>` なし。
- **新しい基準**: Zephyr は `freertos/build/baseline/zephyr/snapshot-1119a/image.bin`（370788 B）。NuttX は `snapshot-1118e` のまま。
- 実機: FreeRTOS 版。

## 共有コアの I2C0 でも DMA の配列などを構造体から出す（2026-11-19、ユーザーの指示）

- 共有コア（`common/drivers/i2c_dw/nano_i2c.c`、FreeRTOS 版・NuttX 版）も、パッチ後の Zephyr のドライバーと同じ置き方にした: `rp_dma_sem`・
  `rp_dma_events`・`rp_dma_tx_channel`・`rp_dma_rx_channel`・`rp_dma_commands[]`・`rp_dma_buffer[]` はファイル内の静的変数、
  `rp_dma_active`・`rp_dma_quarantined` は構造体に残す。`i2c_dw_rp_dma_notify()` は `dw` の引数が無くなった。
- セマフォの初期化は場所も順序も同じ（各版の `nano_i2c_instance.h` の `nano_i2c0_initialize()`。対象が静的変数になっただけ）。
- `i2c_dw_sync.py`: この違いを隠していた読み替えの規則を削除。expected は 285 → 281 行（`notify` の宣言と呼び出しの違いが消えた）。
- ビルド（メインが再ビルド、警告の増加なし）: FreeRTOS 232416 → 232432 B、NuttX 285060 → 285084 B（静的変数を直接指すコードの分）。`i2c_0_runtime` は
  FreeRTOS 10856 → 76 B、NuttX は 88 B。Zephyr は `snapshot-1119a` と一致。ホストテスト: FreeRTOS 51、NuttX 50、Zephyr 41 の `ok`。
- 実機（書き込み直後の最初の計測、送信 約 70 秒、各 1 回）: FreeRTOS `cpu_active` 40.1%・`dma_transfers=7613`、NuttX 49.8%・`dma_transfers=7476`。
  どちらも欠番 0、`unix_regressions` 0、`overruns` 0、`dma_errors=0`、`dma_fallbacks=0`。
- **新しい基準**: NuttX は `freertos/build/baseline/nuttx/snapshot-1119a/image.bin`（285084 B）。Zephyr は `snapshot-1119a` のまま。
- Kconfig.dw の help の行（タブ 1 つ + スペース 2 つ）は上流の書き方なので変えない（ユーザーの了解）。
- 実機: FreeRTOS 版。

## I2C の DMA の内部の名前を `i2c_dw_rp2040_dma_...`・`rp2040_dma_...` にそろえ、Zephyr のパッチはコマンド列の関数を 1 ファイルに（2026-11-19、ユーザーの決定）

- 名前（Zephyr のパッチ 0003 と共有コアの両方）: 内部の関数は `i2c_dw_rp2040_dma_notify()`・`_callback()`・`_stop()`・`_read()`
  （旧 `i2c_dw_rp_dma_*`）、変数と構造体のメンバーは `rp2040_dma_*`（旧 `rp_dma_*`）、マクロは `RP2040_DMA_*`（旧 `RP_DMA_*`）。
  これより前の節の記述は旧名のまま。`i2c_dma` コマンドの出力は変えていない。
- Zephyr のパッチ: `i2c_dw_rp2040_read_commands()` を `drivers/i2c/i2c_dw_rp2040_dma_priv.h` の static 関数にして、`i2c_dw_rp2040_commands.[ch]` と
  `drivers/i2c/CMakeLists.txt` の 1 行を外した（ユーザーの決定: Zephyr 版はまとめる）。パッチが足すファイルは 3 つ、上流のファイルに食い込む箇所は
  14 → 13（`i2c_dw.c` 11、`i2c_dw.h` 1、`Kconfig.dw` 1）。`common/drivers/i2c_dw/i2c_dw_rp2040_commands.[ch]` は共有コアとホストテスト用に残る。
- `i2c_dw_sync.py`: コマンド列の関数は本文を文単位で比べ、コマンドのビットの値も比べる（33 checks）。公開ヘッダーはバイト単位の比較のまま。
- 確認（メインが再ビルド）: NuttX は `snapshot-1119a` と一致。FreeRTOS は 232432 B のまま（同じ `SOURCE_DATE_EPOCH` での前後の一致は subagent が確認）。
  Zephyr は 370788 → 370732 B（関数が `i2c_dw_transfer()` に取り込まれた分）。ホストテスト: FreeRTOS 51、NuttX 50、Zephyr 41 の `ok`。
  subagent の確認: `samples/hello_world` が `CONFIG_I2C_DW_RP2040_DMA=y` でビルドできる、`setup.sh` で旧 0003 から当て直せる。
- 実機（Zephyr、書き込み直後の最初の計測、送信 約 70 秒、1 回）: `cpu_active` 47.2%、欠番 0、`unix_regressions` 0、`overruns` 0、
  `dma_transfers=7722 dma_errors=0 dma_fallbacks=0`、`dmesg` に `<err>`・`<wrn>` なし。
- **新しい基準**: Zephyr は `freertos/build/baseline/zephyr/snapshot-1119b/image.bin`（370732 B）。NuttX は `snapshot-1119a` のまま。
- 実機: FreeRTOS 版。

## Zephyr 版で `CONFIG_I2C_DW_RP2040_DMA=n` の確認（2026-11-19、ユーザーの依頼）

- ビルド: 追加の `.conf` で `CONFIG_I2C_DW_RP2040_DMA=n` にした試験用のイメージ（`zephyr/build/usb-console-nodma/`、リポジトリの変更なし）。
  警告なし、368544 B、`rp2040_dma_*` の記号なし、`i2c_0_runtime` は 108 B。`CONFIG_DMA`・`CONFIG_DMA_RPI_PICO` は `y` のまま。
- 実機（書き込み直後の最初の計測、送信 約 70 秒、1 回）: 欠番 0、`unix_regressions` 0、`overruns` 0、`i2c_errors` 0、`recoveries` 0、
  `cpu_active` 69.4%（DMA ありは 47.2%）。`sensor get` は値を返す。`i2c_dma` は `command not found`（意図どおり）。`dmesg` に `<err>`・`<wrn>` なし。
- 確かめていない: 長時間、Telnet・スキャン・切断復帰と重ねた場合（負荷が高いので余裕は小さい）。
- 実機: FreeRTOS 版に戻した。

## 共有の I2C0 コアのファイル名と名前を Zephyr のドライバーに合わせ、Zephyr の定義を持ち込まない（2026-11-19、ユーザーの決定）

- 方針（ユーザー）: `common/drivers/i2c_dw` は OS 非依存のドライバーとして `common/drivers` に残す。そのために Zephyr の定義を入れない。
  ファイル名と名前は、写しのもとの Zephyr のドライバーに合わせる。これより前の節の記述は旧名のまま。
- ファイル名: `nano_i2c.c` → `i2c_dw.c`、`nano_i2c_dw.h` → `i2c_dw.h`、`nano_i2c_dw_registers.h` → `i2c_dw_registers.h`、
  `nano_dma.[ch]` → `dma_rpi_pico.[ch]`。Zephyr のパッチと同じ区切りで `i2c_dw_rp2040.h`（FIFO の定数）と `i2c_dw_rp2040_dma_priv.h`（DMA の読み出し。
  `i2c_dw.c` が 1 回読む）に分けた。各 OS の側は `i2c_dw_instance.h`・`dma_rpi_pico_instance.h`・`i2c_dw_i2c0.h`（旧 `nano_i2c.h`）、
  FreeRTOS の `dma_rpi_pico_os.c`。テストは `common/tests/i2c_dw.c`。
- 名前: `nano_dma_*` → `dma_rpi_pico_*`（構造体は `dma_rpi_pico_channel_config`・`dma_rpi_pico_block_config`。`dma_rpi_pico_config` は関数名とぶつかる）、
  `nano_i2c0_initialize()` → `i2c_dw_i2c0_initialize()`、`nano_i2c_dw_*`・`nano_dw_*` → `i2c_dw_*`、`NANO_I2C_DW_*` → `I2C_DW_*`、
  `NANO_I2C_SPEED_*` → `I2C_SPEED_*`（Zephyr と同じ書き方と値。NuttX の `i2c_master.h` の同名の Hz の定義は NuttX の `i2c_dw_os_impl.h` が `#undef`）、
  `CONFIG_NANO_I2C0_DMA` → `CONFIG_I2C_DW_RP2040_DMA`（NuttX の Kconfig の記号も）。
- Zephyr の定義: `mm_reg_t` は `uintptr_t` に。`MIN()`・`ARRAY_SIZE()` はコアの `i2c_dw.h` が `#ifndef` で定義。`BIT()`・`I2C_MSG_*`・`LOG_*()`・
  関数名と構造体名はそのまま（ユーザーの決定）。`common/drivers/i2c_dw/` に `<zephyr/...>` の include は無い。
- 公開の宣言: コアの `common/drivers/i2c_dw/i2c_dw_rp2040_dma.h`（メインが追加）。`common/compat/zephyr/drivers/i2c/i2c_dw_rp2040_dma.h` はそれを読むだけの
  ヘッダーにした。`i2c_dw_sync.py` は、パッチの公開ヘッダーとコアのヘッダーを、コメントと include ガードを除いた宣言で比べる（バイト単位の比較をやめた）。
- `i2c_dma_shell.c` は `common/shell/` へ（Zephyr のシェルの API で書いたコマンドなので）。
- パッチ 0003: 公開ヘッダーのコメントの古い名前（`nano_i2c.c`、`CONFIG_NANO_I2C0_DMA`、シェルの場所）だけ直した（メインが作り直し、`setup.sh` で当て直した）。
- NuttX の Kconfig の記号も変えた（ユーザーの指示）: `NANO_I2C0` → `I2C_DW_I2C0`、`NANO_APP_I2C_DMA`（と `_PRIORITY`・`_STACKSIZE`）→
  `I2C_DW_RP2040_DMA_SHELL`。イメージは変わらない（下の基準と一致）。
- 確認（メインが再ビルド）: Zephyr は `snapshot-1119b` と一致。FreeRTOS は同じ日付でビルドした前後が一致。NuttX は 285084 → 285092 B
  （`nano_bringup.c` の 2 つのエラーメッセージが関数名を含むため。subagent が、その 2 つを戻した対照のビルドで基準と一致することを確認）。
  ホストテスト: FreeRTOS 51、NuttX 50、Zephyr 41 の `ok`、`i2c_dw_sync` 33 checks。`format.sh --check` も通る。
- 実機（NuttX、書き込み直後の最初の計測、送信 約 70 秒、1 回）: `cpu_active` 49.9%、欠番 0、`unix_regressions` 0、`overruns` 0、
  `dma_transfers=7474 dma_errors=0 dma_fallbacks=0`。
- **新しい基準**: NuttX は `freertos/build/baseline/nuttx/snapshot-1119b/image.bin`（285092 B）。Zephyr は `snapshot-1119b` のまま。
- 確かめていない: FreeRTOS の `-DNANO_SELFTEST=ON` のビルド。
- 実機: FreeRTOS 版。

## `dma_rpi_pico.[ch]` を Zephyr のスタイルと Pico SDK の名前に、コマンド列の関数はコアでも 1 ファイルに（2026-11-19、ユーザーの指示）

- `common/drivers/i2c_dw/dma_rpi_pico.c`: NuttX のスタイルだったものを、写しのもとの `zephyr/drivers/dma/dma_rpi_pico.c` のスタイル・順序・名前・文に合わせて書き直した
  （`RPI_PICO_DMA_INIT(0)` に当たる実体は `dma_rpi_pico0_config`・`dma_rpi_pico0_data` などとして書き下した）。Zephyr のファイルとの
  `diff -u` の `+`/`-` の行は 806 → 352。残る違い: 移植の説明のコメント、include、`struct device` が無いこと、OS の側に出した reset と割り込みの登録、
  使わない関数（reload・get_status・chan_filter・API の表）が無いこと、DMA の API のチャンネル確保を書き下した関数、公開の関数にした config・start・stop。
- `dma_rpi_pico.h`（Zephyr に同名のファイルは無い。名前はそのまま＝ユーザーの決定）: レジスターの定義を Pico SDK の名前にした（`DMA_BASE`、
  `DMA_CH0_*_OFFSET`、`DMA_CH0_CTRL_TRIG_*_BITS`/`_LSB`、`dma_hw_t` と `dma_hw`、`hw_set_bits()`・`hw_clear_bits()`、`dma_channel_config`、
  `channel_config_set_*()`、`dma_channel_configure()`、`dma_channel_is_busy()`、`dma_irqn_*()`）。SDK のヘッダーは読まない。SDK のヘッダーが先に
  読まれている翻訳単位では、SDK の include ガードの名前を見て自分の定義を出さない（FreeRTOS の `dma_rpi_pico_os.c` がそれで、マクロの一致をコンパイル時に確かめる形になる）。
  Zephyr の DMA の API に当たる名前（`DMA_RPI_PICO_ADDR_ADJ_*` など）と移植の関数名は変えていない。
- `i2c_dw_rp2040_commands.[ch]` を削除し、`i2c_dw_rp2040_read_commands()` はコアの `i2c_dw_rp2040_dma_priv.h` の static 関数にした（Zephyr のパッチと同じ）。
  単独のホストテスト（`common/tests/i2c_dma_commands.c`）は削除し、項目は FreeRTOS のホストテスト（`freertos/tests/i2c_dw/`、コアを 1 つの翻訳単位で
  ビルドする `i2c_dw_core.c`）へ移した。ホストテストの数は FreeRTOS 51 → 50、NuttX 50 → 49、Zephyr 41 → 40（3 版が共通に流していた 1 項目の分）。
- ほかのファイル: NuttX 風の見出しのコメント、`#  define`、include ガードの名前を Zephyr のスタイルに（コードの変更なし）。
- `i2c_dw_sync.py`: DMA も比べる（Zephyr の `dma_rpi_pico.c` の 10 関数と `dma.h` の 2 関数。意図した違いは同じ expected に 50 行）。
- パッチ 0003: コアのファイル名を挙げたコメント 1 か所だけ直した（メインが作り直し、`setup.sh` で当て直した）。
- 確認（メインが再ビルド、警告の増加なし）: Zephyr は `snapshot-1119b` と一致。FreeRTOS 232432 → 232376 B、NuttX 285092 → 285036 B
  （コマンド列の関数が `i2c_dw_transfer()` に取り込まれた分と、`dma_hw->` の形のアドレス計算。subagent の確認: レジスターへのアクセスの順序は逆アセンブルで同じ、
  ホスト上で新旧の `dma_rpi_pico.c` を同じ入力で動かした出力が一致）。ホストテスト: FreeRTOS 50、NuttX 49、Zephyr 40 の `ok`。`format.sh --check` も通る。
- 実機（書き込み直後の最初の計測、送信 約 70 秒、各 1 回）: FreeRTOS `cpu_active` 40.6%・`dma_transfers=7627`、NuttX 50.5%・`dma_transfers=7477`。
  どちらも欠番 0、`unix_regressions` 0、`overruns` 0、`dma_errors=0`、`dma_fallbacks=0`。FreeRTOS は `i2c_dma mode off` → `sensor get` → `on` も動いた。
- **新しい基準**: NuttX は `freertos/build/baseline/nuttx/snapshot-1119c/image.bin`（285036 B）。Zephyr は `snapshot-1119b` のまま。
- 実機: FreeRTOS 版。

## I2C0 の DMA の読み出しを各 OS の DMA の上に（`dma_rpi_pico.[ch]` を削除）（2026-11-19、ユーザーの決定）

- 理由（ユーザー）: ほかの用途で DMA を使うとき、FreeRTOS なら Pico SDK の、NuttX なら NuttX の DMA をそのまま使いたい。コアが DMA のブロック全体を
  持つ作り（リセット、両方の割り込みの線、チャンネルの割り当て。FreeRTOS は SDK の記録で全 12 チャンネルを確保、NuttX は `RP2040_DMAC` を禁止）をやめる。
- 境界: `common/drivers/i2c_dw/i2c_dw_dma_os.h`（OS や SDK の名前なし）。`i2c_dw_dma_os_initialize()`・`_is_ready()`・`_request_channel()`（待たない）・
  `_release_channel()`・`_config_tx()`（16 ビットのコマンド語、DREQ 32）・`_config_rx()`（バイト、DREQ 33）・`_start()`・`_stop()`（中断を頼むだけで待たない）・
  `_is_busy()`・`_acknowledge()`、割り込みからのコールバック（0 か `-EIO`）。コアの条件・コマンド語・しきい値・待ち（100 ms）・カウンター・使用停止は変えていない。
- FreeRTOS（`freertos/drivers/i2c_dw/i2c_dw_dma_os.c`）: 読み出しごとに `dma_claim_unused_channel(false)` で 2 本、終わったら返す。割り込みは
  `irq_add_shared_handler(DMA_IRQ_1, …)` で、自分が持つチャンネルだけを処理。中断は自分で CHAN_ABORT に書く（SDK の `dma_channel_abort()` は待ちに上限が無い）。
- NuttX（`nuttx/boards/…/src/i2c_dw_dma_os.c`）: 起動時に `rp2040_dmachannel()` で 2 本取り、持ち続ける（この関数は空きが無いと待つので、読み出しのたびには呼ばない）。
  設定は `rp2040_txdmasetup()`・`rp2040_rxdmasetup()`、開始は `rp2040_dmastart()`、割り込みは本体のハンドラー（DMA_IRQ_0）。中断は自分で書き、
  `rp2040_dmastop()`（待ちに上限が無い）は呼ばない。Kconfig は `depends on … RP2040_DMAC`、defconfig は `RP2040_DMAC` 有効・`RP2040_SPI_DMA` 無効
  （NINA の SPI は DMA なしのまま）。本体へのパッチは不要だった。
- 以前と違う点: (1) NuttX の `fallbacks` は「起動時に取った 2 本が busy で設定を断られた回数」になり、実質 0。起動時に取れなければ割り込みの経路で動き、
  ログに `No DMA for the reads`。(2) 中断の待ちは自分の 2 本だけを見る。(3) DMA のブロックをリセットしない。(4) NuttX 本体のハンドラーの性質
  （subagent がコードから読み取った。実機では未確認）: エラー時にエラーフラグを起動レジスターへ書き戻すので、アダプターがそのチャンネルをすぐ中断する。
  同じ割り込みの中では、番号の小さいチャンネルのエラーが後のチャンネルにも伝わるので、他人のバスエラーで読み出しが `-EIO` になりうる。
- ほかのコードが守ること（ドライバー説明に記載）: 読み出しが持つ 2 本に触れない、DMA のブロックをリセットしない、INTE・INTS・CHAN_ABORT は自分のビットだけ書く、
  FreeRTOS では DMA_IRQ_1 に排他のハンドラーを入れない、NuttX では DMA_IRQ_0 を本体から奪わない。
- 削除: `common/drivers/i2c_dw/dma_rpi_pico.[ch]`、各 OS の `dma_rpi_pico_instance.h`、`freertos/drivers/i2c_dw/dma_rpi_pico_os.c`（全チャンネルの確保も）。
  `i2c_dw_sync.py` の DMA のドライバーの比較も削除（読み替えの規則は境界の関数へ）。
- ホストテスト: FreeRTOS 50（DMA の読み出しを SDK の DMA の fake の上で実行: 完了、短い読み出し、チャンネルなし、TX・RX のエラー、タイムアウト、mode off、
  終わらない中断 → 使用停止、カウンター。接着の単体も）、NuttX 50（新しい `i2c_dw_dma_os`: `rp2040_dmac.h` の fake）、Zephyr 40。`format.sh --check` も通る。
- ビルド（メインが再ビルド、警告の増加なし）: FreeRTOS 232376 → 231616 B、NuttX 285036 → 284856 B、Zephyr は `snapshot-1119b` と一致。
- 実機（書き込み直後の最初の計測、送信 約 70 秒）: FreeRTOS `cpu_active` 40.2%（1 回）、欠番 0、`dma_transfers=7651 dma_errors=0 dma_fallbacks=0`、
  `i2c_dma mode off` → `sensor get` → `on` も動く、`dmesg` にエラーなし。NuttX 52.1%・52.3%（2 回。変更前は 50.5%。約 1.7 ポイント増。原因は調べていない）、
  欠番 0、`dma_errors=0 dma_fallbacks=0`、`mode off`・`sensor get`・`on`、Telnet の接続も動く。
- 確かめていない: エラー・タイムアウト・使用停止の経路（実機で起こす手段が無い。ホストテストだけ）、FreeRTOS の `selftest`、長時間、ほかの利用者と DMA を
  実際に共有した場合。
- **新しい基準**: NuttX は `freertos/build/baseline/nuttx/snapshot-1119d/image.bin`（284856 B）。Zephyr は `snapshot-1119b` のまま。
- 実機: FreeRTOS 版。

## 共有コアを I2C1 に対応（既定 OFF）（2026-11-19、ユーザーの決定）

- 理由（ユーザー）: I2C1 を OS のドライバーに任せると、同じ種類のハードウェアにドライバーが 2 本になる。共有コアで両方を動かす。I2C1 に何かをつなぐ予定は無く、
  使える状態になっていればよい。
- コア（`i2c_dw.c`）: Zephyr の `I2C_DEVICE_INIT_DW(n)` に当たる形で、`CONFIG_I2C_DW_I2C0` が `i2c_0_runtime`、`CONFIG_I2C_DW_I2C1` が `i2c_1_runtime` を作る。
  番号ごとに違う値は `I2C_DW_INST(dw, v0, v1)`（片方だけのビルドでは判定なし＝既定のイメージが変わらない）。DMA の読み出しは i2c0 だけ（パッチと同じ）、
  FIFO の扱いは両方。
- FreeRTOS: `i2c_dw.cmake` が 2 つの記号を同じ場所で定義。`CONFIG_I2C_DW_I2C0` は常に 1（LSM6DSOX が必要。外すと `lsm6dso.cmake` が設定で止める）、
  `CONFIG_I2C_DW_I2C1` はオプション `-DI2C_DW_I2C1=ON`（既定 OFF）。デバイスは `DEVICE_DT_GET(DT_NODELABEL(i2c1))`、Zephyr の I2C の API。
  ビルドのオプションは `-DNANO_SELFTEST=ON` と合わせて 2 つになった（ユーザーの了解）。
- NuttX: Kconfig の `I2C_DW_I2C1`（既定 n）。2 つ目の `struct i2c_master_s`、`CONFIG_I2C_DRIVER` があれば `/dev/i2c1`。defconfig を変えずに有効にする手順:
  ビルドのあと `.config` に `CONFIG_I2C_DW_I2C1=y` と `CONFIG_I2CTOOL_MAXBUS=1` を足し、`make -C nuttx/nuttx olddefconfig`、`--clean` なしでビルド。
- ピン: SDA GPIO18（D6）、SCL GPIO19（D7）。3 版のボードの定義で未使用。I2C1 は内蔵プルアップを有効にする（ボードに外付けのプルアップが無いため）。
  I2C0 は従来どおりプルなし（ボードが外付けで引き上げている）。定義は FreeRTOS が `i2c_dw_instance.h`、NuttX が `include/board.h`。
- 各 OS の側の公開ヘッダーは `i2c_dw_i2c0.h` → `i2c_dw_controllers.h`。
- 確認（メインが再ビルド）: 既定のビルドは 3 版とも基準と一致（FreeRTOS は同じ日付の前後、NuttX `snapshot-1119d`、Zephyr `snapshot-1119b`）。
  ホストテスト: FreeRTOS 50（`i2c_dw` を i2c0 だけ・i2c1 つきの 2 回）、NuttX 51（新しい `i2c_dw_instance`）、Zephyr 40。`format.sh --check` も通る。
  I2C1 つきのビルド: FreeRTOS +336 B（`.bss` +80 B）、NuttX 284856 → 285280 B。警告の増加なし。
- 実機（I2C1 に何もつないでいない）:
  - FreeRTOS（`-DI2C_DW_I2C1=ON -DNANO_SELFTEST=ON`）: `selftest` は 22 項目すべて通過、I2C は 7 項目（`i2c1` を含む: デバイスが ready、ピンの設定、
    応答の無いアドレスが 100 ms 未満で `-EIO`、40 バイトの読み出しでも DMA を使わない）。I2C0 の送信 約 70 秒は `cpu_active` 39.6%、欠番 0、DMA のエラー 0。
  - NuttX（I2C1 つき）: `i2c bus` は 0・1 とも YES、`i2c dev -b 1 0x08 0x77` は全アドレス応答なしですぐ戻る、`-b 0` は 0x60 と 0x6a、`/dev/i2c1` あり、
    `dmesg` に `NACK on i2c@40048000`。I2C0 の送信 約 70 秒は `cpu_active` 53.0%、欠番 0、DMA のエラー 0。
- 確かめていない: I2C1 で実際のデバイスとのやり取り、内蔵プルアップだけでの波形と速度の上限。
- 実機: 既定の FreeRTOS 版（I2C1 なし）。

## 文書の片づけ（リファクタリングに左右されない分）と、このあとの予定（2026-11-19、ユーザーの決定）

- 予定: NINA と sensor（LSM6DSOX）も I2C と同じ形にする（名前を写しのもとに合わせる、`common/drivers` に Zephyr の定義を入れない、Zephyr 版は上流 + パッチ、
  ずれの検査。OS の資源の話は関係しない）。LSM6DSOX のやり方は相談しながら詰める。指針は `CLAUDE.md`（Git 管理外）の「共有ドライバーの作り方の指針」。
- 文書の整理はそのあとに行う。案: 新しい `design/architecture(-ja).md`（全体像、置き場所の基準、名前とスタイル、ドライバーごとの構成、共有の資源、
  Zephyr とそろえて保つ仕組み、RP2040 の低い層、新しい OS へ移す手順、版ごとの違い）。設計書の 12 節と 11・13 節の構成の話、3 版のドライバー説明の重なり
  （NINA のファームウェアの機能、`nina passthrough`、除算器と `memcpy`）はそこへ寄せ、もとの場所は削除して参照にする。`PORTING_*.md` は残す。
- 今回行った分（subagent、文書だけ、日英）: 「像」→「イメージ」31 か所、経緯の文の削除（実機で確かめた日付と条件は根拠として残した）、ルートの README の
  3 つの節を 1 つに、「リポジトリの構成」の表を短く、I2C と DMA の説明をいまの形に合わせて書き直し（場所は動かしていない）。事実と違っていた記述も直した
  （パッチの公開ヘッダーと `common/compat` のヘッダーは「同じファイル」ではなく、検査は宣言を比べる。`CONFIG_I2C_DW_RP2040_DMA` はパッチでは既定 OFF で
  `prj.conf` が有効にする、など）。リンクと、文書に出てくるパスの実在を確認。
- あとの整理に回したもの: Zephyr のドライバー説明の `i2c_dw_sync` の行が長い、`nuttx-porting` の「テスト 34 本」の数が未確認、設計書 §1 の合意済みの要求と
  確認済みの項目が早い時期の記述のまま、英語版で "Zephyr version" と "Zephyr build" が混在。

## NINA のリファクタリング: 方針と、段階 1・2（名前、シェルの移動）（2026-11-19、ユーザーの決定）

- 方針: NINA には写しのもとの上流のドライバーが無いので、I2C の指針のうち「上流 + パッチ」「ずれの検査」は当てはまらない（ファームウェアとの突き合わせの
  検査も入れない）。行うこと: `nano` の名前を無くす（Arduino Nano は RP2040 の機種だけではない。リポジトリ全体の `nano_` は別の作業）、コマンド番号に名前、
  `common/drivers/nina` に Zephyr の定義を入れない、`nina_net.c` の Wi-Fi の操作を OS 非依存のコアの関数にして NuttX 版はそれを直接呼ぶ。
- 構成（ユーザーの提案）: ドライバーは自分のつなぎも一緒に持つ。`common/drivers/nina/` がコア（OS の定義なし）、その下の `zephyr/`・`nuttx/` が各 OS の API への
  つなぎ。FreeRTOS 版は `zephyr/` のつなぎを互換層の上でビルドし、そのために要る Zephyr の層（ソケットのオフロードなど）を `common/compat` に実装する。
  ボードやチップに固有の部分は各版に残す。うまく行けば `i2c_dw` も同じ形に移す。
- 順番: (1) 名前、(2) シェルを `common/shell/` へ、(3) Wi-Fi の操作をコアへ・つなぎをサブディレクトリーへ・NuttX は直接、(4) OS の境界の実装を Zephyr 版と
  FreeRTOS 版で 1 つに、(5) ソケットのオフロードの層を互換層に、(6) 残りの洗い出し。指針は `CLAUDE.md` の 9・10 番。
- 段階 1（名前）: `nano_nina.h` → `nina_bus.h`（`nina_bus_transceive()`・`nina_bus_ready()`・`nina_bus_initialize()`・`nina_pin_*()`・`NINA_PIN_*`）、各版の
  `nano_nina.c` → `nina_bus.c`、NuttX のボードの `nano_nina_passthrough*` → `nina_passthrough*`（`nano_pt_*` → `nina_pt_*`）、`CONFIG_NANO_NINA*` → `CONFIG_NINA*`
  （3 版。NuttX の名前が Zephyr 版と同じになったので、読み替えの 2 か所を削除）。コマンド番号と応答の値は新しい `nina_cmd.h`（名前は Arduino の WiFiNINA の
  `wifi_spi.h`・`wl_definitions.h`、無いものはファームウェアの関数名から。例: `NINA_CMD_SET_PASSPHRASE` 0x11、`NINA_CMD_START_SCAN_NETWORKS` 0x36、
  `NINA_CMD_GET_CAPABILITIES` 0x6a、`NINA_CMD_SOCKET_POLL` 0x7c）。
- 段階 2（シェル）: `nina_shell.c` と、`nina_passthrough_logic.h` にあった `nina_pt_subcmd_get()`（→ `common/shell/nina_shell.h`）を `common/shell/` へ。
  `nina_passthrough_logic.h` は Zephyr のヘッダーを読まなくなった。`common/drivers/nina/` に残る Zephyr の定義は `nina_net.c` だけ。
- NINA のファイルに残る `nano`: ボード全体の名前（`nano_gpio`、`nano_bringup`、`NANO_PRIO_*`、`CONFIG_NANO_APP_NINA*`、`NANO_SUBCMD` など）。別の作業。
- 確認（メインが再ビルド）: FreeRTOS は同じ日付でビルドした前後が一致、Zephyr は `snapshot-1119b` と一致。NuttX は大きさは同じ（284856 B）で、
  `nano_bringup.c` のエラーメッセージ 1 行（関数名）が 1 文字短くなった分だけ違う（文字列の比較で違うのはこの 1 行。subagent が、その 1 行を戻した対照のビルドで
  基準と一致することを確認）。ホストテスト: FreeRTOS 50、NuttX 51、Zephyr 40 の `ok`。`format.sh --check` も通る。
- NuttX 版は実機に書いていない（違いがエラーメッセージ 1 行だけで、次の段階で実機の確認を一通り行うため）。
- **新しい基準**: NuttX は `freertos/build/baseline/nuttx/snapshot-1119e/image.bin`（284856 B）。Zephyr は `snapshot-1119b` のまま。

## NINA のリファクタリング: 段階 3（Wi-Fi の操作をコアへ、つなぎを `zephyr/`・`nuttx/` のサブディレクトリーへ）（2026-11-19）

- コア: `common/drivers/nina/nina_wifi.[ch]`（`nina_net.c` から移したコード。Zephyr の定義なし）。`nina_wifi_init()`・`_scan()`・`_connect()`・`_disconnect()`・
  `_status()` と、スレッドの本体 `nina_wifi_scan_thread()`・`_connect_thread()`・`_monitor_thread()`。上の層へはコールバックの表 `struct nina_wifi_ops`
  （`ip_set`・`link_up`・`link_down`・`connect_result`・`disconnected`、任意の `connection_allowed`・`reconnect_pause`・`reconnect_request`）。
  `nina_os.h` に足したのは `nina_os_wifi_lock()`・`_unlock()` と `nina_os_wifi_post()`・`_wait()`。スレッドの起動は境界に足さず、各つなぎが自分の OS で起こす
  （名前・スタック・優先度をそのまま保つため）。ロックなしで読む値は C11 の `<stdatomic.h>` の読み書きだけ。
- つなぎ: `common/drivers/nina/zephyr/`（`nina_net.c`＝Wi-Fi の管理への登録、`nina_os.c`、`nina_socket.c`、`posix/`、ドライバーの Kconfig と dts の binding）、
  `common/drivers/nina/nuttx/`（`nina_netdev.c`＝wapi の ioctl からコアを直接呼ぶ、`nina_wifi_mgmt.[ch]`、`nina_os.c`、`nina_usrsock.[ch]`、`nina_poller.c`、`nina_net.h`）。
  NuttX の `Kconfig` と `Make.defs` は `nuttx/drivers/nina/` に残る（NuttX が `nuttx/drivers` を `external/` として読む仕組みのため。`VPATH` で新しい場所を指す）。
  各版に残したもの: Zephyr の `nina_bus.c`（ボードの `spi1` のラベルと中継の判定を持つ）と `nina_passthrough*`、NuttX のボードの `nina_bus.c`・`nina_passthrough*`、
  FreeRTOS の `nina_os.c`・`nina_socket.c`・`nina_netif.c`・`nina_bus.c`・`nina_passthrough*`（段階 4・5 の対象）。FreeRTOS は `zephyr/nina_net.c` を互換層の上でビルドする。
- NuttX: wapi の ioctl → `nina_netdev.c` → コア。互換層のうち NuttX でビルドしなくなったもの: `net_if` のアドレス設定の関数、オフロードの表を通るスキャン・切断・状態の経路。
  まだ Zephyr の Wi-Fi の管理の API を使う NuttX のアプリ（`common/imu/wifi_profile.c`、`wifi cred auto_connect`、`time_sync.c`、`imu_raw_mqtt.c`）には、
  `nina_wifi_mgmt.c` が `.connect` だけの登録と 2 つのイベント（接続の結果、切断の結果）を出して応える。ここをどうするかは別の判断。
- コアの独立の検査: `common/tests/run.sh` の `nina_core_independent`（コアの 15 ファイルを、C ライブラリーと POSIX のソケットのヘッダーだけでコンパイル）。
- ホストテスト（メインが実行）: FreeRTOS 52、NuttX 55、Zephyr 43 の `ok`。新しいもの: `common/tests/nina_wifi.c`（コマンドの順序・間隔・エラー）、Zephyr の `nina_net`、
  NuttX の `nina_netdev`・`nina_wifi_mgmt`。`format.sh --check` も通る。
- ビルド（メインが再ビルド、警告の増加なし）: Zephyr 370732 → 371340 B、NuttX 284856 → 284388 B、FreeRTOS 231616 → 232128 B。
- 実機（各 1 回）:
  - FreeRTOS: 起動後 2 秒で接続、`wifi status`、`wifi scan`（結果あり、接続は保たれる）、送信中のスキャン 1 回で 40/1048 バッチが落ちる（11-18 の記録は 2 回で
    109/1117。スキャン中は無線がチャネルを離れるため）、`wifi disconnect` で切断、`wifi cred auto_connect` で 3 秒で再接続、Telnet、`dmesg` にエラーなし。
    スレッドは `nina_scan_tid`・`nina_connect_tid`（優先度 12）、`nina_monitor_tid`（11）。
  - Zephyr: 起動後 31 秒で接続、`wifi scan`、送信 20 秒は欠番 0・`cpu_active` 45.5%、切断と再接続（3 秒）、Telnet、`dmesg` にエラーなし。スタックの使用量は
    scan 920/1536、connect 576/1024、monitor 496/1536。
  - NuttX: 起動時に接続済み、`wapi show`、`wapi scan`（結果あり、接続は保たれる）、送信 20 秒は欠番 0・`cpu_active` 51.0%、`wapi disconnect` で切断、
    `wifi cred auto_connect` で 3 秒で再接続、`ntp status` は synchronized、Telnet、`dmesg` にエラーなし。スレッドの優先度は 110・110・109。
- 確かめていない: AP を落とす切断からの復帰、元のファームウェア（3.0.2）での動作、手動の `wifi connect`・`wapi psk`、長時間。
- subagent が挙げた違い: Zephyr のつなぎはスキャンのコールバックを静的変数に置く（Zephyr の管理層が常に同じものを渡す前提）。NuttX で手動の `ifconfig` のアドレスが
  プロファイルの「アドレスあり」に数えられる。NuttX はスキャンの結果のイベントを出さなくなった（聞く側が無い）。`nuttx/patches/nuttx/0005` の説明文に古いパスが残る。
- **新しい基準**: Zephyr `freertos/build/baseline/zephyr/snapshot-1119c/image.bin`（371340 B）、NuttX `.../nuttx/snapshot-1119f/image.bin`（284388 B）。
- 実機: FreeRTOS 版。

## NINA のリファクタリング: 段階 4（FreeRTOS 版の OS の境界を FreeRTOS と Pico SDK で直接実装）（2026-11-19）

- 段階 3 の補足: FreeRTOS 版の「送信だけ」（約 70 秒）を測り直した結果は欠番 0、`cpu_active` 40.0%（段階 3 の節の FreeRTOS の行は、送信中にスキャンを入れた確認だけだった）。
- 方針の見直し（ユーザーの指摘）: I2C の DMA は各 OS の API を直接使うのに、NINA の READY と事象の待ちは Zephyr の GPIO の API を互換層に足して借りる、という案
  （Zephyr 版と FreeRTOS 版の `nina_os.c` を 1 つにする）は食い違っている。決めた線引き: **つなぎは、その版が本来持っているものを使う。FreeRTOS 版が持っていないもの
  （ネットワーク層）だけ Zephyr のものを借りる**。1 つにする案は取りやめた（調べると、FreeRTOS 版は READY のピンを Pico SDK で設定するので Zephyr の API の
  「アクティブ Low」の情報が入らず、逆のエッジで待つことになる落とし穴もあった）。
- 実装: `common/drivers/nina/freertos/nina_os.c`（と `nina_freertos.h`）。FreeRTOS と Pico SDK（`hardware/gpio.h`・`irq.h`・`timer.h`）だけで、Zephyr の名前も互換層の
  ヘッダーも使わない。`freertos/drivers/nina/nina_os.c` は削除。互換層には何も足していない。
  - ロック 3 つ: 再帰できるミューテックス（静的に確保。互換層の `k_mutex` が FreeRTOS の上で作っていたものと同じ）。Wi-Fi の依頼と READY: 2 値のセマフォ。
  - スリープと待ちの tick: 互換層と同じ丸め（切り上げて +1 tick）。時計: `time_us_64() / 1000`（互換層と同じ TIMER）。
  - READY: 古い合図を捨ててから、ピンの事象をすべて無効にし、待つエッジを有効にする（これまでと同じ順序）。10 ms ごとの区切り。
  - 事象: 待つ側が自分を一覧に載せ、割り込みが数を進めて一覧の全員にタスク通知を出す。専用の通知（index 2）を使うので
    `configTASK_NOTIFICATION_ARRAY_ENTRIES` を 2 → 3 にした（各タスクの管理ブロックが 8 B 増える）。
  - カーネルのオブジェクトは `nina_os_initialize()` が作る。`main()` がスケジューラーの開始前、`nina_bus_initialize()` の直前に呼ぶ。作る前に使うと FreeRTOS の
    `configASSERT` で止まる。
  - GPIO の割り込み: GPIO10 と GPIO2 をまとめた 1 つのハンドラーを `gpio_add_raw_irq_handler_masked()` で登録（SDK の共有ハンドラーの枠が 6 つしかないため 1 つに）。
    ボードの GPIO の層（`nano_gpio.c`）のマスクは GPIO24（LSM6DSOX の INT1）だけになった。
- ホストテスト（メインが実行）: FreeRTOS 53 の `ok`（`tests/nina` は互換層なしでリンクし、tick 1000 Hz と 250 Hz の 2 回、tick と TIMER の下位 32 ビットの数え直しをまたぐ）。
  NuttX 55・Zephyr 43（subagent）。`format.sh --check` も通る（対象に `common/drivers/*/freertos` を足した）。
- ビルド（メインが再ビルド、警告なし）: Zephyr は `snapshot-1119c` と一致、NuttX は `snapshot-1119f` と一致。FreeRTOS 232128 → 232872 B、bss +376 B。
- 実機（FreeRTOS、書き込み直後の最初の計測、各 1 回）: 送信 約 70 秒は欠番 0・`cpu_active` 40.8%。起動時に接続済み、`wifi scan` で接続は保たれる、送信中のスキャン 1 回で
  27/1048 バッチが落ちる（これまでと同じ水準）、`wifi disconnect` で切断、`wifi cred auto_connect` で 4 秒で再接続、`nina version` は `in use 0x00000007`、Telnet、
  `dmesg` にエラーなし、`fault` は異常停止なし。
- 確かめていない: `zperf`、`selftest`（ビルドは警告なし）、長時間。
- 段階 5 の材料（subagent の調査）: `zephyr/nina_socket.c` が Zephyr に求めるものは、ソケットのオフロードの登録、記述子の表（zvfs）、記述子ごとのロックを持つ呼び出しの
  振り分け、poll の 2 段の ioctl、DNS のオフロード、`net_` の型の名前、`k_heap`。互換層にあるのは `zsock` の名前の対応だけで、FreeRTOS の `nina_socket.c`（622 行）が
  直接実装している。互換層に足す量の見積もりは 400〜600 行。FreeRTOS 版が引き継ぐ違い: poll の失敗がそのソケットの `POLLERR` になる（いまは呼び出し全体が失敗）、
  `sendmsg` の間ずっとロックを持つ、DNS の結果が 2048 B の専用の領域から出る、記述子の番号が変わる。`k_heap` を互換層に足すかどうかは判断が要る。
- 実機: FreeRTOS 版（この変更のビルド）。

## NINA のリファクタリング: 段階 5（FreeRTOS の互換層に Zephyr のソケット層、ソケットのつなぎを Zephyr 版と共有）（2026-11-20、ユーザーの決定）

- 範囲（ユーザーの決定）: ソケット層の「実装の一覧」まで。FreeRTOS 版は `common/drivers/nina/zephyr/nina_socket.c`（Zephyr 版と同じファイル）をビルドする。
  `freertos/drivers/nina/nina_socket.c`（622 行）・`nina_netif.c`・`nina_net.h` は削除。`freertos/drivers/nina/` に残るのはボードに固有のもの
  （`nina_bus.c`、`nina_passthrough*`、`nina.cmake`）だけ。
- 互換層に足したもの（`freertos/drivers/compat/`。置き場とファイル名は Zephyr のツリーに合わせた）:
  - `subsys/net/lib/sockets/sockets.c`（`zsock_*` の振り分け）、`socket_offload.c`（DNS のオフロード）、`getaddrinfo.c`。
  - `lib/os/zvfs/zvfs_fdtable.c`（記述子の表、記述子ごとのロック）、`zvfs_poll.c`（poll の 2 段の ioctl）。
  - ヘッダー: `zephyr/sys/fdtable.h`、`zephyr/net/socket_offload.h`、`zephyr/net/net_ip.h`、`zephyr/net/socket.h`（`NET_SOCKET_OFFLOAD_REGISTER` ほか）。
  - `zephyr_heap.c`（`K_HEAP_DEFINE`・`k_heap_alloc`・`k_heap_free`）、`zephyr_net_init.c`、リンカーの `ld/section_extra_post_text.incl`（実装の一覧の区画）。
- 値: 記述子の表は 16、`CONFIG_ZVFS_POLL_MAX` は 16。
- FreeRTOS 版の動きの違い（Zephyr 版と同じになったもの）: poll の失敗がそのソケットの `POLLERR` になる、`sendmsg` の間ずっとロックを持つ、DNS の結果が専用の領域から出る、
  記述子の番号が変わる。poll の経路のスタックが約 504 B 深くなる。
- Telnet（`nano_telnet.c`）: リスナーの再開に失敗したら 1 秒ごとに再試行する形にした。
- ホストテスト: FreeRTOS 54 の `ok`（`tests/sockets` を新設。メインが実行）、NuttX 55・Zephyr 43（subagent）。`format.sh --check` も通る。
- ビルド（メインが再ビルド、警告なし）: Zephyr は `snapshot-1119c` と一致、NuttX は `snapshot-1119f` と一致。FreeRTOS 232872 → 236136 B（+3264 B）、RAM は約 +3 KB。
- 実機（FreeRTOS、各 1 回）:
  - 送信 約 70 秒: 欠番 0、`cpu_active` 41.0%、`unix_regressions` 0、`publish_max_us` 75155、`queue_high` 2。
  - 送信中に `wifi disconnect` → 6 秒後に `wifi cred auto_connect`: 3 秒で再接続、MQTT はつなぎ直す（基板の `connects` 2、受信器の接続 2 回）。落ちたのは切断の間の
    1 か所だけ（基板の `dropped_batches` 351、受信器の欠番 352、約 10.1 秒）。
  - Telnet: 送信中と、切断から戻ったあとの両方でつながり `version` に応答。
  - `zperf`（PC は `iperf3`、10 秒）: upload 3.58 Mbit/s（エラー 0）、download 2.47 Mbit/s。
  - `ntp status` は synchronized、`dmesg` に `<err>`・`<wrn>` なし、`fault` は異常停止なし。
  - スタックの残り（上のすべてのあとの `ps`）: `telnet_service` 1016、`shell_telnet` 1772、`zperf_server_tid` 872、`imu_raw_mqtt_tid` 848、最小は `nina_connect_tid` の 584。
- 確認の途中で Wi-Fi につながらない場面があったが、PC の AP が落ちていたため（AP を起動し直して解消。変更とは無関係）。
- 確かめていない: `selftest`、長時間、AP を落とす切断、記述子を使い切る場面（ホストテストだけ）、書き込み直後の最初の計測（AP が無く、最初の計測は AP の復帰後になった）。
- 実機: FreeRTOS 版（この変更のビルド）。

## NINA のリファクタリング: 段階 6（コアに残った Zephyr の定義の洗い出し）（2026-11-20）

- 結果: 直すものは無かった。`common/drivers/nina/` のコア（`nina_spi.c`・`nina_sock.c`・`nina_dns.c`・`nina_wait.c`・`nina_wifi.c` とヘッダー）は、
  `<zephyr/...>` を読まず、Zephyr の型・関数・マクロを使っていない。読むのは C ライブラリーと POSIX のソケットのヘッダー（`netinet/in.h`・`sys/socket.h`・`poll.h`・`netdb.h` など）だけ。
  `MIN()`・`CLAMP()` はコアが自分で定義している。ログ（`LOG_*`）も使っていない。
- 確かめ方: コアの 5 つの `.c` を、include パスをコアのディレクトリーだけにして PC の gcc で `-fsyntax-only -Wall`（`CONFIG_NINA_SPI_FREQUENCY`・`CONFIG_NINA_SEND_RETRY_MS`・
  `CONFIG_NINA_TCP_MAX_IO` を `-D` で与える）。すべて警告なしで通る。
- 残っている Zephyr の名前は 2 種類だけ: コメントの中の対応の説明（`k_mutex_lock()` が `nina_os_lock()` に当たる、など）と、ドライバー自身のオプションの記号 `CONFIG_NINA_*`（I2C の
  `CONFIG_I2C_DW_*` と同じ扱い）。
- これで NINA の段階 1〜6 は終わり。残りは `i2c_dw` を同じ構成（`common/drivers/i2c_dw/<OS>/`）へ移すこと、sensor、文書の整理。

## NINA: Zephyr 版の `nina_bus.c` をつなぎの側へ（2026-11-20、ユーザーの指摘）

- 段階 3 では SPI とピンを 3 版とも「ボードに固有」として各版のディレクトリーに残したが、Zephyr 版の `nina_bus.c` は devicetree から `spi1` と 4 本の線を引くだけで、
  ピン番号もチップのレジスターも持たない。Zephyr へのつなぎなので `common/drivers/nina/zephyr/nina_bus.c` へ移した（binding は前からそこの `dts/` にある）。
  FreeRTOS 版と NuttX 版の `nina_bus.c` は GPIO の番号と SPI1 を直接指定しているので、今の場所のまま。
- 中継モードとの関係: `nina_bus_ready()` が呼ぶ `nina_passthrough_active()` はアプリのもの（中継は RP2040 の UART を直接触るのでアプリに残る）。つなぎはアプリのヘッダーを
  読まず、自分で宣言して、常に偽を返す弱い定義を持つ。中継モードを持つアプリ（このアプリ）の定義がそれに取って代わる。
- `zephyr/app/usb-console/drivers/nina/` に残るのは `nina_passthrough.c`・`.h`・`_hw.h` だけ。ホストテストの偽の `zephyr/shell/shell.h`（`nina_fakes`）は要らなくなったので削除。
- ビルド: Zephyr のイメージは大きさが同じ（371340 B）で 11 バイトだけ違う。シンボルの表を前後で比べると、違いはリンカーが作る中継コード（veneer、フラッシュの RAM 上の関数への
  4 つ）の並び順と、それを指す参照だけ（ほかのシンボルの番地は全部同じ。コンパイラーが付ける局所の名前 `CSWTCH.29` が `.30` になっただけ）。弱い定義の区画が 1 つ増えて、
  リンカーの中での順番が変わったため。**Zephyr の基準を `snapshot-1120a` に更新**。NuttX は `snapshot-1119f` と一致。FreeRTOS は 236136 B のまま（コメントだけの変更）、警告なし。
- ホストテスト: Zephyr 43 の `ok`（`nina_boundary` は中継モードあり・なしの 2 通り。なしの側が弱い定義を通る）。`format.sh --check` も通る。
- 実機（Zephyr、書き込み直後の最初の計測、1 回）: 送信 約 70 秒は欠番 0、`cpu_active` 46.7%、`unix_regressions` 0。起動時に接続済み、`nina version` は `in use 0x00000007`、
  `dmesg` に `<err>`・`<wrn>` なし。
- 確かめていない: `nina passthrough`（中継モードに入ったときにバスが使えなくなること。ホストテストだけ）。
- 実機: Zephyr 版（この変更のビルド）。

## NINA: 中継モード（passthrough）をコアへ（2026-11-20、ユーザーの決定）

- ユーザーの方針: `common/drivers` のモジュールは単体で機能を提供する。`common/drivers/nina` は、中継でファームウェアを読み書きする機能も自分で持つ。
  これまでは中継の本体が 3 版に 1 本ずつあり、共有は判断の部分（`nina_passthrough_logic.h`）だけだった。
- 決めた形（案 A）: コアが中継の動きを持ち、境界を 2 つに分ける。チップに固有のモジュール側は各版に残す（Zephyr 版を Zephyr の UART・GPIO の API で書き直す案 B は見送り）。
  - コア `common/drivers/nina/nina_passthrough.c`・`.h`（C ライブラリーだけ）: 再起動の印、DTR・RTS とモジュールの線の対応、ボーレートの追従、16 KiB のバッファ、両方向のデータ、
    数の集計、LED、終了の条件と手順、`nina_passthrough_active()`。`nina_passthrough_logic.h` は取り込んで削除。
  - モジュール側の境界 `nina_passthrough_bus.h`（`nina_pt_bus_*`: 印と再起動、RESETN・GPIO0・UART の TX のピン、UART、LED）。実装は各版:
    `zephyr/app/usb-console/drivers/nina/nina_passthrough_bus.c`（もとの `nina_passthrough_hw.h` のレジスター操作）、`freertos/drivers/nina/nina_passthrough_bus.c`、
    NuttX のボードの `src/nina_passthrough_bus.c`。
  - PC 側の境界 `nina_passthrough_os.h`（`nina_pt_os_*`: 時計、待ち、USB シリアルへの書き込み、割り込みのうちつなぎの担当分、ログ）。実装はつなぎ:
    `common/drivers/nina/zephyr/`（CDC ACM と USB スタックの通知）、`freertos/`（TinyUSB と Pico SDK を直接。スケジューラーなしの 1 本のループ）、`nuttx/`（cdcacm と 2 本のスレッド）の
    `nina_passthrough_os.c`。
  - シェル: `nina passthrough` の本体は `common/shell/nina_shell.c`（Zephyr と FreeRTOS が共有。`nina_shell.h` は削除）。NuttX は `apps/nina` からコアの `nina_passthrough_request()` を呼ぶ。
  - Zephyr の `nina_bus.c` の弱い定義は削除（`nina_passthrough_active()` はコアのもの。`CONFIG_NINA_PASSTHROUGH` が無いビルドでは呼ばない）。
- テスト: コアのホストテスト `common/tests/nina_passthrough.c` を新設（3 版のテストにあった判断の部分を集め、起動の順序・両方向のデータ・tick・終了の手順を足した）。
  各版のテストはつなぎとコマンドだけに。`freertos/tests/passthrough/`・`nuttx/tests/nina_passthrough_fakes/` は削除。
- 動きの違い（subagent の報告。外から見える結果は同じ）: FreeRTOS は `nina passthrough confirm` のあとの再起動が約 100 ms 早い。Zephyr は `to_nina` を数える時点が UART の FIFO に
  入れたときになった。LED の点滅は「遅れたら追いつく」数え方になった（Zephyr・FreeRTOS）。NuttX は割り込みの中の順序と、次に寝る時間の計算の起点が変わった。
- ホストテスト（メインが実行）: Zephyr 44、NuttX 56、FreeRTOS 54 の `ok`。`format.sh --check` も通る。
- ビルド（メインが再ビルド、新しい警告なし）: Zephyr 371340 → 371908 B、NuttX 284388 → 284980 B、FreeRTOS 236136 → 236400 B。**基準を更新: Zephyr `snapshot-1120b`、NuttX `snapshot-1120a`**。
  中継モードなしの構成（Zephyr の `CONFIG_NINA_PASSTHROUGH=n`、NuttX のオプション OFF）もビルドできる（subagent が確認。中継のシンボルは 0 個）。
- 実機（3 版とも、各 1 回）: `nina passthrough confirm` で中継に入り、`esptool` 5.3.0 で NINA のフラッシュ 2 MB を 115200 bps（約 190 秒）と 921600 bps（約 26 秒）で読み出し。
  2 回の読み出しは一致し、2026-11-18 の書き込み後の退避とも一致。300 bps でポートを閉じて通常のファームウェアに戻る。バッファのあふれ（lost）は 3 版とも 0
  （最大の滞留は Zephyr 39、NuttX 18 バイト）。戻ったあとの送信 約 70 秒は 3 版とも欠番 0（`cpu_active`: Zephyr 46.5%、NuttX 48.8%、FreeRTOS 40.7%）。
- 確かめていない: 中継での書き込み（`write-flash`）、RESET ボタンでの終了、中継なしの構成の実機での動作、3 版のモジュール側と FreeRTOS・NuttX のつなぎ（ホストテストが無い。前から）。
- 実機: FreeRTOS 版（この変更のビルド）。

## NINA: `nina_cmd.h` を `nina_proto.h` へ（2026-11-21）

- ユーザーの指摘: `common/drivers/nina/` のヘッダーが 12 個あって役割が分かりにくい。無理のないものだけまとめる。
- `nina_cmd.h`（プロトコルのバイトの名前だけ）を `nina_proto.h`（`nina_spi.c` の要求の関数と機能ビット）に入れて削除。`nina_proto.h` はもともと `nina_cmd.h` を読んでいた。
  `nina_spi.c` が関数の途中で読んでいた `nina_proto.h` は先頭の include へ。文書（Zephyr のドライバー説明、設計書、NuttX の説明、日英）も合わせた。
- 確認: イメージは 3 版とも変更前とバイト一致（Zephyr・NuttX は基準、FreeRTOS は前後を同じ `SOURCE_DATE_EPOCH` で）。ホストテストは 3 版とも通る。実機には書いていない。

## NINA: `common/drivers/nina/` を 3 層に分ける（2026-11-21、ユーザーの決定）

- ユーザーの案（ヘッダーが多くて役割が分かりにくい、から）: 直下はドライバーが提供する機能、`internal/` はコアの内側の部品、`port/` は各版が実装する境界とその実装。
  - 直下: `nina_wifi.[ch]`・`nina_sock.[ch]`・`nina_dns.[ch]`・`nina_passthrough.[ch]`・`nina_sockopt.h`（MQTT のコードと NuttX の互換層が読むので直下）。
  - `internal/`: `nina_proto.[ch]`・`nina_wait.[ch]`。**`nina_spi.c` は `nina_proto.c` に改名**（ヘッダーと対にする。ホストテストも `common/tests/nina_proto.c`、実行名 `nina_proto`）。
    関数名・ログのモジュール名は変えていない。
  - `port/`: 境界のヘッダー 4 つ（`nina_os.h`・`nina_bus.h`・`nina_passthrough_os.h`・`nina_passthrough_bus.h`）と、つなぎ `port/zephyr/`・`port/nuttx/`・`port/freertos/`。
  - **チップに固有の実装も `port/<OS>/` へ移し、名前に `_rp2040` を付けた**（ユーザーの指示）: Zephyr の `nina_passthrough_bus_rp2040.c`（アプリの `drivers/nina/` は無くなった）、
    FreeRTOS の `nina_bus_rp2040.c`・`nina_passthrough_bus_rp2040.c`（`freertos/drivers/nina/` は `nina.cmake` だけ）、NuttX の同じ 2 つ（ボードの `src/` から。ビルドはボードの
    `src/Make.defs` のまま、VPATH で読む。リンクの順序を変えないため）。Zephyr の `nina_bus.c` はデバイスツリーで書かれているので名前はそのまま。
  - include: 3 版とホストテストの include パスに `internal` と `port` を足し、ドライバーの中の `#include "nina_os.h"` などはそのまま。ドライバーの外から `internal/` を読む所が 3 つ残る:
    `common/shell/nina_shell.c` と NuttX の `apps/nina`（`internal/nina_proto.h`。`nina version` のため）、NuttX の互換層の `<zephyr/net/socket.h>`（`internal/nina_wait.h`。
    Zephyr 版の待ちのループを互換層で作るため）。前の 2 つは、版と機能ビットの関数を直下に出して無くす予定。
- 実装は subagent（opus）。**途中で止まった**: ユーザーが vim で開いていたファイル（改名の途中の `internal/nina_spi.h`）のスワップファイルを、subagent の一括置換（未追跡のファイルも対象にしていた）が
  2 回書き換えた。subagent はそこで止めて報告。スワップファイルは、メインが確かめた時点では無くなっていた（ユーザーが vim を開き直していた）。残り（共有のホストテストの `run.sh`、NuttX の互換層の include、
  コメント、文書）はメインが仕上げた。同じ一括置換が `PORTING_STATUS.md` の古い節の 1 行も書き換えていたので戻した。**以後、一括置換は `git ls-files`（追跡しているファイル）だけを対象にし、`.*.sw?` を除く。**
- イメージ: Zephyr・NuttX は基準とバイト一致。FreeRTOS は 236400 → 236408 B（`nina_os.c` の `__FILE__` の文字列が `port/` の分だけ長くなった。イメージの中の文字列で確認。それ以外の差は調べていない）。
- ホストテスト: Zephyr 44、NuttX 56、FreeRTOS 54 の `ok`。`format.sh --check` も通る。
- 実機（FreeRTOS 版、1 回）: 書き込み直後の最初の計測 約 70 秒で欠番 0、`unix_regressions` 0、`cpu_active` 40.7%。中継・切断復帰・Telnet はこのイメージでは確かめていない。
- 実機: FreeRTOS 版（この変更のビルド）。

## NINA: 関数の接頭辞をファイル名に合わせる（2026-11-21、ユーザーの指摘）

- `nina_dns.h`: `nina_getaddrinfo()`・`nina_freeaddrinfo()` → `nina_dns_getaddrinfo()`・`nina_dns_freeaddrinfo()`。
- 中継の境界: `nina_pt_bus_*` → `nina_passthrough_bus_*`、`nina_pt_os_*` → `nina_passthrough_os_*`、`NINA_PT_*` → `NINA_PASSTHROUGH_*`、`struct nina_pt_counts`・`enum nina_pt_gpio0` も同じ。
  NuttX のスレッドの名前（`nina_pt_rx`・`nina_pt_tx`。`ps` に出る文字列）と優先度の `NANO_PRIO_NINA_PT` は変えていない。
- 残っている不一致: `internal/nina_proto.h` の関数（`nina_request()`・`nina_ip_config()`・`nina_capabilities()` など。`nina_proto_` が付いていない）と、そこで宣言している `nina_dns_init()`。
- 確認: イメージは 3 版とも変更前とバイト一致（Zephyr・NuttX は基準、FreeRTOS は前後を同じ `SOURCE_DATE_EPOCH` で）。ホストテスト 44・56・54、`format.sh --check` も通る。実機には書いていない
  （実機のイメージは 1 つ前の commit のビルドで、中身は同じ）。

## NuttX 版のソケットを NuttX の標準の経路へ（案 B）（2026-11-21、ユーザーの決定）

- きっかけ: 3 層に分けたあと、NuttX の互換層の `<zephyr/net/socket.h>` がドライバーの `internal/nina_wait.h` を読んでいた。ユーザーの指摘「この設計が間違ってないですか？」。
  もとの形: 互換層のインライン関数が、Zephyr 版のポーリングのループを NINA 独自のソケットオプション（`NINA_SO_CALLER_POLLS`・`NINA_SO_POLL`）と `nina_wait()` で作っていた
  （モジュールから見たコマンドを Zephyr 版と同じにするため）。汎用の互換層が特定のドライバーを知り、待ち方がドライバーの外にあり、待ちの仕組みが 2 つあった。
- 変更（subagent、メインがレビュー）: `zsock_socket()`・`zsock_poll()`・`zsock_recv()` を NuttX の `socket()`・`poll()`・`recv()` の読み替えだけにした。待ちはつなぎ（usrsock と事象スレッド
  `nina_poller.c`）が受け持つ。互換層が読む NINA のヘッダーは `nina_dns.h` だけ。
  - 削除: `nina_sockopt.h`、コアの `caller_polls` とその分岐、`nina_sock_fionread()`、usrsock の GETSOCKOPT と FIONREAD の処理、Zephyr のつなぎの `NINA_SOL` の拒否、それらのテスト。
    `NINA_POLL_INTERVAL_MS` は `internal/nina_wait.h` へ。
  - `nina_usrsock.c` に 2 つ: TCP の受信が要求の長さいっぱいに読めたときは RECVFROM_AVAIL を残す（MQTT の CONNACK を 2 バイトずつ読むため。事象スレッドは MQTT のスレッドより
    優先度が低く、2 回の読み出しのあいだに印を立て直せない）。UDP のソケットの最初の送信で事象スレッドを起こす（NTP は `connect()` なしの `sendto()` で、返事の事象を予約する問い合わせがほかに無い）。
  - Zephyr 版との違い（NuttX の `poll()`）: 接続していない TCP は POLLIN|POLLOUT|POLLHUP、モジュールのエラーや問い合わせの失敗は POLLERR ではなく POLLHUP。`service_input()` はどちらも -EIO。
  - NuttX のテスト `imu_raw_mqtt`・`time_sync` は、usrsock の `poll()`・`recv()` の偽物（RECVFROM_AVAIL を持ち、ドライバーの残す・落とすの規則に従う）の上に書き直し。
- あわせて、MQTT の接続が失敗したときのログを 1 行足した（ユーザーの了解。`common/imu/imu_raw_mqtt.c`、3 版）: `<wrn> imu_raw_mqtt: MQTT <段階> failed: <エラー>`。段階は
  `connect`・`info`・`status`・`publish`・`input`。同じ段階・同じエラーが続くあいだは 1 回だけ。
- イメージ: Zephyr 371908 → 372120 B、NuttX 284980 → 283724 B、FreeRTOS 236408 → 236520 B。**基準を更新: Zephyr・NuttX とも `snapshot-1121a`**。ホストテスト 44・56・54、`format.sh --check` も通る。
- 実機（NuttX 版、各 約 70 秒）:
  - 変更前 24 回（同じ日）: 欠番 0、到着の最大の空き 221 ms、NTP の往復の遅れ（中央値）6.3〜6.9 ms、`cpu_active` 49.0%（2 回）。
  - 変更後 64 回: 62 回は欠番 0、`cpu_active` 48.1〜48.5%、NTP の遅れ 6.3〜7.6 ms、時刻の逆行 0。**午前の 24 回のうち 2 回で欠番**: 1 回は開始 約 1 秒で MQTT の接続が失敗してつなぎ直し（欠番 37、
    キューは 3 まで）、1 回は到着が 387 ms 空いてキューがあふれた（欠番 2、接続はそのまま）。そのあとの 40 回と、開始 6 秒で止める短い計測 149 回では出ていない。**原因は分かっていない**
    （変更のせいか、その時間帯の電波かを切り分けられていない。ログの行はこのあとに足したので、次に起きれば段階とエラーが残る）。
  - 短い計測の 150 回のうち 1 回の失敗（publish -32、connect -104）は計測側の都合（受信器が 600 秒で終了した）。
  - 送信中の `wapi disconnect wlan0` → `wifi cred auto_connect`: 2 秒で再接続、MQTT はつなぎ直し（1 回）。Telnet は送信中と復帰後に応答（各 1 回）。
- 実機（Zephyr 版、FreeRTOS 版、書き込み直後の各 1 回 約 70 秒）: 欠番 0、時刻の逆行 0、`cpu_active` 45.7%・40.5%。この 2 版で変わるのはコアの削除分とログの行だけ。
- 確かめていない: 事象の無いファームウェア（3.0.2）での NuttX 版（NTP の返事に気づくのが最大 20 ms 遅れうる）、長時間、AP を落とす切断、`zperf`、スタックの使用量（MQTT のスレッドにログの呼び出しが増えた）。
- 実機: FreeRTOS 版（この変更のビルド）。

## NINA: 公開のヘッダーを `include/` へ、`nina_proto_` の接頭辞（2026-11-21、ユーザーの案）

- `common/drivers/nina/include/` に、外から使うヘッダー 4 つ（`nina_wifi.h`・`nina_sock.h`・`nina_dns.h`・`nina_passthrough.h`）を移した。直下は `.c` だけ。3 版のビルドとホストテストの
  include パスは `include`・`internal`・`port`（ドライバーの中）で、NuttX のアプリ（`apps/imu`・`apps/nina`・`apps/init`）は `include` だけ（`apps/init` は `port/nuttx` の `nina_net.h` も）。
- ドライバーの外から `internal/` を読む所を無くした: `nina version`（`common/shell/nina_shell.c`、NuttX の `apps/nina`）は、新しい `nina_wifi_fw_version()`・`nina_wifi_capabilities()`
  （`nina_wifi.h`）を呼ぶ。表示は同じ。
- `internal/nina_proto.h` の関数に接頭辞: `nina_request()` → `nina_proto_request()`、`nina_ip_config()`・`nina_capabilities()`・`nina_module_capabilities()`・`nina_probe_capabilities()` も同じ。
  `nina_dns_init()` の宣言は `nina_dns.h` へ（Zephyr 版の `nina_net.c` が `<netdb.h>` を読めるよう、CMake で POSIX の名前の include パスを足した）。
- 残っている例外: ボードの起動のコード（FreeRTOS の `main.c`、NuttX の `nano_bringup.c`）が `port/nina_bus.h` を読む（`nina_bus_initialize()` を呼ぶため）。
- イメージ: Zephyr 372120 → 372152 B、NuttX 283724 → 283748 B、FreeRTOS 236520 → 236552 B（公開の関数 2 つの分）。**基準を更新: Zephyr・NuttX とも `snapshot-1121b`**。
  ホストテスト 44・56・54、`format.sh --check` も通る。
- 実機（3 版、書き込み直後の各 1 回 約 70 秒）: `nina version` は `3.0.2`、`0x00000007 (in use 0x00000007)`（3 版とも、以前と同じ表示）。送信は欠番 0、時刻の逆行 0、
  `cpu_active` Zephyr 46.1%、NuttX 48.6%、FreeRTOS 40.3%。中継・切断復帰・Telnet はこのイメージでは確かめていない。
- 実機: Zephyr 版（この変更のビルド）。

## NINA: バスの初期化とつなぎのオブジェクトをドライバーの中へ、型とマクロの接頭辞（2026-11-21、ユーザーの指摘）

- ユーザーの指摘: 起動のコード（FreeRTOS の `main.c`、NuttX の `nano_bringup.c`）が `nina_bus_initialize()` を呼ぶために `port/nina_bus.h` を読んでいた。「ninaドライバの初期化から呼び出しちゃだめなんだっけ？」
  Zephyr 版にはこの関数が無かった（SPI1 と CS は Zephyr 本体がデバイスツリーから起動時に設定する）。FreeRTOS 版・NuttX 版は、その時点を写して起動時に呼んでいただけ。SPI1 は RP2040 がマスター。
- 変更: コアの `init_nina()`（`nina_proto.c`。最初の要求のとき）が `nina_bus_initialize()` を呼ぶ。0 で使える（2 回目からは何もしない）、使えないときは `-ENODEV`。
  `nina_bus_ready()` は境界から削除（Zephyr のつなぎはその中身を `nina_bus_initialize()` として持つ）。NuttX のものは中継モードのあいだ `-ENODEV`。起動のコードの呼び出しと include は削除。
- FreeRTOS のつなぎのオブジェクト（ミューテックス 3 つ、セマフォ 3 つ。静的）は、どれかを最初に使うときに作る（`nina_os.c` の `objects()`。クリティカルセクションの中で 1 回だけ）。
  `nina_os_initialize()` と `nina_freertos.h` は削除、`main.c` の呼び出しも削除。アプリは起動時にドライバーの関数を何も呼ばない。
- 接頭辞をヘッダーの名前に合わせた（関数に続いて、型とマクロも）: `nina_pin_*`・`enum nina_pin`・`NINA_PIN_*` → `nina_bus_pin_*`・`NINA_BUS_PIN_*`、`struct nina_param` → `nina_proto_param`、
  `struct nina_passthrough_counts` → `nina_passthrough_os_counts`、`NINA_PASSTHROUGH_LOG_*` → `NINA_PASSTHROUGH_OS_LOG_*`、`enum nina_passthrough_gpio0`・`NINA_PASSTHROUGH_GPIO0_*`・
  `NINA_PASSTHROUGH_UART_*`・`NINA_PASSTHROUGH_MARK_WORDS` → `nina_passthrough_bus_*`・`NINA_PASSTHROUGH_BUS_*`。`nina_proto.h` のプロトコルの定数（`NINA_CMD_*` など）はファームウェアの名前のまま。
- イメージ: Zephyr 372152 → 372144 B、NuttX 283748 → 283676 B、FreeRTOS 236552 → 236496 B（改名の前後では同じ大きさ）。**基準を更新: Zephyr・NuttX とも `snapshot-1121c`**。
  ホストテスト 44・56・54、`format.sh --check` も通る。
- 実機（3 版、書き込み直後の各 1 回 約 70 秒）: 起動後に Wi-Fi に接続、`nina version` は以前と同じ表示、送信は欠番 0、時刻の逆行 0、`cpu_active` Zephyr 46.2%、NuttX 49.2%、FreeRTOS 40.2%。
  FreeRTOS は `dmesg` に `<err>`・`<wrn>` なし、`fault` なし。NuttX 版で中継モードの出入り（1 回）: 2 MB の読み出しが 115200・921600 bps で一致、書き込み後の退避とも一致、抜けたあと `nina version` が応答。
- 確かめていない: Zephyr 版・FreeRTOS 版の中継、切断復帰・Telnet（このイメージでは）、モジュールが応答しないときの起動。
- 実機: Zephyr 版（この変更のビルド）。

## NINA: codex のレビューの 5 点（2026-11-21）

ユーザーが codex にレビューを依頼した結果。5 点ともコードで確かめて指摘のとおりだったので、すべて直した。

1. **SPI のバスをほかのデバイスと安全に共有できない**（高）: コアは CS を下げたまま複数回の転送をするのに、NuttX のつなぎは転送ごとにバスのロックを放し、モードと周波数は最初の 1 回だけ設定していた。
   → 境界に `nina_bus_select()`・`nina_bus_deselect()` を足し、コアは CS をこれで上げ下げする（要求 1 回に 2 度: コマンドと応答）。NuttX は `SPI_LOCK` を取ってモード・ビット・周波数を毎回設定してから
   CS を下げ、CS を上げてから放す。Zephyr は `spi_config` に `SPI_LOCK_ON` を付け、バッファなしの `spi_transceive()` でロックを取ってから CS を下げ、CS を上げてから `spi_release()`。
   FreeRTOS は SPI1 のほかの利用者が無く Pico SDK にバスのロックも無いので、CS だけ。
2. **Zephyr のデバイスツリーで接続先を変えても反映されない**（高）: SPI のコントローラーを `spi1`、CS をその `cs-gpios[0]` に固定していた。→ NINA のノード
   （`DT_COMPAT_GET_ANY_STATUS_OKAY(arduino_nina_spi)`）から、親のバス（`DT_BUS`）、`reg` に対応する CS、`spi-max-frequency` を取る。`CONFIG_NINA_SPI_FREQUENCY` は Zephyr の Kconfig から削除
   （NuttX と FreeRTOS には残る）。`nina_os.c`・`nina_net.c`・中継のモジュール側の `BUILD_ASSERT` も同じノードから。互換層に `DT_COMPAT_GET_ANY_STATUS_OKAY()` を足した（FreeRTOS 版が `nina_net.c` をビルドするため）。
   （ユーザーの質問「起動時に OS 側で設定してくれるのでは」: ハードウェアの設定は Zephyr がするが、「どのバスのどの CS を使うか」をドライバーが固定していた、という指摘。）
3. **NuttX で同じ信号のピン番号をボードとつなぎが別々に持つ**（中）: `nina_os.c` が GPIO10・GPIO2 を直に書いていた。→ `board.h` の `BOARD_NINA_READY_GPIO`・`BOARD_NINA_IRQ_GPIO` を読む。
   FreeRTOS も同じ重複があったので、`port/freertos/nina_board_rp2040.h` に 1 つにまとめた（`nina_bus_rp2040.c` と `nina_os.c` が読む）。
4. **Zephyr のつなぎがアプリの再接続機能を必須にしている**（中）: `nina_net.c`（と NuttX の `nina_wifi_mgmt.c`）が `common/imu` の `wifi_profile_*` を直接参照していた。→ 3 つのフックを
   `nina_wifi_ops` から外し、アプリが登録する `nina_wifi_set_reconnect_ops()`（`nina_wifi.h`）にした。`wifi_profile_init()` が登録する。つなぎは `wifi_profile.h` を読まない。
   NuttX の `nina_wifi_mgmt_*` の 3 つの中継関数と、`CONFIG_NANO_WIFI_PROFILE` の有無で 2 回ビルドしていたテストの変種は削除。
5. **ソケットの公開ヘッダーと内部の境界があいまい**（中）: `include/nina_sock.h` が `internal/` の 2 つを読み、usrsock 用の `connect_xid` とソケットの表も見せていた。使うのはつなぎだけ。
   → `internal/nina_sock.h` に戻した。`include/` は `nina_wifi.h`・`nina_dns.h`・`nina_passthrough.h` の 3 つ。`common/tests/run.sh` に `nina_public_headers`（`include/` だけでコンパイルできること）を足した。

- イメージ: Zephyr 372144 → 372216 B、NuttX 283676 → 283676 B（大きさは同じ、中身は違う）、FreeRTOS 236496 → 236496 B（同じく）。**基準を更新: Zephyr・NuttX とも `snapshot-1121d`**。
  ホストテスト 45・57・55（`nina_public_headers` が増えた）、`format.sh --check` も通る。
- 実機（3 版、書き込み直後の各 1 回 約 70 秒）: 欠番 0、時刻の逆行 0、`cpu_active` Zephyr 46.6%、NuttX 48.8%、FreeRTOS 39.8%。送信中の切断 → `wifi cred auto_connect`: FreeRTOS・NuttX とも 2 秒で再接続し MQTT も
  つなぎ直し、Telnet は送信中と復帰後に応答（各 1 回）。Zephyr は送信していない状態で `wifi disconnect` → `wifi cred auto_connect` が 2 秒で再接続、`dmesg` に `<err>`・`<wrn>` なし（1 回）。
- 確かめていない: SPI1 を別のデバイスと共有した状態（このボードには無い）、別の配線のデバイスツリー、AP を落とす切断（リンクが切れたときの再接続の要求）、中継（このイメージでは）。
- レビューの最後の点（Wi-Fi の状態とソケットの表のインスタンス化）は、NINA を 2 台扱う要件が無いので手を付けていない。
- 実機: Zephyr 版（この変更のビルド）。

## NINA: codex の再レビューの 2 点（2026-11-21）

`d9fd4b8` の再レビュー。前回の 5 点は対応済みとの判定で、残りの 2 点を直した。

1. **Zephyr: 空の転送が成功しても、バスのロックを取ったことにならないコントローラーがある**（高）: `nina_bus_select()` はバッファなしの `spi_transceive()` でロックを取ってから CS を下げる。
   PL022 のドライバー（ポーリング）は先にロックを取るが、同梱の LiteX のドライバーは両方のバッファが NULL だとロックの前に 0 を返す。Zephyr にはロックだけを取る呼び出しが無い。
   → このやり方をポーリングの PL022 に限定した: `BUILD_ASSERT(DT_NODE_HAS_COMPAT(NINA_BUS, arm_pl022))` と、`CONFIG_SPI_PL022_INTERRUPT`・`_DMA` が無いことの `BUILD_ASSERT`。
   ほかのコントローラーで使うときは、ここにそのコントローラーでのバスの取り方を足す（コメントに記載）。
2. **再接続のフックの「いつでも登録できる」に同期が無い**（中）: `nina_wifi_set_reconnect_ops()` は普通のポインターに代入し、接続・切断・監視のスレッドがそれを読んでいた。
   → ポインターを `_Atomic` にして `atomic_store()`・`atomic_load()` で読み書きする（RP2040 ではロックなしの読み書きとメモリーバリア）。契約をヘッダーに書いた: フックは 1 回の呼び出しごとに
   読んだ登録を使うので、差し替えた直後にも前の登録のフックが動きうる。登録する構造体と関数は、ずっと有効でなければならない（静的）。

- イメージ: Zephyr 372216 → 372248 B、NuttX 283676 → 283708 B、FreeRTOS 236496 → 236528 B（バリアの分）。**基準を更新: Zephyr・NuttX とも `snapshot-1121e`**。ホストテスト 45・57・55。
- 実機（3 版、書き込み直後の各 1 回 約 70 秒）: 欠番 0、`cpu_active` Zephyr 46.8%、NuttX 48.7%、FreeRTOS 39.8%。切断 → `wifi cred auto_connect` は 3 版とも 2 秒で再接続
  （NuttX・FreeRTOS は送信中で MQTT もつなぎ直し、Telnet は送信中と復帰後に応答。Zephyr は送信していない状態）。
- 確かめていない: 共有の SPI（実機に相手が無い）、PL022 以外のコントローラー、AP を落とす切断。
- 実機: FreeRTOS 版（この変更のビルド）。

## NINA: codex の 3 回目のレビューの 1 点（2026-11-21）

- `b9c4a98` の確認。再接続フックの同期は解消との判定。残り 1 点: Zephyr で `CONFIG_SPI_ASYNC=y`（`CONFIG_SPI_PERIPHERAL=n`）だと、ポーリングの PL022 でも、同期の `spi_transceive()` が戻るときに
  `spi_context_release()` が `SPI_LOCK_ON` に関係なくロックを返す（`spi_context.h` で確認）。`CONFIG_SPI_ASYNC` は PL022 の割り込み・DMA と独立なので、前回の `BUILD_ASSERT` を通ってしまう。
- 対応: `BUILD_ASSERT(!IS_ENABLED(CONFIG_SPI_ASYNC))` を足し、コメントに理由を書いた。この版は `CONFIG_SPI_ASYNC` なし。
- 確認: Zephyr のイメージは基準（`snapshot-1121e`）とバイト一致、ホストテスト 45。Zephyr のつなぎだけの変更なので NuttX・FreeRTOS はビルドしていない。実機には書いていない。

## I2C: `i2c_dw` を NINA と同じ形へ、FreeRTOS の部分を FreeRTOS で直接（2026-11-21、ユーザーの依頼）

- 配置（subagent が実装、メインがレビュー・再ビルド・実機確認）:
  - `common/drivers/i2c_dw/i2c_dw.c`（コア。1 つの翻訳単位で、OS の部分のヘッダーを取り込む作りはそのまま）。
  - `include/`: `i2c_dw_rp2040_dma.h` だけ（`i2c_dma` コマンドと自己診断が、互換層の `<zephyr/drivers/i2c/i2c_dw_rp2040_dma.h>` を通して読む）。
  - `internal/`: `i2c_dw.h`・`i2c_dw_registers.h`・`i2c_dw_rp2040.h`・`i2c_dw_rp2040_dma_priv.h`。
  - `port/`: 境界 `i2c_dw_os.h`・`i2c_dw_dma_os.h` と、`port/freertos/`（`freertos/drivers/i2c_dw/` から）・`port/nuttx/`（ボードの `src/` から）: `i2c_dw_os_impl.h`・`i2c_dw_instance.h`・
    `i2c_dw_controllers_rp2040.h`・`i2c_dw_dma_os_rp2040.c`。Zephyr 版は上流のドライバー + パッチなので `port/zephyr/` は無い。
  - `freertos/drivers/i2c_dw/` に残るのは `i2c_dw.cmake` と自己診断（`i2c_dw_selftest.[ch]`）。NuttX はボードの `src/Make.defs` が VPATH で読む（リンクの順序を保つため）。
  - コアの名前は変えていない（Zephyr のドライバーの写し。`i2c_dw_sync.py` の期待値は変わらず、読む場所だけ直した）。パッチ 0003 のコメントに古いパスが 2 か所残る（パッチには触れていない）。
  - 外から `port/<OS>/` を読む所: 起動のコード（FreeRTOS の `main.c`、NuttX の `nano_bringup.c`）が `i2c_dw_i2c0_initialize()`・`_i2c1_` を呼ぶ。戻り値の型が OS ごとに違う
    （FreeRTOS は `int`、NuttX は `struct i2c_master_s *`）ので `include/` に置けない。デバイスの登録なので、起動のコードから呼ぶ形は変えていない。
  - `common/tests/run.sh` に `i2c_dw_public_headers`（`include/` だけでコンパイルできること）を足した。
- FreeRTOS の部分（`port/freertos/i2c_dw_os_impl.h`）: 互換層の `k_sem`・`k_msleep`・`k_uptime_get` をやめ、FreeRTOS と Pico SDK で直接書いた。
  - 1 回目はキューで作られた FreeRTOS のセマフォ（`xSemaphoreCreateCountingStatic`）。実機で `cpu_active` が 39.8〜40.3% → 41.3%（3 回とも）に上がった。互換層の `k_sem` は
    タスク通知で作られていて、それより重い。
  - 2 回目（採用）: 3 つとも FreeRTOS のタスク通知で直接。型は数・上限・待っているタスクの並び（待つ側のスタックの上）。give は待っているタスクに数を渡して
    `xTaskNotifyGiveIndexed()`（割り込みの中では `FromISR` と `portYIELD_FROM_ISR()`）、take は数を取るか通知を待つ（結果は done の印で決める。別の通知で早く起きても残りの時間を待つ）。
    通知の番号はドライバー専用の 3（`configTASK_NOTIFICATION_ARRAY_ENTRIES` を 3 → 4。TCB のパディングに収まり RAM は増えない）。バスのロックの待ちは優先度順。
  - タイムアウトの tick の数は同じ（1000 ms → 1001 tick、1 ms → 2 tick、`msleep(1)` → 2 tick。ホストテストが確かめる）。オブジェクトはインスタンスの開始（`i2c_dw_os_initialize()`）で作る。
- イメージ: Zephyr・NuttX は基準（`snapshot-1121e`）とバイト一致。FreeRTOS は移動だけではバイト一致、タスク通知版で 236528 → 236968 B。ホストテスト 46・58・56、`format.sh --check` も通る。
- 実機（FreeRTOS 版）:
  - 送信 約 70 秒 × 4 回（うち 1 回は書き込み直後）: 欠番 0、時刻の逆行 0、`cpu_active` 40.9〜41.0%、I2C のエラー 0、`i2c_dma stats` の `dma_errors`・`dma_fallbacks` 0。
    直前のビルド（39.8〜40.3%）より 0.6〜1.2 ポイント高いが、これまでの別のイメージの幅（39.7〜41.0%）の中で、配置の違いと区別できていない。
  - 送信中の `wifi disconnect` → `wifi cred auto_connect`: 2 秒で再接続、MQTT もつなぎ直し、Telnet は送信中と復帰後に応答（1 回）。`sensor get` も応答。
  - スタックの空き: `imu_fifo_worker` 1368、`imu_stream_tid` 1208、割り込み 1592（前は 1416、1176、1592）。
  - `-DNANO_SELFTEST=ON` のビルドで `selftest`: カーネル 22、I2C 6（標準・fast plus・拒否・NACK・DMA）、LSM6DSOX 3 項目、失敗 0。
- 確かめていない: `-DI2C_DW_I2C1=ON` の実機（ビルドだけ）、NuttX 版の実機（イメージが同じなので書いていない）、長時間。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## I2C: codex のレビューの 7 点のうち 4 点（2026-11-21）

`i2c_dw` の移し替えのあとの codex のレビュー。7 点ともコードで確かめた。つなぎの側（このリポジトリで書いた部分）の 4 点を直し、Zephyr の上流のドライバーから引き継いでいる 3 点は
ユーザーの判断を待つ。

- 直した 4 点:
  - **NuttX: 割り込みの登録に失敗してもデバイスが ready になる**（中）: `irq_attach()` の戻り値を捨てていた。→ 境界の `i2c_dw_os_irq_enable()` が `int` を返し、失敗したら割り込みを有効にせず、
    コアの `i2c_dw_turn_on()` がそのエラーを返す（インスタンスは ready にならない）。FreeRTOS のものは 0 を返す。
  - **NuttX: 最後のメッセージの `I2C_M_NOSTOP` が黙って無視される**（中）: 呼び出しをまたいでバスを持ち続けることはできないので、送る前に `-ENOTSUP` で断る。
  - **FreeRTOS: 速度の読み取りと転送の開始が同じバスのロックで守られていない**（中）: `i2c_dw_transfer_msgs()` が速度を値ではなくポインターで受け取り、バスのロックを取ってから読む
    （Zephyr のドライバーと同じ順序）。NuttX は転送ごとの周波数なので、手元の変数のアドレスを渡す。`i2c_dw_sync.expected` をこの 3 行の分だけ `--update` で更新。
  - **NuttX: クロックの定義が `BOARD_PERI_FREQ`**（高の指摘の一部）: RP2040 の I2C は `clk_sys` で動くので `BOARD_SYS_FREQ` にした（どちらも 125 MHz で、値は変わらない）。
- ホストテスト（NuttX の `i2c_dw_instance`）に 2 つ足した: `irq_attach()` が `-ENOMEM` を返すとデバイスが得られず割り込みも有効にならないこと、最後のメッセージが NOSTOP の転送は何も送らず `-ENOTSUP` になること。
- 判断待ちの 3 点（どれも Zephyr の上流の `i2c_dw.c` の動きで、直すならパッチ 0003 とコアの両方を変えることになる）と、NuttX の DMA の 1 点:
  - 0 バイトの転送が 1 バイトの読み書きになる（上流のコメントは「I2C スキャナーのための回避策」）。
  - SCL の数え方（上流の `I2C_FSP_HCNT`・`I2C_FSP_LCNT`。125 MHz・1 MHz 指定で 31/31）が指定の周波数を超える。実機の記録（3584 バイトの読み出しが 22.9 ms）からの下限の見積もりは約 1.41 MHz。
    波形は測っていない。直すと FIFO の読み出しの時間が延びる（6667 Hz の余裕に関わる）。
  - バスの復旧が何もせず 0 を返す（上流でも、SoC のラッパーが登録しないと同じ）。
  - NuttX: `rp2040_dmachannel()` は空きが無いと待ち続けるので、DMA のチャネルが 1 本しか空いていないと I2C の初期化が止まる。NuttX 本体に待たない取り方が無く、直すには NuttX へのローカルのパッチが要る。
    このビルドでは起動時にほかの利用者がいない。
- イメージ: Zephyr は基準とバイト一致。NuttX 283708 → 283756 B（**基準を `snapshot-1121f` に更新**）、FreeRTOS 236968 → 236952 B。ホストテスト 46・58・56。
- 実機（書き込み直後の各 1 回 約 70 秒）: NuttX 欠番 0、48.3%、`dma_errors`・`dma_fallbacks` 0。FreeRTOS 欠番 0、40.8%、同じく 0、`sensor get` 応答。
- 実機: FreeRTOS 版（この変更のビルド）。

## I2C: codex のレビューの残り（0 バイト転送、復旧の要求、NuttX の DMA）、SCL は直さない（2026-11-21、ユーザーの判断）

- ユーザー: 「はじめの 3 つも修正してください」→ そのあと「クロックは規格内に入ってないのを承知で今の設定にしていたと思うので、そのことを言っているようなら修正なしでもよい」。
  codex の判断: SCL は測って規格内へ、0 バイト転送は 3 版とも拒否、明示的な復旧の要求は未対応を返し通常の転送は再試行できるまま、NuttX の DMA は待たない取り方を足す。
- **SCL は直さない**: 1 MHz 指定での数え方（上流の `I2C_FSP_HCNT`・`_LCNT`: 125 MHz で 31/31 → High 39 クロック 312 ns、Low 32 クロック 256 ns）は LSM6DSOX の規格（1 MHz 以下、Low 0.5 µs 以上）の外。
  記録（3584 バイト 22.9 ms）からの見積もりは約 1.4 MHz（周期 約 710 ns。数え方の 568 ns との差 約 142 ns が立ち上がりなど）。6667 Hz の FIFO のデータは毎秒 約 89.3 万クロックで、
  Low 504 ns・周期 1000 ns 以上にすると実際は約 0.88 MHz になり足りない。設計上の選択として、設計書（日英）の「実機で確認した範囲」に書いた。波形は測っていない。
- 直した 3 点（subagent が実装、メインがレビュー・ホストテスト・実機確認）:
  - **長さ 0 のメッセージは `-ENOTSUP`**（3 版）: 上流の「I2C スキャナーのための回避策」（長さ 0 を 1 バイトにする）を削り、転送の入口で全メッセージを先に調べる（途中に長さ 0 があっても、
    前のメッセージを送る前に断る）。Zephyr はパッチ 0003 を作り直した（一時的な index で `git diff --cached`。`patches.yml` の sha256 も更新。パッチが上流の `i2c_dw.c` に触れる所は 11 → 14）。
    コアも同じ。`i2c_dw_sync.expected` は `--update`（回避策の行が消え、新しいループの添え字と読み方の違いが入った）。
  - **明示的なバスの復旧の要求は `-ENOTSUP`**: Zephyr は上流が `.recover_bus` を登録していて、コールバックが無いと 0 を返していたので、パッチ 0003 で `-ENOTSUP` に。NuttX は `I2C_RESET()` が `-ENOTSUP`。
    FreeRTOS は互換層に `i2c_recover_bus()` が無く、入口が無い（足していない）。転送の前の stuck の判定は従来どおり（状態を消して転送を試す）。
  - **NuttX: DMA のチャネルを待たずに取る**: 新しいローカルのパッチ `nuttx/patches/nuttx/0011-rp2040-dmachannel-trywait.patch`（`rp2040_dmachannel_trywait()`。`rp2040_dmachannel()` はそのまま）。
    つなぎは 2 回呼び、2 本取れなければ取った分を返して `-EBUSY`。I2C0 は DMA なしで ready になり、読み出しは割り込み。ホストテストに、空きなし・1 本だけ（返す）・2 本、と、DMA なしで I2C0 が
    動く変種を足した。`nuttx/README`（日英）のパッチの表に 1 行足した（メイン）。
- 使い方への影響: Zephyr の `i2c scan` は全アドレス `--`・「0 devices found」、`i2c recover` は「Bus recovery failed (err -134)」。NuttX の `i2c dev -z` は全部 `--`、`i2c reset` は失敗。
  NuttX の通常の `i2c dev`（1 バイトの読み出し）と Zephyr の `i2c read_byte` は変わらない。
- イメージ: Zephyr 372248 → 372280 B、NuttX 283756 → 283748 B、FreeRTOS 236952 → 236968 B。**基準を更新: Zephyr・NuttX とも `snapshot-1121g`**。ホストテスト 46・58・56、`format.sh --check` も通る。
- 実機（書き込み直後の各 1 回 約 70 秒）: 3 版とも欠番 0、時刻の逆行 0、`dma_errors`・`dma_fallbacks` 0、`cpu_active` Zephyr 46.9%、NuttX 48.3%、FreeRTOS 40.9%。
  Zephyr: `i2c scan` 全部 `--`（止まらない）、`i2c recover` -134、`i2c read_byte ... 6a 0f` は 0x6c。NuttX: `i2c dev 03 77` に 0x60 と 0x6a、`i2c dev -z` 全部 `--`、`i2c reset` 失敗、
  `dmesg` に「No DMA for the reads」なし。FreeRTOS: `-DNANO_SELFTEST=ON` の `selftest` が 22・6・3 項目とも失敗 0（I2C の refused に長さ 0 の書き込みを含む）。
- 確かめていない: NuttX で DMA のチャネルが足りない状態の実機（ホストテストだけ）、`-DI2C_DW_I2C1=ON` の実機、切断復帰・Telnet（このイメージでは）。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## I2C: SCL の見積もりの訂正（2026-11-21、ユーザーの指摘）

- 1 つ前の節の「Low 504 ns・周期 1000 ns 以上にすると実際は約 0.88 MHz になり足りない」は、数え方の上の周期を 1000 ns にしたうえで遅れ（約 142 ns）を足した、余裕を見すぎた見積もりだった。
  規格が求めるのは Low 0.50 µs 以上、High 0.26 µs 以上、周波数 1 MHz 以下。
- 数え方に直すと（125 MHz、8 ns。Low = (LCNT + 1)、High = (HCNT + 8) クロック）: Low 504 ns（LCNT 62）・High 264 ns（HCNT 25）で数え方の上の周期 768 ns、遅れを足して約 910 ns（約 1.10 MHz、
  周波数だけ超える）。High を 360 ns（HCNT 37）まで延ばすと約 1006 ns（約 0.99 MHz）で、3 つとも満たす。この遅れ（約 142 ns）は過去の記録からの見積もりで、波形は測っていない。
- 規格内の上限（約 1 MHz）でも、FIFO のデータ（毎秒 約 89.3 万クロック）がバスの約 89〜90% を占める。6667 Hz を保てるかは試していない。設計書（日英）の記述をこの内容に直した。

## I2C: SCL を規格内にする試験（2026-11-21、ユーザーの依頼「試してみてください」）

- 設定: コアの `I2C_DW_LCNT_OFFSET` 31・`I2C_DW_HCNT_OFFSET` 6（LCNT 62・HCNT 37: Low 504 ns、High 360 ns）。FreeRTOS 版だけ、一時的な変更（commit していない）。
- 結果（約 70 秒の送信 3 回、3 回とも同じ）: **FIFO のオーバーラン 136〜137 回、復帰 137 回**、バッチ 2313〜2326（通常 2447）、MQTT の欠番 0、I2C のエラー 0、DMA の失敗 0、
  `max_transfer_us` 32081〜32086（FIFO いっぱいの 3584 バイトなら約 1.005 MHz）、`cpu_active` 31.3〜31.5%（I2C の待ちが増えた分下がる）。
- 結論: 規格内（約 1 MHz）では 6667 Hz の FIFO を読み切れない。いまの設定（約 1.4 MHz）を保つ。設計書（日英）に結果を書いた。
- 元に戻した: 定数を 0 に戻し、既定のビルド（236968 B、変更前と同じ大きさ）を実機に書いて送信 約 70 秒 1 回: オーバーラン 0、欠番 0、40.9%。
- 試していない: Low だけ規格内にする設定（周波数は約 1.1 MHz で規格外）、6667 Hz より低い出力レート。

## I2C: codex の再レビューの 2 点（スキャンの未対応の表示、待たない DMA の取り方）（2026-11-21）

`9f9cd4e` の再レビュー。前回の 3 点は対応済みとの判定で、残りの 2 点を直した（subagent が実装、メインがホストテストと実機確認）。

1. **スキャンの未対応が「デバイス 0 台」という成功になる**（中）: 長さ 0 の書き込みで探るスキャンは、ドライバーが `-ENOTSUP` で断るようになったので全アドレスが `--` になり、成功で終わっていた。
   - Zephyr: 新しいローカルのパッチ `zephyr/manifest/zephyr/patches/zephyr/0004-i2c-shell-scan-not-supported.patch`（`drivers/i2c/i2c_shell.c`。DesignWare のドライバーとは別のファイル・別の主題なので
     0003 に入れず独立にした）。`-ENOTSUP` が返ったら `I2C: i2c@40044000 does not support this scan (zero-length writes)` と表示して `-ENOTSUP` を返す。NACK などほかのエラーは従来どおり `--`。
   - NuttX: 新しいローカルのパッチ `nuttx/patches/apps/0007-i2ctool-zero-byte-scan-not-supported.patch`（`system/i2c/i2c_dev.c`: `-z` のとき `errno` が `ENOTSUP` なら
     `i2ctool: dev: bus 0 does not support zero-byte writes; no -z scan` と表示して失敗。`system/i2c/i2c_main.c`: サブコマンドが負を返したら終了状態を失敗にする。もとは常に成功だった。
     `get`・`set`・`reset` などの失敗も終了状態に出るようになる）。通常の `i2c dev`（1 バイトの読み出し）は変わらない。
   - FreeRTOS 版には I2C のスキャンのコマンドが無い。
2. **`rp2040_dmachannel_trywait()` が、空きがあるときは管理用のロックを待つ**（低）: パッチ 0011 を作り直し、ロックも `nxmutex_trylock()` で試す。取れなければセマフォの数を返して NULL
   （チャネルを失わない）。契約（空きが無いか、割り当ての処理がほかのタスクで使用中なら、すぐ NULL。呼び出し側は試し直してよい）を関数のコメントとパッチの説明に書いた。
   つなぎは起動時に 2 回試し、だめなら DMA なしで続ける（試し直さない）。ホストテストに、2 回目が NULL のときに 1 本目を返すことの確認を足した。

- `nuttx/README`（日英）のパッチの表に apps 0007 の行を足し、0011 の説明を直した（メイン）。
- イメージ: Zephyr 372280 → 372376 B、NuttX 283748 → 283868 B、FreeRTOS は変わらない（236968 B）。**基準を更新: Zephyr・NuttX とも `snapshot-1121h`**。ホストテスト 46・58・56。
- 実機（書き込み直後の各 1 回 約 70 秒）: Zephyr 欠番 0、46.8%、`i2c scan` は上の文を表示して `retval` -134、`i2c read_byte ... 6a 0f` は 0x6c。NuttX 欠番 0、48.2%、`i2c dev 03 77` に 0x60 と 0x6a、
  `i2c dev -z 03 77` は上の文を表示して `$?` が 1。2 版とも `dma_errors`・`dma_fallbacks` 0。
- 確かめていない: `rp2040_dmachannel_trywait()` の中（ホストテストは関数ごと偽物に置き換えている。実機ではチャネルが空いている経路だけ通る）。
- 実機: NuttX 版（この変更のビルド）。

## NuttX: `i2c` コマンドの表示と終了状態の食い違い 3 つ（2026-11-21、codex の再レビュー）

- `cda2b55` の再レビュー。前回の 2 点は解消、`main()` で終了状態をまとめて扱う変更も維持してよいとの判定。ただしサブコマンドが失敗を返していない経路が 3 つあった（上流の i2ctool の不具合。
  `main()` の変更で終了状態に意味が出たので表に出た）。パッチ `nuttx/patches/apps/0007-…` を作り直して直した（メイン）。
  - `i2c reset`: リセットの失敗を表示したあと `close()` の結果で上書きしていた → どちらかが失敗したら ERROR。
  - `i2c dev`: 引数の不足・範囲の誤り・バスを開けないときに OK を返していた → 結果を ERROR で始め、走査を終えた経路でだけ OK（応答が 0 台の正常な走査は OK のまま）。
  - `i2c verf`: 書いた値と読んだ値が違うと `<<< FAILURE` を表示しながら OK を返していた → 一度でも不一致があれば ERROR。
- codex の報告: `rp2040_dmachannel_trywait()` の、ロックが取れない分岐をホストで確かめた（パッチ適用後の関数の本体を、同期は POSIX、レジスターは記録用の代替で動かした。残り 1 本でほかのスレッドが
  ロックを持つあいだ NULL で戻り、セマフォの数は 1 に戻り、レジスターへの書き込みは 0 回。解放後の再試行で取れた）。codex の手元の試験で、リポジトリには入っていない。
- イメージ: NuttX 283868 → 283908 B（**基準を `snapshot-1121i` に更新**）。Zephyr・FreeRTOS は変わらない。NuttX のホストテスト 58。
- 実機（NuttX 版）: `i2c reset` → 失敗の表示、`$?` 1。`i2c dev 03 77` → 0x60 と 0x6a、0。`i2c dev -z 03 77` → 1。`i2c dev 80 90`（範囲の誤り）→ 1。`i2c verf -a 6a -r 0f 0`（読み出し専用の
  WHO_AM_I に書く）→ `<<< FAILURE`、1。`i2c get -a 6a -r 0f` → 0x6c、0。送信 約 70 秒 1 回: 欠番 0、48.1%。
- 実機: NuttX 版（この変更のビルド）。

## NuttX: DMA の割り込みの印の消し方、DMA を取る順序（2026-11-21、codex の再レビュー）

`bb890af` までのレビュー。共通コアの作り直しは不要、NuttX の側の 2 点。どちらも NuttX の DMA のドライバーのソースで確かめ、指摘のとおりだった。

1. **DMA のチャネルを確保すると、別のチャネルの完了の割り込みを消すことがある**（高）: `DMA_INTS0` は 1 を書いたビットが消えるレジスター。パッチ 0011 の `rp2040_dmachannel_trywait()` は、
   上流の `rp2040_dmachannel()` を写して `setbits_reg32()`（SET エイリアス: レジスターの値にビットを足して書く）で書いていたので、保留中のほかのチャネルのビットも 1 として書かれて消える。
   上流の `rp2040_dmachannel()`（SET）と `rp2040_dmafree()`（`clrbits_reg32()`、CLR エイリアス）も同じ。
   - パッチ 0011 を直した: `putreg32(bit, RP2040_DMA_INTS0)`（自分のビットだけ）。
   - 新しいパッチ `nuttx/patches/nuttx/0012-rp2040-dmac-ints0-own-bit.patch`: 上流の `rp2040_dmachannel()` と `rp2040_dmafree()` も同じに直す（一時的な index で、0011 までを当てた状態との差分を取った）。
     `DMA_INTE0`（普通のレジスター）へのエイリアスはそのまま。つなぎ（`i2c_dw_dma_os_rp2040.c`）の `INTS0` への書き込みは、もとから `putreg32()`。
2. **I2C0 の初期化に失敗しても、DMA のチャネル 2 本を持ったままになる**（中）: NuttX のつなぎはコントローラーの検出と割り込みの登録より先に DMA を取っていた。
   → `i2c_dw_turn_on()` が成功したあと、`ready` を立てる前に取る。「DMA が取れなければ割り込みで読む」は変わらない。FreeRTOS のつなぎは起動時にチャネルを取らない（読み出しごと）ので、順序は変えていない。
- ホストテスト（NuttX の `i2c_dw_instance`）: 起動の順序の期待値を直し（`reset … attach enable dma-init`）、割り込みの登録に失敗した起動が DMA を求めないことを確かめる。
- `nuttx/README`（日英）のパッチの表に 0012 の行を足した（メイン）。説明の文書（NuttX のドライバー説明、日英）の起動の順序も直した。
- イメージ: NuttX 283908 B（大きさは同じ、中身は違う。**基準を `snapshot-1121j` に更新**）。Zephyr・FreeRTOS は変わらない（`common/` の変更はコメントだけ）。NuttX のホストテスト 58。
- 実機（NuttX 版）: 送信 約 70 秒 × 2 回（1 回は書き込み直後）: 欠番 0、47.9〜48.1%、`dma_transfers` 15041、`dma_errors`・`dma_fallbacks` 0、`dmesg` に「No DMA for the reads」なし。`i2c dev 03 77` に 0x60 と 0x6a。
- 確かめていない: ほかのドライバーが DMA を使っている状態（このビルドには無い。印が消える現象そのものは実機で再現していない）、I2C0 の起動に失敗した状態の実機。
- 実機: NuttX 版（この変更のビルド）。

## NuttX: DMA の割り当ての実物の関数のホストテスト（2026-11-21、codex の再レビュー）

- `4bf8d9a` の再レビュー: 前回の 2 点は解消。残りはテストの側のずれ 1 点: `nuttx/tests/i2c_dw_dma_os.c` の偽の割り当て（`rp2040_dmac.c` の代わり）が、確保時の `INTS0` を修正前の SET エイリアスで書き、
  解放時の印の消去も模していなかった。既存のテストでは、別のチャネルの割り込みを消す不具合の再発を検出できない。
- 対応:
  - 新しいホストテスト `nuttx/tests/rp2040_dmac.c`（+ `tests/rp2040_dmac_fakes/`）: 取得したツリーの `arch/arm/src/rp2040/rp2040_dmac.c`（パッチ 0011・0012 を当てたもの）を**そのまま取り込み**、
    DMA のレジスターの模型（`INTS0` は 1 を書いたビットが消える。SET・CLR のエイリアスは、値にビットを足して／引いて書く）とカーネルのロック・数の偽物の上で動かす。確かめること:
    `rp2040_dmachannel()`・`rp2040_dmachannel_trywait()`・`rp2040_dmafree()` が自分のチャネルの印だけを消し、別のチャネル（4）の保留中の印が残ること。待たない取り方が、空きなしで NULL、
    ロックがほかのタスクにあるとき NULL で数を戻しレジスターに書かないこと、そのあと取れること。ツリーが無ければ skip（`nina_netdev` と同じ扱い）。
  - 変異で確かめた（手元の写しで、リポジトリは変えていない）: 確保を SET エイリアスに戻す、解放を CLR エイリアスに戻す、ロックが取れないときに数を戻さない、の 3 通りとも、このテストが失敗する。
  - 既存の偽の割り当ても、自分のチャネルの印だけを消すように直した（確保と解放）。
- `nuttx/README`（日英）のホストテストの本数を直した（51・35 と古いままだった → 59・39）。
- NuttX のホストテスト 59（1 つ増えた）。イメージは変わらない（テストと文書だけ）。実機には触れていない。

## NuttX: ドライバーのファイルを 1 つのライブラリーでビルドする（2026-11-21、ユーザーの指摘）

- ユーザーの指摘: NuttX のビルドの結果、ドライバーのオブジェクトが `libboard.a`（`i2c_dw.o`・`i2c_dw_dma_os_rp2040.o`・`nina_bus_rp2040.o`・`nina_passthrough_bus_rp2040.o`）と
  `libexternal.a`（NINA のコアとつなぎ）に分かれている。`libdrivers.a` の `i2c_driver.o` などは NuttX 本体の I2C の共通部分で、別物。
- 分かれていた理由: この 4 つは RP2040 の内部のヘッダー（`arm_internal.h`、`rp2040_gpio.h` など）を使い、そのパスがボードのビルドにだけ通っていたので、もとはボードの `src/` にあった。
  ファイルを `common/drivers/…/port/nuttx/` へ移したとき、イメージのバイト一致で確かめるためにリンクの順序を保ち、ビルドする場所を変えなかった。
- 変更: 4 つとも `libexternal.a` でビルドする。NINA の 2 つは `nuttx/drivers/nina/Make.defs`、I2C は新しい `nuttx/drivers/i2c_dw/Make.defs`。チップの内部のヘッダーへのパス
  （`arch/arm/src/chip`・`common`・`armv6-m`、`sched`）は `nuttx/drivers/Makefile` が足す（`boards/Board.mk` がボードのソースに足すのと同じ）。ボードの `src/Make.defs` からは 4 つの記述を削り、
  起動のコードが読むヘッダーへの `-I` だけ残した（NINA は `include` と `port/nuttx`、I2C は `port/nuttx`）。I2C のオプション（`CONFIG_I2C_DW_*`）はボードの Kconfig のまま。
- イメージ: NuttX 283908 → 283920 B（**基準を `snapshot-1121k` に更新**）。NuttX のホストテスト 59。
- 実機（NuttX 版）: 書き込み直後の送信 約 70 秒: 欠番 0、48.4%、`dma_errors`・`dma_fallbacks` 0。`nina version`・`i2c dev 03 77` 正常。中継（1 回）: 2 MB の読み出しが 115200・921600 bps で一致、
  書き込み後の退避とも一致、300 bps で抜ける。送信中の切断 → 2 秒で再接続、MQTT もつなぎ直し、Telnet は送信中と復帰後に応答（1 回）。
- **FreeRTOS は変えなかった**: FreeRTOS は全部のソースが 1 つの実行ファイルに直接入るので、ライブラリーの分かれ方は無い。ただ cmake の記述で、NINA の `nina_wifi.c`・`port/zephyr/nina_net.c` が
  `wifi.cmake` にある。これを `nina.cmake` に移す変更を試したところ、リンクの順序が変わるだけで `cpu_active` が 40.9% → 42.7%（3 回とも。`imu_fifo_worker` が 18.3 → 20.3）になった。
  戻して 40.9%（2 回）。配置の影響と判断して、変更を取りやめた。配置による変動が、これまでの目安（0.6〜0.7 ポイント）より大きいことがあると分かった（CLAUDE.md に追記）。
- Zephyr: NINA は `app` のライブラリーに 1 か所でまとまり、I2C は Zephyr 本体のドライバー（+ パッチ）なので、分かれ方の問題は無い。
- `freertos/drivers/i2c_dw/i2c_dw_selftest.[ch]`（ユーザー: 使わないなら削除してよい）: `-DNANO_SELFTEST=ON` のビルドの `selftest` の I2C の 6 項目で使っている（今日も確認に使った）ので残した。
- 実機: FreeRTOS 版（変更なしの既定のビルド。40.9%）。

## FreeRTOS: ソースの並び順で `cpu_active` が変わる理由（2026-11-22、ユーザーの質問）

- 質問: cmake で `nina_wifi.c`・`nina_net.c` を `nina.cmake` へ移しただけで 40.9% → 42.7% になったのは、配置がどう変わったからか。
- 配置の差（2 つの ELF のシンボルを比べた）: テキストの関数 1616 個のうち、1215 個は同じアドレス、368 個が +2912 バイト、33 個（移した 2 ファイルの関数）が −51396 バイト。
  +2912 動いたのは、リンクの順序で 2 ファイルの新しい位置より後ろ、もとの位置より前にあったもの: I2C のドライバー、LSM6DSO の FIFO、IMU、MQTT、NINA のソケットなど。
  FreeRTOS のカーネル、互換層のカーネルの部分、`nina_proto.c` などは動いていない。フラッシュにある関数は 1487 個、RAM にあるものは 129 個。
- 仕組み: RP2040 はフラッシュのコードを XIP のキャッシュ（16 kB、2 ウェイ）越しに実行する。キャッシュの中の置き場所はアドレスで決まるので、送信の経路で一緒に使われるコードのうち一部だけが
  2912 バイトずれると、同じ置き場所を取り合う組み合わせが変わる。
- 実測（送信中に XIP のカウンターをクリアして 15 秒後に読む。`devmem 0x1400000c`（当たり）・`0x14000010`（アクセス））:

  | ビルド | `cpu_active` | アクセス/秒 | 外れ/秒 | 外れの割合 |
  |---|---|---|---|---|
  | もとの並び（3 回） | 40.8〜41.1% | 約 809〜814 万 | 27.6〜28.5 万 | 3.41〜3.51% |
  | 2 ファイルを移した並び（2 回） | 42.7% | 約 809 万 | 32.1〜32.7 万 | 3.96〜4.04% |

  外れが毎秒 約 4 万回増えた。`cpu_active` の差 1.8 ポイント（毎秒 18 ms）を割ると 1 回あたり約 0.45 µs で、フラッシュから 8 バイトを読み直す時間（62.5 MHz の QSPI で約 28 クロック）と合う。
- わかったこと: 外れは良いビルドでも毎秒 約 28 万回あり、同じ換算で CPU の約 13% がフラッシュの読み直しに使われている。どの関数どうしが取り合っているかまでは調べていない
  （実行の履歴が要る）。
- 実機: もとのビルドに戻した（ELF が変更前と一致。40.8%）。

## FreeRTOS: NINA のソースを全部 `nina.cmake` に（2026-11-22、ユーザーの判断）

- ユーザー: 「配置で cpu_active が変わるのは仕方ない。それよりも構成のずれのほうが気になります」。取りやめていた変更を入れた。
- `nina_wifi.c` と `port/zephyr/nina_net.c` を `freertos/drivers/wifi/wifi.cmake` から `freertos/drivers/nina/nina.cmake`（`freertos_nina()`）へ。`wifi.cmake` に残るのは、互換層のネットワークの部分、
  資格情報、プロファイル、`wifi`・`ifconfig` コマンド。
- イメージ: 236968 B（大きさは同じ、関数の並びが変わる）。ホストテスト 56、`format.sh --check` も通る。
- 実機（この並びのビルド。ELF が、前の節で測ったものと一致）: 送信 約 70 秒 × 6 回（2 回は書き込み直後）: 欠番 0、時刻の逆行 0、**`cpu_active` 42.7%**（もとの並びは 40.9%。理由は前の節）。
  送信中の切断 → 2 秒で再接続、MQTT もつなぎ直し、Telnet は送信中と復帰後に応答（1 回）。`nina version` 正常、`dma_errors`・`dma_fallbacks` 0。
- **FreeRTOS 版の `cpu_active` の目安は、これから約 42.7%**（これまでの 39.8〜41.0% と比べるときは、この変更の分を差し引く）。
- 実機: FreeRTOS 版（この変更のビルド）。

## NuttX: ドライバーのオプションをドライバーの Kconfig へ（2026-11-22、ユーザーの依頼）

- `CONFIG_I2C_DW_I2C0`・`_I2C1`・`_RP2040_DMA` を、ボードの Kconfig から新しい `nuttx/drivers/i2c_dw/Kconfig` へ（`nuttx/drivers/Kconfig` が `rsource`）。
- 同じ種類だった `CONFIG_NINA_PASSTHROUGH` も、ボードの Kconfig から `nuttx/drivers/nina/Kconfig`（`endif # NINA` のあと。NINA の本体とは独立に選べるまま）へ。
  ボードの Kconfig に残るのはボードの `NANO_*` の 6 つ。
- 確認: `.config` の中身は並びを除いて変更前と同じ（整列して比較）、NuttX のイメージは基準（`snapshot-1121k`）とバイト一致。実機には書いていない。

## sensor（LSM6DSOX）: Zephyr 版を上流のドライバー + ローカルのパッチに（2026-11-22、ユーザーの決定）

- これまで: アプリの CMake（`use_local_driver_sources()`）が、Zephyr のセンサーのライブラリーから `lsm6dso.c`・`sensor_shell.c` を抜いて `common/` のものに差し替え、ソース 8 本と include パスを
  Zephyr のライブラリーへ外から足していた。指針 4（上流 + パッチ、パッチはツリーの中で完結）に反していた。
- ユーザーの決定: 全部をパッチに入れる（案 A）。パッチを作るとき、足すファイルをまとめられないか、上流の `drivers/sensor/st/lsm6dso` を使えないかを考える。
- 変更（subagent が実装、メインがレビュー・ホストテスト・実機確認）:
  - 新しいパッチ `0005-sensor-lsm6dso-fifo-stream.patch`（2930 行、17 ファイル: 上流の `lsm6dso.c`・`lsm6dso.h`・`CMakeLists.txt` の変更と、足すファイル。`Kconfig` は上流のまま）と
    `0006-sensor-shell-fifo-attributes.patch`（193 行: `sensor_shell.c` の変更と `sensor_shell_os.h`）。一時的な index で作った。`setup.sh` が 7 つのパッチを当てる。
  - アプリの CMake はセンサーのライブラリーに何も足さない。`use_local_driver_sources()` は削除。残るのは、アプリ自身の include パスにツリーのドライバーのディレクトリーを足すこと
    （`lsm6dso_cal.h`・`lsm6dso_fifo.h` を読むため）と、パッチが当たっていないツリーで止めること。`zephyr/app/usb-console/drivers/lsm6dso/`・`sensor_shell/` は削除（上流と同じだった
    `lsm6dso_shub.c`・`lsm6dso_trigger.c` の写しも）。
- ファイルのまとめ（足すファイル 17 → 13）:
  - `lsm6dso_zero.[ch]` → `lsm6dso_cal.[ch]`（同じバイアスの形式の上の純粋な計算）。`lsm6dso_fifo_packet.h` → `lsm6dso_fifo.h`。
  - `lsm6dso_fifo_decoder.[ch]` → `lsm6dso_decoder.[ch]`、`lsm6dso_compat.c` → `lsm6dso_rtio.c`（`lsm6dso_submit()` もここ）、`lsm6dso_stream.c` → `lsm6dso_rtio_stream.c`
    （兄弟のドライバー `lsm6dsv16x` の名前）。
  - `lsm6dso_attributes.h`・`lsm6dso_fifo_stats.h` → 公開のヘッダー `include/zephyr/drivers/sensor/lsm6dso.h`（上流の `lsm6dsvxxx.h` と同じ形。`sensor_shell.c` が include パスを足さずに読める）。
  - 分けたままにしたもの: `lsm6dso_fifo.[ch]`（アプリと多くのホストテストがセンサーの API なしで使う）、`lsm6dso_fifo_hw.[ch]`・`lsm6dso_fifo_irq.[ch]`・`lsm6dso_decoder.h`（それぞれ単独でコンパイルするホストテストがある）。
- 上流のもの: `lsm6dso.c`・`lsm6dso.h`・`CMakeLists.txt`・`Kconfig` は上流のファイルにパッチ、`lsm6dso_shub.c`・`lsm6dso_trigger.c` は上流のまま。**上流のトリガー（`CONFIG_LSM6DSO_TRIGGER*`）は INT1 に使わなかった**:
  初期化で線を有効にして drdy を書き、ISR で GPIO の割り込みを止め、システムのワークキュー（協調）か専用のスレッドで I2C を読む作りで、こちらの「ストリーム中だけエッジを有効にする、開始時に
  すでに立っているレベルを見る、I2C は協調のワークキューで行わず `imu_fifo_worker`（優先度 10）に回す」と合わない。`fifo_lock` も取らない。上流の `default_rtio_sensor.c` のフォールバックは static なので
  写しのまま（ログのモジュール名 `lsm6dso_compat` も保つ）。
- `common/`: `common/drivers/lsm6dso/`（14 ファイル + `include/lsm6dso.h`）と `common/drivers/sensor_shell/sensor_shell.c` は、パッチを当てたツリーと同じファイル。NuttX・FreeRTOS は公開のヘッダーを
  互換層の `common/compat/zephyr/drivers/sensor/lsm6dso.h` 経由で読む。新しい検査 `zephyr_lsm6dso_patched`（`common/tests/run.sh`）: ツリーの HEAD の写しに 0005・0006 を当て、16 ファイルをバイト単位で
  比べ、ドライバーのディレクトリーに「共有でも版ごとでもない」ファイルがあれば失敗する。版ごとのもの: `CMakeLists.txt`・`Kconfig`、`lsm6dso_instance.h`、`sensor_shell_os.h`、上流のままの 2 ファイル。
- イメージ: 段階 1（ファイルを今のままパッチに）で Zephyr は基準とバイト一致。段階 2（まとめ）で 3 版とも大きさは同じ（372376・283920・236968 B）で中身が変わる（`lsm6dso_submit()` の分け方で 4 バイト、
  ほかはアドレスのずれ）。**基準を更新: Zephyr・NuttX とも `snapshot-1122a`**。ホストテスト 47・60・57（新しい検査が 1 つずつ）、`format.sh --check` も通る。
- 実機（書き込み直後の各 1 回 約 70 秒 + 下記）:
  - Zephyr: 欠番 0、46.7%、オーバーラン 0、I2C のエラー 0。`sensor get` 応答、`imu cal` は校正済みとジャイロのバイアスを表示、`sensor stream` on（6667 Hz では既知のとおり `Read failed` で止まる）・off、
    `dmesg` に `<err>`・`<wrn>` なし。スタックの使用: `sensor_shell_processing_tid` 1184/3072、`imu_stream_tid` 984/2048、`imu_fifo_worker` 904/2048、`sysworkq` 304/1024。
  - NuttX: 欠番 0、48.4%、`sensor get`・`imu cal` 正常、`dma_errors`・`dma_fallbacks` 0。
  - FreeRTOS: `-DNANO_SELFTEST=ON` の `selftest` が 22・6・3 項目とも失敗 0（LSM6DSO のストリーム 501 ms: 55 バッチ、7149 語）。既定のビルドは 4 回測って 3 回が欠番 0・42.8〜42.9%、
    **1 回目に欠番 35**: 約 60 秒のところで 1 回の publish が 1.36 秒かかり（`publish_max_us` 1359284、到着の空き 1308 ms）、キューが 12 まで積まれてあふれた。MQTT の接続はそのまま（connects 1、failures 0）、
    FIFO のオーバーラン 0、I2C のエラー 0。センサーの経路ではなく NINA への送信の側の詰まり。
  - **この詰まりは、NuttX 版でソケットを標準の経路に変えた直後に出た欠番（387 ms の空き、約 1 秒の空き）と同じ種類**。FreeRTOS 版はその変更を受けていない（Zephyr のソケットのつなぎのまま）ので、
    あの欠番も NuttX の変更が原因ではなく、電波か NINA の側の一時的な詰まりだった可能性が高くなった（原因そのものは未確定のまま）。
- 確かめていない: `sensor attr_set`・`attr_get` の FIFO の属性、`imu cal` の校正のやり直し、切断復帰・Telnet（このイメージでは）、長時間。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## sensor: ドライバーのファイルをさらにまとめる（2026-11-22、ユーザーの指摘）

- ユーザー: 分けたままにした理由（単独でコンパイルするホストテスト、アプリがセンサーの API なしで使う）に対して、「アプリの都合で制約しなくてもよい」。ドライバーの筋だけでまとめ直した
  （同じ subagent が実装、メインがホストテストと実機確認）。
- 足すファイルは、兄弟のドライバー `lsm6dsv16x` と同じ組と、あと 3 つ:
  - `lsm6dso_decoder.[ch]`: FIFO のバッファを読み解くもの全部（語の解析 = もとの `lsm6dso_fifo.c`、ストリームのバッファの形、2 つのデコーダー）。
  - `lsm6dso_rtio.c`: RTIO の submit と、センサーの API の 1 回読みのフォールバックの写し。`lsm6dso_rtio.h` は作らない（宣言は `lsm6dso.h` にある）。
  - `lsm6dso_rtio_stream.c`: ストリーム。INT1（しきい値の経路、GPIO の割り込みとワーク = もとの `lsm6dso_fifo_irq.c`）は static で中に。
  - `lsm6dso.c`: FIFO のレジスターの操作（設定・停止・一時停止・状態・まとめ読み = もとの `lsm6dso_fifo_hw.c`）は static でここに（呼ぶのは `fifo_lock` の下のこのファイルの関数だけ）。
  - `lsm6dso_cal.[ch]`: 残した。校正はこのドライバーが足した主題で、換算は 1 回読み（`lsm6dso.c`）とデコーダーの両方が使う。どちらかへ入れると、もう一方がそのファイルに依存する。
    中身はドライバーの内側の分だけ（換算とバイアスの追跡）。
  - `lsm6dso_instance.h`、公開のヘッダー `include/zephyr/drivers/sensor/lsm6dso.h`。
  - 削除: `lsm6dso_fifo.[ch]`・`lsm6dso_fifo_hw.[ch]`・`lsm6dso_fifo_irq.[ch]`、共通のテスト `common/tests/fifo_hw.c`。
- 公開のヘッダーがアプリ向けに宣言するもの: FIFO の語の形式と解析（`lsm6dso_fifo_next()` など）、校正の型と呼び出し（`struct lsm6dso_cal`、`set_lookup`・`reload`・`status`・`zero`）、
  これまでの FIFO の属性・統計・`lsm6dso_timestamp_read()`・`lsm6dso_fifo_request_stop()`。アプリ（`imu_raw_packet.h`・`imu_cal.h`・`imu_stream.c`）はこのヘッダーだけを読む。
  Zephyr のアプリの CMake からツリーのドライバーのディレクトリーへの include パスを削除（残るのは、パッチが当たっていないツリーで止める確認だけ）。NuttX の `apps/imu`・`apps/init`、
  FreeRTOS の `lsm6dso.cmake`・`imu.cmake` からも、`common/drivers/lsm6dso` への `-I` を削除。
- テスト: 偽のヘッダーに `struct device`・`SENSOR_ATTR_PRIV_START`・`struct rtio` などを足し、`fifo_words`・`fifo_decoder`・IMU のテストは `lsm6dso_decoder.c` をコンパイルする。`stream_stop` は INT1 の実物の
  コードも動かす。`fifo_hw` の確認は NuttX の `tests/lsm6dso_driver.c` へ（`lsm6dso.c` と `lsm6dso_rtio_stream.c` を取り込み、ST の実物のコードと偽のレジスターの上で確かめる）。
  **失ったもの**: ST の関数の呼び出し回数の厳密な確認（「n 回目で失敗し、そのあと何も呼ばない」）。終わりの状態だけ確かめる。レジスターの確認は NuttX のホストテストだけで走る（以前は 3 版）。
- パッチ 0005 を作り直した（2860 行）。0006 は変わらない。検査 `zephyr_lsm6dso_patched` は 10 ファイルを照合。
- イメージ: Zephyr 372376 → 372184 B、NuttX 283920 → 283792 B、FreeRTOS 236968 → 236776 B（static になった関数が呼び出し元にインライン化された分。Zephyr のオブジェクトで確認）。
  **基準を更新: Zephyr・NuttX とも `snapshot-1122b`**。ホストテスト 46・59・56（共通の `fifo_hw` が 1 つ減った）、`format.sh --check` も通る。
- 実機（書き込み直後の各 1 回 約 70 秒 + 下記）:
  - Zephyr: 欠番 0、47.1%、オーバーラン 0、I2C のエラー 0。`sensor get`、`imu cal`、`sensor stream` on・off を 2 回、`dmesg` に `<err>`・`<wrn>` なし。
    スタックの使用: `imu_fifo_worker` 944/2048（前は 904）、`imu_stream_tid` 984、`sysworkq` 304、`sensor_shell_processing_tid` 1184。
  - NuttX: 欠番 0、48.1%、`sensor get`・`imu cal` 正常、`dma_errors`・`dma_fallbacks` 0。
  - FreeRTOS: `selftest` 22・6・3 項目とも失敗 0（ストリーム 501 ms: 55 バッチ、7136 語、INT1 128 回）。既定のビルド 2 回: 欠番 0、43.1%。`sensor get`・`imu cal` 正常。
    スタックの空き: `imu_fifo_worker` 1352（前は 1368）。
- 確かめていない: `sensor attr_set`・`attr_get` の FIFO の属性、校正のやり直し、切断復帰・Telnet（このイメージでは）、長時間。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## sensor: コアの切り出し（段階 1）と、Zephyr 版が `common/drivers` を直接使う形（2026-11-22）

- ユーザーの決定: `common/drivers/lsm6dso/` を `include/`・`internal/`・`port/` の形にする。案はすべて A（コアとつなぎを同じ分け方で 3 版が使う、NuttX・FreeRTOS は Zephyr へのつなぎを互換層の上で
  ビルドする、FIFO を読み続ける処理はコアへ）。作業の途中でユーザーの提案: 「コアとつなぎをパッチでツリーに並べるなら、Zephyr のセンサーのドライバーを使わず `common/drivers` を使うほうがよい」
  → そのとおりにした（分けたあとは上流のファイルとの差が意味を持たず、同じファイルを 2 か所に持つだけになる）。NuttX・FreeRTOS が RTIO と sensor を使わない形にする考えは無い（ユーザー）。
- 段階: 1 = コアの切り出し（この節）、2 = FIFO を読み続ける処理と状態をコアへ、3 = OS の境界を各 OS で直接実装。ほかに、互換層の `zephyr_sensor_clock.c`・`zephyr_rtio_workq_os.h`（各版）を
  `common/compat` にまとめる（承認済み、次に行う）。
- できた形（subagent が実装、メインがホストテストと実機確認）:
  - コア（直下、OS の定義なし）: `lsm6dso_chip.c`（初期化の手順、レートとレンジ、1 回読みと換算、校正の状態、FIFO のレジスターの操作と開始・一時停止・停止・読み出し・状態・統計、時刻。
    コアのオブジェクトは `struct lsm6dso_chip`）、`lsm6dso_cal.c`、`lsm6dso_fifo.c`（語の解析。デコーダーから分けた）。`internal/lsm6dso_chip.h`。
  - `include/`: `lsm6dso.h`（デバイスを取る呼び出し、属性）、`lsm6dso_cal.h`、`lsm6dso_fifo.h`。
  - 境界 `port/lsm6dso_os.h`: `i2c_dw` と同じ static inline の形（ロックの型がオブジェクトの中にあり、FIFO の読み出しが直接の呼び出しにコンパイルされるように）。mutex、spinlock、待ち 3 種、
    サイクルの時計、長いバスの読み出し（ST の ctx の `read_reg` は長さが 8 ビットで、3584 バイトの読み出しに足りない）、`LOG_*`。実装はいまは 3 版とも `port/zephyr/lsm6dso_os_impl.h`。
  - つなぎ `port/zephyr/`: `lsm6dso.[ch]`（デバイス、センサーの API）、`lsm6dso_decoder.[ch]`、`lsm6dso_rtio.c`、`lsm6dso_rtio_stream.c`（ストリームの全部。段階 2 でコアへ）、`lsm6dso_instance.h`。
    `port/nuttx/`・`port/freertos/` は `lsm6dso_instance.h`（各版の `drivers/lsm6dso/` から移動）。
- Zephyr 版の取り込み: つなぎは Zephyr のモジュール（`port/zephyr/zephyr/module.yml`、アプリの CMake が `ZEPHYR_EXTRA_MODULES` に足す）。NINA のようにアプリの CMake からライブラリーを作る形は、
  生成されるヘッダーへの依存が付かず、リンクもされなかった。スイッチは `CONFIG_LSM6DSO_LOCAL`（`prj.conf` で上流の `CONFIG_LSM6DSO=n`）。公開のヘッダーは `zephyr_include_directories` で出す。
  デバイスツリーの binding は上流の `st,lsm6dso`。**パッチ 0005 は削除**、0006（センサーのシェル）は残る。検査は `zephyr_sensor_shell_patched` に。
- 削ったもの（どの版でも無効だった）: センサーハブ・トリガー・タップの経路と項目、温度の経路（`CONFIG_LSM6DSO_ENABLE_TEMP`。`DIE_TEMP` は従来どおり `-ENOTSUP`）、`drdy_pulsed`、互換層の `sys_get_le32()`。
- 新しい検査: `lsm6dso_core_independent`（コアをホストの C ライブラリーと ST のヘッダーだけでコンパイル）、`lsm6dso_public_headers`。ホストテスト 48・61・58。
- イメージ: Zephyr 372424 B（+240）、NuttX 284032 B（+240）、FreeRTOS 237168 B（+392）。デバイスを取る呼び出しの包みと、インライン化されなくなった 1 回読みの関数の分。
  **基準を更新: `snapshot-1122c`**。
- 実機（書き込み直後の各 1 回 約 70 秒 + 下記）:
  - Zephyr: 欠番 0、45.7%、オーバーラン 0、I2C のエラー 0。`sensor get`、`imu cal`、`sensor stream` on・off 2 回、`sensor attr_get`・`attr_set`（`all fifo_watermark` 128 → 64 → 128、
    `fifo_timestamp_decimation` 8）、`dmesg` に `<err>`・`<wrn>` なし。スタック: `imu_fifo_worker` 936/2048、`imu_stream_tid` 992、`sysworkq` 304、シェル 1184。
  - NuttX: 欠番 0、48.6%、`sensor get`・`imu cal`・`attr_get` 正常、`dma_errors`・`dma_fallbacks` 0。
  - FreeRTOS: `selftest` 22・6・3 項目とも失敗 0。既定のビルド 2 回: 欠番 0、42.8%・42.9%。`sensor get`・`imu cal`・`attr_get` 正常。スタックの空きは前と同じ（`imu_fifo_worker` 1352）。
- 確かめていない: 校正のやり直し、切断復帰・Telnet（このイメージでは）、長時間、SPI のインスタンス（バスの長い読み出しは `-ENOTSUP`）。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## 互換層の RTIO と sensor を全部 `common/compat` に（2026-11-22）

- ユーザーの質問から: 各版の `drivers/compat` にある RTIO と sensor のファイルを `common/compat` に移せるか → 移した（subagent が実装、メインがホストテストと実機確認）。
- `zephyr_sensor_clock.c`: 2 版の同じ中身を `common/compat/zephyr_sensor_clock.c` の 1 つに。時計は互換層の `k_cycle_get_64()`（2 版とも `nano_mono_us()` の static inline。同じ 1 MHz の TIMER）。
- `zephyr_rtio_workq_os.h`（2 版と、ホストテストの偽物）: 削除。スレッド `RTIO WQ 0` は `common/compat/zephyr_rtio_workq.c` の中で、`K_THREAD_DEFINE()` のスレッドと同じ
  `z_thread_static_start()` で起動する。`K_THREAD_DEFINE()` そのものは使わない（スレッドの名前が C の識別子になる、起動がアプリの初期化に移る）。起動の位置は `zephyr_compat_initialize()` のまま。
  `rtio_workq_initialize()` は `int` を返す。
- イメージ: NuttX 284040 B（+8）、FreeRTOS 237088 B（−80）。Zephyr は変わらない（互換層をビルドしない）。**基準: `snapshot-1122d`**（Zephyr は 1122c と同じ中身）。ホストテスト 48・61・58。
- 実機（書き込み直後の各 1 回 約 70 秒）: NuttX 欠番 0・48.8%、時刻の逆行 0、`sensor get` 応答（このスレッドを通る）、`ps` に `RTIO_WQ_0`（優先度 118、スタック 640/976）。
  FreeRTOS 欠番 0・42.9%、時刻の逆行 0、`sensor get` 応答、`RTIO WQ 0`（優先度 20、空き 536）、`dmesg` に警告なし。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## 方針: いちばんの目的は、対応 OS を簡単に増やすこと（2026-11-22、ユーザー）

- センサーの段階 3 を変更（ユーザーの了解）: OS の境界の既定の実装は互換層の上に 1 つ置き、新しい OS はセンサーのために何も書かない。直接の実装は、計測で差が出た OS だけ。
  I2C と NINA の FreeRTOS の直接の実装はそのまま。
- コアの名前（ユーザーの了解、段階 2 で行う）: コアを `lsm6dso.c`（関数は `lsm6dso_*`）、Zephyr へのつなぎを `port/zephyr/lsm6dso_sensor.[ch]` に。`lsm6dso.h` の名前の重なりを無くす。
- ユーザー: 移植の一覧の文書と、実際に OS を足す試しは、まだ先。先に「互換層の OS の部分」と「ボードの起動、USB のコンソール、シェルの入口、フラッシュ、ビルドの規則」を
  何とかしないといけない。

## OS を足しやすくする整理: 調査と順序（2026-11-22、ユーザーの了解）

- 調査（読むだけ）で分かったこと: `freertos/drivers/compat` の約 5600 行のうち約 2500 行（ソケットの層、記述子の表、`net_if`、`k_heap`）は FreeRTOS も Pico SDK も呼ばない。FreeRTOS 版の
  `zephyr_kernel.c` は約 10 個の基本の操作の上の汎用の実装に近い。NuttX 版は NuttX のセマフォ・wdog への読み替えで、動きに違いがある（`k_sem_give` が起こすポーリングの待ち手、`k_sem_reset` の
  待ち手への戻り値、ポーリングの事象の順序など。一覧は `design/os-boundary(-ja).md`）。シェル・Telnet・ログも FreeRTOS 版のものはほぼ OS に依存しない。
- 順序（1 つずつ commit）: 1 境界の一覧の文書 → 2 OS に依存しないネットワークの層を `common/compat` へ（イメージ一致）→ 3 互換層の自己診断を共有に → 4 OS に依存しないシェルのファイルを共有に →
  5 2 版で同じ起動のつなぎを 1 つに → 6 ログの共通部分 → 7 RP2040 に固有で OS に依存しない部分（タイマーの読み出し、リセット、アイドルの計測）→ 8 FreeRTOS 版のカーネルの実装を
  「汎用の実装 + 基本の操作のヘッダー」に（イメージ一致を目標、だめなら計測して相談）→ 9 コンソール・Telnet・汎用のコマンド → 10 アプリの起動の手順 → 11 フラッシュの書き込みと異常停止の記録。
- しないこと: NuttX 版を汎用のカーネルの実装に載せ替える、nsh と telnetd を置き換える、ドライバーのつなぎの待ちを汎用の境界に通す、GPIO の割り込みや DMA のレジスターを共有のコードで触る。
- NuttX 版だけの `nano_` のファイルと関数は `rp2040_` に改名する（ユーザーの決定。NuttX 本体のボードのファイルの流儀）。共有の境界の `nano_*` は 7・11 で移すときに名前を決める。
  時期はセンサーの段階 2 と 2〜4 のあと。
- 1 を実施: `design/os-boundary.md`・`-ja.md`（今の境界の一覧。宣言された境界、暗黙の境界、ドライバーの境界、OS ごとに残るもの）。コードは変えていない。

## sensor: FIFO を読み続ける処理をコアへ（段階 2）と改名（2026-11-22）

- subagent が実装、メインがホストテストと実機確認。先に改名だけを行い、3 版のイメージが基準とバイト単位で一致することを確かめてから、処理を移した。
- 改名（ユーザーの了解）: コアは `lsm6dso.c`（もとの `lsm6dso_chip.c`。関数は `lsm6dso_*`、オブジェクトは `struct lsm6dso`）、`internal/lsm6dso_core.h`。Zephyr へのつなぎは
  `port/zephyr/lsm6dso_sensor.[ch]`（関数は `lsm6dso_sensor_*`）。公開の API は名前を保ち、同じ名前になるコアの関数は動詞を先に置く（`lsm6dso_reload_cal()`、`lsm6dso_get_fifo_stats()`、
  `lsm6dso_read_timestamp()` など）。`lsm6dso.h` という名前は、公開のヘッダーと、それを `<zephyr/drivers/sensor/lsm6dso.h>` として見せる中継の 2 つ（include パスの順序には依存しない）。
- 移したもの: `lsm6dso_stream.c`（コア）がストリームの状態と処理（開始・停止、INT1 と開始時のレベル、しきい値を下回るまでの読み出し、20 ms の見回り、停止の要求、エラーとオーバーラン、
  しきい値の設定）を持つ。境界は 2 つ、どちらも static inline:
  - `port/lsm6dso_os.h` に追加: 後回しの処理（`lsm6dso_os_work_*`。Zephyr では `imu_fifo_worker` のワークキュー、優先度 10、2048 バイト）、INT1 の線（`lsm6dso_os_int1_*`）、
    バスの長い読み出しができるかの確認、時刻（`lsm6dso_os_clock_ns()`）。
  - `port/lsm6dso_stream_req.h`: ストリームの使い手（2 つの型と 6 つの関数: 同じ要求か、取り出す、取り消されたか、バッファ、完了、エラー）。RTIO 用の実装が `port/zephyr/lsm6dso_stream_req_impl.h`。
  - つなぎの `lsm6dso_rtio_stream.c` は 74 行（デバイスごとの初期化、読み出しの設定の検査、デバイスを取る公開の関数）。
  - 互換層のある OS は何も書かない。互換層なしなら `lsm6dso_os_impl.h`（型と約 17 個の小さな関数）と INT1・スレッド。RTIO 以外の使い手なら `lsm6dso_stream_req_impl.h`。
- バッチごとの経路: 増えたのは `imu_fifo_worker` から `lsm6dso_stream_run()` への直接の呼び出し 1 つ（約 104 回/秒）。コピーとコンテキストスイッチは増えない。I2C は従来どおり `imu_fifo_worker`。
- 小さく変わった動き: INT1 を有効にした時点でレベルがすでに立っている場合、コアがストリームの処理を直接回す（以前はシステムのワークキューを 1 回経由）。
- そのままにした気になる所（subagent の報告）: エッジは ISR → システムのワークキュー → ワーカーの順（ISR から直接でも足りる）、`lsm6dso_stream_int1()` は常に 0 を返す、
  取り消しはバッチを止めずに停止するので FIFO に残った語は捨てられる、20 ms の見回りはエッジが来ていても回る。
- テスト: `common/tests/stream_stop.c` を `lsm6dso_stream.c` に置き換え（RTIO なしでコアのストリームを試す。もとの確認は全部残し、開始・エッジ・しきい値未満・オーバーラン・取り消し・2 つ目の要求を追加）。
  `lsm6dso_core_independent` は 11 ファイル。ホストテスト 48・61・58、`format.sh --check` も通る。
- イメージ: Zephyr 372328 B（−96）、NuttX 283992 B（−48）、FreeRTOS 237056 B（−32）。**基準: `snapshot-1122e`**。
- 実機:
  - Zephyr（書き込み直後 約 70 秒）: 欠番 0、46.0%、オーバーラン 0、I2C のエラー 0。`sensor get`、`sensor stream` on・off 2 回、`imu start`・`stop` 3 回、アプリが送信中の `sensor stream`・`sensor get` は
    `-16`（使用中）、`fifo_watermark` の設定、`dmesg` に `<err>` なし。スタック: `imu_fifo_worker` 928/2048、`imu_stream_tid` 992、`sysworkq` 304。
  - NuttX（同）: 欠番 0、47.5%、`sensor get`、`imu start`・`stop` 2 回、`dma_errors`・`dma_fallbacks` 0、`imu_fifo_worker` のスタック 832/2008。
  - FreeRTOS: `selftest` 22・6・3 項目とも失敗 0。既定のビルド 5 回: 4 回が欠番 0・42.9〜43.0%、**書き込み直後の 1 回目に欠番 29**（開始から約 12 秒で 1 回の publish が 1.19 秒、到着の空き 1205 ms、
    キュー 12、接続は維持、オーバーラン 0、I2C のエラー 0）。今朝の欠番 35（約 60 秒のところ、1.36 秒）と同じ種類。今日の FreeRTOS の計測 約 13 回のうち 2 回で、どちらも書き込み直後の最初の計測。
    原因は未確定（センサーの経路ではなく NINA への送信の側の詰まり）。
- 確かめていない: 切断復帰・Telnet（このイメージでは）、校正のやり直し、長時間。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## 整理 2: OS に依存しないネットワークの層を `common/compat` へ（2026-11-22）

- subagent が実装、メインが照合。移動だけ（include のパス、コメント、ビルドの規則、文書）。
- `common/compat/net_offload/`（自前のネットワーク層を持たない OS 向け）: `subsys/net/lib/sockets/{sockets,getaddrinfo,socket_offload}.c`、`lib/os/zvfs/{zvfs_fdtable,zvfs_poll}.c`、`zephyr_net_if.c`、
  `zephyr_heap.c`、専用の include のルート `include/`（`zephyr/net/{net_if,net_ip,socket,socket_offload}.h`、`zephyr/sys/fdtable.h`、`zephyr_heap.h`）。`k_heap` の宣言は FreeRTOS の `zephyr_os.h` から
  `zephyr_heap.h` へ移し、`zephyr_os.h` が同じ位置で読む。
- `common/compat/libc_net/`（POSIX のネットワークのヘッダーを持たない C ライブラリー向け）: `include/{arpa/inet.h,netdb.h,netinet/in.h,netinet/tcp.h,poll.h,sys/socket.h,sys/uio.h}`、`inet.c`。
- 20 ファイル、約 2500 行。NuttX 版はどちらのディレクトリーも include パスに入れず、コンパイルもしない（自前の `<zephyr/net/...>` を使い続ける）。
- `freertos/drivers/compat` に残ったもの: `zephyr_net_init.c`（中身は OS に依存しないが FreeRTOS だけのヘッダーを読む）、`zephyr_net_mgmt_os.h`（スレッドの起動）、リンカーの断片、`__assert.h`、
  `sys/clock.h`・`zephyr_sys_clock.c`、`drivers/i2c.h`、`libc/stdio_wrap.c`。前の 2 つは整理 5 の対象。
- 照合: FreeRTOS 237056 B は変更前とバイト単位で一致。NuttX・Zephyr も `snapshot-1122e` と一致（`common/imu/mqtt_min_sock.c` のコメントを直したので再ビルドした）。ホストテスト 48・61・58、
  `format.sh --check`（新しい 2 つのディレクトリーも対象に）。実機には書いていない（イメージが同じ）。
- 気づき（今は直さない）: `libc_net/include/arpa/inet.h` はリトルエンディアンの CPU を前提にしている。

## 整理 3・4: 互換層の自己診断とシェルのエンジンを `common/compat` へ（2026-11-22）

- subagent が実装（2 つを分けて。パッチ 2 つで分離）、メインが照合と実機の自己診断。commit も 2 つ。
- 3: `zephyr_selftest.[ch]` → `common/compat/selftest/`。FreeRTOS を呼んでいた約 20 か所（tick、補助のスレッド、優先度、名前、スケジューラーのロック、割り込みの中か）は境界にして、
  `zephyr_selftest.h` に一覧、FreeRTOS の実装は `freertos/drivers/compat/zephyr_selftest_os.h`（static inline。実機とホストで同じ）。22 個のテストは全部共有のファイルにある。
  ボードの部分（アラーム、コンソールの掃き出し、割り込みのマスク）は従来どおり `rp2040/zephyr_selftest_port.c` とホストの `freertos/tests/compat/main.c`。
  - 既定のイメージはバイト単位で一致。自己診断のイメージは +88 B（確認の式を文字列にした失敗時のメッセージが、境界の関数の名前になった分。命令はアドレスを除いて同じ）。実機の `selftest`: 22・6・3 項目とも失敗 0。
  - NuttX 版に入れるなら（今回は入れていない）: `zephyr_selftest_os.h`（`kthread_create`、`nxsched_get_param`、`sched_lock` など）、ボードの部分、走らせる入口。**失敗が見込まれる確認**:
    `sem_reset`（待ち手が待ち続ける。補助のスレッドが終わらないので、飛ばすか作り直しが要る）、`poll_order`（1 回の give で全部の待ち手が起きる）、おそらく `mutex_zero`・`mutex_recursive`
    （`unlock` のエラーの値）、場合により `thread_name`・`timeslice`・tick の数を見る確認。
- 4: `cli.[ch]`・`shell_argv.[ch]`・`nano_shell.[ch]`（約 600 行）→ `common/compat/shell_engine/`（自前のシェルを持たない OS 向け。コマンドと境界 `zshell_os.h` のある `common/shell` とは別）。
  名前は変えていない。`getopt_state.c` は残った（調査の見立てと違い、`nano_console.h` 経由で FreeRTOS を読む）。`nano_console.[ch]`（タスクごとのコンソールと FreeRTOS の mutex）、`nano_stdio.[ch]`、
  `zshell_os_impl.h`・`zephyr_os_shell.h` も残る。既定と自己診断のイメージは 3 のあとの状態とバイト単位で一致。
- NuttX・Zephyr のビルドが読むファイルには触れていない。ホストテスト（FreeRTOS）58、`format.sh --check`（新しい 2 つのディレクトリーも対象）。
- 次の候補: `getopt_state.c`・`nano_console.*`（スレッドごとのコンソールのポインターとロックを境界にすれば共有にできる）、自己診断の境界のうち `thread_create`・`prio_get`・`name_get` は互換層の API にできる。
- 実機: FreeRTOS 版（既定のビルド。センサーの段階 2 のイメージと同じ中身）。

## NuttX 版だけの `nano_` を `rp2040_` に改名（2026-11-22、ユーザーの決定）

- subagent が実装、メインが照合。NuttX 本体のボードのファイルの流儀（`rp2040_bringup.c` など）に合わせた。
- ファイル: ボードの `src/nano_*.c`・`.h` → `rp2040_*`（`nano_connect.h` → `rp2040_pico.h`＝写しのもとの名前）、`include/nano_reset.h` → `rp2040_restart.h`、`scripts/nano-flash.ld` →
  `arduino-nano-rp2040-connect-flash.ld`、`apps/init/nano_init.[ch]`・`nano_telnet.c` → `rp2040_*`、`tests/nano_timeconv.c` → `rp2040_timeconv.c`。関数・型・マクロも同じ（`rp2040_bringup()`、
  `rp2040_oneshot_*`、`rp2040_watchdog_reboot()`、`rp2040_init_main()` など）。
- NuttX 本体とぶつかった名前: `rp2040_reset.c`（ボード共通にある）→ `rp2040_restart.[ch]`、`rp2040_timerisr.c`（パッチ 0004 が外す arch のファイル）→ `rp2040_timerisr_alarm.c`、
  `rp2040_idle.c` → `rp2040_idle_cpuload.c`、`RP2040_HW_DIVIDER`（パッチ 0009）→ `RP2040_PICO_DIVIDER`。
- ボードの Kconfig: `NANO_TICKLESS`・`NANO_PERF`・`NANO_STACK_SENTINEL`・`NANO_FLASH_KV` → `RP2040_*`、`NANO_HW_DIVIDER` → `RP2040_PICO_DIVIDER`、`NANO_ROM_MEM_OPS` は削除
  （arch の `RP2040_ROM_MEM_OPS` を選ぶだけだったので、defconfig が直接設定する）。`.config` の差はこの 6 行と `INIT_ENTRYPOINT` だけ。
- 残した `nano_`: 共有の境界（`include/nano_timer.h` と `nano_mono_us()`、`src/nano_flash_kv.c`、`src/nano_gpio.c`、`NANO_PRIO_*`・`NANO_STACK_*`、`nano_cpuload*`、`CONFIG_NANO_LSM6DSO_STREAM` など。
  整理 7・11 で移すときに名前を決める）、アプリとドライバーの Kconfig の記号（`NANO_APP_*`・`NANO_INIT_*`・`NANO_KV_STORE` など。別の決まりが要る）、`setup.sh` の記録のディレクトリー
  `nano-patches`（変えると既存のクローンが失敗する）。
- パッチ 0004・0007・0009・0010 は説明文だけ新しい名前に（本体はバイト単位で同じ）。
- 照合: 3 版ともイメージは変更前とバイト単位で一致（NuttX 283992 B、FreeRTOS 237056 B、Zephyr 372328 B。改名した名前はイメージに入らない）。ホストテスト 48・61・58。実機には書いていない。
- subagent が迷った点: アプリ（`apps/init`）のファイルにも `rp2040_` を付けたこと、`rp2040_pico.h` という名前（Pico ではないボード）、`RP2040_FLASH_KV` が共有の `nano_flash_kv.c` を有効にする点。

## 整理 5: 2 版で同じだった起動のつなぎを 1 つに（2026-11-22）

- subagent が実装、メインがホストテストと実機確認。
- 共有にしたもの（使う OS の操作は `z_thread_static_start()` だけ）: 新しい `common/compat/zephyr_init.c`（ワークキューのスレッドの本体、`k_work_queue_start()`、`k_sys_work_q`、
  `zephyr_compat_initialize()`、`k_uptime_get()`）、`common/compat/zephyr_net_mgmt.c` の中の net_mgmt のスレッドと起動、`zephyr_net_init.c`（FreeRTOS から `common/compat/net_offload/` へ。
  宣言は `include/zephyr_net_init.h`）。削除: 2 版の `zephyr_net_mgmt_os.h`、FreeRTOS の `zephyr_net_init.c`。
- OS の項目に足したもの: `k_yield()`、`Z_OS_ASSERT()`（スレッドが起きなかったときの各版の反応をそのまま残す）、任意の `Z_SEM_TAKE_FOREVER()`（NuttX 版だけが定義。ワークキューのループが
  `nxsem_wait_uninterruptible()` を直接呼ぶ形を保つため）。OS が実装する関数は 21 + 1 オブジェクトから 18 に。
- 比べて見つかった違いと扱い: 起動のデータの渡し方とスレッドの終わり方（各版の `z_thread_static_start()` に残す）、`zephyr_compat_initialize()` は sysworkq が起きなければ `-ENOMEM`
  （NuttX 版は以前は黙って進んでいた）、NuttX 版の net_mgmt の起動は新しいスレッドが受け取るまで待つ形に（優先度が高くすぐ走るので順序は同じ）、NuttX 版の起動のロックは 1 つに。
- 逆アセンブルの比較（subagent）: `z_work_queue_process`・`k_work_submit`・`k_sem_give`・`k_sem_take` などバッチごとの経路は、両版ともアドレスとリテラルを除いて同じ命令。
  ワークキューと net_mgmt のスレッドは `static_thread_main` を 1 段通るようになった（起動のときだけ）。キュー 1 つあたり RAM が 36 B 増える。
- 行数: `nuttx/drivers/compat` 1559 → 1412、`freertos/drivers/compat`（`rp2040/`・`zephyr/` を含む）2116 → 1910。
- イメージ: NuttX 283956 B（−36）、FreeRTOS 237064 B（+8）、Zephyr は基準と一致。**基準: `snapshot-1122f`**（Zephyr は 1122e と同じ中身）。ホストテスト 48・61・58。
- 実機:
  - NuttX（書き込み直後 約 70 秒）: 欠番 0、46.9%、時刻の逆行 0。`ps`: `sysworkq` 119（312/976）、`RTIO_WQ_0` 118（640/976）、`imu_fifo_worker` 108（840/2008）、`net_mgmt` 119（240/2000）。
    `sensor get` 応答。送信中の Telnet、Wi-Fi を落として 2 秒で再接続、MQTT の再接続（connects 2）、復帰後の Telnet、時刻は synchronized。`dmesg` は意図した切断の警告だけ。
  - FreeRTOS: `selftest` 22・6・3 項目とも失敗 0。既定のビルド 2 回: 欠番 0、43.1%。`ps`: `net_mgmt` 21（空き 1832）、`sysworkq` 21（800）、`imu_fifo_worker` 10（1352）、`RTIO WQ 0` 20（528）。
    `sensor get` 応答。切断復帰と Telnet も NuttX と同じ結果（connects 2、時刻の逆行 0）。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## NuttX 版の Kconfig の記号から `NANO_` を外す、パッチの記録は `local-patches`（2026-11-22、ユーザーの決定）

- subagent が実装、メインが照合。名前だけの変更。
- Kconfig の記号 48 個（`nuttx/apps/*/Kconfig`、`nuttx/drivers/Kconfig`、`nuttx/drivers/lsm6dso/Kconfig`）: `NANO_` を外す（`APP_IMU`・`APP_IMU_PRIORITY`、`INIT_TELNET`、`KV_STORE`、`TIME_SYNC`、
  `IMU_RAW_MQTT`、`WIFI_PROFILE`、`SUBCMD`、`LSM6DSO_STREAM`、`LOG_RAM` など）。Zephyr に同じ意味の記号があるものは Zephyr の名前にした:
  `NANO_LSM6DSO` → `LSM6DSO_LOCAL`（Zephyr の `CONFIG_LSM6DSO` は上流のドライバーで `n`）、`NANO_WIFI_CREDENTIALS_CONNECTION_TIMEOUT` → `WIFI_CREDENTIALS_CONNECT_STORED_CONNECTION_TIMEOUT`、
  `WIFI_CREDENTIALS`・`_MAX_ENTRIES`・`_SAE_PASSWORD_LENGTH`、`LOG_DEFAULT_LEVEL`・`LOG_MAX_LEVEL`（値も 3 版で同じ）。`common/compat/zephyr/net/wifi_credentials.h` の読み替えの 3 か所は既定値の定義だけに。
  NuttX 本体・apps・パッチに同じ名前は無い（`INIT_*` は NuttX 自身の `INIT_PRIORITY` などと同じ接頭辞だが、同名は無い）。`CONFIG_LSM6DSO_STREAM` は Zephyr 版では前も後も未定義。
- `.config` は 48 行が同じ値で名前だけ変わった。3 版ともイメージは変更前とバイト単位で一致（NuttX 283956 B、FreeRTOS 237064 B、Zephyr 372328 B）。ホストテスト 48・61・58。実機には書いていない。
- パッチの適用の記録: `nuttx/nuttx/.git/`・`nuttx/nuttx-apps/.git/`・`zephyr/.west/` の `nano-patches` → `local-patches`。`setup.sh` は古いほうだけがあれば移して 1 行表示し、両方あれば止まる。
  この作業ツリーで移行を実施（3 つとも移動、パッチの再適用なし、取得したツリーの状態は同じ、2 回目は何もしない）。
- 残る `nano_`・`NANO_`（subagent の集計）: フラッシュの KV（`nano_flash_kv*`・`NANO_KV_*`）、FreeRTOS 版のコンソール・シェル・Telnet（`nano_console*`・`nano_shell*`・`nano_telnet*` など）、
  `NANO_PRIO_*`・`NANO_STACK_*`、`nano_gpio*`、タイマー（`nano_mono*`・`NANO_TIMER*`）、`nano_cpuload*`、`NANO_SELFTEST`、Zephyr のアプリの `CONFIG_NANO_HW_DIVIDER`。ファイル名に `nano_` が付くものは 23 個。

## 整理 6: RAM のログを 1 つに（2026-11-22）

- subagent が実装、メインがホストテストと実機確認。
- NuttX 版と FreeRTOS 版の `log_ram.c` を `common/log/log_ram.c` の 1 つに。境界は `common/log/log_ram_os.h`（各版の `log_ram_os_impl.h` を読む。全部 static inline）:
  `LOG_RAM_OS_RETAINED()`（再起動で残る RAM に置く）、割り込みのマスクの保存・復帰、`log_ram_os_uptime_us()`、`log_ram_os_attach()`（OS 自身のログにつなぐ。FreeRTOS は空）。
  NuttX に残るのは syslog のチャネル（`nuttx/drivers/log/log_ram_syslog.c`、64 行）と境界のヘッダー、FreeRTOS は境界のヘッダーだけ（`freertos/drivers/log/` は無くなった）。
  FreeRTOS の `dmesg` コマンドは `common/shell/dmesg_shell.c` へ（Zephyr のシェルの API だけで書かれている）。
- 2 版の違い（境界の後ろに残る）: NuttX では `syslog()` の出力がそのままリングに入る。行の形式、192 バイトの切り詰め、レート制限、割り込みの中や起動前から呼べることは同じ。
- リングの置き場所と形は両版とも変更前と同じ（NuttX 0x20000f14、FreeRTOS 0x20000188、8208 バイト）。
- イメージ: FreeRTOS 237064 B（大きさ同じ。関数の命令はアドレスを除いて同じ、`cmd_dmesg` の位置が動いた）、NuttX 283996 B（+40。ログの 1 行ごとの呼び出しが 1 段減った）。Zephyr は対象外。
  **基準: `snapshot-1122g`**（Zephyr は同じ中身）。
- ホストテスト 49・62・59（新しい `log_ram`: 行の文字列、切り詰め、再起動で残る、壊れたヘッダーで空にする、レート制限、ロックの中でだけ触る）。
- 実機:
  - NuttX（書き込み直後 約 70 秒）: 欠番 0、47.0%。`dmesg` に起動の行、受信器なしの `imu start` の警告、`reboot` のあとも前の行が残る。
  - FreeRTOS: 2 回とも欠番 0、43.2〜43.3%。`dmesg` 同上、`reboot` のあとも残る。`devmem 0x30000000` のあと `fault` に記録（HardFault、タスク console）、`dmesg` に `fault` の行、そのあと通常どおり起動して Wi-Fi 接続。
- 実機: FreeRTOS 版（この変更の既定のビルド。異常停止の記録が 1 件残っている＝こちらの試験）。

## 整理 8: FreeRTOS 版のカーネルの部分を「汎用の実装 + OS の基本の操作」に（2026-11-22）

- subagent が実装（2 段階。それぞれイメージの一致で確認）、メインが照合。commit も 2 つ。
- 段階 1（場所を変えずに）: `zephyr_kernel.c`・`zephyr_mutex.c`・`zephyr_os.h`・`zephyr_os_spinlock.h` から FreeRTOS と Pico SDK の名前を無くし、境界 `zephyr_kernel_os.h`
  （基本の操作 23 個の static inline の関数と型・定数 9 個。それぞれの意味をヘッダーに書いてある）を通す。FreeRTOS の実装は `freertos/drivers/compat/zephyr_kernel_os_impl.h`。
  基本の操作: 割り込みの中か、入れ子にできるスレッドのロック、割り込みのロック、自スレッドと優先度、スレッドの起動・終了・名前、譲る、待ちの準備・待ち（tick の期限つき）・起こす
  （**起こす印は保持されること**。相手がまだ待っていなくてもよい）・割り込みからの起こしと切り替え、スリープ、停止、ビジーウェイト、時計、OS の再帰ミューテックス（優先度の継承は OS のもの）。
  自己診断の OS の項目 12 個のうち 7 個もこの上に載せた。
- イメージの一致を妨げた 3 点と対処（コメントに理由を記載）: 1 回の呼び出しを包む関数を必ずインラインにする（`Z_OS_INLINE`）、ミューテックスの持ち主の比較を整数で行う、
  使われなくなった大域の関数の代わりに同等の `z_yield()` を残す（リンカーのベニアの順序が入力セクションの数で決まるため）。
- 段階 2（移動）: OS に依存しない 8 ファイルを `common/compat/kernel_generic/` へ（`zephyr_kernel.c`・`zephyr_mutex.c`・`zephyr_os.h`・`zephyr_os_spinlock.h`・`zephyr_os_internal.h`・
  `zephyr_kernel_os.h`・`zephyr_compat_os.h`・`zephyr_selftest_os.h`）。NuttX 版はコンパイルしない（自前の実装のまま、ユーザーの決定）。
- FreeRTOS に残るもの: 基本の操作の実装、`zephyr_os_atomic.h`、tick のフック（`tick.c`）、自己診断の FreeRTOS の部分。
- 行数: これまで新しい OS が書く必要があったのは約 1180 行。いま FreeRTOS が書いているのは 437 行（コードは約 230 行）。汎用の部分は 1143 行。
- 候補の OS が満たせないかもしれない要求（subagent の見立て、推測）: 期限つきで保持される起こし（ChibiOS の suspend/resume は保持されない）、入れ子にできて中の起こしで切り替わらないロック、
  スレッドごとの書き換えられる名前、優先度の向き（RIOT は逆）、期限つきのミューテックスと全部 0 が未ロックであること、割り込みの文脈の tick のフック（RIOT は tickless）。
- 照合: 既定 237064 B と自己診断 262264 B は、段階 1 のあとも段階 2 のあともバイト単位で一致。NuttX は `snapshot-1122g` と一致（コメントを直したので再ビルド）。Zephyr は読むファイルが変わらない。
  ホストテスト 49・62・59、`format.sh --check`。実機には書いていない（イメージが同じ）。
- 気になる点: 汎用の `zephyr_os.h` が `net_offload` の `zephyr_heap.h` を読む（汎用のカーネルを使う OS はその include パスも要る）。

## 共有の名前の改名と、整理 7: RP2040 に固有の部分を `common/chip/rp2040/` へ（2026-11-22、名前はユーザーの了解）

- subagent が実装（2 つに分けて）、メインがホストテストと実機確認。commit も 2 つ。
- 名前だけ（3 版と自己診断のイメージがバイト単位で一致）: `NANO_PRIO_*`・`NANO_STACK_*` → `APP_PRIO_*`・`APP_STACK_*`、`nano_gpio*` → `gpio_os*`（`common/compat/gpio_os.h`、NuttX の
  `rp2040_gpio_os.c`、FreeRTOS の `gpio_os_rp2040.c`）、`nano_cpuload*` → `cpu_load_*`（`cpu_load.h` に統合、`struct cpu_load_window`）、`-DNANO_SELFTEST` → `-DSELFTEST`、
  Zephyr の `CONFIG_NANO_HW_DIVIDER` → `CONFIG_RP2040_PICO_DIVIDER`、`nano_mono_us()`・`NANO_TIMER_*` → `rp2040_timer_us()`・`RP2040_TIMER_*`。
  `RP2040_TIMER_BASE` は NuttX 本体にも同じ値の定義がある（同じ字句なので再定義は無害。ヘッダーに記載）。ほかにぶつかる名前は無い。
- 整理 7（イメージが変わる）: `common/chip/rp2040/include/` にヘッダー 2 つ。
  - `rp2040_timer_us.h`: 1 MHz の TIMER の 64 ビットの読み出し（上位・下位・上位を読み、上位が変わっていたらやり直す。ラッチは使わない＝割り込みの中からも安全）。3 か所の実装
    （NuttX のボードのヘッダー、`common/imu/time_sync.c` の中、FreeRTOS は SDK の `time_us_64()`）はどれも同じ方式だった。必ずインライン（`-Os` では複製が外に出て呼び出しになるため）。
    `time_sync.c` のデバイスツリーの参照、互換層の `sys_io.h`、FreeRTOS の `time_sync_prelude.h` は削除。
  - `rp2040_watchdog_reboot.h`: ウォッチドッグでの再起動（Pico SDK と同じ順序）。NuttX と Zephyr（NINA の中継モードの出入り）が共有。Zephyr 版はレジスターの書き込みの順序が少し変わった
    （WDSEL を set のエイリアスで、スクラッチ 4 を先に。結果は同じ）。FreeRTOS は SDK の `watchdog_reboot()` のまま（SDK のドライバーが状態を持つ。指針 3）。
  - 共有にしなかったもの: BOOTSEL への移行（NuttX は ROM の参照、ほかは SDK）、スクラッチレジスター、アイドルの計測（OS のフックとマスク、窓の数が違う）、チップの ID（フラッシュのコマンドは危険が大きい）。
- イメージ: FreeRTOS 237384 B（+320）、NuttX 284076 B（+80）、Zephyr 372328 B（大きさ同じ、中身が違う）。**基準: `snapshot-1122h`**。ホストテスト 50・63・60（新しい `rp2040_timer_us`）。
- 実機（書き込み直後の各 1 回 約 70 秒 + 下記）:
  - Zephyr: 欠番 0、46.0%、時刻の逆行 0（`unix_valid` 4982）、`ntp status` synchronized、`dmesg` 警告なし。`nina passthrough confirm` で中継モードに入り、NINA のフラッシュを 2 回読んで一致
    （書き込み後の退避とも一致）、300 bps で閉じて通常のファームウェアに戻る。
  - NuttX: 欠番 0、47.2%、時刻の逆行 0、synchronized、`ps` の CPU 列（Idle 99.1%）。中継モードの出入りも同じ結果。`reboot` 後に起動。
  - FreeRTOS: `-DSELFTEST=ON` の `selftest` 22・6・3 項目とも失敗 0。既定のビルド 3 回: 欠番 0、42.8〜42.9%、時刻の逆行 0、synchronized。`reboot` のあとログが残る、`devmem 0x30000000` のあと
    `fault` に記録（開始から 12.64 秒＝タイマーの読み出し）。
- 残る `nano_`: フラッシュの KV（整理 11）、FreeRTOS 版のコンソール・シェル・Telnet（整理 9）、`nano_lsm6dso*`（FreeRTOS）、`nano_device_*`。
- 実機: FreeRTOS 版（この変更の既定のビルド。異常停止の記録が 1 件＝こちらの試験）。

## ログを `common/compat/log/` へ、`z_yield()` の削除（2026-11-22、ユーザーの決定）

- `common/log/`（`app_log.h`、`log_ring.[ch]`、`log_ram.[ch]`、`log_ram_os.h`、`fault_record.[ch]`）→ `common/compat/log/`。パスの変更だけ。NuttX は `nuttx/drivers/log/Make.defs` の VPATH と
  各所の `-I`、FreeRTOS は `compat.cmake` とアプリの CMake。3 版と自己診断のイメージはバイト単位で一致（Zephyr・NuttX とも `snapshot-1122h`）。ホストテスト 50・63・60。
  ユーザーとの整理: `app_log.h` は Zephyr の `LOG_*()` の API を Zephyr 以外の版に提供する互換層のもの、リングと RAM のログと異常停止の記録は自前の機能だが、まとめて `common/compat/log/` に置く。
- 整理 8 でリンカーのベニアの順序を保つために残した大域の `z_yield()` を削除（「残さない」）。`k_sleep()` は `z_os_yield()` を直接呼ぶ。FreeRTOS のイメージは大きさ同じ（237384 B）で 208 バイトが違う。
  subagent の逆アセンブルの比較: 関数の命令は全部同じで、違うのはベニア 37 個の並び順とそこを指す所だけ（`z_yield()` はもとのイメージでもインライン化されて残っていなかった）。
  実機（書き込み直後から 3 回 約 70 秒）: 欠番 0、42.8・42.8・42.9%（直前のイメージは 42.8〜42.9%）、時刻の逆行 0。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## 整理 9: コンソール・Telnet・汎用のコマンドを共有に、残りの `nano_` の改名（2026-11-22）

- subagent が実装（4 つに分けて）、メインがホストテストと実機確認。commit も 4 つ。
- A（名前だけ。既定と自己診断のイメージがバイト単位で一致）: `nano_shell.[ch]` → `shell_engine.[ch]`（`shell_engine_*`・`SHELL_ENGINE_*`）、`nano_console` → `shell_console`、`nano_stdio` → `shell_stdio`、
  `nano_telnet` → `shell_telnet`、`nano_ifconfig.c` → `ifconfig_shell.c`、`nano_lsm6dso.[ch]` → `lsm6dso_board.[ch]`。
- A2: 互換層のデバイスの記号 `nano_device_*`・`nano_device_state_*` → `__device_*`・`__devstate_*`（Zephyr のマクロが付ける接頭辞）。FreeRTOS のリンカーは `.bss` を名前順に並べるので
  アドレスが動く（116 語、大きさは同じ）。NuttX は一致。
- B: `id`・`uptime`・`date`・`sleep`・`devmem`・`fault`・`reboot`・`bootsel` を `common/shell/kernel_shell.c` へ。境界 `kernel_shell_os.h` は 4 項目（tick の数、再起動、USB ローダーへの再起動、
  直前の異常停止の記録）。`help` はエンジンへ、`fault_print()` は `fault_record_print()` として `fault_record.c` へ、`ifconfig_shell.c` は `common/shell/` へ。FreeRTOS のアプリに残るのは
  `version`・`free`・`ps`・`selftest` とコマンドの表。NuttX の `apps/bootsel`・`apps/dmesg` は共有のものに替えていない（境界の実装が要り、単純にならない）。
- C: コンソール（`shell_console.[ch]`）、`getopt_state.c`、Telnet（`shell_telnet.[ch]`、790 行）、新しい `shell_serial.[ch]`（`main.c` にあったスレッド `console`）を `common/compat/shell_engine/` へ。
  FreeRTOS の呼び出しは互換層の API に（`k_mutex`、`k_sem`、`z_thread_static_start()`、`k_current_get()`）。基本の操作に 3 つ追加（23 → 26）: `z_os_pre_kernel()`、スレッドごとのポインターの取得・設定
  （Zephyr の `k_thread_custom_data_get/set()` として見せる）。OS が書くコンソールの入出力は `shell_serial_os.h` の 5 項目（FreeRTOS は `freertos/drivers/shell/shell_serial_os_impl.h`、`stdio_usb` の上）。
  スレッドは `console`・`shell_telnet`・`telnet_service` とも優先度 6、スタック 3072・3072・2048 で従来どおり。制御ブロックは FreeRTOS のヒープから取るようになった（`.bss` が 400 B 減り、ヒープの空きが約 430 B 減る）。
- `freertos/drivers` の行数: `shell` 506 → 120、`telnet` 820 → 20、`wifi` 235 → 131。
- イメージ: FreeRTOS 237656 B（+272）。NuttX は `snapshot-1122h` と一致、Zephyr は読むファイルが変わらない。ホストテスト 50・63・60（シェルのテストが `help` と汎用のコマンドの出力、
  偽の入出力の上のシリアルのセッションも確かめる）。
- 実機（FreeRTOS）: 書き込み直後から 2 回 約 70 秒: 欠番 0、**43.8%**（直前は 42.8〜42.9%）。XIP キャッシュの外れは約 36 万回/秒（以前の「悪い配置」は約 32 万回/秒、良い配置は約 28 万回/秒）で、
  配置の影響と見られる（配置だけを変えた対照は取っていない）。コマンドを 1 つずつ: `help`・`version`・`id`・`uptime`・`date`・`sleep 1`・`sleep x`（-22）・`devmem 0 7`（幅の誤り）・`devmem`・`free`・`fault`・
  `ntp status`・`kv list`・`ifconfig`・`nina version`・`i2c_dma stats`・`zperf` とも従来の形。`ps`: `console`・`telnet_service`・`shell_telnet` とも優先度 6。送信中の Telnet、Wi-Fi を落として 2 秒で再接続、
  復帰後の Telnet、2 つ目の Telnet の接続のあとも 1 つ目が使える。`selftest` 22・6・3 項目とも失敗 0。`reboot` のあとログが残る、`devmem 0x30000000` のあと `fault` に記録。
- 残る `nano_`: フラッシュの KV だけ（整理 11）。
- 実機: FreeRTOS 版（この変更の既定のビルド。異常停止の記録が 1 件＝こちらの試験）。

## 計測: NINA への送信の内訳（2026-11-22、ユーザーの依頼。計測のコードは commit していない）

- FreeRTOS 版（整理 9 のあとのソース）に、`nina_proto_request()` の区間ごとの時間をコマンド別に数える計測を入れ、送信中の 60.0 秒を集計した（欠番 0、43.4%）。区間: ロック待ち、準備（READY 待ちと CS）、
  ヘッダーの 1 バイトずつの転送、パラメーターの本体の転送、NINA の処理待ち（READY が戻るまで）、応答の読み出し。
- `SOCKET_SEND`（0x77）: 2184 回（36.4 回/秒）、5,252,376 バイト（1 回 2405 バイト）。1 回あたりの平均: 準備 52 µs、ヘッダー 38 µs、**本体の転送 3117 µs**、**NINA の処理待ち 827 µs**、応答 69 µs。
  合計 約 4.1 ms、最大 6.6 ms（処理待ちの最大 3.2 ms）。
- `SOCKET_POLL`（0x7c）: 2090 回（34.8 回/秒＝送信 1 回につき 1 回）。1 回あたり 準備 63 µs、ヘッダー 39 µs、本体 8 µs、処理待ち 528 µs、応答 68 µs。合計 約 0.7 ms。
- ほかは 2 秒に 1 回の接続状態の確認（0x20）と時刻合わせ（0x79・0x7a）で、合わせて 60 秒に約 40 ms。
- まとめ（60 秒あたり）: 本体の転送 6.81 秒（11.3%。優先度の高いスレッドや割り込みに割り込まれた時間を含む。バイト数 × 1.024 µs の純粋な転送は 5.38 秒＝9.0%）、ヘッダー・準備・応答の
  小さな転送と待ち 約 0.72 秒（1.2%）、NINA の処理待ち 2.9 秒（4.8%。眠って待つ）。バスを使っている時間は合わせて約 17.5%。
- 分かったこと: 送信 1 回の時間の約 4 分の 3 は SPI の転送で、NINA の処理待ちは約 5 分の 1（事前の見立てと逆だった）。平均では送信の経路に約 5 倍の余裕がある。
  送信の回数（2184）は publish の回数（約 2095）より約 4% 多い（1 回で受け取られなかった分の送り直しと見られる。未確認）。この 60 秒には長い詰まり（1.2〜1.4 秒）は無く、その原因はこの計測では分からない。

## 整理 10: アプリの起動の手順とスレッドの一覧を 1 つに（2026-11-22）

- subagent が実装、メインがホストテストと実機確認。
- `common/imu/app_main.[ch]`（3 版）: `app_main()` =「USB console started」のログ、`kv_store_init`（`CONFIG_KV_STORE`）、`imu_cal_init`・`time_sync_init`（`CONFIG_APP_IMU`）、`wifi_profile_init`（`CONFIG_WIFI_PROFILE`）。
  `common/imu/app_init.c`（NuttX・FreeRTOS）: `app_init_post_kernel()`（`wifi_credentials_init`）と `app_threads_start()`（スレッドの一覧。1 行 1 スレッドで、この順:
  nina_scan・nina_connect・nina_monitor（`CONFIG_WIFI_NINA_LOCAL`）、`nina_net_start()`（`CONFIG_NINA_NET`）、imu_stream、imu_raw_mqtt、time_sync、wifi_reconnect、sensor_shell_processing、zperf_server（`CONFIG_NET_ZPERF`））。
  順序は (1) `app_init_post_kernel()` (2) OS のシェルの部分 (3) `app_threads_start()` (4) `app_main()`。Zephyr 版は (1)〜(3) を Zephyr 自身が行う（SYS_INIT と静的なスレッド）。
- オプションの記号: FreeRTOS は各モジュールの cmake が定義（`KV_STORE`・`APP_IMU`・`IMU_RAW_MQTT`・`TIME_SYNC`・`WIFI_PROFILE`・`APP_SENSOR`・`NET_ZPERF`・`WIFI_NINA_LOCAL`）、
  Zephyr はアプリの Kconfig に隠しの記号 3 つ（`KV_STORE`・`APP_IMU`・`WIFI_PROFILE`）。
- 各版の起動のファイル: Zephyr `main.c` 185 → 170 行、NuttX `rp2040_init.c` 383 → 300 行、FreeRTOS `main.c` 329 → 260 行。新しい OS は、デバイスとコンソールを用意して (1)(3)(4) を呼び、ビルドするモジュールの記号を定義する。
- 残した違い: NuttX 版だけ Telnet をスレッドのあとに起動する（Zephyr・FreeRTOS は前。理由は見つからなかったが、そろえていない）。変えた動き: NuttX 版はスレッドの起動に失敗すると 1 行ログを出す（FreeRTOS と同じ。以前は無言）。
- 新しいホストテスト `app_threads`（`common/tests/app_threads.py`）: ソースにある `K_THREAD_DEFINE` と一覧が 1 対 1 であること、各項目のオプションがそのソースをビルドする規則と対応すること。ホストテスト 51・64・61。
- イメージ: Zephyr 372336 B（+8）、NuttX 284236 B（+160）、FreeRTOS 237712 B（+56）。**基準: `snapshot-1122i`**。逆アセンブルで、起動の呼び出しの順序とスレッドを作る順序が前と同じことを確認（subagent）。
- 実機（書き込み直後の各 1 回 約 70 秒 + 下記）:
  - Zephyr: 欠番 0、46.1%、時刻の逆行 0、起動のログの順序は従来どおり（LSM6DSO、fs_nvs、Telnet、USB console started）、警告なし、synchronized、校正済み、Telnet でコマンドに応答、NINA の中継モードの出入り。
  - NuttX: 欠番 0、47.4%、`ps` の順序（nina_scan・nina_connect・nina_monitor、5 つのスレッド、telnetd）、synchronized、送信中の Telnet、Wi-Fi を落として 2 秒で再接続、復帰後の Telnet。
  - FreeRTOS: `selftest` 22・6・3 項目とも失敗 0。既定のビルド 2 回: 欠番 0、43.9〜44.0%。起動のログの順序は従来どおり、synchronized、切断復帰と Telnet も同じ結果。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## 整理 11: フラッシュの KV と異常停止の記録、最後の `nano_` の改名（2026-11-22）

- subagent が実装（3 つに分けて）、メインがホストテストと実機確認（KV は先に退避: `host/recordings/kv-backup-20261122-step11.json`、Git 管理外）。commit も 3 つ。
- A（名前だけ。3 版と自己診断のイメージがバイト単位で一致）: `nano_flash_kv.h` → `flash_kv.h`（共有の部分: レイアウトの判定、ページの補助、消去）と `flash_kv_os.h`（各版が実装する境界:
  `flash_kv_os_ready/read/program/erase/initialize`）。`nano_kv_*`・`NANO_KV_*` → `flash_kv_*`・`FLASH_KV_*`。NuttX `rp2040_flash_kv_os.c`、FreeRTOS `flash_kv_os_rp2040.c`。
  `script/flash_uf2.sh` のテスト用のフック `NANO_FLASH_*` → `FLASH_UF2_*`。**これで `nano_` の名前は無くなった。**
- B: 2 版が別々に書いていた RP2040 の部分（領域がイメージのあとでフラッシュの中にあることの確認、読み・書き・消去の範囲の検査、キャッシュなしの写像での読み出し、ページの RAM への写し）を
  `common/chip/rp2040/rp2040_flash_kv.c` に 1 つに。各版が書くのは `rp2040_flash_kv_os_*` の 4 関数（開始、イメージの終わり、書き込み、消去。割り込みを止めた状態でもう一度範囲を確かめる）。
  生のフラッシュの操作は各版のまま（NuttX は RAM に置いたブート ROM の呼び出し、FreeRTOS は Pico SDK の `flash_range_*`。命令と RAM 上の配置は変更前と同じ）。NuttX 本体のフラッシュのドライバーは
  使えない（イメージのあとを SmartFS 用に消す）。変わった点: イメージの終わりがフラッシュの先頭以下なら拒否する検査が両版に（以前は FreeRTOS だけ）。
- C: 異常停止の記録のうち、例外フレームの読み取り（RAM の範囲と整列の検査つき）と、起動時にどの記録を使うかの選択を `common/compat/log/fault_record.[ch]` へ。割り込みの入口、タスクの名前、
  スクラッチレジスター、再起動は FreeRTOS の `fault.c` のまま（共有にできるものは少なかった）。
- テスト: 新しい `rp2040_flash_kv`（偽のフラッシュを実際のアドレス 0x13ffe000 に置き、領域の前後 8 KiB の全部のオフセット、境界を ±1 と ±4 KiB ずらした 16 通り、読み出しの両端）、
  `flash_kv` の拡張、`fault_record` の拡張（RAM の端のフレーム、不正なフレーム 7 通り、記録の選択）。ホストテスト 52・65・62。
- イメージ: NuttX 284368 B（+132）、FreeRTOS 237912 B（+200）、Zephyr は変わらない。**基準: `snapshot-1122j`**。
- 実機:
  - FreeRTOS: 既存の KV を認識（`fs_nvs` の行、`flash_kv` のエラーなし）、`imu.port` 読める、試験用のキーを書く → 読める → `reboot` 後も読める → 削除 → `reboot` 後は無い（-2）、校正済みのまま、
    Wi-Fi は起動後すぐ接続。送信 2 回: 欠番 0、44.0%。`devmem 0x30000000` のあと `fault` に記録（HardFault、タスク console、pc・lr・sp とレジスター）。`selftest` 22・6・3 項目とも失敗 0。
  - NuttX: KV の確認は同じ結果。送信: 欠番 0、46.5%、時刻の逆行 0。
- 確かめていない: スタックあふれの経路（安全に起こす手段が無い。ホストテストと、変えていないフックだけ）。
- **整理 1〜11 はこれで全部終わり。** 実機: FreeRTOS 版（この変更の既定のビルド。異常停止の記録が 1 件＝こちらの試験）。

## 整理のあとのユーザーの判断（2026-11-22）

- NuttX 版だけ Telnet をスレッドのあとに起動する件: このままでよい。
- FreeRTOS 版の `cpu_active` が整理の前の 42.8% から 44.0% になった件（配置の影響と見られる）: 仕方ない。
- FreeRTOS 版で書き込み直後の最初の計測に出ることがある送信の詰まり（2026-11-22 に 2 回、1.2〜1.4 秒）: 再現していないので様子を見る。

## レビュー（codex、4 つ目の OS を足す人の目線）への対応と、単独の「版」の書き換え（2026-11-22）

- 指摘 1（汎用のカーネルを選ぶとネットワークの部品の include パスも要る）: `kernel_generic/zephyr_os.h` が `net_offload/include/zephyr_heap.h` を読んでいた。`k_heap` を互換層の共通のカーネルの API にした:
  宣言は `common/compat/zephyr_heap.h`（`common/compat/zephyr/kernel.h` の最後で読む。どちらのカーネルの部分でも使える）、実装は `common/compat/zephyr_heap.c`（`k_spin_lock` だけに依存）。
  NuttX 版はコンパイルしない（使う所が無い）。新しい検査 `kernel_generic_independent`（`net_offload`・`libc_net` の include パスなしで汎用のカーネルがコンパイルできる。最小の OS の部分は
  `common/tests/kernel_generic_host/`）と `net_offload_independent`（逆向き。NuttX 式の偽のカーネルの上で）。部品どうしの依存は境界の文書の §1.1 に表にした。
- 指摘 2（IMU だけの構成に要る代わりの実装が NuttX 版に隠れている）: `time_sync_null.c`・`imu_raw_mqtt_null.c` を `nuttx/apps/imu/` から `common/imu/` へ。決まり（`CONFIG_APP_IMU` は両方の API を必要とし、
  モジュールを外すなら null のファイルをリンクする）を `app_main.h` に明記。NuttX の別の構成で確認（subagent）: MQTT と時刻合わせを外して 273508 B、NINA と usrsock も外して 238712 B でビルドできる。
- 指摘 3（LSM6DSOX の構成が入口から分からない）: 境界の文書のドライバーの表のセンサーの行を埋めた。冒頭に「新しい OS で書くもの」（7 段階）、§7「例: IMU だけの構成」（NuttX の構成から確認）を追加。
  ルートの README（日英）: 「LSM6DSO は Zephyr のものにローカルのパッチ」という古い案内を直し、「3 つの OS のファームウェアと `common/`」の節から境界の文書へリンク（メイン）。
- 単独の「版」（ユーザーの指摘: 何を指すか分からない。件数にかかわらず全部直す）: 日本語の文書の全部で書き換えた（`design/imu-raw-mqtt-design-ja.md` 45、`nuttx-porting-ja.md` 26、`nuttx/drivers/README_ja.md` 18、
  `freertos/drivers/README_ja.md` 14、`zephyr/.../drivers/README_ja.md` 12、`os-boundary-ja.md` 11、`host/README_ja.md` 6、ルート・`freertos/`・`nuttx/` の README）。各文書の冒頭で 3 つの OS のファームウェアを説明し、
  「各 OS のファームウェア」「OS の側」、バージョンの意味なら「バージョン」と書く。「Zephyr 版」などの呼び名はそのまま。作業ログ（`PORTING_*.md`）は対象外。
- 照合: 3 版と自己診断のイメージは変更前とバイト単位で一致（基準は `snapshot-1122j` のまま）。ホストテスト 54・67・64。

## NINA の SPI の長い転送を DMA に（2026-11-22〜23、ユーザーの依頼: FreeRTOS で試し、効果があれば Zephyr・NuttX へ）

- 背景（同日の内訳の計測）: 送信 1 回 約 2400 バイトのうち SPI の転送が 3.1 ms、その間 CPU を回して待つ（CPU の約 9〜11%）。
- 作り（境界 `port/nina_bus.h` は変えない。64 バイト以上の転送だけ DMA、短いものはポーリングのまま）:
  - FreeRTOS: `common/drivers/nina/port/freertos/nina_bus_dma_rp2040.[ch]`。Pico SDK からチャネル 2 本（RX がデータレジスターを読み切った時点で完了。RX のバッファが無ければ 1 バイトの捨て先）、
    DMA_IRQ_1 を I2C と共有して自分の印だけ扱う、待ちはタスク通知（NINA の番号 2）、時間切れ（2 ms + 長さ/256 ms）・RX のオーバーラン・バスのエラーはログを出して `-EIO`、チャネルが 2 本取れなければ
    ポーリングのまま。Pico SDK の共有の割り込みハンドラーの上限を 6 → 8 に（6 つ全部を使い切ったため。RAM 24 B）。
  - NuttX: 本体の SPI ドライバーの DMA のモード（`CONFIG_RP2040_SPI_DMA`、`CONFIG_RP2040_SPI_DMATHRESHOLD=63`）。ローカルのパッチ 0013: チャネルを待たずに取り、2 本取れなければポーリング。
    **本体のドライバーは DMA の完了を期限なしで待ち、エラーを呼び出し元に返さない**（ポーリングのときも同じ。期限は足していない）。
  - Zephyr: PL022 のドライバーの DMA のモード（`CONFIG_SPI_PL022_DMA`、`&spi1` に `dmas`＝チャネル 10・11）。ローカルのパッチ 0007: `CONFIG_SPI_PL022_DMA_MIN_LEN`（これ未満はポーリング。長さ 0 の転送は
    従来どおりロックを取るだけ）、DMA のあと SSPDMACR を消す、失敗・時間切れでチャネルを止める。CS を下げているあいだバスを持つ保証は変わらない（`SPI_ASYNC` なしなら `SPI_LOCK_ON` でロックが残る）。
    `nina_bus.c` の検査は、割り込みのモードと `SPI_ASYNC`、しきい値なしの DMA を禁じる。時間切れは 長さ × 1 µs + 200 ms。
- 実機、データの照合（送るデータの全バイトをバッチの番号と位置から決まる値にする一時的な変更 + PC 側で全バイトを照合。どちらも commit していない。DMA あり → なし → あり）:
  FreeRTOS 8737 バッチ・2055 万バイト、NuttX 7683 バッチ・1807 万バイト、Zephyr 8097 バッチ・1904 万バイト、不一致はどれも 0。
- 実機、同じイメージで DMA のあり・なしを切り替えた `cpu_active`（各 3 回、約 70 秒、全部欠番 0・時刻の逆行 0・オーバーラン 0・I2C のエラー 0）:
  FreeRTOS（試作のイメージ）32.5% / 42.4〜42.5%、NuttX（`mw` を足した試験用のイメージ）37.1〜37.2% / 47.4〜47.5%、Zephyr（最終のイメージ）34.6〜34.7% / 44.1%。
- 最終のイメージ: FreeRTOS 239280 B、33.1%（2 回）。NuttX 284788 B、36.4%。Zephyr 374572 B、34.6%。**基準: `snapshot-1123a`**。ホストテスト 56・67・64。
- そのほか（3 つとも）: Wi-Fi を落として再接続、送信中と復帰後の Telnet、スキャン、エラーのログなし。NINA の中継モードは FreeRTOS と Zephyr で確認（NuttX は今回は未確認）。
  スタック: FreeRTOS・NuttX は変わらず。Zephyr の `sysworkq` は Telnet のセッションのあと 824/1024（DMA なしでも同じ。以前からの値で、DMA とは無関係。余裕は 200）。
- 確かめていない: 長時間、DMA のエラーの経路（実機では一度も起きていない。ホストテストだけ）、NuttX 版の中継モード。
- 実機: Zephyr 版（最終のイメージ）。

## 計測: FreeRTOS 版の XIP キャッシュの外れ（2026-11-23、ユーザーの依頼）

- イメージ: `b2dc30c` の既定のビルド（239280 B、NINA の SPI は DMA）。`devmem 0x1400000c`・`0x14000010` をクリアして 15 秒後に読む（`xip` の計測。送信開始から 10 秒後）。
- 計測を止めた状態: アクセス 約 27 万回/秒、外れ 約 5,000 回/秒（1.87%）。
- センサーの読み出しだけ（AP が落ちていて送信できなかった 5 回。`cpu_active` 27.5%）: アクセス 約 775 万回/秒、外れ 24.1〜24.2 万回/秒（3.11〜3.12%）。
- 送信中、DMA あり（3 回、33.0〜33.1%、欠番 0）: アクセス 816〜817 万回/秒、外れ 34.0〜34.5 万回/秒（4.16〜4.22%）。
- 送信中、同じイメージで DMA なし（3 回、43.1〜43.2%、欠番 0）: アクセス 812〜813 万回/秒、外れ 33.1〜33.6 万回/秒（4.07〜4.13%）。
- 分かったこと: DMA のあり・なしで、アクセスも外れもほとんど変わらない（ポーリングの転送はフラッシュへのアクセスをほとんど生んでいなかった）。送信は外れを約 10 万回/秒 増やす。
  過去の値（DMA の前）: 良い配置 約 28 万回/秒（40.9%）、悪い配置 約 32 万回/秒（42.7%）、整理 9 のあと 約 36 万回/秒（43.8%）。
- 途中で PC の AP（`rp2040-test`）が落ちていた（こちらは止めていない）。ユーザーが `script/wifi_ap.sh` で起動し直した。

## 計測: コードを RAM に置いたときの XIP キャッシュの外れと `cpu_active`（FreeRTOS 版、2026-11-23、ユーザーの依頼）

- 同じソース（`b2dc30c` + 試験用の cmake の切り替え、未 commit）から、コードのまとまりを RAM に置いたイメージを 13 個作り（subagent。Pico SDK のリンカースクリプトの差し替え口
  `default_text_excludes.incl`・`default_rodata_excludes.incl` でオブジェクト単位に移す。切り替えが空なら元のイメージとバイト単位で一致）、それぞれ実機に書いて送信中に 15 秒 × 2 回測った。全部欠番 0。
- まとまり: kernel（FreeRTOS の tasks・queue・port・timers と tick・idle、SDK の sync）、compat（互換層のカーネル・ワークキュー・RTIO・GPIO）、i2c（`i2c_dw` と DMA）、lsm6dso（ドライバー一式）、
  imu（`imu_stream`・バッチ・時刻）、send（MQTT・ソケット・NINA）、usb（stdio_usb・TinyUSB・pico_time）、libc（除算・memmove など）。sensor = compat + i2c + lsm6dso + imu。

| イメージ | RAM に移した量（コード / 定数） | `cpu_active` | アクセス/秒 | 外れ/秒 |
|---|---|---|---|---|
| base（今のイメージ） | 0 | 33.1・33.2% | 818 万 | 34.2 万 |
| kernel | 11.0 kB / 0.3 kB | 29.6% | 714 万 | 24.5 万 |
| compat | 5.6 kB / 0.1 kB | 28.5・28.6% | 763 万 | 22.3 万 |
| i2c | 4.5 kB / 0.3 kB | 29.8% | 740 万 | 25.8 万 |
| lsm6dso | 9.4 kB / 0.9 kB | 30.0・30.1% | 566 万 | 25.8 万 |
| imu | 10.5 kB / 1.7 kB | 30.8% | 698 万 | 27.8 万 |
| send | 18.1 kB / 0.6 kB | 25.9・26.0% | 670 万 | 17.5 万 |
| usb | 9.2 kB / 0.2 kB | 33.9・34.1% | 813 万 | 36.4 万 |
| sensor | 30.0 kB / 3.1 kB | 21.4% | 310 万 | 3.2 万 |
| sensor（コードだけ） | 30.0 kB / 0 | 21.3% | 309 万 | 3.2 万 |
| sensor + kernel | 41.0 kB / 3.4 kB | 20.0% | 190 万 | 0.64 万 |
| sensor + kernel + send | 59.0 kB / 4.0 kB | 19.0・19.1% | 29 万 | 数百 |
| 全部（+ usb・libc） | 70.4 kB / 4.3 kB | 18.9% | 9 万 | ほぼ 0 |

- 分かったこと: 外れは `cpu_active` の約 14 ポイントを占めていた（33.1% → 18.9%）。センサーの経路（30 kB）を移すだけで 11.7 ポイント下がり、外れは 10 分の 1 になる（足し合わせより大きい。
  残りが 16 kB のキャッシュに収まるようになるため）。定数を移す効果は見えない。usb だけを移すと逆に少し悪い（配置の変化）。kernel を足して −1.4、send を足して −1.0、usb・libc を足して −0.1。
- RAM: 空きは 83.4 kB（newlib の malloc の領域。使うのはシェルだけ）。sensor で 48.6 kB、sensor + kernel で 37.6 kB、全部で 7.4 kB が残る。
- 実機は元のイメージ（`b2dc30c` の既定のビルド）に戻した。試験用の切り替え（`freertos/app/usb-console/ram_code.cmake` と `CMakeLists.txt` の 2 行）は作業ツリーにあり、未 commit。

- 追加の 3 個（2026-11-23、ユーザーの依頼。コードだけを RAM に、各 2 回、欠番 0）。RAM の増分は `.data` の増え方（中継の命令を含む）:

| イメージ | RAM の増分 | `cpu_active` | アクセス/秒 | 外れ/秒 |
|---|---|---|---|---|
| i2c + lsm6dso + imu（互換層なし） | 26.5 kB | 23.7・23.8% | 363 万 | 10.1 万 |
| i2c + lsm6dso + imu + send（互換層なし） | 44.9 kB | 20.1% | 211 万 | 2.3 万 |
| compat + i2c + lsm6dso + imu + send | 49.9 kB | 19.7% | 155 万 | 0.59 万 |

  互換層（5.6 kB）は、送信なしの構成では −2.3 ポイント（23.7 → 21.4%）、送信ありの構成では −0.4 ポイント（20.1 → 19.7%）。送信（18 kB）は、互換層なしで −3.6、互換層ありで −1.7。
  実機は元のイメージに戻した。

## FreeRTOS 版: よく通るコードを RAM に置く（2026-11-23、ユーザーの決定: 構成 D）

- ユーザーの決定: compat + i2c + lsm6dso + imu + send のコードだけを RAM に（計測の表の最後の行。19.7%）。めったに通らない関数はそれほど大きくないと見て、RAM のままでよい。
- 仕上げ（subagent）: `freertos/app/usb-console/ram_code.cmake`。1 つの一覧 `RAM_CODE_SOURCES`（46 ファイル）を、Pico SDK のリンカースクリプトの差し替え口 `default_text_excludes.incl` で
  フラッシュの `.text` から外す（SDK の `.data` が受け取り、起動時に RAM へ写される）。オプションは無い（試験用の切り替えと、使わないまとまりは削除）。定数（`.rodata`）はフラッシュのまま。
  検査: 一覧のファイルが無ければ cmake が止まる。リンクのあとで map を読み、一覧のオブジェクトの `.text` が RAM に無い・フラッシュに残っている、空きの RAM が 16384 B 未満、のどれかでビルドが失敗する。
- イメージは計測したものとバイト単位で一致（242176 B）。空きの RAM: 既定 33,564 B、`-DI2C_DW_I2C1=ON` 33,204 B、`-DSELFTEST=ON` 18,764 B、両方 18,400 B。
- 安全の確認（subagent、map と逆アセンブル）: 動いたのは 46 個のオブジェクトだけ。異常停止の経路は移した関数を呼ばない。`.data` の写しの前に走るコードは移した中に無い。KV の書き込みの
  割り込み禁止の区間に移したコードへの経路は無い。中継モードの UART の割り込みは変わらない。スタックとヒープは変わらず、減るのは newlib の malloc の領域（シェルのコマンドが 1 回 約 1.2 kB）。
  RAM に入った割り込みハンドラー: GPIO（`gpio_os_isr`・`nina_os_gpio_isr`）、DMA（I2C と NINA）、I2C。
- 実機（既定のビルド）: 書き込み直後から 2 回 約 70 秒: 欠番 0、19.7%、時刻の逆行 0。XIP の外れ 約 5,700 回/秒（アクセス 157 万回/秒）。Wi-Fi を落として 2 秒で再接続、送信中と復帰後の Telnet。
  `help`・`wifi scan`・`sensor get`・`dmesg`・`imu status`・`ps`・`ntp status`・`kv get` の前後で `free` の値は同じ（ヒープの空き 30448、malloc の使用 1196）。KV: 試験用のキーを書く → `reboot` 後も読める →
  削除 → `reboot` 後は無い、校正済み・`imu.port` はそのまま、Wi-Fi は起動後すぐ接続。`devmem 0x30000000` のあと `fault` に記録。NINA の中継モードの出入り。
- 自己診断（`-DSELFTEST=ON`）で I2C の 1 項目が失敗 → 原因は RAM への配置ではなく、NINA の SPI の DMA 化（`25a0335`）: テストが「I2C の DMA はチャネル 0・1」と決め打ちしていて、NINA が 0・1 を
  持つようになった（I2C の読み出しは 2・3 を使って正常）。テストを、読み出しの前に空いている下から 2 つを調べてそれを確かめる形に直した（別の commit）。直したあと 22・6・3 項目とも失敗 0。
  **DMA 化の commit のとき、実機の自己診断を走らせていなかった**（ビルドが通ることだけ確かめていた）。
- 文書: ルートの README（日英）の性能の表（FreeRTOS 19.7%、イメージの大きさを今の値に）、`freertos/drivers/README`（「RAM に置くコード」の節）、設計書 §13。
- ユーザーの依頼（次）: RAM を空けるためにタスクを減らせないかの検討（Wi-Fi 関連は役割ごとのタスクを状態でまとめられないか、IMU も）。`sensor_shell_processing_tid` は名前が長く `ps` の枠に入らないので短くする。
- 実機: FreeRTOS 版（この変更の既定のビルド。異常停止の記録が 1 件＝こちらの試験）。

## スレッド `sensor_shell_processing_tid` → `sensor_shell_tid`（2026-11-23、ユーザーの依頼）

- 理由: 27 文字で、FreeRTOS 版の `ps`（名前の欄は 20 文字）の表が崩れる（Zephyr 版の `kernel ps` も同じ幅）。3 つの OS のファームウェアで `sensor_shell_tid`（16 文字）に。
- 定義は Zephyr 本体の `drivers/sensor/sensor_shell_stream.c`（`sensor_shell.c` ではない）。パッチ 0006 を作り直して、その 1 行も変える（`patches.yml` の sha を更新）。検査 `zephyr_sensor_shell_patched` は
  このファイルも HEAD から取り出す。NuttX・FreeRTOS は `common/compat/sensor_shell_stream.c` の定義と `common/imu/app_init.c` の表。
- イメージ: Zephyr 374560 B（−12）、NuttX 284780 B（−8）、FreeRTOS 242168 B（−8）。命令は 3 つともアドレスを除いて同じ（subagent の比較）。**基準: `snapshot-1123b`**。ホストテスト 56・67・64。
- ほかのスレッドの名前: 20 文字を超えるものは無い（最長は Zephyr の `net_socket_service` と `wifi_reconnect_tid` の 18 文字）。
- 実機（FreeRTOS）: `ps` の表がそろう、`sensor get`、`sensor stream` on・off、送信 1 回 約 70 秒 欠番 0・19.7%。Zephyr・NuttX は実機に書いていない（命令が同じ）。

## 検討: タスクを減らして RAM を空ける（2026-11-23、ユーザーの依頼。読むだけの調査）

- FreeRTOS 版のスレッドは 19 個、スタック 34.8 kB + 管理用 約 2.9 kB、起動用の `main` のスタック 2 kB（起動後は使われない）。分かっている最大の使用量は合わせて約 13.7 kB。
  スレッド 1 つの費用はスタック + 約 144 B。
- まとめて空く量は小さい: NINA の scan・connect・monitor を 1 つにして約 2.9 kB、まとめられるものを全部で約 7 kB。
  - NINA の 3 つ: scan と connect は互いに `-EBUSY` で断る、monitor は接続済みのときだけ働く。同時に動くのは「接続を保ったままのスキャン中の監視」だけ。状態（待機・接続中・スキャン中・結果の取得）を
    持つ 1 つのスレッドにできる。境界 `nina_os.h` に期限つきの待ちが要る。共有のコードで 300〜400 行（テストを含む）。
  - IMU の 4 つ（`imu_fifo_worker`・`imu_stream_tid`・`imu_raw_mqtt_tid`・`RTIO WQ 0`）はまとめない（読み出し・組み立て・送信を分けて優先度に差を付けている。FIFO はしきい値からあふれるまで約 28 ms）。
  - `net_mgmt` を `sysworkq` に載せない（コールバックが表示で待つ。`sysworkq` は INT1 の中継）。`Tmr Svc` は無くせない（Pico SDK のロックが待ち手を起こすのに使う）。
- 大きいのは別の所（見積もり）: heap_4 が 32 kB で使用 2.3 kB（中継モードの 16 kB のリングを取るため）→ 4 kB で 28 kB、`main` のスタック 2 kB、`heartbeat` をアイドルの処理へ 1.2 kB、
  FreeRTOS 版だけのスタックを使用量に合わせて 3.8 kB、共有のコードのスタックを OS ごとにして 2.1 kB、`zperf` のバッファを測定のときだけ 4.6 kB、再起動で残るログ 8 → 4 kB、送信待ちのバッチ 12 → 8 で 9.8 kB。
- ユーザーの判断待ち: どれを行うか。

## RAM を空ける: ヒープ・スタック・バッファ・送信待ちのバッチ（2026-11-23、ユーザーの決定）

- ユーザーの決定: ヒープは 8 kB、共有のコードで決めているスタックはそのまま、NINA の 3 つのタスクは 1 つにまとめる（次の作業）、送信待ちのバッチは 12 → 8、
  再起動で残るログは OS ごとに変えず、4 kB にそろえられるならそろえ、できないなら 8 kB のまま。
- 空きの RAM（`.heap` の終わりから `__HeapLimit`。既定のビルド）: 33,564 → ヒープ 58,140 → 起動用のスタック 60,188 → LED 61,340 → スタック（確定後）約 64.4 kB → `zperf` 約 69.0 kB →
  送信待ちのバッチ 約 78.9 kB（subagent の報告の値からスタックの戻し 1,664 B を引いた見積もり）。
- 1. heap_4 32 kB → 8 kB。中継モードの 16 kB のリングは newlib の `malloc()` から（その起動で最初の確保。リンクのあとの検査の下限 16384 B で足りる）。ヒープを使うのは互換層のスレッドの
  TCB（1 つ約 145 B、16 個で 2,320 B）だけ。自己診断は補助のタスクが残るので `#ifdef SELFTEST` で 16 kB（実機の最大の使用 7,456 B）。
- 2. 起動用のタスク `main` のスタック（2 kB）を、動かさない 2 つ目のコアのスタックの領域（SCRATCH_X）へ。SDK のリンカースクリプトの差し替え口 `section_extra_post_scratch.incl`。
- 3. `heartbeat` のタスクをやめ、アイドルのフックが点滅させる（`idle_blink.h`。`cpu_load_enter()` 用に読んだ時刻を使い、アイドルとして数える窓の中で判定）。新しいホストテスト `idle`。
- 4. スタック（FreeRTOS 版だけの値は `src/priorities_os.h`。`common/compat/priorities.h` の 4 つのマクロは `#ifndef` に）: IDLE 1024 → 256、Tmr Svc 1024 → 384、sysworkq 1024 → 384、
  sensor_shell_tid 3072 → 2048。**`net_mgmt` と `telnet_service` は 2048 のまま**: subagent の案（960・1600）で実機の最悪の場合を動かしたところ、Telnet からの `wifi scan` で `net_mgmt` が
  スタックあふれ（異常停止の記録: stack overflow、タスク net_mgmt）。測り直した最大の使用は `net_mgmt` 1024（以前の測定 632 は USB からの操作だけだった）、`telnet_service` 1164
  （読み出しを止めたクライアント。1600 では空き 27%）。確定後の最悪の場合の空き: IDLE 56%（自己診断で 40%）、Tmr Svc 58%、sysworkq 47%、sensor_shell 46%、net_mgmt 50%、telnet_service 43%、
  console 48%、shell_telnet 50%。
- 5. `zperf` のバッファ（約 4.6 kB）を測定のときだけ `malloc()` で確保。`PICO_MALLOC_PANIC=0`（`malloc()` の失敗は NULL を返す。SDK の既定は異常停止）。
- 6. 送信待ちのバッチ 12 → 8（`IMU_RAW_QUEUE_CAPACITY`。3 つとも約 9.8 kB 空く。8 バッチは約 0.23 秒分）。
- 7. 再起動で残るログ: **8 kB のまま**（変更なし）。記録（`nuttx/PORTING_STATUS.md`）: 4096 B では `PANIC()` の報告の先頭（Assertion failed、最初のレジスター）が押し出された。タスク 20 個で
  報告 1 回 3,831 B（1 タスク 148 B）。今は 20〜22 個で約 3.8〜4.1 kB なので、4 kB では報告だけで埋まる。
- イメージ: FreeRTOS 242024 B、NuttX 284772 B、Zephyr 374544 B。**基準: `snapshot-1123c`**。ホストテスト 56・67・65。
- 実機（FreeRTOS、確定後）: 送信 2 回 欠番 0・19.5〜19.6%、XIP の外れ 約 5,900 回/秒。`free`: ヒープ 8192 のうち空き 5872。受信器を 2 秒止める試験: `queue_high` 8、落とした 59 = 欠番 59、
  接続は維持、時刻の逆行 0。`zperf` の upload・download（iperf3 で 2.5 Mbit/s）・途中で止めた upload のあと、`malloc` の使用は元に戻る（download の間だけ +2.5 kB）。中継モードの出入り、
  `reboot` のあとログが残る、`devmem 0x30000000` の記録。`-DSELFTEST=ON`: `selftest` 2 回とも 22・6・3 項目 失敗 0、ヒープ 16384 のうち最少の空き 8928。LED の点滅は目視していない。
- 実機（NuttX・Zephyr、送信待ち 8）: NuttX 欠番 0・36.2%。Zephyr 欠番 0・**36.0%（3 回）**（DMA 化の直後は 34.6%。スレッドの改名と今回でイメージが数十バイト変わった。配置の影響と見られる）。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## 自己診断の廃止、FreeRTOS 版のログを 4 kB に（2026-11-23、ユーザーの決定）

- 経緯: 自己診断のビルドだけヒープが 16 kB 要る → ユーザー「それなら自己診断なしでも 16 kB に」→ 続けて「自己診断を廃止して 8 kB にするのはありかも」→ 3 案（16 kB にそろえる・廃止・補助のタスクの
  解放を待つよう直して 8 kB）を示し、ユーザーの決定は **廃止**（不具合の検出にそれほど役に立っていない）。
- 廃止: `-DSELFTEST=ON`、`selftest` コマンド、`freertos/drivers/i2c_dw/i2c_dw_selftest.[ch]`・`lsm6dso/lsm6dso_selftest.[ch]`・`compat/rp2040/zephyr_selftest_port.c`。ヒープは 8 kB の 1 つ。
  既定のイメージは変更前とバイト単位で一致（242,024 B）。
- 残したもの: 互換層の 22 項目はホストテスト（`zephyr_kernel_test`）として `common/tests/zephyr_kernel/`（`zephyr_kernel_test.[ch]`、OS の部分の境界 `kernel_generic/zephyr_kernel_test_os.h`。
  FreeRTOS の実装は `freertos/tests/compat/zephyr_kernel_test_os_impl.h`）。
- 実機でしか確かめられず、失ったもの: WHO_AM_I と実際のセンサーの値・レート・温度、本物の NACK、本物の DMA の転送のあとのレジスター、パッドの設定、I2C1 の 100 ms 未満のタイミング、
  FreeRTOS の実体（インスタンスと RTIO の大きさ）を通した 6667 Hz の 500 ms のストリーム。
- ログ（ユーザーの決定: Zephyr が 4 kB なら、8 kB は NuttX だけに）: `LOG_RAM_SIZE` を各 OS の `app_log_os.h` に（FreeRTOS 4096、NuttX 8192。定義が無ければ `#error`）。印は大きさを織り込んだまま
  （4096 で 0x4c4f5752、8192 で 0x4c4f6752、Zephyr 版は 0x4c4f4752）なので、別の大きさや別の OS のリングは起動時に捨てられる。ホストテストは両方の大きさで走る（`log_ring_4096`・`_8192`、`log_ram_*`）。
  NuttX のイメージは基準とバイト単位で一致。空きの RAM は 85,004 B。ホストテスト 58・69・67。
- 実機（FreeRTOS）: `free` はヒープ 8192・空き 5872。書き込み直後の `dmesg` は起動の 7 行だけ（前のリングは捨てられる）。`reboot` を 12 回で 3,965 バイトまで埋まり、折り返しても壊れた文字は無い
  （先頭の 1 行は途中から）。`devmem 0x30000000` のあと `fault` の記録と起動の行が残る。NuttX → FreeRTOS、Zephyr → FreeRTOS と書き換えた直後の `dmesg` は起動の 7 行だけ。`selftest` は「command not found」。
  送信 1 回 約 70 秒: 欠番 0、19.5%。
- 途中で: Claude Code のステータスラインに「6 background tasks」。4 つは、終了した subagent が残した `until ! pgrep -f host_tests.sh; do sleep 3; done` の待ちのループ（自分自身を見つけ続けて終わらない）。
  止めた。途中で指示を足すと実行中のコマンドが裏に回るので、subagent の終了後に取り残しを確かめる。
- ユーザーの決定（次の作業）: FreeRTOS のヒープを無くす（スレッドの管理用の領域も静的に。案 1）。空き 18 kB 以上の検査は、シェルの `malloc()` の分の保証として残す（中継モードのバッファは今のまま `malloc()` から）。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## FreeRTOS のヒープを無くす（2026-11-23、ユーザーの決定: 案 1）

- ユーザーの問い: シェルも `malloc()` を使っているなら、FreeRTOS のヒープは要るのか → 使い手は互換層のスレッドの TCB（16 個、2,320 B、起動時に 1 回）だけ。静的にして、カーネルの動的な確保を切る。
- スレッドの TCB は `K_THREAD_STACK_DEFINE()` が取る配列の後ろに置く: `common/compat/zephyr/kernel.h` の `K_THREAD_STACK_RESERVED`（既定 0。Zephyr にも同じ名前・同じ目的のマクロがある）、
  汎用のカーネルでは基本の操作の定数 `Z_OS_THREAD_RESERVED`（FreeRTOS は `sizeof(StaticTask_t)` を 8 に切り上げた 136）。使えるスタックの大きさは変わらない。スレッドごとに自分の領域を持つので、
  数がずれることが無い。NuttX は 0 で、イメージは基準とバイト単位で一致。
- 取り除いたもの: `FreeRTOS-Kernel-Heap4`、`configTOTAL_HEAP_SIZE`・`configSUPPORT_DYNAMIC_ALLOCATION`（0 に）・`configUSE_MALLOC_FAILED_HOOK`、`vApplicationMallocFailedHook`、
  異常停止の理由 `FAULT_REASON_HEAP`（値 5 は欠番として残す。`fault_reason_name(5)` は "unknown"）、`main()` の試しの確保。
- `free` は 1 行: `malloc: N bytes from the system, in use N, free N, more to take N`（最後は `_sbrk()` の上限までの残り）。
- 空き 16384 B の検査（`ram_code.cmake`）はそのまま（ユーザーの決定: シェルの `malloc()` の分の保証として残す）。中継モードのバッファは今のとおり `malloc()` から。
- イメージ 240,960 B（−1,064）。空きの RAM 85,004 → 91,116 B。逆アセンブルの比較（subagent）: バッチごとの経路は同じ命令。違いは TCB の 1 バイトの項目が無くなった分の、項目のオフセット 1 か所。
  ホストテスト 58・69・67。
- 実機: 起動、`ps`（スレッドと優先度は同じ、スタックの空きも同程度）、`free` の 1 行（使用 1196、まだ取れる量 86016）。送信 2 回 欠番 0・19.5%。最悪の場合の一通り（センサー、開始・停止の繰り返し、
  USB と Telnet からのスキャン、Telnet の出入り、Wi-Fi の切断と再接続）のあと、スタックの空きは 41% 以上。`zperf` の upload 3.77 Mbit/s・download 2.43 Mbit/s、前後で `malloc` の使用は同じ
  （download の間だけ 3732）。中継モードの出入り。`devmem 0x30000000` の記録。`reboot` 5 回。
- 実機: FreeRTOS 版（この変更の既定のビルド。異常停止の記録が 1 件＝こちらの試験）。

## 計測: `sensor stream` の負荷（FreeRTOS 版、2026-11-23、ユーザーの依頼）

- イメージ: `fb2c038` の既定のビルド。USB のコンソールで `sensor stream lsm6dsox@6a on fifo_wm incl`、約 32 秒、`ps` の `cpu_active`（前回の `ps` からの区間）。計測を止めた状態は 0.7%。
- 104 Hz: 3.1%（表示 約 1,700 バイト/秒、トリガー 6.5 回/秒、表示したサンプル 209 個/秒）。
- 833 Hz: 数秒で `Read failed: -105` で止まる（区間の平均 1.1%）。6667 Hz: すぐ `Read failed: -139` で止まる（0.8%）。起動直後でレートを設定していない状態: `Read failed: -22`。
  高いレートで続かないのは以前からの動き（README に記載）で、この経路では 6667 Hz の負荷は測れない。
- 校正にかかわる計算: ジャイロのバイアスの追跡（`lsm6dso_track_gyro_bias()`）は FIFO を読むたびに必ず走る（アプリの `imu start` の 19.5% にも入っている）。校正の適用（`lsm6dso_cal_apply()`）は
  デコーダーを通すときだけ（`sensor stream` の表示と `sensor get`）。アプリの経路は生の値と係数を送り、換算は PC 側。

## NINA の Wi-Fi のスレッドを 3 つから 1 つに（2026-11-23、ユーザーの決定）

- subagent が実装、メインがホストテストと実機確認。`nina_scan_tid`・`nina_connect_tid`・`nina_monitor_tid` → `nina_wifi_tid`（Zephyr の優先度 8、スタック 2048）。3 つの OS のファームウェアとも。
- コア（`nina_wifi.c`）: 要求（SCAN・CONNECT）はビットの語で、`nina_os_wifi_lock()` の下で扱う。スレッドの 1 回の通過は、監視の時刻が来ていれば確認（`monitor_once()`）、要求があれば
  接続（`connect_once()`）かスキャン（`scan_once()`。結果の取得まで）、どちらも無ければ次の監視の時刻まで待つ。接続中・スキャン中の待ちは `busy_sleep_ms()` で、監視の時刻をまたぐなら
  そこで分けて確認を挟む。間隔・回数（100 ms を最大 10 秒、500 ms × 60、2 秒、2 回で切断）、エラーの値、イベントとその順序、`-EBUSY`・`-EALREADY` の決まり、公開の API は同じ。
- 境界 `port/nina_os.h`: `nina_os_wifi_post(void)`（どのスレッドからでも。保留は 1 つまで）と `nina_os_wifi_wait(ms)`（起こされるか時間が来るまで。理由なく早く戻ってもよく、コアが要求と時計を見直す）。
  Zephyr はセマフォ 1 つ、NuttX は `nxsem_tickwait_uninterruptible()`、FreeRTOS はタスク通知（番号 2）。
- 変わる動き: 監視は優先度 8 で走る（以前は 9）。接続した直後の最初の確認はちょうど 2 秒後（以前は 0〜2 秒のどこか）。結果の取得（約 41 個の短いコマンド）のあいだは確認が待つ。
  切断の検出までの最悪の時間は同じ。
- スタック: subagent の案は 1536。実機で Zephyr 版の使用が 1024/1536（空き 33%）と基準ぎりぎりで、最も深い場合（スキャン中に切断を検出して、切断の通知と再接続の要求が重なる）は AP を落とさないと
  動かせないので、2048 にした（メイン）。2048 での使用: Zephyr 1024（空き 50%）、NuttX は 1536 のとき 896、FreeRTOS は空き 1236/2048。
- 空く RAM（スタック 2048 のとき、スタックの分）: 以前の 3 つ 4096 B → 2048 B と、管理用の領域 2 つ、待ちのオブジェクト。FreeRTOS の空きは 91,116 → 約 93.7 kB。
- イメージ: Zephyr 374,608 B、NuttX 284,868 B、FreeRTOS 240,968 B。**基準: `snapshot-1123d`**。ホストテスト 58・69・67（コアのテストに、スキャン中の監視、結果の取得中の要求の拒否、
  接続中の確認、時刻ちょうどの要求、10 秒・30 秒の時間切れを追加。各 OS の待ちのテストを書き直し）。
- 実機:
  - FreeRTOS: 起動後すぐ接続、`ps` に `nina_wifi_tid`（優先度 12）だけ。スキャン（接続中・送信中・未接続）、`wifi disconnect`・`auto_connect` を 3 回（2 秒で復帰）、接続中のスキャンは「Scan request failed」、
    時刻は synchronized、エラーなし。送信 3 回 欠番 0・19.4〜19.6%。Wi-Fi を落として 2 秒で再接続、送信中と復帰後の Telnet。
  - NuttX: 送信 欠番 0・36.2%、スキャン（接続中・未接続）、切断復帰と Telnet、synchronized。
  - Zephyr: 送信 欠番 0・35.8〜35.9%、スキャン、切断と再接続 3 回、接続中のスキャンは拒否、Wi-Fi を落として再接続、送信中と復帰後の Telnet、synchronized。
- 確かめていない: AP を落としたときの検出（約 4 秒）と復帰、スキャン中に AP を落とす場合（AP の操作が要る。コアのホストテストだけ）。
- 送信中のスキャン（ユーザーの問い: 送信待ち 12 → 8 が効いていないか）: FreeRTOS で送信中に `wifi scan` を 1 回。変更前（タスク 3 つ、待ち 8）欠番 53・55、変更後（待ち 8）56・79・84、
  変更後で待ちを一時的に 12 に戻して 78・47。スキャン中は送信が 0.6〜1.1 秒止まり（約 22〜38 バッチ分）、8 でも 12 でも満杯になる。差はばらつきの中。
  ユーザー: 8 でよい。スキャン中に送信が止まるのは分かっていたこと。
- 実機: FreeRTOS 版（この変更の既定のビルド）。

## 計測: 表示を止めた `sensor stream`（FreeRTOS 版、2026-11-23、ユーザーの依頼。計測用の変更は commit していない）

- `sensor_shell.c` のコールバックの `shell_print()`・`shell_info()` を空にした一時的なビルド（換算と校正の適用は行う、表示だけしない）。約 32 秒、`ps` の `cpu_active`。計測を止めた状態は 0.6%。
- 104 Hz: 2.0%（表示ありは 3.1%）。読み出し + バイアスの追跡 + 1 サンプルずつの換算（校正の適用）で約 1.4 ポイント、表示で約 1.1 ポイント。
- 833 Hz: 1.0 秒で `Read failed: -105`（ENOBUFS）。3333 Hz・6667 Hz: 0.1〜0.2 秒で止まる。表示を止めても続かない。
- シェルの読み出し用のプール（`RTIO_DEFINE_WITH_MEMPOOL(sensor_read_rtio, 8, 8, 32, 64, 4)`＝2 kB、しきい値 128 語のバッチ約 2 個分）をアプリと同じ 128 ブロックにしても、833 Hz は 1.8 秒で
  `Read failed: -139`（EOVERFLOW＝FIFO のオーバーラン）、3333 Hz は 0.5 秒、6667 Hz は 0.2 秒で止まる。
- 分かったこと: `sensor stream` が高いレートで続かないのは表示のせいではなく、シェルの側の処理（1 サンプルずつデコーダーを呼ぶ。チャネルの種類ごとにバッファを見直す）が追いつかないため。
  104 Hz での約 1.4 ポイントを単純に 64 倍すると約 90% で、6667 Hz はこの経路では処理しきれない計算になる（833 Hz で止まる理由は CPU だけでは説明できておらず、未調査）。
  6667 Hz のデータはアプリの `imu start`（生の値のまま送る）が受け持つ。実機は commit 済みのイメージに戻した。

## Zephyr 版・NuttX 版: よく通るコードを RAM に置く、RAM の削減（2026-11-23、ユーザーの依頼）

- subagent が実装（NuttX は subagent の fork）、別の subagent が RAM の調査（読むだけ）、メインが実機の計測と確認、RAM の削減の実装。FreeRTOS 版と `common/` のコアは変えていない
  （変えた共有のファイルは Zephyr のつなぎ `common/drivers/nina/port/zephyr/nina_passthrough_os.c` だけ）。
- **Zephyr**: `CONFIG_CODE_DATA_RELOCATION=y` と `zephyr_code_relocate(FILES … LOCATION RAM_TEXT NOKEEP)`。一覧は `zephyr/app/usb-console/ram_code.cmake`（51 ファイル、コード 55.5 kB:
  カーネルのオブジェクト（sem・mutex・poll・work・timeout・queue・mem_slab、atomic、RTIO、GPIO）9.2 kB、I2C + DMA 5.9 kB、LSM6DSO 9.3 kB、IMU 10.5 kB、送信 20.6 kB）。
  リンクのあとの検査 `ram_code_check.py`（並べたものが RAM に無い・並べていないものが RAM にある・残りが下限より少ない、で失敗）。
  パッチ 0008（`gen_relocate_app.py`、7 行）: 生成器が「オブジェクトのディレクトリー名にソースの親ディレクトリー名が入っている」ことで探すので、モジュールのライブラリー
  `lsm6dso_local` のオブジェクトを見つけられず、黙ってフラッシュに残した（検査が最初のビルドで検出）。同じ名前のオブジェクトがほかに無ければそれを使う、を足した。
  異常停止の経路は RAM に移したコード（atomic、必要なら `k_sem`・timeout・`k_work`）を通る（FreeRTOS 版との違い。ドライバー説明に記載）。
- **NuttX**: パッチなし。ボードのリンカースクリプトの `.text` を `EXCLUDE_FILE(一覧)` にし、`.data` に `*(.text .text.*)` を足す（既存の `.data` の写しが運ぶ）。一覧は 41 オブジェクト、
  コード 44.6 kB（互換層 4.1、I2C 4.7、LSM6DSO 9.3、IMU 10.7、送信 15.9）。`nuttx/script/check_ram_code.py`（`build.sh` から）: 一覧と配置、`.data` の写しの前に通るコード・
  異常停止の経路・`svc` を含むコードが RAM に無いこと、ヒープ 80 KiB 以上。`svc` の制約: ARMv6-M では割り込みを止めた状態の `svc` を `arm_hardfault` が中継するのは PC が
  `_stext`〜`_etext` のときだけ。
- 実機の計測（送信 70 秒、各 2 回、別々に書き込んだイメージ。欠番・時刻の逆行はすべて 0）:

  | イメージ | Zephyr `cpu_active` | XIP の外れ | NuttX `cpu_active` |
  |---|---|---|---|
  | いままで（フラッシュから実行） | 35.8% | 約 390,000 回/秒（アクセス 923 万回/秒） | 36.1% |
  | 基本の構成（採用） | 21.4% | 約 5,200 回/秒（アクセス 218 万回/秒） | 23.8% |
  | + スケジューラーまわり | 21.2%（+4.3 kB） | 約 1,300 回/秒 | 23.1%（+5.6 kB） |
  | + ネットワークの層（NuttX だけ） | — | — | 22.8%（さらに +6.7 kB） |

  スケジューラーまわりは入れない: Zephyr は 0.2 ポイント、NuttX は 0.7 ポイントだが検査の決まり（異常停止の経路を RAM に置かない）に反する。
  NuttX の XIP のカウンターは `xd` で読めるがクリアできない（`mw` が無い）ので測っていない。
- **RAM の削減**（調査の結果のうち、ユーザーが選んだものと設定だけのもの）:
  - Zephyr の中継モードのリング 16 KiB を静的な配列から `malloc()`（`pt_start()`、中継モードのときだけ。取れなければ `k_panic()`）へ（ユーザーの決定）。検査の下限を 18432（16 KiB + `malloc()` の分 2 KiB、
    FreeRTOS 版と同じ）に。ホストテストの偽のヘッダーに `k_panic()`。
  - Zephyr: `CONFIG_NET_TC_RX_COUNT=0`（`rx_q[0]` は何も運んでいない）、`CONFIG_NET_TCP_WORKQ_STACK_SIZE=512`（使用 304）、`CONFIG_THREAD_MAX_NAME_LEN=20`（最長 18 文字）。
  - NuttX: `CONFIG_DISABLE_MQUEUE`・`_SYSV`、`CONFIG_PREALLOC_TIMERS=0`、`CONFIG_IDLETHREAD_STACKSIZE=512`（使用 316/488）。
  - 見送り（ユーザー: 無理しなくてよい）: Zephyr の IP の層を外す、コンソールの受信バッファ、NuttX の履歴。スタックを実測に合わせて減らす案（各 1 kB 前後）も今回は入れていない。
  - RAM の残り: Zephyr 104,912 B（RAM にコードなし）→ 47,592（コードを RAM に）→ 63,960（リング）→ **66,696 B**。NuttX のヒープ 158,668 → 112,012 → **114,300 B**
    （実機の `free`: total 114,292、used 24,140、maxused 27,072＝中継モード以外を一通り動かしたあと）。
- 最終のイメージ（RAM の削減まで入れたもの）: Zephyr 378,392 B、NuttX 287,224 B。**基準: `snapshot-1123e`**。ホストテスト 58・69（FreeRTOS は変更なし）。
- 最終のイメージでの実機:
  - Zephyr: 書き込み直後から 2 回 21.9%・欠番 0・時刻の逆行 0（基本の構成だけのイメージの 21.4% との差は配置）。送信中の Telnet 3 回、`kv set`・再起動・`kv get`・`kv delete`・再起動、
    `devmem 0x30000000` のあと `dmesg` に HARD FAULT の記録、`wifi disconnect` と再接続（2 秒）、中継モード（115200 と 921600 で NINA のフラッシュ 2 MB を読み、2 回が一致、
    あふれ 0）。スレッドに `rx_q[0]` なし、`tcp_work` 304/512、ほかのスタックは以前と同じ水準。
  - NuttX: 2 回 23.5・23.6%・欠番 0。送信中の Telnet、切断（欠番 375 は切断中の分）と再接続（2 秒）、復帰後の Telnet、時刻 synchronized、KV の書き込み・再起動・削除、
    `xd 0x30000000 4` のあと `dmesg` に異常停止の記録、中継モード（同じ読み出しで 2 回が一致。SPI の DMA 化のあと未確認だった項目）。
- 確かめていない: `zperf`・`iperf`、長時間、AP を落とす場合。NuttX のヒープの最悪の使用（見積もり 約 55〜70 kB、下限 80 KiB）は実測していない。
- 実機: NuttX 版（この最終のイメージ）。

## `sensor stream` が 833 Hz 以上で止まる原因と、デコーダーの数え直し（2026-11-23）

- ユーザーの依頼（対策を考える。案 A から）。メインが実装と実機の計測（FreeRTOS 版、USB のコンソール、各 32 秒）。
- **案 A（commit）**: `lsm6dso_decoder.c` の `decode()` は、呼ばれるたびに先頭から現在の位置まで「このチャネルの語の数」を数え直していた（タイムスタンプのため）。シェルは 1 フレームずつ呼ぶので、
  バッチの語数の 2 乗の時間になる。フレームの反復子（`fit`、呼ぶ側には中身の見えない値）の上位 16 ビットにその数を持たせ、数え直しを無くした（状態を持たないまま）。
  位置より多い数は `-EINVAL`。ホストテストの期待値を更新し、1 件追加。
- **A だけでは 833 Hz は直らなかった**（1.6 秒で `Read failed: -105`）。バッチごとの数を表示から読むと、15・19・23・…・115・125 サンプルと毎回 約 9% ずつ増えて、FIFO の語数がシェルのプールの
  バッファ（2 kB、約 280 語）を超えたところで -105（`lsm6dso_fifo_read()` の「バッファに入りきらない」）。1 語あたり約 0.7 ms = I2C が 100 kHz の速さ。
- **原因: I2C のバスが 100 kHz のまま**（ボードの既定）。`imu start` は開始時に `i2c_configure(… I2C_SPEED_FAST_PLUS)` で 1 MHz にする（止めても戻さない）が、`sensor stream` は何もしない。
  833 Hz × 2 チャネル × 7 バイト + タイムスタンプ ≒ 12 kB/秒 は、100 kHz で運べる量（約 11 kB/秒）を超える。以前の計測で「`imu start` のあとなら動く」ことに気づかなかったのは、
  毎回、再起動の直後に測っていたため。
- 計測（`imu start`・`imu stop` を 1 回通してバスを 1 MHz にしたあと。しきい値も 128 語になる。表示あり）:

  | レート | A なし | A あり |
  |---|---|---|
  | 104 Hz | 3.1% | 1.7% |
  | 833 Hz | 18.4% | 6.7% |
  | 3333 Hz | 0.2 秒で -105 | 23.8%（表示 14.2 kB/秒） |
  | 6667 Hz | 0.1 秒で -105 | 37.5%（表示 16.6 kB/秒） |

  100 kHz のままでは、A ありでも 833 Hz 以上は止まる（上のとおり）。一時的に「バッファを FIFO にある語数に合わせて取る」変更（案 B の変形）も試したが、100 kHz では同じところで止まった（戻した）。
- A のあとのイメージ: Zephyr 378,360 B、NuttX 287,184 B、FreeRTOS 240,920 B。**基準: `snapshot-1123f`**。ホストテスト 58・69・67。FreeRTOS 版でアプリの送信 19.4%・欠番 0（アプリの経路はデコーダーを通らない）。
- 残り: `sensor stream` のときにバスを 1 MHz にする方法（ユーザーの判断待ち）。README の「ODR を上げると止まる」の記述はそのあとで直す。
- 実機: FreeRTOS 版（A を入れたビルド）。

## I2C0 を起動時から 1 MHz に（2026-11-23、ユーザーの決定: 案 1）

- Zephyr: `app.overlay` の `&i2c0 { clock-frequency = <I2C_BITRATE_FAST_PLUS>; }`。NuttX: `board.h` の `BOARD_I2C0_FREQUENCY` 1000000（互換層のバスのデバイスが持つ速さ。`i2c` ツールは自分の `-f`）。
  FreeRTOS: `i2c_dw_instance.h` の `I2C0_SPEED`（`I2C1_SPEED` は 100 kHz のまま）。コアはコントローラーを 100 kHz で始め、最初の転送がデバイスの速さに合わせる（コアは変えていない）。
  `imu start` の `i2c_configure()` と、そのためのバスのデバイスの参照は削除。
- 検査: `i2c_dw_sync.py` は「Zephyr のデバイスツリー = FreeRTOS の `I2C0_SPEED` = NuttX の `BOARD_I2C0_FREQUENCY`」を比べる形に。FreeRTOS の I2C のテストは最初の転送が 1 MHz（31/31）であることを確かめ、
  そのあと 100 kHz に設定して従来の流れ。NuttX の `lsm6dso_driver` はボードがバスをつないだあとの周波数を 1 MHz に。ホストテスト 58・69・67。
- 速さを変えるコマンド: Zephyr 版・FreeRTOS 版は `i2c speed i2c@40044000 <0〜3>`（バスの設定が変わる）、NuttX 版は `i2c … -f <Hz>`（ツールの設定だけ）。
- イメージ: Zephyr 378,328 B、NuttX 287,088 B、FreeRTOS 241,248 B。**基準: `snapshot-1123g`**。
- 実機（再起動の直後、`imu start` を通さずに `sensor stream … on fifo_wm incl`、watermark 32、USB、各 30 秒）:

  | レート | FreeRTOS | NuttX | Zephyr |
  |---|---|---|---|
  | 104 Hz | 動く（2.6%、1.7 kB/秒） | 動く | 動く |
  | 833 Hz | 動く（13.6%、13.4 kB/秒） | 動く（13.4 kB/秒） | 動く（12.5 kB/秒） |
  | 3333 Hz | 動く（51.9%、53.9 kB/秒） | 動く（46.3 kB/秒） | 0.2 秒で -105 |
  | 6667 Hz | 動く（37.6%、16.6 kB/秒） | 0.2 秒で -105 | 0.1 秒で -105 |

  アプリの送信: FreeRTOS 19.5%、Zephyr 21.4%、NuttX 23.6%、欠番 0。
- 残る -105: 表示が追いつかないとバッチが大きくなり、シェルのプール（2 kB、約 280 語）に入りきらない語数になったところで止まる。FreeRTOS 版の 6667 Hz は、バッチが大きくなったところで釣り合って続く
  （表示が 3333 Hz より少ないのはそのため）。対策の候補は、シェルのプールを FIFO 全体（512 語、約 3.6 kB）が入る大きさにすること（上流のシェルへのパッチ 0006 と写しの変更）。未着手。
- 実機: NuttX 版（この変更のビルド）。

## シェルの読み出しのプールを 4 kB に（2026-11-23、ユーザーの依頼）

- `sensor_shell.c` の `RTIO_DEFINE_WITH_MEMPOOL(sensor_read_rtio, 8, 8, 32, 64, 4)` を 64 ブロック（4096 B）に。FIFO 全体（512 語 × 7 B + ヘッダー）が入るので、
  表示が追いつかないときは -105 にならず、バッチが大きくなって釣り合う。Zephyr はパッチ 0006 を作り直した（一時的な index。`setup.sh` が前の組を戻して当て直すので、
  ツリーの手での変更は先に戻しておく）。RAM は各 2 kB 増: Zephyr の残り 64.7 kB、NuttX のヒープ 112.3 kB。
- イメージ: Zephyr 378,328 B、NuttX 287,088 B、FreeRTOS 241,248 B（大きさは変わらない。`.bss` だけ）。**基準: `snapshot-1123h`**。ホストテスト 58・69・67。
- 実機（条件は前の節と同じ）:

  | レート | FreeRTOS | NuttX | Zephyr |
  |---|---|---|---|
  | 104 Hz | 動く（2.6%） | 動く | 動く |
  | 833 Hz | 動く（13.6%、13.4 kB/秒） | 動く（13.4 kB/秒） | 動く（12.5 kB/秒） |
  | 3333 Hz | 動く（52.0%、53.9 kB/秒） | 動く（46.3 kB/秒） | 動く（7.7 kB/秒） |
  | 6667 Hz | 動く（37.6%、16.6 kB/秒。約 105 サンプル/バッチで安定） | 動く（8.7 kB/秒。約 202 サンプル/バッチで安定） | 0.2 秒で -139 |

  Zephyr 版のアプリの送信は 21.4%・`published` = `batches`（NuttX 版・FreeRTOS 版の送信はこのビルドでは測っていない。変わったのはシェルのプールの大きさだけ）。
- Zephyr 版の 6667 Hz（未解決）: バッチが 15 → 70 → 120 → 165 → 206 サンプルと増えて、FIFO（256 サンプル分）があふれる。バッチの大きさから逆算すると、
  1 バッチの表示にかかる固定の時間が約 8 ms（FreeRTOS は約 2.3 ms）、1 サンプルあたりの換算は約 0.05 ms（FreeRTOS は約 0.04 ms）、I2C の読み出しが 0.09 ms（周期は 0.15 ms）。
  試したが足りなかった設定（入れていない）: `CONFIG_SHELL_BACKEND_SERIAL_TX_RING_BUFFER_SIZE` 8 → 256（2 回目のバッチ 180 → 84 サンプルに減るが、あふれる）、
  さらに 1024 + `CONFIG_SHELL_PRINTF_BUFF_SIZE` 128（ほぼ同じ）。3333 Hz のときの表示が約 17.6 kB/秒（TX 256 のとき）で頭打ちに見えるので、USB の CDC ACM の送信の経路
  （ワークキュー経由）が遅いと見ているが、確かめていない。
- 実機: Zephyr 版（このビルド）。

## 計測: 表示を止めた `sensor stream`、やり直し（FreeRTOS 版、2026-11-23、ユーザーの依頼。計測用の変更は commit していない）

- 前の同名の節は、デコーダーの数え直し・I2C の 100 kHz・2 kB のプールのために 833 Hz 以上が止まっていた。それらを直したあとの `94eb166` に、`sensor_shell.c` の
  `shell_print()`・`shell_info()` を空にする一時的な変更を足して測った（換算と校正の適用は行う）。再起動の直後、watermark 32、各約 32 秒、`ps` の `cpu_active`。
- 止めた状態 0.6%、104 Hz 1.7%、833 Hz 6.3%、3333 Hz 23.0%、6667 Hz 36.7%。4 つとも 30 秒止まらずに動いた。表示ありは 2.6%・13.6%・52.0%・37.6%
  （6667 Hz は表示ありでもバッチが大きくなって表示が減るので、差が小さい）。アプリの `imu start`（生の値のまま MQTT で送る）は 6667 Hz で 19.5%。
- ユーザーの判断: 6667 Hz は結果を取れればよく、表示したいわけではないので、Zephyr 版の 6667 Hz の表示は追わない。
- 実機: FreeRTOS 版（`94eb166` のビルドに戻した）。

## Zephyr の `i2c` コマンドを FreeRTOS 版と NuttX 版に（2026-11-24、ユーザーの依頼）

- きっかけ: バスの速さを変えるコマンドが Zephyr 版にしか無かった（前の節の「FreeRTOS 版も `i2c speed`」は誤り。FreeRTOS 版に `i2c` は無く、NuttX の `-f` はツールの転送だけ）。
- `common/shell/i2c_shell.c`: Zephyr の `drivers/i2c/i2c_shell.c` にパッチ 0004 を当てたものの写し（`common/tests/run.sh` の `zephyr_i2c_shell_patched` がバイト単位で照合）。
  最初は `sensor_shell` に合わせて `common/drivers/i2c_shell/` に置いたが、ユーザーの指摘で `common/shell/` に移した（指針 2: Zephyr の API で書いたコマンドは `common/shell/`）。
  `sensor_shell` を同じく移すかどうかは未決。
- FreeRTOS: `freertos/drivers/i2c_shell/i2c_shell.cmake`、互換層の `i2c.h` に `i2c_get_config()`・`i2c_recover_bus()`・ターゲットの宣言、つなぎの API に `recover_bus`（-ENOTSUP、Zephyr のドライバーと同じ）。
  `get_config` は Zephyr のドライバーにも無いので足していない（`i2c speed` は -ENOSYS を受けて `I2C_MODE_CONTROLLER | I2C_SPEED_SET(n)` を設定する）。コアと `i2c_dw_sync` は変更なし。
- NuttX: `nuttx/apps/i2c`、互換層の `zephyr_i2c.c` に `i2c_transfer()`（Zephyr のフラグを NOSTOP・NOSTART に直す、4 メッセージまで）・`i2c_get_config()`（ここだけ値を返す）・`i2c_recover_bus()`（`I2C_RESET`）。
  NuttX 標準の `i2c` ツール（`CONFIG_SYSTEM_I2CTOOL`）、apps のパッチ 0007、ツールだけが使っていた `/dev/i2c0`・`/dev/i2c1` の登録を削除。
- 共有: `shell_hexdump()`（`common/shell/zshell.c`）、`DEVICE_API_IS()` の i2c のクラス。オプションの I2C1 は互換層のデバイスの一覧に無く、`i2c` コマンドから使えない（NuttX 版では使う手段が無くなった）。
- イメージ: Zephyr は `snapshot-1123h` と同じ、NuttX 283,808 B（−3,280、ヒープ 112,052、RAM のコード 44,862 B）、FreeRTOS は text +2,848 B。**基準: `snapshot-1124a`**。ホストテスト 59・71・68。
- 実機（FreeRTOS 版と NuttX 版、どちらも同じ結果）: `i2c read_byte … 6a 0f` = 0x6c、`read`（16 進の表示）、`direct_read`、`write_byte`、`scan`（パッチ 0004 の表示で失敗）、`recover`（-134、NuttX は -138）、
  `speed … 4` と無いデバイス名は失敗。`i2c speed … 1`（100 kHz）のあと 833 Hz の `sensor stream` は約 3 秒でオーバーラン（FreeRTOS -139、NuttX -75）、`i2c speed … 3` で動く。
  アプリの送信: FreeRTOS 19.3%・欠番 0、NuttX 24.7%・欠番 0（NuttX は前の 23.5〜23.8% より高い。イメージの配置が変わった。1 回だけの計測）。Zephyr 版はイメージが同じなので測っていない。
- 実機: NuttX 版（このビルド）。

## `sensor_shell.c` を `common/shell/` へ（2026-11-24、ユーザーの決定）

- `common/drivers/sensor_shell/sensor_shell.c` → `common/shell/sensor_shell.c`（`i2c_shell.c` と同じ理由。`common/drivers/sensor_shell/` は無くなった）。参照（cmake、NuttX の `apps/sensor` の VPATH、
  ホストテスト、`run.sh` の照合、`patches.yml` の説明、文書）を直した。FreeRTOS のホストテストは `../common/shell/*` にプレリュードを付けない決まりなので、`sensor_shell.c` だけ付ける行を足した。
- イメージ: FreeRTOS と Zephyr は移動の前とバイト単位で同じ。NuttX は同じ大きさ（283,808 B）で、ビルド時刻と、読み出し専用の文字列の並び順だけが違う（リンクの順）。
  基準 `snapshot-1124a` の NuttX を取り直した。ホストテスト 59・71・68。
- 実機（NuttX 版）: `i2c read_byte`、`i2c speed` 1 → 833 Hz の `sensor stream` が止まる・3 → 動く、アプリの送信 24.7%・欠番 0。
- 実機: NuttX 版（このビルド）。
