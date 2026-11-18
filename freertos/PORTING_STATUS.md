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
| Z01、Z02、CP-Z | 完了（2026-11-18） | `13ca85d` まで | Z01 は NINA を共通コアへ。Z02 は共通コアへ載せ替えず、上流のドライバー + ローカルのパッチ（ユーザーの決定）。実機確認は各節 |

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
