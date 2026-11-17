# 共通化と FreeRTOS 版の計画（2026-11-17 にユーザーが承認）

NuttX 移植（`nuttx/PORTING_PLAN.md`）と同じ進め方で、(1) Zephyr 版と NuttX 版の共通部分を `common/` に
切り出し、(2) ドライバーを狭い OS 境界の上に揃え、(4) Pico SDK + FreeRTOS 版を作る。このファイルと
`freertos/PORTING_STATUS.md`・`freertos/PORTING_NOTES.md` は未追跡のまま置く（commit しない。置き場所はユーザーの指示）。

## Context

- 目的は移植性の高いドライバー群（LSM6DSOX、NINA-W102、I2C0 と DMA）。OS はスレッド型（Zephyr、NuttX、
  FreeRTOS、RIOT、ChibiOS）とイベント型（OpenMV、Contiki-NG、Tock、Embassy）の 2 ファミリーに分ける。
  **この計画が扱うのはスレッド型の Zephyr、NuttX、FreeRTOS だけ**。イベント型と RIOT・ChibiOS は対象外。
- 出発点: Zephyr 版と NuttX 版は実機で確認済み。NuttX 版のソースの多くは Zephyr 版の写しで、同名ファイルの
  約 37 本が完全に同一、10 本が数十行以下の差、4 本が大きな差。`freertos/` には環境と CLI だけがある
  （2026-11-16、実機で確認済み、未 commit）。
- NuttX 移植との違い: 今回は確認済みの Zephyr 版・NuttX 版のソースを動かす。回帰の確認が要る。

## 層（2026-11-16 の整理。この計画の軸）

| 層 | 内容 | この計画での扱い |
|---|---|---|
| A | OS にもボードにも依存しない純粋なロジック（`imu_raw_*`、`imu_tick_clock`、`time_offset`、`lsm6dso_fifo`・`_cal`・`_zero`・`_fifo_decoder`、`log_ring`、I2C のコマンド語など） | `common/` へ移す（P1） |
| B | 狭い境界だけに依存するドライバー（NINA: `nina_spi.c`・`nina_sock.c`・`nina_dns.c`、境界は `nina_os.h`・`nano_nina.h`） | `common/` へ移し、境界を OS ごとに実装（P2、P3、P4） |
| C | Zephyr の API に依存するコード（`lsm6dso.c`・`lsm6dso_stream.c`・`nina_net.c`、アプリの `imu_stream.c`・MQTT・時刻同期・Wi-Fi、`sensor_shell.c`） | 1 本にして `common/` へ移す。Zephyr では Zephyr 本体、ほかの OS では互換層の上で動かす（P1〜P3） |
| D1 | RP2040 のレジスターを直接操作するコア（I2C0 と DMA） | NuttX 版から切り出して共有（P2） |
| D2 | OS ごとに書くもの（互換層のカーネル部分、起動、割り込みの登録、OS のドライバーモデルへの接続、コンソール） | OS ごとのディレクトリーに残す |

**LSM6DSOX を C 層から B 層へ移す（sensor API と RTIO への依存を外す）作業は、この計画に入れない**（ユーザーが承認、2026-11-17）。
FreeRTOS 版は互換層の上で今のドライバーを動かす。理由は、互換層の RTIO・work queue の実装はすでに OS に
依存しておらず FreeRTOS でそのまま使えること、ドライバーの API を作り直すと Zephyr 版・NuttX 版・
`sensor` コマンドの再確認が同時に要ること。FreeRTOS 版が動いたあとで別の計画にする。

## 決定事項

| 項目 | 決定 | 決めた人 |
|---|---|---|
| FreeRTOS 版の範囲 | 全機能同等。完了条件は NuttX 移植と同じ（`imu-host` と ImuStudio が、USB の検出を除いて無変更で使える） | ユーザー（2026-11-17） |
| commit | タスクごとに `main` へ commit（push しない）。`CLAUDE.md` と `freertos/PORTING_*.md` は含めない | ユーザー（2026-11-17） |
| subagent | `model: opus` | ユーザー（2026-11-17） |
| コーディングスタイル | Zephyr に合わせる。`freertos/` は `freertos/script/format.sh` | ユーザー（2026-11-16） |
| ツールチェーン | FreeRTOS 版は最新の GCC（15.2.Rel1）。必要なツールは `setup.sh` で入れる | ユーザー（2026-11-16） |
| ライセンス表記 | 新しく書くファイルは MIT。写し・移動したファイルは元の表記のまま | ユーザーが承認（2026-11-17） |
| フェーズの順序 | P1 → P2 → P3（FreeRTOS）→ P4（Zephyr 版のドライバーの載せ替え）。境界を 2 つの OS（NuttX、FreeRTOS）で確かめてから、Zephyr 版の確認済みのドライバーを入れ替える | ユーザーが承認（2026-11-17） |
| `common/` の構成 | 下の「ディレクトリー構成」 | ユーザーが承認（2026-11-17） |
| FreeRTOS の tick | 100 µs（`configTICK_RATE_HZ` 10000）、tick は **32 ビット**（RP2040 ポートは 64 ビットを受け付けない。T01b）。`k_uptime_*` は tick からではなく RP2040 の TIMER から作る。Zephyr 版・NuttX 版と sleep の丸めを揃える（`vTaskDelay(⌈t⌉+1)`）。周期割り込みの分だけ `cpu_active` が高く出るはずで、実測して報告し、大きすぎたら tickless の自作を相談する | ユーザーが承認（2026-11-17。32 ビットへの変更も同日） |
| FreeRTOS の優先度 | `FreeRTOS = 20 − Zephyr`（Zephyr の 0〜14 が 20〜6、コンソールはその下）。順序を保つ | 案 |
| FreeRTOS のシェル | `nuttx/apps/subcmd` を共通化して使い、今の `cli.c` を行編集にする。独自コマンドは構文・出力とも Zephyr 版と同じ。Zephyr 標準のコマンド（`wifi connect`・`scan`・`status`、`kernel reboot` など）は同じ構文で必要な分だけ作る | 案（T01 で `subcmd.c`・`zshell.c` を読んで確定） |
| KV の形式 | フラッシュ末尾 8 KB（`0xffe000`）に、**3 つの版とも Zephyr の NVS と settings の形式**で置く。Zephyr の `subsys/kvss/nvs` と settings の NVS バックエンドを `common/` に写し、FreeRTOS 版と **NuttX 版**が使う（NuttX 版は今の NuttX の NVS から切り替える。T24）。版を書き換えても KV が消えなくなる | ユーザーが承認・指示（2026-11-17） |
| KV 領域に別のデータがあるとき | 起動時に、Zephyr の NVS でも消去済みでもないデータを見つけたら、自動では消さずにエラーを表示し、消去用のコマンドで消す | ユーザーが承認（2026-11-17） |
| 速度測定（FreeRTOS） | iperf3 互換を最小限に自作する（PC 側は `iperf3`。コマンドの構文は Zephyr 版の `zperf` に合わせる） | ユーザーが承認（2026-11-17） |
| Zephyr と揃わない点（FreeRTOS） | 優先度継承は 1 段だけ（Zephyr は連鎖する）。協調スレッド（負の優先度）は高い優先度で近似する（NuttX 版と同じ）。文書に書く | ユーザーが承認（2026-11-17） |
| USB の名前とプロンプト | 製品名 `FreeRTOS USB Console`、プロンプト `freertos:~$ `。`host/firmware_profiles.toml` に足す | 案 |

## T01 の結果による修正（2026-11-17）

詳細（ファイルと行、実験の手順）は `freertos/PORTING_NOTES_T01a.md`（`common/` の参照方法、分類、基準）と
`freertos/PORTING_NOTES_T01b.md`（FreeRTOS 側）にある。**この節は下のタスク表と「進め方」より優先する。** 実装の指示を
書く前に、該当する項目を 2 つのノートで読む。

**バイナリーの比較（P1・P2）**

1. 「移動だけ」のタスクの合格条件は、**像（`zephyr.bin`、`nuttx.bin`）が基準とバイト単位で一致すること**。基準は
   `freertos/build/baseline/{zephyr,nuttx}/snapshot/`（コミット 0f14ce0）、比べるスクリプトは
   `freertos/build/baseline/compare.py`（どちらも Git 管理外。消えたら T01a のノート 6 節の手順で取り直す）。
2. NuttX の比較用のビルドは `SOURCE_DATE_EPOCH=1763337600 ./script/build.sh --clean -j 1`。既定のビルドは日時の
   文字列と `libapps.a` の中の順が変わり、同じソースでも像が一致しない（`cpu_active` を比べるときも注意）。Zephyr は
   そのままで再現する。像に `__FILE__` 由来の文字列は無い。
3. ソースを変えるタスク（T12、T13 など）は像が変わりうる。変わらないはずの変更（`PRId32`、前置きヘッダーなど）は
   1 つずつ像を比べ、変わったものだけ理由を記録する。

**`common/` の参照（P1）**

4. Zephyr: `target_sources()` と `use_local_driver_sources()` はアプリの外を指せる（実験で像が一致）。ドライバーの
   ターゲット（`drivers__sensor__st__lsm6dso`、`drivers__i2c`、`drivers__sensor`）には我々のディレクトリーの `-I` が無いので、
   ファイルを分けるたびに `target_include_directories()` に「残るディレクトリー」と `common/` の両方を足す。
   `target_sources()` の順を変えない（リンク順が変わる）。
5. NuttX: `VPATH`・`DEPPATH` は **cwd からの相対パス**（絶対パスだと `make depend` が止まる）。`-I` は絶対パスでよい。
   移した `.c` が元のディレクトリーに残るヘッダーを読むなら、元のディレクトリーを `-I` に足す（カーネル側に
   `-I $(TOPDIR)/external/lsm6dso` など）。オブジェクトは元の場所にでき、`common/` には `.c`/`.h` しか置かれない。
6. **NuttX のワークスペースを写して試すときは、絶対パスの symlink を写しの中へ向け直してから**ビルドや `distclean` を
   する（T01a で、写しの `distclean` が元のワークスペースの symlink とオブジェクトを消した。直してある）。
7. ホストテスト: 同一の 15 本と `tests/fakes` の 5 本を `common/tests` に置き、`common/tests/run.sh` を両方の
   `host_tests.sh` から呼ぶ。NuttX だけのテストは、対象のソースが `common/` へ移るタスクで一緒に移す。

**タスクの中身の修正（P1・P2・P4）**

8. T10 に足す: `imu_cal.h`（`imu_cal_codec.c` と一緒でないと NuttX でビルドできない）、`i2c_dw_rp2040_commands.[ch]`
   （両方で同一なので T22 を待たない）。
9. T11: `lsm6dso_attributes.h` を移すときに、両方の `sensor_shell.c` の `"../lsm6dso/lsm6dso_attributes.h"` を
   `"lsm6dso_attributes.h"` に直す。
10. T12 の対象は 10 本: 計画の 8 本に `imu_raw_mqtt.h`・`wifi_profile.h` を足す。同一のヘッダー `kv_store.h`・`time_sync.h` も
    ここで移す。差の無くし方はノート T01a の 4.1 の表に従う: NuttX では `common/` のファイルに前置きヘッダーを `-include`
    で付けて `<nuttx/config.h>` と NuttX のヘッダーの行を消す、互換層に `<zephyr/logging/log.h>`・`<zephyr/net/net_ip.h>`・
    `<zephyr/types.h>`・`<zephyr/sys/sys_io.h>` を足す、Kconfig の名前を Zephyr に合わせる、`PRI*` を Zephyr 版にも入れる。
    シェルの登録は互換層の `SHELL_*` マクロを NuttX でも使う形にし、`main()` 以外の入口を作れるようにする
    （`time_sync.c` は `MAINSRC` でない）。ヘルプと引数の誤りの出力が今と同じことを `subcmd` のテストで確かめる。
11. T13: NuttX 版の `lsm6dso_os.[ch]` は互換層に吸収する（`k_busy_wait`・`k_msleep`・`k_usleep`・`k_cyc_to_ns_floor64` を
    互換層に足し、テストの偽物を書き換える）。`nina_os.[ch]` は B 層の境界として残す。`lsm6dso.c` の末尾のデバイスの
    生成は、OS ごとの断片（`lsm6dso_instance.h`）を `#include` する形にして `static` を保つ。差の内訳は 4.2。
12. T20: `kernel.h`・`spinlock.h` の OS の型は OS ごとの `zephyr_os.h` に出す（今の `zephyr_os_host.h` を一般化）。
    `k_tid_t` を静的なスレッドの構造体へのポインターにするか（`wifi_profile.c` の `const`）もここで決める。
13. T23: `subcmd.c` は stdio と `fnmatch` だけなので共通に置ける。`zshell.c` は、照合・ヘルプの部分（共通）と、端末を
    探すバックエンド（NuttX のタスク・`/dev`・pthread・`F_GETPATH`）に分ける。`shell.h` の `pid_t cmd_pid` を OS の型にする。
14. `kv_name.[ch]`（A 層、NuttX だけ）は T24 で `common/` へ移す。
15. Z01 に `nina_dns.c` を足す（T21 で共通にした `nina_dns.c` に Zephyr 版も載せ替える。写しを残さない）。

**FreeRTOS 版の修正（P3）**

16. F01: tick は 32 ビット・10 kHz（決定事項）。`portTICK_PERIOD_MS` は 0 になるので使わない。Pico SDK の `sleep_ms()` は
    最後の 150 µs を空回りで待つので使わない。同じ優先度の時分割は Zephyr 版に合わせて 20 ms
    （`configUSE_TIME_SLICING` 0 と tick フック）。`cpu_active` はアイドルフックで NuttX の `up_idle()` と同じ形
    （割り込み禁止、窓、`wfi`、窓、許可）。`configMAX_TASK_NAME_LEN` 32、`configTASK_NOTIFICATION_ARRAY_ENTRIES` 2、
    `Tmr Svc` の優先度を表で決める（SDK のロックの起床がこのタスクを通る）。割り込み用スタックは `PICO_STACK_SIZE`
    （0x800）で、使用量の見方を決める。異常停止（`panic()`、HardFault、アサート、スタックのあふれ）は共通の出口で
    RAM ログ（`__uninitialized_ram`）と watchdog の scratch に記録して再起動する（今は止まったまま）。
17. F03: `fnmatch` は libc に実装が無いので Zephyr の `c_lib_ext/fnmatch.c`（BSD-3）を写す。`getopt` は USB と Telnet の
    シェルが同時に使うので再入可能にする。
18. F04: `flash_safe_execute()` はこの構成では必ず失敗する。`PICO_FLASH_ASSUME_CORE1_SAFE=1` で割り込みを止めるだけにする。
    末尾 8 KB の除外は `pico_add_linker_script_override_path` で。Zephyr の HAL の `flash_write_partial()` は上流の SDK に
    無いので、ページの残りを 0xff で埋めて書く。消去中は割り込みが 34〜40 ms 止まる（NuttX 版の実測）ので、計測中の
    書き込みを Zephyr 版・NuttX 版がどう扱っているかを確かめて合わせる。
19. F09: newlib にはソケット・poll・netdb・uio・`htons`・`inet_pton`・`clock_gettime` が無い。型と定数のヘッダー、fd の表、
    非ブロッキング、`SO_RCVTIMEO`、`getsockname` を FreeRTOS の互換層に作る。errno の値は newlib と Zephyr 版で同じ
    （85 個）。`PICO_ERROR_*` は errno に直してから出す。
20. チップ ID: MQTT のトピックは KV の `imu.device`（既定 `rp2040`）から作る。チップ ID はセッション ID の先頭 8 バイト
    だけ。`pico_get_unique_board_id()` は Zephyr 版の `hwinfo_get_device_id()` と同じフラッシュの ID を読む。
21. F15: `script/kv_transfer.py` の `detect_firmware` に FreeRTOS 版を足す（T24 で役割が変わる点も合わせる）。
22. リスクの修正: Pico SDK の USB の処理は「約 1 ms ごと」ではなく、USB の割り込みのたびに、すべてのタスクより先に
    走る。送信バッファーは 64 B で、長い出力の間はミューテックスを持ったまま待ち、500 ms 空かなければ残りを捨てる。

## ディレクトリー構成

```
common/
  imu/         A・C: imu_raw_*、imu_tick_clock、time_offset、imu_cal*、imu_stream.c、imu_raw_mqtt.c、time_sync.c、
               wifi_profile.c、mqtt_min*（Zephyr は自分の MQTT ライブラリーを使う）
  drivers/
    lsm6dso/   A・C: 今の drivers/lsm6dso の共通部分
    nina/      B・C: nina_spi.c、nina_sock.c、nina_dns.c、nina_net.c、nina_proto.h と、境界のヘッダー
    i2c_dw/    A・D1: コマンド語、定義、レジスター操作のコア
    sensor_shell/
  compat/      Zephyr の API の互換層のうち OS に依存しない部分（ヘッダー、RTIO、work queue、mem_blocks、
               sensor の既定 iodev、wifi_utils など）。Zephyr 版は使わない
  shell/       subcmd と、Zephyr のシェル API の再現（NuttX 版と FreeRTOS 版が使う）
  log/         log_ring
  tests/       ホストテスト（今は zephyr/…/tests と nuttx/tests に二重にある）
zephyr/app/usb-console/   Zephyr だけのもの（main.c、kv_store.c、log_ram.c、Kconfig、DT、P4 までは NINA と I2C）
nuttx/                    NuttX だけのもの（ボード、usrsock・netdev、互換層の NuttX 実装、apps の main）
freertos/                 FreeRTOS だけのもの（互換層の FreeRTOS 実装、起動、Pico SDK の上の境界実装、シェルの口）
```

## 進め方（1 タスクの流れ）

`nuttx/PORTING_PLAN.md` の「進め方」と同じ。要点と、今回足すもの:

1. **実装**: subagent（`model: opus`）。指示はそれだけで完結させる（対象ファイル、守る定数・順序・出力文字列、
   この計画の該当節、`CLAUDE.md` の注意）。移動・統合のタスクでは、移動前後の diff と、**Zephyr 版・NuttX 版の
   バイナリーの比較結果**を報告させる。FreeRTOS のタスクでは Zephyr 版との機能差分表を報告させる。
2. **レビュー**: メインが diff を読み、元のソースを自分で開いて並べる。subagent の報告は手がかりにとどめる。
3. **修正**: 指摘を具体的に書いて subagent に直させ、再レビュー。機能差分は無くすのが原則。残るものは理由を
   付けてユーザーに相談する。
4. **動作確認**: ビルド（3 つの版）、ホストテスト、整形の検査、節目では実機。測定条件・回数・未確認の範囲を
   区別して記録する。subagent に実機作業を任せるときは、長い待ちを背景に回させない。
5. **commit**: タスクごとに `main` へ。`git add -A` を使わない。

**バイナリーの比較（P1・P2 の回帰確認）**: 作業の前に Zephyr 版と NuttX 版をビルドして基準を取り、タスクの後の
ビルドと `.text`・`.rodata` を比べる。パスの文字列（`__FILE__`、ログのモジュール名）と配置の違いだけなら
「動作は同じ」とみなし、それ以外の差が出たら原因を説明できるまで進めない。比べ方の詳細は T01 で決める。
ソースを変えるタスク（少差分の解消など）は一致しないので、節目の実機確認で確かめる。

**再開のしかた**: `freertos/PORTING_PLAN.md`、`freertos/PORTING_STATUS.md`、`git log`、`git status` だけで再開できる状態を保つ。
`freertos/PORTING_STATUS.md` は状態が変わるたびに、次の作業へ進む前に更新する。

## タスク

### P0 準備

| ID | 内容 | 確認 |
|---|---|---|
| T00 | 今ある `freertos/` の環境、`.gitignore`、ルートの README を commit | `git status` |
| T01 | 事前調査（読むだけ。結果を `freertos/PORTING_NOTES_T01a.md`・`_T01b.md` に書き、計画を直す）: Zephyr の CMake と NuttX の Makefile から `common/` を参照する方法、バイナリーの比べ方と基準の取得、`subcmd.c`・`zshell.c` の依存、互換層のヘッダーの OS 依存箇所、FreeRTOS 側の未確認事項（10 kHz・64 ビットの tick、アイドルの計測の入れ方、Pico SDK のフラッシュ書き込みと割り込み、再起動で消えない RAM の置き場、newlib のソケット系ヘッダーと `fnmatch`、Zephyr の NVS・settings を写せるか、Telnet と iperf の元にするコード） | 計画との違いを報告。大きな変更はユーザーに相談 |

### P1 `common/` への切り出し（Zephyr 版と NuttX 版）

| ID | 内容 | 確認 |
|---|---|---|
| T10 | 構成を作り、A 層（完全に同一の純粋ロジック）とホストテストを `common/` へ移す。両方のビルドとホストテストが `common/` を参照する | ホストテスト 2 つ、バイナリーの比較 |
| T11 | 完全に同一の C 層のファイル（`lsm6dso_stream.c`、`lsm6dso_fifo_irq.c`、`lsm6dso_fifo_hw.c`、ヘッダー類）を移す | 同上 |
| T12 | 少差分のファイルを 1 本にする（`nina_net.c`、`imu_stream.c`、`imu_raw_mqtt.c`、`time_sync.c`、`imu_cal.c`、`wifi_profile.c`、`sensor_shell.c`、`lsm6dso_compat.c`）。差（先頭の `<nuttx/config.h>`、シェルの登録、`PRId32` など）は互換層か OS ごとの小さなファイルへ出す | ホストテスト、ビルド、差の説明。Zephyr 版のソースが変わる |
| T13 | `lsm6dso.c`・`lsm6dso.h` を 1 本にする（デバイスの生成とログを OS ごとのファイルへ）。NuttX 版の `lsm6dso_os.*` は互換層に吸収するか残すかを T01 で決める | 同上 |
| **CP-A** | **省略**（ユーザーの決定、2026-11-17）: P1 の後も Zephyr の像が作業前の基準とバイト単位で同じだったため。P2 以降で Zephyr の像が変わったら、その時点で実機確認を相談する（遅くとも CP-Z） | – |

### P2 ドライバーの境界（NuttX 版で形を作る）

| ID | 内容 | 確認 |
|---|---|---|
| T20 | 互換層を「OS に依存しない実装」（`common/compat`）と「NuttX の実装」（`nuttx/drivers/compat`）に分ける。ヘッダーの中の型（`struct k_mutex` など）は OS ごとのヘッダーに出す（ホストテスト用の `zephyr_os_host.h` と同じ仕組み） | ホストテスト、バイナリーの比較 |
| T21 | NINA のコア（`nina_spi.c`、`nina_sock.c`、`nina_dns.c`、境界のヘッダー）を `common/drivers/nina` へ。NuttX には usrsock・poller・netdev と境界の実装だけを残す | 同上 |
| T22 | I2C0 と DMA: `nano_i2c.c`・`nano_dma.c` を、レジスター操作のコア（`common/drivers/i2c_dw`）と NuttX の接着（`i2c_master_s`、割り込みの登録、完了待ち）に分ける | ホストテスト、ビルド。コードが動くので CP-B で確認 |
| T23 | `subcmd` とシェル API の再現、`log_ring`、時計・CPU 負荷・チップ ID の境界のヘッダーを `common/` へ | ホストテスト、バイナリーの比較 |
| T24 | KV を Zephyr の NVS の形式にそろえる: Zephyr の NVS と settings の NVS バックエンドを `common/` に写し（FreeRTOS 版の F04 と共用）、NuttX 版の KV（`kv_store.c`、`settings_nvs.c`、`nano_flash_kv.c` の上の NuttX の NVS）をこれに切り替える。別のデータがあるときの扱いは決定事項のとおり。`script/kv_transfer.py` と README の「移し替え」の説明を直す | ホストテスト（Zephyr 版が作るのと同じバイト列になること）、CP-B |
| **CP-B** | NuttX 版を実機で回帰確認（`imu-host check`、`i2c_dma stats`、Telnet、Wi-Fi の再接続を各 1 回）。**CP-A で Zephyr 版が書いた KV（校正値、`imu.port`、Wi-Fi の資格情報）を、NuttX 版がそのまま読めること**、NuttX 版が書いた KV を Zephyr 版が読めること（CP-Z で確認） | ユーザーの操作が要る（下） |

### P3 FreeRTOS 版（全機能同等）

| ID | 内容 | 対応する NuttX 移植のタスク | 確認 |
|---|---|---|---|
| F01 | tick 100 µs、優先度とスタックの表、時計、CPU 負荷（アイドルの計測）、異常停止の報告 | T03b、T05 | 実機: sleep の実時間、アイドル時の `cpu_active` |
| F02 | 互換層の FreeRTOS 実装（スレッド、ミューテックス、セマフォ、`k_poll`、タイムアウト、時計、`atomic`、`k_spinlock`）。ゼロ初期化のミューテックス、割り込みからの呼び出し、静的なスレッドの起動を Zephyr の前提どおりにする | 互換層 | 実機の自己診断かホストテスト（T01 で決める） |
| F03 | シェル（`subcmd`、シェル API、行編集）、RAM ログと `dmesg` | T03、T05 | **CP-F1** |
| F04 | KV と `kv` コマンド（T24 で `common/` に置いた Zephyr の NVS と settings を使う。フラッシュの書き込みは `PICO_FLASH_ASSUME_CORE1_SAFE=1`、末尾 8 KB をリンカースクリプトで除外） | T06 | CP-F1。Zephyr 版・NuttX 版が書いた KV を読めること |
| F05 | NINA の境界の実装（`hardware_spi`、`hardware_gpio`）と `nina version` | T07 | CP-F1 |
| F06 | I2C0 コアの FreeRTOS の接着と `i2c_dma` コマンド | T08、T09 | CP-F2 |
| F07 | LSM6DSOX（デバイス・I2C・GPIO の互換層の実装、デバイスの生成、FIFO ストリーム） | T10、T11 | CP-F2 |
| F08 | 取得サービス（`imu_stream.c`、送信はスタブ）と `imu` コマンド | T12 | **CP-F2** |
| F09 | ソケットの接着（`zsock_*` から `nina_sock.c` を直接呼ぶ）、名前解決 | T14、T15 | ビルド、CP-F3 |
| F10 | Wi-Fi（`net_if`・`net_mgmt`・`wifi_mgmt` の互換層、`nina_net.c`、`wifi` コマンド、資格情報と自動再接続） | T16、T17 | **CP-F3** |
| F11 | Telnet（1 接続、2 本目を断って 1 本目を保つ）と速度測定（iperf3 互換の最小限の自作） | T18 | CP-F3 |
| F12 | 時刻同期と `ntp` | T19 | CP-F4 |
| F13 | MQTT と `imu_raw_mqtt.c` | T20、T21 | **CP-F4** |
| F14 | 校正と `imu cal`、`sensor` コマンド | T22、T23 | CP-F5 |
| F15 | ホストの差分ファイル（`firmware_profiles.toml`）と文書（README・設計書・ドライバー説明、各日英） | T24 | **CP-F5** |

節目の内容は `nuttx/PORTING_PLAN.md` の CP1〜CP6 に対応させる（CP-F1 = CP1+CP2、CP-F2 = CP3、CP-F3 = CP4、
CP-F4 = CP5、CP-F5 = CP6）。各タスクで守る忠実性の要点も同じ表を使う。

### P4 Zephyr 版のドライバーの載せ替え

| ID | 内容 | 確認 |
|---|---|---|
| Z01 | NINA: `nina_wifi.c` を共通の `nina_spi.c` と Zephyr の境界の実装に、`nina_socket.c` を共通の `nina_sock.c` とソケットのオフロードの接着に置き換える | ホストテスト、**CP-Z** |
| Z02 | I2C0: `i2c_dw.c` を共通のコアと Zephyr のドライバーの接着に置き換える。着手の前に、得られるものと危険をユーザーに報告して相談する | CP-Z |
| **CP-Z** | Zephyr 版を実機で確認（CP-A と同じ内容に、切断復帰・輻輳・Telnet・Wi-Fi スキャンを足す） | ユーザーの操作が要る |

## 実機確認でユーザーにお願いすること

- **Zephyr 版・NuttX 版へ書き戻す節目（CP-A、CP-B、CP-Z）**: 手動の BOOTSEL、KV の復元（`script/kv_transfer.py restore`、
  Wi-Fi のパスワード入力）、AP の起動（`script/wifi_ap.sh`、sudo）。回数を減らすため、CP-A と CP-B は続けて行う。
- **FreeRTOS 版の節目**: BOOTSEL は `bootsel` コマンドで入れるので操作は要らない。Wi-Fi を使う節目（CP-F3 以降）では
  AP の起動と、最初の 1 回の資格情報の登録（パスワード入力）をお願いする。
- ユーザーが端末（picocom）でコンソールを開いている間は、こちらから読み書きしない。

## リスク

- 移動だけのつもりのタスクでバイナリーが変わる（インクルードの解決順、`__FILE__`、リンク順）。比較で見つけ、
  説明できない差は直す。リンク順だけでも `cpu_active` は 0.6〜0.7 ポイント動く。
- Zephyr のビルドは、ローカルのドライバーで Zephyr 本体のソースを差し替えている（`use_local_driver_sources()`）。
  `common/` を参照する形にしたときに同じ差し替えが効くかは T01 で確かめる。
- FreeRTOS の 100 µs の周期 tick の負荷と、Pico SDK の USB の処理（約 1 ms ごと）が 6667 Hz の取得に与える影響は
  未確認。足りなければ tickless の自作か、tick を粗くして sleep の意味を見直すかをユーザーに相談する。
- FreeRTOS にはソケット層・ネットワークデバイス・KV・Telnet・iperf が無く、作る量が NuttX 移植より多い部分がある。
- Zephyr 版の NINA と I2C の載せ替え（P4）は、確認済みのドライバーの中身を入れ替える。CP-Z で問題が出たら、
  そのタスクの commit を戻せるようにタスクを分ける。

## 完了の確認

1. `common/`・Zephyr 版・NuttX 版・FreeRTOS 版のホストテストが通り、3 つの版がビルドできる。
2. Zephyr 版と NuttX 版が、共通化の後も実機で `CLAUDE.md` の確認方法を通る（CP-A、CP-B、CP-Z）。
3. FreeRTOS 版が CP-F4 と CP-F5 を通り、結果を測定条件・回数・未確認の範囲つきで報告する。
4. 同じソースが 2 か所以上に写しとして残っていない（残すものは理由を文書に書く）。
5. README・設計書・ドライバー説明（各日英）が、`common/` の構成と FreeRTOS 版を説明している。
