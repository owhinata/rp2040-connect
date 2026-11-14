# NuttX 移植の進捗

計画は `nuttx/PORTING_PLAN.md`。このファイルと計画は未追跡のまま置く（commit しない）。
状態が変わるたびに、次の作業へ進む前にここを更新する。再開時は、この 2 ファイルと `git log`・
`git status` を読み、実機の状態を確認してから続ける。

## 進め方の要約

1 タスクごとに、実装（subagent、`model: opus`）→ メインが Zephyr 版と並べてレビュー → 修正（opus）→
動作確認（opus）→ `main` へ commit（push しない）。subagent には Zephyr 版との機能差分表を報告させ、
メインは Zephyr 版のソースを自分で開いて差分を確かめる。機能差分は無くすのが原則で、残るものは
ユーザーに相談する。commit では `CLAUDE.md` と `nuttx/PORTING_*.md` を含めない（`git add -A` を使わない）。

## 確定している前提

- NuttX は 13.1.0。タグ `nuttx-13.1.0` が apache/nuttx と apache/nuttx-apps の両方にあることを
  2026-11-13 に `git ls-remote` で確認した。
- ST のレジスタドライバーは Zephyr の hal_st と同じ版に固定する。hal_st の revision は
  `ce795a9b0de623022c17d2ff394587c024aec850`（`zephyr/zephyr/west.yml`）。この中の stmemsc の版を T01 で調べる。
- PC 側に `iperf`（iperf2）は導入済み（ユーザーが実施）。
- tick は tickless を自作（T03b）。CPU 負荷はアイドル窓の実測を自作（T05）。

## タスクの状態

| ID | 状態 | commit | メモ |
|---|---|---|---|
| T01 環境構築 | 完了 | 80302e7 | 記録は下 |
| T01b 13.1.0 での再確認 | 完了 | – | 結果は `PORTING_NOTES.md`、計画への反映は `PORTING_PLAN.md` の「13.1.0 での再確認による修正」 |
| T02 ボード定義とビルド | 完了（実機未確認） | 4ac1e74 | 記録は下。実機での起動は CP1 で見る |
| T03 USB コンソール、nsh、bootsel、flash.sh | 完了（CP1 通過。手動操作の項目は未確認） | fe732c2 | 記録は下 |
| T03b tickless | 完了 | 6d0c702 | 記録は下 |
| T03c tick 換算の高速化 | 完了 | 351c7fc | 記録は下 |
| T04 共通ソースとホストテスト | 完了 | 67d6845 | 記録は下 |
| T25 KV の移行スクリプト | 実装済み・実機未確認 | c6cbecc | backup は CP1 の前に実行。restore は CP2（KV）と CP4（Wi-Fi）で確認 |
| T05 時計、ログ、dmesg、CPU 負荷、優先度 | 完了（電源断での RAM ログ消去は未確認） | a784cb7 | 記録は下 |
| T06 KV | 完了（電源断の耐性は未確認） | dc2058d | 記録は下 |
| T07 NINA SPI と nina version | 完了 | 96b8f5f | 記録は下 |
| T08 I2C（DMA なし） | 完了（CP2 通過） | 0b7c3f7 | 記録は下 |
| T09 I2C DMA と i2c_dma | 完了 | 6af1618 | 記録は下 |
| T10 LSM6DSO 本体 | 完了 | 8ffc396 | 記録は下 |
| T11 RTIO 互換層と FIFO ストリーム | 完了 | cd21b3f | メインがレビュー・再確認済み |
| T12 取得サービス（送信スタブ） | 完了 | ec6b9f8 | メインがレビュー・再確認済み |
| T13 CP3 の実機確認 | 完了（CP3 通過、メインが実施） | – | T12 の記録を参照 |
| T14 usrsock 制御系 | 完了（"Add the NINA socket control path"） | – | |
| T15 usrsock データ系 | 完了（"Add NINA socket data transfer"） | – | |
| T16 Wi-Fi 管理 | 完了（"Add NINA Wi-Fi management, wlan0 and irqstack"。小作業 A・B も同じ commit） | – | |
| T17 資格情報、自動再接続、wifi cred | 完了（"Add Wi-Fi credentials"）。本物の AP への接続も確認済み（下の「CP4 前半」） | – | |
| T18 Telnet と iperf | 完了 | `0edec80` | CP4 通過（ImuStudio の GUI は未確認） |
| T19 時刻同期と ntp status | 完了 | `c68cd71` | 実機で同期・切断復帰・30 分 1 回 |
| T20 MQTT 最小クライアント | 完了 | "Add the copied part of Zephyr's MQTT library" | |
| T21 imu_raw_mqtt（T21b 含む） | 完了 | `7d7e311`、`c34af5b` | CP5 通過。CPU は Zephyr の記録より約 +3.5 ポイント（CP6 で比較） |
| T22 校正と imu cal | 完了 | `9bce713` | 実機で確認 |
| T23 sensor コマンド（sensor_shell.c の移植） | 完了 | `76bb4d3` | 実機で確認。高い ODR の stream は CP6 で Zephyr と比較 |
| T24 ホストの差分ファイルと文書 | 完了 | `b75336c` | CP6 の NuttX 側は通過。Zephyr 版との実機比較はユーザーに相談中 |

## ユーザーが決めたこと（計画の後で追加）

- セッションの通し番号（`imu.session`）は実行時の状態なので、NuttX 版へ書き戻さない。NuttX 版では初期値から
  始め、`imu start` のたびに +1 して KV に保存する（Zephyr 版の動作どおり）。`kv_transfer.py` は対応済み
  （2026-11-13）。カウンターなどの実行時の状態は初期値からでよい。

- 2026-11-14: エラー番号（errno）は Zephyr の値へ変換しない。NuttX の番号のまま表示する。

- 2026-11-14: **`bootsel` コマンドは完成時に削除する**（開発中は使う。削除後の書き込みは手動 BOOTSEL。`flash.sh`・README・`CLAUDE.md` の記述も合わせる）。
- 2026-11-14: **`PORTING_*.md` は当面残す**（作業ログとして残すかもしれない。消さない、未追跡のまま）。
- 2026-11-14: **NuttX 版だけで測った実測値は README・ドライバー説明に載せない**（起床の遅れ、CPU 時間、flash 消去の割り込み禁止時間、NINA の所要時間、I2C の実効速度・DMA の所要時間、スタック使用量など）。載せるのは Zephyr 版と比べた結果だけ。既存の記載は T14 のレビュー後にメインが削除する（subagent への指示にも入れる）。I2C の実効速度は直さない。
- 2026-11-14: 割り込み用スタックの使用量の表示は、修正量が僅かなら追加する（そうでなければ不要）。→ T14 のレビュー後にメインが見積もって対応。

- 2026-11-14: **異常停止時は自動で再起動する（`BOARD_RESET_ON_ASSERT`）を完成版にも入れる。** 試験版だけでなく最終の defconfig に残す。
  **移植が終わったら Zephyr 版にも反映する**（`CONFIG_RESET_ON_FATAL_ERROR=y` 相当。`zephyr/` を変更してよいのはこの件で、移植完了後。README・設計書も合わせる）。
  → 完了条件「Zephyr 版のソースに差分がない」はこの反映の前の時点で確認する。
- 2026-11-14: 試験コマンドが戻らなくなる型のハング（システムは動いている）は、WDT ではなく**試験コマンドに制限時間を持たせる**などで対処する。

## Zephyr 版との機能差分（残っているもの）

ボード定義（T02、根拠は下の T02 の記録）:

| 項目 | Zephyr 版 | NuttX 版 | 扱い |
|---|---|---|---|
| PLL_USB の VCO | 1200 MHz（fb 100、/5/5） | 480 MHz（fb 40、/5/2、`rp2040_clock.c:246` に直書き） | 出力はどちらも 48 MHz。**合わせない**（2026-11-13 ユーザー決定: 48 MHz で機能は変わらない）。文書に差分として残す |
| ベクター表 | flash | SRAM（`ARCH_RAMVECTORS`、pico の構成どおり） | 機能は同じ。割り込み応答の時間が違い得る |
| クロック初期化時のリセット | SYSCFG・USBCTRL をリセットしない | リセットする | 影響なしと推測 |
| ROSC | 分周などを設定し直す | 触らない | アプリは ROSC を使わない |
| GPIO4/6/7 | SPI0 の機能に割り当て（ボード DT の `&spi0` が okay） | リセット時のまま | アプリは SPI0 を使わない |
| GPIO26〜29 の入力有効 | 触らない | common が IE を切る | 未使用ピン |
| tick | 100 µs の tickless（SysTick、CPU クロック由来。送信中は約 480 ppm 遅れる） | 100 µs の tickless（TIMER、水晶の 1 MHz） | 分解能と丸めの規則（切り上げ +1 tick）は同じ。源は NuttX のほうが正確で、合わせない |
| 起床の遅れ | 未測定 | T03c 後: `usleep` の復帰が tick 境界から 17〜36 µs（RR=20、配置でばらつく）、`usleep(1000)` 1 回の CPU 時間 30〜52 µs | Zephyr 版の値は未測定。必要なら Zephyr 版に書き戻して測る |
| RR の wdog | 待ちに入ると時分割を止める | `g_sched_event` の wdog が待ち中も残り、残り時間より長く眠ると割り込みが 1 回余分に入る（NuttX 13.1.0 の動作） | T05 の CPU 負荷の比較で差になり得る。合わせていない |
| ログの整形 | deferred（記録だけして低優先度スレッドが整形、あふれたら捨てる） | 呼び出し元で同期整形（約 42 µs、スタック約 500 B）、捨てない | ログはまれなのでこのまま。スタックが足りなければ見直す |
| errno の番号 | picolibc（ENOTSUP 134、ECANCELED 140、EOVERFLOW 139、ETIMEDOUT 116、ENOSYS 88、EALREADY 120、EMSGSIZE 122） | NuttX（138、125、75、110、38、114、90） | 表示に出るエラー番号が変わる。ホストツールと文書は番号に依存していない（grep で確認）。**変換しない**（2026-11-14 ユーザー決定）。NuttX の番号のまま。T24 で文書に対応表を載せる |
| 実行時のログレベル変更 | `log` コマンド | なし | 未実装 |
| シェルの優先度 | 14 | 104（118−14） | 規則どおり。上の「シェル優先度 100」の行は古い |
| 時分割の数え方 | 切り替えのたびに 20 ms を与え直す | 実行した分を差し引いて残りを持ち越す（幅は同じ 20 ms） | 残す。同じ優先度のスレッドの順番が変わり得る |
| 時分割の対象 | 優先度 0 以上の preemptible スレッド | 全タスク（SCHED_RR） | 移植するスレッドはどれも対象なので同じ |

コンソール（T03）:

| 項目 | Zephyr 版 | NuttX 版 | 扱い |
|---|---|---|---|
| 再列挙後に最初に開いたとき | 構成後の最初のデータを、ホストが読み始めてから 100 ms 遅らせて送る（`usbd_cdc_acm.c`、`USBD_CDC_ACM_TX_DELAY_MS`）。古いプロンプトが遅れて届く | ホストがポートを閉じている間（DTR なし）の出力は捨てる。開いても何も届かず、最初の入力（Enter や Ctrl-C）でプロンプトが出る | ホストツールは開いた直後に Ctrl-C を送るので同じに見える。端末ソフトでは Enter を 1 回押すまでプロンプトが見えない |
| 未知のコマンドのエラー文 | `foo: command not found` | `nsh: foo: command not found` | nsh のまま |
| Tab 補完 | サブコマンドまで | コマンド名だけ | nsh のまま |
| 履歴 | 全体で 512 B | 8 行×513 B | nsh のまま |
| USB のデバイスクラス | EF/02/01＋IAD | 02/00/00、IAD なし | 標準ドライバーで動く。合わせない |
| 0x80 以上の入力バイト | 捨てる（`SHELL_ASCII_FILTER`） | 行に入れる | 未対応 |
| シェルの優先度 | 14（換算 104） | 100 | 全アプリスレッドより下である点は同じ |
| 終わらないコマンド | 無い | `cat /dev/zero`、長い `sleep` などは Ctrl-C で戻らない（USB の再列挙で戻る） | 未対策 |
| `reboot` | `kernel reboot` | `reboot` | どちらも引数を見ずに再起動する |

FIFO ストリームと互換層（T11、詳細は T11 の記録）:

| 項目 | Zephyr 版 | NuttX 版 | 扱い |
|---|---|---|---|
| sysworkq | 協調（-1） | 119 の通常スレッド | 119 より上のスレッドが無いので同じ順 |
| センサー時計（`sensor_clock_get_cycles`） | カーネルの cycle（負荷で遅れる） | TIMER の 1 µs | 残す |
| GPIO の割り込み優先度 | 3 | 全割り込み同一 | NuttX の制約 |
| `ps` のスレッド名 | `RTIO WQ 0` | `RTIO_WQ_0` | NuttX が空白を置換 |
| スタックの実寸 | 1024／2048 | 976／2008（TLS などの分だけ小さい） | 使用量を CP3 で確認 |

## ユーザーに確認待ち

- （`bootsel`・`PORTING_*.md` は 2026-11-14 に決定済み。上の「ユーザーが決めたこと」）

## 実機の状態

- 2026-11-13 22:41: ユーザーが手動で BOOTSEL に入れ、**NuttX 版（T03、未 commit の作業ツリー）を書き込んだ**。
  `/dev/serial/by-id/usb-NuttX_NuttX_USB_Console_503533748A8C9218-if00`（0525:a4a7）。シリアル文字列は Zephyr 版と
  同じチップ ID。以後の書き込みは `nuttx/script/flash.sh`（`bootsel` 経由）。Zephyr 版の KV は flash 末尾に
  残っている（NuttX 版はまだ KV 領域に触れない）。AP（`rp2040-test`、10.42.0.1）はユーザーが起動したもので止めない。
- KV は `host/recordings/kv-backup.json` に退避済み（`imu.port`=18830、`imu.session`=0x4f、
  `imu.cal.accel.244`、`imu.cal.gyro.70000`。他は未設定）。Zephyr 版でさらに `imu start` したら取り直す。
- **Zephyr 版の基準の記録**（Git 管理外、`host/recordings/`）:
  - `zephyr-reference-outputs.txt`: `imu`・`imu status`（idle）・`imu cal`・`ntp status`・`kv`・`kv get`（値あり／
    無し／`sys.`）・`i2c_dma stats`・`kernel ps`・`kernel thread stacks`・`wifi status`・`dmesg`・`help` の実際の出力。
    出力文字列を合わせるときの基準。
  - `zephyr-baseline-shell.txt`・`zephyr-baseline-check.txt`: 6667 Hz、約 61.5 秒、1 回の計測。
    `cpu_active` 63.2〜64.2%、欠番 0、`unix_regressions` 0、`max_transfer_us` 26192、`publish_max_us` 116644、
    `queue_high` 4、`tick_read_us` 105〜150、NTP の往復 最小 4927 µs・使用 5178 µs、`sync_delay_ms` p50 5.25、
    `i2c_dma` transfers 6454・errors 0。スタック: imu_stream 984、imu_raw_mqtt 1260、imu_fifo_worker 880、
    shell_uart 1640（各 2048／3072 中）。CP5 で NuttX 版と比べる。

- **2026-11-14（T11 後）**: NuttX 13.1.0、T11 の最終ビルド（`Nov 14 2026 05:11:39`、commit cd21b3f と同じソース）。KV は `imu.port`・校正 2 つ
  （Zephyr 版の KV と Wi-Fi 資格情報は T06 で消去済み、退避あり）。センサーは ODR 停止。AP は触っていない。

## タスクごとの記録

（タスクが進むたびに、レビューの指摘と結果、確認の条件・回数・結果を追記する）

### T01 環境構築（完了、80302e7）

- 取得物: nuttx `nuttx-13.1.0`（2b5c0de）、nuttx-apps `nuttx-13.1.0`（3c772ea）、pico-sdk 2.2.0（NuttX の文書と
  CI が使う版）、Arm GNU Toolchain 13.2.Rel1（CI と同じ、gcc 13.2.1）、kconfiglib 14.1.0、ST の
  `lsm6dso_reg.[ch]`（lsm6dso-pid v3.2.2 = stmemsc v2.13.2。LF に直すと Zephyr の hal_st と完全一致）。
  `nuttx/` 全体で約 1.8 GB。
- `kconfig-tweak` は 13.1.0 でも必要（`tools/sethost.sh` が呼ぶ）。`nuttx/script/kconfig-tweak` で代用。
- symlink `nuttx/nuttx/external -> ../drivers`、`nuttx/nuttx-apps/external -> ../apps` は張ってある
  （リンク先はまだ無い。NuttX は `external/Kconfig` が読めるときだけ使う）。
- 確認（subagent、各 1 回）: 新しいシェルで 2 回 source して通る。`raspberrypi-pico:nsh` を
  `CONFIG_RP2040_UF2_BINARY` を切ってビルドでき、`.boot2` が 0x10000000 に 0x100 B。flash 種別を
  `at25sf128a` にすると `boot2_at25sf128a.S` で作り直される。既定の UF2 生成（picotool）は試していない。
- メインのレビュー: 指摘なし。非対話シェルで環境が再現することを確認。
- 後のタスクへの注意: `.gitignore` に `/nuttx/apps/Kconfig`（自動生成）を入れてある。T03 で手書きの最上位
  Kconfig を置くなら外す。非対話シェルでは `bash -c 'source nuttx/script/setup.sh && ...'` で使う。
- ST の LICENSE ファイルは取得していない（Git 管理外で再配布しないため）。

### T04 共通ソースとホストテスト（完了、67d6845）

- 写し（無変更、37 ファイル）: `nuttx/apps/imu/`（`imu_raw_*`、`imu_tick_clock`、`time_offset`、`imu_cal_codec.c`、
  `imu_cal.h`）、`nuttx/drivers/lsm6dso/`（`lsm6dso_{fifo,cal,zero,fifo_decoder}`、`lsm6dso_fifo_packet.h`）、
  `nuttx/tests/`（11 本と `fakes/zephyr/drivers/sensor.h`）。
- `nuttx/script/host_tests.sh`: 11 本すべて ok。テストを足すときは `run 名前 追加ソース...` の 1 行。
- メインのレビュー: 全ファイルを `cmp` で Zephyr 側と照合して一致。テストを自分で走らせて 11 本 ok。指摘なし。
- 後のタスクへの注意:
  - `imu_raw_packet.h:7` は `#include "../drivers/lsm6dso/lsm6dso_fifo.h"`。`nuttx/apps/imu/` からは
    `-I<repo>/nuttx/apps` を足すと `apps/../drivers/...` に解決する。NuttX のビルド（T12）でも同じ `-I` が要る。
  - `lsm6dso_fifo_decoder.[ch]` は Zephyr の `<zephyr/drivers/sensor.h>`（`sensor_decoder_api`、
    `sensor_three_axis_data`、`SENSOR_CHAN_*`、`SENSOR_TRIG_FIFO_WATERMARK`、`q31_t` など）に依存する。
    NuttX の本番ビルドで使うなら互換ヘッダーが要る（T10/T11/T23 で判断）。
  - `lsm6dso_fifo_stats.h` は `<zephyr/device.h>`・`<zephyr/rtio/rtio.h>` に依存するので写していない（T10/T11）。
  - まだ移していないテスト: `fifo_hw`（T10）、`stream_stop`（T11）、`i2c_dma_commands`（T09）、`nina_spi`（T07）。

### T25 KV の移行スクリプト（実装済み、c6cbecc。実機では未確認）

- `script/kv_transfer.py`（`host/.venv/bin/python script/kv_transfer.py backup|restore`）。既定の退避先は
  `host/recordings/kv-backup.json`（Git 管理外、権限 600）。
- 既定のキー: `imu.host`・`imu.port`・`imu.device`・`imu.odr`・`imu.watermark`・**`imu.session`**、校正値は全感度
  （accel 61/122/244/488、gyro 4375/8750/17500/35000/70000）。
- **訂正**: セッションの通し番号は `sys.` のキーではなく `imu.session`（`imu_stream.c:40`、4 B big-endian）で、
  シェルから読める。退避・書き戻しの対象。restore はボードの番号の方が大きければ書かない。
  → チップ ID を生の 8 B で揃えれば（T05/T12）、セッション ID は Zephyr 版から連続する。
- Wi-Fi: `wifi cred add -s <SSID> -k 1 -p <password>` と `wifi cred auto_connect`。SSID の既定は `rp2040-test`、
  パスワードは標準入力（端末なら getpass）。Zephyr 版の `wifi cred add` は**成功時に何も出力しない**ので、
  スクリプトは「出力なし＋プロンプト」を成功とみなす。**T17 の NuttX 版も成功時は何も出さないこと。**
- NuttX 版向けの引用は暫定（英数字・空白・`!%+,-./:=?@^_~` だけ通す）。T17 で nsh の引用規則を確かめて
  `quote_arg` を見直す。ファームウェアの判定は `kernel version`（Zephyr）→ `uname -a`（NuttX）。
  NuttX 版で `uname` を有効にしておく（T03）。
- 確認: subagent が偽のシェルで 49 項目（応答の解釈、上書き防止、読み返し、パスワードが出力に出ないこと
  など）。メインはコードを読んでレビュー、pyflakes 警告なし。実機（Zephyr 版の実際の応答）は未確認。

### T02 ボード定義とビルド（完了、4ac1e74。実機では未起動）

- `nuttx/boards/arduino-nano-rp2040-connect/`（`raspberrypi-pico` の写し。接頭辞 `nano_`）、`drivers/`・`apps/` の
  骨組み、`script/build.sh`、`script/bin2uf2.py`。NuttX 本体へのパッチは不要だった。
- `build.sh`: setup.sh を自分で source、`-a ../nuttx-apps`、入力（defconfig・Kconfig・アプリのディレクトリ・
  パッチ）が変わると distclean から自動で再 configure、`--clean`・`--menuconfig`・`--savedefconfig`。
  検査: `.boot2` が 0x10000000 に 256 B、イメージ終端が 0x10ffe000 以下。出力は `nuttx/build/nuttx.uf2`。
- リンカースクリプト: flash 16376K、`__kv_start`/`__kv_end`、`.noinit`（`_snoinit`/`_enoinit`、`AT > sram` が必要）。
- `board.h` にピン番号（`BOARD_I2C0_*`、`BOARD_LSM6DSOX_*`、`BOARD_NINA_*`、`BOARD_GPIO_LED_PIN`=6）と
  `BOARD_KV_OFFSET`/`SIZE` を定義済み（初期化はしていない）。LED（GPIO6、D13/SCK と共用）は駆動しない。
- **boot2 は Zephyr 版と 256 B すべて一致**（メインも `zephyr/build/usb-console/zephyr/zephyr.bin` の先頭と
  `cmp` で確認）。分周 4（flash 31.25 MHz）。XOSC の起動待ちは Zephyr に合わせて倍率 6（pico の既定は 1）。
  PLL_SYS・clk_*・watchdog tick（1 MHz）は同じ。
- 確認（subagent 各 1 回、メインもビルドを 1 回）: クリーンから通る、差分ビルド、`--clean`、アプリ追加時の
  自動再 configure、`--savedefconfig`、UF2 の読み戻し一致、boot2 の CRC 異常や上限超えで失敗すること。
  `--menuconfig` は未確認。
- メインのレビュー: 流用元との diff を読んで確認。指摘なし。
- 後のタスクへの注意: `CONFIG_RP2040_FLASH_BOOT=y` は既定値なので defconfig に残らない（build.sh が検査）。
  menuconfig の変更は `--savedefconfig` で保存しないと次の再 configure で消える。ビルドのたびに
  `lib_utsname.c` が再コンパイルされ（ビルド日時）、UF2 は毎回少し変わる。

### T03 USB コンソール、nsh、bootsel、flash.sh（確認中、未 commit）

- 実装: `apps/init/`（`nano_init_main`。`/dev/ttyACM0` を開き直しながら nsh のセッションを起動。挨拶文を出さない
  ため nshlib の内部 API `nsh_newconsole`/`nsh_session` を使う）、`apps/bootsel/`（引数があれば何もしない）、
  `boards/…/src/nano_{chipid,usbserial,reset}.c`、`script/flash.sh`。NuttX 本体へのパッチなし。
- 設定の要点: `CDCACM_CONSOLE=n`、`DEV_CONSOLE=n`、`LINE_MAX=512`（13.1.0 に `NSH_LINELEN` は無い）、
  `NSH_MAXARGUMENTS=30`（Zephyr の `SHELL_ARGC_MAX`）、`CDCACM_RXBUFSIZE=2048`、履歴 8 行、Tab 補完、
  `RAMLOG_SYSLOG`（T05 で置き換える）、nsh のスタック 3072・優先度 100。端末は `ICRNL|INLCR`、`OPOST|ONLCR`。
- `nano_chip_id()`（flash の 0x4B を生で読む 8 B）は T05/T12 が使う。
- subagent が挙げた Zephyr 版との差分（実機確認の後で扱いを決める）:
  1. **CDC の通信インターフェースのプロトコルが 1（AT コマンド）**（Zephyr は 0）。ModemManager が probe する
     おそれ。合わせるなら `drivers/usbdev/cdcacm_desc.c` の 1 行パッチ（`CDC_PROTO_ATM` → `CDC_PROTO_NONE`）。
  2. **Ctrl-C**: Zephyr は入力中の行を捨てる。nsh の readline は無視する（`board.py` は開いた直後に `\x03` を送る）。
  3. LF 単独では行が確定しない（Zephyr は CR・LF・CRLF のどれでも確定）。
  4. 未知のコマンドのエラー文が `nsh: foo: command not found`（Zephyr は `foo: command not found`）。
  5. 出力が詰まったとき、Zephyr は 50 ms で捨てる。NuttX はホストが読むまで書き込みが待つ。
  6. USB のデバイスクラス（Zephyr は EF/02/01＋IAD、NuttX は 02/00/00）。Tab 補完の範囲、履歴の大きさ、
     シェルの優先度（100。Zephyr 換算は 104）。
- **CP1 の実機確認（1 回目、2026-11-13 22:45〜、subagent）**
  - 期待どおり: USB 記述子（製造者・製品・シリアル・250 mA）、board.py からの `uname -a`/`help`/`ps`/`free`、
    `kv_transfer.py` の判定（NuttX）、CR と CRLF で 1 回だけ確定、開き直し 10/10、510 文字までの行、16 KB の
    一括送信で取りこぼしなし、読まずに閉じても固まらない、`bootsel --help`/`-h`/`x` は再起動しない、
    `reboot` 3/3 で復帰、`flash.sh` 3/3 成功（約 7.3 秒、デスクトップの自動マウント）、init と nsh が 1 つずつ、
    nsh のスタック 3032 中 最大 1768、`sleep 10` は 10.01 秒、ModemManager は AT を送ってこなかった
    （`not supported by any plugin`。ただし `ID_MM_CANDIDATE=1`、bInterfaceProtocol=1）。
  - **問題 A**: USB の再列挙直後に最初に開くと、起動時のプロンプト `nsh$ \x1b[K` がポートを開くまでデバイス側に
    残っていて、ホストの tty（再列挙で ECHO 有効に戻る）がそれをエコーし返し、ボードの入力行に入る。最初の
    コマンドが `nsh: nsh: command not found` になる（`flash.sh` 後 3/3、`reboot` 後 5 秒待つと発生。メインも再現）。
    Zephyr 版は送信が 50 ms 詰まると捨てるので起きない。
  - **問題 B**: Ctrl-C で入力途中の行が消えない（`abc` → `\x03` → `uname` で `abcuname` を実行）。接続をまたいでも
    残る。`\x05\x15` なら消える。
  - **問題 C**: 511 文字以上の行は分割され、512 文字目以降が別のコマンドとして実行される。
  - **問題 D**: 引数なしの `cat` が stdin を読み続け、シェルの操作では戻せない（Ctrl-D も Ctrl-C も効かない。
    serial は VEOF を扱わず、init が ISIG を外している）。USB の再列挙で復帰する（メインが
    `USBDEVFS_RESET` ioctl で復旧。sudo 不要: `/dev/bus/usb/BBB/DDD` を開いて `ioctl(fd, 0x5514, 0)`）。
  - 未確認（手動操作が要る）: USB の抜き差し、RESET ボタン、ホストのサスペンド。
- **修正と再確認（完了、fe732c2）**: パッチ 7 本（`nuttx/patches/nuttx/0001〜0003`、`patches/apps/0001〜0004`）。
  - A: cdcacm が DTR なしの間の出力を捨てる（DTR が落ちた瞬間に送信バッファも捨てる。DTR を上げないホストでも
    何か受信すれば出力する）。**訂正: Zephyr の `SHELL_TX_TIMEOUT_MS=50` は送信を捨てる設定ではなくミューテックスの
    待ち時間。Zephyr で古いプロンプトがエコーされないのは `usbd_cdc_acm.c` の TX 遅延（ZLP + 100 ms）のため。**
  - B: readline が Ctrl-C で行を捨てて改行し空行を返す（Zephyr `shell.c:1369-1380`）。
  - C: 行が満杯なら以後の文字を捨てる。`LINE_MAX=513` で最大 511 文字（Zephyr `data_insert`）。
  - D: serial のカノニカル読み出しで ^C（ISIG なし）は EOF、^D は VEOF。シグナルも kill も使わない。
    `NSH_DISABLE_LOOPS`・`NSH_DISABLE_WATCH`。builtin アプリ実行中の ^C はバッファに残り、次のプロンプトで行を消す。
  - E: bInterfaceProtocol を 0 に。F: CR・LF・CRLF・LFCR・CR NUL がどれも 1 回の Enter（Zephyr `process_nl`）。
    readline が自分でエコーする（端末の ECHO は読む間だけ切る）。termios から `ICRNL|INLCR` を外した。
  - `setup.sh` は当てたパッチを `.git/nano-patches/` に記録し、差し替え時は逆順に外して当て直す。
    パッチで NuttX の作業ツリーが変わるので `uname` の版は `2b5c0de5-dirty` になる。
  - 実機（subagent）: A は 36/36（4 通りの開き方 × 0.5/5/30 秒 × 3 回）と `flash.sh` 後 5/5、USB リセット後 3/3。
    B・C・D・E・F と回帰（開き直し 10/10、16 KB 一括、`reboot` 36 回以上、`flash.sh` 5 回）は正常。Ctrl-C を
    200 回繰り返しても `free` の used は 15872 のまま。メインも 1 回ずつ再確認（再起動 6 秒後の最初のコマンド、
    `abc`+^C、`cat`+^C、600 文字の行、bInterfaceProtocol=0）。nsh のスタック 3032 中 1728。
  - 未確認（手動操作が要る）: USB の抜き差し、RESET ボタン、ホストのサスペンド。
  - 後のタスクへの注意: (T17) `NSH_MAXARGUMENTS=30`。(T18) telnet ドライバーは ECHO の切り替えのたびに
    WILL/WONT ECHO を送るので、readline の ECHO 切り替えで 1 行ごとにネゴシエーションが 2 回起きる。^C で読み出しを
    終える処理は serial にだけ入っていて Telnet には効かない。(全般) nsh から起動するデーモンは fd 0/1/2 を閉じる。
    (T24) nsh は `$`・引用符・`;`・`#`・`>`・`&` を解釈する。
- 修正の方針（2026-11-13、メイン）: A〜D と、USB のプロトコル値（1 → 0、Zephyr と同じ）、LF 単独での確定を、
  Zephyr 版の振る舞いに合わせて直す。nshlib/readline へのパッチ（`nuttx/patches/apps/`）と NuttX 本体への
  パッチ（`nuttx/patches/nuttx/`）を使ってよい。未知のコマンドのエラー文・Tab 補完の範囲・履歴は nsh のまま。

### T03b tickless（完了、6d0c702）

- `boards/…/include/nano_timer.h`（TIMER のレジスタ定義。T05 の `time_sync_mono_us()` も使う）、
  `src/nano_oneshot.c`（`rp23xx_oneshot.c` の写し。ALARM0、`RP2040_TIMER_IRQ_0`。ISR の入口で INTS を確かめる処理を
  追加）、`src/nano_timerisr.c`、ボードの Kconfig `NANO_TICKLESS`（`ONESHOT`・`ONESHOT_COUNT`・`ALARM_ARCH` を select）、
  defconfig（`SCHED_TICKLESS=y`、`RR_INTERVAL=20`。`USEC_PER_TICK=100` は tickless の既定値）、
  パッチ `patches/nuttx/0004`（`ALARM_ARCH` で `rp2040_timerisr.c`、`ARCH_IDLE_CUSTOM` で `rp2040_idle.c` を外す）。
- TIMER は TIMERAWH → TIMERAWL → TIMERAWH で読む（ラッチ式の TIMELR/TIMEHR は使わない）。過去の時刻のアラームは
  INTF で即座に起こす。32 ビット一周（約 71.6 分）をまたぐ待ちは wdog が設定し直す（実機では未確認、コードの検討のみ）。
- Zephyr 版の時分割: `TIMESLICING=y`、`TIMESLICE_SIZE=20`、`TIMESLICE_PRIORITY=0`。
- `ONESHOT_FAST_DIVISION` は M0+ では逆に遅くなったので選ばない。
- 実機（subagent、TIMER 基準、各 1000 回）: `usleep(100)` 平均 233.5 µs、`usleep(1000)` 1126.0 µs、
  `usleep(10000)` 10117.0 µs、`nxsem_tickwait(10 tick)` 1099.6 µs。アイドル 10 s・60 s の TIMER 割り込みは 1 回。
  90 s で TIMER と `CLOCK_MONOTONIC`・`/proc/uptime` が一致。620 s 放置後も応答。T03 の回帰（開き直し 10/10、
  ^C、`cat`、600 文字、16 KB、`reboot` 3/3、`flash.sh` 3/3）は正常。最初の書き込みは安全網（ALARM3 で BOOTSEL に
  落とす）つきで行い、最終状態からは外した。
- **課題**: 起床の遅れが平均約 95 µs（RR=0 なら 28 µs）。`clock_systime_ticks()` 1 回に 5.6〜6.2 µs（64 ビット除算）。
  → T03c: ボード側で weak の `up_timer_gettick`・`up_alarm_tick_start`・`up_timer_gettime` を上書きし、1 MHz・100 µs に
  特化した換算にする（メインの判断。T05 の CPU 負荷の比較を公平にするため）。
- 後のタスクへの注意: `up_udelay` は TIMER を読んで待つ（1 回の読み出しに約 5 µs）。flash 書き込み（T06）の RAM 関数
  からは呼ばない。RP2040 の構成には割り込みスタックが無い（割り込みは実行中のスレッドのスタックを使う）ので、
  各スレッドのスタックに割り込みの分の余裕が要る。

### T03c tick 換算の高速化（完了、351c7fc）

- `boards/…/src/nano_timeconv.h`（100 での除算を 1 バイトずつ `(n*41944)>>22`、tick×100 はシフトの和）。
  `nano_oneshot.c` が weak の `up_timer_gettick`・`up_timer_gettime`・`up_alarm_tick_start`・`up_alarm_cancel`・
  `up_udelay`・`up_ndelay` を上書き（TIMER 直読み）。`CONFIG_USEC_PER_TICK != 100` は `#error`。NuttX 本体パッチは追加なし。
- ホストテスト `tests/nano_timeconv.c`（汎用の換算の書き写しと全域で一致）。`host_tests.sh` は 12 本。
- 実測（計測用の版、1000 回平均、RR=20）: 起床の遅れ 51〜55 → 17〜36 µs、CPU 時間/`usleep(1000)` 104〜114 → 30〜52 µs、
  `clock_systime_ticks` 5.8 → 1.75 µs、`up_udelay(1)` 10.8 → 1.7 µs。ビルドごとのばらつきは XIP キャッシュの配置（推測）。
- `ONESHOT_FAST_DIVISION` は M0+ では遅くなる。SIO の除算器は割り込みとの共有が要るので不採用。
- メインのレビュー: 汎用の `arch_alarm.c`・`oneshot.h` と並べて意味が同じことを確認（未初期化時 `-EAGAIN`、cancel 後に時刻を返す、
  absolute の開始）。ホストテスト 12 本、ビルド、実機で `uname`・`time "sleep 10"`（10.0004 s）・`ps` を再確認。
- 後続への注意: さらに縮めるならホットな経路を SRAM に置く案（未着手）。`clock_gettime` は 3.6 µs（アプリは TIMER を直読みする）。
- 実機: T03c の最終ビルド（Nov 14 2026 01:08:03）が載っている。

### T05 時計・RAM ログ・dmesg・CPU 負荷・優先度・番兵（完了、a784cb7）

- 置き場所: 共通ヘッダーは `nuttx/drivers/include/`（`app_log.h`・`log_ram.h`・`cpu_load.h`・`priorities.h`。カーネル・ボード・アプリから
  `$(TOPDIR)/external/include` で見える）。`nano_mono_us()` は `include/nano_timer.h`（`<arch/board/nano_timer.h>`）。T19 の
  `time_sync_mono_us()` はこれを呼ぶ。
- ログ: `drivers/log/log_ram.c`＋`log_ring.[ch]`（OS 非依存、ホストテスト `log_ram`）。`LOG_MODULE_REGISTER(name, level)`、
  `LOG_ERR/WRN/INF/DBG`、`*_RATELIMIT`。`<syslog.h>` の `LOG_ERR` とぶつかるので `app_log.h` が `#undef` する（syslog の値は
  `APP_LOG_SYSLOG_ERR`）。syslog チャネルとしても登録（`RAMLOG*` は外した）。行は 192 B で切る。
- CPU 負荷: `src/nano_idle.c`（`up_idle`、`cpu_load_get(bool reset)`）、`nano_cpuload.h`（ホストテスト `nano_cpuload`）。
  実測: アイドル 10 秒で 0（非アイドル 297〜392 µs/10 s）、ビジー 1000、半々 499、毎秒 5099 回の空の wdog 割り込みで 50
  （番兵の割り込み出口の確認を入れた後。入れる前は 45）。割り込み 1 回の入口・出口は約 8〜10 µs。
- 優先度: `priorities.h`（`NANO_PRIO_*`、`NANO_STACK_*`）。**nsh・init・bootsel・dmesg は 104**（118−14。計画の 100 から変更）。
  init は 118 で起動して Zephyr の `main()` 相当の初期化をし、104 に下げてコンソールのループに入る。後続タスク（KV、校正、
  時刻同期、Wi-Fi の初期化）は 118 の区間に足す。`PRIORITY_INHERITANCE=y`。
- 番兵: `src/nano_stackcheck.c`。`SCHED_INSTRUMENTATION(_SWITCH,_IRQHANDLER)` のフックで、切り替え時（`sched_note_suspend`）と
  割り込みハンドラーの出口（`sched_note_irqhandler`、`g_running_tasks[0]`）に最下位ワードを調べ、PANIC。1 割り込みあたり約 1.0 µs。
  レビューで「切り替えのときしか調べていない」ことを指摘して直させた（周辺の割り込みは `exception_direct` 経由で `arm_doirq` を通らない）。
- 残る差分（下の表にも）: ログは同期整形（1 回約 42 µs、スタック約 500 B。Zephyr は deferred）、実行時のレベル変更（`log` コマンド）なし、
  cpu_load の時間源（TIMER 1 µs）、BOOTSEL 経由の書き込みで RAM ログが空になる（Zephyr 版は未確認）、協調優先度は再現できない。
- 後続への注意: 割り込み用スタックが無いので、ISR 内のログは割り込まれたスレッドのスタックを約 500 B 使う（Idle は 1000 B 中 452 B、
  ISR でログを出すと 704 B まで増えた）。T11/T12 でスタックが足りなければ deferred 化か `IDLETHREAD_STACKSIZE` を検討。
  パッチ `0004` の説明文の「Not used yet」は古い（差分は正しい）。
- 未確認: 電源断での RAM ログ消去、RESET ボタン、USB 大量送受信中の負荷。
- 実機: T05 の最終ビルド（Nov 14 2026 01:48:46）。

### T06 KV（完了、dc2058d）

- `boards/…/src/nano_flash_kv.[ch]`（末尾 8 KB の MTD、blocksize 1、消去 4096×2、範囲検査 2 段、RAM 関数 `kv_flash_op`）、
  `MTD_CONFIG_NVS`＋`MTD_CONFIG_NAMED`、`MTD_CONFIG_NAME_LEN=64`、`/dev/config`。`apps/imu/kv_store.[ch]`（名前は `app_kv/<key>`、
  呼び出しごとに open→ioctl→close、排他は NVS のミューテックス）、`kv_name.[ch]`、`apps/kv/`（`kv_hex.[ch]`）。
  `apps/imu/{Kconfig,Make.defs,Makefile}` は `CONFIG_NANO_KV_STORE` のライブラリ（T12 で広げる）。
- **Zephyr 版の KV は最初の NuttX 起動で消去済み**（NuttX の形式でも全面消去でもない内容は起動時に消去する。`<wrn> fs_nvs: Erasing…`）。
  退避は `host/recordings/kv-backup.json`。Zephyr 版に戻すときは `kv_transfer.py restore` が要る（Wi-Fi 資格情報も）。
- 実機の KV: `imu.port`=18830、`imu.cal.accel.244`、`imu.cal.gyro.70000` の 3 つ（`kv_transfer.py restore --usb --no-wifi` で書き戻し済み）。
- `kv_transfer.py restore` は最後の `imu cal reload` が `command not found` で終了コード 1（T22 まで。スクリプトは変えていない）。
- 実測: 4 KB 消去の割り込み禁止 33.8〜40.2 ms、256 B 書き込み 0.37〜0.51 ms、1 項目の書き換えでページ書き込み 4 回。
  容量: 32 文字キー＋128 B 値で 22 項目目が -28。GC 200 回超でエラーなし。試験の前後でイメージと KV 直前の CRC は不変。
- 差分: store が無いときは `-ENODEV`、マウントは起動時の 1 回だけ、`fs_nvs` の alloc/data wra の行は無い、エラー行は stderr、終了状態は 0/1、
  nsh が `$VAR` を展開する。
- 後続への注意: GC を含む KV の書き込みは呼び出し元のスタックを約 1.2〜1.4 KB 使う（T12 の imu_stream は 2048）。満杯に近いと
  書き込みごとに GC（割り込み禁止約 37 ms×最大 2 回。Zephyr も同じ方式）。T17 は同じ `/dev/config` に資格情報を置く。
- メインのレビュー: `kv_store.c`・`main.c` と並べて確認、ホストテスト 17 本、ビルド、実機で `kv` の各出力・`dmesg`・`ps` を再確認。
- T05 の訂正: BOOTSEL 経由の書き込み後も RAM ログが残る場合がある（T06 の間は残った。T05 では空になった）。条件は未確認。
- 未確認: 電源断の耐性、NuttX 版の後に Zephyr 版を書いたときの動作。
- 実機: T06 の最終ビルド（Nov 14 2026 02:18:48）。

### T07 NINA の SPI 転送と nina version（完了、96b8f5f）

- `drivers/nina/nina_spi.c`（Zephyr の `nina_wifi.c` と行単位で同じ。メインが diff で確認: 違いは OS 呼び出しの置き換えと、ロック失敗時に
  値を返す経路だけ）、`nina_proto.h`（写し）、`nina_os.[ch]`（`rmutex_t`、`nxsched_usleep/msleep`、`TICK2MSEC(clock_systime_ticks())`。
  後続は `nina_os_lock()` を使う）、`drivers/include/nano_nina.h`＋ボード `src/nano_nina.c`（ピン、SPI1、`SPI_LOCK`＋`SPI_EXCHANGE`、CS は GPIO）。
  `CONFIG_NANO_NINA`、`CONFIG_NANO_NINA_SPI_FREQUENCY=8000000`（実際は 7.8125 MHz、Zephyr と同じ）。SPI はポーリング（DMA は切った）。
- `apps/subcmd/`（Zephyr シェルのサブコマンドのヘルプ・`wrong parameter count`・ワイルドカードの規則。`SUBCMD()`＝`SHELL_CMD`、
  `SUBCMD_ARG()`＝`SHELL_CMD_ARG`。**後続のコマンドはこれを使う**）。`apps/kv` は subcmd に置き換え済み、`apps/nina/`。
- `nuttx/drivers/README{,_ja}.md` は NINA 転送層の節の骨組みだけ（T24 でまとめる）。
- 実測: 最初の要求は初期化 760 ms を含む。以後 1 要求（0x37）平均 591 µs（READY 待ち 203＋247、送信 44、読み出し 82）、CPU 約 363 µs/要求、
  1 バイト転送 5.1 µs/回、まとめては約 1.25 µs/B、`sleep_us(100)` は 1 回 200 µs（CPU 約 50 µs）。100/100、1000/1000 成功。2 スレッド同時で重なり 0。
- 差分: 起動時に CS が数命令ぶん low になる（in-tree の common の順序）、ワイルドカードがサブコマンドに一致する場合は未実装、
  ヘルプの 80 桁折り返しは未実装。Zephyr 版の所要時間・CPU 時間は未計測。
- 注意: Zephyr 版は起動直後に監視スレッドが NINA に触るので最初の `nina version` にリセット時間が入らない見込み（T16/T17 で同じになる）。
- メインの確認: ホストテスト 19 本、ビルド、実機で `nina version` 20/20、`nina`・`kv` のヘルプ、`dmesg`、`ps`。
- 実機: T07 の最終ビルド（Nov 14 2026 02:43:08）。KV は 3 つのまま。

### T08 I2C0 ドライバー（完了、0b7c3f7）。CP2（dmesg・kv・nina version・WHO_AM_I）通過

- **計画からの変更（メインの判断）**: in-tree の `rp2040_i2c.c` の写しではなく、Zephyr 版の `drivers/i2c_dw/i2c_dw.c` のコントローラー側を
  土台にした（`boards/…/src/nano_i2c.c`、`nano_i2c_dw.h`（OS 非依存、ホストテスト `nano_i2c_dw`）、`nano_i2c_dw_registers.h`）。関数名・分岐・順序は
  Zephyr の写し。メインが `i2c_dw_transfer` を diff で確認（違いは NuttX の API の変換、PM・ターゲットモード・DMA の除去）。
- `/dev/i2c0`、`CONFIG_NANO_I2C0`、`SYSTEM_I2CTOOL`（`i2c get -b 0 -a 6a -r 0f` → 6c。`i2c dev 03 77` で 0x60 と 0x6a）。
  周波数は 100000/400000/1000000 ちょうどだけ（他は `-EINVAL`）。メッセージごとの `frequency` で区分を決める。
- レジスタ値は Zephyr 版と一致（SS 500/625、FS 93/109（400k）、31/31（1M）、SPKLEN 1、SDA_HOLD 1、IC_CON 0x363/0x365）。
- 実測（1M）: WHO_AM_I 95 µs・割り込み約 3、256 B 1.73 ms・68〜80 回、3584 B 23.3 ms・900〜1040 回・CPU 約 8.4 ms。
  **1M の区分の実効ビットレートは約 1.39 MHz、400k は約 541 kHz**（レジスタ値が同じなので Zephyr 版も同じはず。波形は未確認）。
- T09 の DMA 分岐の場所: `struct i2c_dw_dev_config` に `rp_dma_*`、`i2c_dw_transfer()` のバス取得直後（quarantined）、メッセージループの
  `write_rx_tl()` の直後（`msg_left == 1 && READ`。`dw->xfr_len/xfr_buf/xfr_flags` を使う）、`i2c_dw_isr()` の `intr_stat` 読み出し直後、
  コマンド語の生成は `nano_i2c_dw.h`。`rp_dma_enabled` は既にあり IC_CON bit 9 に使用。
- 差分: 割り込み優先度（Zephyr は I2C が最低の 3、NuttX は全 IRQ 同じで入れ子なし）、`i2c` ツールは優先度 100、Zephyr の「受け付けない速さで
  最小カウントが残る」不具合は再現しない、割り込み 1 回の入口・出口コスト（NuttX 約 8〜10 µs、Zephyr 未測定）。
- 注意: NACK のログが割り込み内で出ると Idle のスタックが 452 → 664〜712/1000 B。T09 で割り込み用スタック（`ARCH_INTERRUPTSTACK`）を調べる。
- 未確認: SCL の波形、arbitration lost・stuck の経路、Zephyr 版の割り込み回数と CPU 時間。
- 実機: T08 の最終ビルド（Nov 14 2026 03:09:05）。LSM6DSOX のレジスタは元の値。

### T09 I2C の DMA 読み出し・i2c_dma・割り込み用スタック（完了、6af1618）

- `nano_i2c.c` に Zephyr の DMA 経路（`i2c_dw.c:577-768` ほか）を追加。メインが diff で確認（違いは atomic → 割り込み禁止の OR／バスのセマフォ下の
  カウンター、Zephyr の DMA API → `nano_dma_*`、時計と sleep の置き換えだけ）。`src/nano_dma.[ch]` は Zephyr の `dma_rpi_pico.c` をレジスタ直書きで
  移したもの（チャネル 0 が DMA_IRQ_0、他が DMA_IRQ_1）。**`CONFIG_RP2040_DMAC` は切った**（in-tree は停止が無期限待ち、全チャネル DMA_IRQ_0、確保で待つ）。
- `apps/i2c_dma/`（`mode [on|off]`、`stats`。出力は Zephyr と同じ）、`drivers/include/nano_i2c_dma.h`（`nano_i2c_dma_get_stats/enabled/set_enabled`）。
  「切り替えは止まっているときだけ」は Zephyr 版もコードでは保証していない（説明だけ）。同じ。
- **割り込み用スタック `CONFIG_ARCH_INTERRUPTSTACK=2048` を有効にした**（Zephyr の `ISR_STACK_SIZE=2048` と同じ。パッチ不要。スレッドは PSP、ハンドラーは MSP）。
  **これより前の記録にある「割り込み用スタックが無い」「ISR 内のログがスレッドのスタックを使う」は古い。** Idle は ISR でログを出しても 304 B のまま。
  割り込み用スタックの使用量は通常 364 B、ISR 内ログで 624 B、PANIC 報告で 720 B。コスト変化なし。番兵は割り込み出口・切り替えとも動作。
  差分: 割り込み用スタックの使用量は `ps` に出ない（Zephyr は `kernel thread stacks` の `IRQ 00`）。
- 実測（1 MHz、DMA／割り込み）: 256 B は 1.79／1.76 ms、CPU 0.26／0.76 ms。3584 B は 22.9／23.1 ms、CPU 0.75／8.9 ms、割り込み 4〜5／1139 回。
  NACK・期限切れ（1000.86 ms）・quarantine の各経路と復帰、1000 回連続、2 タスク同時を確認。
- RAM: ヒープ total 238468 B（DMA バッファー 10752 B と割り込み用スタックで −13.2 KB）。
- メインの確認: ホストテスト 22 本、ビルド、実機で `i2c_dma` の各出力、`i2c dump … 64` で `dma_transfers=1 dma_bytes=64`、`nina version`、`kv`、`ps`、`free`。
- 未確認: Zephyr 版の DMA の所要時間・CPU 時間、SCL の波形、DMA のバスエラーの経路。
- 注意: ホスト側 `board.py` はプロンプトを `$` で判定するので、`i2c dump` の hexdump の `$` を誤認することがある（T24 で `nsh$ ` の判定を厳しく）。
- 実機: T09 の最終ビルド（Nov 14 2026 03:42:15）。

### T10 LSM6DSO ドライバー本体（完了、8ffc396）

- `drivers/lsm6dso/lsm6dso.c`・`lsm6dso.h`（Zephyr の写しで OS 呼び出しだけ置き換え。メインが diff で確認）、`lsm6dso_fifo_hw.[ch]`・
  `lsm6dso_fifo_stats.h`・`lsm6dso_attributes.h`（無変更の写し）、`stmemsc.h`・`stmemsc_i2c.c`・`stmemsc_mdelay.c`、`lsm6dso_os.[ch]`、
  `st_lsm6dso_reg.c`（`nuttx/stmemsc` の ST のファイルを include。hal_st と `cmp` 一致）。`CONFIG_NANO_LSM6DSO`。
- **Zephyr 互換層 `nuttx/drivers/compat/`**（`zephyr/device.h`・`devicetree.h`・`drivers/sensor.h`・`drivers/i2c.h`・`sys/util.h`・`rtio/rtio.h`（仮置き、T11 で本物に）、
  `zephyr_i2c.c`・`zephyr_device.c`）: `struct device`、`DEVICE_DT_GET(DT_NODELABEL(lsm6dsox))`、`sensor_attr_set/get`・`sensor_sample_fetch`・
  `sensor_channel_get`、`i2c_configure()`・`I2C_SPEED_SET`。利用者は `-I$(TOPDIR)/external/compat`・`external/lsm6dso`・`../stmemsc` を足す。
  バス速度は互換層の転送にだけ効く（`i2c` ツールは自分の `-f`）。
- ボード `src/nano_lsm6dso.[ch]`、bringup は I2C0 → LSM6DSO → KV の順（Zephyr の順）。dmesg は `LSM6DSO: Initialize device lsm6dsox@6a` →
  `chip id 0x6c` → `fs_nvs` → `usb_console`。
- T11 が入る場所: `CONFIG_NANO_LSM6DSO_STREAM`（Zephyr の `CONFIG_SENSOR_ASYNC_API` に対応）、`compat/zephyr/rtio/rtio.h`、`lsm6dso.h` の `fifo_gpio`、
  `lsm6dso.c` 末尾のデバイス定義の `.fifo_gpio`、`sensor_driver_api` の `get_decoder`・`submit`。`timestamp_decimation` は T10 で `#ifdef` の外に出してある。
- 実機: 初期化後のレジスタは期待どおり（CTRL1_XL=0c、CTRL2_G=0c、CTRL3_C=44、CTRL9_XL=e2 ほか 00。メインも `i2c dump` で確認）。単発取得 |a| 9.87〜9.89、
  属性、FIFO の start/read/pause/drain/stop、`cal_zero`（1.38 s）を subagent が確認。
- 差分: 「Initialize」〜「chip id」の間隔（Zephyr 3.75 ms、NuttX 約 1.3 ms。T11 の stream_init で縮まる見込み）、`i2c_configure` の隔離時の戻り値
  （NuttX は 0 で次の転送が `-EIO`）、FIFO 転送時間の分解能（TIMER 1 µs）、errno の番号（上の表）。
- 未確認: 実機での初期化失敗の経路、電源断からの起動。ODR を入れた直後の数サンプルに大きな値が出た（センサーの立ち上がりと推測、未検証）。
- 実機: T10 の最終ビルド（Nov 14 2026 04:12:32）。ODR は停止。

### T11 RTIO 互換層と FIFO ストリーム（完了、cd21b3f）

- `lsm6dso_stream.c`・`lsm6dso_fifo_irq.[ch]`・`tests/stream_stop.c` は Zephyr 版と `cmp` で一致。`lsm6dso_compat.c`・`compat/sensor_default_rtio.c` はログだけ置換。
- 互換層（`drivers/compat/`）: `zephyr/kernel.h`＋`zephyr_kernel.c`（k_mutex=rmutex、k_sem=sem_t＋上限、k_poll はセマフォだけ・NOTIFY_ONLY、
  ワークキューの kthread は静的スタック、遅延ワークは wdog）、`zephyr_work.c`（`kernel/work.c` の写し。flush/drain/plug/stop なし）、
  `zephyr_rtio.c`＋`rtio/rtio.h`（チェーン・トランザクション・executor 自身の操作なし）、`zephyr_mem_blocks.c`、`zephyr_rtio_workq.c`、`zephyr_gpio.c`＋ボードの `nano_gpio.c`、
  `zephyr_sensor_clock.c`（TIMER の 1 µs）。`zephyr_compat_initialize()` を bringup の最初（DMA・I2C の前）で呼ぶ。
- **`k_spin_lock()` は `sched_lock()`＋`up_irq_save()`**: NuttX は割り込み禁止中でも `nxsem_post()` で即切り替えるため（最初の実機で PANIC、修正済み）。互換層の作法として固定。
- **規則**: NuttX のヘッダーを互換層のヘッダーより先に読む（`<zephyr/sys/atomic.h>` が NuttX の atomic の名前を引き取る）。
- スレッド: `sysworkq` 119/1024（fifo_irq の `k_work_submit` はここ。hpwork は使わない）、`RTIO_WQ_0` 118/1024、`imu_fifo_worker` 108/2048。
- メインのレビュー: `zephyr_work.c`↔`kernel/work.c`、`zephyr_rtio.c`・`rtio.h`↔`rtio_executor.c`・`rtio.h`、`zephyr_rtio_workq.c`↔`rtio_workq.c`、GPIO↔`gpio_rpi_pico.c` を並べて確認、指摘なし。
  ホストテスト 27 本 ok、ビルド ok、実機で `uname`・`dmesg`（`<err>` なし）・`ps`・`free`（total 233180）・`i2c dump` を再確認。
- subagent の実機確認（1 MHz、6667 Hz、WM 128、試験コマンドは削除済み）: 10 秒×8 回（完了 1070〜1072、エラー 0、全転送 DMA）、停止/キャンセル交互 20 回、
  消費停止 3 秒で 2 バッチ後に `-ENOMEM` 終端（21 回）、60 秒 1 回（6418 バッチ、CPU 26.1 %、INT1→CQE 平均 5.48 ms・最大 13.2 ms）。
  スタック: `imu_fifo_worker` 928/2008、`sysworkq` 304/976、`RTIO_WQ_0` 728/976（74.5 %、単発取得後）、割り込み用 364/2048。
- 差分（表にも追記）: sysworkq が協調でない（119 より上は無い）、mpsc が割り込み禁止の FIFO、プールの空きリストは初回使用時に作成、GPIO 割り込み優先度、
  センサー時計が TIMER（Zephyr は SysTick 系の cycle）、`ps` の名前 `RTIO_WQ_0`、`k_work_cancel_sync` の待ちは 1 スレッド、`-Wformat` 警告 4 件（写しを保つため放置）、
  「Initialize」〜「chip id」約 1.75 ms（Zephyr 3.75 ms）。
- 未確認: 実機での CQE あふれ・`-EOVERFLOW`・I2C エラー経路（ホストテストのみ）、Zephyr 版の同条件の CPU 負荷と INT1 回数（NuttX はバッチあたり約 2.5 回）。
- T12 への注意: 互換層に無いもの = `K_MUTEX_DEFINE`、`K_THREAD_DEFINE`、`k_uptime_get`、`k_sleep`、`snprintk`、`sys_get_be32`/`sys_put_be32`、`hwinfo_get_device_id`、shell、`zsock_*`。
  試験時の消費側は優先度 109・スタック 2048。確認用スクリプトを scratchpad 直下に置くと残っている `enum.py` が標準ライブラリを隠すので、サブディレクトリ（`scratchpad/rev/sh.py`）から動かす。

### T12 取得サービス（完了）と T13 CP3（通過）

- `apps/imu/imu_stream.c` は Zephyr 版との diff が 3 か所だけ（先頭の NuttX ヘッダー、`subcmd.h`、末尾のコマンド登録）。`imu_raw_mqtt.h` は include 1 行、`time_sync.h` は無変更。
- **スタブ（後続で置換）**: `imu_raw_mqtt_null.c`（T21。キュー未接続＝全バッチを `dropped_*` に数えて `-EAGAIN`、接続待ち 3 秒はそのまま）、`time_sync_null.c`（T19。`time_sync_mono_us` だけ本物）、
  `imu_cal_null.c`（T22。`imu cal` は `Calibration is not available in this build yet`／-ENOSYS）。`zephyr/net/net_if.h` は空（T16）。T21 では `k_cycle_get_*` が互換層に要る。
- 互換層に追加: `K_MUTEX_DEFINE`、`K_THREAD_DEFINE`（init が `K_THREAD_DEFINE_START(name)` で起動）、`k_uptime_get`（TIMER の ms 切り捨て）、`k_sleep`、`snprintk`、`sys_put/get_be32`、
  `hwinfo_get_device_id`（flash ID の生 8 B）、`shell_print`/`shell_error`（stdout/stderr）、`zsock_inet_pton/ntop`。
- スレッド名は Zephyr の実名 `imu_stream_tid`（109、2048）。`imu` は組み込みタスク（104、3072）。init の順: スレッド起動 → `USB console started` → `kv_store_init` → `imu_cal_init` → `time_sync_init`（`wifi_profile_init` は T17）。
- ヒープ total 233180 → 187932（送信キュー 12×2448 B を静的確保）。
- subagent の実機確認: `imu`・`imu status`（idle）が Zephyr の記録と文字単位で一致。書き込み直後を含む 10 秒×4・60 秒×1、開始停止 20 回、境界コマンド、セッション通し番号 +1、すべて異常なし。
- **メインの再確認（CP3）**: ホストテスト 28 本 ok。5 分連続 1 回（約 308 秒、2,064,371 サンプル＝約 6705 Hz、errors・fifo 全 0、`cpu_active` 39.2 %、`max_transfer_us` 7075、全転送 DMA）。
  ボードの時計は PC と一致（`/proc/uptime` 128 秒で差 0.01 秒未満）。`i2c_dma mode off` で 10 秒: DMA 転送 0・エラー 0・`cpu_active` 58.1 %、`mode on` に戻した。
  スタック: `imu_stream_tid` 1024/2000、`imu_fifo_worker` 928/2008、`sysworkq` 304/976、nsh 1712/3032。`dmesg` に `<err>` なし。
- 注意: `imu stop` は非同期（Zephyr と同じコード）。直後の `imu status` は `running` や 1 つ前のスナップショットを返すことがある。確認スクリプトは stop の後 2〜3 秒待つ。
  メインの `scratchpad/rev/run.py` は一部のコマンドでプロンプト検出が 8 秒のタイムアウトになる（経過時間を PC 側の sleep で数えない）。
- 差分: エラーは stderr・終了状態 0/1（`kv` と同じ）、stdout はコマンド終了時にまとめて出る、`-Wformat` 警告 1 件、`k_uptime_get` の時間源が TIMER。
- 観察（未調査）: `us_per_1e6_ticks` が Zephyr の記録と約 450 ppm 違う（24855xxx 対 24864xxx）。センサー発振器の変動の範囲かは CP5（NTP あり）で見る。
- 未確認: `restart_stream`（FIFO エラーからの回復）、`stop_timeout`、CQE あふれ、計測中の KV の GC、電源断。送信スタブなので Zephyr の基準（`cpu_active` 63〜64 %）との比較は CP5。
- 実機: T12 の最終ビルド（`Nov 14 2026 05:40:38`）。`imu.session` は進んでいる（0x21 以降）。`i2c_dma` は on。ODR は 6667 Hz のまま（Zephyr の `imu stop` と同じ）。

### 書式の警告の修正（2026-11-14、ユーザー指示）

- `imu_stream.c`（`Streaming %d Hz`）、`sensor_default_rtio.c`・`lsm6dso_compat.c`（各 2 か所）の `%d`/`%u` を `PRId32`/`PRIu32` に直した（commit は `git log` の "Use PRId32/PRIu32"）。
  写しとの diff はこの分だけ増える。**方針: Zephyr の写しでも `-Wformat` の警告は PRIx32 で直す**（今後の写しも同じ）。
- 確認済み: `build.sh --clean` で `-Wformat` の警告は 0（残るのはリンカーの RWX の警告だけ）。

### T20 MQTT 最小クライアント（完了。`host_tests.sh` の行は T14 の commit に入れる）

- `drivers/compat/zephyr/net/mqtt.h`（Zephyr の名前のヘッダー）、`apps/imu/mqtt_min.c`（Zephyr `subsys/net/lib/mqtt/` の 3.1.1・TCP の部分を 1 ファイルにまとめた写し。5.0 の分岐とログを除去）、
  `mqtt_min_sock.c`（POSIX ソケット層）、`mqtt_min.h`（keepalive 60、clean_session 0）。NuttX のビルドにはまだ入れていない（T21）。
- ホストテスト `mqtt_min` は**同じテストを Zephyr 本物のライブラリにもリンクして実行**する（`tests/mqtt_ref/` の代用ヘッダー。`zephyr/zephyr` が無ければ skip）。両方 ok。計 30 本。
- メインの確認: `imu_raw_mqtt.c` が呼ぶ関数（client_init、connect、input、live、publish、abort）がすべてあること、`mqtt_rx.c`・`mqtt.c` との行単位の差が 5.0 とログだけであること。
- 未実装: SUBSCRIBE、QoS 2 の送信、`mqtt_publish_qos1_ack`、5.0、TLS、WebSocket、SOCKS、`if_name`（-ENOTSUP）。
- **T15 への注意（重要）**: NuttX の `usrsock_sendmsg` は iovec をそのまま渡す。Zephyr の `nina_socket.c:492-527`（`do_sendmsg`）は 3584 B まで 1 つに連結して NINA へ 1 回で書くので、
  NINA 側の sendto は複数 iovec を連結して 1 回で書く（「ヘッダーと本体を 1 回の送信で出す」）。
- T21 への注意: Makefile に `mqtt_min.c mqtt_min_sock.c` を足し `imu_raw_mqtt_null.c` を外す。互換層に `zsock_poll`・`struct zsock_pollfd`・`ZSOCK_POLL*`、`net_if_get_default`・`net_if_ipv4_get_gw`、
  `k_cycle_get_32/64`・`k_cyc_to_us_*` が要る。送信スレッドは Zephyr 優先度 11（→107）、スタック 2048（Zephyr の実測 1260）。
- 差分: `struct mqtt_disconnect_param` に 1 バイトのダミー、`MQTT_UTF8_LITERAL` のキャスト、ホストテストは `-Wno-sign-compare`。

### T14 NINA usrsock 制御系（コードはレビュー済み、未 commit、実機確認が途中）

- **実機の状態（2026-11-14）: ボードが USB に列挙されない。** T14 の試験版（不具合入り）が載ったまま HardFault で止まり、USB リセット後は `device not accepting address`。
  **ユーザーに手動 BOOTSEL を依頼中。** BOOTSEL に入ったら `nuttx/build/nuttx.uf2`（修正済みの最終版、`--clean` ビルド済み）を `nuttx/script/flash.sh` で書く。
- 原因と修正: 要求バッファ `g_request`（`uint8_t` 配列）が奇数番地に置かれ、そこを `struct sockaddr_in` として読んで Cortex-M0+ の非整列アクセスで HardFault。
  `aligned_data(4)` と、アドレスを `memcpy` で写す `get_addr()` に修正（実機では未確認）。**教訓: usrsock の詰めた構造体の後ろは必ず memcpy で読む（T15 も）。**
- ファイル: `drivers/nina/nina_sock.[ch]`（Zephyr `nina_socket.c` の表・bind/listen/accept/connect/close/setsockopt の写し、OS 非依存）、`nina_usrsock.[ch]`（`usrsock_request()`）、
  `nina_poller.c`（スレッド `nina_event` 106/1536、20 ms ごとに usrsock がまだ持っていないイベントを問い合わせる）、`tests/nina_sock.c`。defconfig に `NET`・`NETDEV_LATEINIT`・`NET_USRSOCK(_CUSTOM/_TCP/_UDP)`・`NET_USRSOCK_PREALLOC_CONNS=12`。
- メインのレビュー: `nina_sock.c` を `nina_socket.c:1-372` と並べて確認（定数、NINA コマンド、エラー値、connect の 10 s／20 ms、失敗後の close が一致）。ホストテスト 31 本 ok、`--clean` ビルド ok。
- メインの判断（subagent の質問への回答）: ioctl は `ENOTTY` のまま（NuttX の netdev 処理に任せるため）。`nina_event` は 106 のまま（取得スレッド 109 を邪魔しない側。CP5 で送信の遅れを見て再検討）。
  誰も待っていないソケットも 20 ms ごとに問い合わせる点と、非ブロッキング connect の `EINPROGRESS` は許容（差分として記録、CP5 で CPU 負荷を見る）。
- 差分（詳細は subagent の報告の表）: connect の確認が呼び出し元でなく `nina_event`、accept はブロッキングなら待つ、close は NINA の失敗を返さない、2 回目の listen は `EISCONN`、
  poll のエラーは接続済み TCP で `REMOTE_CLOSED`、未実装のデータ系は `ENOSYS`（T15）。
- **実機で未確認（BOOTSEL 後にやる）**: 修正版の起動、`socktest opts`（止まった箇所）、未接続時の connect のエラー値と所要時間、listen・accept・poll、NINA 側ソケット 10 個、`nina_event` のスタック、
  回帰（imu 10 秒、`kv get imu.port`、`i2c_dma stats`、`reboot`）。試験コマンドの写しは `scratchpad/t14/socktest_tmp/`（`nuttx/apps/socktest_tmp/` に戻して使う。最終状態には残さない）。
- 済んだ分（試験版、各 1 回）: socket 10 個成功・11 個目 `EMFILE`、close 10 回、`TCP_NODELAY` ほかのオプションのエラー値、`free` total 182740。
- T15 への注意: `reported` のビットをデータ応答で落とす、`MSG_DONTWAIT` はドライバーに届かない（送信の 1 ms 再送と 10 s 期限は `nina_event` 側で）、DATA_ACK は値→データの順、
  送信データは `usrsock_iovec_get()`、複数 iovec は連結して 1 回で NINA へ（T20 の記録）。T16: `NETDEV_LATEINIT`、`DRIVERS_WIRELESS`・`DRIVERS_IEEE80211`・`NETDEV_WIRELESS_IOCTL`。
  T18: telnetd の `SO_REUSEADDR` が `ENOPROTOOPT` になる。
- **T14 の commit 後にやること**: README の NuttX 版だけの実測値を削除（先に STATUS へ写す）、割り込み用スタックの表示コマンド、`host_tests.sh` の mqtt の行も T14 の commit に入る。

### T14 の続き（2026-11-14、完了・commit 済み）

- ユーザーが手動 BOOTSEL → 修正版を書き込み → 実機確認を完了。**実機は T14 の最終版（`Nov 14 2026 06:49:42`）、USB で応答、試験コマンドなし。** 上の「ボードが列挙されない」は解消。
- 実機で見つけて直したバグ: 失敗した connect が `ECONNREFUSED` を返しソケットが接続中のまま残った（`close_id()` が `connect_xid` を消し、最後の応答の代わりに `REMOTE_CLOSED` が出ていた）。
  修正後: 未接続時の TCP connect は 10.02 s で `ETIMEDOUT`（6 回）、つなぎ直せる。非ブロッキングは `EINPROGRESS` → poll で POLLERR|POLLHUP。
- 止まった箇所（UDP connect）は修正版で通過。listen・accept・poll・bind のエラー値は Zephyr 版どおり（2 回目の listen だけ `EISCONN`）。UDP 10 個 bind 成功、11 個目 `EMFILE`。`SOCK_CTRL` は NuttX 側へ委譲。
- **実測（STATUS にだけ記録）**: ソケットを開いて放置したときの CPU 負荷 = ソケットなし 0、UDP 1 個 bind 2.3 %、待ち受け 1 個 2.3 %、UDP 10 個 13.8 %（1 ソケットあたり 20 ms ごとに約 0.46 ms）。
  `nina_event` のスタック 520/1488。`free` total 182740。→ CP5 で送信中の `cpu_active` を Zephyr の基準と比べ、問い合わせ方を見直すか決める。
- 非整列アクセスの見直し: 要求は packed 構造体のメンバー経由、アドレスは `memcpy`、他に同種の箇所なし。前回の panic ダンプは RAM ログに残っていて見立てを裏付けた（`dmesg` に古い `dump_task` 行が出るのはそのため）。
- 回帰: imu 計測（errors・fifo 0）、KV、`i2c_dma`、`reboot` 正常。ホストテスト 31 本。
- 未確認: Wi-Fi 接続時の動作（CP4）、ソケットを使いながらの計測の CPU 負荷、長時間動作。

### 進行中（2026-11-14、T14 の commit 8223064 の後）

- T15 を subagent（opus）に依頼中。並行して別の subagent が README 4 ファイルから NuttX 版だけの実測値を削除中
  （削除した原文は `scratchpad/readme_removed/removed_ja.md` に写させている。届いたらメインがこのファイルへ移して commit）。T15 は README を最後に更新する。
- 残りの小作業: 割り込み用スタックの表示コマンド（`up_check_intstack()` を呼ぶ小さな組み込みコマンド。T15 のレビュー後、ビルドツリーが空いてから）。

## README から外した NuttX 版の実測値（2026-11-14、ユーザー決定により README から削除。原文の写し）

### NuttX 版の文書から削除した実測値（2026-11-14）

ユーザーの決定（2026-11-14）「NuttX 版のみでの計測であれば記載しなくて良い」により、
`nuttx/README_ja.md`・`nuttx/README.md`・`nuttx/drivers/README_ja.md`・`nuttx/drivers/README.md` から
削除した箇所の日本語版の原文。英語版は同じ箇所を同じように削除した。
「→」の後は、削除後に残した（言い換えた）文。

#### nuttx/README_ja.md

##### 「ログ（`dmesg`）」

> `LOG_INF()` 1 回に約 42 µs、スタックを約 500 バイト使います（実機で 100 回の平均）。

→ 文ごと削除。

##### 「キーと値の保存（`kv`）」— 容量

> （実機で 21 項目と `imu.port` が入り、次で `KV write failed: -28`）

→ 「（実機で、満杯の次の書き込みが `KV write failed: -28` になることを確かめました）」

##### 「キーと値の保存（`kv`）」— 書き換えと整理

> 禁止する時間は 4 KiB の消去 1 回が
> 33〜40 ms、256 バイトの書き込み 1 回が 0.37〜0.51 ms です（実機、TIMER で測定。消去 200 回余り、
> 書き込み 3 万回余り）。

→ 「割り込みを禁止するのは、4 KiB の消去 1 回と 256 バイトの書き込み 1 回のそれぞれの間です。」

##### 「NINA-W102（`nina`）」— 初期化

> そのため起動（書き込み・`reboot`）後の最初の `nina version` は約 0.76 秒かかります（実機で 761 ms。
> 2 回目以降は約 0.6〜0.7 ms）。

→ 「そのため起動（書き込み・`reboot`）後の最初の `nina version` は、この待ちの分だけ時間がかかります。」

##### 「NINA-W102（`nina`）」— 実測

> - 実測（実機、TIMER で測定。`nina_request(0x37)` を間を置かずに 1000 回）: 要求 1 回は平均 591 µs
>   （最小 427、最大 697）、CPU 時間は約 360 µs（`cpu_load_get()` の差から）。1 バイトの転送は 1 回あたり
>   約 5.1 µs、まとめての転送は 1 バイトあたり読み出し 1.26 µs・書き込み 1.22 µs（線上の 8 ビットは 1.02 µs）。
>   READY を 1 回見るたびの待ちは約 200 µs、その CPU 時間は約 50 µs です。

→ 項目ごと削除。

##### 「I2C0（`i2c`）」— 割り込みでの転送の実測

> - 実測（実機、TIMER と `cpu_load_get()`、間を置かずに繰り返した平均）:
>
>   | 転送 | 100 kHz | 400 kHz | 1 MHz | CPU 時間 | 割り込み |
>   |---|---|---|---|---|---|
>   | WHO_AM_I（1 バイト書いて 1 バイト読む） | 411 µs | 128 µs | 95 µs | 約 95 µs | 5 回（1 MHz では約 3 回） |
>   | 256 バイト読み出し | 21.6 ms | 4.37 ms | 1.73 ms | 約 0.7 ms | 68〜80 回 |
>   | 3584 バイト読み出し | 298 ms | 59.7 ms | 23.3 ms | 約 8.3〜8.5 ms | 900〜1040 回 |
>
>   3584 バイトの所要時間を 1 ビットあたりにすると、100 kHz で 9.24 µs、400 kHz で 1.85 µs、1 MHz で
>   0.72 µs です。この表は DMA を使わないときの値です（DMA との比較は次の節）。

→ 項目ごと削除。

##### 「DMA での読み出し（`i2c_dma`）」— DMA と割り込みの比較

> 実測（実機、1 MHz、LSM6DSOX のレジスターを 1 バイト書いてから読む転送、TIMER と `cpu_load_get()`、
> 間を置かずに繰り返した平均。割り込みの回数は 1 回の転送で数えた I2C0 の割り込み（レジスターを書く
> メッセージの分を含む）と DMA の割り込み）:
>
> | 読み出し | 方式 | 所要時間 | CPU 時間 | 割り込み |
> |---|---|---|---|---|
> | 256 バイト | DMA | 1.79 ms | 約 0.26 ms | I2C 3 回、DMA 2 回 |
> | 256 バイト | 割り込み | 1.76 ms | 約 0.76 ms | I2C 84 回 |
> | 3584 バイト | DMA | 22.9 ms | 約 0.75 ms | I2C 2〜3 回、DMA 2 回 |
> | 3584 バイト | 割り込み | 23.1 ms | 約 8.9 ms | I2C 1139 回 |
>
> 所要時間と CPU 時間には、`ioctl()` で転送を渡す処理とレジスターを書くメッセージの分が含まれます
> （1000 回・200 回・50 回の平均）。DMA で 256 バイトを 1000 回続けて読んでエラー 0、2 つのタスクから
> 同時に 500 回ずつ読んでもエラー 0（バスのロックで 1 回ずつ進みます）。DMA と割り込みで同じ範囲を
> 読んだ内容は、値の変わるレジスター（出力、状態、タイムスタンプ）を除いて一致しました。

→ 「実機（1 MHz）で、DMA で 256 バイトを 1000 回続けて読んでも、2 つのタスクから同時に 500 回ずつ
読んでも（バスのロックで 1 回ずつ進みます）エラーにならないこと、DMA と割り込みで同じ範囲を読んだ内容が、
値の変わるレジスター（出力、状態、タイムスタンプ）を除いて一致することを確かめました。」

##### 「LSM6DSOX（IMU）」— FIFO のストリーム

> 実機での確認（1 MHz、6667 Hz、ウォーターマーク 128、`imu_stream.c` と同じ RTIO の寸法。T11 の時点の
> 一時的な試験コマンドで、消費側は受け取って解放するだけ）: 10 秒で完了 1070 前後、加速度・ジャイロ各
> 約 67080 ワード（センサーの発振器が約 0.6 % 速い。`freq_fine` 4）、エラー・オーバーラン・`i2c_dma` の
> エラーとフォールバックは 0、INT1 の割り込みから完了を受け取るまで平均 5.5 ms（ほぼ FIFO の読み出しの
> 時間）、CPU 負荷 26 %。60 秒の連続でも同じでした。

→ 「実機で（1 MHz、6667 Hz、ウォーターマーク 128、`imu_stream.c` と同じ RTIO の寸法。T11 の時点の
一時的な試験コマンドで、消費側は受け取って解放するだけ）10 秒と 60 秒の連続で動かし、エラー・
オーバーラン・`i2c_dma` のエラーとフォールバックが出ないことを確かめました。」

##### 「IMU の取得（`imu`）」— 実機での確認

> 実機での確認（送信スタブ、ネットワークなし、6667 Hz、ウォーターマーク 128、1 MHz）: `imu start` → 10 秒 →
> `imu stop` を 4 回（書き込み直後の 1 回を含む）と 60 秒を 1 回。取得の時間は約 7.01 秒（60 秒では 57.12 秒）、
> 加速度・ジャイロは各 約 6700 Hz × 秒数、`errors`・`fifo` の各項目・`time` の異常の数はすべて 0、`i2c_dma` の
> エラーとフォールバックは 0 でした。`cpu_active` は 38.4〜38.5 %（送信なし）、`imu_stream_tid` のスタックは
> 2000 中 1024 バイトでした。

→ 「実機で（送信スタブ、ネットワークなし、6667 Hz、ウォーターマーク 128、1 MHz）、`imu start` → 10 秒 →
`imu stop` を 4 回（書き込み直後の 1 回を含む）と 60 秒を 1 回行い、`errors`・`fifo` の各項目・`time` の
異常の数と、`i2c_dma` のエラーとフォールバックが 0 であることを確かめました。」

##### 「システム時計」— `nano_mono_us()`

> 1 回は約 0.3 µs です。

→ 削除（「どこからでも呼べます。」で文を閉じた）。

##### 「システム時計」— 換算の置き換えの理由

> NuttX の汎用の換算
> （`ONESHOT_COUNT`）は 64 ビットの除算になり、`clock_systime_ticks()` 1 回に約 5.5 µs、tick から
> カウンターへの換算 1 回に約 11 µs かかるためです（置き換え後は約 1.3 µs と約 0.6 µs）。

→ 「NuttX の汎用の換算（`ONESHOT_COUNT`）は 64 ビットの除算になり遅いためです。」

##### 「システム時計」— 起床の遅れ

> - 起床の遅れ（tick の境界から `usleep()` が戻るまで）は約 17〜36 µs、`usleep(1000)` 1 回あたりの
>   CPU 時間は約 30〜52 µs です（置き換え前はそれぞれ約 51〜55 µs、約 104〜114 µs。どちらも
>   一時的な計測コードを入れた版で、1000 回の平均を TIMER で測った値。flash から実行するコードの
>   配置で XIP キャッシュの当たり方が変わり、ビルドごとにこの程度ばらつきます）。

→ 項目ごと削除。

##### 「CPU 負荷」— 実機での値

> - 実機での値（各 1〜2 回）: 何もしない 10 秒で 0（アイドルでない時間は 10 秒中 0.3〜0.4 ms）、
>   優先度の低いビジーループで 1000、10 ms ずつの動作と待ちで 499。割り込みだけが多い状況として、
>   約 196 µs ごとの wdog（毎秒 5099 回）で、ハンドラーの中身が 0 µs のとき 50、50 µs のとき 300
>   （中身だけなら 260）で、割り込み 1 回あたり約 8〜10 µs の入口・出口の処理（タイマー割り込み、wdog、
>   次のアラームの設定）も負荷に入ります。`cpu_load_get()` 1 回は約 4.8 µs です。

→ 「実機で、何もしないとき、優先度の低いビジーループ、10 ms ずつの動作と待ちの繰り返し、割り込みだけが
多い状況（短い間隔の wdog）で値を確かめました。割り込みの入口・出口の処理（タイマー割り込み、wdog、
次のアラームの設定）も負荷に入ります。」

##### 「スレッドの優先度とスタック」— 割り込み用のスタック

> 実機では、何もしていないときに割り込みの中でログを 1 行出しても、アイドルスレッドの使用量は
> 304 バイトのまま変わらず（割り込み用のスタックが無かったときは 452 から 712 バイトに増えた）、割り込み
> 用のスタックの使用量は通常 364 バイト、割り込みの中でログを出した後で 624 バイトでした。割り込み 1 回の
> コストは変わりません（毎秒約 5000 回の空の wdog で、10 秒間のアイドルでない時間が有無とも 500.0 ms。
> 各 3 回）。

→ 「実機で、何もしていないときに割り込みの中でログを 1 行出しても、アイドルスレッドのスタックの使用量が
変わらないことを確かめました。」（割り込み 1 回のコストが変わらないという比較の結果は削除）

##### 「スレッドの優先度とスタック」— スタックの番兵

> 割り込み 1 回あたり約 1.0 µs 増えます（約 200 µs ごとの wdog、
> 毎秒 4999 回で、5 秒間のアイドルでない時間が 225.2 ms から 250.2 ms に。各 3 回）。

→ 文ごと削除。

#### nuttx/drivers/README_ja.md

##### 「NINA-W102」

> - 期限、配線、実測は `nuttx/README_ja.md` の「NINA-W102」の節にあります。

→ 「期限と配線は `nuttx/README_ja.md` の「NINA-W102」の節にあります。」（参照先の実測を削除したため）

### T15 NINA usrsock データ系（実装済み・未 commit、実機確認が途中。2026-11-14）

- **実機の状態: USB には列挙されるがシェルが応答しない。** T15 の試験版（`socktest`＋`BOARD_RESET_ON_ASSERT=2` 入り、修正前）で、試験コマンドのブロッキング送信が戻らず nsh が待ち続けている。
  **ユーザーに USB の抜き差し（電源の入れ直し）を依頼中。** 戻ったら `nuttx/script/flash.sh`（試験版にも `bootsel` あり）。最終版は `nuttx/build/nuttx.uf2`（要再ビルド確認）、
  修正入りの試験版は `scratchpad/t15/nuttx-t15-test.uf2`、試験コマンドのソースは `scratchpad/t15/socktest_tmp/`。
- 固まった原因（修正済み、実機未確認）: 非ブロッキング送信の `EAGAIN` で `SENDTO_READY` を落としていたため、次のブロッキング送信がドライバーに届かず永久にイベント待ちになった。
- 実装: `nina_sock.c` に `udp_send`・`do_send`・`nina_sock_sendmsg`・`udp_recv`・`nina_sock_recv`・`nina_sock_fionread`、`nina_dns.c`（Zephyr `resolve()` の写し、`nina_getaddrinfo`、互換層の `zsock_getaddrinfo`）、
  `nina_poller.c` に 1 ms 再試行のタイマー。Kconfig `NANO_NINA_SEND_RETRY_MS=1`、`NANO_NINA_DNS`。ホストテスト 32 本。
- **NuttX へのパッチ `patches/nuttx/0005-usrsock-dontwait-to-daemon-and-daemon-send-timeout.patch`**（メインが読んで採用）: (1) `MSG_DONTWAIT` を要求に残す（`O_NONBLOCK` でも付ける）、
  (2) ドライバーが `-ETIME` を返したら呼び出しを `-EAGAIN` で終える。10 秒後の `EAGAIN` と UDP の即時 `EAGAIN` を作るため。
- 送信の設計: usrsock の要求 1 回で Zephyr の `do_send` ループ 1 周を**呼び出し元のスレッド**で実行。送れなければ `-EAGAIN`、1 ms＋1 tick 後に `nina_event` が `SENDTO_READY` を立てて再要求。
  10 秒期限と 100 ms 状態確認はソケットごと。FIONREAD は呼び出し元で CMD_POLL 1 回、0 か 1（T19: FIONREAD → 1 なら `recv(MSG_DONTWAIT)`）。
- メインの判断: パッチ 0005 は採用。recv／poll の最初の問い合わせが最大 20 ms 遅れる差と、TCP の `poll(POLLOUT)` が常に書き込み可である差は許容（CP4・CP5 で実害を見る）。
- 固まる前に確認できたこと（試験版、各 1 回）: 未接続時のエラー値、UDP の送受信（NINA のループバック）、peer フィルタ、DNS（数値・`localhost`・失敗時）、1 ms ごとの FIONREAD、TCP ループバックの connect／accept／送受信／2 iovec の `sendmsg`。
- **未確認（復旧後）**: 詰まった状態のブロッキング送信が約 10 秒で `EAGAIN`、その間の他ソケット、EOF、相手が閉じた後の送信、回帰、最終版の書き込み、README（日英）の更新（実測値は書かない）。
- 後続への注意: T21 はソケットをブロッキングのままにする。T19 は peer フィルタで捨てられると FIONREAD 1 → `recv` が `EAGAIN`。優先度 106 より低い呼び出し元は受信後の poll の分待たされる。

### T15 の続き（2026-11-14、完了・commit 済み）と `BOARD_RESET_ON_ASSERT`

- ユーザーが USB を抜き差し → 再開 → 実機確認完了。**実機は T15 の最終版（`Nov 14 2026 07:50:31`）、応答あり、試験コマンドなし。** 上の「シェルが応答しない」は解消。
- 修正版の確認（NINA のループバック 127.0.0.1、各 1 回、Wi-Fi 未接続）: 詰まった状態のブロッキング `send` は 10.002 s で `EAGAIN`（3 回＋`sendmsg` 1 回）、その間も別ソケットの FIONREAD 389 回成功、
  止まった手順（非ブロッキングの `EAGAIN` 後のブロッキング送信）は通過、読み出し後の再送信、EOF（recv 0）、2 iovec の `sendmsg` の内容一致。相手 close 後の最初の send は成功（NINA が受け付ける。2 回目以降の `EPIPE` は未確認）。
- 実測（STATUS だけ）: `FIONREAD` 1.0〜1.5 ms、UDP `sendto` 約 1.4 ms、`recvfrom(DONTWAIT)` 約 2.5 ms、ループバック 256 KB を 3.9 s、`nina_event` スタック 544/1488、
  `cpu_active`: ソケットなし 37.8〜38.1 %、4 本開いたまま 47.6 %（20 ms ごとの問い合わせの分。CP5 で Zephyr と比べる）。`free` total 177700。
- **`CONFIG_BOARD_RESET_ON_ASSERT=2` を最終の defconfig に入れた**（ユーザー決定）。HardFault・`PANIC()`・`__assert()` をスレッドと割り込みの両方で起こし（5 種、各 1 回）、
  約 3.5 秒で自動再起動（通常起動）、再起動を繰り返さない、`dmesg` に報告が残ることを確認。`CONFIG_NDEBUG=y` なので `assert()` はコンパイル時に消える。
  **残課題: `PANIC()`／assert の報告は先頭（`Assertion failed` の行とレジスタ前半）が 4096 B の RAM ログから押し出される**（スタックの内容約 23 行が長い）。HardFault は全部読める。→ メインがスタックダンプを短くして対処する。
- 試験コマンドのハング対策: `socktest -wN`（制限時間を超えたら `boardctl(BOARDIOC_RESET)`）。**今後の試験コマンドにも必ず制限時間を持たせる**（ユーザー決定）。写しは `scratchpad/t15/socktest_tmp/`。
- ホストテスト 32 本。未確認: Wi-Fi 接続時（CP4・CP5）、`EPIPE`、`REMOTE_CLOSED`、スタック番兵による PANIC の再起動、電源断。

### T16 Wi-Fi 管理、異常停止の報告、`irqstack`（2026-11-14、完了・commit 済み）

- **小作業 A**: `# CONFIG_SCHED_DUMP_STACK is not set`（スタックの内容のダンプだけ外す。`ARCH_STACKDUMP_MAX_LENGTH` は SP がスタック外のときしか効かないため不採用）。
  PANIC・`__assert()`・HardFault（スレッド／割り込み、割り込み中の `__assert()` 以外の 5 種）で報告が先頭から読め、起動ログ 4 行も残る。報告は 12 タスクで約 2640 B、1 タスク約 141 B。
  **注意: タスクが T17〜T21 で 6 本ほど増えると約 3.8 KB で余裕 300 B 程度。**増えすぎたら再検討。
- **小作業 B**: `apps/irqstack/`。出力は Zephyr の `kernel thread stacks` の `IRQ 00` の行と同じ書式。
- **T16**: `drivers/nina/nina_net.c` は Zephyr 版と行単位で同一（先頭コメントと `<nuttx/config.h>` だけ追加）。`nina_netdev.c`（`wlan0` 登録、wapi の ioctl 変換、スキャン結果の収集）、
  互換層 `zephyr/net/{net_if,net_mgmt,wifi_mgmt}.h`＋`zephyr_net_if.c`・`zephyr_net_mgmt.c`（スレッド `net_mgmt` 119/2048、待ち行列 5、10 ms。Zephyr も同じスレッドを持つ: `.config` と記録で確認）・`zephyr_wifi_mgmt.c`、
  `wifi_profile_null.c`（T17 まで）。スレッド: `nina_scan_tid` 110/1536、`nina_connect_tid` 110/1024、`nina_monitor_tid` 109/1536。defconfig に `DRIVERS_WIRELESS`・`DRIVERS_IEEE80211`・`NETDEV_WIRELESS_IOCTL`・`WIRELESS_WAPI(_CMDTOOL)`、`NET_ARP`・`NET_ICMP` を切る。
- コマンドの対応: `wifi connect -s S -p P -k 1` → `wapi psk wlan0 P 3` の後 `wapi essid wlan0 S 1`、`wifi disconnect` → `wapi disconnect wlan0`、`wifi scan` → `wapi scan wlan0`、`wifi status` → `wapi show wlan0`＋`ifconfig wlan0`。
- 実機（未接続、subagent とメイン）: `ifconfig` に `wlan0`（MAC xx:xx:xx:xx:xx:xx）、`wapi scan wlan0` 約 10 回（3.5〜4.1 s、`rp2040-test` が毎回見える、ヒープの減りなし）、
  存在しない SSID・ダミーのパスフレーズでの接続は約 30.2 s で ETIMEDOUT（3 回）、スキャン中の計測 errors 0。スタック: scan 816/1488、connect 432/968、monitor 192/1480、net_mgmt 232/2000、割り込み用 364/2048。
  `free` total 172116、UF2 442368 B。`cpu_active` 39.9 %（T15 は 37.8〜38.1 %。原因未調査、CP5 で見る）。
- メインの判断: `net_mgmt` スレッドは残す（Zephyr と同じ）。`LIBC_FLOATINGPOINT` は入れない（スキャンの周波数は `*float*` と出る）。wapi がエラーを表示しない点はそのまま（終了状態 1）。差分表に記録。
- 観察（未調査）: `rp2040-test`（WPA1/WPA2）がスキャンで `8000`（暗号化なし扱い）と出る。Zephyr 版の `scan_security()` の対応表のままなので Zephyr 版でも同じ見込み。CP6 で Zephyr 版に戻したときに `wifi scan` を見比べる。
- 未確認: 実際の接続（IP 取得、監視による切断検出、接続中のスキャン後の復帰）→ CP4。ホストテストなし（T16 分）。
- T17 への注意: `net_mgmt(NET_REQUEST_WIFI_CONNECT…)`・イベントのコールバック・`net_if_get_wifi_sta()` は互換層にある。`NET_REQUEST_WIFI_CONNECT_STORED` と `k_current_get()` は T17 で足す。
  `CONFIG_NANO_WIFI_PROFILE` で実体を用意すると `wifi_profile_null.c` が外れる。

### T17 資格情報・自動再接続・`wifi cred`（2026-11-14、完了・commit 済み）

- `apps/wifi/`: `wifi_profile.c`（Zephyr 版との diff は先頭コメント、ログの include、`wifi_reconnect_tid` の `const` だけ）、Zephyr の wifi_credentials ライブラリとシェルの写し
  （`wifi_credentials.c`・`_backend_settings.c`・`_shell.c`）、`settings_nvs.c`（`/dev/config` の `wifi_cred/<n>`。`kv` からは見えない・消せない）。互換層に `NET_REQUEST_WIFI_CONNECT_STORED`、`k_current_get()`、`shell_warn`、settings。
  `subcmd` に Zephyr のヘルプ整形（字下げ・80 桁折り返し）。スレッド `wifi_reconnect_tid` 110/2048。init の最後に `wifi_profile_init()`。
- **`wifi cred list` はパスフレーズを表示する**（Zephyr のシェルと同じ。Telnet 越しでも出る）。ログ・`dmesg`・エラー文には出ない。
- 実機（ダミーの資格情報だけ、本物の AP には未接続）: コマンドの全エラー経路と境界、最大 2 件（3 件目 -105）、再起動後の保持、再接続の間隔（最初 4.09 s、以後 32.015 s、各回約 30 s で -110）、
  `wapi disconnect wlan0` 後は再接続しない、空なら何もしない、試行中の計測 errors 0、`cpu_active` 37.8〜37.9 %、`wifi_reconnect_tid` スタック 1176/1992、`free` total 169860。
  PANIC の報告は 13 タスクで 2785 B（余裕約 1.3 KB）、先頭から読める。ホストテスト 33 本。
- `script/kv_transfer.py restore` は無変更で NuttX 版に対して動く（ダミーで確認: `wifi cred add -s <SSID> -k 1 -p <pw>` と `wifi cred auto_connect`）。
- メインの判断: パスフレーズ表示は Zephyr どおり残す（ユーザーに報告）。getopt の差（語の並べ替えなし、`<wrn> sys_getopt` なし）は許容。置き場所 `apps/wifi/` でよい。旧形式の移行コードは写しのまま残す。
- **実機の状態: T17 の最終版（`Nov 14 2026 09:02:36`）。資格情報は空（ダミーは削除済み）。KV は `imu.port`・校正 2 つ。**
- **次: ユーザーに `script/kv_transfer.py restore --usb --no-kv`（パスワードは標準入力）を実行してもらい、接続を確認（CP4 の前半）→ T18。**
- T18 への注意: `wifi cred list` は Telnet でもパスフレーズを出す。組み込みコマンドの静的変数はセッション間で共有。タスク 1 本で PANIC の報告が約 141 B 増える。

### CP4 前半: 本物の AP への接続（2026-11-14、メインが実施、各 1 回）

- ユーザーが `script/kv_transfer.py restore --usb --no-kv` を実行（パスワード入力）。**実機に `rp2040-test` の資格情報が 1 件入っている（WPA2-PSK）。以後 `wifi cred list` の出力を記録・報告に写さない（パスフレーズが出る）。**
- 接続: `ifconfig` で `wlan0` RUNNING、10.42.0.97／ゲートウェイ 10.42.0.1、`wapi show` に ESSID・AP・Sense -22。PC からの ping 3/3（約 4〜7 ms）。
- `reboot` 後 約 14 秒以内に自動接続。接続中の `wapi scan wlan0`（10 件）で一旦 0.0.0.0 になり、12 秒後には復帰。`wapi disconnect wlan0` の後 48 秒間再接続しない。`wifi cred auto_connect` で 12 秒以内に復帰。
- `dmesg` に `<err>` なし（今回の起動分）。
- 未確認: 監視による切断検出（AP 側を落とす試験はユーザーの AP に触れるので行わない。`PC側は専用アプリ予定` のメモにある UGREEN AP を落とす方法はユーザーに依頼が必要）、長時間接続。

### T18 Telnet・iperf と CP4（2026-11-14、完了・commit `0edec80`）

- `apps/init/nano_telnet.c`: 受け付けタスク `telnetd`（104/2048、Zephyr の `net_socket_service`）。init が `USB console started` の前に待ち受けを開く（ANY:23、backlog 1、REUSEADDR なし）。
  NuttX の Telnet ドライバー（`NETDEV_TELNET`、`TELNET_MAXLCLIENTS=1`）＋ USB と同じ `nano_nsh_session`（`nsh_telnet` 104/3072）。2 本目は `-ENFILE` → 閉じて `Telnet client already connected.`。
  **`NSH_TELNET`・nuttx-apps の telnetd は不採用**（2 本目で待ち受けごと終了、`SO_REUSEADDR` が ENOPROTOOPT、挨拶文）。telnetd の fd 0/1/2 は `/dev/null` で埋める。
- `dmesg`・`irqstack` のスタック 1024 → 3072（Telnet では出力が呼び出し元のスタックで NINA の SPI まで進み、1024 であふれて PANIC した）。他のコマンドは元から 3072。
- iperf（iperf 2、ポート 5001、PC は `/usr/bin/iperf`）: `NETUTILS_IPERF`、104/3072。`setup.sh` が argtable3 を clone。サーバーは `-B 0.0.0.0` 必須、UDP は `-l 512` 以下。pthread の優先度 100・101 は固定のまま。
- パッチ: `nuttx/0006`（Telnet の改行 CR LF）、`apps/0005`（iperf の argtable 解放漏れ 712 B/回）、`apps/0006`（`-l` と UDP の EAGAIN 再送）。
- CP4（subagent、条件は報告どおり）: USB と Telnet で 19 コマンドの出力が一致、2 本目 3/3、5 秒間隔 10 回成功（ヒープ・fd 増なし）、0.45 秒間隔 20 回と待ちなし 20 回も成功、
  Telnet 接続中の Wi-Fi 切断でセッション終了・復帰後に新規接続可、`board.py` 無変更で `find()`・`Shell(ip=)` が動く、iperf TCP 送信 約 3.7・受信 約 1.0 Mbit/s、UDP 送信 1.98 Mbit/s（`-l 512`）。
  計測中: 待ち受けなし 39.5 %、待ち受けのみ 44.0 %、Telnet 接続中 45.4 %、iperf 同時 86.1 %（errors・fifo 0）。スタック: `nsh_telnet` 2296/3024、`telnetd` 872/2008。
- メインの再確認（1 回）: ホストテスト 33 本、パッチ記録一致、Telnet 1 接続で `imu status`・`uname`・`dmesg`・`irqstack`・`free`・`ps`、2 本目は即 EOF＋ログ。README に NuttX 版だけの実測値なし。
- **残る差分（ユーザーに報告、変更は求められたら）**: Telnet はエコーあり・接続直後にプロンプト・`IAC WILL/WONT ECHO` を送る（Zephyr はエコーなし・交渉なし・入力後にプロンプト）。`board.py`・ImuStudio は読み捨てるので影響なしの見込み。
  accept 失敗時に 100 ms 待つ。Telnet の `cat` は Ctrl-C で終わらない。iperf 2 と zperf（iperf 3）の違い、UDP の速さ指定なし。
- **懸念（CP5 で扱う）**: (1) ソケット 1 本で計測中の `cpu_active` が約 +4.5 ポイント（MQTT・NTP の分が乗る）。(2) TCP 受信が 512 B ごとに `nina_event` の問い合わせを挟み約 1.0 Mbit/s（Zephyr の記録 1.54）。
  案: 512 B 満杯で読めたら `RECVFROM_AVAIL` を保つ。(3) **PANIC の報告は 18 タスクで 3577 B、余裕約 200 B**。T19・T21 のスレッド 2 本（約 141 B ずつ）で、Telnet・iperf・コマンドが全部動いていると先頭が押し出され得る → T21 で `dump_tasks` の行を減らすなどを検討。
- 未確認: ImuStudio の GUI、本物の `telnet` クライアント、AP 側を落としたときの切断、長時間、`accept` 待ち中の `iperf -a`。
- **実機の状態: T18 の最終版（`Nov 14 2026 10:11:41`）、Wi-Fi 接続済み 10.42.0.97、資格情報 1 件、KV は `imu.port`・校正 2 つ。**
- **次: T19（時刻同期、`ntp status`）を subagent に依頼。**

### ユーザーの決定（2026-11-14、T18 の後）

- **PANIC の報告があふれる件は、RAM ログを 4096 B → 8 KB に広げる**（案 A）。T19 の subagent に追加で依頼済み（T19 の commit に含める）。Zephyr 版のログの大きさを合わせるかは、移植完了後に `RESET_ON_FATAL_ERROR` を Zephyr 版へ反映するときに相談する。
- Telnet のエコー、iperf のパッチ（apps/0006）採用、iperf の pthread の優先度 100・101 は、メインの既定（現状のまま）で進める（ユーザーから変更の指示なし）。

### T19 時刻同期・`ntp status`＋RAM ログ 8 KB（2026-11-14、完了・commit `c68cd71`）

- `apps/imu/time_sync.c`: Zephyr 版との diff は 4 か所（先頭の include と `CONFIG_NANO_NINA_NTP_*` の対応、devicetree の include 削除、`time_sync_mono_us()` → `nano_mono_us()`、末尾のシェル登録 → `subcmd`）。本体は無変更。
  `apps/ntp/`（104/3072）。互換層: `zephyr/sys/clock.h`、`zsock_socket/sendto/close/recv`。**`zsock_recv(ZSOCK_MSG_DONTWAIT)` は `ioctl(FIONREAD)` → 1 なら `recv`**（Zephyr のドライバーが毎回 CMD_POLL するのと同じ並び）。ドライバーは無変更。
  スレッド `time_sync_tid` 109/2048（使用 1224）。`NANO_TIME_SYNC`（既定 y、無ければスタブ）。ホストテスト 34 本。
- RAM ログ 8192 B（ユーザー決定）。マジックに大きさを混ぜ、旧 4 KB のリングは起動時に空にする。`dmesg` の打ち切りは 16384 B。PANIC の報告は 19 タスクで 3728 B、余裕約 4.4 KB。`free` total 163004。
- 実機（subagent、PC の chrony 相手）: 起動から約 10 秒で同期、`ntp status` の書式一致（USB・Telnet）、往復は中央値約 8 ms・使用約 6 ms（Zephyr の記録より 0.5〜1 ms 長い）、PC との差 −2.6〜+1.4 ms（USB の遅れ込みの上限、60 回）、
  `wapi disconnect` → 61 秒で無効 → `auto_connect` の 3.2 秒後に復帰、実時間時計は前にだけ進む、計測 57 秒で errors・fifo 0・`cpu_active` 46.6 %（Telnet 越しの 20 秒は 51.5 %）、30 分 1 回（918 問い合わせ全応答、ヒープ増なし）。
  `us_per_1e6_ticks` は 24864299 で Zephyr の記録と同水準（T12 の 450 ppm 差は出ず）。
- メインの再確認（1 回）: diff、ホストテスト 34 本、Telnet で `ntp status`・`ntp`（synchronized、timeouts 0）。README に NuttX 版だけの実測値なし。
- メインの判断: (1) `date` コマンドは有効にする（Zephyr 版に `CONFIG_DATE_SHELL=y`、「Zephyr 標準コマンドは NuttX 標準に任せる」の決定どおり。T21 で `NSH_DISABLE_DATE` を外す）。(2) `nina_event` が NTP のソケットも 20 ms ごとに問い合わせる差は CP5 で CPU 負荷を見て判断。(3) スレッドの起動順は現状のまま。
- 未確認: NTP サーバーを名前で指定、chrony 停止時の timeouts（sudo が要る）、AP 側の切断、送信ありでの往復と CPU、1 時間超。
- **実機の状態: T19 の最終版（`Nov 14 2026 12:08:36`）、Wi-Fi 10.42.0.97、同期済み、資格情報 1 件、KV は `imu.port`・校正 2 つ。`dmesg` に試験の PANIC の報告が残っている。**
- 注意: ユーザーが `vi nuttx/script/build.sh` を開いている（`.build.sh.swp`）。触らない・commit しない。
- 運用の反省: subagent が長い待ちを背景に回して約 55 分止まった（ユーザーの指摘で発覚）。以後の指示に「長い待ちは前景で」を入れ、通知が「背景処理待ち」のまま 15 分以上動きがなければ作業ファイルの更新時刻を見て再開させる。
- **T21（`imu_raw_mqtt.c` の結線、CP5、`date` の有効化）を subagent に依頼済み（2026-11-14 12:20 ごろ）。報告待ち → レビュー → commit → T22。**

### T21 imu_raw_mqtt と CP5（2026-11-14、レビュー通過・未 commit・実機復旧待ち）

- `apps/imu/imu_raw_mqtt.c`: Zephyr 版との diff は先頭 31 行の追加だけ（削除・変更 0 行、メインも `diff` で確認）。`NANO_IMU_RAW_MQTT`（既定 y）。`mqtt_min*.c`・`imu_raw_status.c` をビルドに組み込み。スレッド `imu_raw_mqtt_tid` 107/2048（使用 1272）。
  互換層: `k_cycle_get_32/64`（TIMER の µs）、`zsock_connect/send/sendmsg/poll`。**`zsock_poll` は POLLIN のソケットごとに `ioctl(FIONREAD)` → `poll(0)` → 何も無ければ `poll(timeout)`**。`mqtt_min_sock.c` の受信は `zsock_recv`（分割して届く CONNACK のため）。ドライバー無変更。
  `date` 有効（`# CONFIG_NSH_DISABLE_DATE is not set`）。ホストテスト 35 本。
- CP5（subagent、各 1 回、12:20:34 のビルド＝最終版と同じソース）: 書き込み直後 60 s で欠番 0・`unix_valid` 全数・逆行 0・errors/fifo 0、`cpu_active` 71.0〜71.7 %（**Zephyr 63〜64 % より約 +7 ポイント**）、
  `--drop-at`・`--stall-at`・ボード側の Wi-Fi 切断で復帰と破棄の数が受信器と整合、送信中の Telnet 8 回、10 分連続（21002 バッチ、欠番 0、ヒープ・fd 漏れなし）。送信中の NTP 往復は中央値 約 9 ms（Zephyr 5.25 ms）。`us_per_1e6_ticks` は Zephyr と同水準。
- **事故**: subagent が CPU の内訳を測る一時計測版（`sched_note_*` フック＋`nina_request` のラッパー＋`cpustat`）を書き込んだら、ボードが USB に列挙されなくなった（`device descriptor read/64, error -110`）。原因未調査。計測の変更はリポジトリから除去済み（写しは scratchpad `t21/instr_saved/`）。
  メインが 12:51 に現在のソースから `--clean` で作り直した（`nuttx/build/nuttx.uf2`、warning は RWX 1 件、ホストテスト 35 本）。
- **次: ユーザーに手動 BOOTSEL を依頼 → `nuttx/script/flash.sh` で書き込み → メインが確認（書き込み直後の計測を受信器つきで 1 回、`nina version`、`i2c_dma stats`、`ntp status`、`date`、Telnet 接続中・送信中の PANIC の報告）→ commit → CPU +7 ポイントの扱いをユーザーに相談 → T22。**
- 未確認: 上記の復旧後確認、CPU のスレッド別内訳、`date -s`、実ブローカー、AP 側の切断。
- 提案（subagent、未実装）: `nina_event` の READ の問い合わせを「待っている要求がある間」に限る（MQTT・NTP は `zsock_poll`／`zsock_recv` が自分で問い合わせるので不要）。

### T21 の復旧と commit（2026-11-14 13:00 ごろ、commit `7d7e311`）

- ユーザーが手動 BOOTSEL → メインが `flash.sh` で最終版（`Nov 14 2026 12:51:12`）を書き込み。自動接続・同期、`nina version` 3.0.2、`kv get imu.port`、`date`、`i2c_dma stats` を確認。
- **書き込み直後の最初の計測（メイン、60 s、受信器つき、1 回）**: 2097 バッチ、欠番 0、errors・fifo 0、`unix_unavailable` 0、`cpu_active` 70.9 %、`max_transfer_us` 16678、`publish_max_us` 85867、`queue_high` 3、`dmesg` に `<err>` なし。
  `us_per_1e6_ticks` 24860580（他の回は 24864xxx。書き込み直後のこの回だけ約 150 ppm 低い。センサー発振器の変動の範囲か未調査）。記録は scratchpad `rev/t21_final60.txt`。
- 未確認のまま: 送信中・Telnet 接続中の PANIC の報告（最終版に試験コマンドが無い。T19 で 19 タスク 3728 B／8192 B を確認済みで、スレッド 1 本増の約 141 B は収まる見込み）、CPU のスレッド別内訳。
- **教訓: `sched_note_*` フックを入れた計測版で USB が列挙されなくなった。以後、スケジューラーのフックを使う計測は行わない（前後の `cpu_active` の比較で判断する）。**
- **T21b（CPU 負荷の改善）を subagent に依頼**: 互換層（`zsock_*`）が使うソケット（MQTT・NTP）は Zephyr と同じく「呼び出し元だけが NINA に問い合わせる」形にし、`nina_event` の 20 ms 問い合わせの対象から外す。Telnet・iperf など NuttX の素の API のソケットは今のまま。
  メインの判断で着手（忠実性の原則「機能差分は無くす」に沿う。ユーザーには方針を報告済み、結果を見て相談）。

### T21b ソケットの問い合わせを呼び出し元だけに（2026-11-14、完了・commit `c34af5b`。ユーザーも「進めてよい」と回答）

- `drivers/include/nina_sockopt.h`: ドライバー独自のソケットオプション（レベル `NINA_SOL` 0x4e49）。`NINA_SO_CALLER_POLLS`（setsockopt。`zsock_socket()` が付ける）で `nina_event` の 20 ms 問い合わせの対象から外す（接続中だけは残る）。
  `NINA_SO_POLL`（getsockopt）は呼び出し元の文脈で CMD_POLL 1 回、Zephyr の `do_poll()` の 1 ソケット分（POLLIN/POLLOUT/POLLERR、NINA 側に無い TCP は POLLHUP）。
  `zsock_poll` は `do_poll()` と同じ（即 1 回、以後 `k_sleep(20 ms)`）。`zsock_recv` の待つ受信も 20 ms ごとの FIONREAD。待つ送信の 1 ms タイマーと connect の確認は `nina_event` に残す（スレッドだけの差）。
  **印つきソケットに素の待つ `recv()`・`poll()` は使えない**（互換層専用。文書に記載）。Telnet・iperf は変更なし。NuttX 本体へのパッチなし。ホストテスト 35 本。
- 測定（subagent、60 s、各 1 回）: `cpu_active` 変更前 70.7〜71.4 % → 変更後 67.3〜68.1 %。Zephyr の記録 63.2〜64.2 %。欠番 0。受信器の `sync_delay` p50 は 6.5〜7.0 → 5.5〜6.2 ms（Zephyr 5.25）。送信中の `ntp status` の `last_us` の中央値は約 9.3〜9.5 ms で変わらず。
  回帰（書き込み直後、drop、stall、Wi-Fi 切断、Telnet 8 回、10 分、ブローカー停止中の start、iperf、時刻の無効化と復帰）はすべて整合、`<err>` なし。
- 残り約 +3.5 ポイント: NINA への命令の数は Zephyr と揃った（送信中 CMD_POLL 46 回/s、うち 36 回/s は Telnet の待ち受けで Zephyr にもある）。候補（未確定）: SPI 1 バイトあたりの CPU 時間、usrsock の要求の重さ、RR の wdog、測り方の違い。
  **CP6 で Zephyr 版を書き戻したときに同じ条件で `cpu_active` を測って比べる（Zephyr の記録は 1 回分だけ）。それまで手を入れない。**
- メインがコメント 2 か所（`imu_raw_mqtt.c` の先頭、`mqtt_min_sock.c`）を新しい動作に合わせて修正（コードは無変更なので、実機の `Nov 14 2026 13:25:35` 版と動作は同じ）。
- **実機の状態: `Nov 14 2026 13:25:35`、Wi-Fi 10.42.0.97、同期済み、KV は `imu.port`・校正 2 つ、資格情報 1 件。**
- **T22（校正と `imu cal`）を subagent に依頼済み（2026-11-14）。報告待ち → レビュー → commit → T23。**

### T22 校正と `imu cal`（2026-11-14、完了・commit `9bce713`）

- `apps/imu/imu_cal.c`: Zephyr 版との diff は先頭コメント 10 行と `#include <nuttx/config.h>` だけ。スタブ `imu_cal_null.c` は削除（Kconfig の切り替えは作らない。メイン了承）。ホストテスト `imu_cal`（本物のドライバー＋偽センサー＋偽 KV）で 36 本。
- 実機（subagent、各 1 回）: 起動時に accel・gyro とも calibrated、`imu cal` の出力は `host/recordings/zephyr-reference-outputs.txt` の記録と一致、`zero`・`reload`・誤った引数・計測中の `zero` の拒否・Telnet、校正ありで |a| が 1 g に近く gyro が 0 に近い、
  60 s 計測 2 回（書き込み直後を含む）で欠番 0・errors/fifo 0。`imu-host write --board 10.42.0.97`（同じ値の書き込み）成功。校正値 2 件と `imu.port` は試験前後で一致。
- メインの再確認（1 回）: diff、ホストテスト 36 本、Telnet で `imu cal`（calibrated）・`imu cal foo`（usage）・`kv get imu.port`。
- `kv_transfer.py restore` の終了コード 1 は、`imu cal reload` が動くようになったので解消の見込み（スクリプト自体は未実行。パスワード入力と資格情報の書き換えを伴うため）。`imu cal reload` を送るのは KV を書き戻すとき（`--no-kv` では送らない）。
- 未確認: 実機での `zero` の -EAGAIN・-EBUSY、`kv delete` → `reload`（実機の値を消さないため）、電源断。
- T24 への注意: `imu-host` の USB 自動検出が NuttX のシリアル名を見つけない（`--usb <デバイス>` や `--board <IP>` なら動く）。
- **実機の状態: `Nov 14 2026 14:02:55`、Wi-Fi 10.42.0.97、同期済み、KV は `imu.port`・校正 2 つ、資格情報 1 件。**
- **ユーザーの決定（2026-11-14）: T23 は uORB ではなく、Zephyr 版の修正版 `drivers/sensor_shell/sensor_shell.c`（1196 行、Zephyr v4.5.0 の写し＋独自属性 `fifo_watermark`・`fifo_timestamp_decimation`）を互換層の上にそのまま移植する。`sensor get`・`stream`・`attr_set`・`attr_get` の構文と出力を同じにする。uORB は入れない。**
- **T23 を subagent に依頼済み。報告待ち → レビュー → commit → T24。**

### T23 `sensor` コマンド（2026-11-14、完了・commit `76bb4d3`）

- `apps/sensor/`: `sensor_shell.c`（Zephyr 版の修正版との diff は先頭の説明、`sensor get` が表示を待つ 2 行、ログの PRIu32）、`sensor_shell_stream.c`（コールバック名 1 行）、`sensor_shell.h`（一致）。コマンド表のマクロは写しのまま。
  `apps/subcmd/zshell.c`（962 行。Zephyr 形式のコマンド表の実行、ヘルプ、引数検査、ワイルドカード、端末ごとのバックエンド）。処理スレッド `sensor_shell_processing_tid` 118/3072（init が起動）。
  処理スレッドの出力は 1 行ごとに端末を open → write → close（fd を持ち続けないので Telnet の次の接続を塞がない）。互換層: `shell.h`（`struct shell`、コマンド表のマクロ）、`device.h`（デバイス一覧）、`sensor.h`、`dsp/print_format.h`。ホストテスト 37 本。
- 実機（subagent、主に各 1 回）: ヘルプ・`get`（全チャネル・指定・不明名・ワイルドカード）・`attr_set/get`（FIFO の 2 属性の正常値と不正値、`full_scale` で `imu cal` が nominal/calibrated に切り替わる）・`stream` 104/833/1666 Hz（USB・Telnet、60 s でヒープ増なし）・排他（stream 中の `attr_set` -16、`imu start` の拒否、計測中の `sensor` は -16）・
  Telnet で stream on のまま切断 → 再接続できる・stream 後の計測 2 回で欠番 0（`cpu_active` 67.5 %）。校正値と `imu.port` は試験前後で一致。UF2 554496 B、`free` total 147836。
- メインの再確認（1 回）: diff、ホストテスト 37 本、Telnet で `sensor get`（校正つきで z ≒ 9.77 m/s²）・`attr_get`・不明チャネル・`imu cal`・`ntp status`。
- メインの判断: `sensor get` が表示を待ってからプロンプトに戻る変更は受け入れる（nsh はプロンプトを描き直せない。ホスト側のプロンプト判定にも都合がよい）。
- **CP6 で Zephyr 版と比べること**: (1) 3333 Hz 以上の `sensor stream` が最初の 1 バッチの後 `Read failed: -105` で止まる（プール 2 KB を 1 バッチが占める。Zephyr でも同じか未確認）。(2) 833・1666 Hz で USB のポートを閉じると stream が終わる（104 Hz では続く。原因未調査）。(3) `sensor` の各出力の文字単位の比較（Zephyr 版の記録が無い）。(4) 値の欠けた `attr_set`。
- 残る差分: Tab 補完なし、色なし、stream 中にプロンプトを描き直さない、デバイス一覧は 3 つだけ、errno の値（ENOTSUP 134→138、ENOSYS 88→38。T24 の対応表に入れる）。
- 記録の訂正: T16 の「`LIBC_FLOATINGPOINT` は入れない」は T18 から無効（iperf が select するので有効。`%.6f` は正しく出る）。
- **実機の状態: `Nov 14 2026 15:21:48`、Wi-Fi 10.42.0.97、同期済み、属性は既定、KV は `imu.port`・校正 2 つ、資格情報 1 件。**
- **次: (a) メインが pico-sdk の取得を boot2 に必要な分だけに絞る（ユーザー依頼。sparse checkout で 31 MB → 4.0 MB、boot2 は現在のものとバイト一致を確認済み）→ (b) T24。**

### pico-sdk の部分取得（2026-11-14、メイン実施、commit 済み）

- `setup.sh` に `_nuttx_clone_sparse`（`--depth 1 --filter=blob:none --sparse` ＋ `sparse-checkout set --no-cone`）。`nuttx/pico-sdk` は 31 MB → 4.0 MB。旧ディレクトリを消して取り直し、`--clean` ビルドで boot2 が変更前とバイト一致、warning は RWX 1 件。README 日英を更新。
- 実機には書いていない（載っているのは T23 の `Nov 14 2026 15:21:48`。ソースは同じ）。
- **T24（ホストの差分ファイル、文書、errno 対応表、NuttX 版での CP6）を subagent に依頼済み。報告待ち → レビュー → commit → Zephyr 版との実機比較（KV と資格情報が消えるのでユーザーに相談してから）→ 完成時の片付け（`bootsel` 削除、Zephyr 版へ自動リセット反映）。**

### T24 ホストの差分ファイルと文書、CP6（NuttX 側）（2026-11-14、完了・commit `b75336c`）

- `host/firmware_profiles.toml`（zephyr・nuttx の `usb_glob` と `prompt`）、`host/imu_host/board.py`（`tomllib`。無ければ内蔵の Zephyr 1 件。USB は by-id 名、Telnet は最初の応答のプロンプトで判定）、`host/tests/test_board.py`（14 本）、ImuStudio はテスト 1 本追加だけ（本体無変更、`dotnet test` 49 本）。
  Zephyr 版への影響なし: 旧 `_exchange` と新版を `zephyr-reference-outputs.txt` の 96 通りで比較して一致、`zephyr/script/host_tests.sh` 16 本 ok。`script/kv_transfer.py` は無変更。
- 文書: ルート README（NuttX 版の節）、`nuttx/README`（コマンド対応表、errno 対応表、Zephyr 版との違い、作業中の表現の整理）、設計書 §11、`host/README`（各日英）。NuttX 版だけの実測値・原因未確定の事柄は入れていない。`bootsel` の記述は残っている。
- CP6（subagent、NuttX 版、各 1 回）: `write`（USB 自動・デバイス指定・Telnet 自動・`--board`、同じ値）、`record` 25 s（Telnet 自動・`--board`・`--no-board`）で欠番 0、`check`、`calib`（参照の記録で再計算が一致）。校正値と `imu.port` は試験前後で一致。
- メインの判断: `requires-python >=3.10` のまま（3.10 では NuttX の USB 自動検出が効かない旨は文書にある）。`host/tests/` は commit に含めた。ImuStudio の `EndsWith('$')` と `kv_transfer.py` の glob はそのまま。英語の呼び方（"NuttX build"／"NuttX port"）の不統一は未対応（軽微）。
- **残り**: (1) Zephyr 版との実機比較（`cpu_active` の約 +3.5 ポイント、`sensor stream` の 3333 Hz 以上と USB を閉じたとき、`sensor` の出力の文字比較、`wifi scan` の暗号化表示、`imu-host` の Zephyr 判定）。**Zephyr 版を書くと KV と資格情報が消える（退避から戻せるが、資格情報はパスワード入力が 2 回、NuttX 版へ戻すとき手動 BOOTSEL 1 回）。ユーザーに実施するか相談中。**
  (2) 完成時の片付け: `bootsel` コマンドの削除（`flash.sh`・README・`CLAUDE.md` も）、Zephyr 版へ `CONFIG_RESET_ON_FATAL_ERROR` 相当を反映（「`git diff --stat -- zephyr/` が空」を確認した後）。(3) ユーザーの手動確認（ImuStudio の GUI、AP を落とす、電源断、RESET、ケーブル）。(4) `CLAUDE.md` の更新（NuttX 版の作業上の注意）。
- **実機の状態: `Nov 14 2026 15:21:48`、Wi-Fi 10.42.0.97、同期済み、KV は `imu.port`・校正 2 つ、資格情報 1 件。**

### Zephyr 版との実機比較（2026-11-14 開始、ユーザーの指示「Zephyr版へ書き戻しましょう」）

- 手順: (1) 退避ファイル `host/recordings/kv-backup.json` の校正 2 件と `imu.port` が実機の値と一致することを確認済み（値は表示しない）。(2) メインが NuttX の `bootsel` 経由で `zephyr/build/usb-console/zephyr/zephyr.uf2`（2026-11-13 19:49 のビルド）を書く。
  (3) Zephyr 版は KV が空になるので、ユーザーに `host/.venv/bin/python script/kv_transfer.py restore --usb`（パスワード入力）を依頼。(4) メインが比較（`cpu_active` 60 s×2 以上、`sensor` の出力・stream の高 ODR・USB を閉じたとき、`wifi scan`、`imu-host` の判定、`kernel thread stacks`）。
  (5) ユーザーに手動 BOOTSEL を依頼 → NuttX 版を書く → ユーザーに再度 `kv_transfer.py restore --usb`。(6) 片付け（`bootsel` 削除、Zephyr 版へ自動リセット反映、`CLAUDE.md`）。
- **実機の状態: Zephyr 版を書き込み中／書き込み済み（下に追記）。**
- **2026-11-14 16 時ごろ: Zephyr 版（4.5.0、`zephyr.uf2`）を書き込み済み。KV は空（`kv get imu.port` が -2、`imu cal` は nominal、資格情報なし）。ユーザーに `host/.venv/bin/python script/kv_transfer.py restore --usb` を依頼中。**
- Zephyr 版の `sensor`（メイン、Wi-Fi 不要の分、各 1 回、記録 scratchpad `zcmp/zephyr_sensor.txt`、手順 `zcmp/sens.py`）:
  - ヘルプ・`get`・不明チャネル・`attr_get`・不正値の `attr_set`（-22）・`sensor info`（ヘルプ）の文字列は NuttX 版と同じ（Zephyr は非同期の行の前にプロンプト `uart:~$ ` が付く。`trig` は -88＝ENOSYS、NuttX は -38）。
  - **stream が止まる件は Zephyr 版にもある（移植による退行ではない）**: 104 Hz は続く、833・1666 Hz は 1〜2 バッチの後 `Read failed: -105`、3333・6667 Hz は `Read failed: -139`（EOVERFLOW）。NuttX 版は 833・1666 Hz が続き、3333 Hz 以上で -105。
  - **USB のポートを閉じると stream が終わる件も Zephyr 版にある**: 104 Hz でも 5 秒閉じると `Read failed: -12` で終わる（NuttX 版は 104 Hz では続く）。
- 残りの比較（資格情報の登録後）: `cpu_active`（60 s×2 以上、同じスクリプト）、`wifi scan` の暗号化表示、`imu-host` の判定（USB は確認済み: `Firmware('zephyr')`）、Telnet、NTP の往復。
- **2026-11-14 16:30〜17:00 ごろ: ユーザーが Zephyr 版で `kv_transfer.py restore --usb` を実行（校正・`imu.port`・資格情報が入り、10.42.0.97 で接続・同期）。メインが Zephyr 版を計測（記録 scratchpad `zcmp/`、手順 `zcmp/meas.py`＝`imu_host.board` 経由で両ファームウェア共通）。**
  - **`cpu_active`（60 s、受信器つき、同じ日・同じ PC・同じ AP）: Zephyr 63.3 %、62.6 %、NTP を 2 秒ごとに見ながら 65.4 %、別の 30 s で 63.5 %。NuttX は 67.3〜68.1 %（T21b・T23）。差は約 +4〜5 ポイントで実在する。** どの回も欠番 0・errors/fifo 0。
    Zephyr: `max_transfer_us` 8492／26206／103210、`publish_max_us` 60722／43807／99810、`queue_high` 2／1／3、`lock_wait_max_us` 88〜200（NuttX は 350〜650）、`us_per_1e6_ticks` 24863808〜24864656。
  - 送信中の NTP: Zephyr `last_us` 27 点で最小 3956・中央値 7622・p90 11903・最大 18119 µs、`in_use_us` 5115〜5789（NuttX は中央値 約 9300〜9500、in_use 約 6100〜6600）。
  - **Zephyr のスレッド別 CPU 時間（送信中 30 s の差分、`kernel thread list` の Total execution cycles）: idle 41.85 %、`imu_raw_mqtt_tid` 20.39 %、`imu_fifo_worker` 18.53 %、`imu_stream_tid` 16.18 %、`sysworkq` 1.85 %、usbd 0.63 %、`shell_uart` 0.38 %、`time_sync_tid` 0.17 %。**（NuttX 側の内訳は未取得。スケジューラーのフックは使わない方針。）
  - zperf（10 s、1 回）: TCP 送信 3.00 Mbit/s（1024 B）、TCP 受信 1.57 Mbit/s。NuttX の iperf は送信 約 3.7〜3.9、受信 約 1.0（道具と送信サイズが違う）。
  - `wifi scan`: `rp2040-test` は Zephyr 版でも `OPEN` と表示（NuttX 版の「暗号化なし扱い」と同じ。移植による退行ではない）。スキャン後 4 s 以内に復帰。
  - `imu-host` の判定: USB・Telnet とも `Firmware('zephyr')`。旧 `board.py` と新版が Telnet で同じ応答。
  - Zephyr の Telnet が応答しなくなる現象を 1 回再現（`find()` の直後に接続した後、空の応答 → 次の接続がタイムアウト。旧 `board.py` でも同じ。`kernel reboot` で復帰）。`CLAUDE.md` に既にある現象。NuttX 版では出ていない。
- **次: ユーザーに手動 BOOTSEL を依頼 → `nuttx/script/flash.sh`（`nuttx/build/nuttx.uf2`、15:28 の `--clean` ビルド）→ ユーザーに再度 `kv_transfer.py restore --usb` → メインが NuttX 版で同じ `zcmp/meas.py` を 2〜3 回（同じ日の対比）→ CPU 差の扱いをユーザーに相談 → 片付け。**
- **実機の状態: Zephyr 版（4.5.0）、Wi-Fi 接続済み、KV 復元済み。**
- **2026-11-14 17 時ごろ: ユーザーが手動 BOOTSEL → メインが NuttX 版（`Nov 14 2026 15:28:53`、`--clean`、pico-sdk 部分取得後のビルド）を書き込み。KV は空（Zephyr 形式を消去、`kv get imu.port` -2、`imu cal` nominal）。ユーザーに `host/.venv/bin/python script/kv_transfer.py restore --usb` を依頼中。** restore の終了コード（T22 で解消見込みとした 1 → 0）もここで確かめる。
- **ユーザーの指示（2026-11-14）: ルートの README（日英）で「NuttX 版」の節を文書の最後へ移し、その後に今回測った性能の違いを載せる。** メインが実施（節「Zephyr 版と NuttX 版の性能の違い」）。これは「NuttX 版だけの実測値は README に載せない」の例外（Zephyr 版と同じ日に比べた値）。**未 commit: NuttX 版を `zcmp/meas.py` で測り直して表の NuttX 側を更新してから commit する。**
- **2026-11-14 17:30 ごろ: ユーザーが NuttX 版で `kv_transfer.py restore --usb` を実行（校正 calibrated、`imu.port`、Wi-Fi 接続・同期を確認）。メインが NuttX 版を同じ `zcmp/meas.py` で計測（記録 `zcmp/nuttx_meas.txt`）: `cpu_active` 66.0 %、66.0 %、66.1 %、NTP を見ながら 66.7 %、欠番 0。送信中の NTP `last_us` 30 点で最小 6163・中央値 8636・p90 13354 µs。**
  **同じ手順での差は約 +3 ポイント（Zephyr 62.6／63.3 %）、NTP の往復は約 +1 ms。**（T21b・T23 の 67.3〜67.6 % は別のスクリプト＝2 秒ごとの `ntp status` なしでも USB の読み方が違う。）
- ルートの README（日英）: 「NuttX 版」の節を末尾へ、その後に「Zephyr 版と NuttX 版の性能の違い」（同じ手順の値）を追加して commit 済み。
- **実機の状態: NuttX 版 `Nov 14 2026 15:28:53`、Wi-Fi 10.42.0.97、同期済み、KV 復元済み（`imu.port`・校正 2 つ）、資格情報 1 件。**
- **残り: (1) CPU 差（約 3 ポイント）の扱いをユーザーに相談。(2) 片付け: `bootsel` コマンドの削除（`flash.sh`・README・`CLAUDE.md`）、Zephyr 版へ自動リセットの反映、`CLAUDE.md` の更新。(3) ユーザーの手動確認（ImuStudio の GUI、AP 断、電源断、RESET、ケーブル）。(4) `kv_transfer.py restore` の終了コードをユーザーに確認（未回答）。**

### ユーザーの決定（2026-11-14 夕方）と T26

- `kv_transfer.py restore` はエラーなく完了（ユーザー確認）→ 「終了コード 1 の解消」は確認済み。
- **`ps` に CPU% を出す変更を正式版として入れる。それでスレッド別の内訳を見て、約 3 ポイントの差が直せそうなら直す。無理には直さず、このまま完成でもよい。**
- **`bootsel` は予定どおり削除する**（T26 の計測で書き込みを繰り返すので、削除は T26 の後、最後に行う）。
- **T26（`ps` の CPU%、内訳の比較、可能なら修正）を subagent に依頼。** Zephyr 側の内訳（送信中 30 s）: idle 41.85 %、`imu_raw_mqtt_tid` 20.39 %、`imu_fifo_worker` 18.53 %、`imu_stream_tid` 16.18 %、`sysworkq` 1.85 %、usbd 0.63 %、`shell_uart` 0.38 %、`time_sync_tid` 0.17 %。

### ユーザーの依頼（2026-11-14 夕方、T26 の作業中）

- `nuttx/drivers/README`（日英）の「wapi の ioctl」の表がレンダリングされない → リストの入れ子（字下げ 2）から出して通常の表にした（commit 済み）。ほかの文書に字下げされた表は無い。
- **`nuttx/PORTING_*.md` を作業ログとして残す（公開する）。** grep で確認: パスワード・資格情報・校正値・メールアドレスは無い。伏せるもの 2 か所: `PORTING_STATUS.md` の Wi-Fi の MAC アドレス（T16 の記録）、`PORTING_PLAN.md` にあった手元の別ツリーの絶対パス。最後の片付けで置き換えてから commit し、`CLAUDE.md` の「未追跡のまま置く」も直す。
- **`nuttx/README`（日英、約 1,420／1,625 行）を簡潔にする**: `PORTING_*.md`・設計書 §11・ドライバー説明にある内容（内部の仕組み、設計判断の理由、Zephyr の行番号との照合、スタックの根拠）は外す。残すのは環境構築・ビルド・書き込み・使い方・コマンド対応表・errno 対応表・Zephyr 版との違い・構成・パッチ一覧・テスト。README にしかない情報は `PORTING_NOTES.md` かドライバー説明へ移してから削る。T26 の後に行う（T26 が README を編集中）。`nuttx/drivers/README` は今回は対象外。

### T26 `ps` の CPU 列と内訳（2026-11-14、完了・commit 済み）

- 方式: `SCHED_CRITMONITOR`＋`SCHED_CPULOAD_CRITMONITOR`（切り替え時に perf カウンターを読んで積算。割り込みは増えない）。perf カウンターは `boards/…/src/nano_perf.c`（TIMER、1 MHz、`NANO_PERF`）。
  パッチ `nuttx/0007`（tick への切り捨てをやめ perf の単位で積算、**出ていくタスクが NULL のとき何もしない**＝ARMv6-M の終了タスクで合計が負になり全部 0.0 % になる不具合）、`nuttx/0008`（`IRQCOUNT` を CSECTION／BUSYWAIT の監視のときだけ select）。nsh の `top` は無効。
  `ps` に CPU 列、`/proc/cpuload`、`/proc/<tid>/loadavg`（約 1 秒で半減する移動平均）、`/proc/<tid>/critmon` の 2 つ目の値が起動からの積算（差分で区間の割合）。割り込みの時間は割り込まれたスレッドに入る（Zephyr の thread usage も同じ）。
- オーバーヘッド: 計測そのものは 0.1〜0.2 ポイント（実行時に止められる試験版のオン・オフ差）。**イメージの配置だけで 0.6〜0.7 ポイント動く**（使われない 2 KB を足した対照で確認。同じソースの `--clean` でも並列ビルドで配置が変わることがある）。
  最終版（`Nov 14 2026 17:55:22`）: `cpu_active` 67.7〜67.9 %（NTP を見ながら 68.3・68.5 %）、送信中の NTP 中央値 9.9〜10.0 ms。変更前のイメージは 66.0〜66.8 %、8.3〜8.9 ms。ルートの README の表は最終版の値に更新済み（変更前の値を注記）。
- 送信中 30 s の内訳（NuttX 2 回 vs Zephyr 1 回）: idle 34.6・35.2 %（41.85）、`imu_raw_mqtt_tid` 22.8・22.3（20.39）、`imu_fifo_worker` 19.1・18.8（18.53）、`imu_stream_tid` 17.4・17.7（16.18）、`nina_event` 3.2（Zephyr に無い）、`sysworkq` 2.2（1.85）。アイドル時は `nina_event` 約 2.0 %。
  差は 4 スレッドに散っていて、忠実性を保ったまま小さく直せる箇所は見つからず。**修正なし（ユーザーの意向「無理に直さない」）。** 案: 互換層のソケットが usrsock を通らずドライバーを直接呼ぶ（大掛かり）、よく通るコードを SRAM に置く。
  注意: Zephyr の 30 s の表に `net_socket_service`（Telnet の待ち受けの問い合わせ）が無い。`zcmp` の集計が拾えていない可能性（元の出力は保存していない）。
- PANIC の報告: 20 タスクで 3831 B（8192 B に収まる。タスク 1 行 148 B）。イメージ +1880 B、`free` total 147796。
- メインの再確認: `ps` の CPU 列（アイドル時 Idle 97.1 %、`nina_event` 2.1 %）、`/proc/cpuload`、ホストテスト 37 本。
- **見つかった不具合（T26 と無関係、変更前のイメージでも再現）: 起動後 Wi-Fi が繋がる前（約 5 秒）に `imu start` すると、MQTT のスレッドが動かないまま（`connects` 0・`failures` 0、全バッチ破棄）、以後の `imu start` が `Transport not ready (-16)` になり再起動まで戻らない。→ T27 として subagent に調査・修正を依頼。**
- **実機の状態: `Nov 14 2026 17:55:22`、Wi-Fi 10.42.0.97、同期済み、KV は `imu.port`・校正 2 つ、資格情報 1 件。**

### T27 Wi-Fi 接続前の `imu start` で MQTT スレッドが止まる不具合（2026-11-14、完了・commit 済み）

- 原因: 互換層の `k_mutex`。Zephyr の全部 0 の `k_mutex` は未ロック、NuttX 13.1 の全部 0 の `rmutex_t` は「スレッド 0 が保持」。ゲートウェイが無いと `open_connection()` が `mqtt_client_init()` の前に `-ENETUNREACH` で戻り、
  未初期化のクライアントのミューテックスを `mqtt_abort()` が lock して永久に待った（`failures` も増えず、`worker_active` が落ちず以後 `-EBUSY`）。T21 からあった。一度でも接続した後や `imu.host` 設定時は起きない。
- 修正: `drivers/compat/zephyr_mutex.c`（`struct k_mutex` に `ready`。印のないものは最初の lock／unlock で初期化）。ホストテスト `zephyr_mutex`（NuttX の rmutex を模した偽物。修正前は落ちる）で 38 本。全部 0 のまま使われる `k_mutex` はこの 1 つだけ（grep）。
- 実機（subagent）: 不具合の場面 3 回（Wi-Fi 後に送信開始、欠番＝Wi-Fi 前の破棄数）、切断状態からの `auto_connect`、送信中の切断・復帰、受信器なし開始、drop・stall、Telnet 5 回、iperf。`cpu_active` 66.9・67.0 %（書き込み直後 66.4 %）。
- メインの再確認（1 回）: `reboot` の 0.9 s 後に `imu start` → 4 s 後 `failures` 5 → 20 s 後 `connects` 1・送信中 → `imu stop` → `imu start` が通る、`<err>` なし。
- **実機の状態: `Nov 14 2026 18:57:03`、Wi-Fi 10.42.0.97、同期済み、KV は `imu.port`・校正 2 つ、資格情報 1 件。**
- **次: 片付け。(a) メイン: `bootsel` の削除。(b) subagent: `nuttx/README` の簡潔化。(c) メイン: Zephyr 版へ自動リセット、`PORTING_*.md` の 2 か所を伏せて commit、`CLAUDE.md`。**

### 片付け（2026-11-14 夜）

- 完了条件 5「Zephyr 版のソースに差分がない」: 移植開始の前（`80302e7~1`）から T27 の commit まで `git diff --stat -- zephyr/` は空だったことを確認。
- **Zephyr 版へ自動リセットを反映**（ユーザーの決定）: `zephyr/app/usb-console/prj.conf` に `CONFIG_RESET_ON_FATAL_ERROR=y`。ビルド成功、`.config` に反映、`zephyr/script/host_tests.sh` 16 本 ok。ルートの README（日英）のログの節と設計書 §11.6 を更新して commit 済み。
  **実機では未確認**（Zephyr 版を書くと KV と資格情報が消え、ユーザーのパスワード入力 2 回と手動 BOOTSEL が要るため）。変更前の Zephyr の UF2 は scratchpad `zcmp/zephyr-before-reset.uf2`。
- `PORTING_*.md` の公開前の処理: MAC アドレス 1 か所を `xx:…` に、手元の別ツリーの絶対パスを伏せた。
- `bootsel` の削除と `nuttx/README` の簡潔化は subagent が作業中（最終版の書き込みまで）。
- 残り: 上の subagent のレビューと commit、`PORTING_*.md` の commit、`CLAUDE.md` の更新（未追跡のまま）、ユーザーへの最終報告（手動確認の一覧）。
- **Zephyr 版の自動リセットを実機で確認（2026-11-14 19:30 ごろ、メイン、1 回）**: ユーザーが手動 BOOTSEL（subagent が先に `bootsel` なしの NuttX 最終版 `Nov 14 2026 19:05:03` を書いていたため）→ Zephyr 版（19:03 のビルド）を書き込み → 起動・シェル・`imu status`・`sensor get` → `devmem 0x30000000` で HardFault →
  `dmesg` に `***** HARD FAULT *****`・`ZEPHYR FATAL ERROR 0: CPU exception`・`Current thread: shell_uart`・`Resetting system` が残り、再起動して起動ログが続く（uptime も 0 から）。文書の「実機では未確認」を直して commit 済み。
- **実機の状態: Zephyr 版（自動リセット入り）。KV は空のはず（NuttX 形式 → Zephyr が初期化）。次: ユーザーに手動 BOOTSEL → NuttX の最終版（`nuttx/build/nuttx.uf2`。subagent の報告とレビューの後に、必要なら作り直して）を書く → ユーザーに `kv_transfer.py restore --usb` → メインが最終確認。**
- **2026-11-14 19:45 ごろ: ユーザーが手動 BOOTSEL → メインが NuttX の最終版（`Nov 14 2026 19:05:03`、`bootsel` なし）を書き込み。`bootsel` は `command not found`、KV は空（`kv get imu.port` -2）。ユーザーに `host/.venv/bin/python script/kv_transfer.py restore --usb` を依頼中。その後、メインが最終確認（60 s 計測、回帰）。**

### 完成（2026-11-14 夜）

- `bootsel` コマンドを削除（`apps/bootsel/`、`nano_reset.c` の BOOTSEL 関数、`flash.sh` の自動 BOOTSEL）。以後、書き込みは両方の版とも手動 BOOTSEL。新しい `flash.sh` の実書き込みは未確認（最終版は UF2 を直接コピーして書いた。`RPI-RP2` があるときの分岐は無変更）。
- `nuttx/README`（日英）を簡潔化（日 1458 → 429 行、英 1668 → 460 行）。各機能の動作の詳しい説明は `design/nuttx-porting-ja.md`／`nuttx-porting.md` へ移した（ユーザーの指示でこの名前に。字下げされた表 2 つを修正）。
- `CLAUDE.md` を移植完了後の内容に更新（未追跡のまま）。
- ユーザーが NuttX の最終版（`Nov 14 2026 19:05:03`）で `kv_transfer.py restore --usb` を実行。
- **最終確認（メイン、各 1 回）**: 60 s 計測（受信器つき）で 2094 バッチ・欠番 0・`unix_regressions` 0・errors/fifo 0・`unix_unavailable` 0・`cpu_active` 67.0 %。Telnet から `ps`・`nina version` 3.0.2・`i2c_dma stats`（errors 0）・`date`・`uptime`・`wapi show wlan0`・`sensor get`・`/proc/cpuload`、`dmesg` に `<err>` なし。`imu cal` は calibrated、`kv get imu.port` は 18830。
- この 3 つの作業ログ（`PORTING_PLAN.md`・`PORTING_STATUS.md`・`PORTING_NOTES.md`）を commit し、ユーザーの指示で push。
- **未確認のまま残るもの**: ImuStudio の GUI からの接続、AP 側を落としたときの切断検出と復帰、電源断、RESET ボタン、USB ケーブルの抜き差し、1 時間を超える連続動作、実際のブローカー、新しい `flash.sh` での書き込み、README のコマンド例のうち `iperf`・`i2c get`・`xd`（書き直し後の照合）。
- **残る差**: 送信中の `cpu_active` が Zephyr 版より約 4〜5 ポイント高い（原因未特定。スレッド別の内訳はルートの README）、TCP 受信の速度、Telnet のエコー、`sensor get` の表示の時期、errno の値。
- **実機の状態: NuttX 版 `Nov 14 2026 19:05:03`、Wi-Fi 10.42.0.97、同期済み、KV は `imu.port`・校正 2 つ、資格情報 1 件。**

### 所要時間の集計（会話記録のタイムスタンプから）

対象は、依頼を受けた 2026-11-13 20:04 から push 直後の 2026-11-14 19:34 まで（23.5 時間、連続）。

| 区分 | 時間 | 割合 |
|---|---|---|
| subagent が実行中（実装と実機確認） | 1,166 分（19.4 時間） | 83 % |
| メインだけが作業（調査、計画、レビュー、commit、比較計測） | 169 分（2.8 時間） | 12 % |
| ユーザーの操作・返事待ち（subagent が動いていない間だけ） | 75 分（1.3 時間） | 5 % |
| 合計 | 1,410 分（23.5 時間） | |

タスクごとの時間（分）。各タスクの「開始から次のタスクの開始まで」を 3 つに分けた。

| 開始 | タスク | subagent | メイン | ユーザー待ち | 計 |
|---|---|---|---|---|---|
| 20:04 | 調査と計画（タスク分割まで） | – | 58 | 2 | 60 |
| 21:04 | T01・T01b・T04・T25（環境、共通ソース、KV 移行スクリプト） | 42 | 2 | 0 | 44 |
| 21:49 | T02 ボード定義とビルド | 19 | 1 | 0 | 21 |
| 22:09 | T03 USB コンソール、CP1、その修正 | 77 | 5 | 0 | 82 |
| 23:31 | T03b tickless | 40 | 1 | 0 | 41 |
| 00:12 | T03c tick 換算の高速化 | 57 | 3 | 0 | 60 |
| 01:12 | T05 時計・ログ・CPU 負荷 | 36 | 3 | 0 | 39 |
| 01:51 | T06 KV | 30 | 2 | 0 | 32 |
| 02:23 | T07 NINA の SPI | 21 | 3 | 0 | 24 |
| 02:47 | T08 I2C | 25 | 2 | 0 | 27 |
| 03:14 | T09 I2C の DMA | 31 | 2 | 0 | 33 |
| 03:47 | T10 LSM6DSO 本体 | 29 | 2 | 0 | 31 |
| 04:18 | T11 FIFO ストリーム | 56 | 3 | 0 | 59 |
| 05:17 | T12 取得サービス（CP3 を含む） | 25 | 10 | 0 | 35 |
| 05:52 | T14 ソケット制御系、T20 MQTT クライアント（並行） | 42 | 3 | 15 | 60 |
| 06:53 | T15 ソケットのデータ系 | 50 | 3 | 8 | 61 |
| 07:53 | T16 Wi-Fi 管理 | 36 | 2 | 0 | 38 |
| 08:31 | T17 資格情報 | 39 | 4 | 5 | 47 |
| 09:18 | T18 Telnet・iperf、CP4 | 56 | 3 | 0 | 58 |
| 10:17 | T19 時刻同期 | 114 | 2 | 0 | 115 |
| 12:12 | T21 MQTT 送信、CP5 | 38 | 4 | 1 | 43 |
| 12:55 | T21b ソケットの問い合わせの改善 | 53 | 2 | 0 | 54 |
| 13:50 | T22 校正 | 18 | 17 | 0 | 35 |
| 14:25 | T23 `sensor` コマンド | 62 | 3 | 0 | 65 |
| 15:30 | T24 ホストと文書、Zephyr 版との比較 | 25 | 23 | 30 | 78 |
| 16:48 | T26 `ps` の CPU 列と内訳 | 98 | 2 | 0 | 99 |
| 18:27 | T27 接続前の `imu start` の不具合 | 32 | 3 | 0 | 35 |
| 19:02 | `bootsel` 削除、README 簡潔化、Zephyr 版の確認 | 14 | 4 | 14 | 32 |

待ちが長かった場面:

| 時刻 | 待ち | 内容 |
|---|---|---|
| 06:25 → 06:39 | 14.7 分 | 手動 BOOTSEL（T14 の試験で HardFault） |
| 16:06 → 16:16 | 9.7 分 | Zephyr 版での KV の復元 |
| 07:28 → 07:36 | 7.9 分 | USB の抜き差し（T15 でシェルが塞がった） |
| 16:25 → 16:32 | 6.5 分 | NuttX 版へ戻すときの BOOTSEL |
| 15:56 → 16:02 | 5.9 分 | Zephyr 版へ書き戻すかどうかの返事 |
| 16:41 → 16:47 | 5.3 分 | KV の復元の完了と、CPU 差の扱いの返事 |
| 09:10 → 09:15 | 4.8 分 | 資格情報の登録（T17 の後） |

無駄になっていた時間:

- T19 の約 55 分: subagent が背景処理の完了待ちのまま止まっていた。表では「subagent 実行中」に入っているが、実際には何も進んでいない。ユーザーの指摘で気づいて再開させた。
- T21 の復旧: 計測版で USB が止まり、手動 BOOTSEL を依頼した。待ち自体は 1 分だったが、原因になった計測は結果を出せていない。

集計の前提:

- 「ユーザー待ち」は、メインの発言の終わりから次のユーザーの発言までの時間のうち、subagent が動いていなかった分だけ。subagent の実行中に依頼した操作（たとえば T21 の BOOTSEL）は「subagent 実行中」に入るので、実際に操作してもらった回数より少なく見える。
- 「メイン」は、subagent が動いていない間のメインの作業だけ。subagent の実行中に並行して行ったレビューや質問への回答は含まない。
- 並行したタスク（T14 と T20 など）は、重なった時間を 1 回だけ数えた。
- 最初の調査 3 本と計画は、記録の形が違って subagent の時間として拾えなかったので、まとめて「メイン」に入れた。
- push の後の作業（この集計を含む）は含まない。
