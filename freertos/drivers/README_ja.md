# ドライバーと互換層（FreeRTOS 版）

[English](README.md) | 日本語

FreeRTOS 版（`freertos/`）の内部の説明です。使い方とコマンドの一覧は [`freertos/README_ja.md`](../README_ja.md)、
設計の要約は [設計書](../../design/imu-raw-mqtt-design-ja.md) の §13 にあります。細部は各ファイルの先頭の
コメントに書いてあり、ここではファイルの場所と、Zephyr 版・NuttX 版との関係を示します。

FreeRTOS 版のソースの大半は、3 つの版で共有する `common/` にあります（取得、送信、時刻同期、校正、KV、
LSM6DSO・NINA・I2C0 のドライバー、シェルのコマンド処理）。共有ソースは Zephyr の API で書かれているので、
FreeRTOS 版は NuttX 版と同じく、Zephyr の API を Zephyr の名前のまま提供する互換層の上で動かします。互換層の
OS に依存しない部分は `common/compat/`、FreeRTOS の部分は `freertos/drivers/compat/` です。

## ディレクトリー

| 場所 | 内容 |
|---|---|
| `app/usb-console/` | 起動（`main.c`）、FreeRTOS 版だけのコマンド（`commands.c`）、カーネルの設定（`FreeRTOSConfig.h`）、優先度の対応（`priorities_os.h`）、時計（`os_clock.h`）、tick フック（`tick.c`）、CPU 負荷（`idle.c`）、異常停止（`fault.[ch]`）、割り込み用スタックの空き（`irq_stack.c`） |
| `drivers/compat/` | 互換層の FreeRTOS の部分: カーネル（`zephyr_kernel.c`・`zephyr_mutex.c`・`zephyr_os*.h`）、ネットワークのインターフェース（`zephyr_net_if.c`）、壁時計（`zephyr_sys_clock.c`）、センサーの時計、前置きヘッダー（`common_prelude.h`）、自己診断（`zephyr_selftest.[ch]`）、RP2040 の部分（`rp2040/`: GPIO バンク 0、チップ ID） |
| `drivers/shell/` | 行編集（`cli.c`）、コンソールとタスクごとの出力先（`nano_console.c`）、Pico SDK の stdio の差し替え（`nano_stdio.c`）、コマンドの表と実行（`nano_shell.c`）、Zephyr の引数の分割の写し（`shell_argv.c`）、セッションごとの getopt（`getopt_state.c`） |
| `drivers/libc/` | newlib の FILE 関数の差し替え（`stdio_wrap.c`）、newlib に無いソケット系のヘッダー（`include/`。型と定数だけ）、`inet_pton`・`inet_ntop`（`inet.c`） |
| `drivers/log/` | RAM ログ（`log_ram.c`。リングは `common/log/log_ring.c`） |
| `drivers/kv/` | KV のフラッシュの境界（`nano_flash_kv.c`）とリンカースクリプトの差し替え（`ld/`、`kv.cmake`） |
| `drivers/nina/` | NINA-W102 の SPI1 と制御線（`nano_nina.c`）、境界 `nina_os.h` の実装（`nina_os.c`）、ソケット（`nina_socket.c`）、インターフェース（`nina_netif.c`） |
| `drivers/i2c_dw/` | I2C0 と DMA のコア（`common/drivers/i2c_dw/`）の FreeRTOS の部分、自己診断 |
| `drivers/lsm6dso/` | LSM6DSOX のデバイスの生成（`lsm6dso_instance.h`）と初期化（`nano_lsm6dso.c`）、自己診断 |
| `drivers/imu/` | 取得サービス・MQTT・時刻同期（`common/imu/`）の組み込み（`imu.cmake`）、`time_sync.c` の前置き |
| `drivers/wifi/` | Wi-Fi（`common/drivers/nina/nina_net.c`、資格情報、自動再接続）の組み込みと `ifconfig` |
| `drivers/telnet/` | Telnet のシェル（`nano_telnet.c`） |
| `drivers/zperf/` | `zperf` の組み込み（本体は `common/compat/zperf_iperf3.c`・`zephyr_zperf_shell.c`） |
| `drivers/sensor_shell/` | `sensor` コマンドの FreeRTOS の部分（`sensor_shell_os.[ch]`） |
| `drivers/include/` | `app_log.h` の FreeRTOS の部分 |
| `tests/` | ホストテスト（下の「ホストテスト」） |

FreeRTOS 版だけがビルドする共有ファイル: `common/compat/zephyr_wifi_shell.c`（Zephyr の `wifi_shell.c` から
`connect`・`disconnect`・`scan`・`status` を取り出したもの）、`zephyr_zperf_shell.c`・`zperf_iperf3.[ch]`（`zperf`）、
`common/drivers/nina/nina_shell.c`（`nina version`）、`common/drivers/i2c_dw/i2c_dma_shell.c`（`i2c_dma`）、
`common/log/fault_record.[ch]`（異常停止の記録の形式）。

## Zephyr 互換層の FreeRTOS の部分

何を提供するかは `common/compat/zephyr/kernel.h` の先頭の一覧、FreeRTOS での作りは `drivers/compat/zephyr_os.h`・
`zephyr_kernel.c` の先頭のコメントにあります。Zephyr の前提のうち、次のものを再現しています。

- **ミューテックス**（`zephyr_mutex.c`）: 再帰可能、優先度継承あり。全部 0 の `k_mutex` は未ロックとして扱い、
  最初の lock で作ります（Zephyr の MQTT ライブラリーと settings がこれに頼る）。戻り値は Zephyr と同じ。
- **セマフォと `k_poll`**: Zephyr の規則（give は最も優先度の高い待ち手に渡す、`k_sem_reset()` は待ちを
  `-EAGAIN` で終える）を互換層の数で作り、待ちはタスク通知（番号 1。0 は Pico SDK とアプリ用）で行います。
  割り込みからの give は、ハンドラーの終わりで切り替えを要求し、起こしたタスクがすぐ走ります。
- **静的なスレッド**: `K_THREAD_DEFINE()` のタスクは、起動のタスク（`main`）が Zephyr と同じ順に始めます。
- **時計**: `k_uptime_*()`・`k_cycle_*()`・`time_sync_mono_us()` は RP2040 の TIMER（1 MHz）から作り、tick から
  作りません（`os_clock.h`）。

近似のまま残しているもの:

| 項目 | Zephyr | FreeRTOS 版 |
|---|---|---|
| 優先度 | 小さいほど先、協調 -16〜-1、横取り 0〜14 | `20 − Zephyr`（横取り 0〜14 が 20〜6、協調 -1 が 21）。`Tmr Svc` は 31、アイドルは 0 |
| 協調スレッド | 横取りされない | 高い優先度で近似（より高いタスクには横取りされる。NuttX 版と同じ） |
| 優先度継承 | 持ち主の連鎖をたどり、unlock ごとに戻す | 1 段だけ。持っているミューテックスをすべて離したときに戻す |
| 同じ優先度の選び方 | 横取りされたスレッドが続きを走る。時分割 20 ms（そのスレッドに切り替わってから数える） | FreeRTOS 標準: 切り替えのたびに同じ優先度の次のタスクを順に選ぶので、横取りされたタスクは同じ優先度のほかの準備できたタスクの後になる。時分割は tick フック（`tick.c`）で、20 tick ごとの決まった刻み（切り替えから数えない） |
| tick と sleep | 100 µs。相対の待ちは「tick に切り上げ + 1 tick」 | 1 ms（`configTICK_RATE_HZ` 1000、32 ビット）、同じ丸め（1 ms より短い待ちは 1〜2 ms になる）。周期の SysTick（tickless ではない） |

- 32 ビットの tick は約 49.7 日で一周します。割り込みを止めている間（フラッシュの消去）は tick が数えられず
  遅れます。`vTaskDelay()`、遅延ワーク、時分割はその分遅れ、`k_uptime_*()` は TIMER なので遅れません。
- Pico SDK の `sleep_ms()`（最後の 150 µs を空回りで待つ）は使いません。

## タスク

優先度は `common/compat/priorities.h`（Zephyr 版の値と `20 − Zephyr`）と `app/usb-console/src/priorities_os.h`
（FreeRTOS 版だけのタスク）にあります。スタックはバイト。割り込みは別のスタック（MSP、2048 バイト。`ps` の
`(interrupts)` の行）を使います。

| タスク | 優先度（Zephyr） | スタック | 内容 |
|---|---|---|---|
| `Tmr Svc` | 31（–） | 1024 | カーネルのタイマータスク。割り込みで解放された Pico SDK のロックの待ち手を起こす |
| `net_mgmt`、`sysworkq` | 21（-1） | 2048、1024 | ネットワークのイベント、システムワークキュー（LSM6DSOX の FIFO の割り込みのワーク） |
| `main` | 20（0） | 2048 | 起動の後半（下の「起動」）。終わると消える |
| `sensor_shell_processing_tid`、`RTIO WQ 0` | 20（0） | 3072、1024 | `sensor` の結果の表示、RTIO のワーク |
| `nina_scan_tid`、`nina_connect_tid`、`wifi_reconnect_tid` | 12（8） | 1536、1024、2048 | Wi-Fi のスキャン・接続・自動再接続 |
| `nina_monitor_tid`、`time_sync_tid`、`imu_stream_tid` | 11（9） | 1536、2048、2048 | 接続の監視、時刻同期、取得 |
| `imu_fifo_worker` | 10（10） | 2048 | FIFO の読み出し |
| `imu_raw_mqtt_tid` | 9（11） | 2048 | MQTT の送信 |
| `console`、`shell_telnet`、`telnet_service`、`zperf_server_tid` | 6（14） | 3072、3072、2048、2048 | USB と Telnet のシェル（コマンドはこのタスクで走る）、Telnet の受け付け、`zperf tcp download` のサーバー |
| `heartbeat` | 1（–） | 1024 | LED（GPIO6）の点滅 |
| `IDLE` | 0（–） | 1024 | アイドル。CPU 負荷を測る（`idle.c`） |

ヒープは FreeRTOS の heap_4 で 32 KiB です（`free`）。タスクのスタックは静的に確保します。

**起動**（`main.c`）: `main()` で RAM ログ、異常停止の記録の読み出し、コンソール、互換層、NINA の SPI、DMA、
I2C0、KV のフラッシュの準備を行い、タスク `main` で LSM6DSOX、NINA のインターフェース、Wi-Fi の資格情報、
Telnet、`K_THREAD_DEFINE()` のスレッド、KV、校正値、時刻同期、自動接続の順に始めます（Zephyr 版の初期化の順、
NuttX 版の起動処理の順と同じ）。

**CPU 負荷**（`idle.c`）: アイドルフックで、割り込みを禁止し、窓を開き、`WFI` で待ち、窓を閉じ、割り込みを許可
します（Zephyr 版の `CONFIG_CPU_LOAD`、NuttX 版の `up_idle()` と同じ測り方）。1 ms ごとの tick の割り込みは負荷に
入ります（Zephyr 版・NuttX 版にはこの割り込みが無い）。`ps` の CPU 列は起動からの累積で、時間の源は TIMER です。

## シェルとコンソール

- コマンドは Zephyr のシェルの API（`SHELL_CMD_REGISTER()` など）で書き、照合・ヘルプ・ワイルドカード・引数の
  数の検査は共有の `common/shell/zshell.c` が行います。コマンドの表は `nano_shell.c`、FreeRTOS 版だけのコマンドは
  `app/usb-console/src/commands.c` です。
- コンソール（`nano_console.c`）はセッションの端末（USB と Telnet）で、各タスクを 1 つのコンソールに結び付けます
  （スレッドローカルのポインター）。タスクの `printf()`・`fprintf(stderr)` はそのコンソールに出ます。結び付いて
  いないタスクの出力は USB に出ます。Pico SDK の stdio のドライバーの差し替え（`nano_stdio.c`）と newlib の FILE
  関数の `--wrap`（`drivers/libc/stdio_wrap.c`）で、出力の経路をすべてここに通します。
- USB（`main.c`）: 端末がポートを開くとプロンプトを出します。端末が読まないときは Pico SDK のドライバーが 64 バイトの
  バッファーの空きを最長 500 ms 待ち、残りを捨てます。
- Telnet（`drivers/telnet/nano_telnet.c`）: ポート 23、同時 1 接続。Zephyr の `shell_telnet.c` と同じく、接続しても
  何も送らず、Telnet のコマンドは取り除いて応答せず、エコーしません。Zephyr 版と違えた点はファイルの先頭の
  コメントにあります（2 本目を断る前に 1 本目が閉じていないか確かめ直す、新しいクライアントは空の行から、など）。
- getopt の状態はセッションごとに持ちます（`getopt_state.c`）。USB と Telnet で同時にコマンドを実行できます。

## RAM ログと異常停止

- RAM ログ（`drivers/log/log_ram.c`）は 8 KiB で、起動時に初期化しない RAM（`__uninitialized_ram`）に置きます。
  watchdog の再起動、`reboot`、RESET ボタンでは残り、電源断で消えます。`LOG_*` の行は呼び出した文脈（割り込み
  ハンドラーを含む）で整形し、時刻は TIMER から取ります。`dmesg` で表示します。
- 異常停止（`app/usb-console/src/fault.[ch]`）: `panic()`、HardFault、`configASSERT()` と `assert()`、スタックの
  あふれ（`configCHECK_FOR_STACK_OVERFLOW` 2）、ヒープ不足、`_exit()` が 1 つの出口に入り、記録（理由、PC、LR、SP、
  例外のフレーム、タスク、時刻）を RAM と watchdog の scratch レジスター 0〜3 に残して watchdog で再起動します。
  次の起動で記録を `dmesg` に書き、`fault` とコンソールのバナーで表示します。記録の形式は `common/log/fault_record.[ch]`。

## KV のフラッシュ

KV は Zephyr 版・NuttX 版と同じく、フラッシュの末尾 8 KiB（`0xffe000`〜`0xffffff`、4 KiB のセクター 2 つ）に
Zephyr の NVS と settings の形式で置きます（写しは `common/compat/subsys/`、形式の説明は
[`nuttx/drivers/README_ja.md`](../../nuttx/drivers/README_ja.md) の「KV: Zephyr の NVS と settings の写し」）。
FreeRTOS 版が実装するのは flash の境界（`common/compat/nano_flash_kv.h` の 4 つの関数、`drivers/kv/nano_flash_kv.c`）です。

- リンカースクリプト（`drivers/kv/ld/`、`kv.cmake`）が FLASH の長さを `0xffe000` にして像を領域の外に置き、
  `__kv_start`・`__kv_end` と像の終わりを `ASSERT` で確かめます。
- 書き込みと消去の要求は、`zephyr_flash_kv.c`、境界の関数、フラッシュに触れる直前（割り込み禁止の中）の 3 回、
  領域の中かを確かめます。リンカーの記号と定数が一致するまで、書き込みと消去は断ります。
- `flash_range_program()`・`flash_range_erase()` は `flash_safe_execute()` から呼び、`PICO_FLASH_ASSUME_CORE1_SAFE=1` で
  このコアの割り込みだけを止めます（コア 1 は使わない）。消去の間は割り込みが止まり、tick が遅れます。
- 領域に Zephyr の NVS でも消去済みでもないデータがあれば、消さずに `dmesg` に出し、KV の操作は `-EILSEQ`
  （newlib では 138）を返します。消すのは `kv wipe confirm` です（`common/compat/kv_store_wipe.c`）。

## NINA-W102、ソケット、Wi-Fi

- SPI の転送層・ソケットの表と命令・名前解決は共有のコア（`common/drivers/nina/nina_spi.c`・`nina_sock.c`・
  `nina_dns.c`）です。FreeRTOS 版は境界 `nano_nina.h`（`drivers/nina/nano_nina.c`: SPI1 をポーリングで 7.8125 MHz、
  CS・READY・RESET・IRQ の 4 本。Zephyr のドライバーと同じ順と値で設定）と `nina_os.h`（`nina_os.c`: 再帰・優先度継承の
  ロック、名前解決のロック、「切り上げ + 1 tick」の sleep、ms の時計）を実装します。境界の意味は
  [`nuttx/drivers/README_ja.md`](../../nuttx/drivers/README_ja.md) の「NINA-W102」の表のとおりです。
- コアの READY の待ちは、READY のエッジ割り込み（GPIO10）まで眠り、最長 10 ms ごとに READY を読み直します
  （`nina_os.c`。`nina_os.h` の任意の部分で、ビルドが `NINA_OS_WAIT_READY` で有効にする）。Zephyr 版も同じで、
  NuttX 版は 10 ms に区切らずに待ちます。
- ソケット（`drivers/nina/nina_socket.c`）: 互換層の `zsock_*()` を、10 個の fd の表を介して共有のコアに直接つなぎます。
  Zephyr 版のソケットのオフロードと同じく、呼んだタスクで NINA に問い合わせて待ちます（NuttX 版の usrsock と
  `nina_event` のような、イベントのスレッドはありません）。fd の番号は newlib の fd とは別で、`zsock_*()` だけが使います。
  newlib に無い型と定数は `drivers/libc/include/` にあり、値は Linux と同じです（`EAI_*` は Zephyr の負の値）。
- ファームウェアの追加機能（パッチ 0003〜0006。命令 0x6A のビット）: 動作は Zephyr 版と同じです（説明は
  [`zephyr/app/usb-console/drivers/README_ja.md`](../../zephyr/app/usb-console/drivers/README_ja.md) の同名の節）。
  接続中のスキャンで接続を保つ・非同期のスキャン（`nina_net.c`）、ソケットの事象での待ち（`common/drivers/nina/nina_wait.c`
  と `nina_socket.c` の待ちのループ）。事象は IRQ の線（GPIO2）の立ち上がりのエッジの割り込みで、`nina_os.c` が数を
  数え、待っている全タスクを起こします（`k_sem_reset()` のあと `k_sem_give()`。`nina_os.h` の任意の部分で、ビルドが
  `NINA_OS_EVENTS` で有効にする）。`drivers/nina/nina.cmake` の `NINA_OS_EVENTS=1` を外すと、ビット 2 があっても
  従来の間隔（20 ms、送信は 1 ms）で待ちます。TCP 送信の再試行で、全量を送り直す前に poll 命令で書き込み可能を
  確かめるのは、ファームウェアに依らない変更です。
- Wi-Fi: Zephyr 版と同じ `common/drivers/nina/nina_net.c`、資格情報（Zephyr の `wifi_credentials` の写し）、自動再接続
  （`common/imu/wifi_profile.c`）を、互換層の `net_if`・`net_mgmt`・`wifi_mgmt`（`zephyr_net_if.c`、`drivers/nina/nina_netif.c`）
  の上で動かします。`wifi` コマンドは `common/compat/zephyr_wifi_shell.c` で、このボードで使えるオプションだけを残しています。
  `wifi cred` は Zephyr の `wifi_credentials_shell.c` の無改変の写しです。`ifconfig`（`drivers/wifi/nano_ifconfig.c`）は
  NuttX 版と同じ形式で表示します。

## I2C0 と DMA、LSM6DSOX、GPIO の割り込み

- I2C0 と DMA は共有のコア（`common/drivers/i2c_dw/nano_i2c.c`・`nano_dma.c`）で、FreeRTOS 版は境界 `i2c_dw_os.h` を
  `drivers/i2c_dw/i2c_dw_os_impl.h`（セマフォ、時計、割り込み禁止）と `nano_i2c_instance.h`・`nano_dma_instance.h`
  （互換層のデバイス `i2c0` と Zephyr の I2C の API、割り込みの登録、リセット、ピン）で実装します。境界の一覧は
  [`nuttx/drivers/README_ja.md`](../../nuttx/drivers/README_ja.md) の「I2C0」の表です。
- 割り込みの優先度は Pico SDK の既定の 0x80 のままで、デバイスの割り込みはすべて同じ高さです（NuttX 版と同じ。
  Zephyr 版は 0xC0）。
- LSM6DSOX は共有のドライバー（`common/drivers/lsm6dso/`）と ST の `lsm6dso_reg.c`（`setup.sh` が `freertos/stmemsc/` に
  取得）、stmemsc の層（`common/drivers/stmemsc/`）で動かし、デバイスはボードの値で 1 つだけ作ります
  （`drivers/lsm6dso/lsm6dso_instance.h`）。
- GPIO（`drivers/compat/rp2040/nano_gpio.c`）: 互換層の `gpio0` の下の GPIO バンク 0 を、Zephyr の `gpio_rpi_pico.c` と
  同じ Pico SDK の呼び出しで操作します。割り込みは `gpio_add_raw_irq_handler_masked()` で登録し、使えるのは
  LSM6DSOX の INT1（GPIO24）、NINA の READY（GPIO10）と IRQ の線（GPIO2）だけです（ほかのピンは `-ENOTSUP`）。

## 取得、時刻同期、MQTT

取得サービス（`imu_stream.c`）、時刻同期（`time_sync.c`、`time_offset.c`）、送信（`imu_raw_mqtt.c`、`imu_raw_status.c`）、
校正（`imu_cal.c`）は `common/imu/` の Zephyr 版と同じファイルで、`drivers/imu/imu.cmake` がビルドに入れます。
MQTT のクライアントは NuttX 版と共有する `common/imu/mqtt_min.c`・`mqtt_min_sock.c` です。

- 壁時計（`drivers/compat/zephyr_sys_clock.c`）: TIMER と基準値で作り（NuttX 版と同じ作り）、newlib の
  `gettimeofday()` も同じ時計を読みます。`date` はこれを UTC で表示します。
- `time_sync.c` は Pico SDK の `TIMER_BASE` と名前が重なるので、前置き（`drivers/imu/time_sync_prelude.h`）で避けます。
- チップ ID（セッション ID の先頭 8 バイト）は `pico_get_unique_board_id()` で、Zephyr 版の `hwinfo_get_device_id()` と
  同じフラッシュの ID です（`drivers/compat/rp2040/zephyr_hwinfo.c`）。

## ビルドのオプション

`freertos/script/build.sh -DNAME=VALUE` で渡します（`--clean` するまで残ります）。

| オプション | 既定 | 内容 |
|---|---|---|
| `NANO_SELFTEST` | OFF | `selftest` コマンドを入れる（下） |

## 自己診断（`selftest`）

`-DNANO_SELFTEST=ON` のビルドにだけ入る試験用のコマンドです。計測を止め、ボードを静止させて実行します。
互換層（`drivers/compat/zephyr_selftest.c`: ミューテックス、セマフォ、sleep、時計、割り込みからの起床、`k_poll`、ワーク、
spinlock、atomic、静的なスレッド、時分割の 21 項目）、I2C0（`drivers/i2c_dw/i2c_dw_selftest.c`: 6 項目）、LSM6DSOX（`drivers/lsm6dso/lsm6dso_selftest.c`:
3 項目）の結果を `N of M tests failed` の形で表示します。

## ホストテスト

`freertos/script/host_tests.sh [-v]` が、共有ソースのテスト（`common/tests/run.sh`）に続けて、次を FreeRTOS の POSIX
ポートの上で動かします。ボードは要りません。

| テスト | 対象 |
|---|---|
| `compat` | 互換層の自己診断（`selftest` と同じ `zephyr_selftest.c`） |
| `shell` | 2 つのセッションのシェル、行編集、出力先 |
| `kv` | フラッシュの境界と KV（Pico SDK のフラッシュ関数の偽物の上） |
| `nina` | NINA の SPI と線の境界、READY の割り込み待ち、IRQ の線のエッジの待ち（2 つのタスクを 1 つのエッジで起こす、時間切れ）、カーネルの部分 |
| `i2c_dw` | I2C0 の境界とデバイス、Zephyr の I2C の API |
| `gpio` | GPIO バンク 0 の境界、`gpio0`、FIFO の割り込み |
| `imu` | アドレスの変換、`hwinfo` |
| `mqtt` | MQTT の送信（偽のブローカー）。続けて、ホストの `imu-host check` を相手にする回（`host/.venv` が無ければ skip） |
| `time_sync` | 壁時計、TIMER の読み出し、NTP のやり取り、`ntp status` |
| `socket` | `zsock_*()`、待ちのループ、オプション、poll、名前解決 |
| `wifi` | インターフェース、イベント、資格情報、再接続、`wifi`・`ifconfig`、ファームウェアのビットによるスキャン（接続を保つ、非同期、失敗、終わらない、スキャン中の切断、3.0.2） |
| `telnet` | Telnet のプロトコル、1 接続、再接続、`zperf`（ホストの python3 と iperf3。iperf3 が無ければその部分を skip） |
| `sensor_shell` | `sensor` コマンド、2 つのセッション、校正（ST の `lsm6dso_reg.c` が無ければ skip） |

## Zephyr 版・NuttX 版との違い

独自のコマンド（`imu`、`kv`、`ntp`、`nina`、`dmesg`、`i2c_dma`、`sensor`、`wifi cred`）と `zperf` の構文・出力は Zephyr 版と
同じです。

| 項目 | Zephyr 版 | NuttX 版 | FreeRTOS 版 |
|---|---|---|---|
| プロンプト | USB `uart:~$ `、Telnet `~$ ` | `nsh$ ` | `freertos:~$ ` |
| 版の表示、再起動、タスク一覧、起動からの時間 | `kernel version`、`kernel reboot`、`kernel ps`、`kernel uptime` | `uname -a`、`reboot`、`ps`、`uptime` | `version`、`reboot`、`ps`、`uptime` |
| IP アドレスの表示、日時 | `net iface`、`date get` | `ifconfig`、`date` | `ifconfig`、`date`（UTC、表示だけ） |
| `wifi` | Zephyr 標準のすべて | `wifi cred` だけ（接続は `wapi`） | `connect`（`-s`・`-p`・`-k 0/1`）、`disconnect`、`scan`、`status`、`cred` |
| 速度測定 | `zperf`（iperf3 の方式、TCP と UDP） | `iperf`（iperf2 の方式） | `zperf`（iperf3 の方式、TCP だけ） |
| シェルから BOOTSEL | なし | なし | `bootsel` |
| `sensor get` | すぐ戻り、結果はプロンプトの後に出る | 結果が出るまで最長 2 秒待つ | NuttX 版と同じ |
| Telnet に接続した直後 | 何も送らない | プロンプトを送る | 何も送らない |
| Telnet のエコー | なし | あり | なし |
| エラー番号（errno） | Zephyr の値（`EILSEQ` 138） | NuttX の値（`EILSEQ` 84） | newlib の値（Zephyr と同じ。`EILSEQ` 138） |
| 優先度 | Zephyr | `118 − Zephyr` | `20 − Zephyr` |
| デバイスの割り込みの優先度 | 0xC0 | 0x80 | 0x80 |
| カーネルの時計 | SysTick（送信中に遅れる） | TIMER の tickless | 1 kHz の SysTick（消去中に遅れる）。`k_uptime` は TIMER |
| NINA の READY | エッジ割り込み（GPIO10、10 ms ごとに読み直す） | エッジ割り込み（GPIO10） | エッジ割り込み（GPIO10、10 ms ごとに読み直す） |
| ソケット | オフロード（呼んだスレッドで待つ） | usrsock と `nina_event` | 呼んだタスクで待つ（イベントのスレッドなし） |
| RAM ログ | 4 KiB | 8 KiB | 8 KiB |
| 異常停止の後 | 再起動、`dmesg` に報告 | 再起動、`dmesg` に報告 | 再起動、`dmesg`・`fault`・バナーに記録 |
| `ifconfig` の切断中のマスク | – | 残る | 0.0.0.0 |
