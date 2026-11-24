# FreeRTOS 版のポートと、Pico SDK の上の OS で共有する部分

[English](freertos-port.md) | 日本語

このリポジトリーのファームウェアは、同じファームウェアを Zephyr・NuttX・Pico SDK + FreeRTOS・Pico SDK + ThreadX の 4 つの OS の上で
それぞれビルドしたもので、それぞれを Zephyr 版・NuttX 版・FreeRTOS 版・ThreadX 版と呼びます。この文書は、Pico SDK の上の OS の
ファームウェア（FreeRTOS 版と ThreadX 版）で共有する部分と、FreeRTOS 版（`freertos/`）のポートの内部の説明です。使い方とコマンドの
一覧は [`freertos/README_ja.md`](../freertos/README_ja.md)、各 OS のファームウェアが書くものの一覧は [`os-boundary-ja.md`](os-boundary-ja.md)、
設計の要約は [設計書](imu-raw-mqtt-design-ja.md) の §9.4 にあります。ThreadX 版の部分は [`threadx-port-ja.md`](threadx-port-ja.md) です。
細部は各ファイルの先頭のコメントに書いてあり、ここではファイルの場所と、ほかの OS のファームウェアとの関係を示します。

ソースの大半は、4 つの OS のファームウェアで共有する `common/` にあります（取得、送信、時刻同期、校正、KV、LSM6DSO・NINA・I2C の
ドライバー、シェルのコマンド処理）。共有ソースは Zephyr の API で書かれているので、FreeRTOS 版は NuttX 版・ThreadX 版と同じく、
Zephyr の API を Zephyr の名前のまま提供する層（以下「互換層」。OS に依存しない部分は `common/compat/`、FreeRTOS の部分は
`common/compat/port/freertos/`）の上で動かします。互換層とドライバーそのものの説明は、それぞれの README にあります。

| 部品 | 説明 |
|---|---|
| 互換層（カーネルの部分、ネットワークの部分、シェルのエンジンとコンソール、RAM ログ） | [`common/compat/README_ja.md`](../common/compat/README_ja.md) |
| NINA-W102（SPI、ソケット、Wi-Fi、中継モード） | [`common/drivers/nina/README_ja.md`](../common/drivers/nina/README_ja.md) |
| I2C と、その DMA 読み出し | [`common/drivers/i2c_dw/README_ja.md`](../common/drivers/i2c_dw/README_ja.md) |
| LSM6DSOX、`sensor` コマンド | [`common/drivers/lsm6dso/README_ja.md`](../common/drivers/lsm6dso/README_ja.md) |

`common/` の各モジュールは、ほかのモジュールが読むヘッダーを `include/`、自分だけが読むヘッダーを `internal/`、OS が実装する
境界のヘッダーを `port/`、OS ごとの実装を `port/<OS>/`、Pico SDK の上の OS のファームウェアで共有するもの（FreeRTOS や ThreadX ではなく
Pico SDK と互換層の上で書いたもの。その cmake のファイルとリンカースクリプトの部分も）を `port/pico_sdk/`、その仕組みを自分で
持たない OS のための部分を `port/generic/` に置きます。

## Pico SDK の上の OS で共有する部分

この章は FreeRTOS 版と ThreadX 版で共有するもので、FreeRTOS にも ThreadX にも依存しません。Pico SDK の上に新しい OS を足すときは、
この章の部分をそのまま使い、次の章（FreeRTOS 版の部分）と [`threadx-port-ja.md`](threadx-port-ja.md) に当たる部分を書きます。

### 置き場所

| 場所 | 内容 |
|---|---|
| `common/imu/port/pico_sdk/` | アプリの部分: ビルド（`usb-console.cmake`: ドライバー、インクルードのディレクトリー、SDK のオプション。`ram_code.cmake`: RAM で動かすコード。`imu.cmake`: `common/imu/` の取得サービス・MQTT・時刻同期の組み込み。`ld/`: 起動のスレッドのスタックを SCRATCH_X に）、起動の共有の段階（`usb_console.[ch]`。OS の `main()` が呼ぶ）、コマンド `version`・`free` と全コマンドの表（`commands.c`。境界は `commands_os.h`）、アイドルの 1 回分と CPU 負荷（`idle.c`。LED の点滅は `idle_blink.h`）、異常停止（`fault.[ch]`。OS の入口が呼ぶ関数と、OS が書く関数の宣言も）、割り込み用スタックの空き（`irq_stack.c`）、`app.h` |
| `common/chip/rp2040/port/pico_sdk/` | KV のフラッシュの操作を Pico SDK で（`rp2040_flash_kv_os.c`。境界は `common/chip/rp2040/port/rp2040_flash_kv_os.h`、上は RP2040 の共有の部分 `common/chip/rp2040/rp2040_flash_kv.c`）、リンカースクリプトの差し替え（`ld/`: `pico_flash_region.ld`・`section_extra_post_end.incl`）。ビルドは `common/compat/port/pico_sdk/kv.cmake` |
| `common/compat/port/pico_sdk/` | 互換層の Pico SDK の上の部分（ビルドの `compat.cmake`・`kv.cmake`・`wifi.cmake`・`telnet.cmake`・`zperf.cmake`、ソケットの登録のリンカースクリプトの部分、壁時計、GPIO バンク 0、チップ ID、コンソールの経路、stdio の差し替え、RAM ログの部分）。[`common/compat/README_ja.md`](../common/compat/README_ja.md) |
| `common/shell/port/pico_sdk/` | `sensor` コマンドの組み込み（`sensor_shell.cmake`）と `i2c` コマンドの組み込み（`i2c_shell.cmake`） |
| `common/drivers/nina/port/pico_sdk/` | NINA の SPI1・DMA・制御線・中継モードの Pico SDK の部分とビルド（`nina.cmake`）。[`common/drivers/nina/README_ja.md`](../common/drivers/nina/README_ja.md) |
| `common/drivers/i2c_dw/port/pico_sdk/` | I2C のデバイスと起動、I2C0 の DMA 読み出しの Pico SDK の部分とビルド（`i2c_dw.cmake`）。[`common/drivers/i2c_dw/README_ja.md`](../common/drivers/i2c_dw/README_ja.md) |
| `common/drivers/lsm6dso/port/pico_sdk/` | LSM6DSOX のデバイスと開始、ビルド（`lsm6dso.cmake`）。[`common/drivers/lsm6dso/README_ja.md`](../common/drivers/lsm6dso/README_ja.md) |

Pico SDK の上の FreeRTOS 版と ThreadX 版だけがビルドする共有ファイル: `common/compat/zephyr_wifi_shell.c`（Zephyr の `wifi_shell.c` から
`connect`・`disconnect`・`scan`・`status` を取り出したもの）、`common/compat/zephyr_zperf_shell.c`・`zperf_iperf3.c`・`internal/zperf_iperf3.h`（`zperf`）、
`common/compat/log/fault_record.c`・`common/compat/include/fault_record.h`（異常停止の記録の形式）、`common/shell/dmesg_shell.c`（`dmesg`）、
`common/shell/kernel_shell.c`（`id`・`uptime`・`date`・`sleep`・`devmem`・`fault`・`reboot`・`bootsel`。境界は
`common/shell/port/kernel_shell_os.h`）、`common/shell/ifconfig_shell.c`（`ifconfig`）。RAM ログ `common/compat/log/log_ram.c`（リングは
`log_ring.c`）は NuttX 版とも共有する。`i2c_dma` の `common/shell/i2c_dma_shell.c` と `nina` の
`common/shell/nina_shell.c`（`nina version`・`nina passthrough`）は Zephyr 版もビルドする。`i2c` の
`common/shell/i2c_shell.c` は、Zephyr の `drivers/i2c/i2c_shell.c` にローカルのパッチ 0004 を当てたものとバイト単位で同じ写しで
（`common/tests/run.sh` の `zephyr_i2c_shell_patched` が照合する）、NuttX 版もビルドする（Zephyr 版はツリーの中のもとのファイルをビルドする）。

### OS の側が書くもの

Pico SDK の上の OS のファームウェアは、上の共有の部分に、次のものを足して作ります（一覧と、それぞれの宣言のヘッダーは
[`os-boundary-ja.md`](os-boundary-ja.md)）。

- `<OS>/app/usb-console/`: ビルド（`CMakeLists.txt`。`common/imu/port/pico_sdk/usb-console.cmake` に OS の設定とソースを渡し、
  RAM に置く OS の部分を `RAM_CODE_OS_SOURCES` に並べる）、カーネルの設定、`src/` の起動の入口（`main.c`）、tick（`tick.c`）、
  共有のアプリの境界の OS の部分（アイドル `idle_os.h`、異常停止 `fault_os.c`、`ps` とカーネルのバージョン `commands_os.c`）
- `common/compat/port/<OS>/`: 互換層のカーネルの部分の基本の操作、優先度、ポートのヘッダー（[`common/compat/README_ja.md`](../common/compat/README_ja.md)）
- `common/shell/port/<OS>/`: 汎用のコマンドの OS の部分（`kernel_shell_os_impl.h`）
- `common/drivers/i2c_dw/port/<OS>/`・`common/drivers/nina/port/<OS>/`: ドライバーの待ちとロック
- `<OS>/script/`・`<OS>/tests/`。SDK のロックとスレッドの連携が OS のポートに無ければ、それも（ThreadX 版の `threadx/drivers/pico_sdk_interop/`）

### 起動

入口の `main()` は各 OS のディレクトリーにあり、共有の段階（`common/imu/port/pico_sdk/usb_console.h`）を次の順に呼び、あいだに
OS の部分を書きます。

1. `usb_console_early_init()`: 割り込み用スタックの塗り、RAM ログ、NINA の中継モードの要求の確認、前回の異常停止の記録。
   中継モードならここでそれを動かし、戻りません。
2. カーネルの準備（OS の部分。オブジェクトとスレッドを作る前に要るもの）。
3. `usb_console_init()`: USB の stdio とコンソール "usb"、互換層（ワークキューのスレッドを作る。走らせない）、I2C0（先にその
   読み出しのための DMA の割り込みハンドラー。I2C1 をビルドしたときはそれも）、KV の領域の範囲。
4. スレッド "main"（OS の部分。静的、優先度 `APP_PRIO_MAIN`、スタック `APP_STACK_MAIN` バイト）。本体は `usb_console_main_thread()`。
5. `usb_console_shell_start()`: スレッド "console"（USB のシェル）と、アイドルが点滅させる LED。
6. スケジューラーの開始（OS の部分。戻らない）。

2 から 6 までの共有の段階は、割り込みも時間の経過も要らないことだけをします（OS はそのあいだ割り込みを止めたままでよい）。
プロンプトは OS の名前に `:~$ ` を付けたものです（`freertos:~$ `）。

スレッド "main"（`usb_console_main_thread()`）は LSM6DSOX、NINA のインターフェースを始め、そのあと 4 つの OS のファームウェアで
共有のアプリの起動（`common/imu/app_main.h`。Zephyr 版の初期化の順）に、Pico SDK の上の OS の `wifi` コマンドと Telnet を挟んで、
Wi-Fi の資格情報、`wifi` コマンド、Telnet、`K_THREAD_DEFINE()` のスレッド（`common/imu/app_init.c` の `app_threads_start()`）、
KV、校正値、時刻同期、自動接続の順に始めます。終わると戻り、OS がスレッドを終えます。スレッド "main" のスタックは、コア 1
（起動しない）だけが使う SCRATCH_X にあります（`common/imu/port/pico_sdk/ld/`、OS の `main.c`）。

### アイドルと CPU 負荷

`common/imu/port/pico_sdk/idle.c` が、アイドルでの待ちと、アイドルの区間から測る CPU 負荷（`cpu_load_get()`）を持ちます。
割り込みを禁止し、窓を開き、`WFI` で待ち、窓を閉じ、割り込みを許可します（Zephyr 版の `CONFIG_CPU_LOAD`、NuttX 版の `up_idle()` と
同じ測り方）。待ちを起こした割り込みのハンドラーは窓を閉じたあとに動くので、負荷に入ります。動いている間は LED（GPIO6）を
500 ms ごとに切り替えます。

OS の部分 `idle_os.h` は、`idle.c` の関数を OS のアイドルにどう渡すかを、次のどちらかの形で名付けます。

- `IDLE_OS_HOOK`: 1 回分の全部（割り込みの禁止から許可まで）。OS のアイドルフックから呼ぶ（FreeRTOS 版: `vApplicationIdleHook`）。
- `IDLE_OS_ENTER`・`IDLE_OS_EXIT`: 2 つの半分（窓を開く、閉じる）。割り込みの禁止と `WFI` を OS 自身の待ちが行うとき
  （ThreadX 版: スケジューラーの待ちの `tx_low_power_enter()`・`tx_low_power_exit()`）。

### コマンド

OS と Pico SDK だけが答えられるコマンドは、`common/imu/port/pico_sdk/commands.c`（`version`・`free` と全コマンドの表。境界は
`commands_os.h`: カーネルの名前とバージョン、`ps`）と、OS の `app/usb-console/src/commands_os.c` です。`commands.c` がそれを呼び、
コマンドを登録します。汎用のコマンドは `common/shell/kernel_shell.c` で、OS に求めるもの（tick の数、2 つの再起動、前回の異常停止の
記録）は `common/shell/port/kernel_shell_os.h` が宣言し、OS の実装は `common/shell/port/<OS>/kernel_shell_os_impl.h` です。シェルの
エンジン、コンソール、Telnet は [`common/compat/README_ja.md`](../common/compat/README_ja.md) にあります。

`ps` の `(interrupts)` の行は、割り込み用スタック（MSP、2048 バイト）の使用量です（`irq_stack.c`。起動の最初に塗る）。

### RAM ログと異常停止

- RAM ログ（NuttX 版とも共有の `common/compat/log/log_ram.c`。Pico SDK の上の部分は `common/compat/port/pico_sdk/log_ram_os_impl.h`。
  大きさは `common/compat/port/generic/app_log_os.h` の `LOG_RAM_SIZE`）は Zephyr 版と同じ 4 KiB で（異常停止は数行の記録を残し、
  次の起動でそれをログに書く）、起動時に初期化しない RAM（`__uninitialized_ram`）に置きます。
  watchdog の再起動、`reboot`、RESET ボタンでは残り、電源断で消えます。`LOG_*` の行は呼び出した文脈（割り込み
  ハンドラーを含む）で整形し、時刻は TIMER から取ります。`dmesg`（`common/shell/dmesg_shell.c`）で表示します。
- 異常停止（`common/imu/port/pico_sdk/fault.[ch]`）: `panic()`、HardFault、OS のアサーションと `assert()`、OS 自身の入口（OS の
  `fault_os.c`。FreeRTOS 版はスタックのあふれのフック、ThreadX 版はスタックの検査のハンドラー）、`_exit()` が 1 つの出口に入り、
  記録（理由、PC、LR、SP、例外のフレーム、スレッド、時刻）を RAM と watchdog の scratch レジスター 0〜3 に残して watchdog で
  再起動します。次の起動で記録を `dmesg` に書き、`fault` とコンソールのバナーで表示します。記録の形式、HardFault の例外のフレームの読み取り
  （フレームが RAM の中にあるときだけ）、次の起動での RAM の記録と scratch レジスターの写しの選び方は `common/compat/log/fault_record.c`・`common/compat/include/fault_record.h`。
  `fault.c` に残るのは入口、ハンドラーの入口（`isr_hardfault`）、watchdog のレジスターと再起動です。OS が書くのは、動いている
  スレッドの名前（`fault_os_thread_name()`）と、`fault.h` の `fault_begin()`・`fault_finish()` を呼ぶ OS 自身の入口です。

### KV のフラッシュ

KV は Zephyr 版・NuttX 版と同じく、フラッシュの末尾 8 KiB（`0xffe000`〜`0xffffff`、4 KiB のセクター 2 つ）に
Zephyr の NVS と settings の形式で置きます（写しは `common/compat/subsys/`、形式の説明は [`common/compat/README_ja.md`](../common/compat/README_ja.md) の「KV: Zephyr の NVS と settings の写し」）。
flash の境界（`common/compat/port/flash_kv_os.h` の 5 つの関数）は、NuttX 版と共有の RP2040 の部分（`common/chip/rp2040/rp2040_flash_kv.c`: 検査、読み出し、ページの写し）が、
Pico SDK の上の flash の操作（`common/chip/rp2040/port/rp2040_flash_kv_os.h`。`common/chip/rp2040/port/pico_sdk/rp2040_flash_kv_os.c`: Pico SDK の `flash_range_program()`・`flash_range_erase()` を `flash_safe_execute()` で）の上で実装します。

- リンカースクリプト（`common/chip/rp2040/port/pico_sdk/ld/`、`common/compat/port/pico_sdk/kv.cmake`）が FLASH の長さを `0xffe000` にしてイメージを領域の外に置き、
  `__kv_start`・`__kv_end` とイメージの終わりを `ASSERT` で確かめます。
- 書き込みと消去の要求は、`common/compat/zephyr_flash_kv.c`、RP2040 の共有の部分、フラッシュに触れる直前（割り込み禁止の中）の 3 回、
  領域の中かを確かめます。リンカーの記号と定数が一致するまで、書き込みと消去は断ります。
- `flash_range_program()`・`flash_range_erase()` は `flash_safe_execute()` から呼び、`PICO_FLASH_ASSUME_CORE1_SAFE=1` で
  このコアの割り込みだけを止めます（コア 1 は使わない）。消去の間は割り込みが止まり、tick はそのあと TIMER に追いつきます
  （FreeRTOS 版は下の「tick」、ThreadX 版は [`threadx-port-ja.md`](threadx-port-ja.md) の「tick」）。
- 領域に Zephyr の NVS でも消去済みでもないデータがあれば、消さずに `dmesg` に出し、KV の操作は `-EILSEQ`
  （newlib では 138）を返します。消すのは `kv wipe confirm` です（`common/compat/kv_store_wipe.c`）。

### 除算器、`memcpy`・`memset`、boot2

Pico SDK の既定のままです。整数の割り算は `pico_divider` のハードウェアの除算器です。切り替えのときの除算器の扱いは OS のポートに
よります（FreeRTOS 版は下の「ほかの OS との違い」、ThreadX 版は [`threadx-port-ja.md`](threadx-port-ja.md) の「FreeRTOS 版との違い」）。
0 での割り算は Pico SDK のルーチンの結果で、4 つの OS のファームウェアで同じです。`memcpy`・`memset` は `pico_mem_ops` のブート ROM の
ルーチン、boot2 は Pico SDK のボード定義の W25Q080 用（分周 2、62.5 MHz）です。

### RAM に置くコード

フラッシュは RP2040 の 16 KiB の XIP キャッシュを通して実行され、センサーの 6667 Hz の経路と送信の経路のコードはそこに
収まりません。`common/imu/port/pico_sdk/ram_code.cmake` に並べたオブジェクトのコード（`.text*`。定数はフラッシュに残す）は RAM で
動きます。

- 互換層のカーネルの部分、ワークキュー、RTIO、メモリーブロック、センサーの時計、壁時計、`gpio0` と Pico SDK の `hardware_gpio`
- I2C0（`i2c_dw.c`。I2C1 をビルドしたときはそれも）と DMA の部分、Pico SDK の `hardware_dma`
- LSM6DSOX のドライバーと stmemsc のつなぎ（設定にだけ使う ST の `lsm6dso_reg.c` は除く）
- IMU のサービスのストリーム、バッチ、キュー、時刻付け（NTP のやり取りと校正は除く）
- MQTT、互換層のソケット、NINA のソケット・プロトコル・待ち（SPI と DMA を含む）、Pico SDK の `hardware_spi`

OS の部分は、OS の `CMakeLists.txt` が `RAM_CODE_OS_SOURCES` に足します（FreeRTOS 版・ThreadX 版とも、NINA のコアのつなぎ
`common/drivers/nina/port/<OS>/nina_os.c` だけ）。カーネルはフラッシュに置きます。

`ram_code.cmake` は Pico SDK の `default_text_excludes.incl` を、これらのオブジェクトをフラッシュの `.text` から外す写し
（ビルドディレクトリーに作る）で置き換えます。外したコードは SDK の `.data` に入り、crt0 がほかの何よりも先に RAM へ写します。
配置を確かめるときは、送信中にキャッシュのカウンターをクリアし（`devmem 0x1400000c 32 0`、
`devmem 0x14000010 32 0`。飽和するので必ずクリアする）、15 秒ほどあとに読みます（当たりとアクセスの数）。

FreeRTOS 版では、コードは RAM を約 49 KiB 使います（`.data` は 8 KiB から約 57 KiB に。RAM とフラッシュのあいだの長い分岐の中継を含む）。
その分は、newlib の `malloc()` が使える `.bss` の終わりからスタックまでの領域（`free`）から取ります。
Pico SDK の最小の 2 KiB を除いて約 89 kB が残ります。使うのはシェル（コマンドの行ごとに実行中の約 1.2 KiB と、127 文字を超える出力の行を表示する間）、
`zperf` のバッファー（アップロードの間 約 2.1 KiB、`zperf tcp download` のサーバーが動いている間 約 2.5 KiB）、NINA の中継モードの
16 KiB のバッファーです。空きが無いと `malloc()` は NULL を返し（`PICO_MALLOC_PANIC=0`。Pico SDK の既定は異常停止）、呼び出し側が
失敗します（コマンドは `out of memory` と表示し、`zperf` は -12（`ENOMEM`）で失敗する）。リンクのあとの検査（`ram_code.cmake`。マップを読む）は、並べたオブジェクトのコードが
RAM に無いとき（ビルドされなくなったソースや名前の変わったソース。存在しないソースは構成の段階で止まる）と、残りが 16 KiB より
少ないときにビルドを止めます。

Pico SDK の既定のとおり RAM にあるものもあります: `__not_in_flash_func()` の関数（フラッシュの書き込みと消去、ミューテックス、
SPI の待つ呼び出し、USB のドライバーの一部）、NINA の中継の UART の割り込み、libgcc。異常停止の経路（`common/imu/port/pico_sdk/fault.c`、
`common/compat/log/fault_record.c`）と KV のフラッシュの部分はフラッシュに残ります。XIP が止まっている間に動くのは、割り込みを
止めた状態の Pico SDK 自身の RAM の関数だけです。

### 割り込み

- 割り込みの優先度は Pico SDK の既定の 0x80 のままで、デバイスの割り込みはすべて同じ高さです（NuttX 版と同じ。
  Zephyr 版は 0xC0）。
- 共有の割り込みハンドラー: Pico SDK はすべての線のぶんを決まった数の枠に持ち（`PICO_MAX_SHARED_IRQ_HANDLERS`。USB の stdio が
  あると既定は 6）、それを超えて加えると `hard_assert()` で止まります。使っているのは 6 つです: USBCTRL_IRQ の 2 つ（TinyUSB、
  stdio_usb）、IO_IRQ_BANK0 の 2 つの生のハンドラー（`common/compat/port/pico_sdk/gpio_os_rp2040.c`、NINA の `nina_os.c`）、DMA_IRQ_1 の 2 つ（I2C0 の読み出し、
  NINA の SPI）。アプリが 8 にしています（`common/imu/port/pico_sdk/usb-console.cmake`。枠 1 つで RAM 12 バイト）。
- 割り込みはスレッドのスタックとは別のスタック（MSP、2048 バイト。`ps` の `(interrupts)` の行）を使います。

### 時刻と壁時計、チップ ID

取得サービス（`imu_stream.c`）、時刻同期（`time_sync.c`、`time_offset.c`）、送信（`imu_raw_mqtt.c`、`imu_raw_status.c`）、
校正（`imu_cal.c`）は `common/imu/` の Zephyr 版と同じファイルで、`common/imu/port/pico_sdk/imu.cmake` がビルドに入れます。
MQTT のクライアントは NuttX 版とも共有する `common/imu/mqtt_min.c`・`mqtt_min_sock.c` です。

- 壁時計（`common/compat/port/pico_sdk/zephyr_sys_clock.c`）: TIMER と基準値で作り（NuttX 版と同じ作り）、newlib の
  `gettimeofday()` も同じ時計を読みます。`date` はこれを UTC で表示します。
- `time_sync.c` は互換層の時計と同じく `common/chip/rp2040` の `rp2040_timer_us()` で TIMER を読みます。
- チップ ID（セッション ID の先頭 8 バイト）は `pico_get_unique_board_id()` で、Zephyr 版の `hwinfo_get_device_id()` と
  同じフラッシュの ID です（`common/compat/port/pico_sdk/zephyr_hwinfo.c`）。

## FreeRTOS 版の部分

### ディレクトリー

FreeRTOS 版が自分で持つファイルは `freertos/` にあり、FreeRTOS の実装は `common/compat/port/freertos/`、
`common/shell/port/freertos/`、各ドライバーの `port/freertos/` にあります。以下、`src/` は `freertos/app/usb-console/src/` です。

| 場所 | 内容 |
|---|---|
| `freertos/app/usb-console/` | ビルド（`CMakeLists.txt`: `common/imu/port/pico_sdk/usb-console.cmake` に渡す FreeRTOS の設定とソースの一覧、`RAM_CODE_OS_SOURCES`）、カーネルの設定（`FreeRTOSConfig.h`。コアが 1 つであることも確かめる）、`src/`: 起動の入口（`main.c`: `main()`、タスク "main"、スケジューラー）、tick フック（`tick.c`。tick の追いつきの判定は `common/compat/include/tick_catchup.h`）、共有のアプリの境界の FreeRTOS の部分: アイドルの 1 回分（`idle_os.h`: アイドルフック）、異常停止（`fault_os.c`: 動いているタスクの名前、スタックのあふれのフック）、コマンド（`commands_os.c`: `ps` とその実行時間の時計、カーネルのバージョン） |
| `common/compat/port/freertos/` | 互換層の FreeRTOS の部分: 汎用のカーネルの部分の基本の操作（`zephyr_kernel_os_impl.h`）、優先度の対応（`priorities_os.h`）、ポートのヘッダー（`rp2040/zephyr_os_port.h`） |
| `common/shell/port/freertos/` | 汎用のコマンドの OS の部分（`kernel_shell_os_impl.h`。`uptime` の tick の数も） |
| `common/drivers/i2c_dw/port/freertos/` | I2C のコアの待ち・時計・スリープ・割り込み禁止（`i2c_dw_os_impl.h`） |
| `common/drivers/nina/port/freertos/` | NINA のコアの FreeRTOS へのつなぎ（`nina_os.c`）と SPI の DMA の待ち（`nina_bus_dma_os_impl.h`） |
| `freertos/tests/` | ホストテスト（下の「ホストテスト」） |
| `freertos/script/` | `setup.sh`、`build.sh`、`flash.sh`、`format.sh`、`host_tests.sh` |

### 互換層とドライバーのつなぎ

互換層のカーネルの部分は、FreeRTOS が用意する基本の操作（`common/compat/port/freertos/zephyr_kernel_os_impl.h`）の上の汎用の実装
（`common/compat/kernel_generic/`）です。再現している Zephyr の前提と近似のまま残しているもの（優先度、協調スレッド、優先度継承、
同じ優先度の選び方）は [`common/compat/README_ja.md`](../common/compat/README_ja.md) にあります。I2C と NINA のつなぎは互換層を
使わず、FreeRTOS（タスク通知）と Pico SDK で直接書いてあります（[`common/drivers/i2c_dw/README_ja.md`](../common/drivers/i2c_dw/README_ja.md)、
[`common/drivers/nina/README_ja.md`](../common/drivers/nina/README_ja.md)）。

### タスク

優先度は `common/compat/include/priorities.h`（Zephyr 版の値と `20 − Zephyr`）と `common/compat/port/freertos/priorities_os.h`
（FreeRTOS 版だけのタスクと、FreeRTOS 版で測った使用量に合わせた互換層のスレッドのスタック。最大の使用量の 1.5 倍、
256 バイト以上、64 の倍数。シェルのセッションは Zephyr 版と同じ 3072）にあります。スタックはバイト。割り込みは別のスタック（MSP、2048 バイト。`ps` の
`(interrupts)` の行）を使います。

| タスク | 優先度（Zephyr） | スタック | 内容 |
|---|---|---|---|
| `Tmr Svc` | 31（–） | 384 | カーネルのタイマータスク。割り込みで解放された Pico SDK のロックの待ち手を起こす |
| `net_mgmt`、`sysworkq` | 21（-1） | 2048、384 | ネットワークのイベント、システムワークキュー（LSM6DSOX の FIFO の割り込みのワーク） |
| `main` | 20（0） | 2048 | 起動の後半（上の「起動」）。終わると消える。スタックは、コア 1（起動しない）だけが使う SCRATCH_X にある（`common/imu/port/pico_sdk/ld/`、`src/main.c`） |
| `sensor_shell_tid`、`RTIO WQ 0` | 20（0） | 2048、1024 | `sensor` の結果の表示、RTIO のワーク |
| `nina_wifi_tid`、`wifi_reconnect_tid` | 12（8） | 2048、2048 | Wi-Fi のスキャン・接続・接続の監視（1 つずつ行う）、自動再接続 |
| `time_sync_tid`、`imu_stream_tid` | 11（9） | 2048、2048 | 時刻同期、取得 |
| `imu_fifo_worker` | 10（10） | 2048 | FIFO の読み出し |
| `imu_raw_mqtt_tid` | 9（11） | 2048 | MQTT の送信 |
| `console`、`shell_telnet`、`telnet_service`、`zperf_server_tid` | 6（14） | 3072、3072、2048、2048 | USB と Telnet のシェル（コマンドはこのタスクで走る）、Telnet の受け付け、`zperf tcp download` のサーバー |
| `IDLE` | 0（–） | 256 | アイドル。CPU 負荷を測り、動いている間は LED（GPIO6）を 500 ms ごとに切り替える（`common/imu/port/pico_sdk/idle.c`。`idle_os.h` のアイドルフック） |

カーネルはヒープを持ちません（`configSUPPORT_DYNAMIC_ALLOCATION` 0）。タスク・キュー・セマフォはすべて静的です。互換層の
スレッドは、制御ブロック（136 バイト）を `K_THREAD_STACK_DEFINE()` の配列のスタックのあとに置きます（`Z_OS_THREAD_RESERVED`、
`common/compat/port/freertos/zephyr_kernel_os_impl.h`）。上の表の大きさは、それを除いたスタックです。

**起動**（`src/main.c`）: `main()` が共有の段階（上の「起動」）を順に呼びます。FreeRTOS 版ではカーネルの準備は要らず、
`usb_console_early_init()`、`usb_console_init()` のあと、タスク `main`（`xTaskCreateStatic()`）を作り、`usb_console_shell_start()` でタスク
`console` を作って、clk_sys が tick の整数倍であることを確かめてから、スケジューラー（`vTaskStartScheduler()`）を始めます。

**CPU 負荷と `ps`**: アイドルフック（`idle_os.h` の `IDLE_OS_HOOK`）で上の「アイドルと CPU 負荷」の 1 回分を行います。1 ms ごとの
tick の割り込みは負荷に入ります（Zephyr 版・NuttX 版にはこの割り込みが無い）。`ps` の CPU 列は
前回の `ps`（USB・Telnet のどちらからでも）からの区間の割合で（最初は起動から）、アイドルを含む区間の全時間に
対する割合なので行の合計は約 100 % です。見たい区間の最初と終わりに 1 回ずつ打ちます。時間の源は TIMER です。

### tick

- tick は 1 ms（`configTICK_RATE_HZ` 1000、32 ビット）の周期の SysTick です（tickless ではない）。32 ビットの tick は約 49.7 日で一周します。
- 割り込みを止めている間（フラッシュの消去・書き込み。消去 1 回で約 40〜50 ms）は SysTick の割り込みが 1 回分しか保留されず、
  残りの tick は失われます。tick フック（`src/tick.c`、判定は `common/compat/include/tick_catchup.h`）が tick ごとに TIMER と比べ、1 tick 以上遅れていれば
  SysTick を保留にし直すので、失われた tick はポートの通常のハンドラーが 1 つずつ数えます（起床、切り替えの要求、tick フックの
  互換層のタイムアウトと時分割も各 tick で動く）。tick は TIMER より先に進みません。そのため tick は、割り込みを止めている間を
  除いて、TIMER から 1 tick 以内に保たれます。`xTaskCatchUpTicks()` は使いません（タスクからしか呼べず、補った tick では
  tick フックが呼ばれないため）。`main.c` は clk_sys が tick の整数倍であることを確かめます。`k_uptime_*()` は TIMER です。
- 時分割は tick フック（`tick.c`）で、20 tick ごとの決まった刻みです（切り替えから数えない）。
- Pico SDK の `sleep_ms()`（最後の 150 µs を空回りで待つ）は使いません。

### 異常停止の FreeRTOS の部分

`src/fault_os.c` が、動いているタスクの名前と、`fault.h` の `fault_begin()`・`fault_finish()` を呼ぶスタックのあふれのフック
（`configCHECK_FOR_STACK_OVERFLOW` 2。切り替えのたびに確かめる）を持ちます。`configASSERT()` は共有の `app_assert_failed()` です。
出口と記録は上の「RAM ログと異常停止」です。

### ビルドのオプション

`freertos/script/build.sh -DNAME=VALUE` で渡します（`--clean` するまで残ります）。

| オプション | 既定 | 内容 |
|---|---|---|
| `I2C_DW_I2C1` | OFF | I2C1（デバイス `i2c1`）を入れる（[`common/drivers/i2c_dw/README_ja.md`](../common/drivers/i2c_dw/README_ja.md)） |

### ホストテスト

`freertos/script/host_tests.sh [-v]` が、共有ソースのテスト（`common/tests/run.sh`）に続けて、次を FreeRTOS の POSIX
ポートの上で動かします。ボードは要りません。テストは `freertos/tests/<名前>/` にあります。

| テスト | 対象 |
|---|---|
| `compat` | 互換層のカーネル（`common/tests/zephyr_kernel/zephyr_kernel_test.c`。22 項目: ミューテックス、セマフォ、sleep、時計、`k_poll`、ワーク、spinlock、atomic、静的なスレッド、時分割、tick の割り込みからの起床。このポートでは、実時間の確認、ハードウェアの割り込みからの起床、tick の追いつきは飛ばす） |
| `shell` | 2 つのセッションのシェル、行編集、出力先、`help` と汎用のコマンドとその出力 |
| `kv` | フラッシュの境界と KV（Pico SDK のフラッシュ関数の偽物の上） |
| `nina` | NINA の SPI と線の境界（長い転送の DMA も）と、`nina_os.h` の FreeRTOS へのつなぎ（tick 1 ms と 4 ms で 2 回ビルド）。項目は [NINA-W102 のドライバー](../common/drivers/nina/README_ja.md#テスト) |
| `i2c_dw` | I2C0 の境界とデバイス、Zephyr の I2C の API、`i2c` コマンド、つなぎ、DMA の部分とコアの DMA 読み出し（i2c0 だけと、`CONFIG_I2C_DW_I2C1` 付きの 2 回）。項目は [DesignWare I2C のコア](../common/drivers/i2c_dw/README_ja.md#テスト) |
| `gpio` | GPIO バンク 0 の境界、`gpio0`、FIFO の割り込み |
| `imu` | アドレスの変換、`hwinfo` |
| `mqtt` | MQTT の送信（偽のブローカー）。続けて、ホストの `imu-host check` を相手にする回（`host/.venv` が無ければ skip） |
| `time_sync` | 壁時計、TIMER の読み出し、NTP のやり取り、`ntp status` |
| `sockets` | 偽の実装の上の互換層のソケットの部分: fd の表、登録の順、全呼び出しの振り分け、fcntl、poll（オフロード、ほかの種類、混在）、名前解決、`k_heap` |
| `socket` | 互換層の上の NINA のソケット: fd、待ちのループ、オプション、poll、fd のロック、名前解決 |
| `wifi` | インターフェース、イベント、資格情報、再接続、`wifi`・`ifconfig`、ファームウェアのビットによるスキャン（接続を保つ、非同期、失敗、終わらない、スキャン中の切断、3.0.2） |
| `telnet` | Telnet のプロトコル、1 接続、再接続、`zperf`（ホストの python3 と iperf3。iperf3 が無ければその部分を skip） |
| `sensor_shell` | `sensor` コマンド、2 つのセッション、校正（ST の `lsm6dso_reg.c` が無ければ skip） |

`compat` のうち OS に依存しない本体は `common/tests/zephyr_kernel/`（OS の部分とプラットフォームの部分が用意するものは
`zephyr_kernel_test.h`、基本の操作の上の項目は `kernel_generic/zephyr_kernel_test_os.h`）で、FreeRTOS 版の部分は `freertos/tests/compat/` です。

### ほかの OS との違い

独自のコマンド（`imu`、`kv`、`ntp`、`nina`、`dmesg`、`i2c_dma`、`sensor`、`wifi cred`）、`i2c` と `zperf` の構文・出力は Zephyr 版と
同じです。

| 項目 | Zephyr 版 | NuttX 版 | FreeRTOS 版 | ThreadX 版 |
|---|---|---|---|---|
| プロンプト | USB `uart:~$ `、Telnet `~$ ` | `nsh$ ` | `freertos:~$ ` | `threadx:~$ ` |
| バージョンの表示、再起動、タスク一覧、起動からの時間 | `kernel version`、`kernel reboot`、`kernel ps`、`kernel uptime` | `uname -a`、`reboot`、`ps`、`uptime` | `version`、`reboot`、`ps`、`uptime` | FreeRTOS 版と同じ |
| IP アドレスの表示、日時 | `net iface`、`date get` | `ifconfig`、`date` | `ifconfig`、`date`（UTC、表示だけ） | FreeRTOS 版と同じ |
| `wifi` | Zephyr 標準のすべて | `wifi cred` だけ（接続は `wapi`） | `connect`（`-s`・`-p`・`-k 0/1`）、`disconnect`、`scan`、`status`、`cred` | FreeRTOS 版と同じ |
| 速度測定 | `zperf`（iperf3 の方式、TCP と UDP） | `iperf`（iperf2 の方式） | `zperf`（iperf3 の方式、TCP だけ） | FreeRTOS 版と同じ |
| シェルから BOOTSEL | `bootsel` | `bootsel` | `bootsel` | `bootsel` |
| `sensor get` | すぐ戻り、結果はプロンプトの後に出る | 結果が出るまで最長 2 秒待つ | NuttX 版と同じ | NuttX 版と同じ |
| Telnet に接続した直後 | 何も送らない | プロンプトを送る | 何も送らない | 何も送らない |
| Telnet のエコー | なし | あり | なし | なし |
| エラー番号（errno） | Zephyr の値（`EILSEQ` 138） | NuttX の値（`EILSEQ` 84） | newlib の値（Zephyr と同じ。`EILSEQ` 138） | newlib の値（FreeRTOS 版と同じ） |
| 優先度 | Zephyr | `118 − Zephyr` | `20 − Zephyr` | `15 − Zephyr`（ThreadX の番号は Zephyr + 16、0 が最優先） |
| デバイスの割り込みの優先度 | 0xC0 | 0x80 | 0x80 | 0x80 |
| カーネルの時計 | SysTick（送信中に遅れる） | TIMER の tickless | 1 kHz の SysTick（割り込みを止めたあと TIMER に追いつく）。`k_uptime` は TIMER | FreeRTOS 版と同じ |
| 除算器の退避 | 切り替えで DIRTY のときだけ（ローカルのパッチ） | 切り替えで DIRTY のときだけ（パッチ 0009） | 切り替えのたびに 4 語（Pico SDK のポートの `portUSE_DIVIDER_SAVE_RESTORE`） | しない（ThreadX の Cortex-M0 のポート） |
| NINA の READY | エッジ割り込み（GPIO10、10 ms ごとに読み直す） | エッジ割り込み（GPIO10） | エッジ割り込み（GPIO10、10 ms ごとに読み直す） | エッジ割り込み（GPIO10、10 ms ごとに読み直す） |
| ソケット | オフロード（呼んだスレッドで待つ） | usrsock と `nina_event` | 互換層のソケット層の上の Zephyr 版のオフロード（呼んだタスクで待つ） | FreeRTOS 版と同じ |
| RAM ログ | 4 KiB | 8 KiB | 4 KiB | 4 KiB |
| 異常停止の後 | 再起動、`dmesg` に報告 | 再起動、`dmesg` に報告 | 再起動、`dmesg`・`fault`・バナーに記録 | FreeRTOS 版と同じ |
| `ifconfig` の切断中のマスク | – | 残る | 0.0.0.0 | 0.0.0.0 |

ThreadX 版と FreeRTOS 版の細かい違い（`ps` の行、同じ優先度の選び方、スタックの検査など）は [`threadx-port-ja.md`](threadx-port-ja.md) の
「FreeRTOS 版との違い」にあります。
