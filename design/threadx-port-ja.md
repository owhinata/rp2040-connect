# ThreadX 版のポート

[English](threadx-port.md) | 日本語

このリポジトリーのファームウェアは、同じファームウェアを Zephyr・NuttX・Pico SDK + FreeRTOS・Pico SDK + ThreadX の 4 つの OS の上で
それぞれビルドしたもので、それぞれを Zephyr 版・NuttX 版・FreeRTOS 版・ThreadX 版と呼びます。この文書は ThreadX 版（`threadx/`）の
ポートの内部の説明です。使い方は [`threadx/README_ja.md`](../threadx/README_ja.md)、各 OS のファームウェアが書くものの一覧は
[`os-boundary-ja.md`](os-boundary-ja.md)、設計の要約は [設計書](imu-raw-mqtt-design-ja.md) の §9.5 にあります。細部は各ファイルの
先頭のコメントにあります。

ThreadX 版は、ThreadX のカーネル（Eclipse ThreadX v6.5.2.202603b_rel。Cortex-M0 のポート `ports/cortex_m0/gnu`）だけを使い、起動・クロック・USB・
フラッシュ・DMA は Pico SDK 2.3.1 のものです。USBX・NetX Duo・FileX は使いません。ソースの大半は 4 つの OS で共有する `common/` にあり、
Pico SDK の上の部分（`common/` の各モジュールの `port/pico_sdk/` と `port/generic/`）は FreeRTOS 版と共有します。共有の
ソースは Zephyr の API で書いてあるので、ThreadX 版は FreeRTOS 版と同じく、その API を Zephyr と同じ名前で提供する層（以下「互換層」）の上で
動かします。カーネルの部分は、OS の基本の操作の上に置く汎用の実装（`common/compat/kernel_generic/`）で、ThreadX 版が書くのはその
基本の操作です。ネットワークは互換層のソケット層（`common/compat/net_offload/`）、シェルは共有のシェルのエンジン（`common/compat/shell_engine/`）
です。Pico SDK の上の OS で共有する部分（起動の段階、アイドルの測り方、異常停止、KV のフラッシュ、RAM に置くコード）の説明は
[`freertos-port-ja.md` の「Pico SDK の上の OS で共有する部分」](freertos-port-ja.md#pico-sdk-の上の-os-で共有する部分) にあり、ここでは ThreadX の部分だけを書きます。

## ディレクトリー

以下、`src/` は `threadx/app/usb-console/src/` です。

| 場所 | 内容 |
|---|---|
| `threadx/app/usb-console/` | ビルド（`CMakeLists.txt`: 共有の `common/imu/port/pico_sdk/usb-console.cmake` に渡す設定とソース、`RAM_CODE_OS_SOURCES`。`threadx_kernel.cmake`: ThreadX のカーネル・ポート・実行時間のキットをファームウェアのソースとして加え、ボードの部分、ベクター、Pico SDK のロックの連携を足す）、カーネルの設定（`tx_user.h`）、`src/`: 起動の入口（`main.c`: `main()`、カーネルの準備、スレッド "main"、スケジューラー）、SysTick と例外の優先度（`tx_initialize_low_level.c`）、tick（`tick.c`）、共有のアプリの境界の ThreadX の部分（アイドル `idle_os.h`、異常停止 `fault_os.c`、`ps` とバージョン `commands_os.c`）、スレッドごとの実行時間（`run_time.[ch]`） |
| `common/compat/port/threadx/` | 互換層の ThreadX の部分: 汎用のカーネルの部分の基本の操作（`zephyr_kernel_os_impl.h`）、そのロックの変数（`zephyr_kernel_os.c`）、優先度と時分割（`priorities_os.h`）、ボードのポートのヘッダー（`rp2040/zephyr_os_port.h`） |
| `common/shell/port/threadx/` | 汎用のコマンドの OS の部分（`kernel_shell_os_impl.h`） |
| `threadx/drivers/pico_sdk_interop/` | Pico SDK のミューテックス・セマフォ・スリープの待ちを ThreadX のスレッドにつなぐもの（`pico_sdk_interop.[ch]`、`pico_sdk_interop.cmake`。下の「Pico SDK のロックの連携」） |
| `common/drivers/i2c_dw/port/threadx/` | I2C のコアの待ち・時計・スリープ（`i2c_dw_os_impl.h`） |
| `common/drivers/nina/port/threadx/` | NINA のコアの ThreadX へのつなぎ（`nina_os.c`）と SPI の DMA の待ち（`nina_bus_dma_os_impl.h`） |
| `threadx/tests/` | ホストテスト（`compat/`、`i2c_dw/`、`nina/`）と実機の試験用ファームウェア（`board_kernel/`） |
| `threadx/script/` | `setup.sh`（ThreadX、Pico SDK、ツールチェーン、picotool、clang-format、ST の `lsm6dso_reg.c` の取得）、`build.sh`、`flash.sh`、`format.sh`、`host_tests.sh` |

`threadx/threadx`、`threadx/pico-sdk`、`threadx/toolchain`、`threadx/picotool`、`threadx/stmemsc` は `setup.sh` が取得するもので、Git 管理外です。
ThreadX のソースは変更せずにビルドします。

## 互換層のカーネルの部分とドライバーの待ち

互換層のカーネルの部分の基本の操作（スレッドのロック、起こしの印とスタックのあとの領域、スレッドの開始と終わり、ミューテックス、
優先度と時分割、時計、アサーション）は `common/compat/port/threadx/zephyr_kernel_os_impl.h` にあり、作りは
[`common/compat/README_ja.md`](../common/compat/README_ja.md) にあります。I2C と NINA のつなぎは互換層を通さず ThreadX と Pico SDK で
直接書いてあり、待ちのオブジェクトはドライバーが持つ `TX_SEMAPHORE` です（[`common/drivers/i2c_dw/README_ja.md`](../common/drivers/i2c_dw/README_ja.md)、
[`common/drivers/nina/README_ja.md`](../common/drivers/nina/README_ja.md)）。

- ThreadX のセマフォは数と待ち手の一覧だけの軽いもので、割り込みから put できます（FreeRTOS のキューで作ったセマフォとは事情が違う）。
- ThreadX には、ドライバーが使えるスレッド自身の起こし（FreeRTOS のタスク通知の番号）がありません。スレッドごとのセマフォは互換層の
  起こしの印で、共用すると片方がもう片方の待ちを終わらせたり起こしを食べたりします。スレッドの停止と再開（`tx_thread_suspend()`・
  `tx_thread_resume()`）は、停止の前に来た再開を取りこぼすので使いません。

## tick

- SysTick を 1 ms（`TX_TIMER_TICKS_PER_SECOND` 1000）で使います。設定は `src/tx_initialize_low_level.c`（`_tx_initialize_low_level()`）で、
  `clk_sys / 1000` 周期です。`src/main.c` の `main()` は、スケジューラーを始める前に clk_sys が tick の整数倍であることを確かめます。
- 例外の優先度: PendSV と SVCall は 0xC0（最低）、SysTick は 0x80（Pico SDK がデバイスの割り込みに付ける既定と同じ）。ThreadX の Cortex-M0 の
  スケジューラーは、実行できるスレッドが無いあいだ PendSV のハンドラーの中で待つので、そこから抜けられるのは PendSV より上の割り込みだけです。
  SysTick を PendSV と同じ高さにすると、スレッドが全部眠っているときに tick が入らず、誰も起きなくなります。0xC0 にした割り込みはアイドルの
  あいだ動かないので、デバイスの割り込みはすべて 0x80 です。
- `isr_systick`（`src/tick.c`）: まず TIMER を読み、ThreadX の tick（`_tx_timer_interrupt()`: 時計、時分割、`TX_TIMER_PROCESS_IN_ISR` による
  スリープとタイムアウト）、失われた tick の追いつき、互換層のタイムアウト（`zephyr_compat_tick()`）の順に動かします。タイマーのスレッドは
  ありません。起こされたスレッドへの切り替えは、ハンドラーのあとの PendSV です。
- 追いつき: 割り込みを止めているあいだ（フラッシュの消去）に失われた tick は、共有の `common/compat/include/tick_catchup.h` が TIMER と比べて
  見つけ、1 tick ごとに SysTick を保留にし直します。SysTick は PendSV より上なので、足した tick は切り替えの前に続けて動き、それぞれが
  通常の tick として数えられます（FreeRTOS 版と同じ判定）。tick は TIMER より先に進みません。

## カーネルの開始とベクター

- 入口は `src/main.c` の `main()` です。共有の段階（[`freertos-port-ja.md` の「起動」](freertos-port-ja.md#起動)）を順に呼び、あいだに ThreadX の部分を書きます。
  1. `usb_console_early_init()`: RAM ログ、中継モードの確認と実行（割り込みは許可）、前回の異常停止の記録。
  2. `_tx_initialize_kernel_setup()`（低レベルの初期化と ThreadX の初期化。スケジューラーは始めない）と、スタックの検査のハンドラーの登録。
  3. `usb_console_init()`: 互換層のオブジェクトとスレッド、I2C、KV の領域の範囲。
  4. `z_os_thread_start()`: スレッド "main"（本体は `usb_console_main_thread()`、終わったら `z_os_thread_exit()`）。
  5. `usb_console_shell_start()`: スレッド "console"。
  6. `tx_kernel_enter()`（済んだ初期化は飛ばし、空の `tx_application_define()` のあとスケジューラーへ）。
- `_tx_initialize_low_level()` は割り込みを止めるので、2 からスケジューラーが最初のスレッドを取るまで割り込みは止まったままです。
  3〜5 は、割り込みも時間の経過も要らないことだけをします（USB の stdio はホストを待たない、I2C は転送しない、ログは RAM へ）。
  USB の列挙、最初の tick、保留された割り込みは、スケジューラーが始まってから動きます。
- 起動のスレッド "main" のスタックは SCRATCH_X の noload の領域にあり、起動のコードは 0 にしません。ウォッチドッグの再起動のあとは前の実行の
  ThreadX のオブジェクトの ID が残っているので、`main()` はスタックのあとの領域を 0 で埋めてから始めます。
- ベクター: Pico SDK の crt0 がベクターの表とメインのスタックを持ちます。SDK の弱い `isr_pendsv`・`isr_svcall` を、リンクの `--defsym` で
  ThreadX の `__tx_PendSVHandler`・`__tx_SVCallHandler` にします（`threadx_kernel.cmake`）。`isr_systick` は `src/tick.c` です。割り込みはメインの
  スタック（MSP、2048 バイト。`ps` の `(interrupts)`）で動き、スケジューラーの待ちとアイドルのフックもそこで動きます。
- カーネルは何も確保しません。オブジェクトのメモリーはすべて呼ぶ側のもので、バイトプールもブロックプールも作りません。

## アイドルと `cpu_active`

ThreadX にはアイドルスレッドがありません。実行できるスレッドが無いと、スケジューラーは PendSV の中で、割り込みを止めて
`tx_low_power_enter()`、`WFI`、`tx_low_power_exit()`、割り込みの許可をくり返します（`tx_user.h` の `TX_LOW_POWER`、`TX_ENABLE_WFI`）。
`src/idle_os.h` がこの 2 つのフックを、共有の `common/imu/port/pico_sdk/idle.c` の 2 つの半分（`IDLE_OS_ENTER`: 区間を開き LED を点滅、
`IDLE_OS_EXIT`: 区間を閉じる）にします。測り方はほかの OS のファームウェアと同じで、待ちを起こした割り込みのハンドラーは区間のあとに
動くので負荷に入ります。1 ms ごとの tick の割り込みも負荷に入ります。

## 実行時間（`ps` の CPU 列）

ThreadX 付属の実行時間のキット（`utility/execution_profile_kit`、`TX_EXECUTION_PROFILE_ENABLE`）を変更せずに使います。時計は TIMER の下位
32 ビット（1 µs）で、合計は 64 ビットです。スレッドの時間は切り替えで入ってから出るまでなので、スレッドを割り込んだハンドラーの時間は
そのスレッドに入ります（Pico SDK のハンドラーはキットの `_tx_execution_isr_enter()` を呼ばない。FreeRTOS の実行時間の統計と同じ）。スケジューラーが
待っていた時間はキットのアイドルの時間で、`ps` の `(idle)` の行です。`cpu_active` はキットのアイドルの時間ではなく、上の区間から測ります。
読み出しは `src/run_time.c`、表示は `src/commands_os.c` です。キットの分だけ `TX_THREAD` が 12 バイト大きくなります。

## Pico SDK のロックの連携

Pico SDK の `mutex_t`・`semaphore_t`・`sleep_ms()` は、既定ではコアの番号を持ち主とし、WFE で回って待ちます。1 つのコアに複数のスレッドがあると、
2 本目のスレッドが持たれているミューテックスを自分のものと見て（「デッドロック」と判断し）、`stdio_usb` は出力や入力を黙って捨てます。
`threadx/drivers/pico_sdk_interop/` は SDK の RTOS のアダプターのヘッダー（`PICO_CONFIG_RTOS_ADAPTER_HEADER`。全ソースに読ませる）で `pico/lock_core.h` の
フックを差し替えます（FreeRTOS の RP2040 のポートの `configSUPPORT_PICO_SYNC_INTEROP` と同じ作り）。

- 持ち主の ID は呼んだスレッドの `TX_THREAD`（割り込みの中では例外の番号の 2 倍、スケジューラーの前は 0）。
- 待ちはスピンロックを外してから `TX_EVENT_FLAGS_GROUP` 1 つのビット（スピンロックの番号）を `TX_OR_CLEAR` で取ります。通知はそのビットを
  立て、待っているスレッドをすべて起こします。グループは最初に待つときに作ります。
- スレッドとして待てないところ（スケジューラーの前、割り込みの中、呼んだ側が割り込みを止めているとき、互換層のロックの中）では SDK 自身の
  WFE のままです。`sleep_ms()` は SDK のスリープの通知の上で待ち、最後の 150 µs を回ります（`PICO_TIME_SLEEP_OVERHEAD_ADJUST_US`）。
- `malloc()` も SDK のミューテックスで直列化します（`PICO_USE_MALLOC_MUTEX`）。`flash_safe_execute()` は割り込みを止めるだけなので変更は要りません。

## スタックの検査

`TX_ENABLE_STACK_CHECKING`: スレッドが自分で待ちに入るとき（待ち、スリープ、譲り）と再開されるときに、ThreadX がスタックのいちばん下の語と
上端の上の語の 0xEF の印、保存したスタックポインターを確かめ、外れていれば `fault_os_stack_error()`（`src/fault_os.c`）を呼びます。異常停止の
記録には、そのスレッドの名前が入ります（動いているスレッドとは限らない）。FreeRTOS 版（`configCHECK_FOR_STACK_OVERFLOW` 2）は切り替えのたびに
確かめますが、ThreadX は PendSV による横取りのときは確かめません。ThreadX は各スタックの上端の 1 語を印に使います。`ps` の STACK FREE は、
スタックの下から 0xEF のまま残っているバイト数です。

## RAM に置くコード

よく通るコード（センサーの読み出しと送信の経路のドライバーとアプリ）はフラッシュではなく RAM から実行します
（共有の一覧 `common/imu/port/pico_sdk/ram_code.cmake`。[`freertos-port-ja.md` の「RAM に置くコード」](freertos-port-ja.md#ram-に置くコード)）。OS の分として足すのは、FreeRTOS 版と同じく NINA のコアの
つなぎだけです（`threadx/app/usb-console/CMakeLists.txt` の `RAM_CODE_OS_SOURCES`: `common/drivers/nina/port/threadx/nina_os.c`）。ThreadX のカーネルはフラッシュに置きます。

## ThreadX の内部を使うところ

公開の API に無いものを使うのは次のところだけです。ThreadX を新しいバージョンにするときは、これらの名前と意味が変わっていないことを確かめます。
互換層の ThreadX の部分（`common/compat/port/threadx/`）とドライバーのつなぎの分も、ここにまとめます。

| 使うもの | 場所 | 理由 |
|---|---|---|
| `_tx_thread_preempt_disable` | `common/compat/port/threadx/zephyr_kernel_os_impl.h`（`z_os_task_lock()`）、`threadx/drivers/pico_sdk_interop/pico_sdk_interop.c`（待てるかの判定） | ロックの中で切り替えないため（[`common/compat/README_ja.md`](../common/compat/README_ja.md) のスレッドのロック） |
| `_tx_thread_system_preempt_check()` | `zephyr_kernel_os_impl.h`（`z_os_task_unlock()`） | ロックの中のサービスが残した切り替えを解除で行う |
| `_tx_thread_current_ptr` | `zephyr_kernel_os_impl.h`、`src/fault_os.c`、`src/commands_os.c`、`pico_sdk_interop.c` | 呼んだスレッド（呼び出しなしで） |
| `_tx_thread_system_state` | `zephyr_kernel_os_impl.h`（`z_os_pre_kernel()`） | カーネルの前か（呼び出しなしで。ISR の中かは、ポートのヘッダー `common/compat/port/threadx/rp2040/zephyr_os_port.h` が IPSR で見る） |
| `TX_THREAD` のメンバー（`tx_thread_priority`、`tx_thread_state`、`tx_thread_name`、`tx_thread_stack_start`・`_end`、`tx_thread_timer`、実行時間） | `zephyr_kernel_os_impl.h`、`src/commands_os.c`、`src/fault_os.c`、`src/run_time.c` | 優先度、終わったか、名前、`ps` の状態とスタックの空き |
| `_tx_mutex_created_ptr`、`_tx_mutex_created_count` と `tx_mutex_created_next`・`_previous` の付け替え | `zephyr_kernel_os_impl.h`（`z_tx_mutex_create()`） | 0 で埋められるミューテックスを作成済みの一覧から外す |
| `TX_MUTEX` のメンバーと `_tx_mutex_priority_change()` | `zephyr_kernel_os_impl.h`（`z_tx_mutex_timed_out()`、`z_os_mutex_unlock()`） | 待ち手のタイムアウトで持ち主の優先度を戻す。持ち主の確認 |
| `tx_semaphore_count`、`tx_semaphore_suspended_count` への直接の読み書き | `common/drivers/i2c_dw/port/threadx/i2c_dw_os_impl.h` | `k_sem_reset()`、カーネルの前の 2 回目の初期化（ThreadX に数を書くサービスは無い）、待ち手が 2 つ以上のときの並べ替え |
| `_tx_thread_created_ptr`、`_tx_thread_created_count` | `src/commands_os.c`（`ps`） | スレッドの一覧（割り込みを止めて読む） |
| `_tx_initialize_kernel_setup()` | `src/main.c` | スケジューラーの前にオブジェクトとスレッドを作る |
| `_tx_timer_interrupt()`、`_tx_initialize_unused_memory`、`_tx_thread_system_stack_ptr`、`__tx_PendSVHandler`・`__tx_SVCallHandler` | `src/tick.c`、`src/tx_initialize_low_level.c`、`threadx/app/usb-console/threadx_kernel.cmake` | ポートの tick とハンドラー（ポートの例の `tx_initialize_low_level.S` がするのと同じ） |
| `_tx_execution_idle_time_total`、`tx_low_power_enter()`・`_exit()` | `src/run_time.c`、`src/idle_os.h` | キットのアイドルの時間、スケジューラーの待ちのフック（どちらも ThreadX のヘッダーに宣言が無い） |

## スレッド

優先度は ThreadX の番号（Zephyr の優先度 + 16、0 が最優先。`common/compat/port/threadx/priorities_os.h`）です。カーネル自身のスレッド
（アイドルスレッド、タイマーのスレッド）はありません。

| スレッド | 優先度（ThreadX、Zephyr） | スタック | 内容 |
|---|---|---|---|
| `net_mgmt`、`sysworkq` | 15（−1） | 2048、384 | ネットワークのイベント、システムワークキュー |
| `main` | 16（0） | 2048 | 起動の後半。終わると `ps` に出なくなる。スタックは SCRATCH_X |
| `sensor_shell_tid`、`RTIO WQ 0` | 16（0） | 2048、1024 | `sensor` の結果の表示、RTIO のワーク |
| `nina_wifi_tid`、`wifi_reconnect_tid` | 24（8） | 2048、2048 | Wi-Fi のスキャン・接続・監視、自動再接続 |
| `time_sync_tid`、`imu_stream_tid` | 25（9） | 2048、2048 | 時刻同期、取得 |
| `imu_fifo_worker` | 26（10） | 2048 | FIFO の読み出し |
| `imu_raw_mqtt_tid` | 27（11） | 2048 | MQTT の送信 |
| `console`、`shell_telnet`、`telnet_service`、`zperf_server_tid` | 30（14） | 3072、3072、2048、2048 | USB と Telnet のシェル、Telnet の受け付け、`zperf` のサーバー |

互換層のスレッドのスタックは、今は FreeRTOS 版で測った値です。表の大きさにはスタックのあとの 272 バイトを含みません。

## FreeRTOS 版との違い

コマンドの構文と出力の形は FreeRTOS 版と同じです。違いは次のとおりです（Zephyr 版・NuttX 版とも比べた表は
[`freertos-port-ja.md` の「ほかの OS との違い」](freertos-port-ja.md#ほかの-os-との違い)）。

| 点 | FreeRTOS 版 | ThreadX 版 |
|---|---|---|
| プロンプト、USB の製品名 | `freertos:~$ `、"FreeRTOS USB Console" | `threadx:~$ `、"ThreadX USB Console" |
| `version` | `FreeRTOS kernel <バージョン>, …` | `ThreadX kernel 6.5.2.202603b, …` |
| `ps` の PRIO | 20 − Zephyr（大きいほど急ぐ） | Zephyr + 16（0 が最優先） |
| `ps` の行 | `Tmr Svc`・`IDLE` のタスクがある | カーネルのスレッドは無く、スケジューラーの待ちが `(idle)`。終わった "main" は出さない |
| 同じ優先度の選び方 | 切り替えのたびの順番回し、tick のフックの時分割 | 先着順、横取りされたスレッドは先頭に残る、ThreadX の時分割 |
| 優先度継承で上がった持ち主が下がるとき | 持っているミューテックスをすべて離したとき | 1 つ離すたびに、まだ持っているミューテックスの待ち手の最高まで |
| スタックの検査 | 切り替えのたび | 自分で待ちに入るときと再開のとき |
| 最初の待ちの前の Wi-Fi のスレッドへの post | 残らない | 残る（Zephyr 版と同じ） |
| 除算器 | ポートが切り替えのたびに 4 語を退避 | ポートは退避しない。Pico SDK の除算のルーチンが、途中の除算（DIRTY）を見つけたら自分で退避して戻す |

## ホストテスト

`threadx/script/host_tests.sh [-v]` が、共有ソースのテスト（`common/tests/run.sh`）に続けて、次を ThreadX の Linux ポート（ThreadX のツリーの
`ports/linux/gnu`）の上で動かします。ボードは要りません。割り込みは、ThreadX のコンテキストの保存と復元に包んだ Linux のスレッド 1 本です
（`threadx/tests/i2c_dw/tx_test_isr.c`）。

| テスト | 対象 |
|---|---|
| `zephyr_kernel_os_impl` | ThreadX の部分（`common/compat/port/threadx/zephyr_kernel_os_impl.h`）が互換層のヘッダーを読まずに単独でコンパイルできること |
| `zephyr_kernel_test` | 互換層のカーネル（`common/tests/zephyr_kernel/zephyr_kernel_test.c` の 22 項目を 2 回。2 回目は 1 回目のオブジェクトとスタックを使い直す）。ハードウェアの割り込みからの起床、実時間の確認、tick の追いつきは飛ばす。テストの tick は 1 tick の ThreadX のタイマー。ThreadX の部分は `threadx/tests/compat/` |
| `i2c_dw` | FreeRTOS 版のテストの本体（`freertos/tests/i2c_dw/main.c`）を ThreadX の部分（`threadx/tests/i2c_dw/i2c_dw_test_os.c`）と一緒に。項目は [DesignWare I2C のコア](../common/drivers/i2c_dw/README_ja.md#テスト) |
| `nina` | FreeRTOS 版のテストの本体（`freertos/tests/nina/main.c`）を `threadx/tests/nina/nina_test_os.c` と一緒に。項目は [NINA-W102 のドライバー](../common/drivers/nina/README_ja.md#テスト) |

## 実機の試験用ファームウェア（`tests/board_kernel`）

ThreadX の部分を実機で確かめるための道具（`threadx/tests/board_kernel/`）で、製品のファームウェアではありません。製品と同じ起動の手順とボードの部分（`tx_user.h`、
低レベルの初期化、tick）の上で、次を動かします。

- 互換層のカーネルの試験 22 項目を、ボードのプラットフォームの部分（`test_port.c`: TIMER のアラームの割り込み、実時間、割り込みを止める時間）で。
  ホストで飛ばす項目も動きます。
- ボードの直接の確認（`lock_test.c`）: TIMER で 1 秒のあいだの tick の数、ロックの中の起こしが解除まで走らないこと（入れ子も）、ロックの中で
  期限の来た割り込みが解除で動き、それが起こしたスレッドが解除から戻る前に走ること。
- アイドルと実行時間（`idle_test.c`）: 何もしないときと回し続けるときの `cpu_load_get()`、`tx_low_power_enter()` の回数、2 つのスレッドの実行時間。
- Pico SDK のロックの連携（`sdk_test.c`）: カーネルの前の SDK の待ち、SDK のミューテックスの待ちのあいだ低い優先度が走ること、時間切れ、`sleep_ms()`、
  2 つのスレッドからの同時の `printf()`。

使い方:

```bash
threadx/script/build.sh board_kernel          # threadx/build/board_kernel/board_kernel.uf2
threadx/script/flash.sh threadx/build/board_kernel/board_kernel.uf2
```

書き込みは製品のファームウェアが動いていればその `bootsel` で入ります。試験用ファームウェアには `bootsel` コマンドがありません。
試験用ファームウェアが動いているボードを BOOTSEL に入れるには、その USB のポートを 1200 bps で開きます（Pico SDK の `stdio_usb` の機能）。
そのあと `RPI-RP2` が見えたら書き込みます。USB の製品名は "ThreadX kernel test" です。ホストがポートを開くと（DTR）、ThreadX のバージョン、
TIMER で測った tick、各確認の行と `board_test: <n> passed, <m> failed` を出し、要約を 5 秒ごとにくり返します。USB なしでも LED（GPIO6）で状態が
分かります: 点灯（起動中）、1 Hz（ホストを待つ）、5 Hz（実行中）、2 秒に 1 回の短い点灯（すべて通った）、2 秒に 3 回（失敗あり）、1 秒点灯・1 秒消灯
（アサーションか HardFault）。
