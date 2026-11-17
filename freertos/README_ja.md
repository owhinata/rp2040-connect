# Pico SDK + FreeRTOS 版

[English](README.md) | 日本語

Arduino Nano RP2040 Connect 向けに、Raspberry Pi Pico SDK と FreeRTOS カーネルでファームウェアを
ビルドする環境です。`zephyr/`、`nuttx/`、`openmv/` と同じく、必要なものを `freertos/` の下に取得します。

今あるのは、USB のコマンドライン（CLI）、LED の点滅、Zephyr 版を移す土台になるカーネルの設定・CPU 負荷の
計測・異常停止の扱いだけのアプリ `app/usb-console` と、Zephyr 版・NuttX 版と共有するソース（`common/`）を
動かすための Zephyr 互換層の FreeRTOS 実装（`drivers/compat`）です。IMU の計測（Zephyr 版・NuttX 版）は
まだ移していません。

| 項目 | 内容 |
|---|---|
| Pico SDK | 2.3.1（ボードは `arduino_nano_rp2040_connect`） |
| FreeRTOS カーネル | V11.3.3（RP2040 ポートは `portable/ThirdParty/GCC/RP2040`、1 コアで使用） |
| コンパイラー | Arm GNU Toolchain 15.2.Rel1（GCC 15.2） |

## 必要なもの

- Linux の PC（x86_64）。sudo は要りません。
- `git`、`curl` または `wget`、`tar`、`xz`、`sha256sum`
- `python3`（`venv` モジュールを含む。Ubuntu では `python3-venv`）、`cmake`（3.13 以降）、`make`、`gcc`、`g++`（Pico SDK がホスト用の `picotool` をビルドするため）
- ディスクの空き 約 1.5 GB（ツールチェーンが 1.1 GB）

## 環境構築

リポジトリのルートで読み込みます。足りないものだけを取得するので、何度読み込んでもかまいません。
新しいターミナルでは読み込み直します。

```bash
source freertos/script/setup.sh
```

| 置き場所 | 内容 | 版 |
|---|---|---|
| `freertos/pico-sdk` | Raspberry Pi Pico SDK と、そのサブモジュールのうち TinyUSB（USB コンソール用） | タグ `2.3.1`（shallow clone） |
| `freertos/FreeRTOS-Kernel` | FreeRTOS カーネル | タグ `V11.3.3`（shallow clone） |
| `freertos/toolchain/` | Arm GNU Toolchain（arm-none-eabi、x86_64 Linux 用） | 15.2.Rel1（SHA-256 を検証） |
| `freertos/.venv` | Python の venv（整形に使う `clang-format`） | clang-format 18.1.3（PyPI） |

あわせて、現在のシェルに `PICO_SDK_PATH`、`FREERTOS_KERNEL_PATH`、`PICO_TOOLCHAIN_PATH` と `PATH`
（ツールチェーン、`freertos/.venv/bin`、`freertos/script`）を設定します。版はスクリプトの冒頭で固定しています。取得したものは
Git の管理外です。

## ビルド

```bash
freertos/script/build.sh [--clean] [-j N] [-v] [-DNAME=VALUE ...] [APP]
```

`setup.sh` を自分で読み込み、`freertos/app/APP`（既定は `usb-console`）を `freertos/build/APP/` に
ビルドします。結果は `freertos/build/usb-console/usb-console.uf2` です。`-DNAME=VALUE` は CMake に渡し、
`--clean` するまでビルドディレクトリーに残ります。`usb-console` のオプションは `-DNANO_SELFTEST=ON`
（既定は OFF）だけで、試験用の `selftest` コマンドを入れます（下の「Zephyr 互換層」）。

## ホストテスト

```bash
freertos/script/host_tests.sh [-v]
```

共有ソースのテスト（`common/tests/run.sh`）を走らせたあと、Zephyr 互換層の自己診断（`selftest` コマンドと
同じソース）を、カーネルのツリーにある FreeRTOS の POSIX ポートの上でビルドして 2 回走らせます
（`freertos/tests/compat`）。`freertos/FreeRTOS-Kernel`（`setup.sh`）と、POSIX スレッドの使えるホストの `cc` が
要ります。ボードは要りません。`-v` で自己診断の出力も出します。

最初のビルドでは、Pico SDK が `picotool`（UF2 を作るツール）のソースを GitHub から
`freertos/picotool/` に取得し、ホスト用にビルドします（ネットワークが必要です）。`--clean` のあとは
`picotool` もビルドし直します。

## 書き込み

```bash
freertos/script/flash.sh [--build] [--wait 秒] [UF2]
```

1. `RPI-RP2` ドライブ（BOOTSEL モードの RP2040）があれば、そこへ UF2 をコピーします。
2. 無くて、このファームウェアの USB コンソールがあれば、`bootsel` コマンドを送ってドライブを待ちます。
   操作は要りません。
3. どちらも無ければ、BOOTSEL への入り方を表示してドライブを待ちます。

コピーのあと、USB コンソール（`/dev/serial/by-id/usb-*FreeRTOS_USB_Console*`）が現れるのを待って、
そのパスを表示します。ドライブがマウントされていなければ `udisksctl` でマウントします。

BOOTSEL への入り方:

- このファームウェアが動いているとき: CLI の `bootsel` コマンド。シリアルを 1200 bps で開く方法と
  `picotool reboot -f -u` も使えます（Pico SDK の USB コンソールの標準の機能）。
- Zephyr 版・NuttX 版が動いているとき: RESET ボタンを素早く 2 回押します。効かなければ、REC を GND に
  つないで RESET を押します。
- OpenMV 版が動いているとき: REPL で `import machine; machine.bootloader()` を実行します。

**書き込む前に**: 書き込むと Zephyr 版・NuttX 版・OpenMV 版のファームウェアは消えます。このアプリは
フラッシュの後ろの領域（Zephyr 版・NuttX 版の KV、OpenMV 版のファイルシステム）には書き込みません。

## アプリ `app/usb-console`

USB の CDC ACM ポートがコンソールです（UART0 は使いません）。端末を開くとプロンプト `freertos:~$ ` が
出ます。行の編集は Backspace と Ctrl-C（行を捨てる）だけです。

| コマンド | 内容 |
|---|---|
| `help` | コマンドの一覧 |
| `version` | FreeRTOS カーネル、Pico SDK、コンパイラーの版 |
| `id` | ボードの固有 ID（フラッシュの ID、16 桁の 16 進数） |
| `uptime` | 起動からの時間（RP2040 の TIMER）と tick の数 |
| `sleep <ms>` | ms ミリ秒眠る。丸めは Zephyr と同じ（Zephyr 版の `kernel sleep`） |
| `devmem <addr> [<bits> [<val>]]` | メモリーの読み書き（Zephyr 版の `devmem` と同じ書式: アドレスと値は 16 進、幅は 8・16・32 ビット、値を付けると書き込み）。存在しないアドレスでは異常停止する |
| `ps` | タスクの一覧: 状態、優先度、スタックの空きの最小値（バイト）、起動からの CPU 時間の割合。最後に前回の `ps` からの CPU 負荷（`cpu_active`） |
| `irqstack` | 割り込み用のスタックの起動からの使用量（NuttX 版の `irqstack`） |
| `fault` | この起動の前の実行を止めた異常停止の記録 |
| `free` | FreeRTOS のヒープの大きさ、空き、空きの最小値 |
| `reboot` | 再起動 |
| `bootsel` | USB のブートローダー（BOOTSEL、`RPI-RP2` ドライブ）で再起動 |
| `selftest` | `-DNANO_SELFTEST=ON` のときだけ: Zephyr 互換層の自己診断（下） |

| ファイル | 役割 |
|---|---|
| `CMakeLists.txt` | Pico SDK と FreeRTOS カーネル、Zephyr 互換層（`drivers/compat/compat.cmake`）、`common/` の共有ソースの取り込み、USB の製品名（`FreeRTOS USB Console`）、オプション `NANO_SELFTEST` |
| `FreeRTOSConfig.h` | カーネルの設定 |
| `src/main.c` | 起動（`zephyr_compat_initialize()` を含む）、コンソールのタスク、LED のタスク |
| `src/cli.[ch]` | 行の編集とコマンドの表。カーネルにも SDK にも依存しません |
| `src/commands.c` | 上のコマンド |
| `src/priorities_os.h` | 優先度とスタックの表（`common/compat/priorities.h`）の FreeRTOS の部分 |
| `src/os_clock.h` | 起動からの時間（TIMER）、tick の数、Zephyr と同じ丸めの sleep |
| `src/tick.c` | tick フック: 互換層のタイムアウト（`zephyr_compat_tick()`）と 20 ms の時分割 |
| `src/idle.c` | アイドルフック: CPU 負荷（`cpu_load_get()`、`common/compat/cpu_load.h`） |
| `src/irq_stack.c` | 割り込み用のスタックの最大使用量 |
| `src/fault.[ch]` | 異常停止の共通の出口と、前回の記録（`common/log/fault_record.[ch]`） |

### タスク、優先度、スタック

優先度は移植の表（`common/compat/priorities.h`。NuttX 版と共有）に従い、その FreeRTOS の部分が
`src/priorities_os.h` です: FreeRTOS の優先度 = 20 − Zephyr の優先度。Zephyr のプリエンプティブな優先度
0〜14 は 20〜6、協調の −1 は 21 になります（FreeRTOS では協調スレッドもより高い優先度に割り込まれます。
NuttX 版と同じ近似です）。スタックは Zephyr と同じくバイトで書き、静的に確保します。

| タスク | 優先度 | スタック | Zephyr 版 |
|---|---|---|---|
| `Tmr Svc`（カーネルのタイマータスク） | 31 | 1024 B | 無し（下） |
| `console`（USB コンソール。コマンドもここで動く） | 6 | 3072 B | シェルのスレッド（14、3072 B） |
| `heartbeat`（LED を 500 ms ごとに反転。GPIO6） | 1 | 1024 B | 無し（Zephyr 版は LED を使わない） |
| `IDLE`（CPU 負荷を測る） | 0 | 1024 B | idle（15、256 B） |
| `sysworkq`（互換層のシステムのワークキュー） | 21 | 1024 B | sysworkq（−1、1024 B） |
| `RTIO WQ 0`（互換層の RTIO のワークキュー） | 20 | 1024 B | RTIO WQ 0（0、1024 B） |

`-DNANO_SELFTEST=ON` のときは、最初の `selftest` が `st_static_hi_tid`（7）と `st_static_eq_tid`（6）を起動し、
`st_workq`（6）を作ります（以後ずっと待ったまま残ります）。ほかに短い間だけ補助のタスク（`st_*`）を作ります。

- `Tmr Svc` を最高の優先度にしているのは、割り込みハンドラーが Pico SDK のロック（USB の stdio の
  ミューテックス）を放したときや sleep のアラームが来たときに、待っているタスクをこのタスクが起こすからです。
  Zephyr では割り込みハンドラーが自分でスレッドを起こします。
- 割り込み用のスタック（メインスタック）は Pico SDK の `PICO_STACK_SIZE` で 2048 バイト、Zephyr 版の
  `CONFIG_ISR_STACK_SIZE` と同じ大きさです。`irqstack` が Zephyr 版の `kernel thread stacks` の `IRQ 00` の
  行と同じ書式で使用量を出します。値は起動からの最大（`main()` の最初でスタックを模様で埋める）で、
  スケジューラーの開始後もこのスタックに残る `main()` のフレームを含みます。割り込み用のスタックの底には
  見張りがありません。

### カーネルの設定

- 1 コア（`configNUMBER_OF_CORES` 1）、優先度は 0〜31（数が大きいほど高い）、タスク名は 31 文字まで
  （`configMAX_TASK_NAME_LEN` 32。Zephyr の `CONFIG_THREAD_MAX_NAME_LEN`）、タスクごとの通知は 2 つ
  （`configTASK_NOTIFICATION_ARRAY_ENTRIES` 2）、ヒープは `heap_4` の 32 KB。Pico SDK のロックと sleep は、
  呼んだタスクを待たせます（`configSUPPORT_PICO_SYNC_INTEROP`、`configSUPPORT_PICO_TIME_INTEROP`）。
- tick は Zephyr 版・NuttX 版と同じ 100 µs（`configTICK_RATE_HZ` 10000）で、幅は 32 ビット（RP2040 ポートは
  64 ビットの tick を受け付けない）なので約 4.97 日で一周します。起動からの時間は tick ではなく RP2040 の
  TIMER（64 ビット、1 µs）から得ます。tick は周期割り込み（SysTick）です。Zephyr 版・NuttX 版は tickless
  です。この周期では `portTICK_PERIOD_MS` が 0 になるので使いません。Pico SDK の `sleep_ms()` も使いません
  （最後の 150 µs を空回りで待つ）。
- sleep の丸めは Zephyr と同じです: t の sleep は ⌈t / 100 µs⌉ tick より長く、それより最大 1 tick 長い
  （`vTaskDelay(⌈t⌉ + 1)`。0 は yield だけ）。
- 同じ優先度のタスクは、Zephyr 版（`CONFIG_TIMESLICE_SIZE=20`、`CONFIG_TIMESLICE_PRIORITY=0`）と同じく 20 ms
  ずつ時分割します。tick ごとに切り替える FreeRTOS の時分割は切り（`configUSE_TIME_SLICING` 0）、tick フックが
  同じタスクが 200 tick 続いたら切り替えを頼みます。Zephyr と同じく、別のタスクに切り替わるたびに時間を
  数え直し、より高い優先度に横取りされたタスクは順番を失わず、相手が待ちに入ると続きを走ります（「Zephyr 互換層」）。
  時間は tick 単位で数えるので、tick の途中で切り替わったタスクは (199, 200] tick です（Zephyr は (200, 201]）。

### CPU 負荷

アイドルタスクが、Zephyr 版（`CONFIG_CPU_LOAD` とアイドルフックの方式）・NuttX 版（`up_idle()`）と同じ方法で
CPU 負荷を測ります: 割り込みを禁止して窓を開き、`wfi` で待ち、窓を閉じて割り込みを許可するので、アイドル中に
来た割り込みの処理も負荷に数えます。計算は `common/compat/nano_cpuload.h`、`cpu_load_get()`
（`common/compat/cpu_load.h`）は IMU の取得サービスの `cpu_active`（`imu status`）が使う予定のものです。
Zephyr 版にも NuttX 版にもこれだけを出すコマンドは無いので、ここでは `ps` の最後の行に、前回の `ps` からの
負荷を出します（`cpu_load_get()` の窓はリセットしません）:

```
cpu_active: <負荷>% in the <秒> s since the previous "ps"
```

100 µs の周期 tick は 1 秒に 10000 回 CPU を起こし、その処理も負荷に入ります。Zephyr 版・NuttX 版にはこの
割り込みがありません。

### 異常停止

`panic()`（Pico SDK の中の呼び出し、`hard_assert`、`malloc()` のメモリー不足、戻ってしまったタスクの関数も）、
HardFault の例外、`configASSERT()`・`assert()` の失敗、スタックのあふれ（`configCHECK_FOR_STACK_OVERFLOW` 2）、
ヒープの不足、`exit()` は、すべて 1 つの出口（`src/fault.c`）に集まります。出口は割り込みを禁止し、理由、
PC、LR、SP（HardFault では例外フレームの r0〜r3、r12、xPSR も）、例外番号、動いていたタスク、時刻、
メッセージを、起動処理が消さない RAM（`__uninitialized_ram`、CRC-32 つき）に記録し、理由・PC・LR を
watchdog の scratch レジスター 0〜3 にも残して、watchdog で再起動します（`reboot` と同じ）。Zephyr 版・
NuttX 版も異常停止のあとは再起動します。その場ではコンソールに何も出しません（USB の処理には割り込みと
タスクが要るため）。

再起動のあと、端末を最初につないだときにプロンプトの前に記録を出し、`fault` でも出します。例（値は
ホストテストのもので、実機のものではありません）:

```
*** The last run ended with a fatal error ("fault" shows it again):
Fatal error: HardFault
In task console, 3723.000042 s after the start
pc 0x10001234 lr 0x10005679 sp 0x20010f00
r0 0x30000000 r1 0x00000001 r2 0x00000002 r3 0x00000003
r12 0x0000000c xpsr 0x61000000
```

HardFault 以外では、PC は出口を呼んだ場所で、LR は 0（不明）です。記録は起動時に 1 回読んで消すので、その後の
`reboot` や電源の入れ直しのあとは `No fatal error before this start` と出ます。記録の形式とチェック値は OS に
依存しない部分（`common/log/fault_record.[ch]`）で、`common/tests/run.sh` がテストします。

### Zephyr 互換層（`freertos/drivers/compat`）

Zephyr 版・NuttX 版と共有するソースは Zephyr のカーネルの API を使います。NuttX 版ではそれを互換層が
提供します。互換層は OS に依存しない部分（`common/compat`: ヘッダー、`zephyr_work.c`、RTIO など）と NuttX の
部分（`nuttx/drivers/compat`）に分かれています。`freertos/drivers/compat` は NuttX の部分と対になる FreeRTOS の
部分で、`common/compat/zephyr/kernel.h` の先頭の一覧に従って書いています。アプリはまだ共有のドライバーを
使っていません。この段階でリンクできる互換層の OS 非依存のソース（`zephyr_work.c`、`zephyr_rtio.c`、
`zephyr_rtio_workq.c`、`zephyr_mem_blocks.c`）と `common/log/log_ring.c` を組み込んでいます。

| ファイル | 役割 |
|---|---|
| `zephyr_os.h` | `<zephyr/kernel.h>` の OS の部分: tick（`Z_USEC_PER_TICK`）、`struct k_mutex`、`struct k_sem`、遅延 work のタイマー、`k_tid_t`（タスクのハンドル）、`K_PRIO_PREEMPT()`、`k_current_get()`、`k_cycle_get_32/64()`（TIMER） |
| `zephyr_os_spinlock.h`、`zephyr_os_atomic.h` | `k_spin_lock()`。atomic の方は何もしない（FreeRTOS と Pico SDK のヘッダーはその名前を使わない） |
| `zephyr_kernel.c` | セマフォ、`k_poll()`、スレッド（`K_THREAD_DEFINE()`、ワークキューのスレッド、`k_thread_name_set()`）、タイムアウト、時間と sleep、`zephyr_compat_initialize()`、`zephyr_compat_tick()` |
| `zephyr_mutex.c` | `k_mutex` |
| `zephyr_rtio_workq_os.h` | スレッド `RTIO WQ 0` |
| `zephyr/sys/__assert.h` | `__ASSERT()`: Zephyr 版（`CONFIG_ASSERT` が無効）と同じく何もしない |
| `common_prelude.h` | `common/` の各ソースの前に読む（`-include`）。今は空（Kconfig の値は、それを使うソースと一緒に足す） |
| `zephyr_compat_os.h` | `zephyr_compat_tick()`（アプリの tick フックが呼ぶ） |
| `zephyr_os_internal.h` | 互換層のファイルどうしで共有するもの |
| `zephyr_os_sched.h` | `FreeRTOSConfig.h` の最後で読む: 同じ優先度のタスクの選び方を Zephyr に合わせる（カーネルの切り替えのトレースマクロ） |
| `rp2040/zephyr_os_port.h` | ボードに依存する少しのこと（割り込みの中かどうか、割り込みの終わりの切り替え、タスクのロック、空回りの待ち、`os_clock.h` の時計）。ホストテストは自分のものを使う |
| `compat.cmake` | `freertos_zephyr_compat(<target> [SELFTEST])`: ソース、インクルードパス、前置きヘッダー |
| `zephyr_selftest.[ch]`、`rp2040/zephyr_selftest_port.c` | 自己診断（`selftest`、ホストテスト） |
| `../include/app_log_os.h`、`../log/app_log_min.c` | `LOG_*()`（`common/log/app_log.h`）の最小の受け口: Zephyr の書式の行を普通の RAM のリングに入れる（まだ誰も読まない）。RAM ログと `dmesg` で置き換える |

提供するもの: `k_mutex`（`K_MUTEX_DEFINE()`）、`k_sem`（`K_SEM_DEFINE()`）、セマフォの `k_poll()`
（`K_POLL_MODE_NOTIFY_ONLY`）、タイムアウト（`K_NO_WAIT`、`K_FOREVER`、`K_TICKS()`、`K_MSEC()`、`K_SECONDS()`）、
`k_sleep()`、`k_msleep()`、`k_usleep()`、`k_busy_wait()`、`k_uptime_get()`、`k_cycle_get_32/64()`、
`K_THREAD_DEFINE()`（`K_THREAD_DEFINE_START()` で起動）、`k_thread_name_set()`、`k_current_get()`、ワーク
アイテムとワークキュー（`zephyr_work.c`）とシステムのワークキュー、`k_spin_lock()`、atomic。共有ソースが
使うのはこれだけです。共有ソースが使わないので作っていないもの: `k_thread_create()`、`k_yield()`、
`k_wakeup()`、`k_timer`、`k_msgq`、`k_queue`、`k_fifo`、`k_condvar`、`k_event`、`k_poll_signal`、
`sys_timepoint_*()`、`k_uptime_ticks()`、`irq_lock()`、`k_is_in_isr()`。

Zephyr の前提の再現:

- 全部 0 の `k_mutex` は未ロック（Zephyr の MQTT ライブラリーがこれに頼る）。FreeRTOS にはミューテックスの
  静的な初期化子が無いので、最初の lock か unlock が `k_mutex` 自身のメモリーに FreeRTOS の再帰ミューテックスを
  作ります（`K_MUTEX_DEFINE()` も同じ）。戻り値は Zephyr と同じ: 持たれているものに `K_NO_WAIT` で `-EBUSY`、
  タイムアウトで `-EAGAIN`、誰も持っていないものの unlock で `-EINVAL`、他のスレッドのものの unlock で `-EPERM`。
- `k_sem` と `k_poll()` は、FreeRTOS のセマフォではなく互換層の数の上で Zephyr の `kernel/sem.c`・
  `kernel/poll.c` に従います（そのため `K_SEM_DEFINE()` は静的な初期化子）: give は、待っているうち最も
  優先度の高いもの（同じ優先度なら先着順）に数を直接渡すか、待ち手がいなければ上限まで 1 増やして最初の
  `k_poll()` のイベント（スレッドの優先度の順）を 1 つ知らせます。`k_sem_reset()` は待ちを `-EAGAIN` で
  終わらせます。待つタスクは自分のタスク通知の 1 番で眠ります（0 番は Pico SDK に残す）。
- `k_sem_give()`（したがって `k_work_submit()`）は割り込みハンドラーから呼べます。起こしたタスクは
  ハンドラーの直後に動きます（PendSV）。タイマータスク `Tmr Svc` を通りません。
- `k_spin_lock()`: Zephyr と同じく、unlock まで他のタスクに切り替わりません。タスクではカーネルの
  クリティカルセクション（`taskENTER_CRITICAL()`。PRIMASK だけだと、中で呼ぶ FreeRTOS の関数が割り込みを
  許可してしまう）、割り込みハンドラーでは `taskENTER_CRITICAL_FROM_ISR()`。タスクは、ほかの方法で割り込みを
  禁止したまま呼ばないこと。
- タイムアウト: n tick の相対の待ち（`k_sem_take()`、`k_mutex_lock()`、`k_poll()`、sleep）は FreeRTOS の
  待ち時間 n + 1 なので、Zephyr と同じく n tick より長く n + 1 tick 以内で終わります。`K_NO_WAIT` は待たず
  （sleep は yield）、`K_FOREVER` はいつまでも待ちます。1 回の待ちは最大 `portMAX_DELAY − 1` tick（約 4.97 日）で、
  それより長い sleep は何回かに分けます。
- 遅延 work のタイマーは互換層のリストで、tick フックが tick の割り込みの中で処理します
  （`zephyr_compat_tick()`。Zephyr がタイムアウトをシステムクロックの割り込みで処理するのと同じ）。FreeRTOS の
  ソフトウェアタイマーは使いません（開始と停止がタイマータスクへのメッセージで、`zephyr_work.c` は割り込みを
  止めている間はタイマーが走らないことに頼っている）。tick の割り込みを数えるので、割り込みが 1 tick より長く
  止まると tick と同じく遅れます。
- `K_THREAD_DEFINE()` のスレッドは静的なスタックの上のタスクで、アプリが `K_THREAD_DEFINE_START()` で起動します。
  タスクのハンドルは、タスクが動ける前に `k_tid_t` の変数に入ります（作る間スケジューラーを止める）。タスクの
  制御ブロックは FreeRTOS のヒープから取り、入口の関数が戻ったとき（タスクは消える）も確保したままになります。
- `k_uptime_get()` とサイクルカウンターは tick ではなく TIMER から得ます。
- 同じ優先度のタスクの間では Zephyr の規則（kernel/sched.c の `next_up()`）を守ります: より高い優先度に横取りされた
  タスクは順番の位置に残り、相手が待ちに入ると続きを走ります。次のタスクに順番を渡すのは yield（`k_sleep(K_NO_WAIT)`、
  ワークキューの yield、`sleep 0` コマンド）と時分割の時間切れだけで、別のタスクに切り替わると時間を数え直します
  （`z_time_slice_reset()`）。FreeRTOS は切り替えのたびに同じ優先度の次のタスクを選ぶ（`listGET_OWNER_OF_NEXT_ENTRY()`）
  ので、そのままだと一瞬横取りされたタスク（USB の割り込みのたびに Pico SDK の stdio のミューテックスを通じて起きる
  タイマータスク、ワークキューなど）が順番を失います。`zephyr_os_sched.h` が `FreeRTOSConfig.h` にトレースマクロを
  2 つ与え、カーネルの切り替えの中で動かします: yield でない切り替えで、まだ実行可能なタスクはその優先度のリストの
  位置を 1 つ戻して次も選ばれるようにし、別のタスクに切り替わったら時分割の時間を数え直します。カーネルの内部の
  変数名（V11.3.3、1 コア）を使いますが、カーネルのソースは変えていません。FreeRTOS のままの点: 実行可能になった
  タスクはリストの位置の後ろに入るので、より長く待っていたタスクより先になることがある（同じ優先度の実行可能な
  タスクが 3 つ以上のとき）、優先度の継承が終わって優先度が下がったタスクは順番を保つ（Zephyr は列の最後に回す）、
  FreeRTOS の素の `taskYIELD()` は横取りと同じ扱い。

FreeRTOS で Zephyr と違うところ（近似）:

- 優先度の継承は FreeRTOS のもの: 1 段だけ（Zephyr は持ち主の鎖をたどる）で、持ち主の優先度は持っている
  ミューテックスをすべて放したときに戻ります。タイムアウトした待ち手があれば下げ直します。
- 協調スレッド（Zephyr の負の優先度）は高い優先度（21 以上）にするだけで、より高い優先度には割り込まれます。
  Zephyr ではまったく割り込まれません（NuttX 版と同じ近似）。
- `k_tid_t` は FreeRTOS のタスクのハンドルで、NuttX 版と同じく `K_THREAD_DEFINE()` が定義して起動時に設定する
  変数です。Zephyr のような定数ではありません（そのため `common/imu/wifi_profile.c` には、NuttX 版と同じく
  `const` の無い `wifi_profile_os.h` が要ります）。
- セマフォの待ち手は、待ち始めたときの優先度で並べます（Zephyr は待っている間に優先度が変わると並べ直す）。

自己診断（`zephyr_selftest.c`）はこれらの意味を確かめます: 全部 0 のミューテックス、再帰と戻り値、
タイムアウト、優先度の継承（タイムアウトの後も）、セマフォの上限・タイムアウト・待ち手の順・reset、sleep と
その丸め、時計、tick の割り込みと（ボードでは）TIMER のアラームの割り込みからの `k_sem_give()`・
`k_work_submit()`、`k_poll()`（1 つのイベント、タイムアウト、待ち手の順）、遅延 work、スピンロック、atomic、
`K_THREAD_DEFINE()` のスレッドの起動（ボードの起動ごとに 1 回）、スレッドの名前、20 ms の時分割、横取りされた
タスクが順番を保つこと（`timeslice_preempt`: 同じ優先度で回り続けるタスク 2 つと、4 ms ごとに起きる高い優先度の
タスク。最初に走ったほうがずっと CPU を持つ）。ボードでは測った時間も TIMER で確かめ、ホストでは tick の数だけを
確かめます（POSIX ポートの tick はスレッドの `usleep()`）。`selftest` コマンドは項目ごとの行と測った時間を項目の
後でまとめて出し、USB コンソールが送り終えるのを待ってから次の項目に進みます（送信が起こす USB の割り込みと
タイマータスクが次の計測に重なるため）。最後に `selftest: N of 22 tests failed` を出し、N が 0 でなければエラーを
返します。約 1 秒かかり、そのうち 140 ms を 2 回、優先度 7 のタスク 2 つが回り続けるので、それより低いタスクは
待たされます。

### そのほか

- コンソールのタスクは、USB の受信の通知（Pico SDK の `stdio_set_chars_available_callback()`）で起き、
  端末の接続を見るために 50 ms ごとにも起きます。
- USB の製品名は `FreeRTOS USB Console` です。Pico SDK は USB の文字列を既定で 19 文字に切るので、上限
  （`USBD_DESC_STR_MAX`）を 32 にしています。
- CLI は FreeRTOS-Plus-CLI ではなく、このリポジトリのものです。

## 確かめたこと

2026-11-16 に、ビルドと実機（Arduino Nano RP2040 Connect）で確かめました。

- ビルドが警告なしで通ること（GCC 15.2）。`freertos/script/format.sh --check` が違いを報告しないこと。
- `flash.sh` が `RPI-RP2` ドライブへ UF2 をコピーし、再起動後の USB コンソールを見つけること。
- ファームウェアが動いているときに `flash.sh` が `bootsel` を送り、操作なしで書き込めること（2 回）。端末から
  打った `bootsel` でも BOOTSEL に入ること（1 回）。
- `help`、`version`、`id`、`uptime`、`ps`、`free` の出力と、無いコマンドへのエラー。

確かめていないこと: `reboot`、1200 bps でのオープンと `picotool` による BOOTSEL、長時間の動作。

2026-11-17 に実機で確かめたこと:

- tick の速さ: 23.15 秒で 231543 tick（10000.0/s）。
- `sleep` の往復時間（USB の往復 約 0.9 ms を含む、9 回の中央値）: `sleep 1` 1.99 ms、`sleep 10` 11.10 ms、
  `sleep 100` 101.16 ms、`sleep 1000` 1001.21 ms。
- アイドル時の `cpu_active`: 約 20 秒の窓で 2.0%（互換層を入れる前）、3.0%（入れた後の既定のビルド）。
- `irqstack`: 468 / 2048 バイト。
- 異常停止の記録: `devmem 30000000` で HardFault になり、自動で再起動して、バナーと `fault` にタスク（`console`）と
  PC（`devmem` の読み出しの命令）が出ること。次の `reboot` のあとは記録が無いこと。`devmem` の読み出しと書き込み。
- Zephyr 互換層の自己診断（`-DNANO_SELFTEST=ON` のビルドで `selftest`）: 4 回続けて全 22 項目が通ること。`ps` に
  `sysworkq`（21）と `RTIO WQ 0`（20）があること。繰り返してもヒープの空きが減り続けないこと。
- 既定のビルドに `selftest` コマンドが無いこと。

Zephyr 互換層は、`-DNANO_SELFTEST=ON` の有無の両方で警告なしにビルドでき、自己診断が FreeRTOS の POSIX ポートの
上でも通ります（`host_tests.sh`）。

確かめていないこと: スタックのあふれ・アサート・ヒープ不足からの異常停止の記録、長時間の動作。
