# Pico SDK + FreeRTOS 版

[English](README.md) | 日本語

Arduino Nano RP2040 Connect 向けに、Raspberry Pi Pico SDK と FreeRTOS カーネルでファームウェアを
ビルドする環境です。`zephyr/`、`nuttx/`、`openmv/` と同じく、必要なものを `freertos/` の下に取得します。

今あるのは、USB のコマンドライン（CLI）、LED の点滅、Zephyr 版を移す土台になるカーネルの設定・CPU 負荷の
計測・異常停止の扱いだけのアプリ `app/usb-console` です。IMU の計測（Zephyr 版・NuttX 版）はまだ移していません。

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
freertos/script/build.sh [--clean] [-j N] [-v] [APP]
```

`setup.sh` を自分で読み込み、`freertos/app/APP`（既定は `usb-console`）を `freertos/build/APP/` に
ビルドします。結果は `freertos/build/usb-console/usb-console.uf2` です。

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
| `ps` | タスクの一覧: 状態、優先度、スタックの空きの最小値（バイト）、起動からの CPU 時間の割合。最後に前回の `ps` からの CPU 負荷（`cpu_active`） |
| `irqstack` | 割り込み用のスタックの起動からの使用量（NuttX 版の `irqstack`） |
| `fault` | この起動の前の実行を止めた異常停止の記録 |
| `free` | FreeRTOS のヒープの大きさ、空き、空きの最小値 |
| `reboot` | 再起動 |
| `bootsel` | USB のブートローダー（BOOTSEL、`RPI-RP2` ドライブ）で再起動 |

| ファイル | 役割 |
|---|---|
| `CMakeLists.txt` | Pico SDK と FreeRTOS カーネルと `common/` の共有ソースの取り込み、USB の製品名（`FreeRTOS USB Console`） |
| `FreeRTOSConfig.h` | カーネルの設定 |
| `src/main.c` | 起動、コンソールのタスク、LED のタスク |
| `src/cli.[ch]` | 行の編集とコマンドの表。カーネルにも SDK にも依存しません |
| `src/commands.c` | 上のコマンド |
| `src/priorities_os.h` | 優先度とスタックの表（`common/compat/priorities.h`）の FreeRTOS の部分 |
| `src/os_clock.h` | 起動からの時間（TIMER）、tick の数、Zephyr と同じ丸めの sleep |
| `src/tick.c` | tick フック: 20 ms の時分割 |
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
  同じタスクが 200 tick 続いたら切り替えを頼みます。時間は、そのタスクが動いているのを最初に見た tick から
  数えます。

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

100 µs の tick、時分割、CPU 負荷、`sleep`、`irqstack`、異常停止の記録（2026-11-17）は、ビルド（警告なし）と、
記録の形式のホストテストだけで、実機ではまだ確かめていません。
