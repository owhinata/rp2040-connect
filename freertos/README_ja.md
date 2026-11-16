# Pico SDK + FreeRTOS 版

[English](README.md) | 日本語

Arduino Nano RP2040 Connect 向けに、Raspberry Pi Pico SDK と FreeRTOS カーネルでファームウェアを
ビルドする環境です。`zephyr/`、`nuttx/`、`openmv/` と同じく、必要なものを `freertos/` の下に取得します。

今あるのは、USB のコマンドライン（CLI）と LED の点滅だけのアプリ `app/usb-console` です。IMU の計測
（Zephyr 版・NuttX 版）はまだ移していません。

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
| `ps` | タスクの一覧: 状態、優先度、スタックの空きの最小値（バイト）、起動からの CPU 時間の割合 |
| `free` | FreeRTOS のヒープの大きさ、空き、空きの最小値 |
| `reboot` | 再起動 |
| `bootsel` | USB のブートローダー（BOOTSEL、`RPI-RP2` ドライブ）で再起動 |

| ファイル | 役割 |
|---|---|
| `CMakeLists.txt` | Pico SDK と FreeRTOS カーネルの取り込み、USB の製品名（`FreeRTOS USB Console`） |
| `FreeRTOSConfig.h` | カーネルの設定 |
| `src/main.c` | 起動、コンソールのタスク、LED のタスク、カーネルのフック |
| `src/cli.[ch]` | 行の編集とコマンドの表。カーネルにも SDK にも依存しません |
| `src/commands.c` | 上のコマンド |

- タスクは `console`（優先度 2、スタック 4 KB）と `heartbeat`（優先度 1、LED を 500 ms ごとに反転。
  GPIO6）で、どちらもスタックを静的に確保します。ほかにカーネルの `IDLE` と `Tmr Svc` があります。
- コンソールのタスクは、USB の受信の通知（Pico SDK の `stdio_set_chars_available_callback()`）で起き、
  端末の接続を見るために 50 ms ごとにも起きます。
- カーネルの設定: 1 コア（`configNUMBER_OF_CORES` 1）、tick は 1 kHz、優先度は 0〜31（数が大きいほど高い）、
  ヒープは `heap_4` の 32 KB。Pico SDK のロックと sleep は、呼んだタスクを待たせます
  （`configSUPPORT_PICO_SYNC_INTEROP`、`configSUPPORT_PICO_TIME_INTEROP`）。アサートの失敗、スタックの
  あふれ（`configCHECK_FOR_STACK_OVERFLOW` 2）、ヒープの不足では `panic()` で止まります。
- USB の製品名は `FreeRTOS USB Console` です。Pico SDK は USB の文字列を既定で 19 文字に切るので、上限
  （`USBD_DESC_STR_MAX`）を 32 にしています。
- tick は Zephyr 版・NuttX 版（100 µs）と違います。計測のコードを移すときに決め直します。
- CLI は FreeRTOS-Plus-CLI ではなく、このリポジトリのものです。

## 確かめたこと

2026-11-16 に、ビルドと実機（Arduino Nano RP2040 Connect）で確かめました。

- ビルドが警告なしで通ること（GCC 15.2）。`freertos/script/format.sh --check` が違いを報告しないこと。
- `flash.sh` が `RPI-RP2` ドライブへ UF2 をコピーし、再起動後の USB コンソールを見つけること。
- ファームウェアが動いているときに `flash.sh` が `bootsel` を送り、操作なしで書き込めること（2 回）。端末から
  打った `bootsel` でも BOOTSEL に入ること（1 回）。
- `help`、`version`、`id`、`uptime`、`ps`、`free` の出力と、無いコマンドへのエラー。

確かめていないこと: `reboot`、1200 bps でのオープンと `picotool` による BOOTSEL、長時間の動作。
