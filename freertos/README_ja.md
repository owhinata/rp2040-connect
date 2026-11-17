# Pico SDK + FreeRTOS 版

[English](README.md) | 日本語

Arduino Nano RP2040 Connect 向けに、Raspberry Pi Pico SDK と FreeRTOS カーネルでファームウェアを
ビルドする環境です。`zephyr/`、`nuttx/`、`openmv/` と同じく、必要なものを `freertos/` の下に取得します。

Zephyr 版・NuttX 版の機能を移している途中で、今あるのは USB のシェルと LED の点滅です。IMU の計測は
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
| `freertos/stmemsc/` | ST の LSM6DSO のレジスター定義（`lsm6dso_reg.[ch]`） | NuttX 版と同じコミット（SHA-256 を検証） |

あわせて、現在のシェルに `PICO_SDK_PATH`、`FREERTOS_KERNEL_PATH`、`PICO_TOOLCHAIN_PATH` と `PATH`
（ツールチェーン、`freertos/.venv/bin`、`freertos/script`）を設定します。版はスクリプトの冒頭で固定しています。取得したものは
Git の管理外です。

## ビルド

```bash
freertos/script/build.sh [--clean] [-j N] [-v] [-DNAME=VALUE ...] [APP]
```

`freertos/app/APP`（既定は `usb-console`）を `freertos/build/APP/` にビルドします。結果は
`freertos/build/usb-console/usb-console.uf2` です。`setup.sh` は自分で読み込みます。

最初のビルドでは、Pico SDK が `picotool` のソースを GitHub から取得してホスト用にビルドします
（ネットワークが必要です）。

`-DNAME=VALUE` は CMake に渡すオプションで、`--clean` するまで残ります。

| オプション | 既定 | 内容 |
|---|---|---|
| `-DNANO_SELFTEST=ON` | OFF | 試験用の `selftest` コマンド（Zephyr 互換層、I2C0、LSM6DSOX の自己診断。計測を止め、ボードを静止させて使う）を入れる |
| `-DNANO_SCHED_ZEPHYR=OFF` | ON | 同じ優先度のタスクの選び方を、Zephyr に合わせず FreeRTOS 標準のままにする（比較用） |

## ホストテスト

```bash
freertos/script/host_tests.sh [-v]
```

共有ソースのテストと、Zephyr 互換層・シェルのテストをホストで走らせます。ボードは要りません。
`setup.sh` で取得したカーネルと、ホストの `cc` を使います。

## 書き込み

```bash
freertos/script/flash.sh [--build] [--wait 秒] [UF2]
```

1. `RPI-RP2` ドライブ（BOOTSEL モードの RP2040）があれば、そこへ UF2 をコピーします。
2. 無くて、このファームウェアの USB コンソールがあれば、`bootsel` コマンドを送ってドライブを待ちます。
   操作は要りません。
3. どちらも無ければ、BOOTSEL への入り方を表示してドライブを待ちます。

コピーのあと、USB コンソール（`/dev/serial/by-id/usb-*FreeRTOS_USB_Console*`）のパスを表示します。

BOOTSEL への入り方:

- このファームウェアが動いているとき: `bootsel` コマンド。
- Zephyr 版・NuttX 版が動いているとき: RESET ボタンを素早く 2 回押します。効かなければ、REC を GND に
  つないで RESET を押します。
- OpenMV 版が動いているとき: REPL で `import machine; machine.bootloader()` を実行します。

**書き込む前に**: 書き込むと Zephyr 版・NuttX 版・OpenMV 版のファームウェアは消えます。フラッシュの
末尾 8 KB（KV。Zephyr 版・NuttX 版と共用）は書き換えません。

## コンソール

USB の CDC ACM ポートがコンソールです（UART0 は使いません）。端末を開くとプロンプト `freertos:~$ ` が
出ます。

| コマンド | 内容 |
|---|---|
| `help` | コマンドの一覧 |
| `version` | FreeRTOS カーネル、Pico SDK、コンパイラーの版 |
| `id` | ボードの固有 ID（16 桁の 16 進数） |
| `uptime` | 起動からの時間と tick の数 |
| `sleep <ms>` | ms ミリ秒眠る |
| `devmem <address> [<width> [<value>]]` | メモリーの読み書き。アドレスと値は 16 進、幅は 8・16・32 ビット（省略時 32）。値を付けると書き込み |
| `ps` | タスクの一覧（状態、優先度、スタックの空きの最小値、CPU 時間の割合）と、前回の `ps` からの CPU 負荷 |
| `irqstack` | 割り込み用スタックの使用量 |
| `dmesg` | RAM ログ（8 KB。再起動をまたいで残る） |
| `kv set <key> <value>`、`kv sethex <key> <hex>`、`kv get <key>`、`kv delete <key>` | KV（設定と校正値）の読み書き。`get` は 16 進で表示 |
| `kv wipe confirm` | KV の領域全体（Wi-Fi の資格情報も）を消す |
| `nina version` | NINA-W102 のファームウェアの版 |
| `i2c_dma mode [on\|off]`、`i2c_dma stats` | I2C0 の DMA 読み出しの切り替えと表示、回数の表示 |
| `fault` | 前回の実行を止めた異常停止の記録 |
| `free` | FreeRTOS のヒープと `malloc()` の使用量 |
| `reboot` | 再起動 |
| `bootsel` | BOOTSEL（`RPI-RP2` ドライブ）で再起動 |

- 行の編集は Backspace、Ctrl-C（行を捨てる）、上下の矢印（履歴）です。Tab 補完と行の途中の編集は
  ありません。引用符と `\` の扱いは Zephyr 版のシェルと同じです。
- 異常停止（HardFault、`panic()`、アサート、スタックのあふれ、ヒープ不足）が起こると、記録を残して自動で
  再起動します。記録は、次に端末をつないだときの表示、`fault`、`dmesg` で見られます。
- KV はフラッシュの末尾 8 KB にあり、形式は Zephyr 版・NuttX 版と同じです。版を書き換えても残ります。領域に
  別のデータがあると `kv` は `-138` で失敗し、`dmesg` に案内が出ます。消すのは `kv wipe confirm` です。
- `devmem` で存在しないアドレスを読むと異常停止します（例: `devmem 30000000`）。

## ディレクトリー

| 場所 | 内容 |
|---|---|
| `app/usb-console/` | アプリ: 起動、コマンド、カーネルの設定（`FreeRTOSConfig.h`） |
| `drivers/compat/` | Zephyr 互換層の FreeRTOS の部分（共有ソース `common/` を動かす） |
| `drivers/shell/`、`drivers/log/`、`drivers/libc/` | シェル、RAM ログ、標準出力の接続 |
| `drivers/kv/` | KV のフラッシュの読み書き |
| `drivers/nina/` | NINA-W102 の SPI と制御線 |
| `drivers/i2c_dw/` | I2C0 と DMA（センサーのバス） |
| `drivers/lsm6dso/` | LSM6DSOX のデバイスの生成と初期化（ドライバー本体は `common/drivers/lsm6dso/`） |
| `tests/` | ホストテスト |
| `script/` | `setup.sh`、`build.sh`、`flash.sh`、`format.sh`、`host_tests.sh` |
| `PORTING_PLAN.md`、`PORTING_STATUS.md` | 移植の計画と、進み具合・確かめたことの記録 |

ソースの整形は `freertos/script/format.sh`（`--check` で検査だけ）です。
