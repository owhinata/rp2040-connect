# Pico SDK + ThreadX 版

[English](README.md) | 日本語

Arduino Nano RP2040 Connect 向けに、Raspberry Pi Pico SDK と ThreadX のカーネルでファームウェアを
ビルドする環境です。`zephyr/`、`nuttx/`、`freertos/` と同じく、必要なものを `threadx/` の下に取得します。

Zephyr 版・NuttX 版・FreeRTOS 版と同じく、LSM6DSOX の RAW データを 6667 Hz で取得し、Wi-Fi 経由の MQTT で PC へ送ります。
ソースの大半は 4 つの OS のファームウェアで共有する `common/` にあり、Pico SDK の上の部分（起動、コンソール、KV、
ドライバーのボードの部分）は FreeRTOS 版と同じファイルです。

| 項目 | 内容 |
|---|---|
| Pico SDK | 2.3.1（ボードは `arduino_nano_rp2040_connect`） |
| ThreadX | Eclipse ThreadX v6.5.2.202603b のカーネルだけ（ポートは `ports/cortex_m0/gnu`。USBX・NetX Duo・FileX は使わない） |
| コンパイラー | Arm GNU Toolchain 15.2.Rel1（GCC 15.2） |

## 必要なもの

[FreeRTOS 版](../freertos/README_ja.md)と同じです（Linux の PC、`git`、`python3`、`cmake`、`make`、`gcc` など。ディスクの空き 約 1.5 GB）。

## 環境構築

リポジトリのルートで読み込みます。足りないものだけを取得するので、何度読み込んでもかまいません。

```bash
source threadx/script/setup.sh
```

`threadx/` の下に、ThreadX（`threadx/threadx`）、Pico SDK、Arm GNU Toolchain、`picotool`、`clang-format`、ST の LSM6DSO の
レジスター定義を取得します。バージョンはスクリプトの冒頭で固定しています。取得したものは Git の管理外です。

## ビルド

```bash
threadx/script/build.sh [--clean] [-j N] [-v] [-DNAME=VALUE ...] [APP]
```

`threadx/app/APP`（既定は `usb-console`）を `threadx/build/APP/` にビルドします。結果は
`threadx/build/usb-console/usb-console.uf2` です。

| オプション | 既定 | 内容 |
|---|---|---|
| `-DI2C_DW_I2C1=ON` | OFF | I2C1（SDA は GPIO18 = D6、SCL は GPIO19 = D7）を I2C0 と同じドライバーで使えるようにする。実機では確かめていない。 |

## ホストテスト

```bash
threadx/script/host_tests.sh [-v]
```

共有ソースのテストと、ThreadX の部分（互換層のカーネルの部分、I2C と NINA の待ち）のテストを、ThreadX の Linux 用のポートの上で
走らせます（約 25 秒）。ボードは要りません。項目は [`drivers/README_ja.md`](drivers/README_ja.md) にあります。

## 書き込み

```bash
threadx/script/flash.sh [--build] [--wait 秒] [--dry-run] [UF2]
```

4 つの OS のファームウェアで共通の `script/flash_uf2.sh` です。`RPI-RP2` ドライブがあれば UF2 をコピーし、無ければ、
動いているファームウェアの USB のシェルに `bootsel` を送ってドライブを待ちます（コンソールを別のプログラムで開いているときは
送りません）。コピーのあと、USB コンソール（`/dev/serial/by-id/usb-*ThreadX_USB_Console*`）のパスを表示します。

**書き込む前に**: 書き込むと、ほかの OS のファームウェアは消えます。フラッシュの末尾 8 KB（KV。4 つの OS のファームウェアで共用）は
書き換えません。

## コンソール

USB の CDC ACM ポートがコンソールです。端末を開くとプロンプト `threadx:~$ ` が出ます。コマンドは
[FreeRTOS 版](../freertos/README_ja.md)と同じです（「コンソール」の表）。違いは次のとおりです。

- `version` は ThreadX のバージョンを表示します。
- `ps` の PRIO は ThreadX の数（0 が最も急ぐ。Zephyr の優先度 + 16）です。何も動いていない時間は `(idle)` の行に出ます。
  終わったスレッドは出ません。

## ディレクトリー

| 場所 | 内容 |
|---|---|
| `app/usb-console/` | アプリの ThreadX の部分: 入口の `main()`（`src/main.c`）、カーネルの設定（`tx_user.h`）、tick、優先度、異常停止・`ps` の ThreadX の部分 |
| `drivers/compat/` | Zephyr 互換層の ThreadX の部分（共有ソース `common/` を動かす） |
| `drivers/pico_sdk_interop/` | Pico SDK のロックと待ちを ThreadX のスレッドにつなぐ部分 |
| `../common/pico_sdk/` | Pico SDK の上の OS で共有する部分（FreeRTOS 版と同じ） |
| `../common/drivers/<ドライバー>/port/` | ドライバーのつなぎ: `pico_sdk/`（FreeRTOS 版と同じ）と `threadx/`（待ちとロック） |
| `tests/` | ホストテストと、実機の試験用のファームウェア `board_kernel`（[`drivers/README_ja.md`](drivers/README_ja.md)） |
| `script/` | `setup.sh`、`build.sh`、`flash.sh`、`format.sh`、`host_tests.sh` |
| `PORTING_PLAN.md`、`PORTING_STATUS.md` | 移植の計画と、進み具合・確かめたことの記録 |
