# OpenMV 版 MicroPython のビルド環境

[English](README.md) | 日本語

Arduino Nano RP2040 Connect 向けに、[OpenMV](https://github.com/openmv/openmv) のファームウェア
（OpenMV 版の MicroPython）をソースからビルドする環境です。`zephyr/`、`nuttx/` と同じく、必要なものを
`openmv/` の下に取得します。ここにあるのはビルド環境だけで、このリポジトリ独自のコードはまだありません。
IMU の計測ファームウェア（Zephyr 版・NuttX 版）とは別のものです。

| 項目 | 内容 |
|---|---|
| OpenMV | v4.7.0（MicroPython 1.25.0-preview）。ターゲットは `ARDUINO_NANO_RP2040_CONNECT` |
| ツール | OpenMV SDK 1.6.0（Arm GNU Toolchain 14.3.Rel1、CMake） |

v4.7.0 は、OpenMV がこのボード向けのファームウェアをビルドして配布した最後のリリースです。それより
新しいリリースは、このボードでは使えませんでした。

| リリース | 結果 |
|---|---|
| v4.7.0 | ビルドでき、実機で動きます。 |
| v4.8.1 | ビルドはできますが、実機で `main.py` を Ctrl-C で止めると、トレースバックの 1 行目を出したところで止まり、USB も応答しなくなります（リセットで復帰。原因は調べていません）。 |
| v5.0.1 | rp2 ポートがビルドできません（削除された `common/usbdbg.c` を参照しています）。 |

OpenMV SDK は v5.0.0 から導入されたもので、v4.7.0 は Arm のサイトのツールチェーンを直接使う作りです
（OpenMV の CI は 13.2.Rel1 を使っていました）。ここでは SDK 1.6.0 のコンパイラ（14.3.Rel1）と CMake で
ビルドします。

## 必要なもの

- Linux の PC（x86_64）。sudo は要りません。
- `git`、`curl` または `wget`、`tar`、`xz`、`sha256sum`
- `python3`、`make`、`gcc`、`g++`（ホスト用の `mpy-cross` と `picotool` をビルドするため）
- ディスクの空き 約 7 GB（SDK が 4.6 GB、OpenMV とサブモジュールが 0.9 GB、ダウンロード中の一時ファイルが 0.9 GB）

## 環境構築

リポジトリのルートで読み込みます。足りないものだけを取得するので、何度読み込んでもかまいません。
新しいターミナルでは読み込み直します。

```bash
source openmv/script/setup.sh
```

行うこと:

1. OpenMV v4.7.0 を `openmv/openmv/` に取得します（浅いクローン。タグのコミットを照合します）。
2. rp2 ポートが使うサブモジュールだけを取得します（OpenMV の `lib/micropython` と `modules/ulab`、
   MicroPython の `lib/pico-sdk`、`lib/mbedtls`、`lib/tinyusb`、`lib/mynewt-nimble`、`lib/micropython-lib`）。
3. OpenMV SDK 1.6.0 を用意します。`~/openmv-sdk-1.6.0`（OpenMV の `make sdk` の置き場所）にあればそれを
   使い、無ければ `https://download.openmv.io/sdk/` から取得して（約 0.9 GB、SHA-256 を照合）
   `openmv/openmv-sdk-1.6.0/` に展開します。
4. `OPENMV_SDK_DIR` を設定し、SDK の `gcc/bin` と `cmake/bin`、`openmv/script` を `PATH` に加えます。

バージョンと SHA-256 は `setup.sh` の先頭にまとめてあります。取得したものはすべて Git 管理外です。

## ビルド

```bash
openmv/script/build.sh [--clean] [-j N] [-v]
```

`setup.sh` を自分で読み込み、`mpy-cross`、ファームウェアの順にビルドして、結果を `openmv/build/` に
コピーします（`firmware.uf2`、`firmware.bin`、`firmware.elf`）。`--clean` は `openmv/openmv/build/` を
消してからビルドします。

最初のビルドでは、pico-sdk が `picotool` を GitHub から取得してホスト用にビルドします（ネットワークが
必要です）。

## 書き込み

`openmv/build/firmware.uf2` を、BOOTSEL モードの `RPI-RP2` ドライブにコピーします。書き込み用の
スクリプトはありません。

```bash
cp openmv/build/firmware.uf2 /media/$USER/RPI-RP2/ && sync
```

BOOTSEL への入り方:

- Zephyr 版・NuttX 版が動いているとき: RESET ボタンを素早く 2 回押します。
- このファームウェアが動いているとき: REPL で `import machine; machine.bootloader()` を実行します。

**書き込む前に**: MicroPython はフラッシュの後ろ 14 MB をファイルシステム（FAT）に使います
（`MICROPY_HW_FLASH_STORAGE_BYTES`）。Zephyr 版・NuttX 版の KV 領域（校正値、`imu.port`、Wi-Fi の
資格情報）はこの範囲に入るので、先に退避してください（`script/kv_transfer.py`）。

## 書き込んだあと

ボードは USB のシリアル（`/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_*`）と、14 MB の USB
ドライブ（`main.py` が入っています）として見えます。起動すると既定の `main.py` が青の LED を点滅させ
続けるので、REPL を使うにはシリアルに Ctrl-C を送ります。

実機で確かめたこと（2026-11-15、各 1 回）:

- Ctrl-C で `main.py` が止まり、REPL に入れること
- `lsm6dsox` で加速度と角速度が読めること（I2C0、SCL が GPIO13、SDA が GPIO12）
- `network.WLAN` が NINA のファームウェアのバージョン（3.0.2）を読めること。実機の NINA には、この
  リポジトリで修正したファームウェア（`firmware/nina-fw/`）が入っています。

確かめていないこと: Wi-Fi の接続と通信、Bluetooth、OpenMV IDE からの接続、カメラや音声のモジュール。
