# OpenMV 版 MicroPython のビルド環境

[English](README.md) | 日本語

Arduino Nano RP2040 Connect 向けに、[OpenMV](https://github.com/openmv/openmv) のファームウェア
（OpenMV 版の MicroPython）をソースからビルドする環境です。`zephyr/`、`nuttx/` と同じく、必要なものを
`openmv/` の下に取得します。ここにあるのはビルド環境だけで、このリポジトリ独自のコードはまだありません。
IMU の計測ファームウェア（Zephyr 版・NuttX 版）とは別のものです。

| 項目 | 内容 |
|---|---|
| OpenMV | v5.0.1（MicroPython 1.28.0）に `patches/openmv/` のパッチを当てたもの。ターゲットは `ARDUINO_NANO_RP2040_CONNECT` |
| ツール | OpenMV SDK 1.6.0（Arm GNU Toolchain 14.3.Rel1、CMake） |

OpenMV がこのボード向けのファームウェアを配布したのは v4.7.0 が最後で、v5.0.1 の rp2 ポートは
そのままではビルドできません。v5.0.0 で OpenMV IDE との通信が新しいプロトコル（`protocol/`）に
変わりましたが、rp2 ポートは移行されていないためです。v4.7.0 は古いプロトコルしか話せません。
ここでは v5.0.1 の rp2 ポートをパッチで新しいプロトコルに移行して使います（「パッチ」の節）。
バージョンの表示は `v5.0.1-dirty` になります。

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

1. OpenMV v5.0.1 を `openmv/openmv/` に取得します（浅いクローン。タグのコミットを照合します）。
2. rp2 ポートが使うサブモジュールだけを取得します（OpenMV の `lib/micropython`、`modules/ulab`、`lib/apriltag`、
   MicroPython の `lib/pico-sdk`、`lib/mbedtls`、`lib/tinyusb`、`lib/mynewt-nimble`、`lib/micropython-lib`）。
3. `patches/openmv/*.patch` を OpenMV のクローンに当てます。当てたパッチはクローンの
   `.git/openmv-patches/` に記録し、パッチが増減・変更されていれば、戻してから当て直します。
4. OpenMV SDK 1.6.0 を用意します。`~/openmv-sdk-1.6.0`（OpenMV の `make sdk` の置き場所）にあればそれを
   使い、無ければ `https://download.openmv.io/sdk/` から取得して（約 0.9 GB、SHA-256 を照合）
   `openmv/openmv-sdk-1.6.0/` に展開します。
5. `OPENMV_SDK_DIR` を設定し、SDK の `gcc/bin` と `cmake/bin`、`openmv/script` を `PATH` に加えます。

バージョンと SHA-256 は `setup.sh` の先頭にまとめてあります。取得したものはすべて Git 管理外です。

## ビルド

```bash
openmv/script/build.sh [--clean] [-j N] [-v]
```

`setup.sh` を自分で読み込み、`mpy-cross`、ファームウェアの順にビルドして、結果を `openmv/build/` に
コピーします（`firmware.uf2`、`firmware.bin`、`firmware.elf`）。`--clean` は
`openmv/openmv/build/ARDUINO_NANO_RP2040_CONNECT/` を消してからビルドします。

最初のビルドでは、pico-sdk が `picotool` を GitHub から取得してホスト用にビルドします（ネットワークが
必要です）。

## パッチ

`patches/openmv/` にあり、`setup.sh` が番号順に当てます。

| パッチ | 内容 |
|---|---|
| `0001-rp2-port-to-openmv-v5.patch` | rp2 ポートを v5 に追随させます。`main.c` を新しいプロトコル（`omv_protocol`）に移行し、`soft_timer_init()` を MicroPython の `main.c` と同じく 1 回だけ呼ぶようにします（ソフトリブートのたびに呼ぶと pico-sdk がタイマーの二重確保で止まります）。CMake のソース一覧とビルドの出力先、リンカースクリプト（UMA のメモリブロックとその表）を v5 に合わせます。Cortex-M0+ には DWT のサイクルカウンターが無いので、`omv_cycles` はマイクロ秒のタイマーを使います。 |
| `0002-protocol-no-unaligned-access.patch` | プロトコルの受信処理が、バイト列の途中を 16 ビット・32 ビットの値として直接読んでいた 3 か所を直します。Cortex-M0+ はアラインされていないアクセスで HardFault になります（他のポートの CPU では問題になりません）。 |

パッチを変えたときは `source openmv/script/setup.sh` をやり直します。クローンを直接編集したときは、
`git -C openmv/openmv diff` をパッチに書き出してから、`git -C openmv/openmv checkout -- .` と
`rm -rf openmv/openmv/.git/openmv-patches` で戻して `setup.sh` を読み込み直します。

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
ドライブ（`main.py` が入っています）として見えます。ファイルシステムは `/flash` にマウントされます。
起動すると既定の `main.py` が青の LED を点滅させ続けるので、REPL を使うにはシリアルに Ctrl-C を
送ります。

OpenMV IDE は、シリアルを 921600 bps で開いて新しいプロトコルで通信します。それ以外の速度で開くと
通常の REPL です。

実機で確かめたこと（2026-11-16）:

- Ctrl-C で `main.py` が止まり、REPL に入れること
- `lsm6dsox` で加速度と角速度が読めること（I2C0、SCL が GPIO13、SDA が GPIO12）
- `network.WLAN` が NINA のファームウェアのバージョン（3.0.2）を読めること。実機の NINA には、この
  リポジトリで修正したファームウェア（`firmware/nina-fw/`）が入っています。
- 新しいプロトコルの動作。OpenMV の Python クライアント（PyPI の `openmv` 1.0.9）で、接続、バージョンと
  システム情報の取得、チャネルの一覧、スクリプトの実行と標準出力の取得、実行中のスクリプトの停止、
  ストリーミングの有効化・無効化を行いました。スクリプトの実行とソフトリブートは 10 回以上繰り返しています。

- OpenMV IDE から接続して、サンプルの `lsm6dsox_mlc` が動くこと。サンプルが読む UCF ファイルは、ST の
  [STMems_Machine_Learning_Core](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core) の
  `application_examples/lsm6dsox/` から取得して USB ドライブに置きます。

確かめていないこと: Wi-Fi の接続と通信、Bluetooth、カメラや音声のモジュール、フレームの転送（カメラが
無いため）。

注意:

- PC から USB ドライブに置いたファイルは、ボードの RESET ボタンを押すまで MicroPython からは見えません
  （開くと `OSError: [Errno 2] ENOENT` になります）。
- REPL で 1 行でも実行したあとは、プロトコルからのスクリプトの実行と停止が効きません（プロンプトで
  待っている間は割り込みが無効になるためで、OpenMV の他のポートと共通の作りです）。Ctrl-D で
  ソフトリブートするか、ボードをリセットすると戻ります。
- v4.8.1 は、パッチなしでビルドできますが、実機で `main.py` を Ctrl-C で止めるとボードが止まります
  （原因は調べていません）。v4.7.0 はビルドでき実機でも動きますが、古いプロトコルです。
