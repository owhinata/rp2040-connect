# rp2040-connect

Arduino Nano RP2040 Connect 向けの Zephyr ワークスペースです。
複数のアプリケーションで、`manifest/west.yml` に指定した Zephyr と
モジュールを共有します。モジュールはアプリの機能に応じて追加してください。

## 環境構築

以下は Ubuntu 24.04 向けの手順です。初めにビルドに必要なパッケージを
インストールします。

```bash
sudo apt update
sudo apt install --no-install-recommends \
  git cmake ninja-build gperf ccache dfu-util device-tree-compiler \
  wget python3-dev python3-venv python3-tk xz-utils file make gcc \
  gcc-multilib g++-multilib libsdl2-dev libmagic1
```

AArch64 環境では `gcc-multilib` と `g++-multilib` を除いてください。
他の OS では [Zephyr の公式手順](https://docs.zephyrproject.org/latest/develop/getting_started/)を参照してください。

このリポジトリのルートでセットアップスクリプトを読み込みます。

```bash
source script/setup.sh
```

スクリプトは必要に応じて `.venv`、west ワークスペース、モジュール、
Python パッケージ、Zephyr SDK と Arm ツールチェーンを用意します。
また、現在のシェルに `ZEPHYR_SDK_INSTALL_DIR` と `west` のエイリアスを
設定します。仮想環境の activate は不要です。新しいターミナルでは再度
`source script/setup.sh` を実行してください。

インストール済みかどうかは、west のプロジェクト、主要な Python パッケージ、
SDK と Arm コンパイラの有無から判定します。追加したアプリでモジュールが
必要になった場合は、`manifest/west.yml` を更新し、次を実行してください。

```bash
west update
west packages pip --install
```

## アプリケーションのビルド

`app/blink` は、ボードの `led0` を 500 ms ごとに切り替えるサンプルです。
ビルドには次のコマンドを実行します。

```bash
west build -b arduino_nano_connect app/blink -d build/blink
```

ボードを USB に接続してブートローダーモードにし、UF2 runner を指定して
書き込みます。

RESET ボタンの2回押しでブートローダーモードに入れない場合は、`REC` と
`GND` をジャンパー線で接続した状態で RESET ボタンを押します。USB ストレージが
表示されたらジャンパー線を外してください。

```bash
west flash -d build/blink -r uf2
```

## USB シリアルコンソール

`app/usb-console` は USB CDC ACM 経由で Zephyr シェルを使うアプリです。
次のコマンドでビルドして書き込みます。

```bash
west build -b arduino_nano_connect app/usb-console -d build/usb-console
west flash -d build/usb-console -r uf2
```

書き込み後、USB シリアル端末から接続します。デバイス名は環境によって
異なりますが、例えば `/dev/ttyACM0` の場合は次のように接続できます。

```bash
minicom -D /dev/ttyACM0
```

Enter キーを押すとシェルが表示されます。`help` でコマンド一覧を、`hello` で
挨拶を表示します。ログは RAM のリングバッファに保存され、`dmesg` コマンドで
表示できます。バッファは 4 KiB で、いっぱいになると古いデータから上書きされます。
通常のリセット後もログを参照できるよう初期化されない RAM に保存します。
電源断や、RAM を初期化するリセットでは消えます。リセット直前の未処理のログは
保存されない場合があります。タイムスタンプは起動後の経過時間をマイクロ秒単位で
表示し、リセット後は 0 から始まります。
minicom は `sudo apt install minicom` でインストールでき、
終了するには `Ctrl-A`、続けて `X` を押します。
