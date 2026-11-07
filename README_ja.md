# rp2040-connect

[English](README.md) | 日本語

Arduino Nano RP2040 Connect 向けの Zephyr ワークスペースです。
複数のアプリケーションで、`manifest/west.yml` に指定した Zephyr と
モジュールを共有します。モジュールはアプリの機能に応じて追加してください。

設計と実装の方針は [IMU FIFO ドライバー計画](design/imu-fifo-driver-plan-ja.md) を参照してください。

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
初回やマニフェストを更新した後は `west update` を実行し、次のコマンドで
ビルドして書き込みます。

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

GPIO、I²C、センサー、再起動、ログのシェルコマンドを有効にしています。
`gpio --help`、`i2c --help`、`sensor --help`、`kernel reboot --help`、
`log --help` で使い方を確認できます。`log` はログレベルの参照や変更に使い、
`dmesg` は RAM に保存されたログの表示に使います。

`ps` を実行すると、スレッドの CPU 使用率とスタック使用量を表示します。

## IMU FIFO の測定

`imu_measure` は Sensor Streaming API で FIFO のバッチを取得し、サンプルを
逐次表示せずに件数と転送時間を集計します。まず使用するセンサーのサンプリング
周波数を設定し、測定秒数を指定します。例えば両方 208 Hz で10秒間測定するには：

```text
sensor attr_set lsm6dsox accel_xyz sampling_frequency 208
sensor attr_set lsm6dsox gyro_xyz sampling_frequency 208
imu_measure start 10
```

測定終了後に結果を表示します。途中で停止するには `imu_measure stop`、最後の
結果を再表示するには `imu_measure result` を実行します。測定時間は1〜3600秒です。
単一センサーを測る場合は、もう片方の `sampling_frequency` を `0` に設定します。
`sensor stream` と `imu_measure` は同時に使えません。

センサー時刻と内部温度への対応については、[ドライバーの TODO](app/usb-console/drivers/README_ja.md#todo)
を参照してください。

指定時間の終了または途中停止時には、FIFO への新しいバッチ追加を止め、FIFO に
残ったワードと処理中の RTIO 完了通知を集計してから結果を確定します。
`stop_requested_ms` は停止を要求した時点、`elapsed_ms` は残データの回収と終了処理を
含む時間です。センサーごとのバッチ停止は順番に行うため、停止境界は厳密に同時では
ありません。

結果には加速度・ジャイロの受信サンプル数、未対応タグのワード数、設定 ODR、
FIFO バーストの回数と合計・最大時間、I²C エラー、バッファ不足、FIFO overrun の
観測、RTIO の完了通知の欠落、ストリームエラーを表示します。`cpu_busy` は測定区間中の
スケジューラ上の非アイドル時間で、この集計処理と他スレッドの負荷を含みます。アイドル中の割り込みはアイドル時間に含まれる場合があるため、CPU が実際に動作した時間の合計ではありません。
`observed_hz` は受信したサンプル数を `elapsed_ms` で割った概算値です。開始時の設定と
停止・残データ回収にかかる時間の影響を受けるため、短い測定ほど誤差が大きくなります。
FIFO のハードウェア時刻やセンサーの正確な生成時刻を測るものではありません。

`accounting=match` は、ドライバーが読み出したワード数・転送回数とアプリが集計した
ワード数・バッチ数が一致し、集計を妨げるエラーが報告されなかったことを示します。
`mismatch` は両者の不一致、`incomplete` はエラーや完了通知の欠落などにより照合を
完了できなかったことを示します。センサーが生成した全サンプルの無欠落を保証する
ものではありません。

overrun の観測は、不連続が発生したことを示しますが、失われたサンプル数や独立した
overrun の発生件数は示しません。測定中にバッファ不足・I²C エラー・overrun が発生した
場合はストリームを停止し、`stream_error` にエラーを表示します。`cleanup_timeout=1`
の場合は終了処理が完了せず、次の測定には再起動が必要です。完了通知の欠落や
未知のタグが報告された測定も、取得データの連続性を保証できません。
