# rp2040-connect

[English](README.md) | 日本語

Arduino Nano RP2040 Connect（RP2040、NINA-W102、LSM6DSOX）向けの Zephyr ワークスペースです。
主なアプリ `app/usb-console` は、IMU の加速度・角速度を 6667 Hz の RAW データのまま取得し、
PC の時刻を付けて Wi-Fi 経由の MQTT で送信します。

- IMU の FIFO を I²C（DMA）で読み出し、欠落を検出できる番号を付けてバッチ化します。
- Wi-Fi が切れても取得は続け、送れなかった分は破棄して数えます。
- PC の NTP サーバーと同期し、各バッチに PC 時刻（Unix 時刻、µs）を付けます。
- USB と Telnet のシェルから操作します。

設計の詳細は [IMU RAW・MQTT 配信の設計](design/imu-raw-mqtt-design-ja.md)、ドライバー
（LSM6DSO、I²C、NINA）は [ローカルドライバーの説明](app/usb-console/drivers/README_ja.md) を
参照してください。

現状は実験段階です。計測の開始・停止はシェルからの手動操作で、PC 側には検証用の受信
スクリプトしかありません。確認できた範囲と未確認の範囲は設計書にまとめています。

## 環境構築

Ubuntu 24.04 向けの手順です。まずビルドに必要なパッケージを入れます。

```bash
sudo apt update
sudo apt install --no-install-recommends \
  git cmake ninja-build gperf ccache dfu-util device-tree-compiler \
  wget python3-dev python3-venv python3-tk xz-utils file make gcc \
  gcc-multilib g++-multilib libsdl2-dev libmagic1
```

AArch64 環境では `gcc-multilib` と `g++-multilib` を除いてください。他の OS は
[Zephyr の公式手順](https://docs.zephyrproject.org/latest/develop/getting_started/)を参照してください。

リポジトリのルートでセットアップスクリプトを読み込みます。

```bash
source script/setup.sh
```

スクリプトは必要に応じて `.venv`、west ワークスペース（Zephyr v4.5.0 とモジュール）、
Python パッケージ、Zephyr SDK を用意し、現在のシェルに `ZEPHYR_SDK_INSTALL_DIR` と `west` の
エイリアスを設定します。新しいターミナルでは再度読み込んでください。

## ビルドと書き込み

```bash
west build -b arduino_nano_connect app/usb-console -d build/usb-console
west flash -d build/usb-console -r uf2
```

構成は `app/usb-console/prj.conf` の 1 つだけです。

書き込みの前に、ボードを USB に接続してブートローダーモード（`RPI-RP2` という USB
ストレージが現れる状態）にします。RESET ボタンを 2 回押しても入れない場合は、`REC` と `GND` を
ジャンパー線で接続したまま RESET ボタンを押し、ストレージが現れたらジャンパー線を外します。

`app/blink` は `led0` を 500 ms ごとに切り替える最小のサンプルです。

```bash
west build -b arduino_nano_connect app/blink -d build/blink
```

NINA-W102 側は、Arduino の NINA ファームウェア 3.0.2 に 2 件の修正を当てたものを前提に
しています。ビルドと注意点は [NINA ファームウェアの説明](firmware/nina-fw/README.md) を参照して
ください。ボードに入っているバージョンは `nina version` で確認できます。

## 使い方

### 1. シェルに接続する

USB シリアル（CDC ACM）に端末を接続し、Enter を押すとプロンプトが出ます。

```bash
minicom -D /dev/ttyACM0
```

デバイス名は環境によって変わります。`/dev/serial/by-id/` で確認できます。

### 2. Wi-Fi に接続する

2.4 GHz の WPA2-PSK に対応しています。認証情報を保存しておくと、起動時と予期しない切断の
あとに自動で接続します。

```text
wifi cred add -s "SSID" -k 1 -p "パスフレーズ"
wifi cred auto_connect
wifi status
net iface
```

- パスフレーズは端末に表示されます。`wifi cred list` でも表示されます。
- `wifi connect` は接続だけを行い、設定を保存しません。
- `wifi disconnect` で切断した場合は、次の起動か手動接続まで再接続しません。
- `wifi scan` は接続を一時的に切り、終わると保存済みの設定で接続し直します。

PC 側の Wi-Fi アダプターを一時的な AP にするには `script/wifi_ap.sh` が使えます。

```bash
./script/wifi_ap.sh ap <USB-interface> --ssid rp2040-test --channel 6
./script/wifi_ap.sh station <USB-interface>
```

### 3. PC で NTP サーバーを動かす

ボードは既定で、デフォルトゲートウェイ（上の AP を提供している PC）に 2 秒ごとに NTP で
問い合わせます。PC には chrony を入れ、AP 側のサブネットからの問い合わせを許可します。

```bash
sudo apt install chrony
sudo tee /etc/chrony/conf.d/rp2040-ap.conf >/dev/null <<'CONF'
allow 10.42.0.0/24
local stratum 10
CONF
sudo systemctl restart chrony
```

ファイアウォール（ufw など）が有効なら UDP 123 番を開けてください。ボード側の状態は
`ntp status` で確認できます。

### 4. PC で受信する

PC 側のツールは `host/` にあります。導入方法と、記録（`record`）・校正（`calib`）の使い方は
[`host/README_ja.md`](host/README_ja.md) を参照してください。

`imu-host check` は検証用の受信器です。MQTT クライアント 1 台を直接受け付け、RAW バッチの
形式、連番、サンプル番号、時刻を検査し、終了時に集計を JSON で出力します。汎用の MQTT
ブローカーではなく、保存もしません。

```bash
host/.venv/bin/imu-host check --port 1883
```

Ctrl-C で終了すると結果を表示します。`--drop-at`、`--stall-at`、`--rcvbuf` で受信側の停止や
輻輳を再現できます。

### 5. 計測を開始する

```text
imu start
imu status
imu stop
```

`imu start` は引数を取りません。センサーを 6667 Hz・FIFO watermark 128・I²C 1 MHz に設定し、
止めるまで送信を続けます。

- **MQTT サーバーが動いていないとき**：取得は始まり、繋がるまでのバッチは破棄して数えます。
  繋がった時点から届きます。
- **NTP の同期が取れていないとき**：送信は行い、バッチの PC 時刻を無効にします。センサーの
  tick は付きます。
- **Wi-Fi に繋がっていないとき**：取得だけ始まり、Wi-Fi と MQTT が繋がり次第送信します。

設定は KV ストレージで変えられます。どれも省略でき、省略時は既定値を使います。

| キー | 意味 | 既定値 |
|---|---|---|
| `imu.host` | MQTT サーバーの IPv4 アドレス | デフォルトゲートウェイ |
| `imu.port` | MQTT サーバーのポート | `1883` |
| `imu.device` | トピック `imu/<device>/raw` に使う名前 | `rp2040` |
| `imu.odr` | 加速度・角速度の ODR（Hz） | `6667` |
| `imu.watermark` | FIFO watermark（ワード数） | `128` |

```text
kv set imu.port 18830
kv delete imu.port
```

セッション ID は開始のたびに自動で割り当てます（チップ固有の ID と、保存した通し番号）。

## シェルコマンド

| コマンド | 内容 |
|---|---|
| `imu start` / `stop` / `status` | 計測の開始・停止、カウンターとエラーの表示 |
| `imu cal` / `imu cal reload` | `sensor get` / `stream` が校正値を使っているかの表示、KV の読み直し |
| `ntp status` | PC 時刻との同期の状態 |
| `wifi ...`、`net ...` | Wi-Fi と IP の操作（Zephyr 標準） |
| `nina version` | NINA ファームウェアのバージョン |
| `kv set` / `sethex` / `get` / `delete` | KV ストレージ |
| `sensor ...` | センサーの単発取得・ストリーム・属性（Zephyr 標準を拡張） |
| `i2c_dma mode` / `stats` | I²C 読み出しの DMA 切り替えと統計 |
| `zperf ...` | iperf3 による速度測定（Zephyr 標準、下記） |
| `dmesg` | RAM に保存したログの表示 |
| `kernel ps`、`kernel reboot` | スレッドの CPU・スタック、再起動 |
| `gpio ...`、`i2c ...`、`log ...` | Zephyr 標準 |

### `imu status`

1 項目 1 行で、カテゴリーごとに表示します。`errors` と `fifo` の各項目、`time` の
`unix_unavailable` から `tick_out_of_range` までは、正常なら 0 のままです。

| カテゴリー | 見るところ |
|---|---|
| 先頭 | `state`、セッション、`epoch`（個数不明の欠落ごとに 1 進む）、`cpu_active` |
| `acquired` | 取得したサンプル数とバッチ数（送信できたかどうかに関係なく進む） |
| `errors` | ストリームが止まった原因。0 以外なら計測は終了している |
| `fifo` | FIFO オーバーラン、I²C エラー、自動再開の回数など |
| `time` | PC との同期の有無、時刻を付けられなかった数、タイムスタンプの規則からの逸脱 |
| `mqtt` | 接続先、接続状態、送信数、破棄したバッチ数、キューの最大使用量 |

### 校正値の適用

`sensor get` と `sensor stream` が出す物理量（m/s²、rad/s）には、KV に保存した校正値を適用
できます。校正値は PC 側の `imu-host calib` で求め、`imu-host write` で書き込みます
（[`host/README_ja.md`](host/README_ja.md)）。**MQTT で送る RAW には適用しません。**

```text
imu cal
conversion:
  accel: calibrated (244 ug/LSB, key imu.cal.accel.244)
  gyro:  calibrated (70000 udps/LSB, key imu.cal.gyro.70000)
```

校正値はセンサーと感度ごとのキーで持ち、現在の感度のキーがあるときだけ使います。レンジを
変えるなどして対応するキーがなければ、公称感度での変換（`nominal`）になります。`kv` で
キーを書き換えたあとは `imu cal reload` で読み直します（起動時にも読みます）。

### センサーの属性

FIFO の設定は `sensor attr_set` / `attr_get` で扱えます。どちらも次のストリーム開始時に適用され、
ストリーム中は変更できません。`imu start` は watermark を KV の値（既定 128）で上書きします。

```text
sensor attr_set lsm6dsox all fifo_watermark 128
sensor attr_set lsm6dsox all fifo_timestamp_decimation 8
```

`fifo_timestamp_decimation` は FIFO にタイムスタンプを入れる間隔で、0（入れない）、1、8、32 を
指定できます。既定は 8 です。0 にするとバッチに時刻が付きません。

### 速度の測定（zperf）

Zephyr の `zperf` を iperf3 プロトコルで有効にしています。PC 側は `iperf3` を使います。

ボード → PC（送信）。PC で `iperf3 -s -p 5201` を動かしてから実行します。引数は
宛先、ポート、秒数、1 回の送信サイズ、UDP では続けて速度です。

```text
zperf tcp upload <PC の IPv4 アドレス> 5201 30 1024
zperf udp upload <PC の IPv4 アドレス> 5201 30 500 500K
```

PC → ボード（受信）。ボードでサーバーを起動してから、PC で
`iperf3 -c <ボードの IPv4 アドレス> -p 5201 -t 30` を実行します。

```text
zperf tcp download 5201
zperf tcp download stop
```

実機での 30 秒・1 回ずつの測定では、TCP 送信 2.93 Mbit/s、TCP 受信 1.54 Mbit/s、UDP 送信は
500 kbit/s の指定で欠落 0 でした。IMU の送信を止めた状態での値で、無線環境によって変わります。

### Wi-Fi 経由のシェル

Wi-Fi 接続後は TCP ポート 23 で Telnet のシェルを利用できます。同時に使えるクライアントは
1 つです。認証と暗号化がなく、到達できる相手はフルシェルを操作できるので、信頼できる
ネットワークでのみ使ってください。

### KV ストレージ

Zephyr の Settings/NVS に保存します。キーは英数字、`_`、`-`、`.` の 1〜32 文字、値は
1〜128 バイトです。`set` はテキスト、`sethex` は 16 進数のバイト列を保存し、`get` は 16 進数で
表示します。`sys.` で始まるキーは内部用です。データは暗号化されません。Wi-Fi の認証情報も
同じ 8 KiB の領域に保存されます。

### ログ

ログは RAM の 4 KiB のリングバッファに保存し、`dmesg` で表示します。通常のリセットでは
残りますが、電源断では消えます。

## テスト

```bash
./script/host_tests.sh
```

FIFO ワードの解析、バッチとペイロード、送信キュー、時刻の対応付け、NTP の推定、ドライバーの
FIFO 設定と停止処理、NINA の SPI フレーミングなどを、PC 上で実行します。ボードは不要です。

実機での確認は、上の「使い方」の手順で受信器を動かし、終了時の集計（欠番、時刻の有効数と
逆行、`imu status` のエラー）を見ます。

## 制限

- TLS には対応していません。MQTT は平文で、認証もありません。
- ビルドは Zephyr のテスト用の非暗号乱数を使います。
- 時刻の誤差は、ボードの推定と PC 時刻の間で数 ms 以内を確認していますが、センサー内部の
  遅れは測定していません。
- Zephyr のカーネル時計（`kernel uptime` など）は、6667 Hz の送信中に約 480 ppm 遅れます。
  時刻の基準には使っていません。

## リポジトリの構成

| 場所 | 内容 |
|---|---|
| `app/usb-console/src/` | アプリ本体（計測、MQTT、時刻同期、KV、Wi-Fi 設定） |
| `app/usb-console/drivers/` | LSM6DSO、I²C、NINA、sensor shell のローカルドライバー |
| `app/usb-console/tests/` | ホストテスト |
| `app/blink/` | LED 点滅のサンプル |
| `design/` | 設計と調査手順 |
| `firmware/nina-fw/` | NINA ファームウェアの修正とビルド |
| `host/` | PC 側のツール（受信の診断、記録、校正） |
| `script/` | 環境構築、AP、ホストテスト |
| `manifest/west.yml` | Zephyr とモジュールのバージョン |
